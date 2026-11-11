/**
 * @file    app_wifi_cfg.c
 * @brief   WiFi 凭据掉电保存的实现。分区和记录格式见头文件说明。
 *
 * 一条记录的完整生命周期：
 *   上电  wifi_cfg_init  扫 A/B 两槽 -> 选 seq 最大且校验通过的那份进 RAM
 *   配网  wifi_cfg_request 只写 RAM 待写缓冲
 *   落盘  wifi_cfg_save  擦"没在用的槽"-> 写 -> 回读比对 -> 更新 RAM
 *   救砖  wifi_cfg_fallback 取另一槽的旧参数，以新 seq 重写
 *
 * 为什么每次都写"没在用的槽"：
 *   正在用的那份永远不被原地覆盖。擦到一半掉电、写一半掉电，
 *   旧记录还在，上电照样能读出一份能用的配置。
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_wifi_cfg.h"
#include "app_ota_flag.h"       /* 复用 ota_crc32，别在工程里存两份 CRC 实现 */
#include "app_config.h"
#include "drv_spiflash.h"

/* app_log / Flash 互斥都没有公共头文件，就地声明（和 net_if_esp01s.c 一致） */
extern void app_log(const char *fmt, ...);
extern void app_flash_lock(void);
extern void app_flash_unlock(void);

/* 两个槽各占一个 4KB 扇区，紧挨着放。
 * 0x300000 落在外部 Flash 的"字库预留区"（0x200000~0x400000）里 —— 注意字库
 * 目前是编译进片内 Flash 的（APP/lv_font_cn_14.c），这 2MB 只是分区表上的占位，
 * 实际没被写过，所以在这里放凭据不会和任何东西冲突。 */
#define WIFI_CFG_SLOT_A    0x300000u
#define WIFI_CFG_SLOT_B    0x301000u

#define WIFI_CFG_MAGIC     0x47464357u     /* 'W''C''F''G'（小端） */
#define WIFI_CFG_VERSION   1u

/* 落到 Flash 上的记录。
 * CRC 覆盖本结构里 crc 之前的全部字节，包括魔数 ——
 * 少覆盖一段就可能出现"魔数对、内容坏"的记录被当成有效配置用。 */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t seq;                              /* 越大越新，A/B 靠它选 */
    char     ssid[WIFI_CFG_SSID_MAX + 1u];
    char     pass[WIFI_CFG_PASS_MAX + 1u];
    uint32_t crc;
} wifi_cfg_rec_t;

/* 当前生效的配置（RAM 缓存）。UI 周期刷新直接读它，一次 Flash 都不碰。 */
static wifi_cfg_t s_cur;
static uint8_t    s_custom = 0;   /* 1 = s_cur 来自 Flash，0 = 编译期默认值 */
static uint8_t    s_slot   = 0;   /* 当前记录在哪个槽：0 = A，1 = B */
static uint16_t   s_seq    = 0;   /* 当前记录的序号 */

/* 待写缓冲：UI 放，net_task 取。 */
static wifi_cfg_t       s_pending;
static volatile uint8_t s_pending_flag = 0;

/* ============================ 小工具 ============================ */

/* 记录是否可信：魔数 + 版本 + CRC + 字符串有结尾 0。
 * 末尾那两条不是多余 —— 记录写坏时 ssid/pass 可能整段没有 0，
 * 后面 strlen 会一路读过结构体，读到别的槽去。 */
static int rec_valid(const wifi_cfg_rec_t *r)
{
    if (r->magic != WIFI_CFG_MAGIC) {
        return 0;
    }
    if (r->version != WIFI_CFG_VERSION) {
        return 0;
    }
    if (ota_crc32(r, sizeof(wifi_cfg_rec_t) - 4u) != r->crc) {
        return 0;
    }
    if (memchr(r->ssid, 0, sizeof(r->ssid)) == NULL) {
        return 0;
    }
    if (memchr(r->pass, 0, sizeof(r->pass)) == NULL) {
        return 0;
    }
    if (r->ssid[0] == '\0') {
        return 0;                    /* SSID 空 = 无效配置，不是"开放网络" */
    }
    return 1;
}

/* 读某一槽。返回 1 = 有效且已填进 out，0 = 无效（空白扇区读出来全 0xFF，魔数就对不上） */
static int rec_read(uint32_t addr, wifi_cfg_rec_t *out)
{
    if (drv_flash_read(addr, (uint8_t *)out, (uint32_t)sizeof(*out)) != 0) {
        return 0;
    }
    return rec_valid(out);
}

/* 把一份记录设为当前生效，并记住它在哪个槽 */
static void rec_use(const wifi_cfg_rec_t *r, uint8_t slot)
{
    memset(&s_cur, 0, sizeof(s_cur));
    strncpy(s_cur.ssid, r->ssid, WIFI_CFG_SSID_MAX);
    strncpy(s_cur.pass, r->pass, WIFI_CFG_PASS_MAX);

    s_slot   = slot;
    s_seq    = r->seq;
    s_custom = 1;
}

/* 用户没配过网时用编译期宏 */
static void cfg_default(wifi_cfg_t *out)
{
    memset(out, 0, sizeof(*out));
    strncpy(out->ssid, WIFI_SSID, WIFI_CFG_SSID_MAX);
    strncpy(out->pass, WIFI_PASSWORD, WIFI_CFG_PASS_MAX);
}

/* ============================ 初始化 ============================ */

void wifi_cfg_init(void)
{
    wifi_cfg_rec_t a;
    wifi_cfg_rec_t b;
    int va = rec_read(WIFI_CFG_SLOT_A, &a);
    int vb = rec_read(WIFI_CFG_SLOT_B, &b);

    if (va && vb) {
        /* 两槽都有效：seq 大的那份是最后一次写入的 */
        if (a.seq >= b.seq) {
            rec_use(&a, 0);
        } else {
            rec_use(&b, 1);
        }
    } else if (va) {
        rec_use(&a, 0);
    } else if (vb) {
        rec_use(&b, 1);
    } else {
        cfg_default(&s_cur);
        s_custom = 0;
        s_slot   = 0;
        s_seq    = 0;
    }

    app_log("[wifi] cfg %s: ssid=%s\r\n",
            s_custom ? "custom" : "default", s_cur.ssid);
}

/* ============================ 查询 ============================ */

const char *wifi_cfg_ssid(void)
{
    return s_cur.ssid;
}

const char *wifi_cfg_pass(void)
{
    return s_cur.pass;
}

uint8_t wifi_cfg_is_custom(void)
{
    return s_custom;
}

/* ============================ 请求（UI 侧） ============================ */

int wifi_cfg_request(const char *ssid, const char *pass)
{
    if (ssid == NULL || pass == NULL) {
        return -1;
    }
    if (ssid[0] == '\0') {
        return -2;
    }
    if (strlen(ssid) > WIFI_CFG_SSID_MAX) {
        return -3;
    }
    if (strlen(pass) > WIFI_CFG_PASS_MAX) {
        return -4;
    }

    /* 内容拷完再置标志，net_task 见到标志时数据一定已经就绪。
     * 中间可能被别的任务打断（比如又按了一次 OK），用临界区把这一段包成一步。 */
    taskENTER_CRITICAL();
    memset(&s_pending, 0, sizeof(s_pending));
    strncpy(s_pending.ssid, ssid, WIFI_CFG_SSID_MAX);
    strncpy(s_pending.pass, pass, WIFI_CFG_PASS_MAX);
    s_pending_flag = 1;
    taskEXIT_CRITICAL();

    return 0;
}

int wifi_cfg_take_request(wifi_cfg_t *out)
{
    int got = 0;

    if (out == NULL) {
        return 0;
    }

    taskENTER_CRITICAL();
    if (s_pending_flag) {
        *out = s_pending;
        s_pending_flag = 0;
        got = 1;
    }
    taskEXIT_CRITICAL();

    return got;
}

/* ============================ 落盘 ============================ */

int wifi_cfg_save(const wifi_cfg_t *cfg)
{
    wifi_cfg_rec_t rec;
    wifi_cfg_rec_t back;
    uint8_t  slot;
    uint32_t addr;

    if (cfg == NULL || cfg->ssid[0] == '\0') {
        return -1;
    }

    /* 挑"当前没在用"的槽。s_custom 为 0 时两槽都没有有效记录，从 A 开始。 */
    slot = (s_custom && s_slot == 0u) ? 1u : 0u;
    addr = (slot == 0u) ? WIFI_CFG_SLOT_A : WIFI_CFG_SLOT_B;

    memset(&rec, 0, sizeof(rec));
    rec.magic   = WIFI_CFG_MAGIC;
    rec.version = WIFI_CFG_VERSION;
    rec.seq     = (uint16_t)(s_seq + 1u);
    strncpy(rec.ssid, cfg->ssid, WIFI_CFG_SSID_MAX);
    strncpy(rec.pass, cfg->pass, WIFI_CFG_PASS_MAX);
    rec.crc     = ota_crc32(&rec, sizeof(rec) - 4u);

    app_flash_lock();

    if (drv_flash_erase_sector(addr) != 0) {
        app_flash_unlock();
        return -2;
    }
    if (drv_flash_write(addr, (const uint8_t *)&rec, (uint32_t)sizeof(rec)) != 0) {
        app_flash_unlock();
        return -3;
    }

    /* 回读逐字节比。写完不验等于没写 —— 配置坏掉的表现是"下电后再也连不上"，
     * 那时候已经没法在板子上查了。 */
    if (drv_flash_read(addr, (uint8_t *)&back, (uint32_t)sizeof(back)) != 0 ||
        memcmp(&back, &rec, sizeof(rec)) != 0) {
        app_flash_unlock();
        return -4;
    }

    app_flash_unlock();

    /* Flash 落稳了才更新 RAM：避免"页面显示存好了、其实没存" */
    memset(&s_cur, 0, sizeof(s_cur));
    strncpy(s_cur.ssid, cfg->ssid, WIFI_CFG_SSID_MAX);
    strncpy(s_cur.pass, cfg->pass, WIFI_CFG_PASS_MAX);

    s_slot   = slot;
    s_seq    = rec.seq;
    s_custom = 1;

    return 0;
}

/* ============================ 回退 ============================ */

int wifi_cfg_fallback(void)
{
    wifi_cfg_rec_t old;
    wifi_cfg_t     cfg;
    uint8_t        other;
    uint32_t       addr;

    /* 没配过网就没有另一份可退，默认值本来就在代码里 */
    if (!s_custom) {
        return -1;
    }

    other = (s_slot == 0u) ? 1u : 0u;
    addr  = (other == 0u) ? WIFI_CFG_SLOT_A : WIFI_CFG_SLOT_B;

    if (!rec_read(addr, &old)) {
        app_log("[wifi] fallback: no valid record in other slot\r\n");
        return -2;
    }

    memset(&cfg, 0, sizeof(cfg));
    strncpy(cfg.ssid, old.ssid, WIFI_CFG_SSID_MAX);
    strncpy(cfg.pass, old.pass, WIFI_CFG_PASS_MAX);

    /* 用新序号重写一份：光在 RAM 里换回来没用，下次开机还是按 seq
     * 挑回那份连不上的。save 会写到"另一槽"（正是刚读的这槽），
     * 擦之前内容已经拷进 cfg 了，不会丢。 */
    return wifi_cfg_save(&cfg);
}
