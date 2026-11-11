/**
 * @file    app_ota_flag.c
 * @brief   APP 侧的启动标志读写 + 启动确认。
 *
 * 读 / 写 / CRC32 的实现与 boot/Src/ota_flag.c 保持一致，
 * 改一处必须同步改另一处（两边操作的是同一条 Flash 记录）。
 */

#include "app_ota_flag.h"
#include "drv_spiflash.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"     /* vTaskDelay：固化期间必须定期让出 CPU，见擦除循环 */
#include "task.h"
#include <string.h>
#include <stdio.h>
#include "app_wdg.h"      /* app_wdg_kick：固化期间给 storage_task 打心跳 */

/* APP 区起点，与 boot/Src/app_jump.h 的 APP_BASE 同值。
 * 内部 Flash 内存映射，直接按指针读。 */
#define APP_AREA_BASE   0x08010000u

/* app_log 没有公共头文件，app_ota.c / net_if_esp01s.c 也是这么声明的 */
extern void app_log(const char *fmt, ...);

/* 黄金备份搬运缓冲。固化由 storage_task 调用，和日志写入是同一个任务，
 * 天然串行；别的任务在固化期间会被 app_golden_busy() 挡住，不会并发使用。 */
static uint8_t s_golden_buf[2048];

/* 固化进度：给屏上弹层用。擦除和搬运各占一段百分比，
 * 只有 storage_task 写、ui_task 读，都是单字节，不需要加锁。 */
static volatile uint8_t s_gld_stage  = 0u;   /* 0准备 1擦除 2搬运 3校验 4完成 */
static volatile uint8_t s_gld_pct    = 0u;   /* 0~100 */
static volatile uint8_t s_gld_result = 0u;   /* 0未做 1成功 2失败 */

uint8_t app_golden_stage(void)
{
    return s_gld_stage;
}

uint8_t app_golden_pct(void)
{
    return s_gld_pct;
}

uint8_t app_golden_result(void)
{
    return s_gld_result;
}

/* ============================ CRC32 ============================ */

/* 逐位算法，不占 1KB 查表空间。标志只有 40 字节，慢一点无所谓。 */
uint32_t ota_crc32_begin(void)
{
    return 0xFFFFFFFFu;
}

/* 分段累加：crc 传的是"还没取反"的中间值，可以连续喂任意多段。
 * 下载固件必须这么用 —— 420KB 不可能先全读进 RAM 再一口气算。 */
uint32_t ota_crc32_update(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t i;
    uint32_t bit;

    for (i = 0; i < len; i++) {
        crc ^= ((uint32_t)p[i] << 24);
        for (bit = 0; bit < 8u; bit++) {
            if (crc & 0x80000000u) {
                crc = (crc << 1) ^ 0x04C11DB7u;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

uint32_t ota_crc32_end(uint32_t crc)
{
    return ~crc;
}

uint32_t ota_crc32(const void *data, uint32_t len)
{
    return ota_crc32_end(ota_crc32_update(ota_crc32_begin(), data, len));
}

/* ============================ 读 ============================ */

/* 一份标志是否可信：魔数对 + CRC 对。空白扇区读出来全是 0xFF，魔数就不对。 */
static int flag_valid(const ota_flag_t *f)
{
    if (f->magic != OTA_FLAG_MAGIC) {
        return 0;
    }
    if (ota_crc32(f, sizeof(ota_flag_t) - 4u) != f->crc) {
        return 0;
    }
    return 1;
}

void ota_flag_default(ota_flag_t *f)
{
    f->magic      = OTA_FLAG_MAGIC;
    f->seq        = 1u;
    f->pending    = 0u;
    f->fw_len     = 0u;
    f->fw_crc     = 0u;
    f->golden_len = 0u;
    f->golden_crc = 0u;
    f->boot_try   = 0u;
    f->swap_state = OTA_SWAP_IDLE;
    f->crc        = 0u;
}

int ota_flag_load(ota_flag_t *out)
{
    /* 内部 Flash 是内存映射的，直接按指针读即可，不用调 HAL */
    const ota_flag_t *a = (const ota_flag_t *)OTA_FLAG_A_ADDR;
    const ota_flag_t *b = (const ota_flag_t *)OTA_FLAG_B_ADDR;
    int va = flag_valid(a);
    int vb = flag_valid(b);

    if (va && vb) {
        *out = (a->seq >= b->seq) ? *a : *b;
        return 0;
    }
    if (va) {
        *out = *a;
        return 0;
    }
    if (vb) {
        *out = *b;
        return 0;
    }

    ota_flag_default(out);
    return -1;
}

/* ============================ 写 ============================ */

/* 把一份标志写进指定扇区：先整扇区擦掉，再按字写进去。 */
static int flag_write_one(uint32_t addr, const ota_flag_t *f, uint32_t sector)
{
    FLASH_EraseInitTypeDef er;
    uint32_t sector_err = 0u;
    const uint32_t *src = (const uint32_t *)f;
    uint32_t i;
    int r = 0;

    HAL_FLASH_Unlock();

    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;    /* 2.7~3.6V，板子是 3.3V */
    er.Sector       = sector;
    er.NbSectors    = 1u;

    if (HAL_FLASHEx_Erase(&er, &sector_err) != HAL_OK) {
        r = -1;
    }

    if (r == 0) {
        for (i = 0; i < (sizeof(ota_flag_t) / 4u); i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  addr + (i * 4u), src[i]) != HAL_OK) {
                r = -1;
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    return r;
}

int ota_flag_save(const ota_flag_t *in)
{
    ota_flag_t f = *in;

    f.magic = OTA_FLAG_MAGIC;
    f.crc   = 0u;
    f.crc   = ota_crc32(&f, sizeof(ota_flag_t) - 4u);

    if (flag_write_one(OTA_FLAG_A_ADDR, &f, FLASH_SECTOR_2) != 0) {
        return -1;
    }

    /* 备份份写失败不致命：主份已经好了，下次启动还能读主份。 */
    (void)flag_write_one(OTA_FLAG_B_ADDR, &f, FLASH_SECTOR_3);
    return 0;
}

/* ============================ 黄金备份 ============================ */

/* 映像长度：以链接器给的执行域结束地址为基准，向上取到扇区边界并留余量。
 *
 * 为什么不能直接用 Image$$ER_IROM1$$Limit 就完事：
 *   ER_IROM1 只覆盖 RO（代码 + 常量）。它后面还紧接着 RW 初值的压缩副本
 *   （map 里写作 COMPRESSED[0xC8]，启动时由 __main 解压进 RAM），
 *   按 ER 的结束地址算会正好少备份这最后一段 —— 恢复出来的固件 .data 是坏的。
 *   （AC5 不导出 Load$$LR_IROM1$$Limit，试过，链接期报未定义。）
 *
 * 所以：ER 结束 + 2KB 余量，再向上取到 4KB 扇区边界。宁可多备份一点
 * 擦除态 0xFF（恢复时写回去也无害），也绝不能少备份。
 *
 * 为什么不用 flag->fw_len：那是 boot 装 OTA 包时写的，ST-Link 直接烧录的
 * 固件根本没有这个值，出厂固化会从一开始就失败。链接器符号永远描述
 * "当前正在跑的这一份映像"，对两种烧录方式都成立。 */
extern uint32_t Image$$ER_IROM1$$Limit;

static uint32_t app_image_len(void)
{
    uint32_t end = (uint32_t)&Image$$ER_IROM1$$Limit;
    uint32_t len;

    if (end <= APP_AREA_BASE || end > (APP_AREA_BASE + (1024u * 1024u))) {
        return 0u;
    }

    /* +2KB 容纳 RW 初值副本，再按 4KB 向上取整 */
    len = (end - APP_AREA_BASE) + 2048u;
    len = (len + FLASH_SECTOR_SIZE - 1u) & ~(FLASH_SECTOR_SIZE - 1u);

    if (len == 0u || len > (960u * 1024u)) {
        return 0u;
    }
    return len;
}

/**
 * 把当前 APP 区固件固化成黄金备份（外部 Flash 0x100000，存裸正文）。
 *
 * 为什么是"手动"而不是 OTA 确认时自动搬：
 *   自动搬等于"只要能开机就当成好固件"。而"能开机、按某个键才死"的固件
 *   会立刻覆盖掉上一份真正好用的备份 —— 等真要回滚时，回滚回去的还是它。
 *   所以固化必须由人决定（云端 {"golden":1}），黄金区才算"经人认可"。
 *
 * 顺序是关键：先作废 -> 再擦写 -> 校验通过才回写 len/crc。
 *   中途掉电时 flag 里是 golden_len=0（"没有备份"），boot 会明确打
 *   "no golden, halt"；而不是 flag 指着旧值、区域却是 0xFF 的假备份 ——
 *   那种要等真回滚时才发现 CRC 对不上，那就彻底没救了。
 *
 * 调用约束：由 storage_task 调用（它是外部 Flash 的唯一写者），
 * 且调用前要确认 OTA 没在跑，否则两边抢 SPI 总线。
 *
 * 返回：0 = 成功；-1 = 失败。
 *   只有“作废成功之后的失败”才保证 golden_len 留在 0（= 明确没有可信备份）；
 *   更早的三个出口（读不到标志 / 镜像长度非法 / 作废写失败）根本没动旧标志，
 *   备份区里还是上一次固化的那份 —— 它本来就是好的，留着比抹掉安全。 */
static int golden_promote_body(void)
{
    ota_flag_t flag;
    uint32_t   len;
    uint32_t   crc_src;
    uint32_t   crc_back;
    uint32_t   off;
    uint32_t   n;
    uint32_t   sectors;
    uint32_t   i;
    uint32_t   next_report = 65536u;

    if (ota_flag_load(&flag) != 0) {
        app_log("[golden] no flag, skip\r\n");
        return -1;
    }

    len = app_image_len();
    if (len == 0u) {
        app_log("[golden] bad image len, skip\r\n");
        return -1;
    }

    s_gld_stage = 0u;               /* 准备：算 CRC、作废旧备份 */
    s_gld_pct   = 0u;

    crc_src = ota_crc32((const void *)APP_AREA_BASE, len);
    app_log("[golden] promote len=%u crc=0x%08X\r\n",
            (unsigned)len, (unsigned)crc_src);

    /* 第一步：先把旧备份作废。这一步之后即使掉电，最坏也只是"没有备份"，
     * 绝不会留下一个 flag 说有、实际是空壳的假备份。 */
    flag.golden_len = 0u;
    flag.golden_crc = 0u;
    flag.seq++;
    if (ota_flag_save(&flag) != 0) {
        app_log("[golden] void old FAIL\r\n");
        return -1;
    }

    /* 第二步：擦 -> 写 */
    sectors = (len + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE;
    s_gld_stage = 1u;               /* 擦除占进度 0~40% */
    app_log("[golden] erase %u sectors ...\r\n", (unsigned)sectors);

    /* 先还给 UI 一整段连续 CPU，把弹层建出来、首帧一次性画完。
     *
     * 为什么要"连续的一大段"而不是每扇区挤几毫秒：LVGL 走 partial 绘制，
     * 一次 refresh 要把整个脏区按绘制缓冲高度切成十几块依次渲染并 flush
     * （lv_port.c 的 DISP_BUF_LINES 决定块高，当前实际 18 行 → 约 18 块）。
     * 这一整套跑完才能出完整的一帧，中途被抢占就会停在"画了一半"的样子。
     * 而 ui_task 优先级 0、固化任务优先级 2，擦扇区又是纯忙等 ——
     * 碎片化地给 1~2ms，ui 只能一块一块往前挪，观感就是从屏幕上方
     * 往下一条条刷出来。
     *
     * 2.5 秒 = 500ms 等 ui 的刷新闸 + 整帧绘制余量，宁可多给也不能少给
     * （固化总共二十秒，多这 2.5 秒无所谓）。 */
    vTaskDelay(pdMS_TO_TICKS(2500));

    for (i = 0u; i < sectors; i++) {
        if (drv_flash_erase_sector(FW_GOLDEN_BASE + i * FLASH_SECTOR_SIZE) != 0) {
            app_log("[golden] erase FAIL at %u\r\n", (unsigned)i);
            return -1;
        }
        app_wdg_kick(WDG_HB_STORE);     /* 整段擦除要十几秒，不打点会被判停滞 */
        s_gld_pct = (uint8_t)((i + 1u) * 40u / sectors);

        /* 首帧画完之后，每扇区让 2ms 就够：进度条只是偶尔重绘一小块。
         * 但必须让 —— 擦扇区是纯忙等（flash_wait_busy 里没有 vTaskDelay），
         * 本任务优先级 2、ui_task 只有 0，一点不让 ui 就彻底饿死。 */
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    s_gld_stage = 2u;               /* 搬运占进度 40%~89%（见下方公式说明） */
    app_log("[golden] copy %u bytes ...\r\n", (unsigned)len);
    off = 0u;
    while (off < len) {
        n = len - off;
        if (n > sizeof(s_golden_buf)) {
            n = sizeof(s_golden_buf);
        }
        memcpy(s_golden_buf, (const void *)(APP_AREA_BASE + off), n);
        if (drv_flash_write(FW_GOLDEN_BASE + off, s_golden_buf, n) != 0) {
            app_log("[golden] write FAIL at %u\r\n", (unsigned)off);
            return -1;
        }
        off += n;
        app_wdg_kick(WDG_HB_STORE);
        /* 分母特意 +1：搬完最后一块时 off==len，不加就会算出正好 90，
         * 留不出 90~100 之间给校验阶段，进度条看着像卡住。加了之后
         * 终值是 40+50k/(k+1) < 90（当前 577536 字节的备份约到 89）。 */
        s_gld_pct = (uint8_t)(40u + (off / 1024u) * 50u / (len / 1024u + 1u));

        /* 弹层已经画好，往后只是进度条那一小块重绘 —— 一次 refresh 就够
         * （约 40ms），所以给一次让出就顶用，不用像首帧那样整段等。
         * 577536 字节 / 64KB ≈ 8 次，合计约 320ms。 */
        if ((off & 0xFFFFu) == 0u) {
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        if (off >= next_report && off < len) {
            app_log("[golden] %u/%u\r\n", (unsigned)off, (unsigned)len);
            next_report += 65536u;
        }
    }

    /* 回读校验：坏备份比没备份更坑 —— 等真要回滚时才发现 CRC 错就只能停机 */
    s_gld_stage = 3u;               /* 校验阶段进度固定 92%：回读比搬运快，不再刷 */
    s_gld_pct   = 92u;
    crc_back = ota_crc32_begin();
    off = 0u;
    while (off < len) {
        n = len - off;
        if (n > sizeof(s_golden_buf)) {
            n = sizeof(s_golden_buf);
        }
        if (drv_flash_read(FW_GOLDEN_BASE + off, s_golden_buf, n) != 0) {
            app_log("[golden] readback FAIL at %u\r\n", (unsigned)off);
            return -1;
        }
        crc_back = ota_crc32_update(crc_back, s_golden_buf, n);
        off += n;

        /* 回读同理：进度条局部重绘，每 64KB 让 40ms */
        if ((off & 0xFFFFu) == 0u) {
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
    if (ota_crc32_end(crc_back) != crc_src) {
        app_log("[golden] readback crc FAIL\r\n");
        return -1;
    }

    /* 第三步：校验通过才回写。此刻它才真正是"可信备份" */
    flag.golden_len = len;
    flag.golden_crc = crc_src;
    flag.seq++;
    if (ota_flag_save(&flag) != 0) {
        app_log("[golden] save flag FAIL\r\n");
        return -1;
    }

    s_gld_stage = 4u;
    s_gld_pct   = 100u;
    app_log("[golden] ok %u bytes crc=0x%08X\r\n",
            (unsigned)len, (unsigned)crc_src);
    return 0;
}

/* 固化进行中：这段时间谁也别碰外部 Flash —— 别的任务一读就会和
 * "擦 141 个扇区"交错在 SPI 上，读出来的东西不可信；而如果它去抢
 * Flash 互斥量，会一直阻塞到固化结束（二十多秒），心跳断供，
 * 看门狗反而会把固化打断在半路。所以宁可让调用方先问一句这个。 */
static volatile uint8_t s_busy = 0u;

/* 固化结束时刻，给 UI 补开结果弹层用。
 *
 * 为什么需要它：固化是云端一条命令触发的，屏上没有按键，弹层只能靠
 * ui_task 的 500ms 闸"看到 busy 就开"。可固化一结束 busy 立刻变 0，
 * 而 ui 往往要等到这一刻才抢到 CPU —— 条件已经不成立，弹层永远开不出来。
 * 所以留一个"刚干完"的时间窗，让 ui 还能补开一次。 */
#define GOLDEN_SHOW_MS   3000u
static volatile uint32_t s_gld_end_tick = 0u;

uint8_t app_golden_just_done(void)
{
    uint32_t now;

    if (s_gld_end_tick == 0u) {
        return 0u;                      /* 从没固化过 */
    }

    now = xTaskGetTickCount();
    return ((now - s_gld_end_tick) < pdMS_TO_TICKS(GOLDEN_SHOW_MS)) ? 1u : 0u;
}

uint8_t app_golden_busy(void)
{
    return s_busy;
}

int app_golden_promote(void)
{
    int r;

    s_gld_stage  = 0u;
    s_gld_pct    = 0u;
    s_gld_result = 0u;
    s_busy       = 1u;

    r = golden_promote_body();

    s_gld_result = (r == 0) ? 1u : 2u;
    s_gld_end_tick = xTaskGetTickCount();   /* 先记时刻再清忙标志：UI 那边不会漏这一次 */
    s_busy       = 0u;
    return r;
}

/* 当前运行镜像的 CRC：开机算一次缓存下来。
 * 560KB 逐位 CRC 约 0.2 秒，绝不能放进 UI 的 500ms 刷新里。 */
static uint32_t s_self_crc   = 0u;
static uint32_t s_self_len   = 0u;
static uint8_t  s_self_valid = 0u;

void app_golden_self_check(void)
{
    uint32_t len = app_image_len();

    if (len == 0u) {
        return;
    }

    s_self_len   = len;
    s_self_crc   = ota_crc32((const void *)APP_AREA_BASE, len);
    s_self_valid = 1u;
}

/* 1 = 黄金备份就是当前正在跑的这版；0 = 不是（或还没算过/没备份）。
 * 用来回答"这个备份过没过期" —— 光看时间还得自己拿版本对一遍。 */
uint8_t app_golden_is_current(void)
{
    ota_flag_t flag;

    if (s_self_valid == 0u) {
        return 0u;
    }
    if (ota_flag_load(&flag) != 0) {
        return 0u;
    }
    if (flag.golden_len == 0u) {
        return 0u;
    }

    return (flag.golden_len == s_self_len && flag.golden_crc == s_self_crc) ? 1u : 0u;
}

/* 黄金备份的长度，0 = 还没有可信备份（给关于页显示用） */
uint32_t app_golden_len(void)
{
    ota_flag_t flag;

    if (ota_flag_load(&flag) != 0) {
        return 0u;
    }
    return flag.golden_len;
}

/* ============================ 确认 ============================ */

int app_ota_confirm(void)
{
    ota_flag_t flag;

    /* 没有有效标志就直接放过。建标志是 boot 的活，APP 不该抢 ——
     * 万一 boot 那一轮被跳过，APP 一写可能把正常状态覆盖成 IDLE。 */
    if (ota_flag_load(&flag) != 0) {
        return -1;
    }

    /* 关键优化：只有"刚装完待确认"才写 Flash。
     * 平时是 IDLE 或 DONE，直接返回，一次擦写都不做。
     * 擦两个 16KB 扇区要 200~500ms，还消耗 1 万次寿命里的 1 次，
     * 每次开机都写的话一年多就写废了。 */
    if (flag.swap_state != OTA_SWAP_TRYING) {
        return 0;                   /* 0 = 无需确认 */
    }

    /* 确认成功 = 这份固件至少能亮屏、能进系统，可以停止 boot 的 try 计数。
     *
     * 注意这里不再自动写黄金备份：能开机不等于够可靠（"按某个键才死"的固件
     * 一样能过这一关），自动搬会把上一份真正好用的备份顶掉。固化改由人决定，
     * 见 app_golden_promote()。顺带 OTA 后第一次开机也快了几十秒。 */
    flag.swap_state = OTA_SWAP_DONE;
    flag.boot_try   = 0u;
    flag.seq++;

    return (ota_flag_save(&flag) == 0) ? 1 : -1;   /* 1 = 刚确认完 */
}
