/**
 * @file    app_param.c
 * @brief   参数区读写：外部 Flash 两个 4KB 扇区做 A/B 轮换。
 *
 * 为什么不只存一份：擦除和写入都不是原子操作，写到一半掉电，单份就全废，
 * 参数得重新配。两份轮换 + seq 取大 + CRC 校验，任何时刻至少有一份是完整的。
 *
 * 地址放在日志区之后（日志区到 0x700000 结束），后面的空间原来是空的。
 * 注意：这些是**外部 Flash 芯片内部的偏移**，不是 CPU 地址，只能走 drv_flash_*。
 */

#include "app_param.h"
#include "drv_spiflash.h"

#include <stddef.h>

#define PARAM_MAGIC     0x5041524Du     /* "PARM" */
#define PARAM_VER       1u
#define PARAM_A_ADDR    0x00700000u     /* 第一格 */
#define PARAM_B_ADDR    0x00701000u     /* 第二格 */

/* 落盘记录：头部之后才是参数本体。crc 只覆盖从 seq 到末尾，
 * 这样校验字段自己的值不影响校验结果。 */
typedef struct {
    uint32_t    magic;
    uint16_t    ver;
    uint16_t    crc;
    uint32_t    seq;        /* 写入序号，越大越新 */
    app_param_t val;
} param_rec_t;

static uint32_t s_seq  = 0u;    /* 当前生效记录的序号 */
static uint8_t  s_slot = 1u;    /* 当前生效记录在哪一格：0=A 1=B（初值让首次写落 A） */

/* CRC-16/CCITT。不用查表版：参数区一次只算 24 字节
 * （sizeof - offsetof(seq)，magic/ver/crc 三个字段不进 CRC），表纯属浪费 flash。 */
static uint16_t crc16(const uint8_t *d, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    uint32_t i;
    uint8_t  b;

    for (i = 0u; i < n; i++) {
        c ^= (uint16_t)((uint16_t)d[i] << 8);
        for (b = 0u; b < 8u; b++) {
            if ((c & 0x8000u) != 0u) c = (uint16_t)((uint16_t)(c << 1) ^ 0x1021u);
            else                     c = (uint16_t)(c << 1);
        }
    }
    return c;
}

static uint16_t rec_crc(const param_rec_t *r)
{
    return crc16((const uint8_t *)&r->seq,
                 (uint32_t)(sizeof(param_rec_t) - offsetof(param_rec_t, seq)));
}

void app_param_default(app_param_t *p)
{
    p->mb_ms     = 2000u;
    p->pub_ms    = 2000u;
    p->t_min_x10 = -400;        /* 和有效性闸门同值：-40℃ */
    p->t_max_x10 = 800;         /* 80℃ */
    p->h_min_x10 = 0u;
    p->h_max_x10 = 1000u;
    p->alarm_en  = 0u;          /* 默认不判告警：还没人配阈值，先别乱报 */
}

/* 读一格，返回 0 = 这格内容合法 */
static int read_slot(uint32_t addr, param_rec_t *r)
{
    if (drv_flash_read(addr, (uint8_t *)r, (uint32_t)sizeof(*r)) != 0) return -1;
    if (r->magic != PARAM_MAGIC) return -1;
    if (r->ver   != PARAM_VER)   return -1;
    if (r->crc   != rec_crc(r))  return -1;
    return 0;
}

/* 擦一格再写，写完回读校验 —— 写坏了当场就能发现，不用等下次上电 */
static int write_slot(uint32_t addr, const param_rec_t *r)
{
    param_rec_t chk;

    if (drv_flash_erase_sector(addr) != 0) return -1;
    if (drv_flash_write(addr, (const uint8_t *)r, (uint32_t)sizeof(*r)) != 0) return -1;
    if (drv_flash_read(addr, (uint8_t *)&chk, (uint32_t)sizeof(chk)) != 0) return -1;
    if (chk.magic != PARAM_MAGIC || chk.crc != rec_crc(&chk)) return -1;

    return 0;
}

int app_param_load(app_param_t *p)
{
    param_rec_t a;
    param_rec_t b;
    int oka;
    int okb;

    if (p == NULL) return -1;

    oka = (read_slot(PARAM_A_ADDR, &a) == 0);
    okb = (read_slot(PARAM_B_ADDR, &b) == 0);

    if (oka && okb) {
        /* 两份都好：取序号大的那份，它一定是最后一次写成功的 */
        if (b.seq > a.seq) { *p = b.val; s_seq = b.seq; s_slot = 1u; }
        else               { *p = a.val; s_seq = a.seq; s_slot = 0u; }
        return 0;
    }
    if (oka) { *p = a.val; s_seq = a.seq; s_slot = 0u; return 0; }
    if (okb) { *p = b.val; s_seq = b.seq; s_slot = 1u; return 0; }

    /* 两格都无效（首次上电 / 都被写坏）：用默认值并立刻落盘。
     * 只在这里写一次，之后只要读到合法参数就不再碰 Flash —— 反复上电
     * 不写盘，才谈得上"擦写寿命够用"。 */
    app_param_default(p);
    s_seq  = 0u;
    s_slot = 1u;                /* 让下面这次 save 落到 A 格 */
    (void)app_param_save(p);
    return 1;
}

int app_param_save(const app_param_t *p)
{
    param_rec_t r;
    uint8_t     target;
    uint32_t    addr;

    if (p == NULL) return -1;

    /* 永远写"当前没在用的那一格"：写坏了也不影响生效中的这份 */
    target = (uint8_t)(s_slot ^ 1u);
    addr   = (target != 0u) ? PARAM_B_ADDR : PARAM_A_ADDR;

    r.magic = PARAM_MAGIC;
    r.ver   = PARAM_VER;
    r.seq   = s_seq + 1u;
    r.val   = *p;
    r.crc   = rec_crc(&r);

    if (write_slot(addr, &r) != 0) return -1;

    s_seq  = r.seq;
    s_slot = target;
    return 0;
}
