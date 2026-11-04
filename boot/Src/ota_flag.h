/**
 * @file    ota_flag.h
 * @brief   启动标志：记录「有没有待安装的新固件」「黄金备份在哪」。
 *
 * 设计要点：
 *   1. 两个独立扇区各存一份（S2 / S3），擦 S2 的时候 S3 完好，
 *      任何时刻至少有一份能读出来 —— 掉电也不会把启动信息丢干净。
 *   2. 每份自带 CRC32，内容错一位就整份作废，退回用另一份。
 *   3. 带 seq 递增号，两份都有效时取新的那份。
 */

#ifndef __OTA_FLAG_H
#define __OTA_FLAG_H

#include <stdint.h>

#define OTA_FLAG_A_ADDR   0x08008000u   /* 主份：扇区 2（16KB） */
#define OTA_FLAG_B_ADDR   0x0800C000u   /* 备份：扇区 3（16KB） */
#define OTA_FLAG_MAGIC    0x4641544Fu   /* 'O','T','A','F' 小端拼出来 */

/* swap_state 的取值。
 * 关键设计：只有 TRYING 状态才让 boot 写 Flash。
 * 平时（IDLE / DONE）一次擦写都不做 —— 内部 Flash 寿命只有约 1 万次，
 * 每次启动写两轮的话一年半就报废，而且每次擦两个扇区还要多花
 * 200~500ms 开机时间。 */
#define OTA_SWAP_IDLE     0u            /* 从没做过 OTA：正常启动 */
#define OTA_SWAP_TRYING   1u            /* 刚装完新固件，还没确认能跑 */
#define OTA_SWAP_DONE     2u            /* 新固件已确认，正常启动 */

#define OTA_BOOT_TRY_MAX  3u            /* 连续这么多次起不来就判失败 */

typedef struct {
    uint32_t magic;         /* 必须等于 OTA_FLAG_MAGIC */
    uint32_t seq;           /* 递增序号，越大越新 */
    uint32_t pending;       /* 1 = 外部暂存区有新固件待安装 */
    uint32_t fw_len;        /* 新固件字节数 */
    uint32_t fw_crc;        /* 新固件 CRC32 */
    uint32_t golden_len;    /* 黄金备份字节数，0 = 还没有备份 */
    uint32_t golden_crc;    /* 黄金备份 CRC32 */
    uint32_t boot_try;      /* 新固件已经尝试启动的次数 */
    uint32_t swap_state;    /* OTA_SWAP_IDLE / TRYING / DONE */
    uint32_t crc;           /* 上面所有字段的 CRC32（本字段不参与计算） */
} ota_flag_t;               /* 40 字节 */

/* 标准 CRC32（多项式 0x04C11DB7，初值 0xFFFFFFFF，末尾取反） */
uint32_t ota_crc32(const void *data, uint32_t len);

/* 增量式三段接口，分块数据的 CRC 必须用它（如 400KB 固件校验）：
 *   crc = ota_crc32_begin();
 *   crc = ota_crc32_update(crc, 块1, len1);
 *   crc = ota_crc32_update(crc, 块2, len2);
 *   结果 = ota_crc32_end(crc);
 * 结果与一次性版喂同样字节完全一致。 */
uint32_t ota_crc32_begin(void);
uint32_t ota_crc32_update(uint32_t crc, const void *data, uint32_t len);
uint32_t ota_crc32_end(uint32_t crc);

/* 读标志：两份都试，取有效且 seq 大的那份。0 = 读到，-1 = 两份都无效 */
int ota_flag_load(ota_flag_t *out);

/* 写标志：先算 CRC，再依次覆盖两份。0 = 主份写好，-1 = 主份写失败 */
int ota_flag_save(const ota_flag_t *in);

/* 填一份"干净"的默认值：没有待安装固件、没有备份 */
void ota_flag_default(ota_flag_t *f);

/* 递增 boot_try 和 seq 后落盘。给 boot 在「待确认」状态下用。
 * 0 = 写好，-1 = 写失败 */
int ota_flag_bump_try(ota_flag_t *f);

#endif /* __OTA_FLAG_H */
