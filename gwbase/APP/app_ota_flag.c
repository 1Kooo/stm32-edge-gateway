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
#include <string.h>
#include <stdio.h>
#include "tftlcd.h"       /* 黄金备份进度直接刷在开机自检页上（裸 LCD，不走 LVGL） */

/* APP 区起点，与 boot/Src/app_jump.h 的 APP_BASE 同值。
 * 内部 Flash 内存映射，直接按指针读。 */
#define APP_AREA_BASE   0x08010000u

/* app_log 没有公共头文件，app_ota.c / net_if_esp01s.c 也是这么声明的 */
extern void app_log(const char *fmt, ...);

/* 黄金备份搬运缓冲。app_ota_confirm 只在调度器启动前被调用，
 * 不会和别的任务抢这块内存 */
static uint8_t s_golden_buf[2048];

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

/* 把 APP 区当前固件备份到外部 Flash 黄金区（0x100000，存裸正文），
 * 成功后回填 golden_len / golden_crc。
 *
 * 时机：app_ota_confirm 确认成功时 —— 此刻固件已被证明能跑。
 * 失败不拦确认：固件明明活着，不能因为备份写坏了判它死刑，
 * golden 字段不动，旧的备份继续有效。
 *
 * 耗时约 15~25 秒（擦 104 个 4KB 扇区 + 写 412KB + 回读校验），
 * 只在每次 OTA 确认后的那一次开机执行，平时零开销。 */
static int golden_backup_write(ota_flag_t *flag)
{
    uint32_t fw_len = flag->fw_len;
    uint32_t fw_crc = flag->fw_crc;
    uint32_t crc;
    uint32_t off;
    uint32_t len;
    uint32_t sectors;
    uint32_t i;
    uint32_t next_report = 65536u;
    char     lcd_buf[40];

    /* 合法性兜底。装完新固件后，fw_len / fw_crc 记的就是刚装进 APP 的这份 */
    if (fw_len == 0u || fw_len > (1024u * 1024u)) {
        app_log("[golden] bad fw_len %u, skip backup\r\n", (unsigned)fw_len);
        return -1;
    }

    /* 源校验：APP 区实际内容必须和标志记录的 CRC 一致。
     * boot 装完已经验过一次，这里再兜一道 —— 防止标志被写坏后
     * 把垃圾当成黄金备份存进去。 */
    crc = ota_crc32((const void *)APP_AREA_BASE, fw_len);
    if (crc != fw_crc) {
        app_log("[golden] app crc 0x%08X != 0x%08X, skip\r\n",
                (unsigned)crc, (unsigned)fw_crc);
        return -1;
    }

    /* 擦黄金区：按 4KB 扇区覆盖 fw_len */
    sectors = (fw_len + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE;
    app_log("[golden] erase %u sectors ...\r\n", (unsigned)sectors);
    for (i = 0u; i < sectors; i++) {
        if (drv_flash_erase_sector(FW_GOLDEN_BASE + i * FLASH_SECTOR_SIZE) != 0) {
            app_log("[golden] erase FAIL at %u\r\n", (unsigned)i);
            return -1;
        }
        /* 每 16 个扇区往自检页刷一行进度，纯 ASCII 不用管字库 */
        if ((i % 16u) == 0u) {
            sprintf(lcd_buf, "OTA golden erase %lu/%lu",
                    (unsigned long)(i + 1u), (unsigned long)sectors);
            LCD_ShowString(10, 190, tftlcd_data.width, tftlcd_data.height,
                           16, (u8 *)lcd_buf);
        }
    }

    /* 搬运：内部 Flash 内存映射直接读，分块写外部 */
    app_log("[golden] copy %u bytes ...\r\n", (unsigned)fw_len);
    off = 0u;
    while (off < fw_len) {
        len = fw_len - off;
        if (len > sizeof(s_golden_buf)) {
            len = sizeof(s_golden_buf);
        }
        memcpy(s_golden_buf, (const void *)(APP_AREA_BASE + off), len);
        if (drv_flash_write(FW_GOLDEN_BASE + off, s_golden_buf, len) != 0) {
            app_log("[golden] write FAIL at %u\r\n", (unsigned)off);
            return -1;
        }
        off += len;
        if (off >= next_report && off < fw_len) {
            app_log("[golden] %u/%u\r\n", (unsigned)off, (unsigned)fw_len);
            next_report += 65536u;
            sprintf(lcd_buf, "OTA golden copy %lu%%",
                    (unsigned long)(off * 100u / fw_len));
            LCD_ShowString(10, 190, tftlcd_data.width, tftlcd_data.height,
                           16, (u8 *)lcd_buf);
        }
    }

    /* 回读校验：黄金是最后的救命稻草，坏备份比没备份更坑 ——
     * 等真要回滚时才发现 CRC 错，就只能眼睁睁停机。 */
    crc = ota_crc32_begin();
    off = 0u;
    while (off < fw_len) {
        len = fw_len - off;
        if (len > sizeof(s_golden_buf)) {
            len = sizeof(s_golden_buf);
        }
        if (drv_flash_read(FW_GOLDEN_BASE + off, s_golden_buf, len) != 0) {
            app_log("[golden] readback FAIL at %u\r\n", (unsigned)off);
            return -1;
        }
        crc = ota_crc32_update(crc, s_golden_buf, len);
        off += len;
    }
    if (ota_crc32_end(crc) != fw_crc) {
        app_log("[golden] readback crc FAIL\r\n");
        return -1;
    }

    flag->golden_len = fw_len;
    flag->golden_crc = fw_crc;
    app_log("[golden] backup ok: %u bytes crc=0x%08X\r\n",
            (unsigned)fw_len, (unsigned)fw_crc);
    LCD_ShowString(10, 190, tftlcd_data.width, tftlcd_data.height, 16,
                   (u8 *)"OTA golden backup ok");
    return 0;
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

    /* 8.4：确认成功的同时，把这份刚被证明能跑的固件存成黄金备份。
     * 此刻 fw_len / fw_crc 记的正是 APP 区里这份固件。失败不拦确认。 */
    (void)golden_backup_write(&flag);

    flag.swap_state = OTA_SWAP_DONE;
    flag.boot_try   = 0u;
    flag.seq++;

    return (ota_flag_save(&flag) == 0) ? 1 : -1;   /* 1 = 刚确认完 */
}
