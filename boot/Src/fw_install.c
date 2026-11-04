/**
 * @file    fw_install.c
 * @brief   8.3：把暂存区的新固件搬进内部 Flash APP 区。
 *
 * 流程（顺序是掉电安全的关键）：
 *   1. 先整体验证暂存区正文 CRC —— 只读，APP 区毫发无损；
 *      发现固件坏了直接放弃，上层跳老固件保平安
 *   2. 按需擦 APP 区扇区（这一步之后老固件就没了）
 *   3. 分块搬运，每块写完立刻读回逐字节比对
 *   4. 整体 CRC 终检
 *
 * 掉电推演：
 *   - 步骤 2/3 中途掉电：pending 标志还是 1（改标志在搬运全部成功之后），
 *     下次启动重新走一遍完整流程，幂等
 *   - 步骤 3 完成但标志没写：同样 pending=1 → 重搬一遍，无害
 *   - 标志写成 TRYING 后掉电：走 boot_try 计数，新固件有 3 次机会
 */

#include "fw_install.h"
#include "drv_spiflash.h"
#include "ota_flag.h"
#include "app_jump.h"
#include "boot_uart.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* 搬运缓冲区。boot 的 RAM 目前只用了 1KB 出头，2KB 毫无压力 */
static uint8_t s_buf[2048];

/* F407ZGT6 内部 Flash 扇区大小不规则：
 *   sector 0~3  ：16KB  （0x08000000 ~ 0x0800FFFF，boot + 双份标志）
 *   sector 4    ：64KB  （0x08010000，APP 区起点）
 *   sector 5~11 ：128KB （0x08020000 起）
 * 擦除必须整扇区，所以要算出固件尾部落在哪个扇区。 */
static uint32_t sector_of_addr(uint32_t addr)
{
    if (addr < 0x08010000u) {
        return (addr - 0x08000000u) / 0x4000u;
    }
    if (addr < 0x08020000u) {
        return 4u;
    }
    return 5u + (addr - 0x08020000u) / 0x20000u;
}

/* 步骤 1：验证外部 Flash 某一段正文（只读，不碰内部 Flash）。
 * src_base 传 FW_DATA_BASE 验暂存区，传 FW_GOLDEN_BASE 验黄金备份。 */
static int verify_ext(uint32_t src_base, uint32_t fw_len, uint32_t fw_crc)
{
    uint32_t crc = ota_crc32_begin();
    uint32_t off = 0u;
    uint32_t len;

    while (off < fw_len) {
        len = fw_len - off;
        if (len > sizeof(s_buf)) {
            len = sizeof(s_buf);
        }
        if (drv_flash_read(src_base + off, s_buf, len) != 0) {
            return FW_INS_ERR_READ;
        }
        crc = ota_crc32_update(crc, s_buf, len);
        off += len;
    }

    if (ota_crc32_end(crc) != fw_crc) {
        return FW_INS_ERR_STAGE_CRC;
    }
    return FW_INS_OK;
}

/* 步骤 2：擦 APP 区。扇区号 4..last 连续，一次 HAL 调用全擦。
 * 注意 128KB 扇区擦除最坏要 2 秒一个，5 个扇区最坏约 10 秒，
 * 这期间串口安静属正常，别当成死机。 */
static int erase_app_area(uint32_t fw_len)
{
    FLASH_EraseInitTypeDef er;
    uint32_t sector_err = 0u;
    uint32_t last = sector_of_addr(APP_BASE + fw_len - 1u);

    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;    /* 板子 3.3V */
    er.Sector       = 4u;                       /* 0x08010000 固定是 sector 4 */
    er.NbSectors    = last - 4u + 1u;

    boot_printf("[ins] erase %u sectors ...\r\n", (unsigned)(last - 3u));

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&er, &sector_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return FW_INS_ERR_ERASE;
    }
    HAL_FLASH_Lock();
    return FW_INS_OK;
}

/* 步骤 3：分块搬运 + 每块读回比对。
 * src_base 是外部 Flash 里的源正文起点：装新固件传 FW_DATA_BASE，
 * 回滚传 FW_GOLDEN_BASE（黄金区存的是裸正文，没有头部预留）。
 * 内部 Flash 要按字（4 字节）写；最后一块可能不是 4 的倍数，
 * 尾部补 0xFF 凑整字 —— 多写的字节在固件末尾之后，不影响运行。 */
static int copy_ext_to_app(uint32_t src_base, uint32_t fw_len)
{
    uint32_t off = 0u;
    uint32_t next_report = 65536u;    /* 每 64KB 打一行进度 */
    uint32_t len;
    uint32_t pad_len;
    uint32_t i;
    uint32_t w;

    while (off < fw_len) {
        len = fw_len - off;
        if (len > sizeof(s_buf)) {
            len = sizeof(s_buf);
        }

        if (drv_flash_read(src_base + off, s_buf, len) != 0) {
            return FW_INS_ERR_READ;
        }

        pad_len = (len + 3u) & ~3u;
        for (i = len; i < pad_len; i++) {
            s_buf[i] = 0xFFu;
        }

        HAL_FLASH_Unlock();
        for (i = 0u; i < pad_len; i += 4u) {
            memcpy(&w, &s_buf[i], 4u);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                  APP_BASE + off + i, w) != HAL_OK) {
                HAL_FLASH_Lock();
                return FW_INS_ERR_PROGRAM;
            }
        }
        HAL_FLASH_Lock();

        /* 写完立刻读回比对。内部 Flash 是内存映射的，直接按指针读 */
        if (memcmp((const void *)(APP_BASE + off), s_buf, len) != 0) {
            return FW_INS_ERR_VERIFY;
        }

        off += len;
        if (off >= next_report && off < fw_len) {
            boot_printf("[ins] %u/%u (%u%%)\r\n",
                        (unsigned)off, (unsigned)fw_len,
                        (unsigned)(off * 100u / fw_len));
            next_report += 65536u;
        }
    }
    return FW_INS_OK;
}

/* 装/回滚共用的一条流水线：校验源 → 擦 APP → 搬运 → 终检。
 * 唯一的区别是外部 Flash 里的源地址（暂存区正文 / 黄金备份）。 */
static int install_from(uint32_t src_base, uint32_t fw_len, uint32_t fw_crc)
{
    int r;
    uint32_t crc;

    /* 装不进 APP 区的长度直接拒绝（APP 侧下载时查过一次，这里兜底） */
    if (fw_len == 0u || (APP_BASE + fw_len) > APP_END) {
        return FW_INS_ERR_RANGE;
    }

    r = verify_ext(src_base, fw_len, fw_crc);
    if (r != FW_INS_OK) {
        return r;
    }
    boot_printf("[ins] source crc ok\r\n");

    r = erase_app_area(fw_len);
    if (r != FW_INS_OK) {
        return r;
    }
    boot_printf("[ins] erase done\r\n");

    r = copy_ext_to_app(src_base, fw_len);
    if (r != FW_INS_OK) {
        return r;
    }

    /* 整体终检：确认搬进 APP 区的字节和源头算出的 CRC 一致 */
    crc = ota_crc32((const void *)APP_BASE, fw_len);
    if (crc != fw_crc) {
        return FW_INS_ERR_FINAL_CRC;
    }
    boot_printf("[ins] app crc ok\r\n");
    return FW_INS_OK;
}

int fw_install_run(const fw_header_t *hdr)
{
    return install_from(FW_DATA_BASE, hdr->fw_len, hdr->fw_crc);
}

int fw_restore_golden(uint32_t fw_len, uint32_t fw_crc)
{
    /* 黄金区存的是裸正文，长度和 CRC 都从标志里来 */
    return install_from(FW_GOLDEN_BASE, fw_len, fw_crc);
}
