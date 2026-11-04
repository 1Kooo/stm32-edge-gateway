/**
 * @file    drv_spiflash.h
 * @brief   External SPI NOR flash (EN25Q128 / W25Q128, 16MB).
 *
 * Geometry: 4096 sectors x 4KB = 65536 pages x 256B
 *
 * Rules that bite:
 *   - a page write wraps inside its own 256B page, so split before crossing
 *   - erase/program require WREN first, then wait for BUSY to clear
 *   - reading never needs WREN and has no alignment constraint
 */

#ifndef __DRV_SPIFLASH_H
#define __DRV_SPIFLASH_H

#include <stdint.h>

#define FLASH_SECTOR_SIZE    4096u
#define FLASH_PAGE_SIZE      256u
#define FLASH_TOTAL_SIZE     (16u * 1024u * 1024u)

/* JEDEC id is 24 bit: manufacturer | memory type | capacity.
 * Winbond / EON SPI NOR always reports 0xEF40xx; the last byte is capacity:
 *   0xEF4016 = 4MB, 0xEF4017 = 8MB, 0xEF4018 = 16MB
 */
#define FLASH_EXPECTED_MFR   0xEFu
#define FLASH_EXPECTED_ID    0xEF4018u   /* informational only, not enforced */

/* 0 on success, negative on failure. */
int drv_flash_init(void);

/* Full 24-bit JEDEC id. */
uint32_t drv_flash_read_id(void);

/* Plain read, any length, no WREN needed. */
int drv_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);

/* Program. Target area must already be erased. Handles page boundaries. */
int drv_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);

int drv_flash_erase_sector(uint32_t addr);   /* 4KB  */
int drv_flash_erase_block(uint32_t addr);    /* 64KB */

#endif /* __DRV_SPIFLASH_H */
