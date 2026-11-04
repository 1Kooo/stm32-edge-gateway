/**
 * @file    ota_flag.c
 * @brief   启动标志的读写：内部 Flash 两个扇区各存一份，带 CRC32。
 */

#include "ota_flag.h"
#include "stm32f4xx_hal.h"

/* ============================ CRC32 ============================ */

/* 逐位算法，不占 1KB 查表空间。标志只有 36 字节，慢一点无所谓；
 * 校验 400KB 固件时分块喂（约 0.15 秒 @168MHz），也还能接受。 */
uint32_t ota_crc32(const void *data, uint32_t len)
{
    return ota_crc32_end(ota_crc32_update(ota_crc32_begin(), data, len));
}

/* 增量式三段接口：整块数据放不进 RAM 时（比如校验暂存区 400KB 固件）
 * 必须分块喂。用法：begin 拿初值 → update 逐块喂 → end 取最终值。
 * 算法和一次性版完全同一个位运算循环，两处结果必然一致。 */
uint32_t ota_crc32_begin(void)
{
    return 0xFFFFFFFFu;
}

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

    /* 备份份写失败不致命：主份已经好了，下次启动还能读主份。
     * 但如果是主份坏了才会用到它，所以这里不因失败中断流程。 */
    (void)flag_write_one(OTA_FLAG_B_ADDR, &f, FLASH_SECTOR_3);
    return 0;
}

int ota_flag_bump_try(ota_flag_t *f)
{
    /* seq 必须一起递增：写两份时如果中途掉电，只有 seq 更大才能被
     * ota_flag_load 选中，否则可能读到旧的 boot_try 白数一遍。 */
    f->boot_try++;
    f->seq++;
    return ota_flag_save(f);
}
