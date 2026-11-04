/**
 * @file    fw_image.c
 * @brief   读外部 Flash 暂存区的固件头。
 *
 * 只做读和校验，不擦不写 —— 这一层永远不改动暂存区内容。
 */

#include "fw_image.h"
#include "ota_flag.h"       /* 复用 ota_crc32，两边算法必须一致 */
#include "drv_spiflash.h"

int fw_head_read(fw_header_t *out)
{
    uint8_t *raw;
    uint32_t i;
    uint32_t calc;

    raw = (uint8_t *)out;

    /* 先清零：读失败时上层打印出来的是 0，而不是栈上的随机值，
     * 免得把一串垃圾当成真实数据去分析。 */
    for (i = 0u; i < sizeof(fw_header_t); i++) {
        raw[i] = 0u;
    }

    if (drv_flash_read(FW_STAGE_BASE, raw, sizeof(fw_header_t)) != 0) {
        return FW_ERR_READ;
    }

    /* 空白 flash 读出来是全 0xFF，magic 对不上，到此为止 */
    if (out->magic != FW_MAGIC) {
        return FW_ERR_MAGIC;
    }

    /* hdr_crc 覆盖自己前面的 16 字节 */
    calc = ota_crc32(out, 16u);
    if (calc != out->hdr_crc) {
        return FW_ERR_CRC;
    }

    return FW_OK;
}
