/**
 * @file    fw_image.h
 * @brief   外部 Flash 暂存区里的固件头：布局定义 + 读取校验。
 *
 * 外部 Flash 布局（16MB 芯片）：
 *   0x000000 ~ 0x0FFFFF  暂存区（1MB，APP 下载写入，boot 校验搬运）
 *   0x100000 ~ 0x1FFFFF  黄金备份区（1MB，APP 确认成功后写入裸正文，
 *                         长度/CRC 记在标志的 golden_len/golden_crc，
 *                         boot 回滚时从这里搬回，boot 对它只读）
 *
 * 暂存区细布局：
 *
 *   0x000000  ┌────────────────────┐
 *             │ 固件头 20 字节      │  前 256 字节整段预留，
 *             │ （其余填 0）        │  以后要加签名、分块表直接往后放
 *   0x000100  ├────────────────────┤
 *             │ 固件正文            │
 *             └────────────────────┘
 *
 * 这个结构是 boot 和 APP 之间的契约：
 *   8.2 由 APP 写（收完固件后先写头，再写正文）
 *   8.3 由 boot 读（校验通过才搬进内部 Flash）
 * 两边必须一字不差，改字段要两个工程一起改、一起重烧。
 */

#ifndef __FW_IMAGE_H
#define __FW_IMAGE_H

#include <stdint.h>

#define FW_STAGE_BASE      0x000000u    /* 暂存区起始（外部 Flash） */
#define FW_GOLDEN_BASE     0x100000u    /* 黄金备份区起始：暂存区占 1MB，紧随其后 */
#define FW_HDR_SIZE        20u          /* 结构体本身的字节数 */
#define FW_HDR_RESERVED    256u         /* 头部整段预留，正文从这之后开始 */
#define FW_DATA_BASE       (FW_STAGE_BASE + FW_HDR_RESERVED)

/* 'G','W','F','W' 按小端拼出来的 uint32 */
#define FW_MAGIC           0x57465747u

/* 暂存区最大能放多少正文：1MB 减掉头部预留 */
#define FW_MAX_LEN         (1024u * 1024u - FW_HDR_RESERVED)

/* fw_head_read 的返回值 */
#define FW_OK              0
#define FW_ERR_READ       (-1)          /* SPI 读失败 */
#define FW_ERR_MAGIC      (-2)          /* magic 不对，暂存区是空的或者别的数据 */
#define FW_ERR_CRC        (-3)          /* 头部 CRC 不符，头被写坏了 */

typedef struct {
    uint32_t magic;         /* 必须等于 FW_MAGIC */
    uint32_t version;       /* 高 16 位主版本，低 16 位次版本 */
    uint32_t fw_len;        /* 固件正文字节数 */
    uint32_t fw_crc;        /* 固件正文的 CRC32 */
    uint32_t hdr_crc;       /* 上面 16 字节的 CRC32（本字段不参与计算） */
} fw_header_t;              /* 20 字节 */

/**
 * @brief 从外部 Flash 暂存区读固件头并校验。
 *
 * 不管成功与否都会把读到的内容填进 out，方便上层打印诊断。
 * 失败时 out 里的内容不可信（可能是全 0xFF）。
 *
 * @param out 输出，不能为 NULL
 * @return FW_OK / FW_ERR_READ / FW_ERR_MAGIC / FW_ERR_CRC
 */
int fw_head_read(fw_header_t *out);

#endif /* __FW_IMAGE_H */
