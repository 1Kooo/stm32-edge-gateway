/**
 * @file    fw_install.h
 * @brief   8.3：把外部暂存区的新固件装进内部 Flash APP 区。
 */

#ifndef __FW_INSTALL_H
#define __FW_INSTALL_H

#include <stdint.h>
#include "fw_image.h"

/* fw_install_run 的返回值 */
#define FW_INS_OK             0
#define FW_INS_ERR_RANGE     (-1)   /* 长度越界，装不进 APP 区 */
#define FW_INS_ERR_READ      (-2)   /* 读外部 Flash 失败 */
#define FW_INS_ERR_STAGE_CRC (-3)   /* 外部 Flash 源数据 CRC 不对（此时尚未动 APP） */
#define FW_INS_ERR_ERASE     (-4)   /* 擦内部 Flash APP 区失败 */
#define FW_INS_ERR_PROGRAM   (-5)   /* 写内部 Flash 失败 */
#define FW_INS_ERR_VERIFY    (-6)   /* 写完读回比对不一致 */
#define FW_INS_ERR_FINAL_CRC (-7)   /* 装完后整体 CRC 对不上 */

/**
 * @brief 安装暂存区固件到 APP 区（校验 → 擦 → 搬 → 终检）。
 *
 * 前提：hdr 已通过 fw_head_read 校验（magic + 头部 CRC 都对），
 * 且与标志里的 fw_len / fw_crc 记录一致（main.c 负责比对）。
 *
 * 危险点：一旦开始擦除，老固件就没了。中途失败只能停机等人重烧，
 * 回滚是 8.4 的事。
 *
 * @param hdr 暂存区固件头
 * @return FW_INS_OK 或负错误码（详细原因已用 boot_printf 打出）
 */
int fw_install_run(const fw_header_t *hdr);

/**
 * @brief 8.4 回滚：把黄金备份搬回 APP 区。
 *
 * 长度和 CRC 从标志的 golden_len / golden_crc 读出（黄金区存裸正文）。
 * 校验失败或搬运失败同样只能停机 —— 老固件已经被失败的新固件顶掉了。
 *
 * @param fw_len 黄金备份字节数
 * @param fw_crc 黄金备份 CRC32
 * @return FW_INS_OK 或负错误码
 */
int fw_restore_golden(uint32_t fw_len, uint32_t fw_crc);

#endif /* __FW_INSTALL_H */
