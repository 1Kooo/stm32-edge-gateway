/**
 * @file    modbus_rtu.h
 * @brief   Modbus RTU 主站，只做功能码 0x03（读保持寄存器）。
 *
 * 总线上只挂 HZ8706 温湿度变送器，所以只留一条路径：
 * 发 8 字节请求 -> 收 5+2N 字节响应 -> 校验 CRC。
 * 写寄存器 / 功能码 0x01/0x04 都没做，需要时再加。
 *
 * 阻塞式，最长约 250ms（发送 20 + 等 200），
 * 不要放进 ui_task（要求 5ms 一循环）或 net_task 的内层循环。
 */

#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include <stdint.h>

/* 返回值：0 成功，负数为错误码 */
#define MODBUS_OK            0
#define MODBUS_ERR_PARAM    (-1)    /* 参数非法：cnt 为 0 / 超过 16 / out 为空 */
#define MODBUS_ERR_SEND     (-2)    /* 串口发送失败 */
#define MODBUS_ERR_TIMEOUT  (-3)    /* 超时：一个字节都没收到 */
#define MODBUS_ERR_FRAME    (-4)    /* 缓冲里找不到结构合法的帧 */
#define MODBUS_ERR_CRC      (-5)    /* 找到帧了，但 CRC 不过 */
#define MODBUS_ERR_EXCEPT   (-6)    /* 从站回了异常帧 0x83 */

/* 读 cnt 个保持寄存器到 out[0..cnt-1]，值已按大端拼好。
 * 温度这类有符号量由调用方强转 int16_t，驱动层不猜语义。 */
int modbus_read_regs(uint8_t addr, uint16_t reg_start, uint16_t cnt, uint16_t *out);

#endif /* __MODBUS_RTU_H */
