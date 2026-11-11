/**
 * @file    modbus_rtu.c
 * @brief   Modbus RTU 主站实现（只读保持寄存器）。
 *
 * 判一帧结束靠"静默时间"，不靠长度：从站回异常帧只有 5 字节，
 * 按长度等会永远等不到。9600 8N1 一个字符 10 位约 1.04ms，
 * 3.5 字符约 3.65ms，这里取 4ms。
 *
 * 找帧头不假定缓冲第 0 字节就是帧头：半双工收发器如果 RE 常有效，
 * 自己发的 8 个字节也会进缓冲。所以扫描整段，取第一个 CRC 通过的候选。
 */

#include <string.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_uart.h"
#include "modbus_rtu.h"

#define MB_GAP_MS       4u      /* 帧内静默超过它就算收完 */
#define MB_TIMEOUT_MS   200u    /* 从发出到整帧收完的总时长上限（含首字节等待） */
#define MB_RX_MAX       64u     /* 一次响应的最大字节数，16 个寄存器 37 字节也够 */

/* CRC16/Modbus：初值 0xFFFF，每字节右移 8 次，移出位为 1 则异或 0xA001。
 * 校验时把整帧（含 CRC 两字节）一起送进来，结果为 0 即正确，
 * 比"重算再比对"少一次字节拼接。 */
static uint16_t mb_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFFu;
    uint8_t  i;

    while (n-- > 0u) {
        crc ^= (uint16_t)(*p++);
        for (i = 0u; i < 8u; i++) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

int modbus_read_regs(uint8_t addr, uint16_t reg_start, uint16_t cnt, uint16_t *out)
{
    uint8_t  req[8];
    uint8_t  rx[MB_RX_MAX];
    uint8_t  chunk[16];
    uint16_t crc;
    uint16_t n;
    uint16_t got;
    uint16_t pos;
    uint16_t len;
    uint16_t frame_pos;
    uint16_t frame_len;
    uint16_t i;
    uint32_t t0;
    uint32_t t_last;
    uint8_t  seen;              /* 见过结构合法的候选帧，用来区分"帧错"和"CRC 错" */

    if ((out == NULL) || (cnt == 0u) || (cnt > 16u)) {
        return MODBUS_ERR_PARAM;
    }

    /* ---- 请求帧：地址 功能码 起始(高,低) 数量(高,低) CRC(低,高) ---- */
    req[0] = addr;
    req[1] = 0x03u;
    req[2] = (uint8_t)(reg_start >> 8);
    req[3] = (uint8_t)(reg_start & 0xFFu);
    req[4] = (uint8_t)(cnt >> 8);
    req[5] = (uint8_t)(cnt & 0xFFu);
    crc    = mb_crc16(req, 6u);
    req[6] = (uint8_t)(crc & 0xFFu);        /* CRC 低字节先发 */
    req[7] = (uint8_t)(crc >> 8);

    /* 发之前清干净：上次超时的残帧、半双工回波都从这里进来 */
    drv_uart_flush(UART_RS485);

    if (drv_uart_write(UART_RS485, req, 8u, 50u) < 0) {
        return MODBUS_ERR_SEND;
    }

    /* ---- 收：读空就等 1ms，静默 4ms 判结束，总超时 200ms ---- */
    n      = 0u;
    t0     = HAL_GetTick();
    t_last = t0;

    while ((HAL_GetTick() - t0) < MB_TIMEOUT_MS) {
        got = drv_uart_read(UART_RS485, chunk, (uint16_t)sizeof(chunk));

        if (got > 0u) {
            if ((uint16_t)(n + got) > MB_RX_MAX) {
                return MODBUS_ERR_FRAME;    /* 长度已经超上限，肯定不是我们要的帧 */
            }
            memcpy(&rx[n], chunk, got);
            n     += got;
            t_last = HAL_GetTick();
            continue;                       /* 有数据就接着读，不算帧尾 */
        }

        if ((n > 0u) && ((HAL_GetTick() - t_last) >= MB_GAP_MS)) {
            break;                          /* 静默够了 = 一帧结束 */
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (n == 0u) {
        return MODBUS_ERR_TIMEOUT;
    }

    /* ---- 找帧头：响应第 3 字节是数据字节数(=2*cnt)，回波第 3 字节是起始地址高字节 ---- */
    seen      = 0u;
    frame_pos = 0u;
    frame_len = 0u;

    for (pos = 0u; (uint16_t)(pos + 5u) <= n; pos++) {
        if (rx[pos] != addr) {
            continue;
        }

        if ((rx[pos + 1u] == 0x03u) && (rx[pos + 2u] == (uint8_t)(cnt * 2u))) {
            len = (uint16_t)(5u + cnt * 2u);        /* 正常响应 */
        } else if (rx[pos + 1u] == 0x83u) {
            len = 5u;                               /* 异常响应 */
        } else {
            continue;
        }

        if ((uint16_t)(pos + len) > n) {
            continue;                               /* 还没收全，再往后面找 */
        }

        seen = 1u;
        if (mb_crc16(&rx[pos], len) != 0u) {
            continue;                               /* 这个候选是回波或残帧 */
        }

        frame_pos = pos;
        frame_len = len;
        break;
    }

    if (frame_len == 0u) {
        return (seen != 0u) ? MODBUS_ERR_CRC : MODBUS_ERR_FRAME;
    }

    if (rx[frame_pos + 1u] == 0x83u) {
        return MODBUS_ERR_EXCEPT;
    }

    /* 寄存器值是大端（高字节先），和 CRC 的低字节先发正好相反 */
    for (i = 0u; i < cnt; i++) {
        out[i] = (uint16_t)(((uint16_t)rx[frame_pos + 3u + i * 2u] << 8)
                            | (uint16_t)rx[frame_pos + 4u + i * 2u]);
    }

    return MODBUS_OK;
}
