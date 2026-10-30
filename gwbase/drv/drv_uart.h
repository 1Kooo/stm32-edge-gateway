/**
 * @file    drv_uart.h
 * @brief   Unified driver for the three UARTs of this board.
 *
 *   UART_DEBUG : USART1  PA9/PA10   115200  printf console
 *   UART_RS485 : USART2  PA2/PA3     9600   DE = PG8
 *   UART_WIFI  : USART3  PB10/PB11  115200  ESP-01S
 *
 * RX path: RXNE interrupt -> ring buffer. Reader polls drv_uart_read().
 * TX path: blocking HAL_UART_Transmit (RS485 adds TC-based DE release).
 */

#ifndef __DRV_UART_H
#define __DRV_UART_H

#include <stdint.h>

typedef enum {
    UART_DEBUG = 0,
    UART_RS485,
    UART_WIFI,
    UART_ID_MAX
} uart_id_t;

/* 0 on success, negative on error. */
int drv_uart_init(uart_id_t id, uint32_t baud);

/* Blocking send. Returns bytes sent, or negative on error. */
int drv_uart_write(uart_id_t id, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/* Same as above with an implicit "\r\n". */
int drv_uart_write_str(uart_id_t id, const char *s);

/* Number of bytes waiting in the RX ring buffer. */
uint16_t drv_uart_available(uart_id_t id);

/* Copy up to maxlen bytes out of the RX ring buffer. Returns count. */
uint16_t drv_uart_read(uart_id_t id, uint8_t *buf, uint16_t maxlen);

/* Drop everything currently buffered. */
void drv_uart_flush(uart_id_t id);

/* Number of bytes lost to a full ring buffer since boot. */
uint32_t drv_uart_overflow(uart_id_t id);

/* RX callback. Runs in interrupt context when the ring buffer goes from
 * empty to non-empty. Keep it short: no printing, no blocking.
 * This is what lets the driver stay RTOS-agnostic. */
typedef void (*uart_rx_cb_t)(uart_id_t id);
void drv_uart_set_rx_callback(uart_id_t id, uart_rx_cb_t cb);

#endif /* __DRV_UART_H */
