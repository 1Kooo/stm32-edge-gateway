/**
 * @file    boot_uart.h
 * @brief   Bootloader 的串口输出：轮询发送，不依赖 SysTick、不依赖 RTOS。
 */

#ifndef __BOOT_UART_H
#define __BOOT_UART_H

#include "stm32f4xx_hal.h"

void boot_uart_init(void);
void boot_printf(const char *fmt, ...);

#endif /* __BOOT_UART_H */
