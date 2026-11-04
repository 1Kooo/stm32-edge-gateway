/**
 * @file    boot_uart.c
 * @brief   USART1 轮询输出（PA9/PA10，115200）。
 *
 * 为什么不搬 APP 的 Public/usart.c：
 * 那份实现依赖 SysTick_Init 做延时和超时，Bootloader 里没有 SysTick 也要能打印。
 * 这里只做「等发送寄存器空 → 写一个字节」，足够了。
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "boot_uart.h"
#include "bsp_pinmap.h"

static UART_HandleTypeDef s_uart;

void boot_uart_init(void)
{
    GPIO_InitTypeDef gpio;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9 = TX, PA10 = RX，都设成复用推挽 */
    gpio.Pin       = PIN_UART1_TX_PIN | PIN_UART1_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = PIN_UART1_AF;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_uart.Instance          = PIN_UART1_INSTANCE;
    s_uart.Init.BaudRate     = PIN_UART1_BAUD;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_uart);
}

/* 逐字节阻塞发送。超时 10ms 只是防死等，正常一字节不到 0.1ms。 */
static void uart_send(const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        HAL_UART_Transmit(&s_uart, (uint8_t *)&buf[i], 1u, 10u);
    }
}

void boot_printf(const char *fmt, ...)
{
    char    buf[128];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    uart_send((const uint8_t *)buf, (uint32_t)n);
}
