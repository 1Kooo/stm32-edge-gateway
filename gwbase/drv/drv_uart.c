/**
 * @file    drv_uart.c
 * @brief   Unified UART driver for the three ports of this board.
 *
 *   UART_DEBUG : USART1  PA9/PA10   115200  printf console (TX only here)
 *   UART_RS485 : USART2  PA2/PA3     9600   DE = PG8
 *   UART_WIFI  : USART3  PB10/PB11  115200  ESP-01S
 *
 * Note: HAL_UART_MspInit() is already defined in Public/usart.c and only
 * handles USART1, so this driver configures its own GPIOs.
 * USART1_IRQHandler also lives in Public/usart.c - do NOT define it here.
 */

#include "stm32f4xx_hal.h"
#include "drv_uart.h"
#include "bsp_pinmap.h"
#include "ringbuf.h"

/* 512 bytes per port: enough for an AT modem burst at 115200. */
#define UART_RX_BUF_SIZE    512u
#define RS485_TC_TIMEOUT_MS 10u

typedef struct {
    UART_HandleTypeDef  huart;
    ringbuf_t           rx;
    uint8_t             rx_storage[UART_RX_BUF_SIZE];
    uint8_t             in_use;
    uart_rx_cb_t        rx_cb;          /* called from ISR on first byte */
} uart_ctx_t;

static uart_ctx_t s_uart[UART_ID_MAX];

/* ---------------------------------------------------------------- clocks */
static void uart_clock_enable(uart_id_t id)
{
    switch (id) {
    case UART_DEBUG:
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case UART_RS485:
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();      /* PG8 = DE */
        break;
    case UART_WIFI:
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();      /* PB10 / PB11 */
        __HAL_RCC_GPIOF_CLK_ENABLE();      /* PF6  = ESP RST */
        __HAL_RCC_GPIOC_CLK_ENABLE();      /* PC0  = ESP EN  */
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------- plain output pin */
/* Write the level first, then switch to output: avoids a power-on glitch. */
static void gpio_output_init(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level)
{
    GPIO_InitTypeDef g;

    HAL_GPIO_WritePin(port, pin, level);

    g.Pin   = pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(port, &g);
}

/* ----------------------------------------------------------- AF pin setup */
static void uart_gpio_config(uart_id_t id)
{
    GPIO_InitTypeDef g;

    switch (id) {
    case UART_DEBUG:
        g.Pin       = PIN_UART1_TX_PIN | PIN_UART1_RX_PIN;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FAST;
        g.Alternate = PIN_UART1_AF;
        HAL_GPIO_Init(PIN_UART1_TX_PORT, &g);
        break;

    case UART_RS485:
        g.Pin       = PIN_UART2_TX_PIN | PIN_UART2_RX_PIN;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FAST;
        g.Alternate = PIN_UART2_AF;
        HAL_GPIO_Init(PIN_UART2_TX_PORT, &g);

        /* DE low = receive, so we never drive the bus right after power-up */
        gpio_output_init(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_RESET);
        break;

    case UART_WIFI:
        g.Pin       = PIN_UART3_TX_PIN | PIN_UART3_RX_PIN;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FAST;
        g.Alternate = PIN_UART3_AF;
        HAL_GPIO_Init(PIN_UART3_TX_PORT, &g);

        /* ESP-01S: EN high = enabled, RST high = normal run */
        gpio_output_init(PIN_ESP_RST_PORT, PIN_ESP_RST_PIN, GPIO_PIN_SET);
        gpio_output_init(PIN_ESP_EN_PORT,  PIN_ESP_EN_PIN,  GPIO_PIN_SET);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ init */
int drv_uart_init(uart_id_t id, uint32_t baud)
{
    uart_ctx_t *ctx;

    if (id >= UART_ID_MAX) return -1;

    ctx = &s_uart[id];
    if (ctx->in_use) return 0;                       /* idempotent */

    if (rb_init(&ctx->rx, ctx->rx_storage, UART_RX_BUF_SIZE) != 0) return -2;

    uart_clock_enable(id);
    uart_gpio_config(id);                            /* must precede HAL_UART_Init */

    switch (id) {
    case UART_DEBUG: ctx->huart.Instance = USART1; break;
    case UART_RS485: ctx->huart.Instance = USART2; break;
    case UART_WIFI:  ctx->huart.Instance = USART3; break;
    default: return -1;
    }

    ctx->huart.Init.BaudRate     = baud;
    ctx->huart.Init.WordLength   = UART_WORDLENGTH_8B;
    ctx->huart.Init.StopBits     = UART_STOPBITS_1;
    ctx->huart.Init.Parity       = UART_PARITY_NONE;
    ctx->huart.Init.Mode         = UART_MODE_TX_RX;
    ctx->huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    ctx->huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&ctx->huart) != HAL_OK) return -3;

    /* USART1 RX is owned by Public/usart.c (its own ISR), keep ours off. */
    if (id != UART_DEBUG) {
        __HAL_UART_ENABLE_IT(&ctx->huart, UART_IT_RXNE);

        switch (id) {
        case UART_RS485:
            /* Must be numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
             * (5) for the RX hook to call FreeRTOS FromISR APIs safely. */
            HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
            HAL_NVIC_EnableIRQ(USART2_IRQn);
            break;
        case UART_WIFI:
            HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
            HAL_NVIC_EnableIRQ(USART3_IRQn);
            break;
        default:
            break;
        }
    }

    ctx->in_use = 1;
    return 0;
}

/* -------------------------------------------------------------------- ISR */
static void uart_isr(uart_id_t id)
{
    uart_ctx_t    *ctx = &s_uart[id];
    USART_TypeDef *u   = ctx->huart.Instance;
    uint32_t       sr  = u->SR;

    if (sr & USART_SR_RXNE) {
        /* Reading DR clears both RXNE and ORE, so test RXNE first. */
        uint8_t ch        = (uint8_t)(u->DR & 0xFFu);
        uint8_t was_empty = (rb_used(&ctx->rx) == 0u);

        rb_write(&ctx->rx, &ch, 1);

        /* Notify only on the empty -> non-empty edge: one wake-up per frame
         * instead of one per byte. */
        if (was_empty && ctx->rx_cb != 0) {
            ctx->rx_cb(id);
        }
    } else if (sr & USART_SR_ORE) {
        /* Lone overrun: read DR once, otherwise the IRQ keeps firing. */
        (void)u->DR;
    }
}

void USART2_IRQHandler(void)
{
    uart_isr(UART_RS485);
}

void USART3_IRQHandler(void)
{
    uart_isr(UART_WIFI);
}

/* --------------------------------------------------------------------- TX */
int drv_uart_write(uart_id_t id, const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uart_ctx_t       *ctx;
    HAL_StatusTypeDef st;

    if (id >= UART_ID_MAX || buf == 0 || len == 0) return -1;
    if (!s_uart[id].in_use) return -1;

    ctx = &s_uart[id];

    if (id == UART_RS485) {
        HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_SET);
    }

    st = HAL_UART_Transmit(&ctx->huart, (uint8_t *)buf, len, timeout_ms);

    if (id == UART_RS485) {
        /* Wait until the last stop bit has actually left the shifter,
         * otherwise DE drops early and truncates the frame. */
        uint32_t t0 = HAL_GetTick();
        while ((ctx->huart.Instance->SR & USART_SR_TC) == 0) {
            if ((HAL_GetTick() - t0) > RS485_TC_TIMEOUT_MS) break;
        }
        (void)ctx->huart.Instance->SR;               /* read SR ... */
        (void)ctx->huart.Instance->DR;               /* ... then DR clears TC */
        HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_RESET);
    }

    return (st == HAL_OK) ? (int)len : -2;
}

int drv_uart_write_str(uart_id_t id, const char *s)
{
    uint16_t n = 0;

    if (s == 0) return -1;
    while (s[n] != '\0' && n < 0xFFFEu) n++;

    if (drv_uart_write(id, (const uint8_t *)s, n, 100u) < 0) return -1;
    return drv_uart_write(id, (const uint8_t *)"\r\n", 2u, 100u);
}

/* --------------------------------------------------------------------- RX */
uint16_t drv_uart_available(uart_id_t id)
{
    if (id >= UART_ID_MAX) return 0;
    return rb_used(&s_uart[id].rx);
}

uint16_t drv_uart_read(uart_id_t id, uint8_t *buf, uint16_t maxlen)
{
    if (id >= UART_ID_MAX || buf == 0) return 0;
    return rb_read(&s_uart[id].rx, buf, maxlen);
}

void drv_uart_flush(uart_id_t id)
{
    if (id >= UART_ID_MAX) return;
    rb_flush(&s_uart[id].rx);
}

uint32_t drv_uart_overflow(uart_id_t id)
{
    if (id >= UART_ID_MAX) return 0;
    return s_uart[id].rx.overflow;
}

void drv_uart_set_rx_callback(uart_id_t id, uart_rx_cb_t cb)
{
    if (id >= UART_ID_MAX) return;
    s_uart[id].rx_cb = cb;
}
