/**
 * @file    drv_dht11.c
 * @brief   DHT11 driver - bit-banged single-wire protocol on PG9.
 *
 * Timing: the 40-bit frame needs microsecond accuracy, so
 *   1. the 18ms start pulse uses a coarse delay (accuracy irrelevant)
 *   2. the bit train runs inside taskENTER_CRITICAL() (~4ms)
 *   3. delays use the DWT cycle counter, which keeps counting while
 *      interrupts are masked - SysTick-based delays would stall there.
 */

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_pinmap.h"
#include "drv_dht11.h"

#define DHT_EDGE_TIMEOUT_US   120u   /* longest expected edge is ~80us */
#define DHT_START_LOW_MS      18u

/* ------------------------------------------------------------ DWT delay */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void dwt_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);

    while ((DWT->CYCCNT - start) < ticks) { }
}

/* ------------------------------------------------------------- pin modes */
static void pin_as_output(void)
{
    GPIO_InitTypeDef g;

    g.Pin   = PIN_DHT11_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(PIN_DHT11_PORT, &g);
}

static void pin_as_input(void)
{
    GPIO_InitTypeDef g;

    g.Pin   = PIN_DHT11_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(PIN_DHT11_PORT, &g);
}

static uint8_t pin_level(void)
{
    return (HAL_GPIO_ReadPin(PIN_DHT11_PORT, PIN_DHT11_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

/* Wait until the line reaches level. 0 = reached, -1 = timeout. */
static int wait_level(uint8_t level, uint16_t timeout_us)
{
    while (timeout_us-- > 0u) {
        if (pin_level() == level) return 0;
        dwt_delay_us(1);
    }
    return -1;
}

/* ------------------------------------------------------------------ api */
int drv_dht11_init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();

    dwt_init();

    pin_as_output();
    HAL_GPIO_WritePin(PIN_DHT11_PORT, PIN_DHT11_PIN, GPIO_PIN_SET);
    return 0;
}

int drv_dht11_read(uint8_t *temp_c, uint8_t *humi_pct)
{
    uint8_t data[5] = { 0, 0, 0, 0, 0 };
    uint8_t i;
    int     ret = 0;

    if (temp_c == 0 || humi_pct == 0) return -1;

    /* 1. host start signal: pull low for >=18ms */
    pin_as_output();
    HAL_GPIO_WritePin(PIN_DHT11_PORT, PIN_DHT11_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(DHT_START_LOW_MS));

    /* 2. release the bus, sensor takes over within 20-40us */
    HAL_GPIO_WritePin(PIN_DHT11_PORT, PIN_DHT11_PIN, GPIO_PIN_SET);
    dwt_delay_us(30);
    pin_as_input();

    /* 3. the bit train must not be interrupted by the scheduler */
    taskENTER_CRITICAL();

    do {
        if (wait_level(0, DHT_EDGE_TIMEOUT_US) != 0) { ret = -2; break; }  /* response low  */
        if (wait_level(1, DHT_EDGE_TIMEOUT_US) != 0) { ret = -3; break; }  /* response high */
        if (wait_level(0, DHT_EDGE_TIMEOUT_US) != 0) { ret = -4; break; }  /* first data bit */

        for (i = 0; i < 40u; i++) {
            if (wait_level(1, DHT_EDGE_TIMEOUT_US) != 0) { ret = -5; break; }  /* 50us low ends */
            dwt_delay_us(40);                                                  /* sample point  */
            data[i >> 3] <<= 1;
            if (pin_level() != 0u) {
                data[i >> 3] |= 1u;
            }
            if (wait_level(0, DHT_EDGE_TIMEOUT_US) != 0) { ret = -6; break; }  /* high ends */
        }
    } while (0);

    taskEXIT_CRITICAL();

    if (ret != 0) return ret;

    if (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3])) return -7;

    *humi_pct = data[0];
    *temp_c   = data[2];
    return 0;
}
