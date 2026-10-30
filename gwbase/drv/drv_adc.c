/**
 * @file    drv_adc.c
 * @brief   ADC1 polling driver for two analog sensor modules.
 *
 * Sampling time is set to 480 cycles on purpose: thermistor dividers have
 * source impedance in the 10K range, short sampling windows would load the
 * node and read wrong.
 */

#include "stm32f4xx_hal.h"
#include "bsp_pinmap.h"
#include "drv_adc.h"

#define ADC_AVG_TIMES    8u
#define ADC_VREF_MV      3300u
#define ADC_FULL_SCALE   4096u
#define ADC_TIMEOUT_MS   10u

static ADC_HandleTypeDef s_adc;

static uint32_t ch_to_channel(adc_ch_t ch)
{
    return (ch == ADC_CH_LIGHT) ? ADC_CHANNEL_11 : ADC_CHANNEL_12;
}

static uint16_t adc_convert_once(uint32_t channel)
{
    ADC_ChannelConfTypeDef c;

    c.Channel      = channel;
    c.Rank         = 1;
    c.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    c.Offset       = 0;

    if (HAL_ADC_ConfigChannel(&s_adc, &c) != HAL_OK) return 0u;

    HAL_ADC_Start(&s_adc);

    if (HAL_ADC_PollForConversion(&s_adc, ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(&s_adc);
        return 0u;
    }

    {
        uint16_t v = (uint16_t)HAL_ADC_GetValue(&s_adc);
        HAL_ADC_Stop(&s_adc);
        return v;
    }
}

int drv_adc_init(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* both AO pins as analog inputs */
    g.Pin  = PIN_ADC_LIGHT_PIN | PIN_ADC_TEMP_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PIN_ADC_LIGHT_PORT, &g);

    s_adc.Instance                   = ADC1;
    s_adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_adc.Init.Resolution            = ADC_RESOLUTION_12B;
    s_adc.Init.ScanConvMode          = DISABLE;
    s_adc.Init.ContinuousConvMode    = DISABLE;
    s_adc.Init.DiscontinuousConvMode = DISABLE;
    s_adc.Init.NbrOfDiscConversion   = 0;
    s_adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_adc.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    s_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_adc.Init.NbrOfConversion       = 1;
    s_adc.Init.DMAContinuousRequests = DISABLE;
    s_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&s_adc) != HAL_OK) return -1;

    /* one throwaway conversion: the first sample after power-up is unreliable */
    (void)adc_convert_once(ADC_CHANNEL_11);

    return 0;
}

uint16_t drv_adc_read_raw(adc_ch_t ch)
{
    uint32_t sum = 0;
    uint8_t  i;

    if (ch >= ADC_CH_MAX) return 0u;

    for (i = 0; i < ADC_AVG_TIMES; i++) {
        sum += adc_convert_once(ch_to_channel(ch));
    }
    return (uint16_t)(sum / ADC_AVG_TIMES);
}

uint16_t drv_adc_read_mv(adc_ch_t ch)
{
    uint32_t raw = drv_adc_read_raw(ch);
    return (uint16_t)((raw * ADC_VREF_MV) / ADC_FULL_SCALE);
}
