/**
 * @file    drv_adc.h
 * @brief   ADC1 inputs for the analog sensor modules.
 *
 *   ADC_CH_LIGHT : PC1 / ADC1_IN11 / 光敏电阻模块 AO
 *   ADC_CH_TEMP  : PC2 / ADC1_IN12 / 热敏电阻模块 AO
 *
 * Both channels sample 8 times and average, which kills most of the noise
 * a breadboard picks up.
 */

#ifndef __DRV_ADC_H
#define __DRV_ADC_H

#include <stdint.h>

typedef enum {
    ADC_CH_LIGHT = 0,
    ADC_CH_TEMP,
    ADC_CH_MAX
} adc_ch_t;

/* 0 on success, negative on failure. */
int drv_adc_init(void);

/* Averaged raw value, 0..4095 (12 bit). */
uint16_t drv_adc_read_raw(adc_ch_t ch);

/* Same value converted with VREF = 3300 mV. */
uint16_t drv_adc_read_mv(adc_ch_t ch);

#endif /* __DRV_ADC_H */
