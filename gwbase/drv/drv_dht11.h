/**
 * @file    drv_dht11.h
 * @brief   DHT11 single-wire temperature / humidity sensor (PG9).
 */

#ifndef __DRV_DHT11_H
#define __DRV_DHT11_H

#include <stdint.h>

/* 0 on success, negative on error. */
int drv_dht11_init(void);

/* Both outputs are integer units: temp in C, humi in %RH.
 * DHT11 resolution is 1C / 1%RH, so no scaling here - callers decide. */
int drv_dht11_read(uint8_t *temp_c, uint8_t *humi_pct);

#endif /* __DRV_DHT11_H */
