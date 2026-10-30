/**
 * @file    bsp_pinmap.h
 * @brief   Single source of truth for every GPIO used in this project.
 *
 * RULE: no other file may hard-code a port/pin. If you need a pin,
 *       add it here first.
 *
 * Board: Wulumu "Jinxing" STM32F407ZGT6
 */

#ifndef __BSP_PINMAP_H
#define __BSP_PINMAP_H

#include "system.h"

/* ============ 端口占用速查（改任何引脚前先看这张表）============
 * PA0  KEY_UP          PA2/PA3   USART2 (485)      PA5/PA6/PA7  LCD FSMC
 * PA9/PA10 USART1      PA11/PA12 USB               PA13/PA14    SWD
 *
 * PB0  TP_SCK   PB1 TP_PEN   PB2 TP_MISO   PB3/PB4/PB5 SPI1(Flash)
 * PB8/PB9 I2C(24C02)   PB10/PB11 USART3(WiFi)   PB14 FLASH_CS
 * PB12/PB13/PB15  LCD FSMC（与 SRAM 共用数据线，靠片选区分）
 *
 * PC0  ESP_EN          PC1 ADC_LIGHT   PC2 ADC_TEMP   PC13 TP_CS
 *
 * PE2/PE3/PE4 KEY0-2   PF6 ESP_RST   PF9/PF10 LED    PG8 RS485_DE
 * PG9  DHT11           PD/PE/PF/PG 其余多数被 FSMC(屏+SRAM) 占用
 * ================================================================ */

/* =========================================================
 *  USART1  - debug console (CH340 -> PC)
 * ========================================================= */
#define PIN_UART1_INSTANCE      USART1
#define PIN_UART1_TX_PORT       GPIOA
#define PIN_UART1_TX_PIN        GPIO_PIN_9
#define PIN_UART1_RX_PORT       GPIOA
#define PIN_UART1_RX_PIN        GPIO_PIN_10
#define PIN_UART1_AF            GPIO_AF7_USART1
#define PIN_UART1_BAUD          115200u

/* =========================================================
 *  USART2  - RS485 / Modbus-RTU
 * ========================================================= */
#define PIN_UART2_INSTANCE      USART2
#define PIN_UART2_TX_PORT       GPIOA
#define PIN_UART2_TX_PIN        GPIO_PIN_2
#define PIN_UART2_RX_PORT       GPIOA
#define PIN_UART2_RX_PIN        GPIO_PIN_3
#define PIN_UART2_AF            GPIO_AF7_USART2
#define PIN_UART2_BAUD          9600u

/* RS485 direction: 1 = transmit, 0 = receive */
#define PIN_RS485_DE_PORT       GPIOG
#define PIN_RS485_DE_PIN        GPIO_PIN_8

/* =========================================================
 *  USART3  - ESP-01S (WiFi, AT firmware)
 * ========================================================= */
#define PIN_UART3_INSTANCE      USART3
#define PIN_UART3_TX_PORT       GPIOB
#define PIN_UART3_TX_PIN        GPIO_PIN_10
#define PIN_UART3_RX_PORT       GPIOB
#define PIN_UART3_RX_PIN        GPIO_PIN_11
#define PIN_UART3_AF            GPIO_AF7_USART3
#define PIN_UART3_BAUD          115200u

#define PIN_ESP_RST_PORT        GPIOF
#define PIN_ESP_RST_PIN         GPIO_PIN_6
#define PIN_ESP_EN_PORT         GPIOC
#define PIN_ESP_EN_PIN          GPIO_PIN_0

/* =========================================================
 *  SPI1 + external flash (EN25Q128, 16MB)
 * ========================================================= */
#define PIN_SPI1_INSTANCE       SPI1
#define PIN_SPI1_SCK_PORT       GPIOB
#define PIN_SPI1_SCK_PIN        GPIO_PIN_3
#define PIN_SPI1_MISO_PORT      GPIOB
#define PIN_SPI1_MISO_PIN       GPIO_PIN_4
#define PIN_SPI1_MOSI_PORT      GPIOB
#define PIN_SPI1_MOSI_PIN       GPIO_PIN_5
#define PIN_SPI1_AF             GPIO_AF5_SPI1

#define PIN_FLASH_CS_PORT       GPIOB
#define PIN_FLASH_CS_PIN        GPIO_PIN_14

/* =========================================================
 *  LCD (FSMC 16-bit) + backlight
 * ========================================================= */
#define PIN_LCD_BL_PORT         GPIOB
#define PIN_LCD_BL_PIN          GPIO_PIN_15

/* =========================================================
 *  Touch panel XPT2046 (bit-banged SPI)
 * ========================================================= */
#define PIN_TP_SCK_PORT         GPIOB
#define PIN_TP_SCK_PIN          GPIO_PIN_0
#define PIN_TP_MISO_PORT        GPIOB
#define PIN_TP_MISO_PIN         GPIO_PIN_2
#define PIN_TP_MOSI_PORT        GPIOF
#define PIN_TP_MOSI_PIN         GPIO_PIN_11
#define PIN_TP_CS_PORT          GPIOC
#define PIN_TP_CS_PIN           GPIO_PIN_13
#define PIN_TP_PEN_PORT         GPIOB
#define PIN_TP_PEN_PIN          GPIO_PIN_1

/* =========================================================
 *  EEPROM 24C02 (bit-banged I2C)
 * ========================================================= */
#define PIN_I2C_SCL_PORT        GPIOB
#define PIN_I2C_SCL_PIN         GPIO_PIN_8
#define PIN_I2C_SDA_PORT        GPIOB
#define PIN_I2C_SDA_PIN         GPIO_PIN_9

/* =========================================================
 *  LED
 * ========================================================= */
#define PIN_LED1_PORT           GPIOF
#define PIN_LED1_PIN            GPIO_PIN_9
#define PIN_LED2_PORT           GPIOF
#define PIN_LED2_PIN            GPIO_PIN_10

/* =========================================================
 *  Keys
 * ========================================================= */
#define PIN_KEY_PORT            GPIOE
#define PIN_KEY0_PIN            GPIO_PIN_4
#define PIN_KEY1_PIN            GPIO_PIN_3
#define PIN_KEY2_PIN            GPIO_PIN_2
#define PIN_KEY_UP_PORT         GPIOA
#define PIN_KEY_UP_PIN          GPIO_PIN_0

/* =========================================================
 *  DHT11 (single-wire, reserved for step 5)
 * ========================================================= */
#define PIN_DHT11_PORT          GPIOG
#define PIN_DHT11_PIN           GPIO_PIN_9

/* =========================================================
 *  ADC inputs - analog sensor modules (AO pin)
 *  ADC1_IN11 / ADC1_IN12, channel mapping lives in drv_adc.c
 * ========================================================= */
#define PIN_ADC_LIGHT_PORT      GPIOC
#define PIN_ADC_LIGHT_PIN       GPIO_PIN_1     /* light sensor  -> IN11 */
#define PIN_ADC_TEMP_PORT       GPIOC
#define PIN_ADC_TEMP_PIN        GPIO_PIN_2     /* thermistor    -> IN12 */

#endif /* __BSP_PINMAP_H */
