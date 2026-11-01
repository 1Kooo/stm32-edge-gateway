/**
 * @file    app_main.c
 * @brief   Application entry - Step 2: three UARTs online.
 *
 * Behaviour:
 *   - init USART2 (RS485) and USART3 (WiFi)
 *   - report status on the debug console and on the LCD
 *   - main loop: RS485 echoes bytes back, WiFi traffic is dumped to console
 *
 * USART1 is owned by Public/usart.c (printf), so we never re-init it here.
 */

#include <string.h>

#include "app_main.h"
#include "drv_uart.h"
#include "drv_spiflash.h"
#include "bsp_pinmap.h"
#include "usart.h"
#include "tftlcd.h"
#include "led.h"
#include "SysTick.h"
#include "lv_port.h"

#define APP_VERSION "V0.2"

/* ---------------- step 3: external flash self test ---------------- */
static void flash_selftest(void)
{
    uint8_t  wbuf[FLASH_PAGE_SIZE];
    uint8_t  rbuf[FLASH_PAGE_SIZE];
    uint16_t i;
    uint32_t id;
    int      r;

    r = drv_flash_init();
    id = drv_flash_read_id();
    printf("[flash] id = 0x%06X (init=%d)\r\n", id, r);
    if (r != 0) return;

    r = drv_flash_erase_sector(0);
    printf("[flash] erase %s\r\n", (r == 0) ? "ok" : "FAIL");
    if (r != 0) return;

    for (i = 0; i < FLASH_PAGE_SIZE; i++) {
        wbuf[i] = (uint8_t)i;
    }

    r = drv_flash_write(0, wbuf, FLASH_PAGE_SIZE);
    printf("[flash] write %s\r\n", (r == 0) ? "ok" : "FAIL");
    if (r != 0) return;

    memset(rbuf, 0, sizeof(rbuf));
    drv_flash_read(0, rbuf, FLASH_PAGE_SIZE);

    for (i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (rbuf[i] != wbuf[i]) {
            printf("[flash] verify FAIL at 0x%03X (got %02X, want %02X)\r\n",
                   i, rbuf[i], wbuf[i]);
            return;
        }
    }
    printf("[flash] verify PASS\r\n");
}

void app_main(void)
{
    int      r485, rwifi;

    /* ---------------- init ---------------- */
    r485  = drv_uart_init(UART_RS485, PIN_UART2_BAUD);
    rwifi = drv_uart_init(UART_WIFI,  PIN_UART3_BAUD);

    /* ---------------- report ---------------- */
    printf("\r\n===== GW %s / step2 =====\r\n", APP_VERSION);
    printf("[uart] debug  ok   (usart1 115200)\r\n");
    printf("[uart] rs485  %s (usart2 9600, err=%d)\r\n", r485  ? "FAIL" : "ok", r485);
    printf("[uart] wifi   %s (usart3 115200, err=%d)\r\n", rwifi ? "FAIL" : "ok", rwifi);
    drv_uart_write(UART_WIFI, (const uint8_t *)"AT\r\n", 4u, 100u);

    flash_selftest();

    LCD_Clear(WHITE);
    FRONT_COLOR = BLACK;
    LCD_ShowString(10,  10, tftlcd_data.width, tftlcd_data.height, 24, "GW " APP_VERSION);
    LCD_ShowString(10,  50, tftlcd_data.width, tftlcd_data.height, 16, "USART1 115200 ok");
    LCD_ShowString(10,  75, tftlcd_data.width, tftlcd_data.height, 16,
                   (u8 *)(r485  ? "USART2 9600   FAIL" : "USART2 9600   ok"));
    LCD_ShowString(10, 100, tftlcd_data.width, tftlcd_data.height, 16,
                   (u8 *)(rwifi ? "USART3 115200 FAIL" : "USART3 115200 ok"));
    LCD_ShowString(10, 140, tftlcd_data.width, tftlcd_data.height, 16, "485: echo");
    LCD_ShowString(10, 165, tftlcd_data.width, tftlcd_data.height, 16, "wifi: dump to uart1");

    /* ---------------- LVGL 移植层自检（阶段 1） ----------------
     * 只做 lv_init + display/indev 注册，不建任何界面、不调 lv_timer_handler，
     * 所以屏幕不会被 LVGL 接管，现有自检画面保持不动。
     * 目的是把 LVGL 真正链接进来，验证代码能编、能初始化、并测出真实占用。
     * 等 ui_task 里正式启用 LVGL 后，这一句可以删掉。 */
    lv_port_init();
    printf("[lvgl] port init ok  v%d.%d.%d\r\n",
           LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);

    /* ---------------- hand over to RTOS ---------------- */
    App_freeRTOS_start();
}
