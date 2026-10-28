/**
 * @file    app_main.c
 * @brief   Application entry (Step 1: bring-up)
 *
 * Step 1 goal: prove the whole chain works with our own code.
 *   - printf over USART1  -> PC serial console
 *   - LCD_ShowString      -> on-board TFT (320x480 portrait)
 *
 * Nothing else yet. Sensors / RTOS / LVGL come later, one step at a time.
 */

#include "app_main.h"
#include "usart.h"
#include "tftlcd.h"

#define APP_VERSION "V0.1"

void app_main(void)
{
    /* --- screen --- */
    LCD_Clear(WHITE);

    FRONT_COLOR = BLACK;
    LCD_ShowString(10, 10,  tftlcd_data.width, tftlcd_data.height, 24, "GW " APP_VERSION);
    LCD_ShowString(10, 60,  tftlcd_data.width, tftlcd_data.height, 16, "T: --.- C");
    LCD_ShowString(10, 85,  tftlcd_data.width, tftlcd_data.height, 16, "H: --.- %");
    LCD_ShowString(10, 110, tftlcd_data.width, tftlcd_data.height, 16, "L: --- lux");
    LCD_ShowString(10, tftlcd_data.height - 20,
                   tftlcd_data.width, tftlcd_data.height, 16, "step1 running");

    /* --- serial console --- */
    printf("\r\n===== GW %s =====\r\n", APP_VERSION);
    printf("LCD : %d x %d\r\n", tftlcd_data.width, tftlcd_data.height);
    printf("step1 ok, entering loop\r\n");
}
