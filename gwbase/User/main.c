/**
 * @file    main.c
 * @brief   GW gateway - Step 1 bring-up
 *
 * Boot order matters:
 *   HAL_Init -> clock -> SysTick -> UART -> GPIO -> LCD -> touch
 * Touch init may block waiting for 4-point calibration on first run.
 */

#include "system.h"
#include "SysTick.h"
#include "usart.h"
#include "led.h"
#include "tftlcd.h"
#include "key.h"
#include "touch.h"
#include "app_main.h"

int main(void)
{
    HAL_Init();                     /* HAL library init                    */
    SystemClock_Init(8, 336, 2, 7); /* HSE 8MHz -> 168MHz                  */
    SysTick_Init(168);              /* 1ms tick for delay_ms()             */
    USART1_Init(115200);            /* debug console (CH340 -> Type-C)     */
    LED_Init();
    TFTLCD_Init();                  /* FSMC LCD, 3.5" ILI9488              */
    KEY_Init();
    TP_Init();                      /* XPT2046, calibrates on first run    */
    LCD_Display_Dir(0);             /* 0 = portrait 320x480                */

    app_main();                     /* our application entry               */

    while (1)
    {
        LED1 = !LED1;
        delay_ms(500);
    }
}
