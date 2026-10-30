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

    /* 复位原因诊断：区分上电 / 外部拉低 NRST / 掉电 / 软件 / 看门狗。
     * 上电瞬间 NRST 也被内部拉低，所以 POR 和 PIN 会同时置位；
     * "POR=0 且 PIN=1" 才说明这次是有人从外面把复位脚拉下去了。 */
    {
        uint32_t csr = RCC->CSR;

        printf("\r\n[reset] POR=%u PIN=%u BOR=%u SW=%u IWDG=%u WWDG=%u LPWR=%u\r\n",
               (unsigned)((csr & RCC_CSR_PORRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_PINRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_BORRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_SFTRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_IWDGRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_WWDGRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_LPWRRSTF) ? 1u : 0u));

        __HAL_RCC_CLEAR_RESET_FLAGS();   /* 清标志，下次只反映新的复位 */
    }

    LED_Init();
    TFTLCD_Init();                  /* FSMC LCD, 3.5" ILI9488              */
    KEY_Init();
    TP_Init();                      /* XPT2046, calibrates on first run    */
    LCD_Display_Dir(0);             /* 0 = portrait 320x480                */

    app_main();                     /* never returns: starts the scheduler */
}
