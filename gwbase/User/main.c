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
        /* 值取自 BKP3R 而不是 RCC->CSR：boot 在 HAL_RCC_DeInit 之前把 CSR 快照
         * 存进了备份寄存器，直接读 CSR 只会得到一排 0（连断电后的 POR 都是 0）。
         * 全 0 说明 boot 还是没存快照的旧版本，需要用 ST-Link 重新烧 boot。 */
        uint32_t csr = RTC->BKP3R;

        printf("\r\n[reset] POR=%u PIN=%u BOR=%u SW=%u IWDG=%u WWDG=%u LPWR=%u\r\n",
               (unsigned)((csr & RCC_CSR_PORRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_PINRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_BORRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_SFTRSTF)  ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_IWDGRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_WWDGRSTF) ? 1u : 0u),
               (unsigned)((csr & RCC_CSR_LPWRRSTF) ? 1u : 0u));

        __HAL_RCC_CLEAR_RESET_FLAGS();   /* 清标志，下次只反映新的复位 */
        __HAL_RCC_PWR_CLK_ENABLE();      /* 写备份寄存器要解锁备份域 */
        HAL_PWR_EnableBkUpAccess();
        RTC->BKP3R = 0u;                 /* 快照用完即清，下次开机由 boot 重新存 */
    }

    LED_Init();
    TFTLCD_Init();                  /* FSMC LCD, 3.5" ILI9488              */
    KEY_Init();
    LCD_Display_Dir(1);             /* 1 = landscape 480x320               */
    TP_Init();                      /* XPT2046；置于设方向之后，校准才在横屏坐标系下进行 */

    app_main();                     /* never returns: starts the scheduler */
}
