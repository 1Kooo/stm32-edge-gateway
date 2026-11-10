/**
 * @file    app_jump.c
 * @brief   跳转到 APP 区。
 *
 * 顺序是有讲究的，别随手调换：
 *   1. 先打印再关外设 —— HAL_RCC_DeInit 之后串口时钟就没了，之后再 printf 是白打
 *   2. 必须 HAL_RCC_DeInit —— 把 PLL 关掉、系统时钟退回 HSI。
 *      否则 APP 里 SystemClock_Init() 会返回 HAL_ERROR，而它后面是 while(1)，
 *      现象就是"跳过去了但黑屏死机"，非常难查。
 *   3. 关总中断、停 SysTick、清 NVIC 挂起 —— 免得 boot 期间挂起的中断跑进 APP 向量表
 *   4. 最后才设 MSP 和 VTOR，然后跳
 */

#include "app_jump.h"
#include "boot_uart.h"
#include "stm32f4xx_hal.h"

/* 复位原因快照：把 RCC->CSR 存进备份寄存器 BKP3R，APP 起来自己打印。
 *
 * 为什么非得由 boot 来存：下面 HAL_RCC_DeInit() 会把 CSR 里的复位标志清掉，
 * APP 再去读就永远是一排 0 —— 之前"看门狗复位却显示 IWDG=0"就是这么来的，
 * 连真断电后的 POR 都读成 0。
 * 备份域由 VBAT/待机电路供电，跨复位和跨断电都保留，正好当这个信箱。 */
static void save_reset_cause(void)
{
    /* 写备份寄存器要先开 PWR 时钟并解锁备份域 */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    /* 只存复位相关的高 8 位（bit24~31：RMVF + 各路复位标志），低位是 LSI 状态 */
    RTC->BKP3R = RCC->CSR & 0xFF000000u;
}

int app_jump_try(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_BASE;             /* 向量表第 0 项：栈顶 */
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4u);      /* 向量表第 1 项：复位入口 */
    uint32_t pc_aligned = pc & ~1u;                           /* 去掉 Thumb 标志位 */
    uint32_t i;

    /* 栈顶必须落在 SRAM（0x20000000 ~ 0x2001FFFF）。全 0xFF 或全 0 都过不了这关。 */
    if ((sp & 0xFFF00000u) != 0x20000000u) {
        boot_printf("[boot] bad stack 0x%08X, app looks blank\r\n", (unsigned)sp);
        return -1;
    }

    /* 复位入口必须落在 APP 区里 */
    if ((pc_aligned < APP_BASE) || (pc_aligned >= APP_END)) {
        boot_printf("[boot] entry 0x%08X outside app area\r\n", (unsigned)pc_aligned);
        return -2;
    }

    boot_printf("[boot] jump 0x%08X  sp=0x%08X pc=0x%08X\r\n",
                (unsigned)APP_BASE, (unsigned)sp, (unsigned)pc_aligned);

    /* 必须在 HAL_RCC_DeInit 之前：那一步会把 CSR 的复位标志清掉 */
    save_reset_cause();

    HAL_DeInit();
    HAL_RCC_DeInit();

    __disable_irq();
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL  = 0u;

    for (i = 0; i < 8u; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;    /* 关掉所有外设中断 */
        NVIC->ICPR[i] = 0xFFFFFFFFu;    /* 清掉所有挂起位 */
    }

    SCB->VTOR = APP_BASE;
    __set_MSP(sp);
    __DSB();
    __ISB();

    ((void (*)(void))pc)();

    return 0;   /* 正常跳转不会执行到这里 */
}
