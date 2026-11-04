/**
 * @file    main.c
 * @brief   Bootloader 入口（8.1b：读标志 → 判断该不该跳 → 跳）
 *
 * 8.3 的职责边界：
 *   做的事：读双份标志、首次烧录时建默认标志、pending=1 时调
 *           fw_install 校验暂存区并搬进 APP 区、置 TRYING、
 *           待确认状态下计启动次数、次数超限就停住（不跳）
 *   不做的事：回滚（8.4：黄金备份搬运、boot_try 超限时救场）
 *
 * 后面几步会往这里加：
 *   8.4   boot_try 超限时从黄金备份搬回老版本
 */

#include "stm32f4xx_hal.h"
#include "system.h"
#include "boot_uart.h"
#include "ota_flag.h"
#include "app_jump.h"
#include "drv_spiflash.h"
#include "fw_image.h"
#include "fw_install.h"

/* 测试开关：置 1 时故意不跳转，用来验证 boot_try 的累加和超限停止。
 * 因为平时 swap_state 不是 TRYING，正常使用根本不会走到计数那一段，
 * 不打开这个开关就没办法验证。验完必须改回 0。 */
#define BOOT_TEST_NO_JUMP  0

/* 启动文件里的 SysTick_Handler 是空死循环（B .），而 HAL_Init 一上来就开
 * SysTick 中断。不自接管的话，1ms 后 boot 就卡死在默认中断里，串口一片安静。
 * 这个现象很容易被当成"晶振没起振"。 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* 停住：不点灯、不初始化别的外设，免得和 APP 抢资源。
 * 到这里说明没人能启动了，等人重烧或者断电。 */
static void boot_halt(void)
{
    while (1) {
    }
}

/* 试跳一次。跳成功不会回来；跳失败打印原因后停住。 */
static void boot_jump_or_halt(void)
{
    int r = app_jump_try();
    boot_printf("[boot] app not started (err=%d), halt\r\n", r);
    boot_halt();
}

/* 8.4：KEY0（PE4，低电平有效）按住 2 秒 = 应急重置标志。
 * 用途：OTA 状态机卡死（比如永远 TRYING）时现场自救。
 * 特意选 KEY0 —— 它在 APP 里没有任何功能，不存在误触场景。
 * 重置只清安装状态，golden 字段保留：救命的备份不能跟着陪葬。
 * 没按（默认上拉高电平）时一次读引脚就返回，零开销。 */
static void key0_check(ota_flag_t *flag)
{
    GPIO_InitTypeDef gpio;
    uint32_t keep_len;
    uint32_t keep_crc;

    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin   = GPIO_PIN_4;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) != GPIO_PIN_RESET) {
        return;     /* 没按，正常走 */
    }

    boot_printf("[boot] KEY0 held, reset flag in 2s ...\r\n");
    HAL_Delay(2000);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) != GPIO_PIN_RESET) {
        boot_printf("[boot] KEY0 released, continue\r\n");
        return;     /* 中途松手，当没发生 */
    }

    keep_len = flag->golden_len;
    keep_crc = flag->golden_crc;
    ota_flag_default(flag);
    flag->golden_len = keep_len;
    flag->golden_crc = keep_crc;
    if (ota_flag_save(flag) == 0) {
        boot_printf("[boot] flag reset to default (golden kept)\r\n");
    } else {
        boot_printf("[boot] flag reset FAILED\r\n");
    }
}

int main(void)
{
    ota_flag_t flag;
    fw_header_t hdr;
    int r;
    int rs;

    HAL_Init();
    SystemClock_Init(8, 336, 2, 7);     /* HSE 8MHz -> 168MHz，和 APP 保持一致 */

    boot_uart_init();
    boot_printf("\r\n[boot] v1 start, app area 0x%08X - 0x%08X\r\n",
                (unsigned)APP_BASE, (unsigned)APP_END);

    /* ---------------- 外部 SPI Flash ----------------
     * OTA 暂存区和黄金备份都在这一颗上，boot 必须自己能读它。
     * 读 JEDEC ID 是最省事的通路检查：能读回 0xEF4018 说明
     * SPI1 四根线 + CS 全是通的，后面读固件头才有意义。 */
    rs = drv_flash_init();
    if (rs != 0) {
        boot_printf("[boot] spiflash init FAIL (err=%d)\r\n", rs);
    } else {
        boot_printf("[boot] spiflash id=0x%06X\r\n",
                    (unsigned)drv_flash_read_id());
    }

    /* 读暂存区固件头。现在还没人往里写，读到 0xFF 属于正常；
     * 这一行主要在验证「boot 能把外部 Flash 里的字节读出来」。 */
    rs = fw_head_read(&hdr);
    if (rs == FW_OK) {
        boot_printf("[boot] staging: ver=%u.%u len=%u crc=0x%08X\r\n",
                    (unsigned)(hdr.version >> 16),
                    (unsigned)(hdr.version & 0xFFFFu),
                    (unsigned)hdr.fw_len,
                    (unsigned)hdr.fw_crc);
    } else {
        boot_printf("[boot] staging: empty (magic=0x%08X)\r\n",
                    (unsigned)hdr.magic);
    }

    r = ota_flag_load(&flag);

    /* ================= 情况一：两份标志都无效 ================= */
    if (r != 0) {
        /* 先看 APP 区首字。全 0xFF 说明这芯片压根没烧过程序，
         * 建标志没意义，直接停住等人烧。 */
        if (*(volatile uint32_t *)APP_BASE == 0xFFFFFFFFu) {
            boot_printf("[boot] no app (app area blank), waiting for download\r\n");
            boot_halt();
        }

        /* APP 区有内容：是手动烧进去的，信任它，建一份默认标志。
         * 建完 swap_state = IDLE，以后启动都不会再写 Flash。 */
        ota_flag_default(&flag);
        if (ota_flag_save(&flag) == 0) {
            boot_printf("[boot] flag: empty -> created default (seq=%u)\r\n",
                        (unsigned)flag.seq);
        } else {
            boot_printf("[boot] flag: create FAILED, continue anyway\r\n");
        }

        boot_jump_or_halt();
    }

    /* ================= 情况二：读到有效标志 ================= */
    boot_printf("[boot] flag: seq=%u swap=%u try=%u pending=%u\r\n",
                (unsigned)flag.seq, (unsigned)flag.swap_state,
                (unsigned)flag.boot_try, (unsigned)flag.pending);

    key0_check(&flag);

#if BOOT_TEST_NO_JUMP
    /* 测试模式第一半：强制进入"待确认"状态。
     * 正常时 swap_state 是 IDLE，下面的计数逻辑根本不会跑，没法验证。 */
    if (flag.swap_state != OTA_SWAP_TRYING) {
        ota_flag_t t = flag;
        t.swap_state = OTA_SWAP_TRYING;
        t.boot_try   = 0u;
        t.seq        = flag.seq + 1u;
        (void)ota_flag_save(&t);
        flag = t;
        boot_printf("[boot] TEST: forced swap=TRYING, try reset\r\n");
    }
#endif

    if (flag.pending != 0u) {
        /* ===== 8.3：暂存区有新固件，装它 ===== */
        /* 三重比对：头读得出来、长度和 CRC 与标志里的记录一致。
         * 对不上说明暂存区被写坏或标志过期 —— 不碰 APP 区，
         * 跳老固件保平安。 */
        if (rs != FW_OK || hdr.fw_len != flag.fw_len || hdr.fw_crc != flag.fw_crc) {
            boot_printf("[boot] pending but staging mismatch, skip install\r\n");
        } else {
            boot_printf("[boot] install: %u bytes\r\n", (unsigned)hdr.fw_len);

            if (fw_install_run(&hdr) != FW_INS_OK) {
                /* 能走到这里说明 APP 区已经动过、老固件没了，
                 * 跳出去也是死 —— 停机等人重烧。详细错误码已打印。 */
                boot_printf("[boot] install FAIL, halt\r\n");
                boot_halt();
            }

            /* 装好了：清 pending、进待确认态、次数清零。
             * 下面的 TRYING 分支接着记 try=1 并跳新固件；
             * 新固件正常起来后 app_ota_confirm() 把状态推到 DONE。 */
            flag.pending    = 0u;
            flag.swap_state = OTA_SWAP_TRYING;
            flag.boot_try   = 0u;
            flag.seq        = flag.seq + 1u;
            if (ota_flag_save(&flag) != 0) {
                boot_printf("[boot] warn: save flag failed\r\n");
            } else {
                boot_printf("[boot] install ok, swap=TRYING, wait confirm\r\n");
            }
        }
    }

    /* 只有「刚装完、还没确认」这个状态才计数。
     * 平时 swap_state 是 IDLE 或 DONE，一个字节都不写 Flash。 */
    if (flag.swap_state == OTA_SWAP_TRYING) {
        if (flag.boot_try >= OTA_BOOT_TRY_MAX) {
            /* 8.4：起不来就回滚。没有黄金备份才停住等人。
             * 黄金备份是 APP 上次确认成功时写进外部 Flash 的，
             * 内容一定是「最后被确认能跑」的那一版。 */
            if (flag.golden_len == 0u ||
                (APP_BASE + flag.golden_len) > APP_END) {
                boot_printf("[boot] failed %u times, no golden, halt\r\n",
                            (unsigned)flag.boot_try);
                boot_halt();
            }

            boot_printf("[boot] failed %u times, rollback from golden\r\n",
                        (unsigned)flag.boot_try);

            if (fw_restore_golden(flag.golden_len, flag.golden_crc) != FW_INS_OK) {
                /* 黄金区也坏了（CRC 对不上 / 搬运失败），彻底没救 */
                boot_printf("[boot] rollback FAIL, halt\r\n");
                boot_halt();
            }

            /* 回滚成功：回到干净状态继续跑。golden 字段保留 ——
             * 备份本身还在外部 Flash 里躺着，依然有效。 */
            flag.pending    = 0u;
            flag.fw_len     = 0u;
            flag.fw_crc     = 0u;
            flag.boot_try   = 0u;
            flag.swap_state = OTA_SWAP_IDLE;
            flag.seq        = flag.seq + 1u;
            if (ota_flag_save(&flag) != 0) {
                boot_printf("[boot] warn: save flag after rollback failed\r\n");
            } else {
                boot_printf("[boot] rollback ok, boot golden fw\r\n");
            }
        } else {
            /* 递增由 ota_flag_bump_try 内部完成，这里不要再加一次。
             * 两处都写 ++ 的话每次复位实际加 2，只试 2 次就判死。 */
            if (ota_flag_bump_try(&flag) != 0) {
                boot_printf("[boot] warn: save try=%u failed\r\n", (unsigned)flag.boot_try);
            } else {
                boot_printf("[boot] try %u/%u\r\n",
                            (unsigned)flag.boot_try, (unsigned)OTA_BOOT_TRY_MAX);
            }
        }
    }

#if BOOT_TEST_NO_JUMP
    /* 测试模式：假装跳转失败，让 boot_try 能一直累加 */
    boot_printf("[boot] TEST: skip jump on purpose\r\n");
    boot_halt();
#endif

    boot_jump_or_halt();

    return 0;   /* 不会执行到，只为消除 "missing return" 警告 */
}
