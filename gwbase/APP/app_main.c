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
#include "app_ota_flag.h"
#include "app_wifi_cfg.h"
#include "net_if.h"
#include "app_config.h"

/* 外 Flash 自检第一步就是擦 0x000000 —— 那正是 OTA 固件暂存槽。
 * 只要 APP 跑一次自检，暂存的新固件就没了，OTA 会随机失败且极难查。
 * 所以默认关闭；单独测 Flash 读写时改成 1。 */
#define FLASH_SELFTEST  0

/* ---------------- step 3: external flash self test ---------------- */
#if FLASH_SELFTEST
static void flash_selftest(void)
{
    uint8_t  wbuf[FLASH_PAGE_SIZE];
    uint8_t  rbuf[FLASH_PAGE_SIZE];
    uint16_t i;
    uint32_t id;
    int      r;

    /* 初始化已由 app_main 统一完成，这里只做读写校验 */
    id = drv_flash_read_id();
    printf("[flash] id = 0x%06X\r\n", id);

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
#endif /* FLASH_SELFTEST */

void app_main(void)
{
    int      r485, rwifi;
    int      rf;
    int      ota_r;

    /* ---------------- init ---------------- */
    r485  = drv_uart_init(UART_RS485, PIN_UART2_BAUD);
    rwifi = drv_uart_init(UART_WIFI,  PIN_UART3_BAUD);

    /* 外部 SPI Flash：日志存储、离线缓存、OTA 暂存全靠它，必须无条件初始化。
     * 不能塞进 flash_selftest() 里，那个自检默认关闭时会一起被编掉，
     * 表现就是 [store] write FAIL。 */
    rf = drv_flash_init();

    /* ---------------- report ---------------- */
    printf("\r\n===== GW %s / step2 =====\r\n", APP_VERSION);
    /* 向量表实际地址。跑在 0x08010000 时这里必须也是 0x08010000，
     * 是判断 bootloader 跳转和 VECT_TAB_OFFSET 有没有对上的直接证据。 */
    printf("[app] vtor 0x%08X\r\n", (unsigned)SCB->VTOR);
    printf("[uart] debug  ok   (usart1 115200)\r\n");
    printf("[uart] rs485  %s (usart2 9600, err=%d)\r\n", r485  ? "FAIL" : "ok", r485);
    printf("[uart] wifi   %s (usart3 115200, err=%d)\r\n", rwifi ? "FAIL" : "ok", rwifi);
    printf("[flash] init  %s  id=0x%06X\r\n",
           (rf == 0) ? "ok" : "FAIL", (unsigned)drv_flash_read_id());

    /* WiFi 凭据：读外部 Flash 的配置区（用户屏上配过就用他的，没配过用宏）。
     * 必须在起任务前做 —— 此时只有主线程在跑，不用抢 SPI1 的锁。
     * 读出来的值同时喂给 net_if，net_task 后面直接连，不用再管凭据从哪来。
     * 放在 report 之后是为了让开机的 `===== GW ... =====` 横幅仍是第一行，
     * 否则 [wifi] 那行会跑到上次开机的尾巴里去，看着像串了台。 */
    wifi_cfg_init();
    net_if_set_wifi(wifi_cfg_ssid(), wifi_cfg_pass());

    drv_uart_write(UART_WIFI, (const uint8_t *)"AT\r\n", 4u, 100u);

#if FLASH_SELFTEST
    flash_selftest();
#endif

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

    /* ---------------- OTA 启动确认 ----------------
     * 跑到这里说明时钟、三个串口、LCD、LVGL、内存池全部初始化成功，
     * 可以认为这份固件是能用的。如果是"刚装完、待确认"的新固件，
     * 这里把它标成 DONE，boot 那边就不会再数启动次数了。
     *
     * 放在调度器启动前：此时各任务还没跑，UART 没人读，
     * 擦 Flash 那 200~500ms 里丢掉的数据无所谓。
     * 平时 swap_state 不是 TRYING，这个函数直接返回 0，不碰 Flash。 */
    ota_r = app_ota_confirm();
    if (ota_r > 0) {
        printf("[ota] new firmware confirmed, boot_try cleared\r\n");
    } else if (ota_r < 0) {
        printf("[ota] confirm FAILED (flag write error)\r\n");
    }

    /* ---------------- hand over to RTOS ---------------- */
    App_freeRTOS_start();
}
