/**
 * @file    app_main.c
 * @brief   Application entry: init everything, then hand over to FreeRTOS.
 *
 * 顺序（有依赖关系，不能随意换）：
 *   1. 两个业务串口（RS485 / WiFi）+ 外部 Flash + RTC
 *   2. 自检横幅（串口 + 裸 LCD 自检页）
 *   3. WiFi 凭据（外部 Flash 配置区，喂给 net_if）
 *   4. LVGL 移植层（必须在起任务前，见下方注释）
 *   5. OTA 启动确认（新固件首跑，写 DONE 清 boot_try）
 *   6. App_freeRTOS_start() 起 8 个任务（led/beat/wdg/uart/sample/
 *      storage/ui/net），不再回来
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
#include "drv_rtc.h"
#include "app_event.h"      /* app_event_init / EVT_POWER_ON */
#include "app_wdg.h"        /* app_wdg_start_early / app_wdg_feed */

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
    /* 参数区加载 + 周期/阈值就位（实现在 app_freertos.c） */
    extern void app_cfg_init(void);
    int      r485, rwifi;
    int      rf;
    int      ota_r;

    /* ---------------- 看门狗最早期启动 ----------------
     * 必须排在所有可能长时间阻塞的初始化之前。原因：wdg_task 要等调度器
     * 启动才会跑，在那之前如果卡在 RTC 等晶振、事件区扫描、LCD/LVGL 初始化
     * 上，没有任何东西能把芯片拉回来 —— 现场表现就是"永远停在自检页"，
     * 只能人跑去断电。提前开了之后，这类卡死 30 秒后自动复位；如果这份固件
     * 是 OTA 装进来的，复位还会让 boot 的 try 计数累加，最终触发回滚。
     *
     * 代价要说清：起不来的固件会变成"每 30 秒重启一次"的循环，
     * 而不是安静地停住。这是有意的取舍 —— 循环能被看见，也可能自愈。 */
    app_wdg_start_early();

    /* ---------------- init ---------------- */
    r485  = drv_uart_init(UART_RS485, PIN_UART2_BAUD);
    rwifi = drv_uart_init(UART_WIFI,  PIN_UART3_BAUD);

    /* 外部 SPI Flash：日志存储、离线缓存、OTA 暂存全靠它，必须无条件初始化。
     * 不能塞进 flash_selftest() 里，那个自检默认关闭时会一起被编掉，
     * 表现就是 [store] write FAIL。 */
    rf = drv_flash_init();

    /* 参数区要在起任务前读：net_task 一启动就要用轮询/上报周期，
     * sample_task 一启动就要用告警阈值，晚一步第一轮数据就按默认值跑了。 */
    app_cfg_init();

    app_wdg_feed();     /* 上面这几步都会读写外部 Flash，耗时不固定 */

    /* ---------------- report ---------------- */
    /* 带上编译时间戳：以后上电第一行就能和手头 hex 的时间对一下，
     * 避免"烧的是旧固件却以为是新固件"导致的无效排查。
     * __DATE__ / __TIME__ 是编译器内置宏，每次编译自动更新，零维护。 */
    printf("\r\n===== GW %s build " __DATE__ " " __TIME__ " =====\r\n", APP_VERSION);
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

    /* RTC：给每条传感器记录盖真实时间戳。必须在起任务前初始化 ——
     * 首次开 LSE 晶振要等最多 3 秒，这个等待不能挂在任何任务里。
     * 电池在：一行 [rtc] time ok 后零开销；电池不在：退回编译时刻。 */
    drv_rtc_init();
    app_wdg_feed();     /* 首次开 LSE 最多要等 3 秒 */

    /* 事件日志必须在 RTC 之后初始化：这条"上电"事件要盖时间戳。
     * 早于 drv_rtc_init 记的话，drv_rtc_unix 读到的 RTC 是空的，ts 记 0，
     * 屏上显示成 --:--:--（旧版本补当前时间，会固定显示 08:00，实测踩过）。
     * 注意 RTC 在但没被 NTP 校准过时，app_event_log 也会把 ts 记 0 ——
     * 那时 RTC 里是编译时刻，不是真实的上电时间。 */
    app_event_init();
    app_event_log(EVT_POWER_ON, 0u);
    app_wdg_feed();     /* 事件区 64KB 要逐块读一遍 */

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
    LCD_ShowString(10, 140, tftlcd_data.width, tftlcd_data.height, 16, "485: modbus");
    LCD_ShowString(10, 165, tftlcd_data.width, tftlcd_data.height, 16, "wifi: mqtt");

    /* ---------------- LVGL 移植层初始化 ----------------
     * lv_init + 显示 / indev 注册全工程只有这一处（ui_task 只调
     * lv_timer_handler）。必须在起任务前完成：ui_task 一启动就要用
     * 这些对象，注册晚了第一帧就会踩空。 */
    lv_port_init();
    printf("[lvgl] port init ok  v%d.%d.%d\r\n",
           LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    app_wdg_feed();     /* LVGL 初始化要建一堆对象，堆紧张时会慢 */

    /* ---------------- OTA 启动确认 ----------------
     * 跑到这里说明时钟、三个串口、LCD、LVGL、内存池全部初始化成功，
     * 可以认为这份固件至少"能开机"。如果是刚装完、待确认的新固件，
     * 这里把它标成 DONE，boot 那边就不会再数启动次数了。
     *
     * 只写标志，不再顺手搬黄金备份 —— 能开机不等于够可靠，自动搬会把
     * 上一份真正好用的备份顶掉。固化改由人手动触发（云端 {"golden":1}）。
     * 平时 swap_state 不是 TRYING，这个函数直接返回 0，不碰 Flash。 */
    ota_r = app_ota_confirm();
    if (ota_r > 0) {
        printf("[ota] new firmware confirmed, boot_try cleared\r\n");
    } else if (ota_r < 0) {
        printf("[ota] confirm FAILED (flag write error)\r\n");
    }
    app_wdg_feed();

    /* ---------------- hand over to RTOS ---------------- */
    App_freeRTOS_start();
}
