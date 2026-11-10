/**
 * @file    app_freertos.c
 * @brief   FreeRTOS glue: assertion handler, kernel hooks, task set.
 *
 * Note: everything above App_freeRTOS_start() must be safe to call before
 * the scheduler runs (printf works because fputc is polled, not queued).
 */

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usart.h"
#include "led.h"
#include "drv_uart.h"
#include "modbus_rtu.h"     /* Modbus RTU 主站：485 温湿度变送器 */
#include "drv_rtc.h"        /* RTC：记录时间戳 */
#include "drv_spiflash.h"
#include "tftlcd.h"
#include "bsp_pinmap.h"
#include "data_model.h"
#include "net_if.h"
#include "mqtt_client.h"
#include "app_ota.h"
#include "app_config.h"
#include "app_ui.h"         /* ui_task 入口 + ui_snapshot_t + 显示数据 getter */
#include "app_wifi_cfg.h"   /* 屏上配的 WiFi 凭据：待写缓冲 + 落盘 */

/* ================= 断言与内核钩子 ================= */
void app_assert_failed(const char *file, int line)
{
    __disable_irq();
    printf("\r\n[assert] %s:%d\r\n", file, line);
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    __disable_irq();
    printf("\r\n[stack overflow] task=%s\r\n", pcTaskName);
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    printf("\r\n[malloc failed] heap exhausted\r\n");
    for (;;) { }
}

/* SysTick 归 FreeRTOS 管之后 HAL_IncTick() 不再自动调用，
 * 而所有 HAL 超时（HAL_UART_Transmit、flash_wait_busy）都靠它，所以每 tick 补一次。 */
void vApplicationTickHook(void)
{
    HAL_IncTick();
}

/* ================= 日志（互斥，避免多任务输出交错） ================= */
static SemaphoreHandle_t s_log_mutex = NULL;

/* 外部 Flash 互斥：storage_task 写记录，net_task 读记录 + 改 state 字节，
 * 两者都走 SPI1。少了这把锁就是参考项目 A 那个"锁被注释掉"的同款隐患。 */
static SemaphoreHandle_t s_flash_mutex = NULL;

void app_log(const char *fmt, ...)
{
    va_list ap;

    if (s_log_mutex == NULL) {          /* 调度器启动前，无竞争 */
        va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
        return;
    }

    if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == pdTRUE) {
        va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
        xSemaphoreGive(s_log_mutex);
    }
}

/* ================= 辅助 ================= */
static uint8_t s_buf[64];

/* 成功读数累计次数：每次 modbus 读到有效数据 +1，beat 那行打印出来，
 * 用来判断 485 是在正常应答还是整条线掉着（2 秒一轮，5 秒心跳该涨 2~3）。 */
static volatile uint32_t s_mb_ok = 0;

/* Modbus 最新读数：uart_poll_task 写，sample_task 取走。
 * pending=1 表示有一轮结果没取走，取完清零，避免同一次读数被重复记录。
 * 失败轮次也置 pending（valid=0），否则掉线时屏幕会一直停在最后一个旧值。 */
static volatile int16_t  s_live_temp_x10 = -999;
static volatile uint16_t s_live_humi_x10 = 0u;
static volatile uint8_t  s_live_valid    = 0u;   /* 0 = 这一轮失败或被闸门拦下 */
static volatile uint8_t  s_live_pending  = 0u;

/* MQTT 状态：给 ui_task 显示 */
static volatile uint8_t  s_mqtt_up = 0;
static volatile uint32_t s_pub_cnt = 0;

/* 采样 -> 存储 的数据管道 */
#define DATA_QUEUE_LEN  16
static QueueHandle_t s_data_queue = NULL;

/* 给 ui_task 显示用的快照：sample_task 在临界区里更新，
 * app_ui.c 通过 app_data_get_snapshot() 取，不直接摸这个变量。
 * 类型定义挪到了 app_ui.h（拆文件时随 UI 一起搬走）。 */
static ui_snapshot_t s_latest;

static void ascii_to_buf(char *dst, uint16_t dst_size, const uint8_t *p, uint16_t n)
{
    static const char hex[] = "0123456789ABCDEF";
    uint16_t i, k = 0;

    for (i = 0; i < n && (k + 6u) < dst_size; i++) {
        if (p[i] >= 0x20u && p[i] < 0x7Fu) {
            dst[k++] = (char)p[i];
        } else {
            dst[k++] = '<';
            dst[k++] = hex[p[i] >> 4];
            dst[k++] = hex[p[i] & 0x0Fu];
            dst[k++] = '>';
        }
    }
    dst[k] = '\0';
}

/* ================= LED 任务 ================= */
void led_task(void *args);
#define LED_TASK_STACK_SIZE 256    /* 字（word），256*4 = 1KB */
#define LED_TASK_PRIORITY   1
#define LED_TASK_PERIOD     500    /* ms */
TaskHandle_t led_task_handle;

/* ================= 心跳任务 ================= */
void beat_task(void *args);
#define BEAT_TASK_STACK_SIZE 256
#define BEAT_TASK_PRIORITY   2
#define BEAT_TASK_PERIOD     5000
TaskHandle_t beat_task_handle;

/* ================= 串口轮询任务 ================= */
void uart_poll_task(void *args);
#define UART_TASK_STACK_SIZE 512
#define UART_TASK_PRIORITY   2
#define MB_POLL_PERIOD_MS    2000u      /* 固定 2 秒一轮：从站最快 1Hz 出数，够用又不占总线 */
TaskHandle_t uart_task_handle;

/* 读数合理性闸门（HZ8706 标称 -40~80℃ / 0~100%RH，寄存器单位 0.1）：
 * 线松或从站刚上电时会回一帧 CRC 正确的全 0，只验 CRC 会把 0.0℃/0.0%
 * 当成真实读数显示出来，所以再加一道数值范围判断。 */
#define MB_TEMP_MIN_X10   (-400)
#define MB_TEMP_MAX_X10   (800)
#define MB_HUMI_MAX_X10   (1000)

/* ================= 采样任务 ================= */
void sample_task(void *args);
#define SAMPLE_TASK_STACK_SIZE 512
#define SAMPLE_TASK_PRIORITY   1
#define SAMPLE_TASK_PERIOD     2000
TaskHandle_t sample_task_handle;

/* ================= 存储任务 ================= */
// 唯一允许写外部 Flash 的任务：谁要存数据都往队列里投，不自己动 Flash
void storage_task(void *args);
#define STORAGE_TASK_STACK_SIZE 512
#define STORAGE_TASK_PRIORITY   2
TaskHandle_t storage_task_handle;

/* ================= UI 任务 ================= */
// 唯一碰 LCD 的任务，其他任务只写内存，避免总线竞争
// 任务实现在 app_ui.c，入口声明在 app_ui.h（拆文件时整体搬走）
#define UI_TASK_STACK_SIZE   2048
#define UI_TASK_PRIORITY     0
#define UI_TASK_PERIOD       500
TaskHandle_t ui_task_handle;

/* ================= 网络任务 ================= */
// 唯一拥有 USART3 的任务：AT 指令交互全部在这里，其他任务不许读 WiFi 串口
void net_task(void *args);
#define NET_TASK_STACK_SIZE  768
#define NET_TASK_PRIORITY    2
TaskHandle_t net_task_handle;

/* ================= 创建 + 启动 ================= */
void App_freeRTOS_start(void)
{
    s_log_mutex   = xSemaphoreCreateMutex();
    s_flash_mutex = xSemaphoreCreateMutex();
    s_data_queue  = xQueueCreate(DATA_QUEUE_LEN, sizeof(sensor_record_t));

    printf("[rtos] heap total = %u bytes\r\n", (unsigned)configTOTAL_HEAP_SIZE);

    if (s_data_queue == NULL) {
        printf("[rtos] queue create FAIL\r\n");
        for (;;) { }
    }

    xTaskCreate(led_task,       "led_task",  LED_TASK_STACK_SIZE,
                NULL, LED_TASK_PRIORITY,  &led_task_handle);

    xTaskCreate(beat_task,      "beat_task", BEAT_TASK_STACK_SIZE,
                NULL, BEAT_TASK_PRIORITY, &beat_task_handle);

    xTaskCreate(uart_poll_task, "uart_task", UART_TASK_STACK_SIZE,
                NULL, UART_TASK_PRIORITY, &uart_task_handle);

    xTaskCreate(sample_task,    "sample",    SAMPLE_TASK_STACK_SIZE,
                NULL, SAMPLE_TASK_PRIORITY, &sample_task_handle);

    xTaskCreate(storage_task,   "storage",   STORAGE_TASK_STACK_SIZE,
                NULL, STORAGE_TASK_PRIORITY, &storage_task_handle);

    xTaskCreate(ui_task,        "ui",        UI_TASK_STACK_SIZE,
                NULL, UI_TASK_PRIORITY, &ui_task_handle);

    xTaskCreate(net_task,       "net",       NET_TASK_STACK_SIZE,
                NULL, NET_TASK_PRIORITY, &net_task_handle);

    printf("[rtos] 7 tasks created, starting scheduler\r\n");

    vTaskStartScheduler();          /* 不返回 */

    printf("[rtos] scheduler returned - heap exhausted\r\n");
    for (;;) { }
}

/* ================= 任务实现 ================= */
void led_task(void *args)
{
    (void)args;
    while (1)
    {
        LED1 = !LED1;
        vTaskDelay(pdMS_TO_TICKS(LED_TASK_PERIOD));
    }
}

void beat_task(void *args)
{
    uint32_t sec = 0;
    (void)args;

    while (1)
    {
        sec++;
        app_log("[beat] t=%lus heap=%u stack=%u mb=%lu\r\n",
                (unsigned long)sec,
                (unsigned)xPortGetFreeHeapSize(),
                (unsigned)uxTaskGetStackHighWaterMark(NULL),
                (unsigned long)s_mb_ok);
        vTaskDelay(pdMS_TO_TICKS(BEAT_TASK_PERIOD));
    }
}

/* 485 走阻塞轮询：modbus_read_regs 自己等帧 + 校验，不需要中断唤醒。
 * 原来挂 rx_callback 去 vTaskNotifyGiveFromISR，任务会被自己刚收到的响应
 * 立刻叫醒（pdTRUE 只清计数，不判断是谁发的），2 秒节流形同虚设。 */
void uart_poll_task(void *args)
{
    uint16_t regs[MODBUS_REG_COUNT];
    int16_t  t_x10;
    int16_t  t_abs;
    uint16_t h_x10;
    int      r;
    uint8_t  try;
    (void)args;

    while (1)
    {
        /* 从站偶尔丢一帧很正常（总线干扰 / 上电还没就绪），
         * 连续 3 次都失败才认输，避免云端看到一串断点。 */
        r = MODBUS_ERR_TIMEOUT;
        for (try = 0u; try < 3u; try++) {
            r = modbus_read_regs(MODBUS_SLAVE_ADDR, MODBUS_REG_START,
                                 MODBUS_REG_COUNT, regs);
            if (r == MODBUS_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));      /* 给从站一点恢复时间再重试 */
        }

        if (r == MODBUS_OK) {
            /* 温度是有符号量：-5.3℃ 时寄存器是 0xFFCB(=-53)，
             * 按 uint16_t 会读成 65483，所以这里必须强转 int16_t */
            t_x10 = (int16_t)regs[0];
            h_x10 = regs[1];

            /* 全 0 帧 / 超量程都判无效：这类帧 CRC 是对的，但值不是测量结果。
             * 两路同时为 0 才算全 0 帧，单独的 0.0℃ 或 0%RH 是合法读数。 */
            if ((regs[0] == 0u && regs[1] == 0u) ||
                t_x10 < MB_TEMP_MIN_X10 || t_x10 > MB_TEMP_MAX_X10 ||
                h_x10 > MB_HUMI_MAX_X10) {
                app_log("[mb] read INVALID raw=%04X %04X (try=%u)\r\n",
                        (unsigned)regs[0], (unsigned)regs[1],
                        (unsigned)(try + 1u));

                /* 交给 sample_task 处理：这一轮无效，让它把屏幕刷成 "--.-" */
                taskENTER_CRITICAL();
                s_live_valid   = 0u;
                s_live_pending = 1u;
                taskEXIT_CRITICAL();
            } else {
                t_abs = (t_x10 < 0) ? (int16_t)(-t_x10) : t_x10;

                app_log("[mb] T=%s%d.%dC H=%u.%u%% (try=%u)\r\n",
                        (t_x10 < 0) ? "-" : "",
                        (int)(t_abs / 10), (int)(t_abs % 10),
                        (unsigned)(h_x10 / 10u), (unsigned)(h_x10 % 10u),
                        (unsigned)(try + 1u));

                s_mb_ok++;

                /* 三值 + pending 一起在临界区里写，pending 放最后，
                 * 保证 sample_task 不会读到只更新了一半的组合 */
                taskENTER_CRITICAL();
                s_live_temp_x10 = t_x10;
                s_live_humi_x10 = h_x10;
                s_live_valid    = 1u;
                s_live_pending  = 1u;
                taskEXIT_CRITICAL();
            }
        } else {
            app_log("[mb] read FAIL err=%d\r\n", r);

            /* 掉线时不能让屏幕一直停在最后一个旧值上 */
            taskENTER_CRITICAL();
            s_live_valid   = 0u;
            s_live_pending = 1u;
            taskEXIT_CRITICAL();
        }

        /* 固定 2 秒一轮。轮询自带 200ms 超时，不再需要"串口有数据就提前醒"，
         * 否则收到的每个字节都会把任务从睡眠里拽出来一次。 */
        vTaskDelay(pdMS_TO_TICKS(MB_POLL_PERIOD_MS));
    }
}

/* ================= 网络：AT -> TCP -> MQTT ================= */
#define MQTT_PUB_PERIOD_MS   2000u      /* 正常节奏：和采样同速，稳态不积压 */
#define MQTT_CATCHUP_MS      200u       /* 补传节奏：轮与轮之间的间隔 */
#define MQTT_CATCHUP_GAP_MS  200u       /* 一轮内每条之间的间隔 */
#define MQTT_CATCHUP_MAX     5u         /* 一轮最多补几条；一轮约 1 秒 */
#define MQTT_BACKLOG_MARK    5u         /* 待发超过这个数就进补传模式 */
#define MQTT_KEEPALIVE_S     30u        /* 告诉 broker 的心跳周期 */
#define MQTT_PING_IDLE_MS    10000u     /* 空闲这么久就发一次 PINGREQ */
#define MQTT_PING_FAIL_MAX   3u         /* 连续几次收不到 PINGRESP 才判死 */

#define NTP_TRY_MAX          5u         /* 对时最多试几次，再失败就用现有时间 */

/* 日志区操作（定义在文件后面，这里先声明给 net_task 用） */
static uint32_t log_pending(void);
static int      log_peek_next(log_entry_t *e, uint32_t *addr);
static int      log_mark_sent(uint32_t addr);

/* 配网相关的三个状态量，只在 net_task 里用：
 *   UI 任务把凭据放进 wifi_cfg 的待写缓冲，内层循环发现后先搬到
 *   s_cfg_pending，再 break 回外层落盘 —— 擦 Flash 要 100ms 上下，
 *   不能卡在 MQTT 循环里，更不能让 LVGL 去写。 */
static wifi_cfg_t s_cfg_pending;
static uint8_t    s_cfg_has_pending = 0;
static uint8_t    s_wifi_fail = 0;      /* 连续关联失败次数，到达 3 次就回退 */
static uint8_t    s_ntp_done  = 0;      /* 对时成功置 1，成功过就不再问 */
static uint8_t    s_ntp_try   = 0;      /* 失败次数，够 NTP_TRY_MAX 次就放弃 */

/* WiFi 扫描（UI 发起，net_task 执行）。
 * 串口 AT 机只有 net_task 在开，UI 任务只能隔着这几个变量提需求，
 * 不能自己直接发 AT —— 透传进出的时序被两个任务交叉踩了就全乱。
 * s_scan_state 两边都写（UI 置 1 占位，net_task 置 2 出结果），
 * 都是单字节 volatile 写，M4 上天然原子，无竞态。 */
#define WIFI_SCAN_MAX     8u              /* 屏上一屏放得下、内存也吃得起 */
static uint8_t       s_scan_req   = 0;    /* UI 请求标志，net_task 取走即清 */
static volatile uint8_t s_scan_state = 0; /* 0=空闲 1=扫描中 2=结果就绪 */
static char          s_scan_ssid[WIFI_SCAN_MAX][33];
static int8_t        s_scan_rssi[WIFI_SCAN_MAX];
static int           s_scan_cnt   = 0;    /* state==2 时有效，-2=AT 失败 */

/* net_task 调：有请求就执行扫描（阻塞 3~6 秒）。
 * 挂在两个位置：内层 MQTT 循环（连着时）和外层顶部（断着时）——
 * 用户在屏上点扫描时设备往往正连着，但也可能正断线重试。 */
static void scan_service(void)
{
    if (s_scan_req == 0u) return;
    s_scan_req = 0u;

    s_scan_cnt = net_if_scan(s_scan_ssid, s_scan_rssi, (int)WIFI_SCAN_MAX);
    s_scan_state = 2u;                 /* 失败也置 2，让 UI 拿到失败码 */

    app_log("[wifi] scan done: %d\r\n", s_scan_cnt);
}

void net_task(void *args)
{
    int           r;
    uint16_t      n;
    char          line[96];
    char          topic[40];
    char          payload[96];
    char          tbuf[12];
    char          hbuf[12];
    uint8_t       rx[16];
    uint32_t      last_ping;
    uint32_t      backlog;
    uint32_t      budget;
    uint32_t      sent;
    uint8_t       ping_fail;            /* 连续收不到 PINGRESP 的次数 */
    TickType_t    t_start;

    (void)args;

    /* 用裸主题发布：和控制台手动点「发送」走同一条路径，消息会立刻刷到卡片上。
     * 之前用 "主题/up" 只写历史库（需要开通数据存储才可见），卡片看不到。 */
    snprintf(topic, sizeof(topic), "%s", BEMFA_TOPIC);

    for (;;)
    {
        /* 0. OTA 请求优先处理。
         *
         * OTA 要独占 USART3 那一条 TCP 链路，而 MQTT 正占着它，所以必须
         * 先 mqtt_reset + net_if_close 让出 socket，再连 OTA 服务器。
         * 放在循环最前面是为了保证不会在 MQTT 会话中途插进来 —— 那种时候
         * 串口里可能还有半包 PINGRESP，会被 OTA 的握手误读。
         *
         * 成功时 app_ota_run 内部直接复位装新固件，走不到下面；
         * 失败就回来当什么都没发生，重新连 MQTT 继续上报。 */
        if (app_ota_take_request())
        {
            s_mqtt_up = 0;
            mqtt_reset();
            net_if_close();                 /* +++ 退出透传 */
            vTaskDelay(pdMS_TO_TICKS(500));

            r = app_ota_run();
            if (r != 0) {
                app_log("[ota] FAILED (%d), back to normal\r\n", r);
                app_ota_set_fail((int8_t)r);   /* 转告弹窗：红字显示失败原因 */
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* 0.5 配网：把新凭据落盘 + 换凭据 + 立刻重连。
         *
         * 两个来源都要看：
         *   s_cfg_has_pending —— 内层循环（MQTT 已连）发现请求后搬过来的；
         *   wifi_cfg_take_request —— 链路本来就断着时直接在这里取。
         * 少了后一半会漏掉最关键的场景：WiFi 连不上、用户去屏上改配网，
         * 那时候根本进不了内层循环，请求会一直躺在缓冲里没人取。
         *
         * 擦写 Flash 放在这个位置的原因：
         *   1. 此时链路已经断开，不会一边发 MQTT 一边卡 100ms 擦扇区；
         *   2. 和 storage_task 抢 SPI1 时能安心等锁；
         *   3. 存完直接往下走就是重新 init + connect，新凭据立刻生效。 */
        if (s_cfg_has_pending || wifi_cfg_take_request(&s_cfg_pending)) {
            s_cfg_has_pending = 0;

            if (wifi_cfg_save(&s_cfg_pending) == 0) {
                app_log("[wifi] saved: ssid=%s\r\n", s_cfg_pending.ssid);
            } else {
                /* 存不进去（Flash 坏 / 锁超时）也让本次生效：
                 * 至少这回能连上，代价是断电后又回到旧配置。 */
                app_log("[wifi] save FAIL, apply for this run only\r\n");
            }

            net_if_set_wifi(s_cfg_pending.ssid, s_cfg_pending.pass);
            s_wifi_fail = 0;
            continue;                       /* 回去重新 init + 用新凭据连 */
        }

        /* 0.6 扫描选网：断着的时候也能扫 —— 用户去屏上配网，
         * 本来就有一半场景是"现在的网连不上"。 */
        scan_service();

        /* 1. 模块自检：AT / ATE0 / CWMODE=1 */
        r = net_if_init();
        if (r != 0) {
            app_log("[net] init FAIL (%d), retry in 5s\r\n", r);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        app_log("[net] init ok (module alive)\r\n");

        /* 2. WiFi + TCP 一次走完：CWJAP / CIPSTART / CIPMODE / CIPSEND */
        app_log("[net] connecting to %s:%u ...\r\n",
                BEMFA_HOST, (unsigned)BEMFA_PORT);

        r = net_if_connect(BEMFA_HOST, BEMFA_PORT, 20000);
        if (r != 0) {
            app_log("[net] connect FAIL (%d), retry in 10s\r\n", r);

            /* 失败时把残留回复打出来：看得出是 WiFi 卡住还是 TCP 卡住 */
            n = drv_uart_read(UART_WIFI, s_buf, (uint16_t)sizeof(s_buf));
            if (n > 0u) {
                ascii_to_buf(line, (uint16_t)sizeof(line), s_buf, n);
                app_log("[net][raw] %s\r\n", line);
            }

            /* -3 = CWJAP 关联失败。连续 3 次多半是屏上把密码打错了，
             * 用户配过网就退回上一份配置。
             *
             * 为什么要自动退：设备装在现场，配错一次就永久失联，
             * 只能拆回来接串口。让它自己退回能连的那份，代价小得多。
             * 其它错误码（-4/-5/-6）是 TCP 层的问题，跟密码无关，不计数。 */
            if (r == -3) {
                s_wifi_fail++;
                if (s_wifi_fail >= 3u && wifi_cfg_is_custom()) {
                    s_wifi_fail = 0;
                    if (wifi_cfg_fallback() == 0) {
                        app_log("[wifi] 3 fails -> rollback to %s\r\n",
                                wifi_cfg_ssid());
                        net_if_set_wifi(wifi_cfg_ssid(), wifi_cfg_pass());
                    }
                }
            } else {
                s_wifi_fail = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        app_log("[net] tcp ready\r\n");
        s_wifi_fail = 0;                    /* 关联成功，失败计数清零 */

        /* 2.5 对时：刚拿到 IP，趁 MQTT 还没连，先问一次 NTP。
         * RTC 没电池/首次上电时只有编译时刻，起点天然落后几分钟。
         * 失败不放弃、下一轮重连再问 —— 刚拿到 IP 时 DNS 常还没就绪，
         * 一次失败就作废会让整个上电周期都停在编译时刻（实测踩过）。
         * 上限 NTP_TRY_MAX 次：每次尝试都要拆一次 MQTT 的 TCP（见 net_if_ntp），
         * NTP 端口被长期封死时每轮都拆会把链路拖垮。 */
        if (s_ntp_done == 0u && s_ntp_try < NTP_TRY_MAX) {
            uint32_t ntp_ts;

            if (net_if_ntp(&ntp_ts) == 0) {
                app_log("[ntp] unix=%lu\r\n", (unsigned long)ntp_ts);
                drv_rtc_set_unix(ntp_ts);
                drv_rtc_mark_synced();      /* 从此这份时间可信，屏上可以显示 */
                s_ntp_done = 1u;
            } else {
                s_ntp_try++;
                app_log("[ntp] sync FAIL (%u/%u), retry on next reconnect\r\n",
                        (unsigned)s_ntp_try, (unsigned)NTP_TRY_MAX);
            }
        }

        /* 3. MQTT CONNECT：客户端 ID 用私钥，账号密码留空 */
        r = mqtt_connect(BEMFA_UID, MQTT_KEEPALIVE_S, 3000);
        if (r != 0) {
            app_log("[mqtt] CONNECT FAIL (%d), retry in 5s\r\n", r);
            net_if_close();
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        app_log("[mqtt] connected, publish -> %s\r\n", topic);

        s_mqtt_up = 1;
        last_ping = HAL_GetTick();
        ping_fail = 0;                  /* 新链路，心跳计数从零开始 */

        /* 4. 从 Flash 消费：读一条 -> 发一条 -> 标一条 -> 指针前进。
         *    实时上报和断网补传走同一段代码，区别只在节奏：
         *    积压少时 2 秒一条，积压多时一轮连发 10 条、间隔 200ms 追平。 */
        while (net_if_state() == NET_READY)
        {
            /* MQTT 稳定连接时 net_task 一直待在这个内层循环里，
             * 不回外层 for(;;)，OTA 请求就永远没人取 —— 按了 KEY1 没反应。
             * 这里只查不取：break 走 link_down 清场，回到外层
             * 由 take_request 真正执行（那边有完整的断链逻辑）。 */
            if (app_ota_pending()) {
                app_log("[ota] request seen, dropping mqtt link\r\n");
                break;
            }

            /* 扫描选网：退透传 + CWLAP + 回透传，MQTT 静默 3~6 秒。
             * 心跳判死要连续 3 次 × 10 秒，一次扫描打不断链路。
             * 若回透传失败，net_if_scan 里已把状态判 DOWN，
             * 下面 while 条件不成立，自然走 link_down 重连。 */
            scan_service();

            /* 屏上配网：把待写凭据搬到外层去落盘。这里只搬不写 ——
             * 擦一个扇区要 100ms 上下，留在内层会把心跳周期拖到判死。 */
            if (wifi_cfg_take_request(&s_cfg_pending)) {
                s_cfg_has_pending = 1;
                app_log("[wifi] new config from UI, dropping link\r\n");
                break;
            }

            t_start = xTaskGetTickCount();      /* 用来补偿发送耗时，避免周期漂移 */

            backlog = log_pending();
            budget  = (backlog > MQTT_BACKLOG_MARK) ? MQTT_CATCHUP_MAX : 1u;
            sent    = 0;

            /* 心跳已经失败过一次：说明链路大概率已死，只是还没到判死的次数。
             *
             * 这时候继续发只有一个结果——字节被 ESP 串口缓冲收下，
             * mqtt_publish() 返回成功，log_mark_sent() 把记录标成已发，
             * 但数据根本到不了云端，永久丢失。
             *
             * 停发才是安全的：记录留在 Flash 里标着"未发"，
             * 重连后由同一段补传逻辑发出去。
             * 代价只是网络抖动时少发一两条，下个心跳成功就自动恢复。 */
            if (ping_fail == 0)
            {
                while (sent < budget)
                {
                    log_entry_t e;
                    uint32_t    addr;

                    if (log_peek_next(&e, &addr) != 0) break;   /* 追上写指针了 */

                    /* 温湿度都按 x10 还原成一位小数。
                     * 0x03 起 humi_x10 是湿度（0~1000），不再是光照百分比。 */
                    {
                        int16_t  t  = e.temp_x10;
                        uint16_t hm = e.humi_x10;

                        if (t < 0) snprintf(tbuf, sizeof(tbuf), "-%d.%d", (-t) / 10, (-t) % 10);
                        else       snprintf(tbuf, sizeof(tbuf), "%d.%d",   t / 10,     t % 10);

                        snprintf(hbuf, sizeof(hbuf), "%u.%u",
                                 (unsigned)(hm / 10u), (unsigned)(hm % 10u));

                        /* ts = RTC Unix 时间戳：云端按真实时间落库/画曲线，
                         * 断电重启时间线不归零（RTC 降级时是上电秒数） */
                        snprintf(payload, sizeof(payload),
                                 "{\"ts\":%lu,\"temp\":%s,\"humi\":%s,\"seq\":%lu}",
                                 (unsigned long)e.ts,
                                 tbuf, hbuf, (unsigned long)e.seq);
                    }

                    if (mqtt_publish(topic, (const uint8_t *)payload,
                                     (uint16_t)strlen(payload)) != 0) {
                        app_log("[mqtt] publish FAIL, reconnecting\r\n");
                        goto link_down;
                    }

                    /* 标不上就不能让指针前进，否则这条记录永远补不回来 */
                    if (log_mark_sent(addr) != 0) {
                        app_log("[mqtt] mark FAIL at 0x%06lX\r\n", (unsigned long)addr);
                        goto link_down;
                    }

                    s_pub_cnt++;
                    sent++;

                    if (backlog <= MQTT_BACKLOG_MARK) {
                        app_log("[mqtt] -> %s\r\n", payload);        /* 正常节奏逐条打 */
                    }

                    /* 补传不能连发。实测连发约 26 条/秒，会把 ESP-01S 的串口缓冲
                     * 和 TCP 发送队列灌满，导致 AT 应答挤不进来、心跳 1 秒内收不到
                     * PINGRESP，30 秒后误判链路已死（日志表现：catch-up 刷屏后
                     * 突然 ping FAIL -3 三连）。
                     * 每条间隔 200ms 约 5 条/秒，比稳态快 10 倍，追得动又不噎住。 */
                    if (sent < budget) {
                        vTaskDelay(pdMS_TO_TICKS(MQTT_CATCHUP_GAP_MS));
                    }
                }

                /* 一轮结束汇总一行。放在循环外，中途追上写指针提前收尾时
                 * 也能看出这轮实际补了几条（原来只有凑满整轮才打印）。 */
                if (backlog > MQTT_BACKLOG_MARK && sent > 0) {
                    app_log("[mqtt] catch-up %lu/%lu\r\n",
                            (unsigned long)sent, (unsigned long)backlog);
                }
            }

            /* 把 PINGRESP 之类的字节读掉，别让环形缓冲长期堆积 */
            n = net_if_recv(rx, (uint16_t)sizeof(rx), 10);
            (void)n;

            if ((HAL_GetTick() - last_ping) >= MQTT_PING_IDLE_MS) {
                last_ping = HAL_GetTick();

                r = mqtt_ping();            /* 发 PINGREQ + 等 PINGRESP，最多 2 秒 */
                if (r == 0) {
                    ping_fail = 0;          /* 真收到应答，链路确认活着 */
                } else {
                    /* 不一次判死：网络抖动、ESP 偶尔忙不过来都会漏一两次，
                     * 连续 3 次（约 30 秒）收不到才认为链路已死。 */
                    ping_fail++;
                    app_log("[mqtt] ping FAIL (%d) %u/%u\r\n", r,
                            (unsigned)ping_fail, (unsigned)MQTT_PING_FAIL_MAX);

                    if (ping_fail >= MQTT_PING_FAIL_MAX) {
                        app_log("[mqtt] link dead, reconnecting\r\n");
                        break;
                    }
                }
            }

            /* 积压时快跑追平，追平后回到正常节奏。
             * 扣掉本轮已耗时间：否则周期 = 发送耗时 + 固定延时 > 采样周期，
             * 会缓慢积压，每隔一阵子冒一次 catch-up。
             * 心跳失败期间没在发东西，用正常节奏等下一次心跳即可，
             * 否则 backlog 一直涨会按 200ms 空转。 */
            {
                uint32_t period = (ping_fail == 0 && backlog > MQTT_BACKLOG_MARK)
                                  ? MQTT_CATCHUP_MS : MQTT_PUB_PERIOD_MS;
                uint32_t used   = (uint32_t)(xTaskGetTickCount() - t_start);

                vTaskDelay(pdMS_TO_TICKS((used < period) ? (period - used) : 1u));
            }
        }

    link_down:
        s_mqtt_up = 0;
        mqtt_reset();
        net_if_close();
        app_log("[net] link down, reconnecting\r\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* ================= 存储：外部 Flash 日志区（顺序追加，写满即停） ================= */
#define LOG_BASE        0x00400000u     /* 日志区起点（跳过 OTA 暂存 1MB + 字库 2MB） */
#define LOG_END         0x00700000u     /* 3MB = 196608 条 */
#define LOG_SCAN_CHUNK  512u            /* 扫描块；512 / 16 = 32 条整，不会切断记录 */

static uint32_t s_log_wptr = LOG_BASE;  /* 下一个空位 */
static uint32_t s_log_rptr = LOG_BASE;  /* 最老的未发记录 */
static uint8_t  s_log_full = 0;
static volatile uint8_t s_log_ready = 0; /* 扫描完成前 net_task 不许消费 */
static volatile uint32_t s_last_seq = 0; /* 日志区最后一条的序号，给采样任务续号 */

/* 扫描缓冲。必须 4 字节对齐：下面要按 log_entry_t* 访问里面的 uint32_t。 */
static uint32_t s_scan_word[LOG_SCAN_CHUNK / 4u];

/* 上电扫描：恢复写指针（第一个空位）和最老的未发记录。
 * 读量只与实际数据量成正比：只写了 4KB 就只读 8 个块。 */
static int log_store_init(void)
{
    uint8_t *buf  = (uint8_t *)s_scan_word;
    uint32_t addr = LOG_BASE;
    uint32_t n, i;
    int      rptr_found = 0;

    s_log_wptr = LOG_END;
    s_log_rptr = LOG_END;
    s_log_full = 1;

    while (addr < LOG_END) {
        n = LOG_END - addr;
        if (n > LOG_SCAN_CHUNK) n = LOG_SCAN_CHUNK;

        if (drv_flash_read(addr, buf, n) != 0) return -1;

        for (i = 0; i < n; i += SENSOR_RECORD_SIZE) {
            const log_entry_t *e = (const log_entry_t *)(buf + i);

            if (e->magic != LOG_MAGIC || e->rsv != LOG_FMT_VER) {  /* 空位或旧格式 */
                s_log_wptr = addr + i;
                s_log_full = 0;

                /* 旧格式（magic 对得上、版本号不对）：这块区域从没擦过。
                 * NOR 只能 1→0，直接往上写会把新旧字节"与"在一起，
                 * magic/rsv 永远写不对 —— 每次扫描都判成空位，反复写同一处，
                 * 日志区等于废掉。把写指针推到下一个扇区边界，
                 * 让 log_append 里的"扇区起点先擦后写"生效。
                 * 真空位（magic 是擦除态 0xFF）本来就干净，留在原位即可。 */
                if (e->magic == LOG_MAGIC) {
                    s_log_wptr = (s_log_wptr + FLASH_SECTOR_SIZE - 1u)
                                 & ~(FLASH_SECTOR_SIZE - 1u);
                    if (s_log_wptr >= LOG_END) {
                        s_log_wptr = LOG_END;
                        s_log_full = 1;
                    }
                }

                if (!rptr_found) s_log_rptr = s_log_wptr;
                goto done;
            }

            s_last_seq = e->seq;              /* 一路覆盖，最后就是最大序号 */

            if (!rptr_found && e->state == LOG_ST_UNSENT) {
                s_log_rptr = addr + i;            /* 最老的未发 */
                rptr_found = 1;
            }
        }
        addr += n;
    }

done:
    s_log_ready = 1;                  /* 指针已有效，net_task 可以开始消费 */
    app_log("[store] scan: wptr=0x%06lX rptr=0x%06lX %s\r\n",
            (unsigned long)s_log_wptr,
            (unsigned long)s_log_rptr,
            s_log_full ? "FULL" : "ok");
    return 0;
}

/* 0 = ok, -1 = io error, -2 = area full */
static int log_append(const sensor_record_t *rec)
{
    log_entry_t e;
    int         rc = 0;

    if (s_flash_mutex) xSemaphoreTake(s_flash_mutex, portMAX_DELAY);

    if (s_log_full) { rc = -2; goto out; }

    /* 4KB 一扇区 = 256 条；进入新扇区先擦除。只有顺序追加时这一步才安全：
     * 回绕覆盖会毁掉别的未发数据，等 7.3 做真环形时再改。 */
    if ((s_log_wptr % FLASH_SECTOR_SIZE) == 0u) {
        if (drv_flash_erase_sector(s_log_wptr) != 0) { rc = -1; goto out; }
    }

    memcpy(&e, rec, sizeof(sensor_record_t));
    e.magic = LOG_MAGIC;
    e.state = LOG_ST_UNSENT;          /* 擦除后的自然值，不用改写 */
    e.rsv   = LOG_FMT_VER;            /* 格式版本：老记录扫描时会被判无效 */

    if (drv_flash_write(s_log_wptr, (const uint8_t *)&e, sizeof(e)) != 0) { rc = -1; goto out; }

    s_log_wptr += sizeof(e);

    if (s_log_wptr + sizeof(e) > LOG_END) {
        s_log_full = 1;
    }

out:
    if (s_flash_mutex) xSemaphoreGive(s_flash_mutex);
    return rc;
}

/* ---------------- 7.2 补传：net_task 侧的三个只读/标记操作 ---------------- */

/* 积压条数 = (wptr - rptr) 折算。正常运行 [rptr, wptr) 全是未发记录；
 * 掉电刚恢复时可能短暂包含"已标记已发但 rptr 没来得及前进"的条目，
 * 只会高估一点，用来选上报节奏足够了。 */
static uint32_t log_pending(void)
{
    uint32_t p;

    if (!s_log_ready) return 0u;      /* 扫描期间指针是临时值，不能拿来算 */

    if (s_flash_mutex) xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    p = (s_log_wptr > s_log_rptr)
        ? ((s_log_wptr - s_log_rptr) / SENSOR_RECORD_SIZE) : 0u;
    if (s_flash_mutex) xSemaphoreGive(s_flash_mutex);

    return p;
}

/* 取最老的一条未发记录。0 = 拿到，-1 = 已经追平写指针。
 * 途中会把"已标记但指针没来得及前进"的记录跳过（掉电时才会出现）。 */
static int log_peek_next(log_entry_t *e, uint32_t *addr)
{
    int      rc = -1;
    uint32_t end;

    if (!s_log_ready) {
        return -1;
    }

    if (s_flash_mutex) {
        xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    }

    /* 先把写指针存一份：循环里只读它，避免中途被别的任务改掉导致判断错位 */
    end = s_log_wptr;

    while (s_log_rptr + SENSOR_RECORD_SIZE <= end) {
        if (drv_flash_read(s_log_rptr, (uint8_t *)e, sizeof(*e)) != 0) {
            break;                  /* Flash 读失败，指针不动，下次再试 */
        }

        if (e->magic != LOG_MAGIC || e->rsv != LOG_FMT_VER) {
            break;                  /* 空位或旧格式，说明后面还没写数据 */
        }

        if (e->state == LOG_ST_SENT) {
            s_log_rptr += SENSOR_RECORD_SIZE;
            continue;               /* 这条已经发过，跳过继续找 */
        }

        *addr = s_log_rptr;
        rc = 0;
        break;
    }

    if (s_flash_mutex) {
        xSemaphoreGive(s_flash_mutex);
    }

    return rc;
}

/* 原地把 state 从 0xFF 写成 0x00，只写一个字节，不擦扇区。
 * 成功后读指针前进一格；写失败不动指针 —— 否则这条记录永远补不回来。 */
static int log_mark_sent(uint32_t addr)
{
    uint8_t st = LOG_ST_SENT;
    int     rc;

    if (s_flash_mutex) xSemaphoreTake(s_flash_mutex, portMAX_DELAY);

    rc = drv_flash_write(addr + (uint32_t)offsetof(log_entry_t, state), &st, 1u);
    if (rc == 0 && s_log_rptr == addr) {
        s_log_rptr += SENSOR_RECORD_SIZE;
    }

    if (s_flash_mutex) xSemaphoreGive(s_flash_mutex);
    return rc;
}

void storage_task(void *args)
{
    sensor_record_t rec;
    uint32_t n = 0;
    uint32_t dropped = 0;
    int      r;
    (void)args;

    log_store_init();                 /* 必须先扫，否则会从 0x400000 覆盖旧数据 */

    while (1)
    {
        /* OTA 正在往外部 Flash 写固件，这里必须让路。
         *
         * 两个任务优先级相同（都是 2），FreeRTOS 按时间片轮转，本任务随时
         * 可能在 OTA 那一页 256 字节写到一半时被调度上来插一笔 —— SPI 总线
         * 上两组写命令交错，固件数据就坏了，而 CRC 要到最后一刻才发现。
         *
         * 让路的代价是这段时间的采样数据先攒在队列里（16 条 = 32 秒），
         * 攒满就丢。OTA 一分钟一次的事，丢几条远好过把固件写坏。 */
        if (app_ota_busy()) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (xQueueReceive(s_data_queue, &rec, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        r = log_append(&rec);
        if (r == 0) {
            n++;
            if ((n % 50u) == 0u) {
                app_log("[store] %lu recs, next=0x%06lX\r\n",
                        (unsigned long)n, (unsigned long)s_log_wptr);
            }
        } else if (r == -2) {
            if (dropped++ == 0u) {    /* 只报一次，写满后不刷屏 */
                app_log("[store] log area FULL, stop storing\r\n");
            }
        } else {
            app_log("[store] write FAIL at 0x%06lX\r\n", (unsigned long)s_log_wptr);
        }
    }
}

/* ================= 采样 ================= */
void sample_task(void *args)
{
    sensor_record_t rec;
    int16_t  t_x10;
    uint16_t h_x10;
    uint8_t  has_new;
    uint8_t  ok;
    uint32_t dropped = 0u;
    (void)args;

    /* 初值：还没拿到任何读数，屏幕显示 "--.-" */
    taskENTER_CRITICAL();
    s_latest.seq      = 0u;
    s_latest.temp_x10 = -999;
    s_latest.humi_x10 = 0xFFFFu;
    s_latest.temp_mv  = 0u;
    taskEXIT_CRITICAL();

    while (1)
    {
        /* 取走 uart_poll_task 这一轮的结果。清零也在临界区里，
         * 否则两次采样之间可能把同一轮读数记录两遍。 */
        taskENTER_CRITICAL();
        has_new = s_live_pending;
        t_x10   = s_live_temp_x10;
        h_x10   = s_live_humi_x10;
        ok      = s_live_valid;
        s_live_pending = 0u;
        taskEXIT_CRITICAL();

        /* 这一轮还没出新读数（485 正在重试），不动屏幕也不产生记录 */
        if (has_new == 0u) {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_TASK_PERIOD));
            continue;
        }

        /* 读数无效：不写 Flash、不上报，只把屏幕刷成无效态。
         * 否则云端会出现 0.0℃ 这种假数据点，比少一条更糟。 */
        if (ok == 0u) {
            taskENTER_CRITICAL();
            s_latest.temp_x10 = -999;
            s_latest.humi_x10 = 0xFFFFu;
            taskEXIT_CRITICAL();

            vTaskDelay(pdMS_TO_TICKS(SAMPLE_TASK_PERIOD));
            continue;
        }

        /* 序号接着 Flash 里的最大号往下排，重启后不会从 0 重来 */
        rec.seq      = ++s_last_seq;
        rec.ts       = drv_rtc_unix();   /* RTC 真实时间；降级时等于上电秒数 */
        rec.temp_x10 = t_x10;
        rec.humi_x10 = h_x10;
        rec.valid    = 1u;

        /* 队列满就丢这一条：宁可少一条，也不能阻塞采样 */
        if (xQueueSend(s_data_queue, &rec, 0) != pdTRUE) {
            if (dropped++ == 0u) {
                app_log("[sample] queue FULL, drop\r\n");
            }
        }

        taskENTER_CRITICAL();
        s_latest.seq      = rec.seq;
        s_latest.temp_x10 = t_x10;
        s_latest.humi_x10 = h_x10;
        taskEXIT_CRITICAL();

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TASK_PERIOD));
    }
}

/* ================= 给 app_ui.c 的显示数据接口 =================
 *
 * UI 段拆去 app_ui.c 后，ui_task 不能再直接读这边的私有变量
 * （s_latest / s_mqtt_up / s_pub_cnt / log_pending()），
 * 统一从这四个只读 getter 走。临界区逻辑原来在 ui_task 里，
 * 原样搬进 app_data_get_snapshot()，行为不变。 */

void app_data_get_snapshot(ui_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *out = s_latest;
    taskEXIT_CRITICAL();
}

uint8_t app_net_mqtt_up(void)
{
    return s_mqtt_up;
}

uint32_t app_net_pub_cnt(void)
{
    return s_pub_cnt;
}

uint32_t app_store_log_pending(void)
{
    return log_pending();
}

/* ================= WiFi 扫描：UI 侧接口 =================
 * 状态机见 net_task 前面的 s_scan_* 注释。 */

void app_wifi_scan_request(void)
{
    if (s_scan_state == 1u) return;     /* 正在扫，重复点按直接忽略 */
    s_scan_req   = 1u;
    s_scan_state = 1u;                  /* 先占住"扫描中"，net_task 随后执行 */
}

uint8_t app_wifi_scan_state(void)
{
    return s_scan_state;
}

/* 读走结果（拷贝到调用方缓冲）。只读 s_scan_state==2 的瞬间，
 * 期间 net_task 不会再动这几个数组 —— 新扫描要先过 request，
 * 而 request 的入口在 UI 的扫描层上，层关着就点不到。 */
int app_wifi_scan_get(char (*out_ssid)[33], int8_t *out_rssi, int max)
{
    int n;
    int i;

    if (s_scan_state != 2u) return -1;

    n = (s_scan_cnt < max) ? s_scan_cnt : max;
    if (n < 0) {
        return s_scan_cnt;              /* AT 失败码原样交给 UI 显示 */
    }

    for (i = 0; i < n; i++) {
        strcpy(out_ssid[i], s_scan_ssid[i]);
        out_rssi[i] = s_scan_rssi[i];
    }
    return n;
}

/* ================= 外部 Flash 互斥（给别的模块用） =================
 *
 * s_flash_mutex 是本文件的私有变量。配网模块要擦写自己的配置区，
 * 又必须和 storage_task 的日志写入串行，所以在这两个薄封装里拿锁，
 * 而不是把信号量句柄暴露出去 —— 暴露句柄等于允许任何地方乱 Give。 */

void app_flash_lock(void)
{
    if (s_flash_mutex) {
        xSemaphoreTake(s_flash_mutex, portMAX_DELAY);
    }
}

void app_flash_unlock(void)
{
    if (s_flash_mutex) {
        xSemaphoreGive(s_flash_mutex);
    }
}

