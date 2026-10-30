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
#include "drv_dht11.h"
#include "drv_adc.h"
#include "drv_spiflash.h"
#include "tftlcd.h"
#include "bsp_pinmap.h"
#include "data_model.h"
#include "net_if.h"
#include "mqtt_client.h"
#include "app_config.h"

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

/* 串口唤醒计数：只在中断叫醒任务时 +1，用来说明任务确实在睡 */
static volatile uint32_t s_uart_wake = 0;

/* MQTT 状态：给 ui_task 显示 */
static volatile uint8_t  s_mqtt_up = 0;
static volatile uint32_t s_pub_cnt = 0;

/* 采样 -> 存储 的数据管道 */
#define DATA_QUEUE_LEN  16
static QueueHandle_t s_data_queue = NULL;

/* 给 ui_task 显示用的快照（临界区拷贝） */
typedef struct {
    uint32_t seq;
    uint8_t  light_pct;      /* 0~100 % */
    int16_t  temp_x10;       /* 253 = 25.3 C，-999 = 无效 */
    uint16_t light_mv;
    uint16_t temp_mv;
} ui_snapshot_t;

static ui_snapshot_t s_latest;

/* 热敏电阻电路方向：
 *   1 = VCC -- 10K -- AO -- NTC -- GND  （温度↑ 电压↓）← 最常见
 *   0 = VCC -- NTC -- AO -- 10K -- GND  （温度↑ 电压↑）
 * 实测：捏住热敏看 mV，变小填 1，变大填 0 */
#define NTC_LOW_SIDE    1

#define NTC_R25         10000.0f
#define NTC_B           3950.0f
#define NTC_R_FIXED     10000.0f
#define ADC_VREF_F      3.3f

/* 返回 温度 x10（253 = 25.3C）；-999 表示开路/短路 */
static int16_t ntc_mv_to_temp_x10(uint16_t mv)
{
    float v = (float)mv / 1000.0f;
    float r_ntc, inv_t, t_c;

    if (v <= 0.05f || v >= 3.25f) return -999;

#if NTC_LOW_SIDE
    r_ntc = NTC_R_FIXED * v / (ADC_VREF_F - v);
#else
    r_ntc = NTC_R_FIXED * (ADC_VREF_F - v) / v;
#endif

    inv_t = 1.0f / 298.15f + logf(r_ntc / NTC_R25) / NTC_B;
    t_c   = 1.0f / inv_t - 273.15f;

    if (t_c < -40.0f) t_c = -40.0f;
    if (t_c >  99.0f) t_c =  99.0f;

    return (int16_t)(t_c * 10.0f);
}

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
// 最小推荐填写128 => 128*4 = 512B
#define LED_TASK_STACK_SIZE 256
// 任务优先级 => 数值越小优先级越小 => 最大7 => 推荐使用最小优先级0
#define LED_TASK_PRIORITY   1
// LED任务周期 500ms
#define LED_TASK_PERIOD     500
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
#define UART_TASK_PERIOD     10
TaskHandle_t uart_task_handle;

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
void ui_task(void *args);
#define UI_TASK_STACK_SIZE   768
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
        app_log("[beat] t=%lus heap=%u stack=%u wake=%lu\r\n",
                (unsigned long)sec,
                (unsigned)xPortGetFreeHeapSize(),
                (unsigned)uxTaskGetStackHighWaterMark(NULL),
                (unsigned long)s_uart_wake);
        vTaskDelay(pdMS_TO_TICKS(BEAT_TASK_PERIOD));
    }
}

/* 串口中断回调（中断上下文）：叫醒等待中的任务 */
static void uart_rx_hook(uart_id_t id)
{
    BaseType_t woken = pdFALSE;
    (void)id;

    if (uart_task_handle != NULL) {
        vTaskNotifyGiveFromISR(uart_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void uart_poll_task(void *args)
{
    uint16_t   n;
    TickType_t wait = 0;        /* 第一轮不阻塞：先处理开机期间已堆积的数据 */
    (void)args;

    /* USART3 交给 net_task 独占，这里只管 485 */
    drv_uart_set_rx_callback(UART_RS485, uart_rx_hook);

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, wait);     /* 无数据时完全睡眠，不占 CPU */
        wait = portMAX_DELAY;
        s_uart_wake++;

        /* 把 485 缓冲读空并原样回显 */
        for (;;) {
            n = drv_uart_read(UART_RS485, s_buf, (uint16_t)sizeof(s_buf));
            if (n == 0) break;
            drv_uart_write(UART_RS485, s_buf, n, 100u);
        }
    }
}

/* ================= 网络：AT -> TCP -> MQTT ================= */
#define MQTT_PUB_PERIOD_MS   2000u      /* 正常节奏：和采样同速，稳态不积压 */
#define MQTT_CATCHUP_MS      200u       /* 补传节奏：积压时快追 */
#define MQTT_CATCHUP_MAX     10u        /* 一轮最多补几条 */
#define MQTT_BACKLOG_MARK    5u         /* 待发超过这个数就进补传模式 */
#define MQTT_KEEPALIVE_S     30u        /* 告诉 broker 的心跳周期 */
#define MQTT_PING_IDLE_MS    10000u     /* 空闲这么久就发一次 PINGREQ */
#define MQTT_PING_FAIL_MAX   3u         /* 连续几次收不到 PINGRESP 才判死 */

/* 日志区操作（定义在文件后面，这里先声明给 net_task 用） */
static uint32_t log_pending(void);
static int      log_peek_next(log_entry_t *e, uint32_t *addr);
static int      log_mark_sent(uint32_t addr);

void net_task(void *args)
{
    int           r;
    uint16_t      n;
    char          line[96];
    char          topic[40];
    char          payload[72];
    char          tbuf[12];
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

            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        app_log("[net] tcp ready\r\n");

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

                    if (e.temp_x10 == -999) {
                        snprintf(payload, sizeof(payload),
                                 "{\"light\":%u,\"seq\":%lu}",
                                 (unsigned)e.humi_x10, (unsigned long)e.seq);
                    } else {
                        int16_t t = e.temp_x10;

                        if (t < 0) snprintf(tbuf, sizeof(tbuf), "-%d.%d", (-t) / 10, (-t) % 10);
                        else       snprintf(tbuf, sizeof(tbuf), "%d.%d",   t / 10,     t % 10);

                        snprintf(payload, sizeof(payload),
                                 "{\"temp\":%s,\"light\":%u,\"seq\":%lu}",
                                 tbuf, (unsigned)e.humi_x10, (unsigned long)e.seq);
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
                    } else if ((sent % 10u) == 0u || sent == budget) {
                        app_log("[mqtt] catch-up %lu/%lu\r\n",        /* 补传按批汇总，防刷屏 */
                                (unsigned long)sent, (unsigned long)backlog);
                    }
                }
            }

            /* 把 PINGRESP 之类的字节读掉，别让环形缓冲长期堆积 */
            n = net_if_recv(rx, (uint16_t)sizeof(rx), 10);
            (void)n;

            if ((HAL_GetTick() - last_ping) >= MQTT_PING_IDLE_MS) {
                last_ping = HAL_GetTick();

                r = mqtt_ping();            /* 发 PINGREQ + 等 PINGRESP，最多 1 秒 */
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

/* 积压量（含已标记的，用来判断该用正常节奏还是补传节奏） */
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
    int rc = -1;

    if (!s_log_ready) return -1;

    if (s_flash_mutex) xSemaphoreTake(s_flash_mutex, portMAX_DELAY);

    while (s_log_rptr + SENSOR_RECORD_SIZE <= s_log_wptr) {
        if (drv_flash_read(s_log_rptr, (uint8_t *)e, sizeof(*e)) != 0) break;
        if (e->magic != LOG_MAGIC || e->rsv != LOG_FMT_VER) break;  /* 空位/旧格式 */

        if (e->state == LOG_ST_SENT) {                    /* 已发，跳过 */
            s_log_rptr += SENSOR_RECORD_SIZE;
            continue;
        }

        *addr = s_log_rptr;
        rc = 0;
        break;
    }

    if (s_flash_mutex) xSemaphoreGive(s_flash_mutex);
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
    uint32_t seq;
    uint16_t light, temp;
    (void)args;

    drv_adc_init();

    /* 等存储区扫描完成，接着上次的序号往下数。
     * 否则每次重启 seq 都从 1 开始，云端会看到序号周期性回退。 */
    {
        uint32_t waited = 0;

        while (!s_log_ready && waited < 250u) {     /* 最多等 5 秒，防止卡死 */
            vTaskDelay(pdMS_TO_TICKS(20));
            waited++;
        }
        seq = s_last_seq;
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TASK_PERIOD));

        light = drv_adc_read_raw(ADC_CH_LIGHT);
        temp  = drv_adc_read_raw(ADC_CH_TEMP);
        seq++;

        /* 落盘和上屏必须是同一份换算结果，否则补传会发出原始比例而不是温度。
         * humi_x10 这个字段被复用为光照百分比（0~100），和 s_latest 保持一致。 */
        {
            uint16_t light_mv = (uint16_t)((uint32_t)light * 3300u / 4096u);
            uint16_t temp_mv  = (uint16_t)((uint32_t)temp  * 3300u / 4096u);
            int16_t  t_x10    = ntc_mv_to_temp_x10(temp_mv);
            uint8_t  l_pct    = (uint8_t)(100u - ((light * 100u) / 4095u));

            rec.seq      = seq;
            rec.ts       = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
            rec.temp_x10 = t_x10;
            rec.humi_x10 = (uint16_t)l_pct;
            rec.valid    = (t_x10 == -999) ? 0u : 1u;
            rec.rsv[0]   = 0;
            rec.rsv[1]   = 0;
            rec.rsv[2]   = 0;

            /* 不打印，只更新快照，由 ui_task 显示 */
            taskENTER_CRITICAL();
            s_latest.seq       = seq;
            s_latest.light_pct = l_pct;
            s_latest.temp_x10  = t_x10;
            s_latest.light_mv  = light_mv;
            s_latest.temp_mv   = temp_mv;
            taskEXIT_CRITICAL();
        }

        if (xQueueSend(s_data_queue, &rec, 0) != pdTRUE) {
            app_log("[sample] queue full, drop #%lu\r\n", (unsigned long)seq);
        }
    }
}

/* ================= UI 实现 ================= */
static void ui_draw_static(void)
{
    LCD_Clear(WHITE);
    FRONT_COLOR = BLACK;

    LCD_ShowString(10,  10, 300, 480, 24, (u8 *)"GW V0.3");
    LCD_ShowString(10,  70, 300, 480, 16, (u8 *)"LIGHT");
    LCD_ShowString(10, 170, 300, 480, 16, (u8 *)"TEMP");
    LCD_ShowString(10, 280, 300, 480, 16, (u8 *)"------ system ------");
}

void ui_task(void *args)
{
    ui_snapshot_t snap;
    char          line[40];
    uint32_t      last_seq = 0;
    uint8_t       tick_cnt = 0;
    (void)args;

    ui_draw_static();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(UI_TASK_PERIOD));

        taskENTER_CRITICAL();
        snap = s_latest;                /* 16 字节快照，几微秒 */
        taskEXIT_CRITICAL();

        if (snap.seq != last_seq)
        {
            last_seq = snap.seq;

            /* 光照 */
            sprintf(line, "%3u %%       ", (unsigned)snap.light_pct);
            LCD_ShowString(10,  95, 300, 480, 24, (u8 *)line);
            sprintf(line, "%4u mV     ", (unsigned)snap.light_mv);
            LCD_ShowString(10, 130, 300, 480, 16, (u8 *)line);

            /* 温度 */
            if (snap.temp_x10 == -999)
            {
                sprintf(line, " --.- C     ");
            }
            else
            {
                int16_t t    = snap.temp_x10;
                char    sign = ' ';

                if (t < 0) { sign = '-'; t = (int16_t)(-t); }
                sprintf(line, "%c%2d.%d C    ", sign, t / 10, t % 10);
            }
            LCD_ShowString(10, 195, 300, 480, 24, (u8 *)line);
            sprintf(line, "%4u mV     ", (unsigned)snap.temp_mv);
            LCD_ShowString(10, 230, 300, 480, 16, (u8 *)line);
        }

        if (++tick_cnt >= 2u)           /* 系统信息 1 秒刷一次 */
        {
            tick_cnt = 0;

            sprintf(line, "uptime %6lu s  ",
                    (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ));
            LCD_ShowString(10, 310, 300, 480, 16, (u8 *)line);

            sprintf(line, "heap   %6u    ", (unsigned)xPortGetFreeHeapSize());
            LCD_ShowString(10, 335, 300, 480, 16, (u8 *)line);

            sprintf(line, "stack  %6u    ", (unsigned)uxTaskGetStackHighWaterMark(NULL));
            LCD_ShowString(10, 360, 300, 480, 16, (u8 *)line);

            sprintf(line, "wake   %6lu    ", (unsigned long)s_uart_wake);
            LCD_ShowString(10, 385, 300, 480, 16, (u8 *)line);

            /* 网络状态：down / connecting / ready */
            sprintf(line, "NET    %s        ", net_if_state_str());
            LCD_ShowString(10, 410, 300, 480, 16, (u8 *)line);

            /* MQTT：是否在线 + 已上报条数 */
            sprintf(line, "MQTT   %s  %lu     ",
                    s_mqtt_up ? "up  " : "down", (unsigned long)s_pub_cnt);
            LCD_ShowString(10, 435, 300, 480, 16, (u8 *)line);
        }
    }
}
