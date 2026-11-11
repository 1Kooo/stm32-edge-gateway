/**
 * @file    app_event.h
 * @brief   系统事件日志：外部 Flash 独立环形区，记录"发生过什么"。
 *
 * 和数据日志（app_freertos.c 里的 log_*）是两回事：
 *   数据日志 = 采样值，必须保证不丢（写满有积压就停记）
 *   事件日志 = 系统动作，允许覆盖最老（屏上只看最近几十条）
 *
 * 谁都能记（app_event_log 入队即返回），但真正写 Flash 只由 storage_task
 * 调 app_event_flush() 完成 —— 外部 Flash 只有它有写权限。
 */

#ifndef __APP_EVENT_H
#define __APP_EVENT_H

#include <stdint.h>

/* 事件码。加新码只在这里加，名字表在 app_event.c 里同步 */
#define EVT_POWER_ON     1u
#define EVT_NET_UP       2u
#define EVT_NET_DOWN     3u
#define EVT_NTP_OK       4u
#define EVT_NTP_FAIL     5u
#define EVT_PARAM_CHG    6u
#define EVT_FACTORY      7u
#define EVT_REBOOT       8u
#define EVT_OTA_START    9u
#define EVT_OTA_OK      10u
#define EVT_OTA_FAIL    11u
#define EVT_ALARM_ON    12u
#define EVT_ALARM_OFF   13u
#define EVT_STORE_FULL  14u
#define EVT_STORE_WRAP  15u
#define EVT_GOLDEN      16u   /* 黄金备份固化成功（时间戳给关于页显示用） */

typedef struct {
    uint32_t ts;        /* Unix 秒 */
    uint8_t  code;      /* 事件码 */
    uint8_t  arg;       /* 附加信息，含义随事件码而定 */
} evt_t;

/* 上电扫描写指针。必须在 drv_flash_init() 之后、调度器启动之前调用 */
void app_event_init(void);

/* 记一条事件：只入队，不碰 Flash，任意任务/中断外上下文都能调 */
void app_event_log(uint8_t code, uint8_t arg);

/* 把队列里的事件写进 Flash。只由 storage_task 调 */
void app_event_flush(void);

/* 已写入的总条数（含覆盖），UI 用它判断"要不要重载列表" */
uint32_t app_event_seq(void);

/* 读最近 max 条，最新的放在 buf[0]。返回实际读到的条数 */
int app_event_recent(evt_t *buf, int max);

/* 最近一次「固化黄金备份」的时间戳（Unix 秒）。0 = 从没固化过。
 * 由 app_event_init 扫描事件区时顺手缓存，不额外读 Flash。 */
uint32_t app_event_last_golden_ts(void);

/* 事件码 -> 中文名（返回的是常量字符串，不要改） */
const char *app_event_name(uint8_t code);

/* 1 = 这条算"异常"，屏上给个标记（单 label 无法逐行改色，用符号区分） */
uint8_t app_event_is_warn(uint8_t code);

/* arg 的简短说明，参数在 < 0 时表示这条事件没有参数（不要显示） */
const char *app_event_arg_text(uint8_t code, uint8_t arg, char *buf, int bufsz);

#endif /* __APP_EVENT_H */
