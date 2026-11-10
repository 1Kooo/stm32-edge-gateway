/**
 * @file    app_ui.h
 * @brief   LVGL 界面模块：所有屏幕构建、刷新、ui_task 都在 app_ui.c。
 *
 * app_freertos.c 只负责创建任务，不碰任何 lv_ 函数。
 * UI 需要的运行数据（采样快照 / MQTT 状态 / 补传积压）通过
 * app_freertos.c 提供的只读 getter 获取，双方不直接摸对方的变量。
 */

#ifndef __APP_UI_H
#define __APP_UI_H

#include <stdint.h>

/* 唯一碰 LCD 的任务入口，App_freeRTOS_start() 里 xTaskCreate 用。
 * 其他任务只写内存，避免总线竞争。 */
void ui_task(void *args);

/* 给 ui_task 显示用的快照：sample_task 每轮在临界区里更新 s_latest，
 * UI 侧通过 app_data_get_snapshot() 取，避免直接读对方的变量。 */
typedef struct {
    uint32_t seq;
    int16_t  temp_x10;       /* 253 = 25.3 C，-999 = 无效 */
    uint16_t humi_x10;       /* 450 = 45.0 %RH，0xFFFF = 无效 */
    uint16_t temp_mv;        /* 老字段，当前无生产者，留着占位 */
} ui_snapshot_t;

/* app_freertos.c 实现的显示数据接口（ui_snapshot_t 定义在本头文件） */
void     app_data_get_snapshot(ui_snapshot_t *out);
uint8_t  app_net_mqtt_up(void);          /* 1 = MQTT 在线 */
uint32_t app_net_pub_cnt(void);          /* 累计已上报条数 */
uint32_t app_store_log_pending(void);    /* 待补传积压条数 */

/* WiFi 扫描（UI 发起请求，net_task 执行 —— 串口只有一个主人，
 * 透传进出的活别的任务碰会撞车）。状态轮询 + 一次拷贝，全程不阻塞 UI：
 *   app_wifi_scan_request()  发起；正在扫时再发会被忽略
 *   app_wifi_scan_state()    0=空闲 1=扫描中(约 3~6 秒) 2=结果就绪
 *   app_wifi_scan_get()      拷走结果，返回条数；SSID 每项 33 字节
 *                            中文网络名按 UTF-8 原样存放 */
void     app_wifi_scan_request(void);
uint8_t  app_wifi_scan_state(void);
int      app_wifi_scan_get(char (*out_ssid)[33], int8_t *out_rssi, int max);

#endif /* __APP_UI_H */
