/**
 * @file    app_wdg.h
 * @brief   看门狗心跳约定：谁在哪个槽位打卡。
 *
 * 只有 app_freertos.c 里的 wdg_task 会读这些槽位，其他任务只管打自己的卡。
 * 加减任务时两边一起改，WDG_HB_NUM 会自动跟着变。
 */

#ifndef __APP_WDG_H
#define __APP_WDG_H

#include <stdint.h>

enum {
    WDG_HB_UART = 0,    /* uart_poll_task：一轮 Modbus 读写 */
    WDG_HB_SAMPLE,      /* sample_task：一次取数 */
    WDG_HB_STORE,       /* storage_task：一条入库或一次空转 */
    WDG_HB_UI,          /* ui_task（在 app_ui.c）：一轮 lv_timer_handler */
    WDG_HB_NET,         /* net_task：一轮 MQTT 收发，OTA 每块、NTP 每次尝试 */
    WDG_HB_NUM
};

/* 记下"此刻该任务推进过了"。id 越界直接忽略，不当错误处理 */
void app_wdg_kick(uint8_t id);

#endif /* __APP_WDG_H */
