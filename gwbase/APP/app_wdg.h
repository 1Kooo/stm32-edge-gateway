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

/**
 * @brief 启动最早期就把 IWDG 跑起来（幂等）。
 *
 * 为什么要在调度器之前就开：wdg_task 要等调度器启动才跑，在那之前如果
 * 卡在 LCD / LVGL / 事件区扫描里，没有任何东西能把芯片拉回来 —— 表现为
 * "永远停在自检页"，只能人跑去拔电。提前开了之后，这类卡死 30 秒后自动
 * 复位；若走的是 OTA 安装，复位还会让 boot 的 try 计数累加，最终触发回滚。
 *
 * 代价：起不来的固件会变成"每 30 秒重启一次"的循环，而不是静静停住。
 *
 * @return 0 = 看门狗已运行（或本来就在运行），1 = 未启用/初始化失败
 */
uint8_t app_wdg_start_early(void);

/**
 * @brief 启动阶段直接喂狗（不经过心跳判定）。
 *
 * 只在 app_main 里那些可能耗时较长的初始化步骤前后调用；调度器起来之后
 * 由 wdg_task 统一按心跳决定喂不喂，不要再从别处调这个。
 */
void app_wdg_feed(void);

#endif /* __APP_WDG_H */
