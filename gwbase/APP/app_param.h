/**
 * @file    app_param.h
 * @brief   可掉电保存的运行参数（外部 Flash，A/B 双备份）。
 *
 * 存的是"用户能改的东西"：轮询/上报周期、告警阈值。
 * 不存传感器数据、不存 WiFi 凭据（那个在 app_wifi_cfg 里另有一套）。
 */

#ifndef __APP_PARAM_H
#define __APP_PARAM_H

#include <stdint.h>

/* 改这个结构要同步升 PARAM_VER，否则老数据会被版本号判废、回默认值 */
typedef struct {
    uint32_t mb_ms;         /* 485 轮询周期，毫秒 */
    uint32_t pub_ms;        /* MQTT 上报周期，毫秒 */
    int16_t  t_min_x10;     /* 温度告警下限，0.1℃ */
    int16_t  t_max_x10;     /* 温度告警上限，0.1℃ */
    uint16_t h_min_x10;     /* 湿度告警下限，0.1%RH */
    uint16_t h_max_x10;     /* 湿度告警上限，0.1%RH */
    uint8_t  alarm_en;      /* 0 = 不判告警；收到任何阈值命令后自动置 1 */
} app_param_t;

void app_param_default(app_param_t *p);

/* 上电读一次。返回 0 = 读到合法参数；1 = 两份都无效，已把默认值写进去。
 * 必须在 drv_flash_init() 之后、调度器启动之前调用。 */
int app_param_load(app_param_t *p);

/* 落盘。会擦掉"另一格"再写，写完回读校验。
 * 任务上下文里只能在 storage_task 调 —— 外部 Flash 只有它有写权限。
 * 唯一例外：app_param_load() 发现两格都无效时会就地写一次默认值，
 * 那是在调度器启动前、单线程跑，不会和其他写者冲突。 */
int app_param_save(const app_param_t *p);

#endif /* __APP_PARAM_H */
