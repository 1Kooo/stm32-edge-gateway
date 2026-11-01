/**
 * @file    lv_port.h
 * @brief   LVGL 移植层对外接口。
 *
 * 只负责把 LVGL 接到现有硬件上，不创建任何界面。
 * 调用 lv_port_init() 之后就可以正常使用 lv_obj_xxx_create() 系列 API。
 */

#ifndef __LV_PORT_H
#define __LV_PORT_H

#include "lvgl.h"

/**
 * 初始化 LVGL：时基 + 显示 + 输入。
 *
 * 必须在 FreeRTOS 任务上下文里调用（lv_conf.h 中 LV_USE_OS = LV_OS_FREERTOS，
 * lv_init() 内部会创建互斥量，调度器没起来会失败）。
 *
 * 调用之后，需要周期性调用 lv_timer_handler()（建议 5~10ms 一次），
 * 否则界面不会刷新、动画不会动、触摸也没反应。
 */
void lv_port_init(void);

/**
 * 取走一次滑动结果并清零。
 *
 * 不用 LVGL 自带的 LV_EVENT_GESTURE：那套逻辑逐帧累加位移，任何一帧没动就
 * 整体清零，电阻屏轻按时采样会失败、坐标不更新，导致手势时灵时不灵。
 * 这里改成按下记起点、全程跟踪最大位移，松手时用峰值判定，和力度、速度无关。
 *
 * 需要在循环里轮询（建议 20ms 一次以内），取到值就切屏。
 *
 * @return 0 = 本次没有滑动，1 = 往左滑，2 = 往右滑
 */
uint8_t lv_port_take_swipe(void);

#endif /* __LV_PORT_H */
