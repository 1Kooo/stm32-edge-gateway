#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#include "system.h"

/* Step 1: bring-up entry. Called once from main() after peripherals init. */
void app_main(void);

/* Step 4: create all tasks and start the scheduler. Never returns. */
void App_freeRTOS_start(void);

/* 外部 SPI Flash 的互斥（底层是 app_freertos.c 里的 s_flash_mutex）。
 * 日志区、OTA 暂存、WiFi 配置区共用一颗 Flash，任何模块自己擦写前
 * 都要先拿这把锁，否则会和 storage_task 的写操作交织成一堆乱数据。 */
void app_flash_lock(void);
void app_flash_unlock(void);

#endif
