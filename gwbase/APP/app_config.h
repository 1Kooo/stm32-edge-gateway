/**
 * @file    app_config.h
 * @brief   Configuration entry point (committed to git).
 *
 * Real credentials live in app_config_local.h, which is git-ignored and
 * enabled by defining APP_CONFIG_LOCAL in the Keil project options.
 * If that macro is not defined, the placeholders below are used and the
 * network simply will not connect - that is intended, so a fresh clone
 * still compiles without leaking anything.
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef APP_CONFIG_LOCAL
#include "app_config_local.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID        "YOUR_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD    "YOUR_PASSWORD"
#endif

#ifndef BEMFA_UID
#define BEMFA_UID        "YOUR_UID"
#endif

#ifndef BEMFA_TOPIC
#define BEMFA_TOPIC      "YOUR_TOPIC"
#endif

#ifndef BEMFA_HOST
#define BEMFA_HOST       "mqtt.bemfa.com"
#endif

#ifndef BEMFA_PORT
#define BEMFA_PORT       9501
#endif

/* OTA 升级服务器：跑在 PC 上的 ota_server.py（D:\gw\tools\）。
 * 默认值是 Windows 移动热点的网关地址 —— PC 开了热点时，
 * PC 自己就是 192.168.137.1，板子连这个热点后能直接访问。
 * 换成别的网络环境时，在 app_config_local.h 里覆盖这两项即可。 */
#ifndef OTA_HOST
#define OTA_HOST         "192.168.137.1"
#endif

#ifndef OTA_PORT
#define OTA_PORT         8888
#endif

/* 固件版本号：串口横幅、开机 LCD、LVGL 主页标签统一用它。
 * 之前 app_main.c 私有 + app_freertos.c 硬编码两处 "GW V0.3"，
 * 屏幕和串口各说各话；现在收口到这一处。
 * 每次发版改这里（或加 build 号），OTA 验证时才能靠横幅分辨新旧。
 *
 * V0.5 = V0.4 的回滚底座 + 屏上配网全链路（外部 Flash 双槽存储 /
 * 断链自动换网重连 / 配错 3 次自动回退）+ 中文字库与中文界面 +
 * 触摸 y 轴镜像修复 + UI 拆分到 app_ui.c + 扫描选网。 */
#ifndef APP_VERSION
#define APP_VERSION      "V0.5"
#endif

#endif /* __APP_CONFIG_H */
