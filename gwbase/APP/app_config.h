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
 * 触摸 y 轴镜像修复 + UI 拆分到 app_ui.c + 扫描选网。
 * V0.6 = V0.5 + Modbus RTU 温湿度采集（485 / P8 端子）+ 数据链路接入
 * （LOG_FMT_VER 0x03：humi_x10 语义从光照换湿度）+ OTA 屏显进度。
 * V0.7 = V0.6 + RTC 真实时间戳（LOG_FMT_VER 0x04：ts 换 Unix 秒，
 * MQTT 报文加 ts 键）+ OTA 弹窗（遮罩/版本行/失败反馈/黄金备份进度）
 * + KEY2 三页循环导航。
 * V0.8 = V0.7 + 首页顶栏改显 RTC 实时时间（运行时长移到关于页）、
 * 关于页加「时间」行、OTA 弹窗背景渐变 + 面板阴影。
 * V0.9 = V0.8 + 联网 NTP 对时（UDP 直发 48 字节请求，不挑 ESP AT
 * 固件版本），抹掉"编译时刻"起点带来的固定偏差。
 * V1.0 = V0.5~V0.9 全链路实机验收版：Modbus 采集 + Flash 存储 +
 * MQTT 上报/补传 + RTC/NTP 时间戳 + OTA 升级/回滚 + 屏上配网。 */
#ifndef APP_VERSION
#define APP_VERSION      "V1.0"
#endif

/* Modbus RTU 主站：RS485 上挂的从站。
 * 换从站型号只改这三个值，寄存器地址和换算关系以从站手册为准。
 * HZ8706：地址 1、9600 8N1 无校验、0001H 温度 / 0002H 湿度，值除以 10。 */
#ifndef MODBUS_SLAVE_ADDR
#define MODBUS_SLAVE_ADDR   0x01u
#endif

#ifndef MODBUS_REG_START
#define MODBUS_REG_START    0x0001u
#endif

#ifndef MODBUS_REG_COUNT
#define MODBUS_REG_COUNT    2u
#endif

#endif /* __APP_CONFIG_H */
