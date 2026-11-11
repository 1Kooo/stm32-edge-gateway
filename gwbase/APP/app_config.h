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
 * MQTT 上报/补传 + RTC/NTP 时间戳 + OTA 升级/回滚 + 屏上配网。
 * V1.1 = V1.0 + 时间可信度显示（未校准显示 --:--）+ 独立看门狗
 * （心跳门控喂狗：五个任务都推进才喂，任一停滞由 IWDG 复位）。
 * V1.2 = V1.1 + MQTT 下行：订阅主题、解析云端 PUBLISH，可在线改
 * 485 轮询周期与上报周期（RAM 生效，断电回默认）。
 * V1.3 = V1.2 + 参数区（外部 Flash A/B 双备份，周期与告警阈值断电保留）
 * + 告警阈值判定：越限大数字变色、上报带 alm 字段。
 * V1.5 = V1.3 + 日志区写满回绕（全发完才回收，有积压则停记）
 * + 关于页「存储」状态行 + 告警事件化（ev 字段：开始/提醒/恢复）
 * + 远程 {"reboot":1} / {"reset":1}。
 * V1.6 = V1.5 + 系统事件日志（外部 Flash 0x702000 起 64KB 环形，
 * 断电留痕）与第 4 页「事件」列表，界面由三页扩为四页。
 * V1.7 = V1.6 + 黄金备份改为手动固化（云端 {"golden":1}，OTA 确认不再自动搬，
 * 避免"能开机但一操作就死"的固件顶掉真正好用的备份）；IWDG 提前到启动最早期，
 * 启动阶段卡死 30 秒自动复位，OTA 新固件因此能累加 boot_try 触发回滚。
 * （V1.4 是息屏实验版，因背光未接通已回退，未作为正式版本发布）
 *
 * V2.0 = 定版。V1.7 之后只做了缺陷修复，没有新增功能，故不逐个占小版本号，
 * 统一收在 V2.0 作为功能完整的交付版：
 *   1. MQTT PUBLISH 解析 remlen-2-tlen 无符号下溢 → memcpy 越界读，
 *      表现为串口乱码、流错位后报文永远判"没收全"、ping FAIL 直到 link dead；
 *   2. 黄金固化期间 storage_task 不让出 CPU + busy 标志转瞬即逝，
 *      导致弹窗永不出现、看门狗误判 net_task 停滞而复位；
 *   3. 事件区扫描缓冲游标跨块累加，越界踩 s_scan_ssid_ui 与 lv_global；
 *   4. 黄金镜像长度用 ER_IROM1$$Limit 计算，漏掉 RW 初值压缩副本；
 *   5. OTA 首块前板端擦 141 扇区（6~14 秒）超过服务端 10 秒读超时被掐断；
 *   6. 绘制缓冲 6 行 → 12 行（分块数 36 → 18，全屏重绘的固定开销减半）；
 *   7. 事件页对"时间不可信"的记录显示 --:--:--（原来会把编译时刻当成
 *      真实上电时间显示）；
 *   8. 移除未参与编译、零引用的 mpu6050/eMPL 姿态解算库（5774 行）。 */
#ifndef APP_VERSION
#define APP_VERSION      "V2.0"
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
