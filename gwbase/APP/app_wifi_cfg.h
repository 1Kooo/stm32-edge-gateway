/**
 * @file    app_wifi_cfg.h
 * @brief   WiFi 凭据的掉电保存（外部 Flash A/B 双槽）。
 *
 * 为什么放外部 Flash：
 *   内部 Flash 的扇区已经被 ota 标志（Sector 2/3）和 APP 本体占满，
 *   再切一个扇区出来会把 APP 区切碎。外部 16MB 里 0x300000 那段
 *   （字库预留之后、日志区 0x400000 之前）没有任何人用。
 *
 * 为什么是 A/B 双槽：
 *   擦到一半掉电，另一槽还是完好的。上电取 seq 最大且 CRC 对得上的那份，
 *   等于自带掉电保护，不用额外的恢复流程。
 *
 * 线程约束（重要）：
 *   wifi_cfg_request() 任意任务可调，只写 RAM。
 *   wifi_cfg_save() / wifi_cfg_fallback() 只能在 net_task 里调 ——
 *   它们要擦写 SPI Flash，得和 storage_task 走同一把互斥。
 */

#ifndef __APP_WIFI_CFG_H
#define __APP_WIFI_CFG_H

#include <stdint.h>

#define WIFI_CFG_SSID_MAX   32u     /* 802.11 SSID 上限 */
#define WIFI_CFG_PASS_MAX   64u     /* WPA2 密码上限 */

typedef struct {
    char ssid[WIFI_CFG_SSID_MAX + 1u];   /* 带结尾 0 */
    char pass[WIFI_CFG_PASS_MAX + 1u];   /* 空串 = 开放网络 */
} wifi_cfg_t;

/**
 * @brief 开机初始化：扫描 A/B 两槽，把有效记录读进 RAM。
 * 调度器启动前调用（app_main 里），此时没有并发，不用加锁。
 */
void wifi_cfg_init(void);

/* 当前生效的凭据。永远返回有效指针，没配过就是编译期默认值。 */
const char *wifi_cfg_ssid(void);
const char *wifi_cfg_pass(void);

/** @return 1 = 用户配过且已存进 Flash，0 = 还在用 app_config_local.h 的默认值 */
uint8_t wifi_cfg_is_custom(void);

/**
 * @brief UI 侧调用：把新凭据放进待写缓冲并置标志，立即返回。
 *
 * 不碰 Flash。真正落盘由 net_task 做，原因有两个：
 *   1. LVGL 任务不该抢 SPI1，那是 storage_task / net_task 的地盘；
 *   2. 断链后正好在同一处用新凭据重连，逻辑集中。
 *
 * @return 0 = 已受理，负 = 参数非法（空 SSID / 超长）
 */
int wifi_cfg_request(const char *ssid, const char *pass);

/**
 * @brief net_task 侧调用：取走待写内容（取走即清标志）。
 * @return 1 = 取到了，0 = 没有待写内容
 */
int wifi_cfg_take_request(wifi_cfg_t *out);

/**
 * @brief 落盘：擦另一槽 + 写记录 + 回读校验，成功后更新 RAM 缓存。
 * @return 0 = 成功，负 = Flash 操作失败（此时 RAM 不变，调用方可让本次先临时生效）
 */
int wifi_cfg_save(const wifi_cfg_t *cfg);

/**
 * @brief 回退到另一槽那份旧配置，并以新序号重写，让下次开机也用它。
 *
 * 用途：新配的密码连不上 AP 时，设备不至于变成砖（现场没法捞）。
 * @return 0 = 已回退，负 = 没有可退的记录
 */
int wifi_cfg_fallback(void);

#endif /* __APP_WIFI_CFG_H */
