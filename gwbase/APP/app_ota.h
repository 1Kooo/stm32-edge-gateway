/**
 * @file    app_ota.h
 * @brief   OTA 下载端：从 PC 服务器拉新固件，写进外部 Flash 暂存区。
 *
 * 分工：
 *   本文件（APP 侧）  负责"把固件搬进来"——连服务器、拉块、写暂存区、置标志
 *   boot 侧           负责"把固件装上去"——校验、擦内部 Flash、搬运、切换
 *
 * 为什么要单独一个文件而不是塞进 net_task：
 *   协议收发、擦写 Flash、算 CRC 是完整的一件事，和 MQTT 上报没有半点关系。
 *   塞一起会让 net_task 那个本来就不短的循环更难读。
 *
 * 线程约束（很重要）：
 *   app_ota_run() 必须在 net_task 里调用，因为它要独占 USART3 那一条 TCP 链路。
 *   其它任务只能调 app_ota_request() 置个标志，由 net_task 下一轮自己执行。
 */

#ifndef __APP_OTA_H
#define __APP_OTA_H

#include <stdint.h>

/**
 * @brief 请求一次 OTA。任意任务都可以调，立即返回，不阻塞。
 *
 * 只是把标志置 1，真正的下载由 net_task 在下一轮循环里执行。
 */
void app_ota_request(void);

/**
 * @brief 取走一次待处理的 OTA 请求（取走即清零）。
 * @return 1 = 有请求，0 = 没有
 */
int app_ota_take_request(void);

/**
 * @brief 只查不取：请求还在不在。
 * net_task 内层 MQTT 循环用它发现请求后 break，
 * 真正取走留给外层 for(;;) 的 take_request。
 */
uint8_t app_ota_pending(void);

/**
 * @brief 是否有 OTA 正在进行中（给 UI 或其它任务查询用）。
 */
uint8_t app_ota_busy(void);

/**
 * @brief OTA 当前阶段（给 UI 显示进度条用）。仅当 app_ota_busy() 为 1 时有意义。
 * @return 1 = 准备（连服务器 + 擦外部 Flash）  2 = 下载
 */
uint8_t app_ota_phase(void);

/**
 * @brief OTA 当前进度百分比 0~100（给 UI 显示进度条用）。
 *        仅当 app_ota_busy() 为 1 时有意义。
 */
uint8_t app_ota_pct(void);

/**
 * @brief OTA 是否"真的成功"：下载完 + CRC 校验过 + 固件头写成功 + pending
 *        标志写成功，此刻几秒内必然复位。UI 看到 1 才能显示"升级完成"。
 */
uint8_t app_ota_done(void);

/**
 * @brief 最近一次 OTA 的失败码（= app_ota_run 的返回值）。0 = 无。
 *        仅当 busy()==0 且 done()==0 且 非 0 时表示"刚失败"。
 */
int8_t app_ota_err(void);

/**
 * @brief net_task 专用：app_ota_run 返回非 0 时把失败码转告 UI。
 *        失败出口在 app_ota_run 里有 8 个，net_task 的这一个 if 是唯一汇聚点。
 */
void app_ota_set_fail(int8_t code);

/**
 * @brief 目标固件版本（握手成功后从固件头取，高 16 位主版本 / 低 16 位次版本）。
 *        0 = 还没握手到、版本未知。
 */
uint32_t app_ota_ver(void);

/**
 * @brief 执行一次完整的下载。仅在 net_task 上下文调用。
 *
 * 调用前必须已经断开 MQTT（net_if_close），本函数自己会重新建立
 * 到 OTA 服务器的 TCP 连接；失败返回时链路状态是 DOWN，
 * 调用方按正常的重连流程处理即可。
 *
 * @return 0 = 成功（已置 pending 标志，调用方可以复位重启）
 *         -1 = 连不上服务器
 *         -2 = 服务器没回应 / 握手超时
 *         -3 = 固件头非法（magic 或 CRC 不对）
 *         -4 = 固件长度超出暂存区容量
 *         -5 = 擦除外部 Flash 失败
 *         -6 = 收数据块超时
 *         -7 = 写外部 Flash 失败
 *         -8 = 正文 CRC 与原厂声明不符（数据坏了）
 *         -9 = 写启动标志失败
 */
int app_ota_run(void);

#endif /* __APP_OTA_H */
