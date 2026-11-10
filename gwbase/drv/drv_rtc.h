/**
 * @file    drv_rtc.h
 * @brief   板载 RTC 驱动（LSE 32.768kHz 晶振 + VBAT 纽扣电池）。
 *
 * 作用：把"真实世界时间"交给数据链路 —— sensor_record_t.ts 从
 * "上电后第几秒"升级成 Unix 时间戳，掉电不归零。
 */

#ifndef __DRV_RTC_H
#define __DRV_RTC_H

#include <stdint.h>

/* 时区偏移（秒）。RTC 寄存器里存的是本地时间，Unix 时间戳必须是 UTC：
 * 读出来要减 8 小时才是真正的 epoch，屏上显示又要加回去。
 * 换时区或做夏令时，只改这一个宏。 */
#define RTC_TZ_OFFSET_SEC   (8L * 3600L)

/* 上电初始化一次：配置 LSE 和 RTC，判断时间是否被电池保住 */
void drv_rtc_init(void);

/* Unix 时间戳（UTC 秒）。LSE 起不来时降级返回上电秒数，不会比没 RTC 更差 */
uint32_t drv_rtc_unix(void);

/* 网络对时用：把 UTC Unix 秒写进 RTC（内部换算回本地时区再写）。
 * 返回 0 成功；-1 = RTC 没在跑（LSE 起不来）或参数无效，写进去也不会走 */
int drv_rtc_set_unix(uint32_t ts);

/* UTC 时间戳 -> 本地时间（年月日时分秒），给屏幕显示用。
 * 纯数学换算，不碰硬件。 */
void drv_rtc_split(uint32_t ts, uint16_t *year, uint8_t *mon, uint8_t *day,
                   uint8_t *hour, uint8_t *min, uint8_t *sec);

/* 1 = RTC 在跑真实时间；0 = 降级态（上电秒数） */
uint8_t drv_rtc_ok(void);

/* NTP 对时成功后调一次：写备份寄存器里的"已校准"标记（跨断电保留） */
void drv_rtc_mark_synced(void);

/* 1 = 当前时间经过校准（或电池保住了校准标记），可以当真时间显示；
 * 0 = 只是编译时刻兜底值，界面应显示 --:-- 而不是这个假时间 */
uint8_t drv_rtc_synced(void);

#endif /* __DRV_RTC_H */
