/**
 * @file    drv_rtc.c
 * @brief   板载 RTC 驱动实现。
 *
 * 硬件：PC14/PC15 接 32.768kHz 晶振（LSE），BAT1 纽扣电池挂在 VBAT。
 * RTC 和备份寄存器都在"备份域"里 —— 主电源断电后由电池接管，
 * 时间和"时间有效"标记都不会丢。
 *
 * 设计取舍：
 *   - 默认时间用编译时刻：电池在就基本不用它，电池不在也能退回编译时刻
 *     而不是 1970。缺点是起点天然落后"编译到上板"那几分钟。
 *   - 联网后由 net_task 问一次 NTP（net_if_ntp），拿到真实时间用
 *     drv_rtc_set_unix 写进来，把编译时刻的偏差抹掉。
 *   - LSE 晶振起不来（没焊 / 坏）就降级：ts 返回上电秒数，
 *     行为和没有 RTC 的老版本一致，开机日志会明说。
 */

#include "drv_rtc.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define RTC_BKP_MAGIC   0x32F1u     /* 写在 BKP0R：值还在 = 时间没断过 */

static RTC_HandleTypeDef s_rtc;    /* HAL 句柄，只有本文件用 */
static uint8_t           s_rtc_ok = 0;   /* 1 = RTC 在跑真实时间；0 = 降级用上电秒数 */

/* 备份域是否已经有有效时间：RTC 使能 + LSE 在跑 + magic 还在。
 * 断电丢电池时这三样会一起清零，不会出现"magic 在但时钟没走"的假象。 */
static uint8_t rtc_time_retained(void)
{
    uint32_t bdcr = RCC->BDCR;

    if ((bdcr & RCC_BDCR_RTCEN) == 0u) return 0;
    if ((bdcr & RCC_BDCR_LSERDY) == 0u) return 0;
    if (HAL_RTCEx_BKUPRead(&s_rtc, RTC_BKP_DR0) != RTC_BKP_MAGIC) return 0;
    return 1;
}

/* 民用日期 -> 自 1970-01-01 的天数（Howard Hinnant 算法）。
 * 只在 drv_rtc_unix 里用一次，但单独放一个函数是值得的：
 * 这是纯数学，和硬件读数解耦，验证时可以拿已知日期手算对答案。 */
static long days_from_civil(int y, int m, int d)
{
    long era;
    unsigned yoe;
    unsigned doy;
    unsigned doe;

    y -= (m <= 2);
    era = ((y >= 0) ? ((long)y) : ((long)y - 399L)) / 400L;
    yoe = (unsigned)((long)y - era * 400L);
    doy = (unsigned)((153u * (unsigned)(m + ((m > 2) ? -3 : 9)) + 2u) / 5u
                     + (unsigned)d - 1u);
    doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097L + (long)doe - 719468L;
}

/* 把编译器内置宏 __DATE__ ("Nov  9 2026") / __TIME__ ("20:15:30")
 * 解析进 RTC。注意 __DATE__ 的日期是空格补位的两位，先用 atoi 再截。 */
static void rtc_set_build_time(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    char mon[4];
    int  day, year, hour, min, sec;
    int  mi;
    int  mon_num = 0;
    static const char *k_months[12] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    sscanf(__DATE__, "%3s %d %d", mon, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec);

    for (mi = 0; mi < 12; mi++) {
        if (strcmp(mon, k_months[mi]) == 0) {
            mon_num = mi + 1;
            break;
        }
    }
    if (mon_num == 0) {         /* 宏格式意外，兜底到 2026-01-01 00:00:00 */
        mon_num = 1; year = 2026; day = 1; hour = 0; min = 0; sec = 0;
    }

    t.Hours   = (uint8_t)hour;
    t.Minutes = (uint8_t)min;
    t.Seconds = (uint8_t)sec;
    t.TimeFormat      = 0;
    t.DayLightSaving  = RTC_DAYLIGHTSAVING_NONE;
    t.StoreOperation  = RTC_STOREOPERATION_RESET;

    d.WeekDay = RTC_WEEKDAY_MONDAY;     /* 星期不用，随便填个合法值 */
    d.Month   = (uint8_t)mon_num;
    d.Date    = (uint8_t)day;
    d.Year    = (uint8_t)(year - 2000);

    HAL_RTC_SetTime(&s_rtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&s_rtc, &d, RTC_FORMAT_BIN);
}

void drv_rtc_init(void)
{
    uint32_t start;
    uint8_t  retained;

    /* 备份域上锁着，写备份寄存器 / 改 RTC 前必须先解锁 */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    s_rtc.Instance               = RTC;
    s_rtc.Init.HourFormat        = RTC_HOURFORMAT_24;
    s_rtc.Init.AsynchPrediv      = 127u;     /* 32768/128/256 = 1Hz */
    s_rtc.Init.SynchPrediv       = 255u;
    s_rtc.Init.OutPut            = RTC_OUTPUT_DISABLE;

    retained = rtc_time_retained();

    if (!retained) {
        /* 首次上电或电池失效：开 LSE 并等它起振（最长 3 秒）。
         * 起振慢是石英晶振的物理特性，1~2 秒很正常。 */
        __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
        start = HAL_GetTick();
        while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0u) {
            if ((HAL_GetTick() - start) > 3000u) {
                printf("[rtc] LSE not ready, ts = uptime fallback\r\n");
                s_rtc_ok = 0;
                return;
            }
        }

        __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
        __HAL_RCC_RTC_ENABLE();
        if (HAL_RTC_Init(&s_rtc) != HAL_OK) {
            printf("[rtc] init FAIL, ts = uptime fallback\r\n");
            s_rtc_ok = 0;
            return;
        }

        rtc_set_build_time();
        HAL_RTCEx_BKUPWrite(&s_rtc, RTC_BKP_DR0, RTC_BKP_MAGIC);
        printf("[rtc] time set to build time (first boot or battery lost)\r\n");
    } else {
        /* 电池把时间和配置都保住了：一行都不用写，直接用 */
        printf("[rtc] time ok (battery kept)\r\n");
    }

    s_rtc_ok = 1;
}

uint32_t drv_rtc_unix(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    long     days;
    uint32_t y;
    unsigned m;
    unsigned dd;

    if (s_rtc_ok == 0u) {
        /* 降级：保持老行为（上电秒数），云端只是拿不到真实时间而已 */
        return (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
    }

    /* HAL 要求 GetTime 之后必须跟一个 GetDate，否则影子寄存器锁住，
     * 下次读到的秒不会刷新 —— 少这一行表现为"秒数永远不变"。 */
    HAL_RTC_GetTime(&s_rtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&s_rtc, &d, RTC_FORMAT_BIN);

    y  = 2000u + (uint32_t)d.Year;
    m  = (unsigned)d.Month;
    dd = (unsigned)d.Date;

    /* 备份域读数异常（写坏 / 电池临界）时不给垃圾时间 */
    if (d.Year > 99u || m < 1u || m > 12u || dd < 1u || dd > 31u) {
        return 0u;
    }

    days = days_from_civil((int)y, (int)m, (int)dd);

    /* RTC 存的是本地时间，转成 UTC 时间戳要减掉时区偏移。
     * 云端按 Unix 时间落库，不补偿就会整体偏 8 小时。 */
    {
        long local = days * 86400L
                     + (long)t.Hours * 3600L
                     + (long)t.Minutes * 60L
                     + (long)t.Seconds;

        local -= RTC_TZ_OFFSET_SEC;
        return (local > 0L) ? (uint32_t)local : 0u;
    }
}

uint8_t drv_rtc_ok(void)
{
    return s_rtc_ok;
}

int drv_rtc_set_unix(uint32_t ts)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    uint16_t y;
    uint8_t  mo, dd, h, mi, s;

    if (s_rtc_ok == 0u || ts == 0u) {
        return -1;       /* RTC 没在跑，写进去也没人走 */
    }

    /* RTC 存的是本地时间：UTC 先加回时区偏移再拆年月日时分秒
     * （drv_rtc_split 内部已经做了，正好和读出时减掉的凑成一对） */
    drv_rtc_split(ts, &y, &mo, &dd, &h, &mi, &s);

    t.Hours          = h;
    t.Minutes        = mi;
    t.Seconds        = s;
    t.TimeFormat     = 0;
    t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    t.StoreOperation = RTC_STOREOPERATION_RESET;

    d.WeekDay = RTC_WEEKDAY_MONDAY;     /* 星期不用，合法值即可 */
    d.Month   = mo;
    d.Date    = dd;
    d.Year    = (uint8_t)(y - 2000u);

    HAL_RTC_SetTime(&s_rtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&s_rtc, &d, RTC_FORMAT_BIN);

    /* 电池断电再来时靠这个 magic 判断"时间没断过" */
    HAL_RTCEx_BKUPWrite(&s_rtc, RTC_BKP_DR0, RTC_BKP_MAGIC);

    printf("[rtc] synced to %04u-%02u-%02u %02u:%02u:%02u\r\n",
           (unsigned)y, (unsigned)mo, (unsigned)dd,
           (unsigned)h, (unsigned)mi, (unsigned)s);
    return 0;
}

/* 天数 -> 民用日期（days_from_civil 的逆运算，同一套 Hinnant 算法）。
 * 屏幕要显示 "2026-11-09"，只有 Unix 秒不够，必须反算回年月日。 */
static void civil_from_days(long z, int *y, int *m, int *d)
{
    long     era;
    unsigned doe;
    unsigned yoe;
    unsigned doy;
    unsigned mp;
    int      yy;

    z += 719468L;
    era = ((z >= 0) ? z : (z - 146096L)) / 146097L;
    doe = (unsigned)(z - era * 146097L);
    yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    yy  = (int)yoe + (int)(era * 400L);
    doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    mp  = (5u * doy + 2u) / 153u;

    *d = (int)(doy - (153u * mp + 2u) / 5u + 1u);
    *m = (int)mp + (((int)mp < 10) ? 3 : -9);
    yy += (*m <= 2);
    *y  = yy;
}

void drv_rtc_split(uint32_t ts, uint16_t *year, uint8_t *mon, uint8_t *day,
                   uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    /* 传进来是 UTC，显示要本地时间：先加回时区偏移 */
    uint32_t local = ts + (uint32_t)RTC_TZ_OFFSET_SEC;
    long     days  = (long)(local / 86400u);
    long     rem   = (long)(local % 86400u);
    int      y, m, d;

    civil_from_days(days, &y, &m, &d);

    *year = (uint16_t)y;
    *mon  = (uint8_t)m;
    *day  = (uint8_t)d;
    *hour = (uint8_t)(rem / 3600L);
    *min  = (uint8_t)((rem / 60L) % 60L);
    *sec  = (uint8_t)(rem % 60L);
}
