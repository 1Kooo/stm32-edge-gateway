/**
 * @file    app_event.c
 * @brief   事件日志的落盘与读取。
 *
 * 区布局：外部 Flash 0x00702000 起 64KB，紧跟参数区之后。
 * 每条 8 字节，共 8192 条，写满就从头部覆盖最老的 —— 事件允许丢，
 * 屏上只看最近几十条，这点和数据日志（一条都不能丢）完全不同。
 */

#include "app_event.h"
#include "drv_spiflash.h"
#include "drv_rtc.h"

#include <stdio.h>          /* snprintf：事件参数文字 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define EVT_BASE        0x00702000u
#define EVT_REC_SIZE    8u
#define EVT_COUNT       (65536u / EVT_REC_SIZE)     /* 8192 条 */
#define EVT_MAGIC       0xE7u
#define EVT_VER         0x01u
#define EVT_QUEUE_LEN   16u

/* 落盘记录：比 evt_t 多两个字节用于识别"这里写过东西" */
typedef struct {
    uint32_t ts;
    uint8_t  code;
    uint8_t  arg;
    uint8_t  magic;
    uint8_t  ver;
} evt_rec_t;

static QueueHandle_t s_q = NULL;
static uint32_t      s_widx;        /* 下一条要写的位置（环内下标） */
static uint32_t      s_written;     /* 累计写入条数，UI 用它判断要不要重载 */

/* 扫描缓冲：一次读 512 字节（64 条），8192 条只要 128 次读 */
static uint32_t s_scan[128];

/* 最近一次固化时间（开机扫描时填），0 = 没固化过 */
static uint32_t s_golden_ts = 0u;

void app_event_init(void)
{
    uint32_t i;
    uint32_t n;

    if (s_q == NULL) {
        s_q = xQueueCreate(EVT_QUEUE_LEN, sizeof(evt_t));
    }

    /* 找第一个"不是事件"的位置 = 写指针。全满就从头覆盖 */
    s_widx    = 0u;
    s_written = 0u;
    s_golden_ts = 0u;

    /* 分块读（64 条 = 512 字节，正好是一个块）。每块都从缓冲头部开始解析，
     * 指针绝不跨块累加 —— 早期版本把读指针一路往上推，第二块起就写到
     * s_scan 之外，把相邻的 .bss（WiFi 凭据、LVGL 全局状态）踩成 Flash 里的
     * 原始数据。表现为：SSID 显示成乱码、lv_init 报 already initialized 卡自检页。 */
    for (i = 0u; i < EVT_COUNT; ) {
        uint32_t          left = EVT_COUNT - i;
        const evt_rec_t  *r    = (const evt_rec_t *)s_scan;
        uint32_t          k;

        n = (left > 64u) ? 64u : left;
        if (drv_flash_read(EVT_BASE + i * EVT_REC_SIZE, (uint8_t *)s_scan,
                           n * EVT_REC_SIZE) != 0) {
            break;
        }

        for (k = 0u; k < n; k++) {
            if (r[k].magic != EVT_MAGIC || r[k].ver != EVT_VER) {
                s_widx    = i + k;
                s_written = i + k;
                return;
            }

            /* 顺手记下最近一次固化时间：这一遍本来就要把所有记录读一遍，
             * 单独再扫一次纯属浪费。 */
            if (r[k].code == EVT_GOLDEN) {
                s_golden_ts = r[k].ts;
            }
        }

        i        += n;
        s_written = i;
    }

    s_widx    = 0u;             /* 区满：从头开始覆盖最老的 */
    s_written = EVT_COUNT;
}

void app_event_log(uint8_t code, uint8_t arg)
{
    evt_t e;

    if (s_q == NULL) return;    /* init 之前（理论上不会）直接丢 */

    /* 时间不可信就记 0，别记一个"看着合法的假时间"。
     *
     * 电池丢失时 RTC 会被写成编译时刻 __TIME__，比如 22:15 —— 这个值
     * 比任何"早于某年就算无效"的阈值都新，用数值范围根本拦不住它，
     * 屏上就会把 11-11 22:15 当成真实上电时间显示出来。
     * 能判断"这段时间到底准不准"的只有 synced 标记（备份寄存器里的
     * magic，断电才丢），所以只认它。RTC 降级（LSE 起不来）时
     * drv_rtc_unix 返回的是上电秒数，同样没有意义，一并记 0。
     *
     * 0 在事件页显示成 --:--:--，落盘时也不会被 flush 的兜底改掉。 */
    if (drv_rtc_ok() != 0u && drv_rtc_synced() != 0u) {
        e.ts = drv_rtc_unix();
    } else {
        e.ts = 0u;
    }
    e.code = code;
    e.arg  = arg;

    /* 不阻塞业务：事件丢了就丢了，绝不能因为写日志把采集拖住 */
    (void)xQueueSend(s_q, &e, 0);
}

void app_event_flush(void)
{
    evt_t e;

    if (s_q == NULL) return;

    while (xQueueReceive(s_q, &e, 0) == pdTRUE) {
        evt_rec_t r;
        uint32_t  addr = EVT_BASE + s_widx * EVT_REC_SIZE;
        uint32_t  per_sector = FLASH_SECTOR_SIZE / EVT_REC_SIZE;

        /* 进新扇区先擦：这个扇区里是最老的事件，本来就该被覆盖 */
        if ((s_widx % per_sector) == 0u) {
            (void)drv_flash_erase_sector(addr);
        }

        r.ts    = e.ts;
        /* 兜底只处理"非 0 的异常小值"（RTC 刚起振读到临界值之类）。
         * 0 是"当时时间不可信"的明确标记，必须原样保留 —— 补成当前时间
         * 就等于把假时间又写回去了，屏上照样会显示一个看着合法的时刻。
         * 不改的话，下面这个 1700000000 的阈值也拦不住编译时刻
         * （2026 年的编译时刻已经有 17.9 亿秒）。 */
        if (r.ts != 0u && r.ts < 1700000000u) r.ts = drv_rtc_unix();
        r.code  = e.code;
        r.arg   = e.arg;
        r.magic = EVT_MAGIC;
        r.ver   = EVT_VER;

        if (drv_flash_write(addr, (const uint8_t *)&r, EVT_REC_SIZE) != 0) {
            break;              /* 写失败就停手，剩下的下次再试，别死循环 */
        }

        s_widx = (s_widx + 1u) % EVT_COUNT;
        s_written++;

        /* 固化事件落盘后立刻更新缓存：不然关于页要等到下次开机
         * 才显示得出这个时间（缓存本来只在开机扫描时填）。 */
        if (r.code == EVT_GOLDEN) {
            s_golden_ts = r.ts;
        }
    }
}

uint32_t app_event_seq(void)
{
    return s_written;
}

int app_event_recent(evt_t *buf, int max)
{
    evt_rec_t r;
    uint32_t  i;
    int       n = 0;

    if (buf == NULL || max <= 0 || s_written == 0u) return 0;

    i = (s_widx + EVT_COUNT - 1u) % EVT_COUNT;   /* 最新一条 */

    while (n < max) {
        if (drv_flash_read(EVT_BASE + i * EVT_REC_SIZE, (uint8_t *)&r, EVT_REC_SIZE) != 0) {
            break;
        }
        if (r.magic != EVT_MAGIC || r.ver != EVT_VER) {
            break;                               /* 撞到空位，到头了 */
        }

        buf[n].ts   = r.ts;
        buf[n].code = r.code;
        buf[n].arg  = r.arg;
        n++;

        if ((uint32_t)n >= s_written) break;     /* 只读真正写过的条数 */
        i = (i + EVT_COUNT - 1u) % EVT_COUNT;
    }

    return n;
}

uint32_t app_event_last_golden_ts(void)
{
    return s_golden_ts;
}

const char *app_event_name(uint8_t code)
{
    switch (code) {
    case EVT_POWER_ON:    return "上电";
    case EVT_NET_UP:      return "联网成功";
    case EVT_NET_DOWN:    return "网络断开";
    case EVT_NTP_OK:      return "对时成功";
    case EVT_NTP_FAIL:    return "对时失败";
    case EVT_PARAM_CHG:   return "参数变更";
    case EVT_FACTORY:     return "恢复出厂";
    case EVT_REBOOT:      return "远程重启";
    case EVT_OTA_START:   return "OTA 开始";
    case EVT_OTA_OK:      return "OTA 完成";
    case EVT_OTA_FAIL:    return "OTA 失败";
    case EVT_ALARM_ON:    return "告警开始";
    case EVT_ALARM_OFF:   return "告警恢复";
    case EVT_STORE_FULL:  return "存储停记";
    case EVT_STORE_WRAP:  return "存储回绕";
    case EVT_GOLDEN:      return "已固化";
    default:              return "未知";
    }
}

uint8_t app_event_is_warn(uint8_t code)
{
    switch (code) {
    case EVT_NET_DOWN:
    case EVT_NTP_FAIL:
    case EVT_OTA_FAIL:
    case EVT_STORE_FULL:
    case EVT_ALARM_ON:
        return 1u;
    default:
        return 0u;
    }
}

const char *app_event_arg_text(uint8_t code, uint8_t arg, char *buf, int bufsz)
{
    if (buf == NULL || bufsz <= 0) return NULL;

    switch (code) {
    case EVT_NTP_FAIL:
        snprintf(buf, (size_t)bufsz, "第 %u 次", (unsigned)arg);
        return buf;

    case EVT_OTA_FAIL:
        /* app_ota_run 的返回码是负数，这里按原样显示，方便直接对代码 */
        snprintf(buf, (size_t)bufsz, "err -%u", (unsigned)arg);
        return buf;

    case EVT_ALARM_ON:
        if (arg == 1u)      return "温度";
        if (arg == 2u)      return "湿度";
        if (arg == 3u)      return "温度+湿度";
        return NULL;

    case EVT_NET_DOWN:
        if (arg == 1u)      return "心跳超时";
        if (arg == 2u)      return "发送失败";
        return NULL;

    default:
        return NULL;        /* 其它事件没有参数 */
    }
}
