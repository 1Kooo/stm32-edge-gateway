/**
 * @file    data_model.h
 * @brief   The one and only payload format of this project.
 *
 * Sampling -> queue -> ring buffer -> MQTT -> cloud all use this struct.
 * Changing a field here means changing it everywhere - that is the point.
 *
 * Size is 16 bytes on purpose: it matches the flash record alignment and
 * keeps the sequence number trivially recoverable when scanning the log.
 */

#ifndef __DATA_MODEL_H
#define __DATA_MODEL_H

#include <stdint.h>

#define SENSOR_RECORD_SIZE   16u

typedef struct {
    uint32_t seq;        /* monotonic counter, the key for resuming after reboot */
    uint32_t ts;         /* Unix 秒：RTC/NTP 真实时间，RTC 降级时退回上电秒数 */
    int16_t  temp_x10;   /* temperature x10, 253 = 25.3 C                       */
    uint16_t humi_x10;   /* humidity x10,    450 = 45.0 %RH                     */
    uint8_t  valid;      /* 0 = sensor read failed this round                   */
    uint8_t  rsv[3];     /* padding / future flags                              */
} sensor_record_t;

/* =========================================================================
 *  On-flash record.
 *
 *  Same 16 bytes as sensor_record_t - the three spare bytes are reused as
 *  magic/state so that 4096/16 = 256 records fit one sector exactly. A
 *  20-byte header would make records straddle sector boundaries and force
 *  us to split writes across 256-byte pages.
 *
 *  state semantics follow the hardware: NOR flash can only turn 1 into 0,
 *  so "unsent" is the erased value and marking a record sent is a single
 *  0x00 byte written in place - no sector erase needed.
 * ========================================================================= */
#define LOG_MAGIC        0xA5u      /* record is valid          */
#define LOG_ST_UNSENT    0xFFu      /* erased value = not sent  */
#define LOG_ST_SENT      0x00u      /* marked sent, never erased*/
#define LOG_FMT_VER      0x04u      /* record layout version, stored in rsv
                                     * 0x04: ts 是 RTC Unix 时间戳；
                                     * 0x03 及以前 ts 是上电秒数，语义不同，
                                     * 升版本让老记录在扫描时被判无效 */

typedef struct {
    uint32_t seq;
    uint32_t ts;
    int16_t  temp_x10;
    uint16_t humi_x10;
    uint8_t  valid;
    uint8_t  magic;     /* LOG_MAGIC when written       */
    uint8_t  state;     /* LOG_ST_UNSENT / LOG_ST_SENT  */
    uint8_t  rsv;       /* LOG_FMT_VER - layout version */
} log_entry_t;

/* Fails the build if the layout ever drifts away from 16 bytes. */
typedef char log_entry_size_check[(sizeof(log_entry_t) == 16) ? 1 : -1];

#endif /* __DATA_MODEL_H */
