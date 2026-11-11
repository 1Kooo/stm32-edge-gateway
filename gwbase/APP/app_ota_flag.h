/**
 * @file    app_ota_flag.h
 * @brief   APP 侧的启动标志读写（与 boot/Src/ota_flag.h 一一对应）
 *
 * ⚠ 这里的 ota_flag_t 必须和 boot/Src/ota_flag.h 里的**一字不差**。
 *   两边是同一条 Flash 记录的两个读者，字段顺序或类型差一个，
 *   偏移就全错位，读出来是垃圾 —— 而且不报错，只是行为诡异。
 *   改结构体时两个文件一起改，两个工程一起重烧。
 *
 * APP 侧只做两件事：
 *   1. 读标志（判断自己是不是"刚装完、待确认"的新固件）
 *   2. 确认（把 swap_state 从 TRYING 改成 DONE，只写这一次）
 * 写标志的其它场合（建默认值、安装完切状态）都归 boot 管。
 */

#ifndef __APP_OTA_FLAG_H
#define __APP_OTA_FLAG_H

#include <stdint.h>

#define OTA_FLAG_A_ADDR   0x08008000u   /* 主份：扇区 2（16KB） */
#define OTA_FLAG_B_ADDR   0x0800C000u   /* 备份：扇区 3（16KB） */
#define OTA_FLAG_MAGIC    0x4641544Fu   /* 'O','T','A','F' 小端拼出来 */

/* 黄金备份区（外部 SPI Flash）：暂存区占 0~1MB，黄金区紧随其后。
 * 存裸正文（没有固件头），长度/CRC 记在标志的 golden_len/golden_crc。
 * 与 boot/Src/fw_image.h 里的 FW_GOLDEN_BASE 必须同值。 */
#define FW_GOLDEN_BASE    0x100000u

/* swap_state 的取值，和 boot 侧同一套 */
#define OTA_SWAP_IDLE     0u            /* 从没做过 OTA：正常启动 */
#define OTA_SWAP_TRYING   1u            /* 刚装完新固件，还没确认能跑 */
#define OTA_SWAP_DONE     2u            /* 新固件已确认，正常启动 */

#define OTA_BOOT_TRY_MAX  3u            /* 连续这么多次起不来就判失败 */

typedef struct {
    uint32_t magic;         /* 必须等于 OTA_FLAG_MAGIC */
    uint32_t seq;           /* 递增序号，越大越新 */
    uint32_t pending;       /* 1 = 外部暂存区有新固件待安装 */
    uint32_t fw_len;        /* 新固件字节数 */
    uint32_t fw_crc;        /* 新固件 CRC32 */
    uint32_t golden_len;    /* 黄金备份字节数，0 = 还没有备份 */
    uint32_t golden_crc;    /* 黄金备份 CRC32 */
    uint32_t boot_try;      /* 新固件已经尝试启动的次数 */
    uint32_t swap_state;    /* OTA_SWAP_IDLE / TRYING / DONE */
    uint32_t crc;           /* 上面所有字段的 CRC32（本字段不参与计算） */
} ota_flag_t;               /* 40 字节 */

/* 标准 CRC32（多项式 0x04C11DB7，初值 0xFFFFFFFF，末尾取反）。
 * 注意这是"高位优先"的非反射实现（CRC-32/BZIP2），
 * 和 PC 端 zlib.crc32 算出来的值完全不同，两边必须用同一套。 */
uint32_t ota_crc32(const void *data, uint32_t len);

/* 分段版：下载固件时边收边算，420KB 不可能先塞进 RAM。
 * 用法：crc = begin(); 反复 crc = update(crc, buf, n);  最后 end(crc) */
uint32_t ota_crc32_begin(void);
uint32_t ota_crc32_update(uint32_t crc, const void *data, uint32_t len);
uint32_t ota_crc32_end(uint32_t crc);

/* 读标志：两份都试，取有效且 seq 大的那份。0 = 读到，-1 = 两份都无效 */
int ota_flag_load(ota_flag_t *out);

/* 写标志：先算 CRC，再依次覆盖两份。0 = 主份写好，-1 = 主份写失败 */
int ota_flag_save(const ota_flag_t *in);

/* 填一份"干净"的默认值 */
void ota_flag_default(ota_flag_t *f);

/**
 * @brief  APP 启动成功的确认动作。
 *
 * 只在"刚装完新固件、待确认"时才会真正写 Flash；平时直接返回 0，
 * 一个字节都不碰 —— 内部 Flash 只有约 1 万次寿命，不能每次开机都擦写。
 *
 * @return  1 = 刚确认完（写了一次 Flash），
 *          0 = 无需确认（平时状态，没碰 Flash），
 *         -1 = 没有有效标志或写入失败
 */
int app_ota_confirm(void);

/**
 * @brief  把当前 APP 区固件固化成黄金备份（人工确认"这一版可用"）。
 *
 * 为什么是手动：自动备份会把"能开机但一操作就死"的固件也存成黄金，
 * 等真要回滚时回去的还是它。所以固化点由人决定（云端发 {"golden":1}）。
 *
 * 顺序是"先作废旧备份 -> 擦写 -> 校验 -> 才回写 len/crc"，
 * 中途掉电最坏结果是"没有备份"，不会留下指向空壳的假备份。
 *
 * 只能由 storage_task 调用（外部 Flash 的唯一写者），且 OTA 不能在跑。
 *
 * @return 0 = 成功，-1 = 失败
 */
int app_golden_promote(void);

/**
 * @brief  固化是否正在进行（1 = 正在擦写黄金区）。
 *
 * 固化要二十多秒，期间外部 Flash 不能被别的任务读写：一读就会和擦除
 * 操作交错在 SPI 上；去抢 Flash 互斥量则会阻塞到固化结束，心跳断供，
 * 看门狗反而把固化打断。所以 net_task / UI 在碰 Flash 前先问一句。
 */
uint8_t app_golden_busy(void);

/**
 * @brief  固化刚结束（3 秒内）：给 UI 补开结果弹层用。
 *
 * 固化由云端命令触发，屏上没有按键。而固化一结束 app_golden_busy() 立刻
 * 变 0，ui_task 往往正是这一刻才抢到 CPU —— 自动开弹层的条件已经失效。
 * UI 在 busy==0 之后再问一句这个，就能把"固化完成/失败"补显示出来。
 *
 * @return 1 = 刚固化完（3 秒内），0 = 没有
 */
uint8_t app_golden_just_done(void);

/**
 * @brief  固化进度，给屏上弹层用。
 * @return stage: 0=准备 1=擦除 2=搬运 3=校验 4=完成；pct: 0~100
 */
uint8_t app_golden_stage(void);
uint8_t app_golden_pct(void);

/**
 * @brief  上一次固化的结果：0=没做过/正在做，1=成功，2=失败。
 *         下次固化开始时清 0。
 */
uint8_t app_golden_result(void);

/**
 * @brief  黄金备份的字节数，0 = 还没有可信备份（关于页显示用）。
 */
uint32_t app_golden_len(void);

/**
 * @brief  算一次当前镜像的 CRC 并缓存（开机调一次，约 0.2 秒）。
 *         不调的话 app_golden_is_current() 永远返回 0。
 */
void app_golden_self_check(void);

/**
 * @brief  黄金备份是不是当前正在跑的这版：1 = 是，0 = 不是/未算/没备份。
 */
uint8_t app_golden_is_current(void);

#endif /* __APP_OTA_FLAG_H */
