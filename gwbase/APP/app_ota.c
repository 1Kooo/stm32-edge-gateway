/**
 * @file    app_ota.c
 * @brief   OTA 下载端：从 PC 服务器拉固件，写进外部 Flash 暂存区。
 *
 * 协议（拉取式，一块一确认）：
 *   ① MCU -> PC    "OTA?\r\n"
 *   ② PC  -> MCU   20 字节固件头（magic/version/fw_len/fw_crc/hdr_crc）
 *   ③ MCU 擦除外部 Flash 暂存区
 *   ④ 循环：MCU -> PC  "GET <偏移>\r\n"    PC -> MCU  最多 2048 字节
 *   ⑤ 收完算 CRC，对上才写头部、置 pending 标志
 *
 * 为什么是"拉"而不是"服务器一直推"：
 *   串口 115200，420KB 要传 35 秒以上，而外部 Flash 每擦一个 4KB 扇区会
 *   阻塞几十毫秒，这期间串口收不了数据。服务器猛推的话 ESP 内部缓冲
 *   （约 2KB）立刻溢出丢字节，CRC 永远对不上，而且丢在哪都查不出来。
 *   一问一答天然限速：要一块、收一块、写完再要下一块。
 *
 * 为什么由 MCU 发命令而不是服务器主动发：
 *   ESP-01S 跑在透传模式（CIPMODE=1），服务器发来的字节会原样落进串口，
 *   MCU 不主动开口就分不清收到的是数据还是应答。拉取模式下每个字节都在
 *   预期之内。
 */

#include <stdio.h>
#include <string.h>

#include "app_ota.h"
#include "app_ota_flag.h"
#include "app_config.h"
#include "app_wdg.h"        /* 下载期间给 net_task 打心跳，别被看门狗误判 */
#include "drv_spiflash.h"
#include "net_if.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* app_log 没有公共头文件，net_if_esp01s.c 也是这么声明的 */
extern void app_log(const char *fmt, ...);

/* ================= 暂存区布局 =================
 * 必须和 boot/Src/fw_image.h 一字不差：boot 按同样的偏移去读。
 *   0x000000  固件头 20 字节（整段前 256 字节预留）
 *   0x000100  固件正文
 */
#define FW_STAGE_BASE      0x000000u
#define FW_HDR_RESERVED    256u
#define FW_DATA_BASE       (FW_STAGE_BASE + FW_HDR_RESERVED)
#define FW_MAGIC           0x57465747u    /* 'G','W','F','W' */
#define FW_MAX_LEN         (1024u * 1024u - FW_HDR_RESERVED)

typedef struct {
    uint32_t magic;
    uint32_t version;       /* 高 16 位主版本，低 16 位次版本 */
    uint32_t fw_len;
    uint32_t fw_crc;
    uint32_t hdr_crc;       /* 前 16 字节的 CRC32 */
} fw_header_t;              /* 20 字节 */

/* 每次向服务器要多少字节。和服务端 ota_server.py 的 chunk_size 一致。 */
#define OTA_CHUNK          2048u

/* 每次从串口环形缓冲搬多少到 Flash。
 * 那个缓冲只有 512 字节（drv_uart.c:20），一次搬 2048 必然溢出，
 * 所以收到 256 就先落盘，腾出空间再接下一批。 */
#define OTA_READ_STEP      256u

/* 下载完是否自动复位。调试阶段置 0 可以停在原地看日志。 */
#define OTA_REBOOT_AFTER_OK   1

static uint8_t s_chunk[OTA_READ_STEP];   /* 收数缓冲，放静态区不占任务栈 */
static volatile uint8_t s_req  = 0;      /* 有 OTA 请求待处理 */
static volatile uint8_t s_busy = 0;      /* 正在下载，storage_task 别来抢 SPI */

/* OTA 进度：net_task 里写，ui_task 500ms 刷一次屏时读。
 * uint8_t 单字节，读写天然原子，不需要临界区。
 * 失败路径不用清零：UI 只在 app_ota_busy()==1 时才显示这些值，
 * 任何失败出口都会把 s_busy 清 0，屏幕自然回到正常显示。 */
static volatile uint8_t s_ota_phase = 0; /* 1 = 准备(连接+擦Flash)  2 = 下载 */
static volatile uint8_t s_ota_pct   = 0; /* 0~100 */

/* 弹窗用的结果状态：
 * s_ota_done 只在最后一步（CRC 过 + 头写成功 + pending 标志写成功）才置 1，
 * 此刻 300ms 后就复位 —— UI 看到 done=1 就敢写"升级完成"，不存在假成功。
 * s_ota_err 由 net_task 在 app_ota_run 返回非 0 时单点写入（十几个失败出口
 * 逐个加太噪，net_task 那一个 if 是唯一汇聚点）。 */
static volatile uint8_t  s_ota_done = 0;   /* 1 = 下载+校验+标志全过，马上复位 */
static volatile int8_t   s_ota_err  = 0;   /* app_ota_run 的失败返回码，0 = 无 */
static volatile uint32_t s_ota_ver  = 0u;  /* 目标固件版本（握手后从固件头取） */

/* ================= 请求标志 ================= */

void app_ota_request(void)
{
    s_req = 1;

    /* 发新请求时把上一次的失败码清掉：否则弹层重开后会先闪一下
     * 上一次的红字「升级失败」，等 net_task 真正开跑才恢复。 */
    s_ota_err = 0;
}

int app_ota_take_request(void)
{
    if (s_req == 0u) {
        return 0;
    }
    s_req = 0;
    return 1;
}

/* 只查不清。net_task 的内层 MQTT 循环用它发现请求后 break 出来，
 * 真正的"取走"必须留给外层 for(;;) 里的 take_request ——
 * 那边带完整的断链逻辑，如果这里就把标志清了，这次请求会被漏掉。 */
uint8_t app_ota_pending(void)
{
    return s_req;
}

uint8_t app_ota_busy(void)
{
    return s_busy;
}

uint8_t app_ota_phase(void)
{
    return s_ota_phase;
}

uint8_t app_ota_pct(void)
{
    return s_ota_pct;
}

uint8_t app_ota_done(void)
{
    return s_ota_done;
}

int8_t app_ota_err(void)
{
    return s_ota_err;
}

void app_ota_set_fail(int8_t code)
{
    s_ota_err = code;
}

uint32_t app_ota_ver(void)
{
    return s_ota_ver;
}

/* ================= 收数 ================= */

/* 收满 len 字节。透传模式下服务器发来的就是裸字节，没有任何封装。 */
static int recv_exact(uint8_t *buf, uint16_t len)
{
    uint16_t got = 0;
    uint32_t t0  = HAL_GetTick();

    while (got < len) {
        uint16_t n = net_if_recv(buf + got, (uint16_t)(len - got), 50);

        if (n > 0u) {
            got += n;
            t0 = HAL_GetTick();     /* 有进展就重置计时，避免大块被误判超时 */
            continue;
        }

        if ((HAL_GetTick() - t0) > 5000u) {
            return -1;              /* 5 秒一个字节都没进来，链路大概率断了 */
        }
    }
    return 0;
}

/* 从服务器要一块固件，边收边写进外部 Flash，顺手累加 CRC。
 * 边收边写是关键：等 2048 字节全进了 512 字节的环形缓冲，早就溢出了。 */
static int fetch_block(uint32_t fw_off, uint16_t len, uint32_t *crc)
{
    uint32_t got = 0;
    uint32_t t0  = HAL_GetTick();
    int      r;

    while (got < len) {
        uint16_t want = (uint16_t)((len - got) > OTA_READ_STEP
                                   ? OTA_READ_STEP : (len - got));
        uint16_t n = net_if_recv(s_chunk, want, 50);

        if (n == 0u) {
            if ((HAL_GetTick() - t0) > 5000u) {
                return -1;
            }
            continue;
        }

        t0 = HAL_GetTick();

        r = drv_flash_write(FW_DATA_BASE + fw_off + got, s_chunk, n);
        if (r != 0) {
            return -2;
        }

        *crc = ota_crc32_update(*crc, s_chunk, n);
        got += n;
    }
    return 0;
}

/* ================= 主流程 ================= */

int app_ota_run(void)
{
    fw_header_t hdr;
    ota_flag_t  flag;
    char        cmd[40];
    uint32_t    crc;
    uint32_t    sectors;
    uint32_t    i;
    uint32_t    off;
    uint32_t    chunk_cnt;
    uint16_t    len;
    int         r;

    s_busy = 1;
    s_ota_phase = 1;      /* 准备阶段：连服务器 + 擦外部 Flash */
    s_ota_pct   = 0;
    s_ota_done  = 0;      /* 清上一次的结果，弹窗从干净状态开始 */
    s_ota_err   = 0;

    app_log("\r\n[ota] ===== start =====\r\n");
    app_log("[ota] server %s:%u\r\n", OTA_HOST, (unsigned)OTA_PORT);

    /* 1. 连服务器。net_if_connect 内部会先确认 WiFi 已关联，
     *    再用 AT+CIPSTART 开新 socket。此时旧 MQTT socket 已被调用方关掉。 */
    r = net_if_connect(OTA_HOST, OTA_PORT, 20000);
    if (r != 0) {
        app_log("[ota] connect FAIL (%d)\r\n", r);
        s_busy = 0;
        return -1;
    }

    /* 2. 握手：要固件头 */
    net_if_flush_rx();
    net_if_send((const uint8_t *)"OTA?\r\n", 6u);

    r = recv_exact((uint8_t *)&hdr, (uint16_t)sizeof(hdr));
    if (r != 0) {
        app_log("[ota] no header (timeout)\r\n");
        net_if_close();
        s_busy = 0;
        return -2;
    }

    if (hdr.magic != FW_MAGIC) {
        app_log("[ota] bad magic 0x%08lX\r\n", (unsigned long)hdr.magic);
        net_if_close();
        s_busy = 0;
        return -3;
    }

    if (ota_crc32(&hdr, sizeof(hdr) - 4u) != hdr.hdr_crc) {
        app_log("[ota] header CRC mismatch (calc 0x%08lX, hdr 0x%08lX)\r\n",
                (unsigned long)ota_crc32(&hdr, sizeof(hdr) - 4u),
                (unsigned long)hdr.hdr_crc);
        net_if_close();
        s_busy = 0;
        return -3;
    }

    app_log("[ota] fw v%u.%u  len=%lu  crc=0x%08lX\r\n",
            (unsigned)(hdr.version >> 16), (unsigned)(hdr.version & 0xFFFFu),
            (unsigned long)hdr.fw_len, (unsigned long)hdr.fw_crc);
    s_ota_ver = hdr.version;   /* 给弹窗显示 "V0.6 -> V0.7" 用 */

    if (hdr.fw_len == 0u || hdr.fw_len > FW_MAX_LEN) {
        app_log("[ota] fw_len %lu out of range (max %lu)\r\n",
                (unsigned long)hdr.fw_len, (unsigned long)FW_MAX_LEN);
        net_if_close();
        s_busy = 0;
        return -4;
    }

    /* 3. 擦暂存区。头部预留区也要擦，所以从 FW_STAGE_BASE 起算。
     *    扇区数 = (256 + fw_len + 4095) / 4096：560KB 固件约 141 个，
     *    每扇区 30~100ms，总共 4~14 秒。 */
    sectors = (FW_HDR_RESERVED + hdr.fw_len + FLASH_SECTOR_SIZE - 1u)
              / FLASH_SECTOR_SIZE;

    app_log("[ota] erasing %lu sectors ...\r\n", (unsigned long)sectors);

    for (i = 0; i < sectors; i++) {
        app_wdg_kick(WDG_HB_NET);       /* 擦除总耗时 3~10 秒，必须打点 */
        r = drv_flash_erase_sector(FW_STAGE_BASE + i * FLASH_SECTOR_SIZE);
        if (r != 0) {
            app_log("[ota] erase FAIL at sector %lu (addr 0x%06lX)\r\n",
                    (unsigned long)i,
                    (unsigned long)(FW_STAGE_BASE + i * FLASH_SECTOR_SIZE));
            net_if_close();
            s_busy = 0;
            return -5;
        }

        if ((i % 20u) == 19u) {
            app_log("[ota] erase %lu/%lu\r\n",
                    (unsigned long)(i + 1u), (unsigned long)sectors);
        }

        /* 擦除进度也给 UI：一扇区一更新，开销就是一次写单字节 */
        s_ota_pct = (uint8_t)((i + 1u) * 100u / sectors);
    }

    /* 4. 逐块拉取 */
    crc       = ota_crc32_begin();
    chunk_cnt = (hdr.fw_len + OTA_CHUNK - 1u) / OTA_CHUNK;

    app_log("[ota] downloading %lu chunks ...\r\n", (unsigned long)chunk_cnt);

    for (i = 0; i < chunk_cnt; i++) {
        /* 整个下载在这里一口气跑完（实测 60 秒上下），
         * 不打点的话 wdg_task 会把这当成 net_task 停滞，装到一半就被复位。 */
        app_wdg_kick(WDG_HB_NET);

        off = i * OTA_CHUNK;
        len = (uint16_t)((hdr.fw_len - off) > OTA_CHUNK
                         ? OTA_CHUNK : (hdr.fw_len - off));

        snprintf(cmd, sizeof(cmd), "GET %lu\r\n", (unsigned long)off);
        if (net_if_send((const uint8_t *)cmd, (uint16_t)strlen(cmd)) != 0) {
            app_log("[ota] send GET %lu FAIL\r\n", (unsigned long)off);
            net_if_close();
            s_busy = 0;
            return -6;
        }

        r = fetch_block(off, len, &crc);
        if (r == -1) {
            app_log("[ota] recv timeout at offset %lu\r\n", (unsigned long)off);
            net_if_close();
            s_busy = 0;
            return -6;
        }
        if (r == -2) {
            app_log("[ota] flash write FAIL at offset %lu\r\n", (unsigned long)off);
            net_if_close();
            s_busy = 0;
            return -7;
        }

        /* 每 20 块（约 40KB）打一次，不刷屏 */
        if (((i + 1u) % 20u) == 0u || (i + 1u) == chunk_cnt) {
            app_log("[ota] %lu/%lu  (%lu%%)\r\n",
                    (unsigned long)(off + len), (unsigned long)hdr.fw_len,
                    (unsigned long)((off + len) * 100u / hdr.fw_len));
        }

        /* 下载进度给 UI：和上面日志同一个公式，白拿 */
        s_ota_phase = 2;
        s_ota_pct   = (uint8_t)((off + len) * 100u / hdr.fw_len);
    }

    crc = ota_crc32_end(crc);

    /* 5. 校验。这一步不过就一个字都别往标志区写 ——
     *    暂存区留着一堆坏数据没关系，boot 看 magic 不对就不会装。 */
    app_log("[ota] crc calc 0x%08lX / expect 0x%08lX\r\n",
            (unsigned long)crc, (unsigned long)hdr.fw_crc);

    if (crc != hdr.fw_crc) {
        app_log("[ota] CRC MISMATCH, discard\r\n");
        net_if_close();
        s_busy = 0;
        return -8;
    }

    /* 6. 写头部。放在最后：中途断电时头部还不存在，
     *    boot 读到的 magic 是全 0xFF，当成"没有新固件"，绝不会装半截。 */
    r = drv_flash_write(FW_STAGE_BASE, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));
    if (r != 0) {
        app_log("[ota] write header FAIL\r\n");
        net_if_close();
        s_busy = 0;
        return -7;
    }

    /* 7. 置 pending：告诉 boot"暂存区有一份完整的新固件，下次启动装它"。
     *    swap_state 保持原值 —— 从现在到 boot 搬完之前，正在跑的还是老固件。 */
    if (ota_flag_load(&flag) != 0) {
        ota_flag_default(&flag);        /* 理论上 boot 已经建过，这里兜底 */
    }

    flag.pending = 1u;
    flag.fw_len  = hdr.fw_len;
    flag.fw_crc  = hdr.fw_crc;
    flag.seq++;

    if (ota_flag_save(&flag) != 0) {
        app_log("[ota] save flag FAIL\r\n");
        net_if_close();
        s_busy = 0;
        return -9;
    }

    app_log("[ota] ===== OK, %lu bytes staged =====\r\n",
            (unsigned long)hdr.fw_len);

    s_ota_done = 1;    /* 到这里 = CRC 过 + 头写成功 + pending 已置，板上重启倒计时 */

    net_if_close();
    s_busy = 0;

#if OTA_REBOOT_AFTER_OK
    app_log("[ota] rebooting to install ...\r\n");
    vTaskDelay(pdMS_TO_TICKS(300));     /* 让最后几行日志吐完 */
    NVIC_SystemReset();
#endif

    return 0;
}
