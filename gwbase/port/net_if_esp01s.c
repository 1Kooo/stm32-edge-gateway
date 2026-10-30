/**
 * @file    net_if_esp01s.c
 * @brief   net_if implementation for ESP-01S (ESP-AT firmware).
 *
 * Design notes:
 *  - every AT exchange is "send, then wait for OK / ERROR / expected token /
 *    timeout" instead of matching one exact reply: AT firmware versions differ
 *    in their replies and hard matching breaks on the next version
 *  - passthrough mode (CIPMODE=1) is used, so after CIPSEND everything written
 *    to USART3 goes straight to the socket and everything read comes from it
 */

#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "net_if.h"
#include "drv_uart.h"
#include "app_config.h"
#include "bsp_pinmap.h"

#define AT_LINE_MAX      128u
#define AT_REPLY_MS      200u

/* 日志统一走 app 层的互斥封装。这里直接用 printf 的话，net_task 的输出
 * 会和 beat/storage 等任务的 app_log 交错，出错时 ESP 的原始回复会被
 * 切成两半，反而看不到关键信息。app_log 没有公共头文件，就地声明。 */
extern void app_log(const char *fmt, ...);

static net_state_t s_state = NET_DOWN;
static char        s_line[AT_LINE_MAX];

/* ---------------------------------------------------------------- helpers */
/* Read one line (terminated by '\n'). Returns length, or -1 on timeout. */
static int at_read_line(char *out, uint16_t out_size, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t n  = 0;
    uint8_t  ch;

    out[0] = '\0';

    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (drv_uart_read(UART_WIFI, &ch, 1u) == 0u) {
            vTaskDelay(1);
            continue;
        }
        if (n + 1u < out_size) {
            out[n++] = (char)ch;
        }
        if (ch == '\n') break;
    }

    while (n > 0u && (out[n - 1u] == '\r' || out[n - 1u] == '\n')) n--;
    out[n] = '\0';

    return (n > 0u) ? (int)n : -1;
}

typedef enum { AT_OK = 0, AT_ERROR, AT_TIMEOUT } at_result_t;

/* expect == NULL means "wait for OK". */
static at_result_t at_send_wait(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    int      n;

    drv_uart_flush(UART_WIFI);
    drv_uart_write_str(UART_WIFI, cmd);

    while ((HAL_GetTick() - t0) < timeout_ms) {
        n = at_read_line(s_line, (uint16_t)sizeof(s_line), AT_REPLY_MS);
        if (n < 0) continue;

        if (strstr(s_line, "ERROR") != NULL) {
            app_log("[at] ERROR: %s\r\n", s_line);      /* 记下 ESP 原话，方便定位 */
            return AT_ERROR;
        }
        if (strstr(s_line, "FAIL") != NULL) {
            app_log("[at] FAIL: %s\r\n", s_line);
            return AT_ERROR;
        }

        /* "ALREADY CONNECTED" 里含有 "CONNECT"，如果放在下面按 expect 匹配，
         * AT+CIPSTART 会被误判成成功 —— 而那个 socket 往往是半死的，
         * 后面 AT+CIPSEND 等不到 '>'，表现为 connect FAIL (-6)。
         * 所以必须先拦下来当失败处理，让 CIPSTART 的重试循环真正生效。 */
        if (strstr(s_line, "ALREADY") != NULL) {
            app_log("[at] ALREADY: %s\r\n", s_line);
            return AT_ERROR;
        }

        if (expect != NULL) {
            if (strstr(s_line, expect) != NULL) return AT_OK;
        } else {
            if (strstr(s_line, "OK") != NULL) return AT_OK;
        }
    }
    return AT_TIMEOUT;
}

/* 连续发几次 AT，只要有一次回 OK 就算活着 */
static at_result_t at_probe(void)
{
    at_result_t r = AT_TIMEOUT;
    int         i;

    for (i = 0; i < 3; i++) {
        r = at_send_wait("AT", "OK", 800);
        if (r == AT_OK) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return r;
}

/* 退出透传：+++ 前后各需 1 秒静默，且不能带 \r\n */
static void at_exit_passthrough(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));                          /* 前置静默 */
    drv_uart_write(UART_WIFI, (const uint8_t *)"+++", 3u, 500u);
    vTaskDelay(pdMS_TO_TICKS(1000));                          /* 等模块回 OK / CLOSED */
    drv_uart_flush(UART_WIFI);
}

/* 硬复位：EN 拉低等于给模块断电，比 "+++" 可靠得多。
 * 场景：MCU 被外部拉低 NRST 复位（串口工具的 DTR/RTS 会触发），
 * 但 ESP 有独立供电不会跟着复位，于是它停在透传模式里，
 * 我们发的 AT 全被当成业务数据发给服务器 —— 表现为永远 init FAIL (-1)。
 *
 * 注意：PC0/PF6 是否真的接到模块座，是从厂商例程推的，没从原理图核实过。
 * 如果没接，这一步无效，但也不会更糟（后面还有返回 -1 的兜底）。
 * 验证方法：生效时串口会先冒出一小段乱码（ESP 用 74880 打 boot log）。 */
static void at_hard_reset(void)
{
    HAL_GPIO_WritePin(PIN_ESP_EN_PORT,  PIN_ESP_EN_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PIN_ESP_RST_PORT, PIN_ESP_RST_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(200));

    HAL_GPIO_WritePin(PIN_ESP_RST_PORT, PIN_ESP_RST_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PIN_ESP_EN_PORT,  PIN_ESP_EN_PIN,  GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1800));                          /* 等 AT 固件就绪 */

    drv_uart_flush(UART_WIFI);
}

/* -------------------------------------------------------------------- api */
int net_if_init(void)
{
    at_result_t r;
    int         i;

    s_state = NET_CONNECTING;

    vTaskDelay(pdMS_TO_TICKS(1200));          /* ESP-AT 从上电到能应答要 1s 以上 */

    r = at_probe();

    if (r != AT_OK) {
        /* 没响应：多半是模块还停在上次留下的透传模式里，AT 被当成业务数据发走了。
         * MCU 复位不会给 ESP 断电，这个状态会跨重启保留，所以必须在这里自愈。 */
        at_exit_passthrough();
        r = at_probe();
    }

    if (r != AT_OK) {
        /* +++ 也没救回来，别再猜了，直接给模块断电重启，状态全部清空 */
        app_log("\r\n[net] AT dead -> hard reset ESP\r\n");
        at_hard_reset();
        r = at_probe();
    }

    if (r != AT_OK) { s_state = NET_DOWN; return -1; }

    at_send_wait("ATE0", "OK", 1000);         /* 关回显，回应才干净 */

    /* 先确保不在透传模式。MCU 复位不会给 ESP 断电，上一轮可能就停在
     * CIPMODE=1 里。不清掉的话，后面 AT+CIPSEND 会被当成业务数据直接
     * 发进 socket、永远等不到 '>'，表现为 connect FAIL (-6)。 */
    at_send_wait("AT+CIPMODE=0", "OK", 1000);

    r = at_send_wait("AT+CWMODE=1", "OK", 1500);
    if (r != AT_OK) { s_state = NET_DOWN; return -2; }

    /* 清掉上次遗留的 socket。
     * 门道：ESP 关 socket 要发 FIN 再等 ACK，回 OK 常常要 1 秒以上，
     * 原来只给 500ms 会"以为关了其实没关"，下一轮 AT+CIPSTART 就回
     * ALREADY CONNECTED，接着 CIPSEND 等不到 '>'，永久 -6。
     * 所以放宽到 3 秒，并且连关两次确保干净。
     * 本来就没有 socket 时模块回 ERROR，属正常噪音，忽略。 */
    for (i = 0; i < 2; i++) {
        at_send_wait("AT+CIPCLOSE", "OK", 3000);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    return 0;
}

int net_if_connect(const char *host, uint16_t port, uint32_t timeout_ms)
{
    char        cmd[96];
    char        expect[48];
    at_result_t r;
    int         i;

    (void)timeout_ms;
    s_state = NET_CONNECTING;

    /* MCU 复位不会切断 ESP 的供电，所以走到这里时模块很可能还连着
     * 上一轮那个 AP。此时再发 AT+CWJAP 去连同一个 AP，固件会直接回
     * ERROR（拒绝重复连接），表现为 connect FAIL (-3) —— 这正是
     * "断电重启能连、按复位键连不上"的根因。
     *
     * 所以先查 AT+CWJAP?：回复里已经带着目标 SSID 就当已连接，跳过。
     * 既避开 ERROR，又省掉重新关联 AP 的 5 秒。
     * 期望串带上引号，避免 "1Ko_PC" 误匹配到 "1Ko_PC2"。 */
    snprintf(expect, sizeof(expect), "\"%s\"", WIFI_SSID);

    r = at_send_wait("AT+CWJAP?", expect, 2000);
    if (r == AT_OK) {
        app_log("[net] wifi already on %s\r\n", WIFI_SSID);
    } else {
        /* 没连、或连的是别的 AP：先断开把状态清干净，再重新关联。
         * CWQAP 在本来就没连时会回 ERROR，忽略即可。 */
        at_send_wait("AT+CWQAP", "OK", 3000);
        vTaskDelay(pdMS_TO_TICKS(300));

        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
        r = at_send_wait(cmd, "WIFI GOT IP", 15000);
        if (r != AT_OK) { s_state = NET_DOWN; return -3; }
    }

    /* 刚拿到 IP 时 DNS 往往还没就绪，立刻 CIPSTART 会直接 ERROR */
    vTaskDelay(pdMS_TO_TICKS(300));

    at_send_wait("AT+CIPMUX=0", "OK", 2000);

    for (i = 0; i < 3; i++) {
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);
        /* 只用 "CONNECT" 匹配是安全的：at_send_wait 已经在匹配前把
         * "ALREADY CONNECTED" 拦成 AT_ERROR，不会被它糊弄过去。 */
        r = at_send_wait(cmd, "CONNECT", 8000);
        if (r == AT_OK) break;

        /* 失败就先把可能半开的 socket 关干净再重试。
         * 给足 3 秒：ESP 关连接要等 FIN/ACK，500ms 往往不够。 */
        at_send_wait("AT+CIPCLOSE", "OK", 3000);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    if (r != AT_OK) {
        app_log("[net] CIPSTART failed (%d), socket unusable\r\n", (int)r);
        s_state = NET_DOWN; return -4;
    }

    r = at_send_wait("AT+CIPMODE=1", "OK", 2000);
    if (r != AT_OK) {
        app_log("[net] CIPMODE=1 failed (%d)\r\n", (int)r);
        s_state = NET_DOWN; return -5;
    }

    r = at_send_wait("AT+CIPSEND", ">", 3000);
    if (r != AT_OK) {
        /* 1 = ESP 明确拒绝（socket 没了）；2 = 三秒内一声不响（状态卡住）。
         * 区分开才知道是"重连能好"还是"得靠 AT+RST 才能救"。 */
        app_log("[net] CIPSEND failed (%d, 1=error 2=timeout)\r\n", (int)r);
        s_state = NET_DOWN; return -6;
    }

    s_state = NET_READY;
    return 0;
}

int net_if_send(const uint8_t *buf, uint16_t len)
{
    if (s_state != NET_READY || buf == NULL || len == 0u) return -1;
    return (drv_uart_write(UART_WIFI, buf, len, 2000u) == (int)len) ? 0 : -2;
}

uint16_t net_if_recv(uint8_t *buf, uint16_t maxlen, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    if (s_state != NET_READY || buf == NULL) return 0u;

    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint16_t n = drv_uart_read(UART_WIFI, buf, maxlen);
        if (n > 0u) return n;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 0u;
}

void net_if_flush_rx(void)
{
    /* 透传模式下 socket 来的字节直接落进串口环形缓冲，
     * 上一条报文（或 broker 的下推）可能还没被读走。
     * 心跳前必须清干净，否则旧字节会被当成 PINGRESP。 */
    drv_uart_flush(UART_WIFI);
}

void net_if_close(void)
{
    if (s_state == NET_READY) {
        /* +++ 前后各需 1 秒静默才能退出透传 */
        vTaskDelay(pdMS_TO_TICKS(1000));
        drv_uart_write(UART_WIFI, (const uint8_t *)"+++", 3u, 500u);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    s_state = NET_DOWN;
}

net_state_t net_if_state(void)
{
    return s_state;
}

const char *net_if_state_str(void)
{
    switch (s_state) {
    case NET_CONNECTING: return "connecting";
    case NET_READY:      return "ready";
    default:             return "down";
    }
}
