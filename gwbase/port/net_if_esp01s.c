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
#include <stdlib.h>     /* strtol：CWLAP 行里的信号强度用它转 */
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

/* 运行期 WiFi 凭据。初始值取编译期宏（app_config_local.h），
 * 开机后由 app_main 用配置文件里的值覆盖。
 *
 * 放在这里而不是直接读 wifi_cfg_*：port 层不该依赖 APP 层，
 * 以后把 ESP-01S 换成 4G 模块，这层照样能单独用。 */
static char        s_ssid[33] = WIFI_SSID;
static char        s_pass[65] = WIFI_PASSWORD;

/* 1 = 下次连接不要走"已连同名 AP 就跳过"的捷径。
 * 用户刚改过凭据时必须重连，否则改密码这种场景永远不生效。 */
static uint8_t     s_force_rejoin = 0;

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

int net_if_set_wifi(const char *ssid, const char *pass)
{
    int changed;

    if (ssid == NULL || pass == NULL || ssid[0] == '\0') {
        return -1;
    }

    /* 先判有没有变，再覆盖。反过来的话 strcmp 永远相等，判断等于没写。 */
    changed = (strcmp(s_ssid, ssid) != 0) || (strcmp(s_pass, pass) != 0);

    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1u);
    s_ssid[sizeof(s_ssid) - 1u] = '\0';
    strncpy(s_pass, pass, sizeof(s_pass) - 1u);
    s_pass[sizeof(s_pass) - 1u] = '\0';

    /* 只有真换了凭据才强制重连：开机时配置文件里的值和编译期宏往往一样，
     * 白白断一次再关联要多花 5 秒，还平白多一轮重连风险。
     *
     * 必须置位的原因：只改密码不改 SSID 时，模块还连着那个 AP，
     * 下面"已连同名 AP 就跳过"的捷径会让新密码永远用不上。 */
    if (changed) {
        s_force_rejoin = 1;
    }
    return 0;
}

/* ---------------------------------------------------------------- 扫描 */
/* 解析一行 +CWLAP:(3,"SSID",-67,"mac",ch,enc)。
 *
 * ESP-AT 会把 SSID 里的 \ 和 , 转义成 \\ 和 \,（不转义的话它们会被
 * 当成字段分隔符）。这里要还原成原始字节，否则存下来的名字和真实
 * SSID 对不上，连接时怎么填都连不上。
 * 返回 1 = 解析成功，0 = 这行不能用（没名字的隐藏网络等）。 */
static int cwlap_parse(const char *line, char *ssid, uint16_t ssid_size,
                        int8_t *rssi)
{
    const char *p;
    const char *q;
    uint16_t    i = 0;

    p = strchr(line, '"');
    if (p == NULL) return 0;
    p++;

    /* 找收尾引号。转义符 \X 算两个字节，不能停在 \ 后面那个字符上 */
    q = p;
    while (*q != '\0' && *q != '"') {
        q += (*q == '\\' && q[1] != '\0') ? 2 : 1;
    }
    if (*q != '"') return 0;

    /* 还原转义：\X -> X（\, 和 \\ 都覆盖了） */
    while (p < q && (i + 1u) < ssid_size) {
        if (*p == '\\' && (p + 1) < q) p++;
        ssid[i++] = *p++;
    }
    ssid[i] = '\0';
    if (i == 0u) return 0;          /* 没名字：隐藏网络，点选连不上，跳过 */

    /* 信号强度在收尾引号后面第一个逗号之后，形如 ,-67 */
    p = q + 1;
    if (*p == ',') p++;
    *rssi = (int8_t)strtol(p, NULL, 10);
    return 1;
}

int net_if_scan(char (*out_ssid)[33], int8_t *out_rssi, int max)
{
    uint32_t t0;
    int      n;
    int      cnt = 0;
    int      was_ready;

    if (out_ssid == NULL || out_rssi == NULL || max <= 0) return -1;

    was_ready = (s_state == NET_READY);

    /* 透传中必须先退出来，否则 AT+CWLAP 会原样发进 socket 给服务器。
     * 扫完在函数尾部重新 CIPSEND 回透传，TCP 不断。 */
    if (was_ready) {
        at_exit_passthrough();
    }

    drv_uart_flush(UART_WIFI);
    drv_uart_write_str(UART_WIFI, "AT+CWLAP");

    /* 扫一轮要 2 秒上下，期间没有任何输出，不能按"单行超时"判失败，
     * 得给整体 6 秒。超时也把已收到的行交出去 —— 半份结果好过没有。 */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 6000u) {
        n = at_read_line(s_line, (uint16_t)sizeof(s_line), AT_REPLY_MS);
        if (n < 0) continue;

        if (strstr(s_line, "ERROR") != NULL || strstr(s_line, "FAIL") != NULL) {
            cnt = -2;
            break;
        }
        if (strncmp(s_line, "+CWLAP:", 7) == 0) {
            if (cnt < max &&
                cwlap_parse(s_line, out_ssid[cnt], 33u, &out_rssi[cnt]) == 1) {
                cnt++;
            }
        }
        if (strstr(s_line, "OK") != NULL) break;
    }

    /* 按信号从强到弱排序，屏上列表把最好连的排最前面。
     * 最多 8 条，插入排序就够，不引排序函数。 */
    if (cnt > 1) {
        int a, b;

        for (a = 1; a < cnt; a++) {
            char    tmp[33];
            int8_t  r = out_rssi[a];

            strcpy(tmp, out_ssid[a]);
            for (b = a - 1; b >= 0 && out_rssi[b] < r; b--) {
                out_rssi[b + 1] = out_rssi[b];
                strcpy(out_ssid[b + 1], out_ssid[b]);
            }
            out_rssi[b + 1] = r;
            strcpy(out_ssid[b + 1], tmp);
        }
    }

    /* 回透传。等不到 '>' 说明 socket 已经死了（扫描期间被路由器掐了等），
     * 把状态判 DOWN，net_task 的 while(NET_READY) 自然退出走重连。 */
    if (was_ready) {
        if (at_send_wait("AT+CIPSEND", ">", 3000) != AT_OK) {
            app_log("[net] re-enter passthrough FAIL, drop link\r\n");
            s_state = NET_DOWN;
        }
    }

    return cnt;
}

/* CWJAP 的 SSID/密码参数里 \ 和 , 必须转义，否则会被当成分隔符。
 * 扫描列表里选出这种名字的网络时，不转义就永远连不上。 */
static void cmd_append_escaped(char *dst, uint16_t dst_size, const char *src)
{
    uint16_t n = (uint16_t)strlen(dst);

    while (*src != '\0' && (n + 2u) < dst_size) {
        if (*src == '\\' || *src == ',') {
            dst[n++] = '\\';
        }
        dst[n++] = *src++;
    }
    dst[n] = '\0';
}

int net_if_connect(const char *host, uint16_t port, uint32_t timeout_ms)
{
    char        cmd[160];
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
     * 期望串带上引号，避免 "1Ko_PC" 误匹配到 "1Ko_PC2"。
     *
     * 例外：用户刚在屏上改过凭据（s_force_rejoin），此时"同名 SSID"
     * 不能证明密码是对的，直接断开重来，不查捷径。 */
    if (s_force_rejoin) {
        /* 用户刚改过凭据：不能拿"同名 SSID"当已连接（密码可能变了）。
         * 直接落到下面的"断开 + 重新关联"分支，那里本来就会先 CWQAP。 */
        s_force_rejoin = 0;
        r = AT_ERROR;
    } else {
        snprintf(expect, sizeof(expect), "\"%s\"", s_ssid);
        r = at_send_wait("AT+CWJAP?", expect, 2000);
    }

    if (r == AT_OK) {
        app_log("[net] wifi already on %s\r\n", s_ssid);
    } else {
        /* 没连、或连的是别的 AP：先断开把状态清干净，再重新关联。
         * CWQAP 在本来就没连时会回 ERROR，忽略即可。 */
        at_send_wait("AT+CWQAP", "OK", 3000);
        vTaskDelay(pdMS_TO_TICKS(300));

        /* CWJAP 参数手工拼：\ 和 , 要转义（见 cmd_append_escaped），
         * snprintf 的 %s 做不了这件事。160 字节 = 指令头 + 32 + 64
         * 两个转义后上限，绰绰有余（原来 96 其实放不下满配密码）。 */
        strcpy(cmd, "AT+CWJAP=\"");
        cmd_append_escaped(cmd, (uint16_t)sizeof(cmd), s_ssid);
        strcat(cmd, "\",\"");
        cmd_append_escaped(cmd, (uint16_t)sizeof(cmd), s_pass);
        strcat(cmd, "\"");
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

    /* CIPSTART 刚回 CONNECT 时，ESP 的 TCP 栈还在收尾（状态机没完全就位），
     * 紧接着发 CIPSEND 会撞上 busy 窗口、等不到 '>'。歇 500ms 让它站稳。 */
    vTaskDelay(pdMS_TO_TICKS(500));

    r = at_send_wait("AT+CIPMODE=1", "OK", 2000);
    if (r != AT_OK) {
        app_log("[net] CIPMODE=1 failed (%d)\r\n", (int)r);
        s_state = NET_DOWN; return -5;
    }

    /* CIPSEND 最多试 3 次，间隔 500ms：偶发撞上 busy 一次就放弃太脆，
     * 重试一次通常就过了。3 次全失败才判定 socket 真的不行了。 */
    for (i = 0; i < 3; i++) {
        r = at_send_wait("AT+CIPSEND", ">", 3000);
        if (r == AT_OK) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
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
