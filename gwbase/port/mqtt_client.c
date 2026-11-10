/**
 * @file    mqtt_client.c
 * @brief   手写 MQTT 3.1.1 报文：CONNECT / PUBLISH(QoS0) / PINGREQ。
 *
 * 这里没有引任何 MQTT 库，全部按规范逐字节拼装。只依赖 net_if_send()
 * 往外送字节，所以本文件不知道 ESP-01S 存在，也不知道 AT 指令。
 *
 * 只实现"够用就行"的子集：
 *   上行：CONNECT / CONNACK 解析 / PUBLISH(QoS0) / PINGREQ
 *   下行：SUBSCRIBE / SUBACK 解析 / PUBLISH 解析（云端改参数）
 *   未实现：QoS1/2、遗嘱消息、TLS、用户名密码
 * 少这些东西，代码量从"几十 KB"降到 ~2KB，对双区 OTA 的 flash 预算很关键。
 *
 * 报文通用结构（MQTT 3.1.1 第 2 章）：
 *
 *   固定头(1B)  | 剩余长度(1~4B 变长) | 可变头 | 载荷
 *   bit7~4=类型  | 7 位一组，最高位     |
 *   bit3~0=标志  | 表示"后面还有字节"   |
 *
 * 三种报文的具体布局：
 *
 *   CONNECT : 0x10 | remlen | 00 04 "MQTT" | 04 | flags | keepalive(2)
 *             | client-id(len16 + bytes)
 *   PUBLISH : 0x30 | remlen | topic(len16 + bytes) | payload
 *   PINGREQ : 0xC0 0x00
 */

#include <string.h>

#include "mqtt_client.h"
#include "net_if.h"

/* 发送缓冲固定 192 字节，放静态区。
 * 不走 malloc 的原因：heap_4 长时间运行会产生碎片，而报文最大也就
 * 一个主题名 + 一百多字节 JSON，192 足够，静态更可预测。 */
#define MQTT_PKT_MAX   192u

/* 发 PINGREQ 后等 PINGRESP 的上限。
 * 本地局域网往返 1 秒本来够，但补传时 ESP-01S 正忙着转发报文，AT 应答
 * 可能被压后，1 秒容易误判（日志里 -3 三连就是这么来的）。放到 2 秒留余量，
 * 代价是判死的总时间从约 30 秒变成约 36 秒。 */
#define MQTT_PING_TIMEOUT_MS   2000u

static uint8_t  s_pkt[MQTT_PKT_MAX];
static uint8_t  s_connected = 0;    /* 只有收到合法 CONNACK 才置 1 */

/* ------------------------------------------------ 下行（云端 -> 设备）
 * 透传模式下 broker 推来的字节和 PINGRESP 走同一条串口流，必须自己拆包。
 * 认三种报文：PUBLISH(3) 取载荷当命令、SUBACK(9) / PINGRESP(13) 只置标志。
 * 残包留在 s_rx 里等下一轮补齐 —— 一条下行报文往往一轮 10ms 收不全。 */
#define RX_BUF   192u               /* 残包缓冲 */
#define CMD_MAX  64u                /* 最近一条云命令 */

static uint8_t  s_rx[RX_BUF];
static uint16_t s_rxlen;
static char     s_cmd[CMD_MAX];
static uint16_t s_cmdlen;
static uint8_t  s_cmd_new;          /* 1 = 有命令等调用方取走 */
static uint8_t  s_got_suback;
static uint8_t  s_got_pingresp;

/* ------------------------------------------------------------- 编码辅助 */

/**
 * 写入 MQTT 的"UTF-8 字符串"字段：2 字节大端长度 + 内容。
 * 规范里所有字符串（协议名、客户端 ID、主题名）都是这个格式，
 * 和 C 的 '\0' 结尾字符串不同，必须显式写长度。
 *
 * @return 写入的总字节数（2 + len），供调用方推进下标
 */
static uint16_t put_str(uint8_t *p, const char *s)
{
    uint16_t len = (uint16_t)strlen(s);

    p[0] = (uint8_t)(len >> 8);         /* 长度高字节在前（大端） */
    p[1] = (uint8_t)(len & 0xFFu);
    memcpy(&p[2], s, len);              /* 注意：不写结尾的 '\0' */

    return (uint16_t)(2u + len);
}

/**
 * 剩余长度变长编码（规范 2.2.3）。
 *
 * 每字节存 7 位有效数据，最高位是"还有后续字节"标志：
 *   0~127         -> 1 字节
 *   128~16383     -> 2 字节
 *   ...
 *   最大 4 字节，覆盖到 256MB
 *
 * 这样 127 字节以内的小报文只多花 1 字节，比固定 4 字节省 3 字节——
 * 对每秒都要发的小包，这个省法有意义。
 *
 * @return 编码占用的字节数（1~4）
 */
static uint16_t enc_remlen(uint8_t *p, uint32_t len)
{
    uint16_t n = 0;
    uint8_t  b;

    do {
        b = (uint8_t)(len % 128u);      /* 取低 7 位 */
        len /= 128u;                    /* 剩下的高位 */
        if (len > 0u) b |= 0x80u;       /* 还有后续，置最高位 */
        p[n++] = b;
    } while (len > 0u);

    return n;
}

/* ------------------------------------------------------- 下行报文解析 */

/**
 * 从 s_rx 头部拆一个完整报文，返回消费掉的字节数。
 *
 * @return 0 = 还没收全，等下一轮；>0 = 已消费这么多字节
 */
static uint16_t parse_one(void)
{
    uint32_t remlen = 0u;
    uint32_t mult   = 1u;
    uint32_t plen;
    uint16_t i      = 1u;
    uint16_t hdr;
    uint16_t tlen;
    uint16_t n;
    uint8_t  type;

    if (s_rxlen < 2u) return 0u;

    type = (uint8_t)(s_rx[0] >> 4);

    /* 剩余长度是变长编码：每字节 7 位有效，最高位表示"后面还有" */
    while (i < s_rxlen && i <= 4u) {
        remlen += (uint32_t)(s_rx[i] & 0x7Fu) * mult;
        mult *= 128u;
        i++;
        if ((s_rx[i - 1u] & 0x80u) == 0u) break;
    }
    if (i > 4u) return s_rxlen;                 /* 长度字段不合法，整段扔掉 */

    hdr = i;                                    /* 可变头从这里开始 */
    if ((uint32_t)s_rxlen < (uint32_t)hdr + remlen) return 0u;   /* 还没收全 */

    if (type == 3u) {
        /* PUBLISH：可变头是 topic（2 字节长度 + 名字），剩下才是载荷 */
        if (remlen >= 4u) {
            tlen = (uint16_t)(((uint16_t)s_rx[hdr] << 8) | s_rx[hdr + 1u]);
            plen = remlen - 2u - (uint32_t)tlen;

            if (plen > 0u) {
                n = (plen > (CMD_MAX - 1u)) ? (CMD_MAX - 1u) : (uint16_t)plen;
                memcpy(s_cmd, &s_rx[hdr + 2u + tlen], n);
                s_cmd[n]  = '\0';
                s_cmdlen  = n;
                s_cmd_new = 1u;
            }
        }
    } else if (type == 9u) {
        s_got_suback = 1u;
    } else if (type == 13u) {
        s_got_pingresp = 1u;
    }

    return (uint16_t)(hdr + remlen);
}

/**
 * 收一轮字节并尽量多解析几条报文。timeout_ms 传给 net_if_recv：
 * 0/10 = 轮询，100 = 等应答时的小步长。
 */
static void rx_poll(uint32_t timeout_ms)
{
    uint16_t n;
    uint16_t used;

    n = net_if_recv(&s_rx[s_rxlen], (uint16_t)(RX_BUF - s_rxlen), timeout_ms);
    s_rxlen = (uint16_t)(s_rxlen + n);

    for (;;) {
        used = parse_one();
        if (used == 0u) break;
        if (used > s_rxlen) used = s_rxlen;
        if (used < s_rxlen) {
            memmove(s_rx, &s_rx[used], (size_t)(s_rxlen - used));
        }
        s_rxlen = (uint16_t)(s_rxlen - used);
    }

    /* 缓冲塞满却拼不出完整报文 = 数据流已经错位，整段丢掉重来 */
    if (s_rxlen >= RX_BUF) s_rxlen = 0u;
}

/* ------------------------------------------------------------------ API */

/**
 * 发送 CONNECT 并同步等待 CONNACK。
 *
 * CONNECT 报文逐字段对应规范 3.1：
 *
 *   0x10              固定头：类型 1(CONNECT) << 4 | 标志 0
 *   remlen            剩余长度 = 可变头 10 字节 + 客户端 ID 字段(2+len)
 *   00 04 "MQTT"      协议名，长度前缀 + 4 个字符
 *   0x04              协议级别：4 = MQTT 3.1.1
 *   0x02              连接标志：bit1 = clean session
 *                       （不设 will、不设用户名密码、不设保留会话）
 *   keepalive (2B)    告诉 broker"我保证这么久内至少说一次话"
 *   client-id         **巴法云要求这里填私钥 UID**，用户名密码留空
 *
 * 返回码约定（打印到串口直接能看出原因）：
 *    0   成功
 *   -1   参数错（client_id 空）
 *   -2   发送失败（socket 已断）
 *   -3   超时，等不到 CONNACK（TCP 通了但 broker 没回，多为端口/域名问题）
 *   -4   收到的不是 CONNACK（首字节不是 0x20）
 *   -(10+n)  broker 明确拒绝，n 是 CONNACK 返回码：
 *            2 = 客户端 ID 被拒（私钥抄错/带了空格）
 *            4 = 用户名密码错   5 = 未授权
 *            例如打 -12 就是返回码 2
 */
int mqtt_connect(const char *client_id, uint16_t keepalive_s, uint32_t timeout_ms)
{
    uint16_t idlen, remlen, i = 0;
    uint8_t  rx[8];                     /* CONNACK 只有 4 字节，8 够余量 */
    uint16_t n;

    if (client_id == NULL || client_id[0] == '\0') return -1;

    idlen  = (uint16_t)strlen(client_id);
    /* 可变头固定 10 字节（2+4 协议名 + 1 版本 + 1 标志 + 2 keepalive）+ 客户端 ID */
    remlen = (uint16_t)(10u + 2u + idlen);

    s_pkt[i++] = 0x10u;                                     /* CONNECT */
    i = (uint16_t)(i + enc_remlen(&s_pkt[i], remlen));

    i = (uint16_t)(i + put_str(&s_pkt[i], "MQTT"));         /* 协议名 */
    s_pkt[i++] = 0x04u;                                     /* 协议级别 3.1.1 */
    s_pkt[i++] = 0x02u;                                     /* clean session，无账号密码 */
    s_pkt[i++] = (uint8_t)(keepalive_s >> 8);               /* keepalive 大端 */
    s_pkt[i++] = (uint8_t)(keepalive_s & 0xFFu);
    i = (uint16_t)(i + put_str(&s_pkt[i], client_id));

    s_connected = 0;
    if (net_if_send(s_pkt, i) != 0) return -2;

    /* 这里同步等 CONNACK，破坏了 MQTT 的异步性，但换来"连没连上"能立刻判定。
     * MQTT 严格按序回应：CONNACK 一定是 CONNECT 之后收到的第一个报文。 */
    n = net_if_recv(rx, (uint16_t)sizeof(rx), timeout_ms);
    if (n == 0u)              return -3;                    /* 等不到 CONNACK */
    if (rx[0] != 0x20u)       return -4;                    /* 0x20 = CONNACK 类型 */
    if (n >= 4u && rx[3] != 0x00u) return (int)(-10 - (int)rx[3]);

    s_connected = 1;
    return 0;
}

/**
 * 发布一条消息（QoS 0）。
 *
 * PUBLISH 报文：
 *   0x30            固定头：类型 3(PUBLISH) << 4 | 标志 0
 *                   标志低 4 位 = DUP(1) | QoS(2) | RETAIN(1)，全 0 即 QoS0
 *   所以 0x30 就是"PUBLISH + QoS0"：没有 packet-id，也不需要等 PUBACK
 *   remlen          剩余长度 = 2 + 主题长度 + 载荷长度
 *   topic           主题名（长度前缀字符串）—— 巴法云这里填**主题名**，不是昵称
 *   payload         载荷：发什么都行，我们发 JSON
 *
 * @return 0 成功 / -1 未连接或参数错 / -2 报文超出 192 字节 / -3 发送失败
 */
int mqtt_publish(const char *topic, const uint8_t *payload, uint16_t len)
{
    uint16_t tlen, remlen, i = 0;

    if (!s_connected || topic == NULL || payload == NULL) return -1;

    tlen   = (uint16_t)strlen(topic);
    remlen = (uint16_t)(2u + tlen + len);
    /* 剩余长度字段本身最多占 5 字节（1 固定头 + 4 变长），先做上界检查再拼 */
    if ((uint32_t)remlen + 5u > MQTT_PKT_MAX) return -2;

    s_pkt[i++] = 0x30u;                                     /* PUBLISH, QoS0 */
    i = (uint16_t)(i + enc_remlen(&s_pkt[i], remlen));
    i = (uint16_t)(i + put_str(&s_pkt[i], topic));
    memcpy(&s_pkt[i], payload, len);
    i = (uint16_t)(i + len);

    return (net_if_send(s_pkt, i) == 0) ? 0 : -3;
}

int mqtt_subscribe(const char *topic)
{
    uint16_t tlen, remlen, i = 0;
    uint8_t  try;

    if (!s_connected || topic == NULL || topic[0] == '\0') return -1;

    tlen   = (uint16_t)strlen(topic);
    remlen = (uint16_t)(2u + 2u + tlen + 1u);    /* 报文ID(2) + topic + QoS(1) */

    /* SUBSCRIBE 固定头低 4 位规范规定必须是 0010，所以是 0x82 不是 0x80 */
    s_pkt[i++] = 0x82u;
    i = (uint16_t)(i + enc_remlen(&s_pkt[i], remlen));
    s_pkt[i++] = 0x00u;                          /* 报文 ID 固定 1（只订一次） */
    s_pkt[i++] = 0x01u;
    i = (uint16_t)(i + put_str(&s_pkt[i], topic));
    s_pkt[i++] = 0x00u;                          /* 请求 QoS0，和我们发布保持一致 */

    s_got_suback = 0u;
    if (net_if_send(s_pkt, i) != 0) return -2;

    /* 20 × 100ms ≈ 2 秒。不等 SUBACK 也能收下行（broker 通常会直接推），
     * 但等一等能立刻看出"订阅有没有被拒"，省得后面一堆猜。 */
    for (try = 0; try < 20u && !s_got_suback; try++) {
        rx_poll(100u);
    }
    return (s_got_suback != 0u) ? 0 : -3;
}

/**
 * 心跳 PINGREQ + 等 PINGRESP，2 个字节：0xC0（类型 12 << 4，标志 0）+ 剩余长度 0。
 * 应答是 0xD0 0x00（类型 13 << 4）。
 *
 * 为什么必须有它：
 *   TCP 长连接会"假死"——路由重启、NAT 表项超时、手机切网之后，
 *   TCP 层可能几十分钟都不报错，你这边还以为连着，实际数据全进了黑洞。
 *   定期发 PINGREQ、broker 回 PINGRESP，双方才都能确认链路还活着。
 *
 * 为什么必须"等应答"而不是"发完就算"：
 *   net_if_send() 只保证字节进了 ESP 的串口缓冲，不代表出了网卡。
 *   断网时它照样返回 0 —— 只发不等，就等于永远认为自己在线上，
 *   于是 PUBLISH 一路"成功"，Flash 里的记录被逐条标记已发，
 *   断网期间的数据被永久丢弃。所以这里要真发、真等、真校验。
 *
 * 三步顺序不能反：
 *   1) 先 flush —— socket 里可能残留上一条报文的尾巴，不清掉会被当成 PINGRESP
 *   2) 发 PINGREQ
 *   3) 1 秒内等 2 字节并校验 0xD0 0x00
 *
 * 节奏要和 CONNECT 里声明的 keepalive 配合：我们声明 30 秒，
 * 实际空闲 20 秒就发一次，留出网络抖动的余量。
 *
 * @return  0 成功（收到合法 PINGRESP）
 *         -1 未连接
 *         -2 发送失败（socket 已断）
 *         -3 超时，1 秒内没等到应答——链路大概率已死
 *         -4 收到的不是 PINGRESP（只来了半个包，或报文错位）
 */
int mqtt_ping(void)
{
    uint8_t try;

    if (!s_connected) return -1;

    s_got_pingresp = 0u;

    s_pkt[0] = 0xC0u;                                       /* 2. 发 PINGREQ */
    s_pkt[1] = 0x00u;                                       /*    剩余长度 0 */
    if (net_if_send(s_pkt, 2u) != 0) return -2;

    /* 3. 20 × 100ms ≈ 2 秒等 PINGRESP。
     * 原来是一把 flush 再直接读 2 字节，撞上 broker 正在推下行就误判；
     * 现在走同一个解析器，下行报文被顺手消化掉，PINGRESP 照样能认出来。 */
    for (try = 0; try < 20u && !s_got_pingresp; try++) {
        rx_poll(100u);
    }

    return (s_got_pingresp != 0u) ? 0 : -3;
}

int mqtt_pump(char *out, uint16_t max, uint16_t *len)
{
    uint16_t n;

    if (out == NULL || len == NULL || max == 0u) return 0;

    rx_poll(10u);                   /* 有就取走，没有就最多等 10ms */

    if (s_cmd_new == 0u) return 0;

    n = (s_cmdlen < (uint16_t)(max - 1u)) ? s_cmdlen : (uint16_t)(max - 1u);
    memcpy(out, s_cmd, n);
    out[n] = '\0';
    *len   = n;

    s_cmd_new = 0u;
    return 1;
}

/**
 * 清掉"已连接"标志，不改动任何硬件状态。
 * 断线、重连前调用，避免 mqtt_publish() 往已死的 socket 里写。
 */
void mqtt_reset(void)
{
    s_connected = 0;
    s_rxlen     = 0u;       /* 断链后残包作废：QoS0 的报文 broker 不会补发 */
    s_cmd_new   = 0u;
    s_got_suback   = 0u;
    s_got_pingresp = 0u;
}

int mqtt_is_connected(void)
{
    return (int)s_connected;
}
