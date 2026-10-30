/**
 * @file    net_if.h
 * @brief   Network abstraction. Business code never touches AT commands.
 *
 * Keeping this layer separate means swapping ESP-01S for a 4G module or an
 * Ethernet PHY later only requires a new net_if_xxx.c - nothing above changes.
 * This file has no FreeRTOS and no ESP dependency on purpose.
 */

#ifndef __NET_IF_H
#define __NET_IF_H

#include <stdint.h>

typedef enum {
    NET_DOWN = 0,       /* not connected            */
    NET_CONNECTING,     /* WiFi / TCP in progress   */
    NET_READY           /* socket open, send allowed*/
} net_state_t;

/* 0 on success, negative on failure. */
int net_if_init(void);

/* Bring up WiFi and open a TCP socket. Blocking, up to timeout_ms. */
int net_if_connect(const char *host, uint16_t port, uint32_t timeout_ms);

/* Returns 0 on success. Only valid in NET_READY. */
int net_if_send(const uint8_t *buf, uint16_t len);

/* Returns bytes read, 0 if nothing arrived within timeout_ms. */
uint16_t net_if_recv(uint8_t *buf, uint16_t maxlen, uint32_t timeout_ms);

/* Drop everything already buffered on the RX side.
 * Needed before a request/response exchange (e.g. PINGREQ -> PINGRESP):
 * leftover bytes from a previous packet would otherwise be mistaken for
 * the reply, which makes a dead link look perfectly healthy. */
void net_if_flush_rx(void);

void net_if_close(void);

net_state_t net_if_state(void);
const char *net_if_state_str(void);

#endif /* __NET_IF_H */
