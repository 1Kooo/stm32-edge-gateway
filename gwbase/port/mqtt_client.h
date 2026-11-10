/**
 * @file    mqtt_client.h
 * @brief   Minimal MQTT 3.1.1 client on top of net_if.
 *
 * Uplink: CONNECT, PUBLISH (QoS0), PINGREQ.
 * Downlink: SUBSCRIBE + inbound PUBLISH parsing (so the cloud can push
 * commands). QoS1/2, will, TLS and auth are intentionally left out - QoS0
 * over TCP is enough for telemetry plus occasional commands, and keeps the
 * client small enough to read in one sitting.
 *
 * Depends on net_if only, so it works with any transport implementation.
 */

#ifndef __MQTT_CLIENT_H
#define __MQTT_CLIENT_H

#include <stdint.h>

/* Username/password are left empty: Bemfa identifies the device by client id
 * (which is the private UID). timeout_ms is the CONNACK wait.
 * 0 = accepted, negative = failure (-3 no CONNACK, -10-n = broker refused, code n). */
int mqtt_connect(const char *client_id, uint16_t keepalive_s, uint32_t timeout_ms);

/* QoS0 publish. 0 on success. */
int mqtt_publish(const char *topic, const uint8_t *payload, uint16_t len);

/* Subscribe the topic at QoS0 and wait up to ~2s for SUBACK.
 * 0 = subscribed, -1 not connected, -2 send failed, -3 no SUBACK. */
int mqtt_subscribe(const char *topic);

/* Keepalive: send PINGREQ and wait up to ~2s for PINGRESP.
 * Inbound PUBLISH messages arriving in the same window are parsed, not
 * mistaken for garbage.
 * 0 = link alive, -3 = no answer (link dead), -1/-2 = not connected / send failed.
 * Broker drops the session if it hears nothing for 1.5 x keepalive. */
int mqtt_ping(void);

/* Take the latest inbound command if any. Returns 1 and NUL-terminated text in
 * out when a message was waiting, 0 otherwise. Drives the same parser as
 * mqtt_ping(), so it must be called regularly (net_task does it every loop). */
int mqtt_pump(char *out, uint16_t max, uint16_t *len);

/* Forget the session (call after the TCP link is gone). */
void mqtt_reset(void);

int mqtt_is_connected(void);

#endif /* __MQTT_CLIENT_H */
