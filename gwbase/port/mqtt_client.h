/**
 * @file    mqtt_client.h
 * @brief   Minimal MQTT 3.1.1 client on top of net_if.
 *
 * Only what this project needs: CONNECT, PUBLISH (QoS0), PINGREQ. No SUBSCRIBE,
 * no QoS1 retransmission - QoS0 over TCP is enough for periodic telemetry and
 * keeps the code small enough to read in one sitting.
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

/* Keepalive: send PINGREQ and wait up to 1s for PINGRESP.
 * 0 = link alive, -3 = no answer (link dead), -4 = garbage, -1/-2 = not connected / send failed.
 * Broker drops the session if it hears nothing for 1.5 x keepalive. */
int mqtt_ping(void);

/* Forget the session (call after the TCP link is gone). */
void mqtt_reset(void);

int mqtt_is_connected(void);

#endif /* __MQTT_CLIENT_H */
