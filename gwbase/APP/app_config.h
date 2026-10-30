/**
 * @file    app_config.h
 * @brief   Configuration entry point (committed to git).
 *
 * Real credentials live in app_config_local.h, which is git-ignored and
 * enabled by defining APP_CONFIG_LOCAL in the Keil project options.
 * If that macro is not defined, the placeholders below are used and the
 * network simply will not connect - that is intended, so a fresh clone
 * still compiles without leaking anything.
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef APP_CONFIG_LOCAL
#include "app_config_local.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID        "YOUR_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD    "YOUR_PASSWORD"
#endif

#ifndef BEMFA_UID
#define BEMFA_UID        "YOUR_UID"
#endif

#ifndef BEMFA_TOPIC
#define BEMFA_TOPIC      "YOUR_TOPIC"
#endif

#ifndef BEMFA_HOST
#define BEMFA_HOST       "mqtt.bemfa.com"
#endif

#ifndef BEMFA_PORT
#define BEMFA_PORT       9501
#endif

#endif /* __APP_CONFIG_H */
