/*
 * MIT License
 *
 * Copyright (c) 2022 Teledatics, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

 /**
 * @file teledatics_gui.h
 * @author James Ewing
 * @date 24 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */
 
#include "nrc_sdk.h"

#include <api_wifi.h>
#include <dhcpserver.h>
#include <esp_http_server.h>
#include <esp_err.h>
#include <lwip.h>
#include <nrc_types.h>
#include <mbedtls/tls_base64.h>

#include "wifi_config_setup.h"
#include "wifi_config.h"
#include "wifi_connect_common.h"
#include "nrc_types.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_config.h"


#ifndef TELEDATICS_WIFI_H
#define TELEDATICS_WIFI_H

#define TD_WIFI_MODE_TAG "wifi_mode"
#define TD_WIFI_SSID_TAG NVS_SSID
#define TD_WIFI_SECURITY_TAG NVS_WIFI_SECURITY
#define TD_WIFI_PASSWORD_TAG NVS_WIFI_PASSWORD
#define TD_WIFI_COUNTRY_TAG "wifi_country"
#define TD_WIFI_CHANNEL_TAG NVS_WIFI_CHANNEL
#define TD_WIFI_IP_MODE_TAG NVS_IP_MODE
#define TD_WIFI_REMOTE_IP_TAG NVS_REMOTE_ADDRESS
#define TD_WIFI_REMOTE_PORT_TAG NVS_REMOTE_PORT
#define TD_WIFI_STATIC_IP_TAG NVS_STATIC_IP
#define TD_WIFI_STATIC_IPV6_TAG NVS_STATIC_IP6
#define TD_WIFI_DHCP_SERVER_TAG NVS_DHCP_SERVER_ON_WLAN

#define TD_WIFI_MODE_DEFAULT WIFI_MODE_AP
#define TD_WIFI_SSID_DEFAULT "teledatics"
#define TD_WIFI_PASSWORD_DEFAULT TD_WIFI_SSID_DEFAULT
#define TD_WIFI_SECURITY_DEFAULT WIFI_SEC_OPEN
#define TD_WIFI_COUNTRY_DEFAULT "US"
#define TD_WIFI_CHANNEL_DEFAULT 9225
#define TD_WIFI_IP_MODE_DEFAULT WIFI_STATIC_IP;
#define TD_WIFI_STATIC_IP_DEFAULT "192.168.200.1"
#define TD_WIFI_STATIC_IPV6_DEFAULT "fec0::1"
#define TD_WIFI_REMOTE_IP_DEFAULT NRC_REMOTE_ADDRESS
#define TD_WIFI_REMOTE_PORT_DEFAULT NRC_REMOTE_PORT
#define TD_WIFI_DHCP_SERVER_DEFAULT 1

#define MAX_WIFI_INIT_TRIES 64

extern int should_quit;
extern httpd_handle_t http_server;

typedef struct {
        uint8_t wifi_mode;
        char ipv6_addr[255];
        WIFI_CONFIG nrc_wifi_config;
} td_wifi_config_t;

void restart_system(void);
void td_print_settings(td_wifi_config_t *tf_config);
nrc_err_t td_save_wifi_config(td_wifi_config_t *tf_config);
nrc_err_t td_set_wifi_default(td_wifi_config_t *tf_config);
nrc_err_t td_get_wifi_config(td_wifi_config_t *tf_config);

void td_shutdown_ap(void);
void td_shutdown_client(void);
void td_shutdown_mesh(void);
nrc_err_t td_run_wifi_ap(td_wifi_config_t* tf_config);
nrc_err_t td_run_wifi_client(td_wifi_config_t* tf_config);
nrc_err_t td_run_wifi_mesh(td_wifi_config_t* tf_config);

extern httpd_uri_t setup_page;
extern httpd_uri_t update_settings;
esp_err_t setup_page_http(httpd_req_t *req);
esp_err_t update_settings_handler(httpd_req_t *req);
httpd_handle_t run_http_server(td_wifi_config_t* tf_config);
void stop_http_server(void);

#endif /* TELEDATICS_WIFI_H */
