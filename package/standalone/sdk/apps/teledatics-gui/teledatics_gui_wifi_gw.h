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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
 * @file teledatics_gui_wifi_gw.h
 * @author James Ewing
 * @date 5 May 2022
 * @brief Teledatics Wi-Fi GW hAT utilities for Halo TD-XPAH
 */

#include "teledatics_gui.h"

#ifndef TELEDATICS_WIFI_GW_H
#define TELEDATICS_WIFI_GW_H

/* Maximum retry count*/
#define RETRY_COUNT 5

/* Constants / macro */
#define CONTROL_PATH_TASK_STACK_SIZE 4096

#define PARAM_STR_YES "yes"
#define PARAM_STR_HT20 "20"
#define PARAM_STR_HT40 "40"
#define PARAM_STR_WPA_WPA2_PSK "WPA_WPA2_PSK"
#define PARAM_STR_WPA2_PSK "WPA2_PSK"
#define PARAM_STR_WPA_PSK "WPA_PSK"
#define PARAM_STR_OPEN "OPEN"

#define PARAM_STR_SOFTAP_STATION "SOFTAP+STATION"
#define PARAM_STR_STATION_SOFTAP "STATION+SOFTAP"
#define PARAM_STR_SOFTAP "SOFTAP"
#define PARAM_STR_STATION "STATION"

#define DEFAULT_SOFTAP_CHANNEL 1
#define DEFAULT_SOFTAP_MAX_CONNECTIONS 4
#define DEFAULT_LISTEN_INTERVAL 3

/** constants/macros **/
typedef enum
{
  MODE_ESP32_NULL = 0x0,
  MODE_ESP32_STATION = 0x1,
  MODE_ESP32_SOFTAP = 0x2,
  MODE_ESP32_SOFTAP_STATION = (MODE_ESP32_STATION | MODE_ESP32_SOFTAP),
  MODE_ESP32_MAX
} operating_mode;

typedef enum control_path_events_s
{
  STATION_ESP32_CONNECTED,
  STATION_ESP32_DISCONNECTED,
  SOFTAP_ESP32_STARTED,
  SOFTAP_ESP32_STOPPED
} control_path_events_e;

/* Possible operating modes are "STATION" or "SOFTAP" or "SOFTAP+STATION" */
/* "SOFTAP+STATION" and "STATION+SOFTAP" are same */
#ifndef INPUT__OPERATING_MODE
#define INPUT__OPERATING_MODE "SOFTAP+STATION"
#endif

/* Please refer commands.h for more details about below fields */
#ifndef INPUT_STATION__SSID
#define INPUT_STATION__SSID "teledatics"
#endif

#ifndef INPUT_STATION_PASSWORD
#define INPUT_STATION_PASSWORD "teledatics"
#endif

#ifndef INPUT_STATION_BSSID
#define INPUT_STATION_BSSID ""
#endif

#ifndef INPUT_STATION_IS_WPA3_SUPPORTED
#define INPUT_STATION_IS_WPA3_SUPPORTED "no"
#endif

#ifndef INPUT_STATION_LISTEN_INTERVAL
#define INPUT_STATION_LISTEN_INTERVAL "3"
#endif

/* softap means, ESP will act as Access point */
#ifndef INPUT_SOFTAP__SSID
#define INPUT_SOFTAP__SSID "teledatics"
#endif

#ifndef INPUT_SOFTAP_PASSWORD
#define INPUT_SOFTAP_PASSWORD "teledatics"
#endif

/* Channel to be used on soft ap */
#ifndef INPUT_SOFTAP_CHANNEL
#define INPUT_SOFTAP_CHANNEL "1"
#endif

#ifndef INPUT_SOFTAP_ENCRYPTION
#define INPUT_SOFTAP_ENCRYPTION "WPA2_PSK"
#endif

/* Software limit of Max clients attached to softAP. Max value possible is 10 */
#ifndef INPUT_SOFTAP_MAX_CONN
#define INPUT_SOFTAP_MAX_CONN "4"
#endif

/* 0 -> visible */
#ifndef INPUT_SOFTAP_SSID_HIDDEN
#define INPUT_SOFTAP_SSID_HIDDEN "no"
#endif

/* possible values, "HT20" "HT40" */
#ifndef INPUT_SOFTAP_BANDWIDTH
#define INPUT_SOFTAP_BANDWIDTH "40"
#endif

/* periodically scan neighbouring APs */
#ifndef INPUT_GET_AP_SCAN_LIST
#define INPUT_GET_AP_SCAN_LIST "yes"
#endif

/* TD_XPAH station self ip */
#ifndef INPUT_STATION_SRC_IP
#define INPUT_STATION_SRC_IP "192.168.1.233"
#endif

/* TD_XPAH station self ip */
#ifndef INPUT_SOFTAP_SRC_IP
#define INPUT_SOFTAP_SRC_IP "192.168.2.1"
#endif

/* station - ARP destination ip */
#ifndef INPUT_STATION_ARP_DEST_IP
#define INPUT_STATION_ARP_DEST_IP "192.168.1.11"
#endif

/* softap - ARP destination ip */
#ifndef INPUT_SOFTAP_ARP_DEST_IP
#define INPUT_SOFTAP_ARP_DEST_IP "192.168.2.22"
#endif

#define WIFI_MAX_STR_LEN 19

#define SUCCESS 0
#define FAILURE -1

#define SSID_LENGTH 32
#define MAX_MAC_STR_LEN 18
#define BSSID_LENGTH MAX_MAC_STR_LEN
#define PASSWORD_LENGTH 64
#define STATUS_LENGTH 14
#define VENDOR_OUI_BUF 3

#define CALLBACK_SET_SUCCESS 0
#define CALLBACK_AVAILABLE 0
#define CALLBACK_NOT_REGISTERED -1
#define MSG_ID_OUT_OF_ORDER -2

/* If request is already being served and
 * another request is pending, time period for
 * which new request will wait in seconds
 * */
#define WAIT_TIME_B2B_CTRL_REQ 5
#define DEFAULT_CTRL_RESP_TIMEOUT 30
#define DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT (60 * 3)

#define SUCCESS_STR "success"
#define FAILURE_STR "failure"
#define NOT_CONNECTED_STR "not_connected"

#define TO_SLAVE_QUEUE_SIZE 10
#define FROM_SLAVE_QUEUE_SIZE 10

#define TRANSACTION_TASK_STACK_SIZE 4096
#define PROCESS_RX_TASK_STACK_SIZE 4096

#define MAX_SPI_BUFFER_SIZE 1600
#define MAX_PAYLOAD_SIZE                                                       \
  (MAX_SPI_BUFFER_SIZE - sizeof(struct gw_payload_header))

#define htole16(x) ((uint16_t)(x))
#define le16toh(x) ((uint16_t)(x))

/*---- Control structures ----*/

enum
{
  CTRL_ERR_NOT_CONNECTED = 1,
  CTRL_ERR_NO_AP_FOUND,
  CTRL_ERR_INVALID_PASSWORD,
  CTRL_ERR_INVALID_ARGUMENT,
  CTRL_ERR_OUT_OF_RANGE,
  CTRL_ERR_MEMORY_FAILURE,
  CTRL_ERR_UNSUPPORTED_MSG,
  CTRL_ERR_INCORRECT_ARG,
  CTRL_ERR_PROTOBUF_ENCODE,
  CTRL_ERR_PROTOBUF_DECODE,
  CTRL_ERR_SET_ASYNC_CB,
  CTRL_ERR_TRANSPORT_SEND,
  CTRL_ERR_REQUEST_TIMEOUT,
  CTRL_ERR_REQ_IN_PROG,
  OUT_OF_RANGE
};

typedef enum
{
  CTRL_MSGTYPE_INVALID = 0,
  CTRL_REQ = 1,
  CTRL_RESP = 2,
  CTRL_EVENT = 3,
  CTRL_MSGTYPE_MAX,

} AppMsgType_e;

typedef enum
{
  CTRL_MSGID_INVALID = 0,
  CTRL_REQ_BASE = 100,
  CTRL_REQ_GET_MAC_ADDR = 101,
  CTRL_REQ_SET_MAC_ADDR = 102,
  CTRL_REQ_GET_WIFI_MODE = 103,
  CTRL_REQ_SET_WIFI_MODE = 104,
  CTRL_REQ_GET_AP_SCAN_LIST = 105,
  CTRL_REQ_GET_AP_CONFIG = 106,
  CTRL_REQ_CONNECT_AP = 107,
  CTRL_REQ_DISCONNECT_AP = 108,
  CTRL_REQ_GET_SOFTAP_CONFIG = 109,
  CTRL_REQ_SET_SOFTAP_VND_IE = 110,
  CTRL_REQ_START_SOFTAP = 111,
  CTRL_REQ_GET_SOFTAP_CONN_STA_LIST = 112,
  CTRL_REQ_STOP_SOFTAP = 113,
  CTRL_REQ_SET_PS_MODE = 114,
  CTRL_REQ_GET_PS_MODE = 115,
  CTRL_REQ_OTA_BEGIN = 116,
  CTRL_REQ_OTA_WRITE = 117,
  CTRL_REQ_OTA_END = 118,
  CTRL_REQ_SET_WIFI_MAX_TX_POWER = 119,
  CTRL_REQ_GET_WIFI_CURR_TX_POWER = 120,
  CTRL_REQ_CONFIG_HEARTBEAT = 121,
  CTRL_REQ_MAX = 122,
  CTRL_RESP_BASE = 200,
  CTRL_RESP_GET_MAC_ADDR = 201,
  CTRL_RESP_SET_MAC_ADDRESS = 202,
  CTRL_RESP_GET_WIFI_MODE = 203,
  CTRL_RESP_SET_WIFI_MODE = 204,
  CTRL_RESP_GET_AP_SCAN_LIST = 205,
  CTRL_RESP_GET_AP_CONFIG = 206,
  CTRL_RESP_CONNECT_AP = 207,
  CTRL_RESP_DISCONNECT_AP = 208,
  CTRL_RESP_GET_SOFTAP_CONFIG = 209,
  CTRL_RESP_SET_SOFTAP_VND_IE = 210,
  CTRL_RESP_START_SOFTAP = 211,
  CTRL_RESP_GET_SOFTAP_CONN_STA_LIST = 212,
  CTRL_RESP_STOP_SOFTAP = 213,
  CTRL_RESP_SET_PS_MODE = 214,
  CTRL_RESP_GET_PS_MODE = 215,
  CTRL_RESP_OTA_BEGIN = 216,
  CTRL_RESP_OTA_WRITE = 217,
  CTRL_RESP_OTA_END = 218,
  CTRL_RESP_SET_WIFI_MAX_TX_POWER = 219,
  CTRL_RESP_GET_WIFI_CURR_TX_POWER = 220,
  CTRL_RESP_CONFIG_HEARTBEAT = 221,
  CTRL_RESP_MAX = 222,
  CTRL_EVENT_BASE = 300,
  CTRL_EVENT_ESP_INIT = 301,
  CTRL_EVENT_HEARTBEAT = 302,
  CTRL_EVENT_STATION_DISCONNECT_FROM_AP = 303,
  CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP = 304,
  CTRL_EVENT_MAX = 305
} AppMsgId_e;

typedef enum
{
  WIFI_MODE_ESP32_NONE = 0,
  WIFI_MODE_ESP32_STA = 1,
  WIFI_MODE_ESP32_AP = 2,
  WIFI_MODE_ESP32_APSTA = 3,
  WIFI_MODE_ESP32_MAX
} wifi_mode_e;

typedef enum
{
  WIFI_AUTH_ESP32_OPEN = 0,
  WIFI_AUTH_ESP32_WEP = 1,
  WIFI_AUTH_ESP32_WPA_PSK = 2,
  WIFI_AUTH_ESP32_WPA2_PSK = 3,
  WIFI_AUTH_ESP32_WPA_WPA2_PSK = 4,
  WIFI_AUTH_ESP32_WPA2_ENTERPRISE = 5,
  WIFI_AUTH_ESP32_WPA3_PSK = 6,
  WIFI_AUTH_ESP32_WPA2_WPA3_PSK = 7,
  WIFI_AUTH_ESP32_MAX,
} wifi_auth_mode_e;

typedef enum
{
  WIFI_ESP32_BW_HT20 = 1,
  WIFI_ESP32_BW_HT40 = 2,
} wifi_bandwidth_e;

typedef enum
{
  WIFI_PS_MIN_MODEM = 1,
  WIFI_PS_MAX_MODEM = 2,
  WIFI_PS_INVALID,
} wifi_ps_type_e;

typedef enum
{
  WIFI_VND_IE_TYPE_BEACON = 0,
  WIFI_VND_IE_TYPE_PROBE_REQ = 1,
  WIFI_VND_IE_TYPE_PROBE_RESP = 2,
  WIFI_VND_IE_TYPE_ASSOC_REQ = 3,
  WIFI_VND_IE_TYPE_ASSOC_RESP = 4,
} wifi_vendor_ie_type_e;

typedef enum
{
  WIFI_VND_IE_ID_0 = 0,
  WIFI_VND_IE_ID_1 = 1,
} wifi_vendor_ie_id_e;

typedef void* wlan_buf_handle_t;

typedef struct
{
  union
  {
    wlan_buf_handle_t wlan_buf_handle;
    void* priv_buffer_handle;
  };
  uint8_t if_type;
  uint8_t if_num;
  uint8_t* payload;
  uint8_t flag;
  uint16_t payload_len;
  uint16_t seq_num;

  // 	void (*free_buf_handle)(void *buf_handle);
} interface_buffer_handle_t;

typedef struct
{
  /* Should be set to WIFI_VENDOR_IE_ELEMENT_ID (0xDD) */
  uint8_t element_id;
  /* Len of all bytes in the element data
   * following this field. Minimum 4 */
  uint8_t length;
  /* Vendor identifier (OUI) */
  uint8_t vendor_oui[VENDOR_OUI_BUF];
  /* Vendor-specific OUI type */
  uint8_t vendor_oui_type;
  /*length of payload field*/
  uint16_t payload_len;
  /* Payload. Length is equal to value in 'length' field, minus 4 */
  uint8_t* payload;

} vendor_ie_data_t;

typedef struct
{
  uint8_t ssid[SSID_LENGTH];
  uint8_t bssid[BSSID_LENGTH];
  int rssi;
  int channel;
  int encryption_mode;
} wifi_scanlist_t;

typedef struct
{
  uint8_t bssid[BSSID_LENGTH];
  int rssi;
} wifi_connected_stations_list_t;

typedef struct
{
  int mode;
  char mac[MAX_MAC_STR_LEN];
} wifi_mac_t;

typedef struct
{
  int mode;
} wifi_mode_t;

typedef struct
{
  uint8_t ssid[SSID_LENGTH];
  uint8_t pwd[PASSWORD_LENGTH];
  uint8_t bssid[BSSID_LENGTH];
  bool is_wpa3_supported;
  int rssi;
  int channel;
  int encryption_mode;
  uint16_t listen_interval;
  char status[STATUS_LENGTH];
  char out_mac[MAX_MAC_STR_LEN];
} wifi_ap_config_t;

typedef struct
{
  uint8_t ssid[SSID_LENGTH];
  uint8_t pwd[PASSWORD_LENGTH];
  int channel;
  int encryption_mode;
  int max_connections;
  bool ssid_hidden;
  wifi_bandwidth_e bandwidth;
  char out_mac[MAX_MAC_STR_LEN];
} softap_config_t;

typedef struct
{
  int count;
  /* dynamic size */
  wifi_scanlist_t* out_list;
} wifi_ap_scan_list_t;

typedef struct
{
  int count;
  /* dynamic list*/
  wifi_connected_stations_list_t* out_list;
} wifi_softap_conn_sta_list_t;

typedef struct
{
  int ps_mode;
} wifi_power_save_t;

typedef struct
{
  bool enable;
  wifi_vendor_ie_type_e type;
  wifi_vendor_ie_id_e idx;
  vendor_ie_data_t vnd_ie;
} wifi_softap_vendor_ie_t;

typedef struct
{
  uint8_t* ota_data;
  uint32_t ota_data_len;
} ota_write_t;

typedef struct
{
  int power;
} wifi_tx_power_t;

typedef struct
{
  /* event */
  uint32_t hb_num;
  /* Req */
  uint8_t enable;
  uint32_t duration;
} event_heartbeat_t;

typedef struct
{
  int32_t reason;
  char mac[MAX_MAC_STR_LEN];
} event_station_disconn_t;

struct gw_payload_header
{
  uint8_t if_type : 4;
  uint8_t if_num : 4;
  uint8_t flags;
  uint16_t len;
  uint16_t offset;
  uint16_t checksum;
  uint16_t seq_num;
  uint8_t reserved2;
  union
  {
    uint8_t reserved3;
    uint8_t hci_pkt_type;
    uint8_t priv_pkt_type;
  };
} __attribute__((packed));

typedef struct Ctrl_cmd_t
{
  /* msg type could be 1. req 2. resp 3. notification */
  uint8_t msg_type;

  /* control path protobuf msg number */
  uint16_t msg_id;

  /* statusof response or notification */
  uint8_t resp_event_status;

  union
  {
    wifi_mac_t wifi_mac;
    wifi_mode_t wifi_mode;

    wifi_ap_scan_list_t wifi_ap_scan;
    wifi_ap_config_t wifi_ap_config;

    softap_config_t wifi_softap_config;
    wifi_softap_vendor_ie_t wifi_softap_vendor_ie;
    wifi_softap_conn_sta_list_t wifi_softap_con_sta;

    wifi_power_save_t wifi_ps;

    ota_write_t ota_write;

    wifi_tx_power_t wifi_tx_power;

    event_heartbeat_t e_heartbeat;

    event_station_disconn_t e_sta_disconnected;
  } u;

  /* By default this callback is set to NULL.
   * When this callback is set by app while triggering request,
   * it will be automatically called asynchronously
   * by hosted control lib on receiving control response
   * in this case app will not be waiting for response.
   *
   * Whereas, when this is not set i.e. is NULL, it is understood
   * as synchronous response, and app after sending request,
   * will wait till getting a response
   */
  int (*ctrl_resp_cb)(struct Ctrl_cmd_t* data);

  /* Wait for timeout duration, if response not received,
   * it will send timeout response.
   * Default value for this time out is DEFAULT_CTRL_RESP_TIMEOUT */
  int cmd_timeout_sec;

  /* assign the data pointer to free by lower layer.
   * Ignored if assigned as NULL */
  void* free_buffer_handle;

  /* free handle to be registered
   * Ignored if assigned as NULL */
  void (*free_buffer_func)(void* free_buffer_handle);
} ctrl_cmd_t;

nrc_err_t
td_check_wifi_gw_hat(void);

static inline uint16_t
compute_checksum(uint8_t* buf, uint16_t len)
{
  uint16_t checksum = 0;
  uint16_t i = 0;

  while (i < len) {
    checksum += buf[i];
    i++;
  }

  return checksum;
}

#endif /* TELEDATICS_WIFI_GW_H */
