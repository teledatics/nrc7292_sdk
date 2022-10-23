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

#ifndef TELEDATICS_WIFI_GW_H
#define TELEDATICS_WIFI_GW_H

#include "teledatics_gui.h"
#include "nrc_esp32_if.h"
#include "esp_hosted_config.pb.h"

/* Maximum retry count*/
#define RETRY_COUNT 5

#define GPIO_RX_READY_PIN   GPIO_09
#define GPIO_HANDSHAKE_PIN  GPIO_10
#define GPIO_RESET_PIN      GPIO_17

#define CTRL_EP_NAME_RESP		"ctrlResp"
#define CTRL_EP_NAME_EVENT		"ctrlEvnt"
#define PROTO_PSER_TLV_T_EPNAME		0x01
#define PROTO_PSER_TLV_T_DATA		0x02
#define SIZE_OF_TYPE			1
#define SIZE_OF_LENGTH			2

#define CTRL_SEND_REQ(msGiD) do {                                     \
    req.msg_id = msGiD;                                               \
    if(SUCCESS != ctrl_app_send_req(&req)) {                          \
        nrc_usr_print("Failed to send control req %u\n", req.msg_id);        \
        return NULL;                                                  \
    }                                                                 \
} while(0);

#define CTRL_DECODE_RESP_IF_NOT_ASYNC() do {                          \
  if (CALLBACK_AVAILABLE == is_async_resp_callback_registered(req))   \
    return NULL;                                                      \
  return ctrl_wait_and_parse_sync_resp(&req);                         \
} while(0);


#define CLEANUP_APP_MSG(app_msg) do {                                         \
  if (app_msg) {                                                              \
    if (app_msg->free_buffer_handle) {                                        \
      if (app_msg->free_buffer_func) {                                        \
        app_msg->free_buffer_func(app_msg->free_buffer_handle);               \
        app_msg->free_buffer_handle = NULL;                                   \
      }                                                                       \
    }                                                                         \
    mem_free(app_msg);                                                        \
  }                                                                           \
} while(0);


#define CHECK_CTRL_MSG_NON_NULL_VAL(msGparaM, prinTmsG)                       /*\
    if (!payload.msGparaM) {                                                          \
        nrc_usr_print(prinTmsG"\n");                                            \
        goto err;                                             \
    }*/

#define CHECK_CTRL_MSG_NON_NULL(msGparaM)                                     /*\
    if (!resp->payload.msGparaM) {                                                \
        nrc_usr_print("Failed to process rx data\n");                           \
        goto err;                                             \
    }*/

#define CHECK_CTRL_MSG_FAILED(msGparaM)                                       /*\
    if (resp->payload.msGparaM->resp) {                                           \
        nrc_usr_print("Failure resp/event: possibly precondition not met\n");   \
        goto err;                                             \
    }*/


#define CTRL_ALLOC_ASSIGN(TyPe,MsG_StRuCt)                                    \
	TyPe *req_payload = &ctrl.payload.MsG_StRuCt;                         \
	memset(req_payload, 0, sizeof(TyPe))

#define SSID_LENGTH                          32
#define MAX_MAC_STR_LEN                      18
#define BSSID_LENGTH                         MAX_MAC_STR_LEN
#define PASSWORD_LENGTH                      64
#define STATUS_LENGTH                        14
#define VENDOR_OUI_BUF                       3

#define CALLBACK_AVAILABLE                   0

#define DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT    (60*3)
        
#define MAX_PWD_LENGTH                      64
#define MIN_PWD_LENGTH                      8

#define MIN_CHNL_NO                         1
#define MAX_CHNL_NO                         13
#define MIN_CONN_NO                         1
#define MAX_CONN_NO                         10
        
#define NOT_CONNECTED_STR                    "not_connected"
#define SUCCESS_STR                          "success"
#define FAILURE_STR                          "failure"

#define SUCCESS                             0
#define FAILURE                             -1


typedef struct {
	union {
		void *priv_buffer_handle;
	};
	uint8_t if_type;
	uint8_t if_num;
	uint8_t *payload;
	uint8_t flag;
	uint16_t payload_len;
	uint16_t seq_num;

	void (*free_buf_handle)(void *buf_handle);
} interface_buffer_handle_t;

typedef enum {
	ESP_WIFI_MODE_NONE = CTRL_WIFI_MODE_NONE,
	ESP_WIFI_MODE_STA = CTRL_WIFI_MODE_STA,
	ESP_WIFI_MODE_AP = CTRL_WIFI_MODE_AP,
	ESP_WIFI_MODE_APSTA = CTRL_WIFI_MODE_APSTA,
	ESP_WIFI_MODE_MAX
} wifi_mode_e;

typedef enum {
	ESP_WIFI_AUTH_OPEN = CTRL_WIFI_SEC_PROT_OPEN,
	ESP_WIFI_AUTH_WEP = CTRL_WIFI_SEC_PROT_WEP,
	ESP_WIFI_AUTH_WPA_PSK = CTRL_WIFI_SEC_PROT_WPA_PSK,
	ESP_WIFI_AUTH_WPA2_PSK = CTRL_WIFI_SEC_PROT_WPA2_PSK,
	ESP_WIFI_AUTH_WPA_WPA2_PSK = CTRL_WIFI_SEC_PROT_WPA_WPA2_PSK,
	ESP_WIFI_AUTH_WPA2_ENTERPRISE = CTRL_WIFI_SEC_PROT_WPA2_ENTERPRISE,
	ESP_WIFI_AUTH_WPA3_PSK = CTRL_WIFI_SEC_PROT_WPA3_PSK,
	ESP_WIFI_AUTH_WPA2_WPA3_PSK = CTRL_WIFI_SEC_PROT_WPA2_WPA3_PSK,
	ESP_WIFI_AUTH_MAX,
} wifi_auth_mode_e;

/*---- Control structures ----*/

enum {
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


typedef enum {

	CTRL_MSGTYPE_INVALID = CTRL_MSG_TYPE_MSG_TYPE_INVALID,
	CTRL_REQ = CTRL_MSG_TYPE_REQ,
	CTRL_RESP = CTRL_MSG_TYPE_RESP,
	CTRL_EVENT = CTRL_MSG_TYPE_EVENT,
	CTRL_MSGTYPE_MAX = CTRL_MSG_TYPE_MSG_TYPE_MAX,

} AppMsgType_e;

typedef enum {

	CTRL_MSGID_INVALID = CTRL_MSG_ID_MSG_ID_INVALID,
	/*
	 ** Request Msgs *
	 */
	CTRL_REQ_BASE                      = CTRL_MSG_ID_REQ_BASE,
	CTRL_REQ_GET_MAC_ADDR              = CTRL_MSG_ID_REQ_GET_MAC_ADDRESS, //0X65
	CTRL_REQ_SET_MAC_ADDR              = CTRL_MSG_ID_REQ_SET_MAC_ADDRESS, //0X66
	CTRL_REQ_GET_WIFI_MODE             = CTRL_MSG_ID_REQ_GET_WIFI_MODE,   //0X67
	CTRL_REQ_SET_WIFI_MODE             = CTRL_MSG_ID_REQ_SET_WIFI_MODE,   //0X68

	CTRL_REQ_GET_AP_SCAN_LIST          = CTRL_MSG_ID_REQ_GET_AP_SCAN_LIST, //0X69
	CTRL_REQ_GET_AP_CONFIG             = CTRL_MSG_ID_REQ_GET_AP_CONFIG,   //0X6A
	CTRL_REQ_CONNECT_AP                = CTRL_MSG_ID_REQ_CONNECT_AP,     //0X6B
	CTRL_REQ_DISCONNECT_AP             = CTRL_MSG_ID_REQ_DISCONNECT_AP,  //0X6C

	CTRL_REQ_GET_SOFTAP_CONFIG         = CTRL_MSG_ID_REQ_GET_SOFT_AP_CONFIG,            //0X6D
	CTRL_REQ_SET_SOFTAP_VND_IE         = CTRL_MSG_ID_REQ_SET_SOFT_AP_VENDOR_SPECIFIC_IE,  //0X6E
	CTRL_REQ_START_SOFTAP              = CTRL_MSG_ID_REQ_START_SOFT_AP,                //0X6F
	CTRL_REQ_GET_SOFTAP_CONN_STA_LIST  = CTRL_MSG_ID_REQ_GET_SOFT_AP_CONNECTED_STA_LIST,  //0X70
	CTRL_REQ_STOP_SOFTAP               = CTRL_MSG_ID_REQ_STOP_SOFT_AP,                 //0X71

	CTRL_REQ_SET_PS_MODE               = CTRL_MSG_ID_REQ_SET_POWER_SAVE_MODE,   //0X72
	CTRL_REQ_GET_PS_MODE               = CTRL_MSG_ID_REQ_GET_POWER_SAVE_MODE,   //0X73

	CTRL_REQ_OTA_BEGIN                 = CTRL_MSG_ID_REQ_OTA_BEGIN,           //0X74
	CTRL_REQ_OTA_WRITE                 = CTRL_MSG_ID_REQ_OTA_WRITE,           //0X75
	CTRL_REQ_OTA_END                   = CTRL_MSG_ID_REQ_OTA_END,             //0X76

	CTRL_REQ_SET_WIFI_MAX_TX_POWER     = CTRL_MSG_ID_REQ_SET_WIFI_MAX_TX_POWER,  //0X77
	CTRL_REQ_GET_WIFI_CURR_TX_POWER    = CTRL_MSG_ID_REQ_GET_WIFI_CURR_TX_POWER, //0X78

	CTRL_REQ_CONFIG_HEARTBEAT          = CTRL_MSG_ID_REQ_CONFIG_HEARTBEAT,    //0X79
	/*
	 * ADD NEW CONTROL PATH COMMAND RESPONSE BEFORE REQ_MAX
	 * AND UPDATE REQ_MAX
	 */
	CTRL_REQ_MAX = CTRL_MSG_ID_REQ_MAX,
	/*
	 ** RESPONSE MSGS *
	 */
	CTRL_RESP_BASE                     = CTRL_MSG_ID_RESP_BASE,
	CTRL_RESP_GET_MAC_ADDR             = CTRL_MSG_ID_RESP_GET_MAC_ADDRESS,  //0X65 -> 0XC9
	CTRL_RESP_SET_MAC_ADDRESS          = CTRL_MSG_ID_RESP_SET_MAC_ADDRESS,  //0X66 -> 0XCA
	CTRL_RESP_GET_WIFI_MODE            = CTRL_MSG_ID_RESP_GET_WIFI_MODE,    //0X67 -> 0XCB
	CTRL_RESP_SET_WIFI_MODE            = CTRL_MSG_ID_RESP_SET_WIFI_MODE,    //0X68 -> 0XCC

	CTRL_RESP_GET_AP_SCAN_LIST         = CTRL_MSG_ID_RESP_GET_AP_SCAN_LIST,  //0X69 -> 0XCD
	CTRL_RESP_GET_AP_CONFIG            = CTRL_MSG_ID_RESP_GET_AP_CONFIG,    //0X6A -> 0XCE
	CTRL_RESP_CONNECT_AP               = CTRL_MSG_ID_RESP_CONNECT_AP,      //0X6B -> 0XDF
	CTRL_RESP_DISCONNECT_AP            = CTRL_MSG_ID_RESP_DISCONNECT_AP,   //0X6C -> 0XD0

	CTRL_RESP_GET_SOFTAP_CONFIG        = CTRL_MSG_ID_RESP_GET_SOFT_AP_CONFIG,           //0X6D -> 0XD1
	CTRL_RESP_SET_SOFTAP_VND_IE        = CTRL_MSG_ID_RESP_SET_SOFT_AP_VENDOR_SPECIFIC_IE, //0X6E -> 0XD2
	CTRL_RESP_START_SOFTAP             = CTRL_MSG_ID_RESP_START_SOFT_AP,               //0X6F -> 0XD3
	CTRL_RESP_GET_SOFTAP_CONN_STA_LIST = CTRL_MSG_ID_RESP_GET_SOFT_AP_CONNECTED_STA_LIST, //0X70 -> 0XD4
	CTRL_RESP_STOP_SOFTAP              = CTRL_MSG_ID_RESP_STOP_SOFT_AP,                //0X71 -> 0XD5

	CTRL_RESP_SET_PS_MODE              = CTRL_MSG_ID_RESP_SET_POWER_SAVE_MODE, //0X72 -> 0XD6
	CTRL_RESP_GET_PS_MODE              = CTRL_MSG_ID_RESP_GET_POWER_SAVE_MODE, //0X73 -> 0XD7

	CTRL_RESP_OTA_BEGIN                = CTRL_MSG_ID_RESP_OTA_BEGIN,         //0X74 -> 0XD8
	CTRL_RESP_OTA_WRITE                = CTRL_MSG_ID_RESP_OTA_WRITE,         //0X75 -> 0XD9
	CTRL_RESP_OTA_END                  = CTRL_MSG_ID_RESP_OTA_END,           //0X76 -> 0XDA

	CTRL_RESP_SET_WIFI_MAX_TX_POWER     = CTRL_MSG_ID_RESP_SET_WIFI_MAX_TX_POWER,  //0X77 -> 0XDB
	CTRL_RESP_GET_WIFI_CURR_TX_POWER    = CTRL_MSG_ID_RESP_GET_WIFI_CURR_TX_POWER, //0X78 -> 0XDC

	CTRL_RESP_CONFIG_HEARTBEAT          = CTRL_MSG_ID_RESP_CONFIG_HEARTBEAT,    //0X79 -> 0XDD
	/*
	 * ADD NEW CONTROL PATH COMM       AND RESPONSE BEFORE RESP_MAX
	 * AND UPDATE RESP_MAX
	 */
	CTRL_RESP_MAX = CTRL_MSG_ID_RESP_MAX,
	/*
	 ** EVENTS
	 */
	CTRL_EVENT_BASE            = CTRL_MSG_ID_EVENT_BASE,
	CTRL_EVENT_ESP_INIT        = CTRL_MSG_ID_EVENT_ESP_INIT,
	CTRL_EVENT_HEARTBEAT       = CTRL_MSG_ID_EVENT_HEARTBEAT,
	CTRL_EVENT_STATION_DISCONNECT_FROM_AP =
		CTRL_MSG_ID_EVENT_STATION_DISCONNECT_FROM_AP,
	CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP =
		CTRL_MSG_ID_EVENT_STATION_DISCONNECT_FROM_ESP_SOFT_AP,
	/*
	 * ADD NEW CONTROL PATH COMMAND NOTIFICATION BEFORE EVENT_MAX
	 * AND UPDATE EVENT_MAX
	 */
	CTRL_EVENT_MAX = CTRL_MSG_ID_EVENT_MAX,
} AppMsgId_e;


typedef enum {
	WIFI_BW_HT20 = CTRL_WIFI_BW_HT20,
	WIFI_BW_HT40 = CTRL_WIFI_BW_HT40,
} wifi_bandwidth_e;

typedef enum {
	WIFI_PS_MIN_MODEM = CTRL_WIFI_POWER_SAVE_MIN_MODEM,
	WIFI_PS_MAX_MODEM = CTRL_WIFI_POWER_SAVE_MAX_MODEM,
	WIFI_PS_INVALID,
} wifi_ps_type_e;

typedef enum {
	WIFI_VND_IE_TYPE_BEACON      = CTRL_VENDOR_IE_TYPE_BEACON,
	WIFI_VND_IE_TYPE_PROBE_REQ   = CTRL_VENDOR_IE_TYPE_PROBE_REQ,
	WIFI_VND_IE_TYPE_PROBE_RESP  = CTRL_VENDOR_IE_TYPE_PROBE_RESP,
	WIFI_VND_IE_TYPE_ASSOC_REQ   = CTRL_VENDOR_IE_TYPE_ASSOC_REQ,
	WIFI_VND_IE_TYPE_ASSOC_RESP  = CTRL_VENDOR_IE_TYPE_ASSOC_RESP,
} wifi_vendor_ie_type_e;

typedef enum {
	WIFI_VND_IE_ID_0 = CTRL_VENDOR_IEID_ID_0,
	WIFI_VND_IE_ID_1 = CTRL_VENDOR_IEID_ID_1,
} wifi_vendor_ie_id_e;

typedef struct {
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

typedef struct {
	uint8_t ssid[SSID_LENGTH];
	uint8_t bssid[BSSID_LENGTH];
	int rssi;
	int channel;
	int encryption_mode;
} wifi_scanlist_t;

typedef struct {
	uint8_t bssid[BSSID_LENGTH];
	int rssi;
} wifi_connected_stations_list_t;

typedef struct {
	int mode;
	char mac[MAX_MAC_STR_LEN];
} wifi_mac_t;

typedef struct {
	int mode;
} wifi_mode_t;

typedef struct {
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

typedef struct {
	uint8_t ssid[SSID_LENGTH];
	uint8_t pwd[PASSWORD_LENGTH];
	int channel;
	int encryption_mode;
	int max_connections;
	bool ssid_hidden;
	wifi_bandwidth_e bandwidth;
	char out_mac[MAX_MAC_STR_LEN];
} softap_config_t;

typedef struct {
	int count;
	/* dynamic size */
	wifi_scanlist_t *out_list;
} wifi_ap_scan_list_t;

typedef struct {
	int count;
	/* dynamic list*/
	wifi_connected_stations_list_t *out_list;
} wifi_softap_conn_sta_list_t;

typedef struct {
	int ps_mode;
} wifi_power_save_t;

typedef struct {
	bool enable;
	wifi_vendor_ie_type_e type;
	wifi_vendor_ie_id_e idx;
	vendor_ie_data_t vnd_ie;
} wifi_softap_vendor_ie_t;

typedef struct {
	uint8_t *ota_data;
	uint32_t ota_data_len;
} ota_write_t;

typedef struct {
	int power;
} wifi_tx_power_t;

typedef struct {
	/* event */
	uint32_t hb_num;
	/* Req */
	uint8_t enable;
	uint32_t duration;
} event_heartbeat_t;

typedef struct {
	int32_t reason;
	char mac[MAX_MAC_STR_LEN];
} event_station_disconn_t;

typedef struct Ctrl_cmd_t {
	/* msg type could be 1. req 2. resp 3. notification */
	uint8_t msg_type;

	/* control path protobuf msg number */
	uint16_t msg_id;

	/* statusof response or notification */
	uint8_t resp_event_status;

	union {
		wifi_mac_t                  wifi_mac;
		wifi_mode_t                 wifi_mode;

		wifi_ap_scan_list_t         wifi_ap_scan;
		wifi_ap_config_t            wifi_ap_config;

		softap_config_t             wifi_softap_config;
		wifi_softap_vendor_ie_t     wifi_softap_vendor_ie;
		wifi_softap_conn_sta_list_t wifi_softap_con_sta;

		wifi_power_save_t           wifi_ps;

		ota_write_t                 ota_write;

		wifi_tx_power_t             wifi_tx_power;

		event_heartbeat_t           e_heartbeat;

		event_station_disconn_t     e_sta_disconnected;
	}u;

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
	int (*ctrl_resp_cb)(struct Ctrl_cmd_t *data);

	/* Wait for timeout duration, if response not received,
	 * it will send timeout response.
	 * Default value for this time out is DEFAULT_CTRL_RESP_TIMEOUT */
	int cmd_timeout_sec;

	/* assign the data pointer to free by lower layer.
	 * Ignored if assigned as NULL */
	void *free_buffer_handle;

	/* free handle to be registered
	 * Ignored if assigned as NULL */
	void (*free_buffer_func)(void *free_buffer_handle);
} ctrl_cmd_t;

nrc_err_t td_check_wifi_gw_hat(void);
#endif /* TELEDATICS_WIFI_GW_H */
