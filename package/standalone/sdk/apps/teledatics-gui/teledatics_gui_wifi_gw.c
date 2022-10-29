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
 * @file teledatics_gui_wifi_gw.c
 * @author James Ewing
 * @date 5 May 2022
 * @brief Teledatics Wi-Fi GW hAT Routines
 */

#include "teledatics_gui_wifi_gw.h"
#include "lwip/pbuf.h"
#include "nrc_eth_if.h"
#include "standalone.h"
#include "wlan/protocol.h"

#include "pb_encode.h"
#include "pb_decode.h"
#include "esp_hosted_config.pb.h"

#include "adapter.h"

#include <time.h>
#include <machine/endian.h>

#define MAX_SPI_BUFFER_SIZE               1600
#define MAX_ESP_PAYLOAD_SIZE (MAX_SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))
#define le16toh(x)                        ((uint16_t)(x))
#define htole16(x)                        ((uint16_t)(x))

// Wi-Fi Gateway hAT
bool wifi_init_done = false;
bool wifi_gw_exists = false;
spi_device_t esp_spi;
esp32_mac_t *esp32_mac = NULL;

static SemaphoreHandle_t semaphore_rx = NULL;
static SemaphoreHandle_t semaphore_rx_serial = NULL;
static SemaphoreHandle_t mutex_spi = NULL;

static TaskHandle_t rx_task_id = 0;
static TaskHandle_t rx_serial_task_id = 0;

static QueueHandle_t rx_queue = NULL;
static QueueHandle_t tx_queue = NULL;

#define MAX_SERIAL_INTF    2
#define TO_SERIAL_INFT_QUEUE_SIZE         100
static QueueHandle_t rx_queue_serial = NULL;

// forwards
nrc_err_t esp_send_req(ctrl_cmd_t *cmd);

/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(bool *is_valid_tx_buf)
{
	struct  esp_payload_header *payload_header;
	static uint8_t sendbuf[MAX_SPI_BUFFER_SIZE];
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

	nrc_usr_print("[%s]\n", __func__);
	
	*is_valid_tx_buf = false;

        memset(&buf_handle, 0, sizeof(interface_buffer_handle_t));

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */
	if (pdTRUE == xQueueReceive(tx_queue, &buf_handle, 0)) {
		len = buf_handle.payload_len;
	}

        memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

        if (len) {
		*is_valid_tx_buf = true;

		nrc_usr_print("[%s] rec'd from tx_queue, sending %d bytes w offset %d bytes\n", __func__, len, sizeof(struct esp_payload_header));
		
		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload = sendbuf + sizeof(struct esp_payload_header);
		payload_header->len     = htole16(len);
		payload_header->offset  = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num  = buf_handle.if_num;
		memcpy(payload, buf_handle.payload, min(len, MAX_ESP_PAYLOAD_SIZE));
		payload_header->checksum = htole16(compute_checksum(sendbuf,
				sizeof(struct esp_payload_header)+len));
		
		/* free allocated buffer */
		if (buf_handle.free_buf_handle && buf_handle.payload)
			buf_handle.free_buf_handle(buf_handle.payload);
		
		print_hex(sendbuf, sizeof(struct esp_payload_header) + len);
	}

	return sendbuf;
}

/**
  * @brief  Schedule SPI transaction if -
  *         a. valid TX buffer is ready at SPI host (nrc7292)
  *         b. valid TX buffer is ready at SPI peripheral (ESP)
  *         c. Dummy transaction is expected from SPI peripheral (ESP)
  * @param  argument: Not used
  * @retval None
  */
static nrc_err_t spi_transaction(void)
{
        uint8_t rxbuf[MAX_SPI_BUFFER_SIZE], *txbuf=NULL, tmpbuf[MAX_SPI_BUFFER_SIZE];
        interface_buffer_handle_t buf_handle = {0};
        struct  esp_payload_header *payload_header;
	int gpio_handshake = GPIO_LEVEL_LOW;
	int gpio_rx_data_ready = GPIO_LEVEL_LOW;
        nrc_err_t ret = NRC_FAIL;
        uint16_t rx_checksum = 0, checksum = 0;
        uint16_t len = MAX_SPI_BUFFER_SIZE, offset;
        bool is_valid_tx_buf = false;

        nrc_usr_print("[%s]\n", __func__);
        
	memset(rxbuf, 0, MAX_SPI_BUFFER_SIZE);

	nrc_gpio_inputb(GPIO_HANDSHAKE_PIN, &gpio_handshake);

	nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready);
	
	nrc_usr_print("[%s] rx_ready %d handshake %d\n", __func__, gpio_rx_data_ready, gpio_handshake);

	if (gpio_handshake == GPIO_LEVEL_HIGH) {
          
          txbuf = get_tx_buffer(&is_valid_tx_buf);
          
          // rx ready or xmit
          if ( (gpio_rx_data_ready == GPIO_LEVEL_HIGH) || is_valid_tx_buf ) {
            
            if (!is_valid_tx_buf) {
              txbuf = tmpbuf;
              memset(tmpbuf, 0, sizeof(tmpbuf));
            }
              
            xSemaphoreTake(mutex_spi, portMAX_DELAY);
            
            nrc_spi_start_xfer(&esp_spi);
            ret = nrc_spi_xfer(&esp_spi, txbuf, rxbuf, len);
            nrc_spi_stop_xfer(&esp_spi);
            
            xSemaphoreGive(mutex_spi);
	
	   nrc_gpio_inputb(GPIO_HANDSHAKE_PIN, &gpio_handshake);
	   nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready);
	    
            nrc_usr_print("[%s] ret %d, read %d bytes is_valid_tx_buf %d rx_ready %d handshake %d\n", __func__, ret, len, is_valid_tx_buf, gpio_rx_data_ready, gpio_handshake);
//             print_hex(rxbuf, len);
          }
        }

	switch(ret)
	{
		case NRC_SUCCESS:

			/* Transaction successful */

			/* create buffer rx handle, used for processing */
			payload_header = (struct esp_payload_header *) rxbuf;

			/* Fetch length and offset from payload header */
			len = le16toh(payload_header->len);
			offset = le16toh(payload_header->offset);
                        
			if ((!len) ||
				(len > MAX_ESP_PAYLOAD_SIZE) ||
				(offset != sizeof(struct esp_payload_header))) {

                                nrc_usr_print("[%s] header error len %d\n", __func__, ret, len);
                        
				/* Free up buffer, as one of following -
				 * 1. no payload to process
				 * 2. input packet size > driver capacity
				 * 3. payload header size mismatch,
				 * wrong header/bit packing?
				 * */

				/* Give chance to other tasks */
				vTaskDelay(0);

			} else {
                                wifi_gw_exists = true;
                                
				nrc_usr_print("[%s] decode header\n", __func__);
				
				print_hex(rxbuf, sizeof(struct esp_payload_header) + len);
				
				rx_checksum = le16toh(payload_header->checksum);
				payload_header->checksum = 0;

				checksum = compute_checksum(rxbuf, len+offset);

				if (checksum == rx_checksum) {
                                        uint8_t* rx_queue_buf = NULL;
                                        
                                        rx_queue_buf = malloc(sizeof(struct esp_payload_header) + len);
                                        if(!rx_queue_buf) {
                                          ret = NRC_FAIL;
                                          break;
                                        }
                                        
                                        memcpy(rx_queue_buf, rxbuf, sizeof(struct esp_payload_header) + len);
                                        
                                        // allow queue buf
					buf_handle.priv_buffer_handle = rx_queue_buf;
					buf_handle.free_buf_handle = free;
					buf_handle.payload_len = len;
					buf_handle.if_type     = payload_header->if_type;
					buf_handle.if_num      = payload_header->if_num;
					buf_handle.payload     = rx_queue_buf + offset;
					buf_handle.seq_num     = le16toh(payload_header->seq_num);
					buf_handle.flag        = payload_header->flags;

                                        nrc_usr_print("[%s] header OK, len %d checksum 0x%X\n", __func__, len, checksum);
					
					if (pdTRUE != xQueueSend(rx_queue,
								&buf_handle, portMAX_DELAY)) {
						nrc_usr_print("Failed to send buffer\n\r");
                                                ret = NRC_FAIL;
						break;
					}
				}
			}
			break;

		case NRC_FAIL:
			nrc_usr_print("Error in SPI transaction\n\r");
			break;
		default:
			nrc_usr_print("default handler: Error in SPI transaction\n\r");
			break;
	}

	return ret;
}

bool rx_ready(void)
{
  int gpio_rx_data_ready_pin_val = 0;
  
  nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready_pin_val);
  
  return (gpio_rx_data_ready_pin_val == GPIO_LEVEL_HIGH);
}

/**
 * @brief spi receive isr
 *
 * Interrupt service routine to trigger rx thread
 *
 * @param optional callback vector
 * @return none
 */
void rx_spi_isr(int vector)
{
  static BaseType_t pxHigherPriorityTaskWoken;
  
  nrc_usr_print("[%s]\n", __func__);
    
  if (!rx_ready())
    return;

  nrc_usr_print("[%s] take semaphore_rx\n", __func__);
  
  pxHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(semaphore_rx, &pxHigherPriorityTaskWoken);
  portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}

uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length)
{
	char* ep_name = CTRL_EP_NAME_RESP;
	uint16_t ep_length = strlen(ep_name);
	uint16_t count = 0;
	buf[count++] = PROTO_PSER_TLV_T_EPNAME;
	buf[count++] = (ep_length & 0xFF);
	buf[count++] = ((ep_length >> 8) & 0xFF);
	strncpy((char *)&buf[count], ep_name, ep_length);
	count = count + ep_length;
	buf[count++]= PROTO_PSER_TLV_T_DATA;
	buf[count++] = (data_length & 0xFF);
	buf[count++] = ((data_length >> 8) & 0xFF);
	memcpy(&buf[count], data, data_length);
	count = count + data_length;
	return count;
}

nrc_err_t parse_tlv(uint8_t* data, uint32_t* pro_len)
{
	char* ep_name = CTRL_EP_NAME_RESP;
	char* ep_name2 = CTRL_EP_NAME_EVENT;
	uint64_t len = 0;
	uint16_t val_len = 0;
	
	nrc_usr_print("[%s]\n", __func__);
	
	if (data[len] == PROTO_PSER_TLV_T_EPNAME) {
		len++;
		val_len = data[len++];
		val_len = (data[len++] << 8) + val_len;

		/* Both CTRL_EP_NAME_RESP and CTRL_EP_NAME_EVENT
		 * are expected to have exactly same length
		 **/
		if (val_len == strlen(ep_name)) {
			if ((strncmp((char* )&data[len],ep_name,strlen(ep_name)) == 0) ||
			    (strncmp((char* )&data[len],ep_name2,strlen(ep_name2)) == 0)) {
				len = len + strlen(ep_name);
				if (data[len] == PROTO_PSER_TLV_T_DATA) {
					len++;
					val_len = data[len++];
					val_len = (data[len++] << 8) + val_len;
					*pro_len = val_len;
					nrc_usr_print("[%s] parsed TLV successfully\n", __func__);
					return NRC_SUCCESS;
				} else {
					nrc_usr_print("Data Type not matched, exp %d, recvd %d\n",
							PROTO_PSER_TLV_T_DATA, data[len]);
				}
			} else {
				nrc_usr_print("Endpoint Name not matched, exp [%s] or [%s], recvd [%s]\n",
						ep_name, ep_name2, (char* )&data[len]);
			}
		} else {
			nrc_usr_print("Endpoint length not matched, exp [For %s, %lu OR For %s, %lu], recvd %d\n",
					ep_name, (long unsigned int)(strlen(ep_name)),
					ep_name2, (long unsigned int)(strlen(ep_name2)), val_len);
		}
	} else {
		nrc_usr_print("Endpoint type not matched, exp %d, recvd %d\n",
				PROTO_PSER_TLV_T_EPNAME, data[len]);
	}
	return NRC_FAIL;
}

nrc_err_t td_init_esp_wifi(ctrl_cmd_t* cmd)
{
	nrc_err_t retval = NRC_FAIL;
	
	nrc_usr_print("[%s]\n", __func__);
	
	// set cmd SSID, channel, mode, security
	cmd->msg_type = CTRL_MSG_TYPE_REQ;
	cmd->msg_id = CTRL_MSG_ID_REQ_SET_WIFI_MODE;
	cmd->u.wifi_mode.mode = CTRL_WIFI_MODE_STA;
	
	retval = esp_send_req(cmd);
// 	if(!retval)
// 		goto err;

	nrc_usr_print("[%s] retval %d\n", __func__, retval);
	
	cmd->msg_type = CTRL_MSG_TYPE_REQ;
	cmd->msg_id = CTRL_MSG_ID_REQ_CONNECT_AP;
	cmd->u.wifi_mode.mode = ESP_WIFI_MODE_AP;
	strcpy((char*)cmd->u.wifi_ap_config.ssid, "OnHub");
	strcpy((char*)cmd->u.wifi_ap_config.pwd, "zippity321");
	strcpy((char*)cmd->u.wifi_ap_config.bssid, "74:E5:F9:97:13:FA");
// 	memset(cmd->u.wifi_ap_config.bssid, 0, sizeof(cmd->u.wifi_ap_config.bssid));
	cmd->u.wifi_ap_config.is_wpa3_supported = 0;
	cmd->u.wifi_ap_config.listen_interval = 0;
	
	retval = esp_send_req(cmd);
	
	cmd->msg_type = CTRL_MSG_TYPE_REQ;
	cmd->msg_id = CTRL_MSG_ID_REQ_GET_AP_CONFIG;
	retval = esp_send_req(cmd);
	
// 	if(!retval)
// 		goto err;
	
	nrc_usr_print("[%s] retval %d\n", __func__, retval);
	
// err:
	return retval;
}

/* This is entry level function when control request APIs are used
 * This function will encode control request in protobuf and send to ESP32
 * It will copy application structure `ctrl_cmd_t` to
 * protobuf control req `CtrlMsg`
 **/
nrc_err_t esp_recv_resp(uint8_t* buf, int len, ctrl_cmd_t* cmd)
{
  ctrl_msg_t* resp, ctrl_msg = CTRL_MSG_INIT_DEFAULT;
  pb_istream_t npb_istream;
  bool status;
  nrc_err_t retval = NRC_FAIL;
  
  nrc_usr_print("[%s]\n", __func__);
  
  if(!buf || !cmd || len <= 0) {
    goto err;
  }

  resp = &ctrl_msg;

  npb_istream = pb_istream_from_buffer(buf, len);
  
  print_hex(buf, len);

  status = pb_decode_ex(&npb_istream, CTRL_MSG_FIELDS, resp, PB_ENCODE_NULLTERMINATED);
  
  if (!status) {
    goto err;
  }
  
  if (resp->msg_type == CTRL_MSG_TYPE_EVENT) {
		/* Events are handled only asynchronously */
		nrc_usr_print("[%s] CTRL_MSG_TYPE_EVENT msg_id %d\n", __func__, resp->msg_id);

// 		if(resp->msg_id == CTRL_MSG_EVENT_ESP_INIT_TAG) {
// 			td_init_esp_wifi(cmd);
// 		}
		/* check if callback is available.
		 * if not, silently drop the msg */
// 		if (CALLBACK_AVAILABLE ==
// 				is_event_callback_registered(proto_msg->msg_id)) {
// 			/* if event callback is registered, we need to
// 			 * parse the event into app structs and
// 			 * call the registered callback function
// 			 **/
// 
// 			/* Allocate app struct for event */
// 			app_event = (ctrl_cmd_t *)hosted_malloc(sizeof(ctrl_cmd_t));
// 			if (!app_event) {
// 				nrc_usr_print("Failed to allocate app_event\n");
// 				goto free_buffers;
// 			}
// 			memset(app_event, 0, sizeof(ctrl_cmd_t));
// 
// 			/* Decode protobuf buffer of event and
// 			 * copy into app structures */
// 			ctrl_app_parse_event(proto_msg, app_event);
// 
// 			/* callback to registered function */
// 			call_event_callback(app_event);
// 
// 			//CLEANUP_APP_MSG(app_event);
// 		} else {
// 			/* silently drop */
			goto err;
// 		}
        }
        
	/* 3. Check if it is response msg */
	else if (resp->msg_type == CTRL_MSG_TYPE_RESP) {

		/* Ctrl responses are handled asynchronously and
		 * asynchronpusly */

		nrc_usr_print("[%s] CTRL_MSG_TYPE_RESP msg_id %d\n", __func__, resp->msg_id);
		memset(cmd, 0, sizeof(ctrl_cmd_t));

		/* If this was async procedure, timer would have
		 * been running for response.
		 * As response received, stop timer */
// 		if (async_timer_handle) {
// 			/* async_timer_handle will be cleaned in hosted_timer_stop */
// 			hosted_timer_stop(async_timer_handle);
// 			async_timer_handle = NULL;
// 		}

		/* Decode protobuf buffer of response and
		 * copy into app structures */
// 		ctrl_app_parse_resp(proto_msg, app_resp);

		/* Is callback is available,
		 * progress as async response */
// 		if (CALLBACK_AVAILABLE ==
// 			is_async_resp_callback_registered_by_resp_msg_id(app_resp->msg_id)) {
// 
// 			/* User registered control async response callback
// 			 * function is available for this proto_msg,
// 			 * so call to that function should be done and
// 			 * return to select
// 			 */
// 			call_async_resp_callback(app_resp);
// 
// 			//CLEANUP_APP_MSG(app_resp);
// 
// 		} else {

			/* as control async response callback function is
			 * NOT available/registered, treat this response as
			 * synchronous response. forward this response to app
			 * using 'esp_queue' and help of semaphore
			 **/

// 			elem = (esp_queue_elem_t*)hosted_malloc(sizeof(esp_queue_elem_t));
// 			if (!elem) {
// 				nrc_usr_print("%s %u: Malloc failed\n",__func__,__LINE__);
// 				goto free_buffers;
// 			}

			/* User is RESPONSIBLE to free memory from
			 * app_resp in case of async callbacks NOT provided
			 * to free memory, please refer CLEANUP_APP_MSG macro
			 **/
// 			elem->buf = app_resp;
// 			elem->buf_len = sizeof(ctrl_cmd_t);
// 			if (esp_queue_put(ctrl_msg_Q, (void*)elem)) {
// 				nrc_usr_print("%s %u: ctrl Q put fail\n",__func__,__LINE__);
// 				goto free_buffers;
// 			}

			/* Call up rx ind to unblock user */
// 			if (ctrl_rx_func)
// 				ctrl_rx_func();
// 		}
// 		hosted_post_semaphore(ctrl_req_sem);

	} else {
		nrc_usr_print("[%s] CTRL_MSG_TYPE_ERROR\n", __func__);
		/* 4. some unsupported msg, drop it */
// 		nrc_usr_print("Incorrect Ctrl Msg Type[%u]\n",proto_msg->msg_type);
		goto err;
	}

	cmd->msg_type = CTRL_RESP;
	cmd->msg_id = resp->msg_id;

	nrc_usr_print("[%s] resp->msg_id %d", __func__, resp->msg_id);
	
	/* 3. parse CtrlMsg into ctrl_cmd_t */
	switch (resp->msg_id) {
		case CTRL_RESP_GET_MAC_ADDR : {
			uint8_t len_l = min(resp->payload.resp_get_mac_address.mac.size, MAX_MAC_STR_LEN-1);

			CHECK_CTRL_MSG_NON_NULL(resp_get_mac_address);
			CHECK_CTRL_MSG_NON_NULL(resp_get_mac_address.mac.bytes);
			CHECK_CTRL_MSG_FAILED(resp_get_mac_address);

			strncpy(cmd->u.wifi_mac.mac,
				(char *)resp->payload.resp_get_mac_address.mac.bytes, len_l);
			cmd->u.wifi_mac.mac[len_l] = '\0';
			break;
		} 
                case CTRL_RESP_SET_MAC_ADDRESS : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_mac_address);
			CHECK_CTRL_MSG_FAILED(resp_set_mac_address);
			break;
		} 
                case CTRL_RESP_GET_WIFI_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_wifi_mode);
			CHECK_CTRL_MSG_FAILED(resp_get_wifi_mode);

			cmd->u.wifi_mode.mode = resp->payload.resp_get_wifi_mode.mode;
			break;
		} 
                case CTRL_RESP_SET_WIFI_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_wifi_mode);
			CHECK_CTRL_MSG_FAILED(resp_set_wifi_mode);
			break;
		} 
                case CTRL_RESP_GET_AP_SCAN_LIST : {
			ctrl_msg_resp_scan_result_t *rp = &resp->payload.resp_scan_ap_list;
			wifi_ap_scan_list_t *ap = &cmd->u.wifi_ap_scan;
			wifi_scanlist_t *list = NULL;

			CHECK_CTRL_MSG_NON_NULL(resp_scan_ap_list);
			CHECK_CTRL_MSG_FAILED(resp_scan_ap_list);

			ap->count = rp->count;
			if (rp->count) {

				CHECK_CTRL_MSG_NON_NULL_VAL(ap->count,"No APs available");
				list = (wifi_scanlist_t *)calloc(ap->count,
						sizeof(wifi_scanlist_t));
				CHECK_CTRL_MSG_NON_NULL_VAL(list, "Malloc Failed");
			}

			for (int i=0; i<rp->count; i++) {

				if (rp->entries[i].ssid.size)
					memcpy(list[i].ssid, (char *)rp->entries[i].ssid.bytes,
						rp->entries[i].ssid.size);

				if (rp->entries[i].bssid.size)
					memcpy(list[i].bssid, (char *)rp->entries[i].bssid.bytes,
						rp->entries[i].bssid.size);

				list[i].channel = rp->entries[i].chnl;
				list[i].rssi = rp->entries[i].rssi;
				list[i].encryption_mode = rp->entries[i].sec_prot;
			}

			ap->out_list = list;
			/* Note allocation, to be freed later by app */
			cmd->free_buffer_func = free;
			cmd->free_buffer_handle = list;
			break;
		} 
                case CTRL_RESP_GET_AP_CONFIG : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_ap_config);
			wifi_ap_config_t *p = &cmd->u.wifi_ap_config;

			cmd->resp_event_status = resp->payload.resp_get_ap_config.resp;

			switch (resp->payload.resp_get_ap_config.resp) {

				case CTRL_ERR_NOT_CONNECTED:
					strncpy(p->status, NOT_CONNECTED_STR, STATUS_LENGTH);
					p->status[STATUS_LENGTH-1] = '\0';
					nrc_usr_print("Station is not connected to AP \n");
					goto err;
					break;

				case SUCCESS:
					strncpy(p->status, SUCCESS_STR, STATUS_LENGTH);
					p->status[STATUS_LENGTH-1] = '\0';
					if (resp->payload.resp_get_ap_config.ssid.size) {
						strncpy((char *)p->ssid,
								(char *)resp->payload.resp_get_ap_config.ssid.bytes,
								resp->payload.resp_get_ap_config.ssid.size);//MAX_SSID_LENGTH-1);
						//p->ssid[MAX_SSID_LENGTH-1] ='\0';
						p->ssid[resp->payload.resp_get_ap_config.ssid.size-1] ='\0';
					}
					if (resp->payload.resp_get_ap_config.bssid.size) {
						uint8_t len_l = 0;

						len_l = min(resp->payload.resp_get_ap_config.bssid.size,
								MAX_MAC_STR_LEN-1);
						strncpy((char *)p->bssid,
								(char *)resp->payload.resp_get_ap_config.bssid.bytes,
								len_l);
						p->bssid[len_l] = '\0';
					}

					p->channel = resp->payload.resp_get_ap_config.chnl;
					p->rssi = resp->payload.resp_get_ap_config.rssi;
					p->encryption_mode = resp->payload.resp_get_ap_config.sec_prot;
					break;

				case FAILURE:
				default:
					/* intentional fall-through */
					strncpy(p->status, FAILURE_STR, STATUS_LENGTH);
					p->status[STATUS_LENGTH-1] = '\0';
					nrc_usr_print("Failed to get AP config \n");
					goto err;
					break;
			}
			break;
		} 
                case CTRL_RESP_CONNECT_AP : {
			uint8_t len_l = 0;
			CHECK_CTRL_MSG_NON_NULL(resp_connect_ap);

			cmd->resp_event_status = resp->payload.resp_connect_ap.resp;

			switch(resp->payload.resp_connect_ap.resp) {
				case CTRL_ERR_INVALID_PASSWORD:
					nrc_usr_print("Invalid password for SSID\n");
					goto err;
					break;
				case CTRL_ERR_NO_AP_FOUND:
					nrc_usr_print("SSID: not found/connectable\n");
					goto err;
					break;
				case SUCCESS:
					CHECK_CTRL_MSG_NON_NULL(resp_connect_ap.mac.bytes);
					CHECK_CTRL_MSG_FAILED(resp_connect_ap);
					break;
				default:
					CHECK_CTRL_MSG_FAILED(resp_connect_ap);
					nrc_usr_print("Connect AP failed\n");
					goto err;
					break;
			}
			len_l = min(resp->payload.resp_connect_ap.mac.size, MAX_MAC_STR_LEN-1);
			strncpy(cmd->u.wifi_ap_config.out_mac,
					(char *)resp->payload.resp_connect_ap.mac.bytes, len_l);
			cmd->u.wifi_ap_config.out_mac[len_l] = '\0';
			break;
		} 
                case CTRL_RESP_DISCONNECT_AP : {
			CHECK_CTRL_MSG_NON_NULL(resp_disconnect_ap);
			CHECK_CTRL_MSG_FAILED(resp_disconnect_ap);
			break;
		} 
                case CTRL_RESP_GET_SOFTAP_CONFIG : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_softap_config);
			CHECK_CTRL_MSG_FAILED(resp_get_softap_config);

			if (resp->payload.resp_get_softap_config.ssid.bytes) {
				uint16_t len = resp->payload.resp_get_softap_config.ssid.size;
				uint8_t *data = resp->payload.resp_get_softap_config.ssid.bytes;
				uint8_t *app_str = cmd->u.wifi_softap_config.ssid;

				memcpy(app_str, data, len);
				if (len<MAX_SSID_LENGTH)
					app_str[len] = '\0';
				else
					app_str[MAX_SSID_LENGTH-1] = '\0';
			}

			if (resp->payload.resp_get_softap_config.pwd.bytes) {
				memcpy(cmd->u.wifi_softap_config.pwd,
						resp->payload.resp_get_softap_config.pwd.bytes,
						resp->payload.resp_get_softap_config.pwd.size);
				cmd->u.wifi_softap_config.pwd[resp->payload.resp_get_softap_config.pwd.size-1] = '\0';
			}

			cmd->u.wifi_softap_config.channel =
				resp->payload.resp_get_softap_config.chnl;
			cmd->u.wifi_softap_config.encryption_mode =
				resp->payload.resp_get_softap_config.sec_prot;
			cmd->u.wifi_softap_config.max_connections =
				resp->payload.resp_get_softap_config.max_conn;
			cmd->u.wifi_softap_config.ssid_hidden =
				resp->payload.resp_get_softap_config.ssid_hidden;
			cmd->u.wifi_softap_config.bandwidth =
				resp->payload.resp_get_softap_config.bw;

			break;
		} 
                case CTRL_RESP_SET_SOFTAP_VND_IE : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_softap_vendor_specific_ie);
			CHECK_CTRL_MSG_FAILED(resp_set_softap_vendor_specific_ie);
			break;
		} 
                case CTRL_RESP_START_SOFTAP : {
			uint8_t len_l = 0;
			CHECK_CTRL_MSG_NON_NULL(resp_start_softap);
			CHECK_CTRL_MSG_FAILED(resp_start_softap);
			CHECK_CTRL_MSG_NON_NULL(resp_start_softa.mac.bytes);

			len_l = min(resp->payload.resp_connect_ap.mac.size, MAX_MAC_STR_LEN-1);
			strncpy(cmd->u.wifi_softap_config.out_mac,
					(char *)resp->payload.resp_connect_ap.mac.bytes, len_l);
			cmd->u.wifi_softap_config.out_mac[len_l] = '\0';
			break;
		} 
                case CTRL_RESP_GET_SOFTAP_CONN_STA_LIST : {
			wifi_softap_conn_sta_list_t *ap = &cmd->u.wifi_softap_con_sta;
			wifi_connected_stations_list_t *list = ap->out_list;
			ctrl_msg_resp_soft_ap_connected_sta_t *rp =
				&resp->payload.resp_softap_connected_stas_list;

			CHECK_CTRL_MSG_FAILED(resp_softap_connected_stas_list);

			ap->count = rp->num;
			CHECK_CTRL_MSG_NON_NULL_VAL(ap->count,"No Stations connected");
			if(ap->count) {
				CHECK_CTRL_MSG_NON_NULL(resp_softap_connected_stas_list);
				list = (wifi_connected_stations_list_t *)calloc(
						ap->count, sizeof(wifi_connected_stations_list_t));
				CHECK_CTRL_MSG_NON_NULL_VAL(list, "Malloc Failed");
			}

			for (int i=0; i<ap->count; i++) {
				memcpy(list[i].bssid, (char *)rp->stations[i].mac.bytes,
						rp->stations[i].mac.size);
				list[i].rssi = rp->stations[i].rssi;
			}
			cmd->u.wifi_softap_con_sta.out_list = list;

			/* Note allocation, to be freed later by app */
			cmd->free_buffer_func = free;
			cmd->free_buffer_handle = list;

			break;
		} 
                case CTRL_RESP_STOP_SOFTAP : {
			CHECK_CTRL_MSG_NON_NULL(resp_stop_softap);
			CHECK_CTRL_MSG_FAILED(resp_stop_softap);
			break;
		} 
                case CTRL_RESP_SET_PS_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_set_power_save_mode);
			CHECK_CTRL_MSG_FAILED(resp_set_power_save_mode);
			break;
		} 
                case CTRL_RESP_GET_PS_MODE : {
			CHECK_CTRL_MSG_NON_NULL(resp_get_power_save_mode);
			CHECK_CTRL_MSG_FAILED(resp_get_power_save_mode);
			cmd->u.wifi_ps.ps_mode = resp->payload.resp_get_power_save_mode.mode;
			break;
		} 
                case CTRL_RESP_OTA_BEGIN : {
			CHECK_CTRL_MSG_NON_NULL(resp_ota_begin);
			CHECK_CTRL_MSG_FAILED(resp_ota_begin);
			if (resp->payload.resp_ota_begin.resp) {
				nrc_usr_print("OTA Begin Failed\n");
				goto err;
			}
			break;
		} 
                case CTRL_RESP_OTA_WRITE : {
			CHECK_CTRL_MSG_NON_NULL(resp_ota_write);
			CHECK_CTRL_MSG_FAILED(resp_ota_write);
			if (resp->payload.resp_ota_write.resp) {
				nrc_usr_print("OTA write failed\n");
				goto err;
			}
			break;
		} 
                case CTRL_RESP_OTA_END : {
			CHECK_CTRL_MSG_NON_NULL(resp_ota_end);
			if (resp->payload.resp_ota_end.resp) {
				nrc_usr_print("OTA write failed\n");
				goto err;
			}
			break;
		} 
                case CTRL_RESP_SET_WIFI_MAX_TX_POWER: {
			CHECK_CTRL_MSG_NON_NULL(req_set_wifi_max_tx_power);
			switch (resp->payload.resp_set_wifi_max_tx_power.resp)
			{
				case FAILURE:
					nrc_usr_print("Failed to set max tx power\n");
					goto err;
					break;
				case SUCCESS:
					break;
				case CTRL_ERR_OUT_OF_RANGE:
					nrc_usr_print("Power is OutOfRange. Check api doc for reference\n");
					goto err;
					break;
				default:
					nrc_usr_print("unexpected response\n");
					goto err;
					break;
			}
			break;
		} 
                case CTRL_RESP_GET_WIFI_CURR_TX_POWER: {
			CHECK_CTRL_MSG_NON_NULL(resp_get_wifi_curr_tx_power);
			CHECK_CTRL_MSG_FAILED(resp_get_wifi_curr_tx_power);
			cmd->u.wifi_tx_power.power =
				resp->payload.resp_get_wifi_curr_tx_power.wifi_curr_tx_power;
			break;
		} 
                case CTRL_RESP_CONFIG_HEARTBEAT: {
			CHECK_CTRL_MSG_NON_NULL(resp_config_heartbeat);
			CHECK_CTRL_MSG_FAILED(resp_config_heartbeat);
			break;
		} 
                default: {
			nrc_usr_print("Unsupported Control Resp[%u]\n", resp->msg_id);
			goto err;
			break;
		}
	}

	cmd->resp_event_status = SUCCESS;
        
  retval = NRC_SUCCESS;
  
err:

  return retval;
}

/**
  * @brief  RX virtual serial port task
  * @param  argument: Not used
  * @retval None
  */
static void rx_serial_task(void* pvParameters)
{
	uint16_t init_read_len = 0;
	int ret = 0;
        interface_buffer_handle_t buf_handle = {0};
        
	/* Any of `CTRL_EP_NAME_EVENT` and `CTRL_EP_NAME_RESP` could be used,
	 * as both have same strlen in adapter.h */
	const char* ep_name = CTRL_EP_NAME_RESP;
	uint32_t buf_len = 0;
        ctrl_cmd_t cmd = {0};

	nrc_usr_print("[%s]\n", __func__);
	
        while(true) // getReceiveSerialSem semaphore
        {
	/* This is **blocking** receive.
	 *
	 * Although not needed in normal circumstances,
	 * User can convert it to non blocking using below steps:
	 *
	 * To make it non blocking:
	 *   As an another design option, serial_rx_callback can also be
	 *   thought of incoming data indication, i.e. asynchronous rx
	 *   indication, which can be used by higher layer in seperate
	 *   dedicated rx task to receive and process rx data.
	 *
	 * In our example, first approach of blocking read is used.
	 */
        
// 	nrc_usr_print("[%s] xSemaphoreTake semaphore_rx_serial\n", __func__);
//         xSemaphoreTake( semaphore_rx_serial, ( TickType_t ) portMAX_DELAY );
        
	nrc_usr_print("[%s] xQueueReceive rx_queue_serial\n", __func__);
	if (pdTRUE != xQueueReceive(rx_queue_serial, &buf_handle, portMAX_DELAY)) {
		nrc_usr_print("[%s] serial queue recv failed\n", __func__);
		continue;
	}
	
	nrc_usr_print("[%s] rec'd queue entry\n", __func__);
	
// 	print_hex(&buf_handle, buf_handle.payload_len + sizeof(buf_handle));
	
// 	nrc_usr_print("[%s] xSemaphoreGive rx_queue_serial\n", __func__);
// 	xSemaphoreGive( semaphore_rx_serial);

	/* proceed only if payload and length are sane */
	if (!buf_handle.payload || !buf_handle.payload_len) {
		if(buf_handle.free_buf_handle && buf_handle.payload)
			buf_handle.free_buf_handle(buf_handle.payload);
		nrc_usr_print("[%s] payload length error\n", __func__);
		continue;
	}
/*
 * Read Operation happens in two steps because total read length is unkown
 * at first read.
 *      1) Read fixed length of RX data
 *      2) Read variable length of RX data
 *
 * (1) Read fixed length of RX data :
 * Read fixed length of received data in below format:
 * ----------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
 * ----------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 *  ---------------------------------------------------------------------------
 *      1         |       2         | Endpoint Length |     1     |     2     |
 *  ---------------------------------------------------------------------------
 *
 *  int_read_len = 1 + 2 + Endpoint length + 1 + 2
 */
	init_read_len = SIZE_OF_TYPE + SIZE_OF_LENGTH + strlen(ep_name) +
		SIZE_OF_TYPE + SIZE_OF_LENGTH;

	if(buf_handle.payload_len < init_read_len) {
		nrc_usr_print("[%s] Incomplete serial buf, return\n", __func__);
		if(buf_handle.free_buf_handle && buf_handle.payload)
			buf_handle.free_buf_handle(buf_handle.payload);
		continue;
	}
        
	/* parse_tlv function returns variable payload length
	 * of received data in buf_len
	 **/
	ret = parse_tlv(buf_handle.payload, &buf_len);
	if (ret || !buf_len) {
		if(buf_handle.free_buf_handle && buf_handle.payload)
			buf_handle.free_buf_handle(buf_handle.payload);
		nrc_usr_print("[%s] Failed to parse RX data\n", __func__);
		continue;
	}

	if (buf_handle.payload_len < (init_read_len + buf_len)) {
		nrc_usr_print("[%s] Buf read on serial iface is smaller than expected len\n", __func__);
		if(buf_handle.free_buf_handle)
			buf_handle.free_buf_handle(buf_handle.payload);
		continue;
	}
/*
 * (2) Read variable length of RX data:
 */
        
        // parse and handle 'serial' xface receive
        // cmd contains parsed data
        esp_recv_resp(buf_handle.payload+init_read_len, buf_len, &cmd);
	
	if(buf_handle.free_buf_handle && buf_handle.payload)
		buf_handle.free_buf_handle(buf_handle.payload);

        buf_len = 0;
        }

}

/**
  * @brief Serial rx handler is called when there
  *        is incoming data with interface type is Serial.
  * @param  if_num - interface instance
  *         rxbuff - buffer from spi driver
  *         rx_len - size of rxbuff
  *         seq_num - serial sequence number
  *         flag_more_frags - Flags for fragmentation
  * @retval 0 on success, else failure
  */

nrc_err_t serial_rx_handler(interface_buffer_handle_t * buf_handle)
{

#define SERIAL_ALLOC_REALLOC_RDATA() \
	do { \
		if(!r_data) { \
			r_data = (uint8_t *)malloc(buf_handle->payload_len); \
		} else { \
			r_data = (uint8_t *)realloc(r_data, r_len + buf_handle->payload_len); \
		} \
		if (!r_data) { \
			nrc_usr_print("Failed to allocate serial data\n\r"); \
			goto err; \
		} \
	} while(0);

	uint8_t *serial_buf = NULL;
	interface_buffer_handle_t new_buf_handle = {0};
	int r_len = 0;
	uint8_t *r_data = NULL;
	nrc_err_t ret = NRC_FAIL;
        
        nrc_usr_print("[%s]\n\r", __func__);

	/* Check valid handle and length */
	if (!buf_handle || !buf_handle->payload_len) {
		nrc_usr_print("%s:%u Invalid parameters\n\r", __func__, __LINE__);
		goto err;
	}

	/* Accumulate fragments */
	if (buf_handle->flag & MORE_FRAGMENT) {

                nrc_usr_print("%s:%u More fragments\n\r", __func__, __LINE__);
                
		SERIAL_ALLOC_REALLOC_RDATA();

		memcpy((r_data + r_len), buf_handle->payload, buf_handle->payload_len);
		r_len += buf_handle->payload_len;
		return NRC_SUCCESS;
	}

	SERIAL_ALLOC_REALLOC_RDATA();

	/* No or last fragment */
	memcpy((r_data + r_len), buf_handle->payload, buf_handle->payload_len);
	r_len += buf_handle->payload_len;

	serial_buf = (uint8_t *)malloc(r_len);
	if(!serial_buf) {
		nrc_usr_print("Malloc failed, drop pkt\n\r");
		goto err;
	}
	
	memcpy(serial_buf, r_data, r_len);

	/* form new buf handle for processing of serial msg */
	new_buf_handle.if_type = ESP_SERIAL_IF;
	new_buf_handle.if_num = buf_handle->if_num;
	new_buf_handle.payload_len = r_len;
	new_buf_handle.payload = serial_buf;
	new_buf_handle.priv_buffer_handle = serial_buf;
	new_buf_handle.free_buf_handle = free;

	r_len = 0;
	free(r_data);
	r_data = NULL;

        nrc_usr_print("[%s] xQueueSend rx_queue_serial\n", __func__);
        
	/* send to serial queue */
	if (pdTRUE != xQueueSend(rx_queue_serial,
		    &new_buf_handle, portMAX_DELAY)) {
		nrc_usr_print("Failed send serialif queue[%u]\n\r", new_buf_handle.if_num);
		goto err;
	}

	/*nrc_usr_print("[%s] xSemaphoreGive rx_queue_serial\n", __func__);
        xSemaphoreGive(rx_queue_serial);
        */        
// 	if( xSemaphoreTake( rx_queue_serial, ( TickType_t ) 10 ) != pdTRUE ) {
//           nrc_usr_print("[%s] could not sem take rx_queue_serial\n", __func__);
//         }

        
	/* Indicate higher layer about data ready for consumption */
// 	if (serial_ll_hdl->serial_rx_callback) {
// 		(*serial_ll_hdl->serial_rx_callback) ();
// 	} else {
// 		goto err;
// 	}

	ret = NRC_SUCCESS;

err:
	r_len = 0;
	
        if(serial_buf){
          free(serial_buf);
          serial_buf = NULL;
	}
	
	if(r_data){
          free(r_data);
          r_data = NULL;
        }
        
        nrc_usr_print("[%s] ret %d\n", __func__, ret);
	
	return ret;
}

/**
  * @brief  RX SPI task
  * @param  argument: Not used
  * @retval None
  */
static void rx_task(void* pvParameters)
{
  int gpio_rx_data_ready = GPIO_LEVEL_LOW;
  esp_err_t ret = ESP_OK;
  interface_buffer_handle_t buf_handle = {0};
  uint8_t *payload = NULL;
  struct pbuf *buffer = NULL;
  struct esp_priv_event *event = NULL;
  struct esp_private *priv = NULL;

  nrc_usr_print("[%s]\n", __func__);

  for (;;) 
  {
        nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready);

        while(gpio_rx_data_ready == GPIO_LEVEL_HIGH) 
        {
          nrc_usr_print("[%s] receive\n", __func__);

          if(spi_transaction() != NRC_SUCCESS) {
            return;
          }
          
          nrc_usr_print("[%s] before xQueueReceive\n", __func__);
          
          if (pdTRUE != xQueueReceive(rx_queue, &buf_handle, portMAX_DELAY)){
            break;
          }
          
           nrc_usr_print("[%s] xQueueReceive received %d bytes\n", __func__, buf_handle.payload_len);
	   
// 	   print_hex(&buf_handle, sizeof(buf_handle) + buf_handle.payload_len);
          
          /* point to payload */
          payload = buf_handle.payload;
          
          /* process received buffer for all possible interface types */
          if (buf_handle.if_type == ESP_SERIAL_IF) {
            
            nrc_usr_print("[%s] ESP_SERIAL_IF - calling serial_rx_handler\n", __func__);
            
            /* serial interface path */
            serial_rx_handler(&buf_handle);

          } 
          else if((buf_handle.if_type == ESP_STA_IF) ||
                    (buf_handle.if_type == ESP_AP_IF)) {
            
            nrc_usr_print("[%s] ESP_AP_IF || ESP_STA_IF\n", __func__);
            // hand to lwip layer above
//             priv = get_priv(buf_handle.if_type, buf_handle.if_num);
//             
//             if (priv) {
              buffer = (struct pbuf *)malloc(sizeof(struct pbuf));
              assert(buffer);
              buffer->len = buf_handle.payload_len;
              buffer->payload = malloc(buf_handle.payload_len);
              assert(buffer->payload);
//               
              memcpy(buffer->payload, buf_handle.payload, buf_handle.payload_len);
//               // network receive call
//               netdev_rx(priv->netdev, buffer);
//             }

          } 
          else if (buf_handle.if_type == ESP_PRIV_IF) {
            uint8_t *pos = NULL;

            nrc_usr_print("[%s] ESP_PRIV_IF\n", __func__);
            
            event = (struct esp_priv_event *) (payload);

            if (event->event_type == ESP_PRIV_EVENT_INIT) {
		    pos = event->event_data;
		    nrc_usr_print("[%s] Chip ID 0x%X SPI CLK %d Cap 0x%X\n", __func__, *(pos+2), *(pos+5), *(pos+8));
            }
            
            if (buf_handle.free_buf_handle) {
                buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
            }
          }
            
          nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready);
        }
        
        nrc_usr_print("[%s] xSemaphoreTake semaphore_rx\n", __func__);
        xSemaphoreTake( semaphore_rx, ( TickType_t ) portMAX_DELAY );
  }
}

/* This is entry level function when control request APIs are used
 * This function will encode control request in protobuf and send to ESP32
 * It will copy application structure `ctrl_cmd_t` to
 * protobuf control req `ctrl_msg_t`
 **/
nrc_err_t esp_send_req(ctrl_cmd_t *cmd)
{
	int       ret = NRC_FAIL;
	ctrl_msg_t ctrl = CTRL_MSG_INIT_ZERO;
	uint8_t  *payload_buf = NULL;
	void     *vendor_data_buf = NULL;
	uint8_t   failure_status = 0;
	pb_ostream_t ostream;
	interface_buffer_handle_t buf_handle = {0};
	char* ep_name = CTRL_EP_NAME_RESP;
	int tlv_init_len = SIZE_OF_TYPE + SIZE_OF_LENGTH + strlen(ep_name) + SIZE_OF_TYPE + SIZE_OF_LENGTH;
	uint8_t  tx_data[sizeof(ctrl)+tlv_init_len];
	
	if (!cmd) {
		failure_status = CTRL_ERR_INCORRECT_ARG;
		goto err;
	}

	ostream = pb_ostream_from_buffer(tx_data, sizeof(tx_data));

	/* 1. Check if any ongoing request present
	 * Send failure in that case */
// 	ret = hosted_get_semaphore(ctrl_req_sem, WAIT_TIME_B2B_CTRL_REQ);
// 	if (ret) {
// 		failure_status = CTRL_ERR_REQ_IN_PROG;
// 		goto err;
// 	}

	cmd->msg_type = CTRL_REQ;

	/* 2. Protobuf msg init */
// 	ctrl_msg__init(&ctrl);

	ctrl.msg_id = cmd->msg_id;
	/* payload case is exact match to msg id in esp_hosted_config.pb-c.h */
// 	ctrl.payload_case = (CtrlMsg_PayloadCase) cmd->msg_id;

	/* 3. identify request and compose CtrlMsg */
	switch(ctrl.msg_id) {
		case CTRL_REQ_GET_WIFI_MODE:
		case CTRL_REQ_GET_AP_CONFIG:
		case CTRL_REQ_DISCONNECT_AP:
		case CTRL_REQ_GET_SOFTAP_CONFIG:
		case CTRL_REQ_GET_SOFTAP_CONN_STA_LIST:
		case CTRL_REQ_STOP_SOFTAP:
		case CTRL_REQ_GET_PS_MODE:
		case CTRL_REQ_OTA_BEGIN:
		case CTRL_REQ_OTA_END:
		case CTRL_REQ_GET_WIFI_CURR_TX_POWER: {
			/* Intentional fallthrough & empty */
			break;
		} case CTRL_REQ_GET_AP_SCAN_LIST: {
			if (cmd->cmd_timeout_sec < DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT)
				cmd->cmd_timeout_sec = DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT;
			break;
		} case CTRL_REQ_GET_MAC_ADDR: {
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_get_mac_address_t, req_get_mac_address);

			if ((cmd->u.wifi_mac.mode <= ESP_WIFI_MODE_NONE) ||
			    (cmd->u.wifi_mac.mode >= ESP_WIFI_MODE_APSTA)) {
				nrc_usr_print("Invalid parameter\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
// 			ctrl_msg__req__get_mac_address__init(req_payload);

			req_payload->mode = cmd->u.wifi_mac.mode;

			break;
		} case CTRL_REQ_SET_MAC_ADDR: {
			wifi_mac_t * p = &cmd->u.wifi_mac;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_set_mac_address_t, req_set_mac_address);

			if ((p->mode <= ESP_WIFI_MODE_NONE) ||
			    (p->mode >= ESP_WIFI_MODE_APSTA)||
			    (!strlen(p->mac)) ||
			    (strlen(p->mac) > MAX_MAC_STR_LEN)) {
				nrc_usr_print("Invalid parameter\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
// 			ctrl_msg__req__set_mac_address__init(req_payload);
// 			*req_payload = CTRL_MSG_REQ_SET_MAC_ADDRESS_INIT_ZERO;

			req_payload->mode = p->mode;
			req_payload->mac.size = min(strlen(p->mac), MAX_MAC_STR_LEN);
			memcpy(req_payload->mac.bytes, (uint8_t *)p->mac, req_payload->mac.size);

			break;
		} case CTRL_REQ_SET_WIFI_MODE: {
			wifi_mode_t * p = &cmd->u.wifi_mode;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_set_mode_t, req_set_wifi_mode);

			if ((p->mode < ESP_WIFI_MODE_NONE) || (p->mode >= ESP_WIFI_MODE_MAX)) {
				nrc_usr_print("Invalid wifi mode\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
			req_payload->mode = p->mode;
// 			ctrl_msg__req__set_mode__init(req_payload);
// 			*req_payload = CTRL_MSG_REQ_SET_MODE_INIT_ZERO;
			ctrl.which_payload = CTRL_MSG_REQ_SET_MODE_MODE_TAG;
			break;
		} case CTRL_REQ_CONNECT_AP: {
			wifi_ap_config_t * p = &cmd->u.wifi_ap_config;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_connect_ap_t, req_connect_ap);

			if ((strlen((char *)p->ssid) > MAX_SSID_LENGTH) ||
					(!strlen((char *)p->ssid))) {
				nrc_usr_print("Invalid SSID length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if (strlen((char *)p->pwd) > MAX_PWD_LENGTH) {
				nrc_usr_print("Invalid password length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if (strlen((char *)p->bssid) > MAX_MAC_STR_LEN) {
				nrc_usr_print("Invalid BSSID length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
// 			ctrl_msg__req__connect_ap__init(req_payload);
// 			*payload = CTRL_MSG_REQ_CONNECT_AP_INIT_ZERO;

			memcpy(req_payload->ssid, (char *)&p->ssid, strlen((char *)p->ssid));
			memcpy(req_payload->pwd, (char *)&p->pwd, strlen((char *)p->pwd));
			memcpy(req_payload->bssid, (char *)&p->bssid, strlen((char *)p->bssid));
			req_payload->is_wpa3_supported = p->is_wpa3_supported;
			req_payload->listen_interval = p->listen_interval;

			ctrl.which_payload = CTRL_MSG_REQ_CONNECT_AP_TAG;			
			break;
		} case CTRL_REQ_SET_SOFTAP_VND_IE: {
			wifi_softap_vendor_ie_t *p = &cmd->u.wifi_softap_vendor_ie;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_set_soft_ap_vendor_specific_ie_t,
					req_set_softap_vendor_specific_ie);

			if ((p->type > WIFI_VND_IE_TYPE_ASSOC_RESP) ||
			    (p->type < WIFI_VND_IE_TYPE_BEACON)) {
				nrc_usr_print("Invalid vendor ie type \n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if ((p->idx > WIFI_VND_IE_ID_1) || (p->idx < WIFI_VND_IE_ID_0)) {
				nrc_usr_print("Invalid vendor ie ID index \n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if (!p->vnd_ie.payload) {
				nrc_usr_print("Invalid vendor IE buffer \n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
// 			ctrl_msg__req__set_soft_apvendor_specific_ie__init(req_payload);
// 			*req_payload = CTRL_MSG_REQ_SET_SOFT_AP_VENDOR_SPECIFIC_IE_INIT_ZERO;

			req_payload->enable = p->enable;
			req_payload->type = (ctrl_vendor_ie_type_t) p->type;
			req_payload->idx = (ctrl_vendor_ieid_t) p->idx;

// 			req_payload->vendor_ie_data = (CtrlMsgReqVendorIEData *)malloc(sizeof(CtrlMsgReqVendorIEData));

// 			if (!req_payload->vendor_ie_data) {
// 				nrc_usr_print("Mem alloc fail\n");
// 				goto err;
// 			}
// 			vendor_data_buf = req_payload->vendor_ie_data;

// 			ctrl_msg__req__vendor_iedata__init(req_payload->vendor_ie_data);

			req_payload->vendor_ie_data.element_id = p->vnd_ie.element_id;
			req_payload->vendor_ie_data.length = p->vnd_ie.length;
			memcpy(req_payload->vendor_ie_data.vendor_oui.bytes, p->vnd_ie.vendor_oui, VENDOR_OUI_BUF);
			req_payload->vendor_ie_data.vendor_oui.size = VENDOR_OUI_BUF;

			memcpy(req_payload->vendor_ie_data.payload.bytes, p->vnd_ie.payload, p->vnd_ie.payload_len);
			req_payload->vendor_ie_data.payload.size = p->vnd_ie.payload_len;
			break;
		} case CTRL_REQ_START_SOFTAP: {
			softap_config_t *p = &cmd->u.wifi_softap_config;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_start_soft_ap_t, req_start_softap);

			if ((strlen((char *)&p->ssid) > MAX_SSID_LENGTH) ||
			    (!strlen((char *)&p->ssid))) {
				nrc_usr_print("Invalid SSID length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if ((strlen((char *)&p->pwd) > MAX_PWD_LENGTH) ||
			    ((p->encryption_mode != ESP_WIFI_AUTH_OPEN) &&
			     (strlen((char *)&p->pwd) < MIN_PWD_LENGTH))) {
				nrc_usr_print("Invalid password length\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if ((p->channel < MIN_CHNL_NO) ||
			    (p->channel > MAX_CHNL_NO)) {
				nrc_usr_print("Invalid softap channel\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if ((p->encryption_mode < ESP_WIFI_AUTH_OPEN) ||
			    (p->encryption_mode == ESP_WIFI_AUTH_WEP) ||
			    (p->encryption_mode > ESP_WIFI_AUTH_WPA_WPA2_PSK)) {

				nrc_usr_print("Asked Encryption mode not supported\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if ((p->max_connections < MIN_CONN_NO) ||
			    (p->max_connections > MAX_CONN_NO)) {
				nrc_usr_print("Invalid maximum connection number\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

			if ((p->bandwidth < WIFI_BW_HT20) ||
			    (p->bandwidth > WIFI_BW_HT40)) {
				nrc_usr_print("Invalid bandwidth\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
// 			ctrl_msg__req__start_soft_ap__init(req_payload);

			memcpy(req_payload->ssid, (char *)&p->ssid, strlen((char *)&p->ssid));
			memcpy(req_payload->pwd, (char *)&p->pwd, strlen((char *)&p->pwd));
			req_payload->chnl = p->channel;
			req_payload->sec_prot = p->encryption_mode;
			req_payload->max_conn = p->max_connections;
			req_payload->ssid_hidden = p->ssid_hidden;
			req_payload->bw = p->bandwidth;
			break;
		} case CTRL_REQ_SET_PS_MODE: {
			wifi_power_save_t * p = &cmd->u.wifi_ps;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_set_mode_t, req_set_power_save_mode);

			if ((p->ps_mode < WIFI_PS_MIN_MODEM) ||
			    (p->ps_mode >= WIFI_PS_INVALID)) {
				nrc_usr_print("Invalid power save mode\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}
// 			ctrl_msg__req__set_mode__init(req_payload);

			req_payload->mode = p->ps_mode;
			break;
		} case CTRL_REQ_OTA_WRITE: {
			ota_write_t *p = & cmd->u.ota_write;
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_ota_write_t, req_ota_write);

			if (!p->ota_data || (p->ota_data_len == 0)) {
				nrc_usr_print("Invalid parameter\n");
				failure_status = CTRL_ERR_INCORRECT_ARG;
				goto err;
			}

// 			ctrl_msg__req__otawrite__init(req_payload);
			memcpy(req_payload->ota_data.bytes, p->ota_data,p->ota_data_len);
			req_payload->ota_data.size = p->ota_data_len;
			break;
		} case CTRL_REQ_SET_WIFI_MAX_TX_POWER: {
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_set_wifi_max_tx_power_t,
					req_set_wifi_max_tx_power);
// 			ctrl_msg__req__set_wifi_max_tx_power__init(req_payload);
			req_payload->wifi_max_tx_power = cmd->u.wifi_tx_power.power;
			break;
		} case CTRL_REQ_CONFIG_HEARTBEAT: {
			CTRL_ALLOC_ASSIGN(ctrl_msg_req_config_heartbeat_t, req_config_heartbeat);
// 			ctrl_msg__req__config_heartbeat__init(req_payload);
			req_payload->enable = cmd->u.e_heartbeat.enable;
			req_payload->duration = cmd->u.e_heartbeat.duration;
			if (req_payload->enable) {
				nrc_usr_print("Enable heartbeat with duration %ld\n", (long int)req_payload->duration);
// 				if (CALLBACK_AVAILABLE != is_event_callback_registered(CTRL_EVENT_HEARTBEAT))
// 					nrc_usr_print("Note: ** Subscribe heartbeat event to get notification **\n");
			} else {
				nrc_usr_print("Disable Heartbeat\n");
			}
			break;
		} default: {
			failure_status = CTRL_ERR_UNSUPPORTED_MSG;
			nrc_usr_print("Unsupported Control Req[%u]",ctrl.msg_id);
			goto err;
			break;
		}
	}

	/* 4. Protobuf msg size */
// 	tx_len = ctrl_msg__get_packed_size(&ctrl);
// 	if (!tx_len) {
// 		nrc_usr_print("Invalid tx length\n");
// 		failure_status = CTRL_ERR_PROTOBUF_ENCODE;
// 		goto err;
// 	}

	/* 5. Allocate protobuf msg */
// 	tx_data = (uint8_t *)calloc(1, tx_len);
// 	if (!tx_data) {
// 		nrc_usr_print("Failed to allocate memory for tx_data\n");
// 		failure_status = CTRL_ERR_MEMORY_FAILURE;
// 		goto err;
// 	}

	/* 6. Assign response callback
	 * a. If the response callback is not set, this will reset the
	 *    callback to NULL.
	 * b. If the non NULL response is assigned, this will set the
	 *    callback to user defined callback function */
// 	ret = set_async_resp_callback(req->msg_id, req->ctrl_resp_cb);
// 	if (ret < 0) {
// 		nrc_usr_print("could not set callback for req[%u]\n",req.msg_id);
// 		failure_status = CTRL_ERR_SET_ASYNC_CB;
// 		goto err;
// 	}

	/* 7. Start timeout for response for async only
	 * For sync procedures, hosted_get_semaphore takes care to
	 * handle timeout situations */
// 	if (req->ctrl_resp_cb) {
// 		async_timer_handle = hosted_timer_start(req->cmd_timeout_sec, CTRL__TIMER_ONESHOT,
// 				ctrl_async_timeout_handler, req->ctrl_resp_cb);
// 		if (!async_timer_handle) {
// 			nrc_usr_print("Failed to start async resp timer\n");
// 			goto err;
// 		}
// 	}

	/* 8. Pack in protobuf and send the request */
// 	ctrl_msg__pack(&ctrl, tx_data);
// 	if (transport_pserial_send(tx_data, tx_len)) {
// 		nrc_usr_print("Send control req[%u] failed\n",req.msg_id);
// 		failure_status = CTRL_ERR_TRANSPORT_SEND;
// 		goto err;
// 	}
	
	pb_encode_ex(&ostream, CTRL_MSG_FIELDS, &ctrl, PB_ENCODE_NULLTERMINATED);
	nrc_usr_print("[%s] ostream.bytes_written %d\n", __func__, ostream.bytes_written);
	print_hex(tx_data, ostream.bytes_written);

	memset(&buf_handle, 0, sizeof(buf_handle));
	buf_handle.if_type = ESP_SERIAL_IF;
	buf_handle.if_num = 0;
	buf_handle.payload = calloc(1, sizeof(tx_data));
	buf_handle.free_buf_handle = free;
	buf_handle.priv_buffer_handle = buf_handle.payload;
  
	if(!buf_handle.payload){
		nrc_usr_print("[%s] calloc error\n", __func__);
		goto err;
	}
		
	// wrap in tlv and push to tx_queue, then call spi_transaction
	buf_handle.payload_len = compose_tlv(buf_handle.payload, tx_data, ostream.bytes_written);
	
	nrc_usr_print("[%s] send to tx_queue buf_handle.payload_len %d\n", __func__, buf_handle.payload_len);
	
	print_hex(buf_handle.payload, buf_handle.payload_len);
	
	if (pdTRUE != xQueueSend(tx_queue, &buf_handle, portMAX_DELAY)) {
		nrc_usr_print("Failed to send buffer to_slave_queue\n\r");
		goto err;
	}
        
        spi_transaction();
	
        ret = NRC_SUCCESS;

err:	


	if (cmd->ctrl_resp_cb) {
		/* 11. In case of async procedure,
		 * Let application know of failure using callback itself
		 **/
// 		ctrl_cmd_t *app_resp = NULL;
// 		app_resp = (ctrl_cmd_t *)malloc(sizeof(ctrl_cmd_t));
// 		if (!app_resp) {
// 			nrc_usr_print("Failed to allocate app_resp\n");
// 			goto err2;
// 		}
// 		memset(app_resp, 0, sizeof(ctrl_cmd_t));
// 		cmd->msg_type = CTRL_RESP;
// 		cmd->msg_id = (cmd->msg_id - CTRL_REQ_BASE + CTRL_RESP_BASE);
// 		cmd->resp_event_status = failure_status;

		/* 12. In async procedure, it is important to get
		 * some kind of acknowledgement to user */
// 		cmd->ctrl_resp_cb(cmd);
        }
        
// 	if (cmd->free_buffer_handle) {
// 		if (cmd->free_buffer_func) {
// 			cmd->free_buffer_func(cmd->free_buffer_handle);
// 		}
// 	}
	
// 	if(tx_data)
//           free(tx_data);
//         if(vendor_data_buf)
//           free(vendor_data_buf);
//         if(payload_buf)
//           free(payload_buf);
	
	return ret;
}

/**
  * @brief  Set Wi-Fi mode
  * @param  cmd
  * @retval NRC_SUCCESS / NRC_FAIL
  */
nrc_err_t set_wifi_mode(ctrl_cmd_t *cmd)
{
  nrc_usr_print("[%s]\n", __func__);
  
  if(!cmd)
    return NRC_FAIL;
  
  cmd->msg_id = CTRL_REQ_SET_WIFI_MODE;
  cmd->u.wifi_mode.mode = ESP_WIFI_MODE_AP;
  
  return esp_send_req(cmd);
}

nrc_err_t netdev_transmit(esp32_mac_t* esp32_mac, uint8_t* buffer, uint16_t length)
{
  interface_buffer_handle_t buf_handle = {0};
  uint8_t iface_type = 0;
  uint8_t iface_num = 0; 
  
  nrc_usr_print("[%s]\n", __func__);
  
  if(!buffer || !length || length > MAX_ESP_PAYLOAD_SIZE)
    return NRC_FAIL;
  
  memset(&buf_handle, 0, sizeof(buf_handle));
  buf_handle.if_type = iface_type;
  buf_handle.if_num = iface_num;
  buf_handle.payload_len = length;
  buf_handle.payload = buffer;
  buf_handle.priv_buffer_handle = buffer;
  buf_handle.free_buf_handle = NULL;
  

  if (pdTRUE != xQueueSend(tx_queue, &buf_handle, portMAX_DELAY)) {
    nrc_usr_print("Failed to send buffer to_slave_queue\n\r");
    return ESP_FAIL;
  }
	
  return spi_transaction();
}

nrc_err_t netdev_recv(esp32_mac_t* esp32_mac, uint8_t* buffer, uint16_t* length)
{
  return NRC_FAIL;
}

nrc_err_t netdev_mac(esp32_mac_t *mac, uint8_t *addr)
{
  return NRC_FAIL;
}

/**
  * @brief  Alloc ESP32 hosted mac struct
  * @param  None
  * @retval mac struct
  */
esp32_mac_t* esp_hosted_new_mac(void)
{
  if(!esp32_mac){
    esp32_mac = malloc(sizeof(esp32_mac_t));
    
    if(!esp32_mac)
      return NULL;
  }
  
  esp32_mac->transmit = netdev_transmit;
  esp32_mac->receive = netdev_recv;
  esp32_mac->get_addr = netdev_mac;
  
  return esp32_mac;
}

/**
  * @brief  Reset ESP32
  * @param  None
  * @retval None
  */
static void reset_esp32(void)
{
  NRC_GPIO_CONFIG config;
  int gpio_reset_pin_val = 0;
    
  nrc_usr_print("[%s]\n", __func__);

  config.gpio_pin = GPIO_RESET_PIN;
  config.gpio_dir = GPIO_OUTPUT;
  config.gpio_alt = GPIO_NOMAL_OP;
  config.gpio_mode = GPIO_FLOATING;
  nrc_gpio_config(&config);
  
  nrc_gpio_outputb(GPIO_RESET_PIN, GPIO_LEVEL_LOW);
  _delay_ms(50);

  config.gpio_pin = GPIO_RESET_PIN;
  config.gpio_dir = GPIO_INPUT;
  config.gpio_alt = GPIO_NOMAL_OP;
  config.gpio_mode = GPIO_FLOATING;
  nrc_gpio_config(&config);

  /* stop spi transactions short time to avoid slave sync issues */
  _delay_ms(3000);
  
  nrc_gpio_inputb(GPIO_RESET_PIN, &gpio_reset_pin_val);
  
  nrc_usr_print("[%s] gpio_reset %d\n", __func__, gpio_reset_pin_val);
  
}

/**
 * @brief Init TD-XPAH Wi-Fi GW hAT
 *
 * Initialize Wi-Fi gateway add-on board
 *
 * @param none
 * @returns nrc_err_t
 */

#define RX_TASK_STACK_SIZE 2048
nrc_err_t init_wifi_gw(void)
{
  tx_queue = xQueueCreate(5, sizeof(interface_buffer_handle_t)); 
  assert(tx_queue);
  
  rx_queue = xQueueCreate(5, sizeof(interface_buffer_handle_t));
  assert(rx_queue);

  rx_queue_serial = xQueueCreate(TO_SERIAL_INFT_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
  assert(rx_queue_serial);

  mutex_spi = xSemaphoreCreateMutex();
  assert(mutex_spi);
  
  semaphore_rx = xSemaphoreCreateBinary();
  assert(semaphore_rx);
  
  semaphore_rx_serial = xSemaphoreCreateBinary();
  assert(semaphore_rx_serial);
  
  xTaskCreate(rx_task,
              "rx_task",
              RX_TASK_STACK_SIZE,
              NULL,
              configMAX_PRIORITIES - 1,
              &rx_task_id);

  xTaskCreate(rx_serial_task,
              "rx_serial_task",
              RX_TASK_STACK_SIZE,
              NULL,
              configMAX_PRIORITIES - 2,
              &rx_serial_task_id);
  
  _delay_ms(100);
  
  nrc_gpio_register_interrupt_handler(GPIO_RX_READY_PIN, rx_spi_isr);
  
  return NRC_SUCCESS;
}

/**
 * @brief Shutdown TD-XPAH Wi-Fi GW hAT
 *
 * @param none
 * @returns nrc_err_t
 */
nrc_err_t shutdown_wifi_gw(void)
{
  nrc_spi_enable(&esp_spi, false);

  nrc_gpio_register_interrupt_handler(GPIO_RX_READY_PIN, NULL);
  
  if(tx_queue) {
    xQueueReset( tx_queue );
    vQueueDelete( tx_queue );
    tx_queue = NULL;
  }
  
  if(rx_queue) {
    xQueueReset( rx_queue );
    vQueueDelete( rx_queue );
    rx_queue = NULL;
  }
  
  if(rx_queue_serial){
    xQueueReset( rx_queue_serial );
    vQueueDelete( rx_queue_serial );
    rx_queue_serial = NULL;
  }
  
  if(rx_task_id)
    vTaskDelete( rx_task_id);
  
  if(rx_serial_task_id)
    vTaskDelete( rx_serial_task_id);
  
  if(mutex_spi)
    vSemaphoreDelete(mutex_spi);
  
  if(semaphore_rx)
    vSemaphoreDelete(semaphore_rx);
  
  if(semaphore_rx_serial)
    vSemaphoreDelete(semaphore_rx);
  
  return NRC_SUCCESS;
}

/**
 * @brief Init SPI & GPIO pings for GW hAT
 *
 * Initialize Wi-Fi gateway add-on board
 *
 * @param none
 * @returns nrc_err_t
 */
nrc_err_t init_spi(void)
{
  int i = 0;
  int rev;

  NRC_GPIO_CONFIG config;
  int gpio_handshake_pin_val = 0;
  int gpio_rx_data_ready_pin_val = 0;

  nrc_usr_print("[%s]\n", __func__);
  
  config.gpio_pin = GPIO_RX_READY_PIN;
  config.gpio_dir = GPIO_INPUT;
  config.gpio_alt = GPIO_FUNC;
  config.gpio_mode = GPIO_FLOATING;

  nrc_gpio_config(&config);

  config.gpio_pin = GPIO_HANDSHAKE_PIN;
  config.gpio_dir = GPIO_INPUT;
  config.gpio_alt = GPIO_FUNC;
  config.gpio_mode = GPIO_FLOATING;

  nrc_gpio_config(&config);

  /* set esp_spi config */
  esp_spi.pin_miso = 12;
  esp_spi.pin_mosi = 13;
  esp_spi.pin_cs = 14;
  esp_spi.pin_sclk = 15;
  esp_spi.frame_bits = SPI_BIT8;
  esp_spi.clock = 3000000;
  esp_spi.mode = SPI_MODE3;
  esp_spi.controller = SPI_CONTROLLER_SPI0;
  esp_spi.bit_order = SPI_MSB_ORDER;
  esp_spi.irq_save_flag = 0;
  esp_spi.isr_handler = NULL;

  nrc_spi_master_init(&esp_spi);
  nrc_spi_enable(&esp_spi, true);
  _delay_ms(100);

  wifi_init_done = true;
  
  return NRC_SUCCESS;
}


/**
 * @brief Init TD-XPAH Wi-Fi GW hAT
 *
 * Initialize Wi-Fi gateway add-on board
 *
 * @param none
 * @returns nrc_err_t
 */
nrc_err_t
td_check_wifi_gw_hat(void)
{
  int gpio_rx_data_ready_pin_val = GPIO_LEVEL_LOW;
  int gpio_handshake = GPIO_LEVEL_LOW;
  ctrl_cmd_t cmd;

  nrc_usr_print("[%s]\n", __func__);  
  reset_esp32();

  nrc_gpio_inputb(GPIO_HANDSHAKE_PIN, &gpio_handshake);
  
  // if GW present, handshake GPIO should be high
  if(gpio_handshake == GPIO_LEVEL_LOW) {
    return NRC_FAIL;
  }

  if(!wifi_init_done){  
    init_spi();
    init_wifi_gw();
  }

  if(wifi_gw_exists) {	    
    memset(&cmd, 0, sizeof(ctrl_cmd_t));
    td_init_esp_wifi(&cmd);
    return NRC_SUCCESS;
  }
  else {
    shutdown_wifi_gw();
  }
  
  return NRC_FAIL;
}
