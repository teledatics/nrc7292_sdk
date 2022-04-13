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
 * @file teledatics_gui_http.c
 * @author James Ewing
 * @date 27 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */

#include "teledatics_gui.h"
#include "teledatics_base64_html.h"
#include "teledatics_gui_air_quality.h"

extern const char html_base64[];

// global http server handle
httpd_handle_t http_server = NULL;

// global HTML buffer
static char html_buffer[HTML_BUF_LEN];

/**
 * @brief get air quality sensor values
 * 
 * Get values from air quality sensor data
 * 
 * @param data buffer
 * @param len of buffer
 * @returns int length of data
 */
int td_get_air_quality_data(char* data, int len)
{
        float temp;
        float humidity;
        float co2;
        float voc;
        
        temp = td_get_air_quality_temperature();
        humidity = td_get_air_quality_humidity();
        
        td_set_absolute_humidity(temp, humidity);
        
        co2 = td_get_air_quality_co2();
        voc = td_get_air_quality_voc();
                
        return snprintf(data, len, "temp %f C humidity %f %% co2 %f voc %f", temp, humidity, co2, voc);
}

/**
 * @brief substitute settings in HTML
 * 
 * Macro substitution of current settings
 * 
 * @param HTML buffer
 * @param Wifi settings
 * @returns int, 0 on success
 */
int subst_wifi_values(char *html, td_wifi_config_t *tf_config)
{
        char val[255] = {0};
        char* subst = NULL;
        int i =0;
        
        for(i=0;i<WIFI_SUBST_SIZE;i++)
        {
                subst = strstr(html, html_subst[i]);

                if(subst) {
                        nrc_usr_print("found %s in html\n", html_subst[i]);
                        switch(i) {
                                case WIFI_MODE_SUBST:
                                        if(WIFI_MODE_STATION == tf_config->wifi_mode) {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", "sta");
                                        }
                                        else if(WIFI_MODE_MESH_POINT == tf_config->wifi_mode) {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", "mesh");
                                        }
                                        else {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", "ap");
                                        }
                                        break;
                                case WIFI_SSID_SUBST:
                                        snprintf(val, sizeof(val)-1, "\"%s\"", tf_config->nrc_wifi_config.ssid);
                                        break;
                                case WIFI_SECURITY_SUBST:
                                        if(WIFI_SEC_WPA2 == tf_config->nrc_wifi_config.security_mode) {
                                                snprintf(val, sizeof(val)-1, "%s", "\"wpa2\"");
                                        }
                                        else if(WIFI_SEC_WPA3_OWE == tf_config->nrc_wifi_config.security_mode) {
                                                snprintf(val, sizeof(val)-1, "%s", "\"wpa3_owe\"");
                                        }
                                        else if(WIFI_SEC_WPA3_SAE == tf_config->nrc_wifi_config.security_mode) {
                                                snprintf(val, sizeof(val)-1, "%s", "\"wpa3_sae\"");
                                        }
                                        else {
                                                snprintf(val, sizeof(val)-1, "%s", "\"none\"");
                                        }
                                        break;
                                case WIFI_PASSWORD_SUBST:
                                        if(strlen((char*)tf_config->nrc_wifi_config.password) > 0) {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", tf_config->nrc_wifi_config.password);
                                        }
                                        else {
                                                snprintf(val, sizeof(val)-1, "\"\"");
                                        }
                                        break;
                                case WIFI_COUNTRY_SUBST:
                                        snprintf(val, sizeof(val)-1, "\"%s\"", tf_config->nrc_wifi_config.country);
                                        break;
                                case WIFI_CHANNEL_SUBST:
                                        snprintf(val, sizeof(val)-1, "%ld", tf_config->nrc_wifi_config.channel);
                                        break;
                                case WIFI_IPMODE_SUBST:
                                        if(WIFI_STATIC_IP == tf_config->nrc_wifi_config.ip_mode) {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", "static_ip");
                                        }
                                        else{
                                                snprintf(val, sizeof(val)-1, "\"%s\"", "dynamic_ip");
                                        }
                                        break;
                                case WIFI_STATICIP_SUBST:
                                        if(WIFI_MODE_STATION == tf_config->wifi_mode ||
                                           WIFI_MODE_MESH_POINT == tf_config->wifi_mode) {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", tf_config->nrc_wifi_config.static_ip);
                                        }
                                        else {
                                                snprintf(val, sizeof(val)-1, "\"%s\"", tf_config->nrc_wifi_config.ap_ip);
                                        }
                                        break;
                                case WIFI_DHCPSERVER_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->nrc_wifi_config.dhcp_server);
                                        break;
                                case WIFI_INTERVAL_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->nrc_wifi_config.interval);
                                        break;
                                case WIFI_SHORT_BCN_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->nrc_wifi_config.short_bcn_interval);
                                        break;
                                case WIFI_TXPOWER_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->nrc_wifi_config.tx_power);
                                        break;
                                case WIFI_COUNT_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->nrc_wifi_config.count);
                                        break;
                                case WIFI_DURATION_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->nrc_wifi_config.duration);
                                        break;
                                case PPP_ENABLE_SUBST:
                                        snprintf(val, sizeof(val)-1, "%d", tf_config->ppp_enable);
                                        break;
                                case TDXPAH_ADDON_SUBST:
                                        snprintf(val, sizeof(val)-1, "%lld", tf_config->accessories);
                                        break;
                                default:
                                        break;
                        }

                        nrc_usr_print("subst %s in html\n", val);
                        subst_string(html, (char*) html_subst[i], val);
                }
        }

        return 0;
}

/**
 * @brief http configuration handler callback
 * 
 * Callback function that sends configuration page
 * 
 * @param http request
 * @returns esp error type
 */
esp_err_t setup_page_http(httpd_req_t *req)
{
        td_wifi_config_t *tf_config;
        size_t html_len;
        int ret;

        memset(html_buffer, 0, sizeof(html_buffer));

        ret = mbedtls_base64_decode((uint8_t*)html_buffer, sizeof(html_buffer), &html_len, 
                                    (uint8_t*)html_base64, strlen(html_base64));
        if(ret || html_len <= 0) {
                nrc_usr_print("base64 decode error ret %d html_len %d\n", ret, html_len);
                return ESP_ERR_HTTPD_INVALID_REQ;
        }

        tf_config = (td_wifi_config_t *)req->user_ctx;

        subst_wifi_values(html_buffer, tf_config);

        return httpd_resp_send(req, html_buffer, strlen(html_buffer));
}


httpd_uri_t setup_page = {
	.uri = "/",
	.method = HTTP_GET,
	.handler = setup_page_http,
        .is_websocket = false,
	.user_ctx = NULL
};

/**
 * @brief http update settings handler callback
 * 
 * Callback function that updates settings and saves to NV flash
 * 
 * @param http request
 * @returns esp error type
 */
esp_err_t update_settings_handler(httpd_req_t *req)
{
        td_wifi_config_t *tf_config;
        char tmp[255] = {0};
        int req_len;
        int network_index = -1;

        req_len = httpd_req_get_url_query_len(req) + 1;
        
        if (req_len < sizeof(html_buffer)) {
                
                tf_config = (td_wifi_config_t *)req->user_ctx;
                
                if (httpd_req_get_url_query_str(req, html_buffer, req_len) == ESP_OK) {

                        if (httpd_query_key_value(html_buffer, "wifi_mode", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_mode=%s\n", tmp);
                                if(!strncasecmp(tmp, "ap", sizeof("ap")-1))
                                        tf_config->wifi_mode = WIFI_MODE_AP;
                                else if(!strncasecmp(tmp, "sta", sizeof("sta")-1))
                                        tf_config->wifi_mode = WIFI_MODE_STATION;
                                else if(!strncasecmp(tmp, "mesh", sizeof("mesh")-1))
                                        tf_config->wifi_mode = WIFI_MODE_MESH_POINT;
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_ssid", (char *) tf_config->nrc_wifi_config.ssid, sizeof(tf_config->nrc_wifi_config.ssid)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => ssid=%s\n", tf_config->nrc_wifi_config.ssid);
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_security", tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => security=%s\n", tmp);

                                if (strncasecmp(tmp, "wpa2", sizeof("wpa2")-1) == 0) {
                                        tf_config->nrc_wifi_config.security_mode = WIFI_SEC_WPA2;
                                } 
                                else if (strncasecmp(tmp, "wpa3_owe", sizeof("wpa3_owe")-1) == 0) {
                                        tf_config->nrc_wifi_config.security_mode = WIFI_SEC_WPA3_OWE;
                                } 
                                else if (strncasecmp(tmp, "wpa3_sae", sizeof("wpa3_sae")-1) == 0) {
                                        tf_config->nrc_wifi_config.security_mode = WIFI_SEC_WPA3_SAE;
                                }
                                else {
                                        tf_config->nrc_wifi_config.security_mode = WIFI_SEC_OPEN;
                                }
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_password", (char *) tf_config->nrc_wifi_config.password, sizeof(tf_config->nrc_wifi_config.password)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => passwd=%s\n", tf_config->nrc_wifi_config.password);
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_country", (char *) tf_config->nrc_wifi_config.country, sizeof(tf_config->nrc_wifi_config.country)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => country=%s\n", tf_config->nrc_wifi_config.country);
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_channel", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_channel=%s\n", tmp);
                                tf_config->nrc_wifi_config.channel = atoi(tmp);
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_ip_mode", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_ip_mode=%s\n", tmp);
                                if (strncasecmp(tmp, "dynamic_ip", sizeof("dynamic_ip")-1) == 0) {
                                        tf_config->nrc_wifi_config.ip_mode = WIFI_DYNAMIC_IP;
                                }
                                else {
                                        tf_config->nrc_wifi_config.ip_mode = WIFI_STATIC_IP;
                                }
                        }

                        if (httpd_query_key_value(html_buffer, "wifi_static_ip1", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                char ipv4[16] = "";
                                
                                strcat(ipv4, tmp);
                                strcat(ipv4, ".");
                                
                                httpd_query_key_value(html_buffer, "wifi_static_ip2", (char *) tmp, sizeof(tmp));
                                
                                strcat(ipv4, tmp);
                                strcat(ipv4, ".");
                                
                                httpd_query_key_value(html_buffer, "wifi_static_ip3", (char *) tmp, sizeof(tmp));
                                
                                strcat(ipv4, tmp);
                                strcat(ipv4, ".");
                                
                                httpd_query_key_value(html_buffer, "wifi_static_ip4", (char *) tmp, sizeof(tmp));
                                
                                strcat(ipv4, tmp);
                                
                                nrc_usr_print ("Found URL query parameter => wifi_static_ip=%s\n", ipv4);
                                
                                snprintf((char*)tf_config->nrc_wifi_config.static_ip, sizeof(tf_config->nrc_wifi_config.static_ip), "%s", ipv4);
                                
                                snprintf((char*)tf_config->nrc_wifi_config.ap_ip, sizeof(tf_config->nrc_wifi_config.ap_ip), "%s", ipv4);
                                
                        }
                        
                        if (httpd_query_key_value(html_buffer, "wifi_dhcp", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_dhcp=%s\n", tmp);
                                
                                tf_config->nrc_wifi_config.dhcp_server = atoi(tmp);
                        }
                        
                        if (httpd_query_key_value(html_buffer, "wifi_interval", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_interval=%s\n", tmp);
                                
                                tf_config->nrc_wifi_config.interval = atoi(tmp);
                        }
                        
                        if (httpd_query_key_value(html_buffer, "wifi_bcn_interval", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_bcn_interval=%s\n", tmp);
                                
                                tf_config->nrc_wifi_config.short_bcn_interval = atoi(tmp);
                        }
                        
                        if (httpd_query_key_value(html_buffer, "wifi_txpower", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_txpower=%s\n", tmp);
                                
                                tf_config->nrc_wifi_config.tx_power = atoi(tmp);
                        }
                        
                        if (httpd_query_key_value(html_buffer, "wifi_count", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_count=%s\n", tmp);
                                
                                tf_config->nrc_wifi_config.count = atoi(tmp);
                        }
                        
                        if (httpd_query_key_value(html_buffer, "wifi_duration", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => wifi_duration=%s\n", tmp);
                                
                                tf_config->nrc_wifi_config.duration = atoi(tmp);
                        }
                        
                        if (httpd_query_key_value(html_buffer, "ppp_enable", (char *) tmp, sizeof(tmp)) == ESP_OK) {
                                nrc_usr_print ("Found URL query parameter => ppp_enable=%s\n", tmp);
                                
                                tf_config->ppp_enable = atoi(tmp);
                        }
                }

                td_validate_params(tf_config);
                td_save_wifi_config(tf_config);
                td_print_settings(tf_config);
                
                _delay_ms(1000);
                
                restart_system();
                
                return ESP_OK;
        }


        return ESP_FAIL;
}

httpd_uri_t update_settings = {
	.uri       = "/update_settings",
	.method    = HTTP_GET,
	.handler   = update_settings_handler,
        .is_websocket = false,
	.user_ctx  = NULL,
};

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    td_wifi_config_t *tf_config;
};

/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_send(void *arg)
{
        static char data[128] = "Async data";
        struct async_resp_arg *resp_arg = arg;
        httpd_handle_t hd = resp_arg->hd;
        int fd = resp_arg->fd;
        httpd_ws_frame_t ws_pkt;
        td_wifi_config_t *tf_config = resp_arg->tf_config;
    
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    
        if (has_air_quality_hat(tf_config)) {
                float temp = td_get_air_quality_temperature();
                float humidity = td_get_air_quality_humidity();
                
                snprintf(data, sizeof(data), "Async data temp %f humidity %f", temp, humidity);
        }
    
        ws_pkt.payload = (uint8_t*)data;
        ws_pkt.len = strlen(data);
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        httpd_ws_send_frame_async(hd, fd, &ws_pkt);
        
        free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req, td_wifi_config_t *tf_config)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    resp_arg->tf_config = tf_config;
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

/**
 * @brief http update settings handler callback
 * 
 * Callback function that updates settings and saves to NV flash
 * 
 * @param http request
 * @returns esp error type
 */
esp_err_t ws_handler(httpd_req_t *req)
{
        httpd_ws_frame_t ws_pkt;
        uint8_t *buf=NULL;
        esp_err_t ret;
        td_wifi_config_t *tf_config;
        
        tf_config = (td_wifi_config_t *)req->user_ctx;
                        
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        
        nrc_usr_print ("[%s]\n", __func__);
        
        if (req->method == HTTP_GET) {
                nrc_usr_print("Handshake done, new ws connection opened\n");
                return ESP_OK;
        }
    
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
        ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK) {
                nrc_usr_print("httpd_ws_recv_frame failed to get frame len with %d\n", ret);
                return ret;
        }
        
        nrc_usr_print("ws frame len is %d\n", ws_pkt.len);
        
        if (ws_pkt.len) {
                /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
                buf = calloc(1, ws_pkt.len + 1);
                if (buf == NULL) {
                        nrc_usr_print("Failed to calloc memory for buf\n");
                        return ESP_ERR_NO_MEM;
                }
                
                ws_pkt.payload = buf;
        
                /* Set max_len = ws_pkt.len to get the frame payload */
                ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
                if (ret != ESP_OK) {
                        nrc_usr_print("httpd_ws_recv_frame failed with %d\n", ret);
                        free(buf);
                        return ret;
                }
                nrc_usr_print("Got packet with message: %s\n", ws_pkt.payload);
        }
    
        nrc_usr_print("Packet type: %d\n", ws_pkt.type);
    
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
            strcmp((char*)ws_pkt.payload,"Trigger async") == 0) {
                free(buf);
                return trigger_async_send(req->handle, req, tf_config);
        }
        else{
                char data[128];
                memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    
                if (has_air_quality_hat(tf_config)) {
                        td_get_air_quality_data(data, sizeof(data));
                }
                else {
                        snprintf(data, sizeof(data), "No accessories found!");
                }
    
                ws_pkt.payload = (uint8_t*)data;
                ws_pkt.len = strlen(data);
                ws_pkt.type = HTTPD_WS_TYPE_TEXT;

                httpd_ws_send_frame(req, &ws_pkt);
        }

        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK) {
                nrc_usr_print("httpd_ws_send_frame failed with %d\n", ret);
        }
        
        free(buf);
        
        return ret;
}

httpd_uri_t update_ws_data = {
	.uri       = "/ws",
	.method    = HTTP_GET,
	.handler   = ws_handler,
        .is_websocket = true,
	.user_ctx  = NULL,
};

/**
 * @brief start http server
 * 
 * Run configuration GUI http server
 * 
 * @param ptr to configuration struct
 * @returns handle to running server
 */
httpd_handle_t run_http_server(td_wifi_config_t* tf_config)
{
        httpd_config_t conf = HTTPD_DEFAULT_CONFIG();

        if(http_server) {
                httpd_stop(http_server);
                http_server=NULL;
        }
        
        if (httpd_start(&http_server, &conf) == ESP_OK) {
                nrc_usr_print("httpd server on port : %d\n", conf.server_port);
        }

        if (http_server) {
                nrc_usr_print("[%s]: set up callbacks\n", __func__);
                setup_page.user_ctx = update_settings.user_ctx = update_ws_data.user_ctx = (void*)tf_config;
		httpd_register_uri_handler(http_server, &setup_page);
		httpd_register_uri_handler(http_server, &update_settings);
                httpd_register_uri_handler(http_server, &update_ws_data);
                nrc_usr_print("[%s]: http server started\n", __func__);
                return http_server;
	}
	
	nrc_usr_print("[%s]: error, returning NULL\n", __func__);
        
        return NULL;
}
