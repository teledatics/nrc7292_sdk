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

extern const char html_base64[];

// global http server handle
httpd_handle_t http_server = NULL;

/**
 * @brief substitute settings in HTML
 * 
 * Macro substitution of current settings
 * 
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
                                case WIFI_SHORT_BCN_INTERVAL_SUBST:
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
                                default:
                                        break;
                        }

                        nrc_usr_print("subst %s in html\n", val);
                        subst_string(html, (char*) html_subst[i], val);
                }
        }

        return 0;
}

// global HTML buffer
static char html_buffer[HTML_BUF_LEN];

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
                                char ipv4[32] = "";
                                
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
	.user_ctx  = NULL
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
                setup_page.user_ctx = update_settings.user_ctx = (void*)tf_config;
		httpd_register_uri_handler(http_server, &setup_page);
		httpd_register_uri_handler(http_server, &update_settings);
                nrc_usr_print("[%s]: http server started\n", __func__);
                return http_server;
	}
	
	nrc_usr_print("[%s]: error, returning NULL\n", __func__);
        
        return NULL;
}
