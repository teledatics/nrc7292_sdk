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
 * @file teledatics_gui_main.c
 * @author James Ewing
 * @date 24 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */

#include "teledatics_gui.h"

/**
 * @brief validate parameter settings
 * 
 * Validate critical wifi parameters, set to defaults if out of range
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_validate_params(td_wifi_config_t *tf_config)
{
        if(!tf_config)
                return NRC_FAIL;
        
        if(tf_config->nrc_wifi_config.count < WIFI_MIN_COUNT || 
           tf_config->nrc_wifi_config.count > WIFI_MAX_COUNT) {
                nrc_usr_print("count %d out of range %d to %d\n", tf_config->nrc_wifi_config.count, WIFI_MIN_COUNT, WIFI_MAX_COUNT);
                tf_config->nrc_wifi_config.count = TD_WIFI_COUNT_DEFAULT;
        }
        
        if(tf_config->nrc_wifi_config.interval < WIFI_MIN_INTERVAL || 
           tf_config->nrc_wifi_config.interval > WIFI_MAX_INTERVAL) {
                nrc_usr_print("interval %d out of range %d to %d\n", tf_config->nrc_wifi_config.interval, WIFI_MIN_INTERVAL, WIFI_MAX_INTERVAL);
                tf_config->nrc_wifi_config.interval = TD_WIFI_INTERVAL_DEFAULT;
        }
        
        if(tf_config->nrc_wifi_config.short_bcn_interval < WIFI_MIN_BCN_INTERVAL || 
           tf_config->nrc_wifi_config.short_bcn_interval > WIFI_MAX_BCN_INTERVAL) {
                nrc_usr_print("short_bcn_interval %d out of range %d to %d\n", tf_config->nrc_wifi_config.short_bcn_interval, WIFI_MIN_BCN_INTERVAL, WIFI_MAX_BCN_INTERVAL);
                tf_config->nrc_wifi_config.short_bcn_interval = TD_WIFI_SHORT_BCN_INTERVAL_DEFAULT;
        }
        
        if(tf_config->nrc_wifi_config.duration < WIFI_MIN_DURATION || 
           tf_config->nrc_wifi_config.duration > WIFI_MAX_DURATION) {
                nrc_usr_print("duration %d out of range %d to %d\n", tf_config->nrc_wifi_config.duration, WIFI_MIN_DURATION, WIFI_MAX_DURATION);
                tf_config->nrc_wifi_config.duration = TD_WIFI_DURATION_DEFAULT;
        }

        if(tf_config->nrc_wifi_config.tx_power < WIFI_MIN_TXPOWER || 
           tf_config->nrc_wifi_config.tx_power > WIFI_MAX_TXPOWER) {
                nrc_usr_print("tx_power %d out of range %d to %d\n", tf_config->nrc_wifi_config.tx_power, WIFI_MIN_TXPOWER, WIFI_MAX_TXPOWER);
                tf_config->nrc_wifi_config.tx_power = TD_WIFI_TXPOWER_DEFAULT;
        }

        return NRC_SUCCESS;
}

/**
 * @brief print out settings
 * 
 * Print settings to debug serial port
 * 
 * @param wifi configuration ptr
 * @returns none
 */
void td_print_settings(td_wifi_config_t *tf_config)
{
        nrc_usr_print("[%s] wifi settings:\n\n", __func__);
        nrc_usr_print("wifi_mode %d\n", tf_config->wifi_mode);
        nrc_usr_print("ssid %s\n", tf_config->nrc_wifi_config.ssid);
        nrc_usr_print("pasword %s\n", tf_config->nrc_wifi_config.password);
        nrc_usr_print("security %d\n", tf_config->nrc_wifi_config.security_mode);
        nrc_usr_print("country %s\n", tf_config->nrc_wifi_config.country);
        nrc_usr_print("channel %d\n", tf_config->nrc_wifi_config.channel);
        nrc_usr_print("ip_mode %d\n", tf_config->nrc_wifi_config.ip_mode);
        nrc_usr_print("static ip %s\n", tf_config->nrc_wifi_config.static_ip);
        nrc_usr_print("ap ip %s\n", tf_config->nrc_wifi_config.ap_ip);
        nrc_usr_print("static ipv6 %s\n", tf_config->ipv6_addr);
        nrc_usr_print("dhcp_server %d\n", tf_config->nrc_wifi_config.dhcp_server);
        nrc_usr_print("count %d\n", tf_config->nrc_wifi_config.count);
        nrc_usr_print("interval %d\n", tf_config->nrc_wifi_config.interval);
        nrc_usr_print("short bcn interval %d\n", tf_config->nrc_wifi_config.short_bcn_interval);
        nrc_usr_print("duration %d\n", tf_config->nrc_wifi_config.duration);
        nrc_usr_print("txpower %d\n", tf_config->nrc_wifi_config.tx_power);
        nrc_usr_print("ppp_enable %d\n", tf_config->ppp_enable);
}

/**
 * @brief save wifi configration data
 * 
 * Save wifi configration to key+value storage
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_save_wifi_config(td_wifi_config_t *tf_config)
{
        nvs_err_t err = NVS_OK;
        nvs_handle_t nvs_handle = 0;
        size_t length = 0;
        int32_t nvs_signed_int = 0;

        if(!tf_config)
                goto failed;

        err = nvs_open(NVS_DEFAULT_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(NVS_OK != err)
                goto failed;
        
        td_validate_params(tf_config);
        
        nvs_set_u8 (nvs_handle, TD_WIFI_MODE_TAG, tf_config->wifi_mode);
        nvs_set_str(nvs_handle, TD_WIFI_SSID_TAG, (char*) tf_config->nrc_wifi_config.ssid);
        nvs_set_str(nvs_handle, TD_WIFI_PASSWORD_TAG, (char*) tf_config->nrc_wifi_config.password);
        nvs_set_u8 (nvs_handle, TD_WIFI_SECURITY_TAG, tf_config->nrc_wifi_config.security_mode);
        nvs_set_str(nvs_handle, TD_WIFI_COUNTRY_TAG, (char*) tf_config->nrc_wifi_config.country);
        nvs_set_u32 (nvs_handle, TD_WIFI_CHANNEL_TAG, tf_config->nrc_wifi_config.channel);
        nvs_set_u8 (nvs_handle, TD_WIFI_IP_MODE_TAG, tf_config->nrc_wifi_config.ip_mode);
        nvs_set_str(nvs_handle, TD_WIFI_STATIC_IP_TAG, (char*) tf_config->nrc_wifi_config.static_ip);
        nvs_set_str(nvs_handle, TD_WIFI_STATIC_IP_TAG, (char*) tf_config->nrc_wifi_config.ap_ip);
        nvs_set_str(nvs_handle, TD_WIFI_STATIC_IPV6_TAG, (char*) tf_config->ipv6_addr);
        nvs_set_str(nvs_handle, TD_WIFI_REMOTE_IP_TAG, (char*) tf_config->nrc_wifi_config.remote_addr);
        nvs_set_u16 (nvs_handle, TD_WIFI_REMOTE_PORT_TAG, tf_config->nrc_wifi_config.remote_port);
        nvs_set_u8 (nvs_handle, TD_WIFI_DHCP_SERVER_TAG, tf_config->nrc_wifi_config.dhcp_server);
        nvs_set_u8 (nvs_handle, TD_WIFI_COUNT_TAG, tf_config->nrc_wifi_config.count);
        nvs_signed_int = (int32_t)tf_config->nrc_wifi_config.interval;
        nvs_set_i32 (nvs_handle, TD_WIFI_INTERVAL_TAG, nvs_signed_int);
        nvs_signed_int = (int32_t)tf_config->nrc_wifi_config.short_bcn_interval;
        nvs_set_i32 (nvs_handle, TD_WIFI_SHORT_BCN_INTERVAL_TAG, nvs_signed_int);
        nvs_signed_int = (int32_t)tf_config->nrc_wifi_config.duration;
        nvs_set_i32 (nvs_handle, TD_WIFI_DURATION_TAG, nvs_signed_int);
        nvs_signed_int = (int32_t)tf_config->nrc_wifi_config.tx_power;
        nvs_set_i32 (nvs_handle, TD_WIFI_TXPOWER_TAG, nvs_signed_int);
        nvs_set_u8 (nvs_handle, TD_PPP_ENABLE_TAG, tf_config->ppp_enable);

       err =  nvs_commit(nvs_handle);

       if(NVS_OK != err)
               goto failed;
        
        if(nvs_handle)
                nvs_close(nvs_handle);

        return NRC_SUCCESS;

failed:
        nrc_usr_print("[%s] Failed to save wifi settings\n", __func__);

        if(nvs_handle)
                nvs_close(nvs_handle);

        return NRC_FAIL;
}

/**
 * @brief set default configuration
 * 
 * Set default wifi configuration in memory
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_set_wifi_defaults(td_wifi_config_t *tf_config)
{
        if(!tf_config)
                return NRC_FAIL;

#if NRC_WIFI_SCAN_LIST
	tf_config->nrc_wifi_config.scan_freq_num = NRC_WIFI_SCAN_FREQ_NUM;
	for (int i=0; i < tf_config->nrc_wifi_config.scan_freq_num; i++)
		tf_config->nrc_wifi_config.scan_freq_list[i] = nrc_scan_freq_list[i];
#endif

        tf_config->wifi_mode = TD_WIFI_MODE_DEFAULT;
        memcpy(tf_config->nrc_wifi_config.ssid, TD_WIFI_SSID_DEFAULT, sizeof(TD_WIFI_SSID_DEFAULT));
        memcpy(tf_config->nrc_wifi_config.password, TD_WIFI_PASSWORD_DEFAULT, sizeof(TD_WIFI_PASSWORD_DEFAULT));
        tf_config->nrc_wifi_config.security_mode = TD_WIFI_SECURITY_DEFAULT;
        memcpy(tf_config->nrc_wifi_config.country, TD_WIFI_COUNTRY_DEFAULT, sizeof(TD_WIFI_COUNTRY_DEFAULT));
        tf_config->nrc_wifi_config.channel = TD_WIFI_CHANNEL_DEFAULT;
        tf_config->nrc_wifi_config.ip_mode = TD_WIFI_IP_MODE_DEFAULT;
        memcpy(tf_config->nrc_wifi_config.static_ip, TD_WIFI_STATIC_IP_DEFAULT, sizeof(TD_WIFI_STATIC_IP_DEFAULT));
        memcpy(tf_config->nrc_wifi_config.ap_ip, TD_WIFI_STATIC_IP_DEFAULT, sizeof(TD_WIFI_STATIC_IP_DEFAULT));
        memcpy( tf_config->ipv6_addr, TD_WIFI_STATIC_IPV6_DEFAULT, sizeof(TD_WIFI_STATIC_IPV6_DEFAULT));
        tf_config->nrc_wifi_config.dhcp_server = TD_WIFI_DHCP_SERVER_DEFAULT;
        memcpy(tf_config->nrc_wifi_config.remote_addr, TD_WIFI_REMOTE_IP_DEFAULT, sizeof(TD_WIFI_REMOTE_IP_DEFAULT));
        tf_config->nrc_wifi_config.remote_port = TD_WIFI_REMOTE_PORT_DEFAULT;
        tf_config->nrc_wifi_config.count = TD_WIFI_COUNT_DEFAULT;
        tf_config->nrc_wifi_config.interval = TD_WIFI_INTERVAL_DEFAULT;
        tf_config->nrc_wifi_config.short_bcn_interval = TD_WIFI_SHORT_BCN_INTERVAL_DEFAULT;
        tf_config->nrc_wifi_config.duration = TD_WIFI_DURATION_DEFAULT;
        tf_config->nrc_wifi_config.tx_power = TD_WIFI_TXPOWER_DEFAULT;
        tf_config->ppp_enable = TD_PPP_ENABLE_DEFAULT;
        
        td_print_settings(tf_config);
        
        return NRC_SUCCESS;
}

/**
 * @brief save default configuration
 * 
 * Save default configuration to key+value storage
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_save_wifi_defaults(td_wifi_config_t *tf_config)
{
        if(!tf_config)
                return NRC_FAIL;
        
        td_set_wifi_defaults(tf_config);

        td_print_settings(tf_config);
        
        return td_save_wifi_config(tf_config);
}


/**
 * @brief get wifi configration data
 * 
 * Retrieve wifi configration from key+value storage
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_get_wifi_config(td_wifi_config_t *tf_config)
{
        nvs_err_t err = NVS_OK;
        nvs_handle_t nvs_handle = 0;
        size_t length = 0;
        int32_t nvs_signed_int = 0;

        if(!tf_config)
                goto failed;
        
        memset(tf_config, 0x0, sizeof(td_wifi_config_t));

        td_set_wifi_defaults(tf_config);
        
        err = nvs_open(NVS_DEFAULT_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(NVS_OK != err)
                goto failed;

        err = nvs_get_u8(nvs_handle, TD_WIFI_MODE_TAG, &tf_config->wifi_mode);
        
        nrc_usr_print("[%s] wifi_mode is %d\n", __func__, tf_config->wifi_mode);

        if(NVS_ERR_NVS_NOT_FOUND == err) { /* no configuration set */
                nvs_close(nvs_handle);

                nrc_usr_print("[%s] configuration empty, saving default\n", __func__);
                
                td_save_wifi_defaults(tf_config);

                err = nvs_open(NVS_DEFAULT_NAMESPACE, NVS_READWRITE, &nvs_handle);
                if(NVS_OK != err)
                        goto failed;
        }

        nvs_get_u8 (nvs_handle, TD_WIFI_MODE_TAG, (uint8_t*) &tf_config->wifi_mode);
        nvs_get_str(nvs_handle, TD_WIFI_SSID_TAG, (char*) tf_config->nrc_wifi_config.ssid, &length);
        nvs_get_str(nvs_handle, TD_WIFI_PASSWORD_TAG, (char*) tf_config->nrc_wifi_config.password, &length);
        nvs_get_u8 (nvs_handle, TD_WIFI_SECURITY_TAG, (uint8_t*) &tf_config->nrc_wifi_config.security_mode);
        nvs_get_str(nvs_handle, TD_WIFI_COUNTRY_TAG, (char*) tf_config->nrc_wifi_config.country, &length);
        nvs_get_u32 (nvs_handle, TD_WIFI_CHANNEL_TAG, (uint32_t*) &tf_config->nrc_wifi_config.channel);
        nvs_get_u8 (nvs_handle, TD_WIFI_IP_MODE_TAG, (uint8_t*) &tf_config->nrc_wifi_config.ip_mode);
        nvs_get_str(nvs_handle, TD_WIFI_STATIC_IP_TAG, (char*) tf_config->nrc_wifi_config.static_ip, &length);
        nvs_get_str(nvs_handle, TD_WIFI_STATIC_IP_TAG, (char*) tf_config->nrc_wifi_config.ap_ip, &length);
        nvs_get_str(nvs_handle, TD_WIFI_STATIC_IPV6_TAG, (char*) tf_config->ipv6_addr, &length);
        nvs_get_str(nvs_handle, TD_WIFI_REMOTE_IP_TAG, (char*) tf_config->nrc_wifi_config.remote_addr, &length);
        nvs_get_u16 (nvs_handle, TD_WIFI_REMOTE_PORT_TAG, (uint16_t*) &tf_config->nrc_wifi_config.remote_port);
        nvs_get_u8 (nvs_handle, TD_WIFI_DHCP_SERVER_TAG, (uint8_t*) &tf_config->nrc_wifi_config.dhcp_server);
        nvs_get_u8 (nvs_handle, TD_WIFI_COUNT_TAG, (uint8_t*) &tf_config->nrc_wifi_config.count);
        nvs_get_i32 (nvs_handle, TD_WIFI_INTERVAL_TAG, &nvs_signed_int);
        tf_config->nrc_wifi_config.interval = nvs_signed_int;
        nvs_get_i32 (nvs_handle, TD_WIFI_SHORT_BCN_INTERVAL_TAG, &nvs_signed_int);
        tf_config->nrc_wifi_config.short_bcn_interval = nvs_signed_int;
        nvs_get_i32 (nvs_handle, TD_WIFI_TXPOWER_TAG, &nvs_signed_int);
        tf_config->nrc_wifi_config.tx_power = nvs_signed_int;
        nvs_get_i32 (nvs_handle, TD_WIFI_DURATION_TAG, &nvs_signed_int);
        tf_config->nrc_wifi_config.duration = nvs_signed_int;
        nvs_get_u8 (nvs_handle, TD_PPP_ENABLE_TAG, (uint8_t*) &tf_config->ppp_enable);
        
        td_validate_params(tf_config);

        if(nvs_handle)
                nvs_close(nvs_handle);
        
        td_print_settings(tf_config);

        return NRC_SUCCESS;

failed:
        nrc_usr_print("[%s] Failed to retrieve wifi settings\n", __func__);

        if(nvs_handle)
                nvs_close(nvs_handle);

        return NRC_FAIL;
}

/**
 * @brief initialization function
 * 
 * Initialization function; set defaults, get to stable state
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_init(td_wifi_config_t *tf_config)
{
        nvs_err_t err = NVS_OK;

        nrc_uart_console_enable(true);

        if(td_get_wifi_config(tf_config) != NRC_SUCCESS)
                goto failed;

        return NRC_SUCCESS;

failed:

        nrc_usr_print ("[%s] Failed to initialize!\n", __func__);

        nrc_uart_console_enable(false);

        return NRC_FAIL;
}

/**
 * @brief run wifi state machine
 * 
 * Run Wi-Fi AP or client depending on wifi_mode
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_run_wifi(td_wifi_config_t* tf_config)
{
        if(WIFI_MODE_AP == tf_config->wifi_mode) {
                td_run_wifi_ap(tf_config);
        }
        else if(WIFI_MODE_STATION == tf_config->wifi_mode){
                td_run_wifi_client(tf_config);
        }
        else if(WIFI_MODE_MESH_POINT == tf_config->wifi_mode){
                td_run_wifi_mesh(tf_config);
        }
        else
                goto failed;

        return NRC_SUCCESS;
failed:

        nrc_usr_print ("[%s] Failed to run Wi-Fi mode %d!\n", __func__, (tf_config) ? tf_config->wifi_mode : -1);

        return NRC_FAIL;
}

/**
 * @brief Newracom nrc7292 FreeRTOS SDK main function
 * 
 * Start Code for User Application, Initialize User function
 * 
 * @param none
 * @returns none
 */
int should_quit = 0;
void user_init(void)
{
        td_wifi_config_t tf_config; 

        nrc_usr_print ("[%s] \n", __func__);

        if (td_init(&tf_config) != NRC_SUCCESS) {
                return;
        }

        if(tf_config.ppp_enable) {
                td_start_ppp();
        }

	while(should_quit == 0) {
                td_run_wifi(&tf_config);
                _delay_ms(1);
	}

}
