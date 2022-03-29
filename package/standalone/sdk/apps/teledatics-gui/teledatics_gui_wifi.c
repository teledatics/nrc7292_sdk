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
 * @file teledatics_gui_wifi.c
 * @author James Ewing
 * @date 24 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */
 
 #include "teledatics_gui.h"
 
/**
 * @brief shutdown access point wifi
 * 
 * Shutdown AP Wi-Fi and reset to initial state
 * 
 * @param none
 * @returns none
 */
void td_shutdown_ap(void)
{
	int network_index = -1;

	nrc_usr_print ("[%s]\n", __func__);
	
	nrc_wifi_get_network_index(&network_index);
	if(network_index >= 0) {
		reset_ip_address(network_index);
		nrc_wifi_remove_network(network_index);
		dhcps_stop();
	}
}

/**
 * @brief shutdown wifi client
 * 
 * Shutdown Wi-Fi STA and reset to initial state
 * 
 * @param none
 * @returns none
 */
void td_shutdown_client(void)
{
        int network_index = -1;

	nrc_usr_print ("[%s]\n", __func__);

	nrc_wifi_get_network_index(&network_index);
	if(network_index >= 0) {
		wifi_station_dhcpc_stop(0);
		nrc_wifi_disconnect(network_index);
		reset_ip_address(network_index);
		nrc_wifi_remove_network(network_index);
	}
}

/**
 * @brief shutdown wifi mesh
 * 
 * Shutdown Wi-Fi mesh and reset to initial state
 * 
 * @param none
 * @returns none
 */
void td_shutdown_mesh(void)
{

	nrc_usr_print ("[%s]\n", __func__);
}

/**
 * @brief reset wifi
 * 
 * Reset Wi-Fi to initial state
 * 
 * @param none
 * @returns none
 */
void td_wifi_reset(void)
{
	tWIFI_DEVICE_MODE mode;
	
	nrc_wifi_get_device_mode(&mode);
	
	switch(mode)
	{
		case WIFI_MODE_AP:
			nrc_usr_print ("[%s] WIFI_MODE_AP\n", __func__);
			td_shutdown_ap();
			break;
		case WIFI_MODE_STATION:
			nrc_usr_print ("[%s] WIFI_MODE_STATION\n", __func__);
			td_shutdown_client();
			break;
		case WIFI_MODE_MESH_POINT:
			nrc_usr_print ("[%s] WIFI_MODE_MESH_POINT\n", __func__);
			td_shutdown_mesh();
			break;
		default:
			nrc_usr_print ("Error! Unknown Wi-Fi mode\n");
			break;
	}
	
	_delay_ms(1000);
}

 /**
 * @brief run wifi access point
 * 
 * Run Wi-Fi AP
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_run_wifi_ap(td_wifi_config_t* tf_config)
{
	int max_tries, i;
	tWIFI_STATE_ID wifi_state = WIFI_STATE_INIT;

        if(!tf_config)
                goto failed;

        td_wifi_reset();
	
	/* set initial wifi configuration */
	for(max_tries=0; max_tries < MAX_WIFI_INIT_TRIES; max_tries++) {
		if (wifi_init(&tf_config->nrc_wifi_config) == WIFI_SUCCESS) {
			nrc_usr_print ("[%s] wifi_init Success !!\n", __func__);
			break;
		} else {
			nrc_usr_print ("[%s] wifi_init Failed !!\n", __func__);
			_delay_ms(1000);
		}
	}
	
	if (MAX_WIFI_INIT_TRIES <= max_tries)
                goto failed;

	nrc_usr_print ("[%s] calling wifi_start_softap\n", __func__);

	if (wifi_start_softap(&tf_config->nrc_wifi_config) != WIFI_SUCCESS) {
		nrc_usr_print ("[%s] ASSERT! Fail to start softap\n", __func__);
		goto failed;
	}

	nrc_usr_print ("[%s] calling nrc_wifi_softap_set_ip\n", __func__);

        if (nrc_wifi_softap_set_ip((char *)&tf_config->nrc_wifi_config.ap_ip) != WIFI_SUCCESS) {
		nrc_usr_print("[%s] Fail set AP's IP\n", __func__);
		goto failed;
	}

	nrc_usr_print ("[%s] dhcp_server start? %d\n", __func__, tf_config->nrc_wifi_config.dhcp_server);
	
        if (1 == tf_config->nrc_wifi_config.dhcp_server) {
		i = 0;
		do {
			nrc_wifi_get_state(&wifi_state);
			if (wifi_state == WIFI_STATE_SOFTAP_START) {
				nrc_usr_print("[%s] WIFI_STATE_SOFTAP_START!\n",__func__);
				break;
			}
			_delay_ms(1000);
			i++;
		} while (i < 10);

		nrc_wifi_get_state(&wifi_state);
		if(wifi_state != WIFI_STATE_SOFTAP_START) {
			nrc_usr_print("[%s] Failed to find WIFI_STATE_SOFTAP_START\n", __func__);
			goto failed;
		}
		
		nrc_usr_print("[%s] Trying to start DHCP Server\n",	__func__);
		if(nrc_wifi_softap_start_dhcp_server() != WIFI_SUCCESS) {
			nrc_usr_print("[%s] Fail to start dhcp server\n", __func__);
			goto failed;
		}
	}
	
	run_http_server(tf_config);
	
	nrc_wifi_get_state(&wifi_state);
	nrc_usr_print("[%s] Success state %d\n", __func__, wifi_state);

        return NRC_SUCCESS;
failed:
        nrc_usr_print ("[%s] failed\n", __func__);
        
        return NRC_FAIL;
}

 /**
 * @brief run wifi client
 * 
 * Run Wi-Fi client
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_run_wifi_client(td_wifi_config_t* tf_config)
{
	int i = 0;
	int count =0;
	int network_index = 0;
	int dhcp_server = 0;
	tWIFI_STATE_ID wifi_state = WIFI_STATE_INIT;
	
        if(!tf_config)
                goto failed;

	td_wifi_reset();
		
	count = tf_config->nrc_wifi_config.count;
	dhcp_server = tf_config->nrc_wifi_config.dhcp_server;

	if (wifi_init(&tf_config->nrc_wifi_config)!= WIFI_SUCCESS) {
		nrc_usr_print ("[%s] ASSERT! Fail for init\n", __func__);
		return NRC_FAIL;
	}

	nrc_usr_print ("[%s] Trying to connect to AP - %s...\n", __func__, tf_config->nrc_wifi_config.ssid);
	do {
		if (wifi_connect(&tf_config->nrc_wifi_config) == WIFI_SUCCESS) {
			nrc_usr_print ("[%s] Wi-Fi connection successful...\n", __func__);
			break;
		}
		nrc_usr_print ("[%s] Connect to AP(%s) timed out, trying again...\n", __func__, tf_config->nrc_wifi_config.ssid);
		i++;
	} while (i < 10);

	if (i >= 10) {
		nrc_usr_print ("[%s] Wi-Fi connection failed. Check AP availability and SSID, and try again...\n", __func__);
		return NRC_FAIL;
	}

	nrc_wifi_get_network_index(&network_index );
	nrc_usr_print ("[%s] network_index = %d\n", __func__, network_index);

	i = 0;
	do {
		nrc_wifi_get_state(&wifi_state);
		if (wifi_state == WIFI_STATE_GET_IP) {
			nrc_usr_print("[%s] IP received!\n",__func__);
			break;
		}
		_delay_ms(1000);
		i++;
	} while (i < 10);

	if (wifi_state != WIFI_STATE_GET_IP) {
		nrc_usr_print("[%s] Fail to connect or get IP !\n",__func__);
		return NRC_FAIL;
	}

	nrc_usr_print("[%s] Device is online connected to %s\n",__func__, tf_config->nrc_wifi_config.ssid);
	
	return NRC_SUCCESS;

failed:
        nrc_usr_print ("[%s] failed\n", __func__);
        
        return NRC_FAIL;
}

 /**
 * @brief run wifi client
 * 
 * Run Wi-Fi client
 * 
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t td_run_wifi_mesh(td_wifi_config_t* tf_config)
{
        td_wifi_reset();

        return NRC_SUCCESS;
}
