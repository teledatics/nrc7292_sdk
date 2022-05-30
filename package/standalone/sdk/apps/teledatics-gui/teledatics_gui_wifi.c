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
 * @file teledatics_gui_wifi.c
 * @author James Ewing
 * @date 24 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */

#include "teledatics_gui.h"

#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#ifdef SUPPORT_ETHERNET_ACCESSPOINT
#include "nrc_eth_if.h"
#endif

extern struct netif* nrc_netif[];
extern struct netif br_netif;

tWIFI_STATUS
nrc_wifi_dhcp_start(uint32_t timeout);
static void
td_wifi_event_handler(tWIFI_EVENT_ID event, int data_len, char* data);

tWIFI_STATUS
td_wifi_softap_set_ip(void)
{
  tWIFI_STATUS status;
  ip4_addr_t ipaddr;
  ip4_addr_t netmask;
  ip4_addr_t gw;
  struct netif* n_if = nrc_netif[0];

  td_wifi_config_t* tf_config = td_get_global_config();

  nrc_usr_print("[%s]\n", __func__);

  if (!tf_config) {
    return WIFI_FAIL;
  }

#ifdef SUPPORT_ETHERNET_ACCESSPOINT
  if (get_network_mode() == NRC_NETWORK_MODE_BRIDGE) {
    n_if = &br_netif;
  }
#endif

  ip4addr_aton((const char*)tf_config->nrc_wifi_config.ap_ip, &ipaddr);
  IP4_ADDR(&gw,
           ip4_addr1(&ipaddr),
           ip4_addr2(&ipaddr),
           ip4_addr3(&ipaddr),
           ip4_addr4(&ipaddr));
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  netif_set_addr(n_if, &ipaddr, &netmask, &gw);
  nrc_wifi_set_state(WIFI_STATE_SOFTAP_START);
  td_wifi_event_handler(WIFI_EVT_SET_SOFT_AP_IP, 0, NULL);
  status = WIFI_SUCCESS;
  return status;
}

tWIFI_STATUS
td_wifi_set_ip_address(void)
{
  tWIFI_STATUS status;
  ip_addr_t dnsserver;
  ip4_addr_t ipaddr;
  ip4_addr_t netmask;
  ip4_addr_t gw;
  struct netif* n_if = nrc_netif[0];
  tWIFI_STATE_ID state;
  td_wifi_config_t* tf_config = td_get_global_config();
  tWIFI_IP_MODE ip_mode;

  nrc_usr_print("[%s]\n", __func__);

  if (!tf_config) {
    return WIFI_FAIL;
  }

  nrc_wifi_get_ip_mode(&ip_mode);

  if (ip_mode == WIFI_STATIC_IP) {

    nrc_wifi_get_state(&state);

    if (state == WIFI_STATE_CONNECTED) {
      nrc_wifi_set_state(WIFI_STATE_TRY_GET_IP);
      ip4addr_aton((const char*)tf_config->nrc_wifi_config.static_ip, &ipaddr);
      IP4_ADDR(
        &gw, ip4_addr1(&ipaddr), ip4_addr2(&ipaddr), ip4_addr4(&ipaddr), 1);
      IP4_ADDR(&netmask, 255, 255, 255, 0);
#ifdef SUPPORT_ETHERNET_ACCESSPOINT
      if (get_network_mode() == NRC_NETWORK_MODE_BRIDGE) {
        n_if = &br_netif;
      }
#endif
      netif_set_addr(n_if, &ipaddr, &netmask, &gw);
      lwip_inet_pton(2, "8.8.8.8", &dnsserver);
      dns_setserver(0, &dnsserver);
      lwip_inet_pton(2, "8.8.4.4", &dnsserver);
      dns_setserver(1, &dnsserver);
      td_wifi_event_handler(WIFI_EVT_GET_IP, 0, NULL);
      status = WIFI_SUCCESS;
    } else {
      status = WIFI_FAIL;
    }
  } else {
    status = nrc_wifi_dhcp_start(10000);
  }

  return status;
}

static void
td_wifi_event_handler(tWIFI_EVENT_ID event, int data_len, char* data)
{
  char* ip_addr = NULL;
  tWIFI_STATUS ret = WIFI_FAIL;
  int cnt = 0;

  switch (event) {
    case WIFI_EVT_CONNECT_SUCCESS:
      nrc_usr_print("[%s] Receive Connection Success Event\n", __func__);
      if (!netif_is_link_up(nrc_netif[0])) {
        netif_set_link_up(nrc_netif[0]);
      }
      // #ifdef SUPPORT_ETHERNET_ACCESSPOINT
      // 			if (get_network_mode() == NRC_NETWORK_MODE_BRIDGE)
      // { 				break;
      // 			}
      // #endif
      do {
        ret = td_wifi_set_ip_address();
        if (ret != WIFI_SUCCESS) {
          cnt++;
          nrc_usr_print("[%s] Fail to set IP addr(cnt %d)\n", __func__, cnt);
        }
        if (cnt == MAX_WIFI_CONNECT_TRIES) {
          nrc_wifi_set_state(WIFI_STATE_TRY_DISASSOC);
          break;
        }
      } while (ret != WIFI_SUCCESS);

      break;
    case WIFI_EVT_CONNECT_FAIL:
      nrc_usr_print("[%s] Receive Connection Fail Event\n", __func__);
      break;
    case WIFI_EVT_GET_IP:
      nrc_usr_print("[%s] Receive IP_GET Success Event\n", __func__);
      if (nrc_wifi_get_ip_address(&ip_addr) == WIFI_SUCCESS) {
        nrc_usr_print(
          "[%s] IP Address : %s\n", __func__, ip_addr ? ip_addr : "null");
      }
      break;
    case WIFI_EVT_GET_IP_FAIL:
      nrc_usr_print("[%s] Receive IP_GET Fail Event\n", __func__);
      break;
    case WIFI_EVT_DISCONNECT:
      nrc_usr_print("[%s] Receive Disconnection Event\n", __func__);
      /* send netif down indication to LWIP */
      if (netif_is_link_up(nrc_netif[0])) {
        netif_set_link_down(nrc_netif[0]);
      }
      break;
    case WIFI_EVT_SCAN_DONE:
      nrc_usr_print("[%s] Receive Scan Done Event\n", __func__);
      break;
    case WIFI_EVT_START_SOFT_AP:
      nrc_usr_print("[%s] Receive Start Soft AP Event\n", __func__);
      break;
    case WIFI_EVT_SET_SOFT_AP_IP:
      nrc_usr_print("[%s] Receive SET IP Event\n", __func__);
      if (nrc_wifi_get_ip_address(&ip_addr) == WIFI_SUCCESS) {
        nrc_usr_print("[%s] IP Address : %s\n", __func__, ip_addr);
      }
      break;
    case WIFI_EVT_START_DHCP_SERVER:
      nrc_usr_print("[%s] Receive Start DHCPS Event\n", __func__);
      break;
    default:
      nrc_usr_print("[%s] Receive Unknown Event %d\n", __func__, event);
      break;
  }
}

int
td_wifi_init(WIFI_CONFIG* param)
{
  int txpower;

  /* Register Wi-Fi Event Handler */
  nrc_wifi_register_event_handler(td_wifi_event_handler);

  /* Set IP mode config (Dynamic IP(DHCP client) or STATIC IP) */
  if (nrc_wifi_set_ip_mode((bool)param->ip_mode, (char*)param->static_ip) < 0) {
    nrc_usr_print("[%s] Fail to set static IP \n", __func__);
    return WIFI_SET_IP_FAIL;
  }

  /* Set TX Power */
  txpower = param->tx_power;

  if (nrc_wifi_set_tx_power(txpower) != WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail set TX Power\n", __func__);
    return WIFI_FAIL;
  }
  txpower = 0;
  nrc_wifi_get_tx_power(&txpower);
  nrc_usr_print("[%s] TX Power (%d dBm)\n", __func__, txpower);

  /* Set Country Code */
  if (nrc_wifi_set_country(
        nrc_wifi_country_from_string((char*)param->country)) != WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail to set Country\n", __func__);
    return WIFI_FAIL;
  }

  return WIFI_SUCCESS;
}

int
td_wifi_connect(td_wifi_config_t* tf_config)
{
  int index = -1;

  /* Try to connect with ssid and security */
  nrc_usr_print("[%s] Trying to Wi-Fi Connection...\n", __func__);

  if (nrc_wifi_add_network(&index) < 0) {
    nrc_usr_print("[%s] Fail to init \n", __func__);
    return WIFI_INIT_FAIL;
  }

  if (nrc_wifi_set_ssid(index, (char*)tf_config->nrc_wifi_config.ssid) !=
      WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail to set SSID\n", __func__);
    return WIFI_FAIL;
  }

  if (strlen((char*)tf_config->nrc_wifi_config.bssid) != 0) {
    if (nrc_wifi_set_bssid(index, (char*)tf_config->nrc_wifi_config.bssid) !=
        WIFI_SUCCESS) {
      nrc_usr_print("[%s] Fail to set BSSID\n", __func__);
      return WIFI_FAIL;
    }
  }

  /* Set Non-S1G channel */
  if (tf_config->nrc_wifi_config.s1g_channel != 0) {
    if (nrc_wifi_set_s1g_config(tf_config->nrc_wifi_config.s1g_channel) !=
        WIFI_SUCCESS) {
      nrc_usr_print("[%s] Fail to set S1G channel\n", __func__);
      return WIFI_FAIL;
    }
  }

  if (nrc_wifi_set_security(index,
                            (int)tf_config->nrc_wifi_config.security_mode,
                            (char*)tf_config->nrc_wifi_config.password) !=
      WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail to set Security\n", __func__);
    return WIFI_FAIL;
  }

  if (nrc_wifi_set_scan_freq(index,
                             tf_config->nrc_wifi_config.scan_freq_list,
                             tf_config->nrc_wifi_config.scan_freq_num) !=
      WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail to set Scan Freq\n", __func__);
    return WIFI_FAIL;
  }

  if (nrc_wifi_connect(index) != WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail to Connect\n", __func__);
    return WIFI_CONNECTION_FAIL;
  }

  // static IP, run server now
  if (tf_config->nrc_wifi_config.ip_mode == WIFI_STATIC_IP) {
    run_http_server(tf_config);
  }

  if (td_wifi_set_ip_address() != WIFI_SUCCESS) {
    // #ifdef SUPPORT_ETHERNET_ACCESSPOINT
    // 	/* Do not set ip on bridge interface */
    // 	/* TODO: incomplete. Should be fixed if bridge needs IP */
    // 	if (get_network_mode() == NRC_NETWORK_MODE_BRIDGE) {
    // 		return WIFI_SUCCESS;
    // 	}
    // #endif
    nrc_usr_print("[%s] Fail to set IP Address\n", __func__);
    return WIFI_SET_IP_FAIL;
  }

  return WIFI_SUCCESS;
}

/**
 * @brief shutdown access point wifi
 *
 * Shutdown AP Wi-Fi and reset to initial state
 *
 * @param none
 * @returns none
 */
void
td_shutdown_ap(void)
{
  int network_index = -1;

  nrc_usr_print("[%s]\n", __func__);

  nrc_wifi_get_network_index(&network_index);
  if (network_index >= 0) {
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
void
td_shutdown_client(void)
{
  int network_index = -1;

  nrc_usr_print("[%s]\n", __func__);

  nrc_wifi_get_network_index(&network_index);
  if (network_index >= 0) {
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
void
td_shutdown_mesh(void)
{

  nrc_usr_print("[%s]\n", __func__);
}

/**
 * @brief reset wifi
 *
 * Reset Wi-Fi to initial state
 *
 * @param none
 * @returns none
 */
void
td_wifi_reset(void)
{
  tWIFI_DEVICE_MODE mode;

  nrc_wifi_get_device_mode(&mode);

  switch (mode) {
    case WIFI_MODE_AP:
      nrc_usr_print("[%s] WIFI_MODE_AP\n", __func__);
      td_shutdown_ap();
      break;
    case WIFI_MODE_STATION:
      nrc_usr_print("[%s] WIFI_MODE_STATION\n", __func__);
      td_shutdown_client();
      break;
    case WIFI_MODE_MESH_POINT:
      nrc_usr_print("[%s] WIFI_MODE_MESH_POINT\n", __func__);
      td_shutdown_mesh();
      break;
    default:
      nrc_usr_print("Error! Unknown Wi-Fi mode\n");
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
nrc_err_t
td_run_wifi_ap(td_wifi_config_t* tf_config)
{
  int max_tries, i;
  tWIFI_STATE_ID wifi_state = WIFI_STATE_INIT;

  if (!tf_config)
    goto failed;

  //         td_wifi_reset();

  /* set initial wifi configuration */
  for (max_tries = 0; max_tries < MAX_WIFI_INIT_TRIES; max_tries++) {
    if (wifi_init(&tf_config->nrc_wifi_config) == WIFI_SUCCESS) {
      nrc_usr_print("[%s] wifi_init Success !!\n", __func__);
      break;
    } else {
      nrc_usr_print("[%s] wifi_init Failed !!\n", __func__);
      _delay_ms(1000);
    }
  }

  if (MAX_WIFI_INIT_TRIES <= max_tries)
    goto failed;

  nrc_usr_print("[%s] calling wifi_start_softap\n", __func__);

  if (wifi_start_softap(&tf_config->nrc_wifi_config) != WIFI_SUCCESS) {
    nrc_usr_print("[%s] ASSERT! Fail to start softap\n", __func__);
    goto failed;
  }

  nrc_usr_print("[%s] calling nrc_wifi_softap_set_ip\n", __func__);

  //         if (nrc_wifi_softap_set_ip((char
  //         *)&tf_config->nrc_wifi_config.ap_ip) != WIFI_SUCCESS) {
  if (td_wifi_softap_set_ip() != WIFI_SUCCESS) {
    nrc_usr_print("[%s] Fail set AP's IP\n", __func__);
    goto failed;
  }

  nrc_usr_print("[%s] dhcp_server start? %d\n",
                __func__,
                tf_config->nrc_wifi_config.dhcp_server);

  if (1 == tf_config->nrc_wifi_config.dhcp_server) {
    i = 0;
    do {
      nrc_wifi_get_state(&wifi_state);
      if (wifi_state == WIFI_STATE_SOFTAP_START) {
        nrc_usr_print("[%s] WIFI_STATE_SOFTAP_START!\n", __func__);
        break;
      }
      _delay_ms(1000);
      i++;
    } while (i < 10);

    nrc_wifi_get_state(&wifi_state);
    if (wifi_state != WIFI_STATE_SOFTAP_START) {
      nrc_usr_print("[%s] Failed to find WIFI_STATE_SOFTAP_START\n", __func__);
      goto failed;
    }

    nrc_usr_print("[%s] Trying to start DHCP Server\n", __func__);
    if (nrc_wifi_softap_start_dhcp_server() != WIFI_SUCCESS) {
      nrc_usr_print("[%s] Fail to start dhcp server\n", __func__);
      goto failed;
    }
  }

  run_http_server(tf_config);

  nrc_wifi_get_state(&wifi_state);
  nrc_usr_print("[%s] Success state %d\n", __func__, wifi_state);

  return NRC_SUCCESS;
failed:
  nrc_usr_print("[%s] failed\n", __func__);

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
nrc_err_t
td_run_wifi_client(td_wifi_config_t* tf_config)
{
  int i = 0;
  int count = 0;
  int network_index = 0;
  int dhcp_server = 0;
  tWIFI_STATE_ID wifi_state = WIFI_STATE_INIT;

  if (!tf_config)
    goto failed;

  // 	td_wifi_reset();

  count = tf_config->nrc_wifi_config.count;
  dhcp_server = tf_config->nrc_wifi_config.dhcp_server;

  if (wifi_init(&tf_config->nrc_wifi_config) != WIFI_SUCCESS) {
    nrc_usr_print("[%s] ASSERT! Fail for init\n", __func__);
    goto failed;
  }

  nrc_usr_print("[%s] Trying to connect to AP - %s...\n",
                __func__,
                tf_config->nrc_wifi_config.ssid);
  do {
    if (td_wifi_connect(tf_config) == WIFI_SUCCESS) {
      nrc_usr_print("[%s] Wi-Fi connection successful...\n", __func__);
      break;
    }

    nrc_usr_print("[%s] Connect to AP(%s) timed out, trying again...\n",
                  __func__,
                  tf_config->nrc_wifi_config.ssid);
    i++;
  } while (i < 10);

  if (i >= 10) {
    nrc_usr_print("[%s] Wi-Fi connection failed. Check AP availability and "
                  "SSID, and try again...\n",
                  __func__);
    goto failed;
  }

  nrc_wifi_get_network_index(&network_index);
  nrc_usr_print("[%s] network_index = %d\n", __func__, network_index);

  i = 0;
  do {
    nrc_wifi_get_state(&wifi_state);
    if (wifi_state == WIFI_STATE_GET_IP) {
      nrc_usr_print("[%s] IP received!\n", __func__);
      break;
    }

    _delay_ms(1000);
    i++;
  } while (i < 10);

  if (wifi_state != WIFI_STATE_GET_IP) {
    nrc_usr_print("[%s] Fail to connect or get IP !\n", __func__);
    goto failed;
  }

  if (tf_config->nrc_wifi_config.ip_mode != WIFI_STATIC_IP) {
    run_http_server(tf_config);
  }

  nrc_usr_print("[%s] Device is online connected to %s\n",
                __func__,
                tf_config->nrc_wifi_config.ssid);

  return NRC_SUCCESS;

failed:
  nrc_usr_print("[%s] failed\n", __func__);

  return NRC_FAIL;
}

// forward declaration - in libModem.a
void
set_device_mode(uint8_t vif_id, uint8_t v);

/**
 * @brief run wifi mesh
 *
 * Run Wi-Fi mesh node
 *
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t
td_run_wifi_mesh(td_wifi_config_t* tf_config)
{
  int i = 0;
  int count = 0;
  int network_index = 0;
  int dhcp_server = 0;
  tWIFI_STATE_ID wifi_state = WIFI_STATE_INIT;

  if (!tf_config)
    goto failed;

  // NOTE: probably have to initialize nic[0] as AP and nic[1] as mesh node

  set_device_mode(0, WIFI_MODE_MESH_POINT);

failed:
  return NRC_FAIL;
}
