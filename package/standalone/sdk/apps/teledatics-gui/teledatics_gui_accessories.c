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
 * @file teledatics_gui_accessories.c
 * @author James Ewing
 * @date 8 Apr 2022
 * @brief Teledatics Accessory Routines
 */

#include "nrc_eth_if.h"
#include "nrc_esp32_if.h"
#include "teledatics_gui.h"
#include "teledatics_gui_air_quality.h"
#include "teledatics_gui_ethernet.h"
#include "teledatics_gui_wifi_gw.h"

/**
 * @brief Identify Arducam SPI camera
 *
 * Identify if Arudcam SPI camera is attached
 *
 * @param none
 * @returns nrc_err_t
 */
nrc_err_t
td_check_arducam_camera_hat(void)
{

  return NRC_FAIL;
}

/**
 * @brief Identify attached accessories
 *
 * Identify and populate accessories config indicator
 *
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t
td_init_accessories(td_wifi_config_t* tf_config)
{
  nrc_usr_print("[%s] start\n", __func__);

  if (!tf_config)
    return NRC_FAIL;

  tf_config->accessories = 0;
  
  if (td_check_arducam_camera_hat() == NRC_SUCCESS) {
    nrc_usr_print("[%s] found SPI camera hAT\n", __func__);
    tf_config->accessories |= HAVE_ARDUCAM_SPI_CAMERA;
  } 
#if defined(SUPPORT_SHT30) || defined(SUPPORT_SGP30)
  else if (td_init_air_quality_hat() == NRC_SUCCESS) {
    nrc_usr_print("[%s] found air quality sensor hAT\n", __func__);
    tf_config->accessories |= HAVE_AIR_QUALITY_SENSOR_HAT;
  }
#endif
#if defined(SUPPORT_ETHERNET_ACCESSPOINT) && defined(ETH_DRIVER_ENC28J60)
  else if (td_check_ethernet_hat() == NRC_SUCCESS) {
    nrc_usr_print("[%s] found ethernet hAT\n", __func__);
    tf_config->accessories |= HAVE_ETHERNET_HAT;
  }
#endif
#if defined(SUPPORT_ETHERNET_ACCESSPOINT) && defined(SUPPORT_ESP_HOSTED)
  else if (td_check_wifi_gw_hat() == NRC_SUCCESS) {
#if defined(SUPPORT_SHT30) || defined(SUPPORT_SGP30)
    td_shutdown_air_quality_hat();
#endif
    nrc_usr_print("[%s] found Wi-Fi gateway hAT\n", __func__);
    tf_config->accessories |= HAVE_WIFI_GW_HAT;
  }
#endif
  else {
    nrc_usr_print("[%s] no accessories found\n", __func__);
#if defined(SUPPORT_SHT30) || defined(SUPPORT_SGP30)
    td_shutdown_air_quality_hat();
    shutdown_wifi_gw();
#endif
  }

  if (!has_ethernet_hat(tf_config) && !has_wifi_gw_hat(tf_config)) {
#if defined(SUPPORT_ETHERNET_ACCESSPOINT)
    set_network_mode(NRC_NETWORK_MODE_NAT);
#endif
  } 
  else if (has_ethernet_hat(tf_config)) {
#if defined(SUPPORT_ETHERNET_ACCESSPOINT) && defined(ETH_DRIVER_ENC28J60)
    uint8_t* eth_mac;
    set_network_mode(NRC_NETWORK_MODE_BRIDGE);
    eth_mac = get_eth_standalone_macaddr();
    ethernet_init(eth_mac);
#endif
  }
  else if (has_wifi_gw_hat(tf_config)) {
#if defined(SUPPORT_ETHERNET_ACCESSPOINT) && defined(SUPPORT_ESP_HOSTED)
    uint8_t* esp32_mac;
    set_network_mode(NRC_NETWORK_MODE_BRIDGE);
    esp32_mac = get_eth_standalone_macaddr();
    esp32_init(esp32_mac);
#endif
  }

  nrc_usr_print("[%s] exit\n", __func__);
  
  return NRC_SUCCESS;
}

/**
 * @brief Shutdown attached accessories
 *
 * Run any necessary shutdown routines for accessories
 *
 * @param wifi configuration ptr
 * @returns nrc_err_t
 */
nrc_err_t
td_shutdown_accessories(td_wifi_config_t* tf_config)
{
  if(has_air_quality_hat(tf_config)){
    td_shutdown_air_quality_hat();
  }
  else if (has_wifi_gw_hat(tf_config)) {
    return shutdown_wifi_gw();
  }
  
  return NRC_SUCCESS;
}
