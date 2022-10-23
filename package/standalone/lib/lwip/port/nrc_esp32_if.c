#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/dhcp.h"
#include "netif/etharp.h"
#include "netif/bridgeif.h"

#include "nrc_wifi.h"
#include "driver_nrc.h"

#include "eth.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"

#include "nrc_eth_if.h"
#include "nrc_esp32_if.h"

struct netif wifi_esp32_netif;
extern struct netif br_netif;
extern bridgeif_initdata_t bridge_data;
#if defined(SUPPORT_ETHERNET_ACCESSPOINT)
extern struct eth_addr peer_mac;
#endif

extern esp32_mac_t *esp32_mac;
static nrc_eth_mode_t eth_mode = NRC_ETH_MODE_AP;
static nrc_network_mode_t network_mode = NRC_NETWORK_MODE_BRIDGE;
bool esp32_dhcp_started = false;

extern struct netif* nrc_netif[MAX_IF];

static err_t nrc_esp32_output( struct netif *netif, struct pbuf *p )
{
	struct pbuf *q;
	err_t xReturn = ERR_OK;
	static uint8_t buffer[ETH_MAX_PACKET_SIZE];
	uint32_t length = 0;

	for( q = p; q != NULL; q = q->next )
	{
		memcpy(buffer + length, q->payload, q->len);
		length += q->len;
	}

	esp32_mac->transmit(esp32_mac, buffer, length);

	LINK_STATS_INC(link.xmit);

	return ERR_OK;
}

static err_t wifi_esp32_init( struct netif *netif )
{
	/*
	* Initialize the snmp variables and counters inside the struct netif.
	* The last argument should be replaced with your link speed, in units
	* of bits per second.
	*/
	NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, 100);

	netif->name[0] = 'w';
	netif->name[1] = 'l';

	/* We directly use etharp_output() here to save a function call.
	* You can instead declare your own function an call etharp_output()
	* from it if you have to do some checks before sending (e.g. if link
	* is available...)
	*/
	#if LWIP_IPV4
	netif->output = etharp_output;
	#endif /* LWIP_IPV4 */
	#if LWIP_IPV6
	netif->output_ip6 = ethip6_output;
	#endif /* LWIP_IPV6 */
	netif->linkoutput = nrc_esp32_output;

	/* initialize the hardware */
	/* set MAC hardware address length */
	netif->hwaddr_len = NETIF_MAX_HWADDR_LEN;

	/* maximum transfer unit */
	netif->mtu = 1500;

	/* broadcast capability */
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_ETHERNET;

	return ERR_OK;
}

static void status_callback(struct netif *eth_if)
{
	struct netif *target_if;

	if (network_mode == NRC_NETWORK_MODE_BRIDGE) {
		target_if = &br_netif;
	} else {
		target_if = eth_if;
	}

	if (netif_is_up(eth_if)) {
		if (!ip4_addr_isany_val(*netif_ip4_addr(target_if))) {
			nrc_usr_print("[%s] netif_is_up, local interface IP is %s\n",
					  __func__,
					  ip4addr_ntoa(netif_ip4_addr(target_if)));
			nrc_usr_print("[%s] IP is ready\n", __func__);
		}
	} else {
		nrc_usr_print("[%s] netif_is_down\n", __func__);
	}
}

static void link_callback(struct netif *eth_if)
{
	if (netif_is_link_up(eth_if)) {
		nrc_usr_print("[%s] UP\n", __func__);
	} else {
		nrc_usr_print("[%s] DOWN\n", __func__);
	}
}

static void nrc_bind_esp32_if(esp32_mac_t *mac)
{
    ip4_addr_t ipaddr, netmask, gw;
	const struct eth_addr ethbroadcast = {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};

    ip4_addr_set_zero(&gw);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);

    wifi_esp32_netif.num = 2;
    /* save mac as nrc_eth_mac to be used in LWIP nrc_eth_output callback */
    esp32_mac = mac;

    /* set MAC hardware address to be used by lwIP */
    mac->get_addr(mac, wifi_esp32_netif.hwaddr);
    netif_add(&wifi_esp32_netif, &ipaddr, &netmask, &gw, NULL, wifi_esp32_init, ethernet_input);
    netif_set_status_callback(&wifi_esp32_netif, status_callback);
    netif_set_link_callback(&wifi_esp32_netif, link_callback);
    
    netif_set_flags(&wifi_esp32_netif, NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET);
    
    /* bring it up */
    netif_set_up(&wifi_esp32_netif);

	if (network_mode == NRC_NETWORK_MODE_BRIDGE) {
		if (eth_mode == NRC_ETH_MODE_AP) {
			memcpy(bridge_data.ethaddr.addr, wifi_esp32_netif.hwaddr, 6);
		} else {
			memcpy(bridge_data.ethaddr.addr, nrc_netif[0]->hwaddr, 6);
		}
		bridge_data.max_ports = 2;
		bridge_data.max_fdb_dynamic_entries = 128;
		bridge_data.max_fdb_static_entries = 16;

		netif_add(&br_netif, &ipaddr, &netmask, &gw, &bridge_data, bridgeif_init, ethernet_input);
		
		bridgeif_add_port(&br_netif, &wifi_esp32_netif);

		bridgeif_add_port(&br_netif, nrc_netif[0]);
		
		bridgeif_add_port(&br_netif, nrc_netif[1]);

		bridgeif_fdb_add(&br_netif, &ethbroadcast, BR_FLOOD);
		netif_set_default(&br_netif);
		netif_set_up(&br_netif);
	} else {
		if (eth_mode == NRC_ETH_MODE_AP) {
			netif_set_default(&wifi_esp32_netif);
		}
	}
}

nrc_err_t esp32_init(uint8_t *mac_addr)
{
	esp32_mac_t* mac = NULL;
	
//     eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
//     mac_config.smi_mdc_gpio_num = -1;
//     mac_config.smi_mdio_gpio_num = -1;
// #ifdef ETH_DRIVER_ENC28J60
//     eth_enc28j60_config_t enc_config;
//     enc_config.int_gpio_num = GPIO_10;
//     esp32_mac_t *mac = esp_eth_mac_new_enc28j60(&enc_config, &mac_config);
// #endif
// #ifdef ETH_DRIVER_W5500
// 	esp32_mac_t *mac = esp_eth_mac_new_w5500(&mac_config);
// #endif
//     eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
//     phy_config.autonego_timeout_ms = 0; // ENC28J60 doesn't support auto-negotiation
//     phy_config.reset_gpio_num = -1; // ENC28J60 doesn't have a pin to reset internal PHY
// #ifdef ETH_DRIVER_ENC28J60
//     phy_config.reset_gpio_num = GPIO_09;
//     esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);
// #endif
// #ifdef ETH_DRIVER_W5500
// 	esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
// #endif
//     esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
//     eth_config.stack_input = eth_stack_input_handler;
//     eth_config.on_linkup = eth_linkup_handler;
//     eth_config.on_linkdown = eth_linkdown_handler;
//     esp_eth_handle_t eth_handle = NULL;
    
//     if (esp_eth_driver_install(&eth_config, &eth_handle) != NRC_SUCCESS) {
// 		nrc_usr_print("[%s] Error installing ethernet driver...\n", __func__);
// 		return NRC_FAIL;
// 	}
	
    /* ENC28J60 doesn't burn any factory MAC address, we need to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
// 	if (mac_addr == NULL) {
// 		mac->set_addr(mac, (uint8_t[]) {
// 	0x02, 0x00, 0x00, 0x12, 0x55, 0x00
// 		});
// 	} else {
// 		mac->set_addr(mac, mac_addr);
// 	}

	/* set ethernet interface in promiscuous mode to act as bridge */
// 	mac->set_promiscuous(mac, true);
	
	mac = esp_hosted_new_mac();
	
	/* attach ESP32 driver to TCP/IP stack */
	nrc_bind_esp32_if(mac);

	/* start ESP32 driver state machine */
// 	if (esp32_start(eth_handle) != NRC_SUCCESS) {
// 		nrc_usr_print("[%s] Error starting ethernet...\n", __func__);
// 		return NRC_FAIL;
// 	}

	return NRC_SUCCESS;
}
