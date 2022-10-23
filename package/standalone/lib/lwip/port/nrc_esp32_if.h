#ifndef __NRC_ESP32_IF_H__
#define __NRC_ESP32_IF_H__

#include "nrc_types.h"
#include "lwip/prot/ethernet.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp32_mac_s esp32_mac_t;

struct esp32_mac_s {
	nrc_err_t (*transmit)(esp32_mac_t *mac, uint8_t *buf, uint16_t length);
	nrc_err_t (*receive)(esp32_mac_t *mac, uint8_t *buf, uint16_t *length);
	nrc_err_t (*get_addr)(esp32_mac_t *mac, uint8_t *addr);
};

nrc_err_t esp32_init(uint8_t *mac_addr);
esp32_mac_t* esp_hosted_new_mac(void);

#endif /* __NRC_ESP32_IF_H__ */
