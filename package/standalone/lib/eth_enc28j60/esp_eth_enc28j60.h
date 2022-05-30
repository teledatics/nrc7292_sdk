/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
// #include "driver/spi_master.h"
#include "enc28j60.h"
#include "nrc_sdk.h"

#define CS_HOLD_TIME_MIN_NS 210

/* buffer boundaries applied to internal 8K ram
 * entire available packet buffer space is allocated.
 * Give TX buffer space for one full ethernet frame (~1500 bytes)
 * receive buffer gets the rest */
#define TXSTART_INIT 0x1A00
#define TXEND_INIT 0x1FFF

/* Put RX buffer at 0 as suggested by the Errata datasheet */
#define RXSTART_INIT 0x0000
#define RXEND_INIT 0x19FF

/* maximum ethernet frame length */
#define MAX_FRAMELEN 1518

#define SPI_TRANSFER_BUF_LEN (4 + MAX_FRAMELEN)

/*
 * ENC28J60 Control Registers
 * Control register definitions are a combination of address,
 * bank number, and Ethernet/MAC/PHY indicator bits.
 * - Register address	(bits 0-4)
 * - Bank number	(bits 5-6)
 * - MAC/MII indicator	(bit 7)
 */
#define ADDR_MASK 0x1F
#define BANK_MASK 0x60
#define SPRD_MASK 0x80

/* SPI operation codes */
#define ENC28J60_READ_CTRL_REG 0x00
#define ENC28J60_READ_BUF_MEM 0x3A
#define ENC28J60_WRITE_CTRL_REG 0x40
#define ENC28J60_WRITE_BUF_MEM 0x7A
#define ENC28J60_BIT_FIELD_SET 0x80
#define ENC28J60_BIT_FIELD_CLR 0xA0
#define ENC28J60_SOFT_RESET 0xFF

/* ENC28J60 Receive Status Vector */
#define RSV_RXLONGEVDROPEV 0
#define RSV_CARRIEREV 2
#define RSV_CRCERROR 4
#define RSV_LENCHECKERR 5
#define RSV_LENOUTOFRANGE 6
#define RSV_RXOK 7
#define RSV_RXMULTICAST 8
#define RSV_RXBROADCAST 9
#define RSV_DRIBBLENIBBLE 10
#define RSV_RXCONTROLFRAME 11
#define RSV_RXPAUSEFRAME 12
#define RSV_RXUNKNOWNOPCODE 13
#define RSV_RXTYPEVLAN 14

#define RSV_SIZE 6
#define RSV_BITMASK(x) (1 << (x))
#define RSV_GETBIT(x, y) (((x)&RSV_BITMASK(y)) ? 1 : 0)

#define SPI_OPLEN 1

typedef spi_device_t spi_device_handle_t;

/**
* This structure describes one SPI transaction. The descriptor should not be
* modified until the transaction finishes.
*/
struct spi_transaction_t
{
uint32_t flags; ///< Bitwise OR of SPI_TRANS_* flags
uint16_t
cmd; /**< Command data, of which the length is set in the ``command_bits``
* of spi_device_interface_config_t.
*
*  <b>NOTE: this field, used to be "command" in ESP-IDF 2.1 and
* before, is re-written to be used in a new way in ESP-IDF 3.0.</b>
*
*  Example: write 0x0123 and command_bits=12 to send command 0x12,
* 0x3_ (in previous version, you may have to write 0x3_12).
*/
uint64_t
addr;          /**< Address data, of which the length is set in the
* ``address_bits`` of spi_device_interface_config_t.
*
*  <b>NOTE: this field, used to be "address" in ESP-IDF 2.1 and
* before, is re-written to be used in a new way in ESP-IDF3.0.</b>
*
*  Example: write 0x123400 and address_bits=24 to send address of
* 0x12, 0x34, 0x00 (in previous version, you may have to write
* 0x12340000).
*/
size_t length;   ///< Total data length, in bits
size_t rxlength; ///< Total data length received, should be not greater than
 ///< ``length`` in full-duplex mode (0 defaults this to the
 ///< value of ``length``).
void*
user; ///< User-defined variable. Can be used to store eg transaction ID.
union
{
const void*
tx_buffer; ///< Pointer to transmit buffer, or NULL for no MOSI phase
uint8_t tx_data[4]; ///< If SPI_TRANS_USE_TXDATA is set, data set here is
///< sent directly from this variable.
};
union
{
void* rx_buffer; ///< Pointer to receive buffer, or NULL for no MISO
 ///< phase. Written by 4 bytes-unit if DMA is used.
uint8_t rx_data[4]; ///< If SPI_TRANS_USE_RXDATA is set, data is received
///< directly to this variable
};
};

typedef struct spi_transaction_t spi_transaction_t;

#define SPI_TRANS_MODE_DIO (1 << 0) ///< Transmit/receive data in 2-bit mode
#define SPI_TRANS_MODE_QIO (1 << 1) ///< Transmit/receive data in 4-bit mode
#define SPI_TRANS_USE_RXDATA                                                   \
(1 << 2) ///< Receive into rx_data member of spi_transaction_t instead into
 ///< memory at rx_buffer.
#define SPI_TRANS_USE_TXDATA                                                   \
(1 << 3) ///< Transmit tx_data member of spi_transaction_t instead of data at
 ///< tx_buffer. Do not set tx_buffer when using this.
#define SPI_TRANS_MODE_DIOQIO_ADDR                                             \
(1 << 4) ///< Also transmit address in mode selected by
 ///< SPI_MODE_DIO/SPI_MODE_QIO
#define SPI_TRANS_VARIABLE_CMD                                                 \
(1 << 5) ///< Use the ``command_bits`` in ``spi_transaction_ext_t`` rather
 ///< than default value in ``spi_device_interface_config_t``.
#define SPI_TRANS_VARIABLE_ADDR                                                \
(1 << 6) ///< Use the ``address_bits`` in ``spi_transaction_ext_t`` rather
 ///< than default value in ``spi_device_interface_config_t``.
#define SPI_TRANS_VARIABLE_DUMMY                                               \
(1 << 7) ///< Use the ``dummy_bits`` in ``spi_transaction_ext_t`` rather than
 ///< default value in ``spi_device_interface_config_t``.
#define SPI_TRANS_CS_KEEP_ACTIVE                                               \
(1 << 8) ///< Keep CS active after data transfer
#define SPI_TRANS_MULTILINE_CMD   (1 << 9) ///< The data lines used at command phase is the same as data phase
 ///< (otherwise, only one data line is used at command phase)
#define SPI_TRANS_MODE_OCT        (1 << 10) ///< Transmit/receive data in 8-bit mode
#define SPI_TRANS_MULTILINE_ADDR SPI_TRANS_MODE_DIOQIO_ADDR ///< The data lines used at address phase is the
///< same as data phase (otherwise, only one data
///< line is used at address phase)

/**
 * @brief ENC28J60 specific configuration
 *
 */
typedef struct
{
spi_device_handle_t spi_hdl; /*!< Handle of SPI device driver */
int int_gpio_num;            /*!< Interrupt GPIO number */
} eth_enc28j60_config_t;

/**
 * @brief ENC28J60 Supported Revisions
 *
 */
typedef enum
{
  ENC28J60_REV_B1 = 0b00000010,
  ENC28J60_REV_B4 = 0b00000100,
  ENC28J60_REV_B5 = 0b00000101,
  ENC28J60_REV_B7 = 0b00000110
} eth_enc28j60_rev_t;

/**
 * @brief Default ENC28J60 specific configuration
 *
 */
#define ETH_ENC28J60_DEFAULT_CONFIG(spi_device)                                \
{                                                                            \
  .spi_hdl = spi_device, .int_gpio_num = 4,                                  \
}

/**
 * @brief Compute amount of SPI bit-cycles the CS should stay active after the
 * transmission to meet ENC28J60 CS Hold Time specification.
 *
 * @param clock_speed_mhz SPI Clock frequency in MHz (valid range is <1, 20>)
 * @return uint8_t
 */
static inline uint8_t enc28j60_cal_spi_cs_hold_time(int clock_speed_mhz)
{
  if (clock_speed_mhz <= 0 || clock_speed_mhz > 20) {
    return 0;
  }
  
  int temp = clock_speed_mhz * CS_HOLD_TIME_MIN_NS;
  uint8_t cs_posttrans = temp / 1000;
  
  if (temp % 1000) {
    cs_posttrans += 1;
  }

  return cs_posttrans;
}

/**
 * @brief Create ENC28J60 Ethernet MAC instance
 *
 * @param[in] enc28j60_config: ENC28J60 specific configuration
 * @param[in] mac_config: Ethernet MAC configuration
 *
 * @return
 *      - instance: create MAC instance successfully
 *      - NULL: create MAC instance failed because some error occurred
 */
esp_eth_mac_t* esp_eth_mac_new_enc28j60(const eth_enc28j60_config_t* enc28j60_config, const eth_mac_config_t* mac_config);

/**
 * @brief Create a PHY instance of ENC28J60
 *
 * @param[in] config: configuration of PHY
 *
 * @return
 *      - instance: create PHY instance successfully
 *      - NULL: create PHY instance failed because some error occurred
 */
esp_eth_phy_t* esp_eth_phy_new_enc28j60(const eth_phy_config_t* config);

/**
 * @brief Get ENC28J60 silicon revision ID
 *
 * @param mac ENC28J60 MAC Handle
 * @return eth_enc28j60_rev_t
 *           - returns silicon revision ID read during initialization
 */
eth_enc28j60_rev_t emac_enc28j60_get_chip_info(esp_eth_mac_t* mac);

#ifdef __cplusplus
}
#endif
