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
 * @file teledatics_gui_ethernet.c
 * @author James Ewing
 * @date 8 Apr 2022
 * @brief Teledatics Ethernet hAT Routines
 */

#include "teledatics_gui_ethernet.h"
#include "nrc_eth_if.h"
#include "standalone.h"
#include "teledatics_gui.h"
#include "wlan/protocol.h"

/**
 * @brief ENC28J60 SPI write op routine
 *
 * Access register on ENC28J60 Ethernet chip via SPI
 *
 * @param spi device handle
 * @param byte operation
 * @param enc28j60 address
 * @param value to write
 * @returns nrc_err_t
 */
static nrc_err_t
spi_write_op(spi_device_t* spi, uint8_t op, uint8_t addr, uint8_t val)
{
  int ret;
  static uint8_t spi_transfer_buf[8];

  memset(spi_transfer_buf, 0, sizeof(spi_transfer_buf));

  spi_transfer_buf[0] = op | (addr & ADDR_MASK);
  spi_transfer_buf[1] = val;
  nrc_spi_start_xfer(spi);
  ret = nrc_spi_xfer(spi, spi_transfer_buf, spi_transfer_buf, 2);
  nrc_spi_stop_xfer(spi);

  return ret;
}

/**
 * @brief ENC28J60 SPI read op routine
 *
 * Access register on ENC28J60 Ethernet chip via SPI
 *
 * @param spi device handle
 * @param byte operation
 * @param enc28j60 address
 * @returns byte value read
 */
static uint8_t
spi_read_op(spi_device_t* spi, uint8_t op, uint8_t addr)
{
  uint8_t tx_buf[2];
  uint8_t rx_buf[4];
  int slen = SPI_OPLEN;

  /* do dummy read if needed */
  if (addr & SPRD_MASK)
    slen++;

  memset(rx_buf, 0, sizeof(rx_buf));
  tx_buf[0] = op | (addr & ADDR_MASK);
  nrc_spi_start_xfer(spi);
  nrc_spi_xfer(spi, tx_buf, rx_buf, 1);
  memset(tx_buf, 0, sizeof(tx_buf));
  nrc_spi_xfer(spi, tx_buf, rx_buf, slen);
  nrc_spi_stop_xfer(spi);

  return rx_buf[slen - 1];
}

/**
 * @brief ENC28J60 SPI set register bank routine
 *
 * Set register bank on ENC28J60 Ethernet chip via SPI
 *
 * @param spi device handle
 * @param enc28j60 address
 * @returns none
 */
static void
enc28j60_set_bank(spi_device_t* spi, uint8_t addr)
{
  uint8_t b = (addr & BANK_MASK) >> 5;
  static uint8_t bank = 0;

  /* These registers (EIE, EIR, ESTAT, ECON2, ECON1)
   * are present in all banks, no need to switch bank.
   */
  if (addr >= EIE && addr <= ECON1)
    return;

  /* Clear or set each bank selection bit as needed */
  if ((b & ECON1_BSEL0) != (bank & ECON1_BSEL0)) {
    if (b & ECON1_BSEL0)
      spi_write_op(spi, ENC28J60_BIT_FIELD_SET, ECON1, ECON1_BSEL0);
    else
      spi_write_op(spi, ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL0);
  }
  if ((b & ECON1_BSEL1) != (bank & ECON1_BSEL1)) {
    if (b & ECON1_BSEL1)
      spi_write_op(spi, ENC28J60_BIT_FIELD_SET, ECON1, ECON1_BSEL1);
    else
      spi_write_op(spi, ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL1);
  }
  bank = b;
}

/**
 * @brief ENC28J60 register write routine
 *
 * Write data to register on ENC28J60 Ethernet chip via SPI
 *
 * @param spi device handle
 * @param enc28j60 address
 * @param value to write
 * @returns none
 */
static void
spi_regb_write(spi_device_t* spi, uint8_t address, uint8_t data)
{
  enc28j60_set_bank(spi, address);
  spi_write_op(spi, ENC28J60_WRITE_CTRL_REG, address, data);
}

/**
 * @brief ENC28J60 SPI register read routine
 *
 * Read register data on ENC28J60 Ethernet chip via SPI
 *
 * @param spi device handle
 * @param enc28j60 address
 * @returns byte register value
 */
static int
spi_regb_read(spi_device_t* spi, uint8_t address)
{
  enc28j60_set_bank(spi, address);
  return spi_read_op(spi, ENC28J60_READ_CTRL_REG, address);
}

/**
 * @brief Reset ENC28J60 device
 *
 * Reset Ethernet board
 *
 * @param none
 * @returns none
 */
static void
enc28j60_soft_reset(spi_device_t* spi)
{
  spi_write_op(spi, ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
}

/**
 * @brief Identify TD-XPAH Ethernet hAT
 *
 * Identify if Ethernet add-on board is attached
 *
 * @param none
 * @returns nrc_err_t
 */
nrc_err_t
td_check_ethernet_hat(void)
{
  int i = 0;
  int rev;
  spi_device_t spi;
  NRC_GPIO_CONFIG config;

  config.gpio_pin = GPIO_10;
  config.gpio_dir = GPIO_INPUT;
  config.gpio_alt = GPIO_FUNC;
  config.gpio_mode = GPIO_FLOATING;

  nrc_gpio_config(&config);

  config.gpio_pin = GPIO_09;
  config.gpio_dir = GPIO_OUTPUT;
  config.gpio_alt = GPIO_FUNC;
  config.gpio_mode = GPIO_FLOATING;

  nrc_gpio_config(&config);

  /* reset chip */
  nrc_gpio_outputb(GPIO_09, GPIO_LEVEL_LOW);
  _delay_ms(100);
  nrc_gpio_outputb(GPIO_09, GPIO_LEVEL_HIGH);
  _delay_ms(100);

  /* set spi config */
  spi.pin_miso = 12;
  spi.pin_mosi = 13;
  spi.pin_cs = 14;
  spi.pin_sclk = 15;
  spi.frame_bits = SPI_BIT8;
  spi.clock = 12000000;
  spi.mode = SPI_MODE0;
  spi.controller = SPI_CONTROLLER_SPI0;
  spi.bit_order = SPI_MSB_ORDER;
  spi.irq_save_flag = 0;
  spi.isr_handler = NULL;

  nrc_spi_master_init(&spi);
  nrc_spi_enable(&spi, true);
  _delay_ms(100);

  enc28j60_soft_reset(&spi);
  _delay_ms(100);
  spi_write_op(&spi, ENC28J60_WRITE_CTRL_REG, ECON1, 0x00);
  spi_regb_write(&spi, ECON2, ECON2_AUTOINC | ECON2_VRPS);
  rev = spi_regb_read(&spi, EREVID);
  nrc_usr_print("chip RevID: 0x%02x\n", rev);
  nrc_spi_enable(&spi, false);

  /* reset chip */
  nrc_gpio_outputb(GPIO_09, GPIO_LEVEL_LOW);
  _delay_ms(100);
  nrc_gpio_outputb(GPIO_09, GPIO_LEVEL_HIGH);
  _delay_ms(100);
  
  if (rev >= ENC28J60_REV_B1 && rev <= ENC28J60_REV_B7) {
    return NRC_SUCCESS;
  }

  return NRC_FAIL;
}

/**
 * @brief Create unique ethernet MAC address
 *
 * Derive unique ethernet MAC address from
 * existing Wi-Fi MAC addresses
 *
 * @param none
 * @returns ptr to mac address
 */
uint8_t*
get_eth_standalone_macaddr(void)
{
  static uint8_t mac[MAC_ADDR_LEN];

  get_standalone_macaddr(0, mac);

  mac[MAC_ADDR_LEN - 1] += 2; // add MAC address after vif[1]

  // skip zero
  if (!mac[MAC_ADDR_LEN - 1]) {
    mac[MAC_ADDR_LEN - 1]++;
  }

  return mac;
}
