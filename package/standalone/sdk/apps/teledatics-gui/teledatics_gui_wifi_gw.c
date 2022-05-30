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
 * @file teledatics_gui_wifi_gw.c
 * @author James Ewing
 * @date 5 May 2022
 * @brief Teledatics Wi-Fi GW hAT Routines
 */

#include "teledatics_gui_wifi_gw.h"
#include "nrc_eth_if.h"
#include "standalone.h"
#include "teledatics_gui.h"
#include "wlan/protocol.h"

#include <machine/endian.h>

// Wi-Fi Gateway hAT
#define GPIO_RX_READY_PIN GPIO_09
#define GPIO_HANDSHAKE_PIN GPIO_10
#define GPIO_RESET_PIN GPIO_17

static QueueHandle_t to_slave_queue = NULL;
static QueueHandle_t from_slave_queue = NULL;
static spi_device_t spi;

/**
 * @brief  Next TX buffer in SPI transaction
 * @param  argument: Not used
 * @retval sendbuf - Tx buffer
 */
static uint8_t*
get_tx_buffer(uint8_t* is_valid_tx_buf)
{
  struct gw_payload_header* payload_header;
  uint8_t* payload = NULL;
  uint16_t len = 0;
  interface_buffer_handle_t buf_handle = { 0 };
  static uint8_t sendbuf[MAX_SPI_BUFFER_SIZE] = { 0 };

  *is_valid_tx_buf = 0;

  /* Check if higher layers have anything to transmit, non blocking.
   * If nothing is expected to send, queue receive will fail.
   * In that case only payload header with zero payload
   * length would be transmitted.
   */
  if (pdTRUE == xQueueReceive(to_slave_queue, &buf_handle, 0)) {
    len = buf_handle.payload_len;
  }

  if (len) {

    memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

    *is_valid_tx_buf = 1;

    /* Form Tx header */
    payload_header = (struct gw_payload_header*)sendbuf;
    payload = sendbuf + sizeof(struct gw_payload_header);
    payload_header->len = htole16(len);
    payload_header->offset = htole16(sizeof(struct gw_payload_header));
    payload_header->if_type = buf_handle.if_type;
    payload_header->if_num = buf_handle.if_num;
    memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));
    payload_header->checksum = htole16(
      compute_checksum(sendbuf, sizeof(struct gw_payload_header) + len));
  }

  /* free allocated buffer */
  // 	if (buf_handle.free_buf_handle)
  // 		buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

  return sendbuf;
}

/**
 * @brief  Do SPI transaction if -
 *         a. valid TX buffer is ready at SPI host (nrc7292)
 *         b. valid TX buffer is ready at SPI peripheral (ESP32)
 *         c. Dummy transaction is expected from SPI peripheral (ESP32)
 * @param  argument: Not used
 * @retval None
 */

static void
check_and_execute_spi_transaction(void)
{
  uint8_t is_valid_tx_buf = 0;
  int gpio_handshake;
  int gpio_rx_data_ready;
  uint8_t spi_buf[MAX_SPI_BUFFER_SIZE];
  uint8_t *txbuf, *rxbuf;

  /* handshake line SET -> slave ready for next transaction */
  nrc_gpio_inputb(GPIO_HANDSHAKE_PIN, &gpio_handshake);

  /* data ready line SET -> slave wants to send something */
  nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready);

  if (gpio_handshake == GPIO_LEVEL_HIGH) {

    /* Get next tx buffer to be sent */
    txbuf = rxbuf = get_tx_buffer(&is_valid_tx_buf);

    if ((gpio_rx_data_ready == GPIO_LEVEL_HIGH) || (is_valid_tx_buf)) {

      /* Execute transaction only if EITHER holds true-
       * a. A valid tx buffer to be transmitted towards slave
       * b. Slave wants to send something (Rx for host)
       */
      nrc_spi_start_xfer(&spi);
      nrc_spi_xfer(&spi, (uint8_t*)txbuf, (uint8_t*)rxbuf, MAX_SPI_BUFFER_SIZE);
      // 			retval = HAL_SPI_TransmitReceive(&spi,
      // (uint8_t*)txbuf, (uint8_t *)rxbuff, MAX_SPI_BUFFER_SIZE,
      // HAL_MAX_DELAY);
      nrc_spi_stop_xfer(&spi);
    }
  }
}

/**
 * @brief  Full duplex transaction SPI transaction for ESP32-C3 hardware
 * @param  txbuf: TX SPI buffer
 * @retval NRC_SUCCESS / NRC_FAIL
 */
static nrc_err_t
spi_transaction_esp32s2(uint8_t* txbuff)
{
  uint8_t* rxbuff = NULL;
  interface_buffer_handle_t buf_handle = { 0 };
  struct gw_payload_header* payload_header;
  uint16_t len, offset;
  nrc_err_t retval = NRC_FAIL;
  uint16_t rx_checksum = 0, checksum = 0;
  static uint8_t recv_buf[MAX_SPI_BUFFER_SIZE];

  /* Allocate rx buffer */
  // 	rxbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
  // 	assert(rxbuff);
  // 	memset(rxbuff, 0, MAX_SPI_BUFFER_SIZE);

  // if receiving
  if (!txbuff) {
    /* Even though, there is nothing to send,
     * valid reseted txbuff is needed for SPI driver
     */
    // 		txbuff = (uint8_t *)malloc(MAX_SPI_BUFFER_SIZE);
    // 		assert(txbuff);
    // 		memset(txbuff, 0, MAX_SPI_BUFFER_SIZE);
    rxbuff = txbuff = recv_buf;
  }
  // if sending
  else {
    rxbuff = txbuff;
  }

  /* SPI transaction */
  nrc_spi_start_xfer(&spi);
  retval =
    nrc_spi_xfer(&spi, (uint8_t*)txbuff, (uint8_t*)rxbuff, MAX_SPI_BUFFER_SIZE);
  nrc_spi_stop_xfer(&spi);
  // 	HAL_GPIO_WritePin(USR_SPI_CS_GPIO_Port, USR_SPI_CS_Pin, GPIO_PIN_RESET);
  // 	retval = HAL_SPI_TransmitReceive(&spi, (uint8_t*)txbuff,
  // 			(uint8_t *)rxbuff, MAX_SPI_BUFFER_SIZE, HAL_MAX_DELAY);
  // 	while( spi.State == HAL_SPI_STATE_BUSY );
  // 	HAL_GPIO_WritePin(USR_SPI_CS_GPIO_Port, USR_SPI_CS_Pin, GPIO_PIN_SET);

  switch (retval) {
    case NRC_SUCCESS:

      /* Transaction successful */

      /* create buffer rx handle, used for processing */
      payload_header = (struct gw_payload_header*)rxbuff;

      /* Fetch length and offset from payload header */
      len = le16toh(payload_header->len);
      offset = le16toh(payload_header->offset);

      if ((!len) || (len > MAX_PAYLOAD_SIZE) ||
          (offset != sizeof(struct gw_payload_header))) {

        /* Free up buffer, as one of following -
         * 1. no payload to process
         * 2. input packet size > driver capacity
         * 3. payload header size mismatch,
         * wrong header/bit packing?
         * */
        // 				if (rxbuff) {
        // 					free(rxbuff);
        // 					rxbuff = NULL;
        // 				}
        /* Give chance to other tasks */
        // 				osDelay(0);

      } else {
        rx_checksum = le16toh(payload_header->checksum);
        payload_header->checksum = 0;

        checksum = compute_checksum(rxbuff, len + offset);

        if (checksum == rx_checksum) {
          buf_handle.priv_buffer_handle = rxbuff;
          // 					buf_handle.free_buf_handle =
          // free;
          buf_handle.payload_len = len;
          buf_handle.if_type = payload_header->if_type;
          buf_handle.if_num = payload_header->if_num;
          buf_handle.payload = rxbuff + offset;
          buf_handle.seq_num = le16toh(payload_header->seq_num);
          buf_handle.flag = payload_header->flags;

          if (pdTRUE !=
              xQueueSend(from_slave_queue, &buf_handle, portMAX_DELAY)) {
            printf("Failed to send buffer\n\r");
            goto done;
          }
        } else {
          // 					if (rxbuff) {
          // 						free(rxbuff);
          // 						rxbuff = NULL;
          // 					}
        }
      }

      /* Free input TX buffer */
      // 			if (txbuff) {
      // 				free(txbuff);
      // 				txbuff = NULL;
      // 			}
      break;

      // 		case HAL_TIMEOUT:
      // 			printf("timeout in SPI transaction\n\r");
      // 			goto done;
      // 			break;
      //
      // 		case HAL_ERROR:
      // 			printf("Error in SPI transaction\n\r");
      // 			goto done;
      // 			break;
    case NRC_FAIL:
    default:
      printf("default handler: Error in SPI transaction\n\r");
      goto done;
      break;
  }

  return NRC_SUCCESS;

done:
  /* error cases, abort */
  // 	if (txbuff) {
  // 		free(txbuff);
  // 		txbuff = NULL;
  // 	}
  //
  // 	if (rxbuff) {
  // 		free(rxbuff);
  // 		rxbuff = NULL;
  // 	}
  return NRC_FAIL;
}

/**
 * @brief Identify TD-XPAH Wi-Fi GW hAT
 *
 * Identify if Wi-Fi gateway add-on board is attached
 *
 * @param none
 * @returns nrc_err_t
 */
nrc_err_t
td_check_wifi_gw_hat(void)
{

  int i = 0;
  int rev;
  spi_device_t spi;
  NRC_GPIO_CONFIG config;
  int gpio_handshake_pin_val = 0;
  int gpio_rx_data_ready_pin_val = 0;

  config.gpio_pin = GPIO_RX_READY_PIN;
  config.gpio_dir = GPIO_INPUT;
  config.gpio_alt = GPIO_NOMAL_OP;
  config.gpio_mode = GPIO_FLOATING;

  nrc_gpio_config(&config);

  config.gpio_pin = GPIO_HANDSHAKE_PIN;
  config.gpio_dir = GPIO_INPUT;
  config.gpio_alt = GPIO_NOMAL_OP;
  config.gpio_mode = GPIO_FLOATING;

  nrc_gpio_config(&config);

  config.gpio_pin = GPIO_RESET_PIN;
  config.gpio_dir = GPIO_OUTPUT;
  config.gpio_alt = GPIO_NOMAL_OP;
  config.gpio_mode = GPIO_FLOATING;
  nrc_gpio_config(&config);

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
  _delay_ms(20);

  nrc_gpio_outputb(GPIO_RESET_PIN, GPIO_LEVEL_LOW);
  _delay_ms(100);
  nrc_gpio_outputb(GPIO_RESET_PIN, GPIO_LEVEL_HIGH);
  _delay_ms(50);

  nrc_gpio_inputb(GPIO_HANDSHAKE_PIN, &gpio_handshake_pin_val);
  nrc_gpio_inputb(GPIO_RX_READY_PIN, &gpio_rx_data_ready_pin_val);

  // check payload header

  nrc_spi_enable(&spi, false);

  return NRC_FAIL;
}
