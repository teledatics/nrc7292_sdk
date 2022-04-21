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
 * @file teledatics_gui_ppp.c
 * @author James Ewing
 * @date 27 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */

#include "teledatics_gui.h"

#include "lwip/dns.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef PPPOS_SUPPORT
#define PPPOS_SUPPORT 0
#endif /* PPPOS_SUPPORT */

#if PPPOS_SUPPORT
#include "netif/ppp/pppos.h"
#include "lwip/sio.h"
#define PPP_PTY_TEST 1
#endif /* PPPOS_SUPPORT */

#if PPPOS_SUPPORT
static sio_fd_t ppp_sio;
static ppp_pcb *ppp;
static struct netif pppos_netif;

#define PPP_UART NRC_UART_CH2
#define PPP_UART_BASE HSUART2_BASE_ADDR
#define PPP_BAUD_RATE 115200

static NRC_UART_CONFIG sio_uart_dev;
static QueueHandle_t ppp_recv_queue;

/**
 * @brief ppp specific slprintf 
 *
 * Implementation of slprintf for ppp debugging
 *
 * @param buffer to tansfer formatted string to
 * @param length of buffer
 * @param format string
 * @param variadic paramters
 * @return number of characters transferred
 */
int ppp_slprintf(char *buf, int buflen, const char *fmt, ...)
{
        int n = 0;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(buf, buflen, fmt, ap);
        va_end(ap);

        if (n < 0 || n >= buflen)
                return 0;

        nrc_usr_print("[%s]%s\n", "dbglog", buf);
        
        return n;
}

/**
 * @brief ppp specific strlcpy 
 *
 * Implementation of strlcpy for ppp debugging
 *
 * @param destination string
 * @param source string
 * @param max number of characters to transfer
 * @return number of characters transferred
 */
size_t ppp_strlcpy(char *dest, const char *src, size_t len)
{
        return strlcpy(dest, src, len);
}

/**
 * @brief ppp debug print routine
 *
 * Implementation of debug message printing
 *
 * @param format string
 * @param source string
 * @param variadic paramters
 * @return none
 */
void ppp_dbglog(const char *fmt, ...)
{
        int n = 0;
        size_t size = 0;
        char *p = NULL;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n < 0)
                return;

        /* One extra byte for '\0' */

        size = (size_t) n + 1;
        p = malloc(size);
        if (p == NULL)
                return;

        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n >= 0) {
                nrc_usr_print("[%s]%s\n", "dbglog", p);
        }

        free(p);
}


/**
 * @brief ppp debug info print routine
 *
 * Implementation of debug message printing
 *
 * @param format string
 * @param source string
 * @param variadic paramters
 * @return none
 */
void ppp_info(const char *fmt, ...)
{
        int n = 0;
        size_t size = 0;
        char *p = NULL;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n < 0)
                return;

        /* One extra byte for '\0' */

        size = (size_t) n + 1;
        p = malloc(size);
        if (p == NULL)
                return;

        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n >= 0) {
                nrc_usr_print("[%s]%s\n", "info", p);
        }

        free(p);
}


/**
 * @brief ppp debug notice print routine
 *
 * Implementation of debug message printing
 *
 * @param format string
 * @param source string
 * @param variadic paramters
 * @return none
 */
void ppp_notice(const char *fmt, ...)
{
        int n = 0;
        size_t size = 0;
        char *p = NULL;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n < 0)
                return;

        /* One extra byte for '\0' */

        size = (size_t) n + 1;
        p = malloc(size);
        if (p == NULL)
                return;

        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n >= 0) {
                nrc_usr_print("[%s]%s\n", "notice", p);
        }

        free(p);
}


/**
 * @brief ppp debug warn print routine
 *
 * Implementation of debug message printing
 *
 * @param format string
 * @param source string
 * @param variadic paramters
 * @return none
 */
void ppp_warn(const char *fmt, ...)
{
        int n = 0;
        size_t size = 0;
        char *p = NULL;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n < 0)
                return;

        /* One extra byte for '\0' */

        size = (size_t) n + 1;
        p = malloc(size);
        if (p == NULL)
                return;

        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n >= 0) {
                nrc_usr_print("[%s]%s\n", "warn", p);
        }

        free(p);
}


/**
 * @brief ppp debug error print routine
 *
 * Implementation of debug message printing
 *
 * @param format string
 * @param source string
 * @param variadic paramters
 * @return none
 */
void ppp_error(const char *fmt, ...)
{
        int n = 0;
        size_t size = 0;
        char *p = NULL;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n < 0)
                return;

        /* One extra byte for '\0' */

        size = (size_t) n + 1;
        p = malloc(size);
        if (p == NULL)
                return;

        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n >= 0) {
                nrc_usr_print("[%s]%s\n", "error", p);
        }

        free(p);
}


/**
 * @brief ppp debug fatal print routine
 *
 * Implementation of debug message printing
 *
 * @param format string
 * @param source string
 * @param variadic paramters
 * @return none
 */
void ppp_fatal(const char *fmt, ...)
{
        int n = 0;
        size_t size = 0;
        char *p = NULL;
        va_list ap;

        /* Determine required size */
        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n < 0)
                return;

        /* One extra byte for '\0' */

        size = (size_t) n + 1;
        p = malloc(size);
        if (p == NULL)
                return;

        va_start(ap, fmt);
        n = vsnprintf(p, size, fmt, ap);
        va_end(ap);

        if (n >= 0) {
                nrc_usr_print("[%s]%s\n", "fatal", p);
        }

        free(p);
}

volatile int uart_busy(void)
{
        volatile uint32_t busy;
        
        busy = (uint32_t)(RegHSUART_FR(PPP_UART_BASE) & FR_BUSY);
        
        return (int)busy;
}

/**
 * @brief ppp uart output isr
 *
 * Interrupt service routine to send chars to uart
 *
 * @param optional callback vector
 * @return none
 */
void ppp_uart_isr(int vector)
{
        BaseType_t xHigherPriorityTaskWoken;
        NRC_UART_INT_TYPE int_type;
        char recv_char;

        if(uart_busy())
                return;

        int_type = nrc_uart_get_interrupt_type(sio_uart_dev.ch, &int_type);

        if(NRC_UART_INT_ERROR == int_type)
                return;

        if(NRC_UART_INT_RX_DONE == int_type)
                return;

        xHigherPriorityTaskWoken = pdFALSE;

        while(nrc_uart_get(sio_uart_dev.ch, &recv_char) == NRC_SUCCESS)
        {
                xQueueSendToBackFromISR( ppp_recv_queue, &recv_char, &xHigherPriorityTaskWoken );
                
                 if(uart_busy())
                         break;
        }
}

/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
        NRC_UART_INT_TYPE int_type;
        gpio_io_t gpio;
        uio_sel_t uio;
        
        if(nrc_uart_set_channel(devnum) != NRC_SUCCESS) {
                nrc_usr_print("[%s] nrc_uart_set_channel failed\n", __func__);
                return NULL;
        }
        
        sio_uart_dev.ch = devnum;
	sio_uart_dev.db = NRC_UART_DB8;                        /**< Data bit */
	sio_uart_dev.br = PPP_BAUD_RATE;                       /**< Baud rate */
	sio_uart_dev.stop_bit = NRC_UART_SB1;                  /**< Stop bit */
	sio_uart_dev.parity_bit = NRC_UART_PB_NONE;            /**< Parity bit */
	sio_uart_dev.hw_flow_ctrl = NRC_UART_HFC_DISABLE;      /**< HW flow control */
	sio_uart_dev.fifo = NRC_UART_FIFO_ENABLE;
                
        if(nrc_uart_set_config(&sio_uart_dev) != NRC_SUCCESS) {
                nrc_usr_print("[%s] nrc_uart_set_config failed\n", __func__);
                return NULL;
        }

        if(nrc_uart_register_interrupt_handler(devnum, ppp_uart_isr) != NRC_SUCCESS) {
                nrc_usr_print("[%s] nrc_uart_set_interrupt failed\n", __func__);
                return NULL;
        }

        if(nrc_uart_clear_interrupt(devnum, 1, 1, 1) != NRC_SUCCESS) {
                nrc_usr_print("[%s] nrc_uart_clear_interrupt failed\n", __func__);
                return NULL;
        }

        if(nrc_uart_set_interrupt(devnum, 0, 1) != NRC_SUCCESS) {
                nrc_usr_print("[%s] nrc_uart_set_interrupt failed\n", __func__);
                return NULL;
        }

        nrc_usr_print("[%s] successfully opened %d\n", __func__, devnum);

        return &sio_uart_dev;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{       
        while(nrc_uart_put(sio_uart_dev.ch, c) != NRC_SUCCESS)
                ;
        
        return;
}

/**
 * Receives a single character from the serial device.
 *
 * @param fd serial device handle
 *
 * @note This function will block until a character is received.
 */
u8_t sio_recv(sio_fd_t fd)
{
        char recv_ch = -1;

        while(xQueuePeek(ppp_recv_queue, &recv_ch, (TickType_t)10)){
               taskYIELD();
        }

        xQueueReceive(ppp_recv_queue, &recv_ch, (TickType_t)10);

        return recv_ch;
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
        char recv_ch = -1;
        u32_t recv_len = 0;

        while(recv_len < len && xQueueReceive(ppp_recv_queue, &recv_ch, (TickType_t)10)){
                data[recv_len++] = recv_ch;
        }

        return recv_len;
}

/**
 * Writes to the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data to send
 * @param len length (in bytes) of data to send
 * @return number of bytes actually sent
 *
 * @note This function will block until all data can be sent.
 */
u32_t sio_write(sio_fd_t fd, u8_t *data, u32_t len)
{
        u32_t sent = 0;
        int ret = -1;
        
        while(sent < len)
        {
                ret = nrc_uart_put(sio_uart_dev.ch, data[sent]);
                
                if(ret != NRC_SUCCESS) {
                        nrc_usr_print("[%s]: failed to write %d bytes of %d total\n\r", __func__, sent, len);
                }
                else {
                        sent++;
                }
                
                while(uart_busy()) {
                        RegHSUART_IMSC(PPP_UART_BASE) |= IMSC_TX;
                        taskYIELD();
                };
                
        }

        return sent;
}

/**
 * @brief blocking ppp read abort
 * 
 * Aborts a blocking sio_read() call.
 *
 * @param fd serial device handle
 * @return none
 */
void sio_read_abort(sio_fd_t fd)
{
        return;
}

/**
 * @brief ppp receive chars task
 * 
 * Reads uart chars from a queue
 *
 * @param arg ptr
 * @return none
 */
static void pppos_rx_thread(void *arg)
{
        u32_t len;
        u8_t buffer[1024];
        LWIP_UNUSED_ARG(arg);

        while (1) 
        {
                len = sio_read(ppp_sio, buffer, sizeof(buffer));

                if (len > 0) {
                        pppos_input_tcpip(ppp, buffer, len);
                }
        }

        vTaskDelete(NULL);
}

/**
 * @brief ppp network status callback
 * 
 * Report network status based on current state
 *
 * @param network interface
 * @param status code
 * @param arbitary ptr
 * @return none
 */
static void ppp_link_status_cb(ppp_pcb *pcb, int err_code, void *ctx)
{
        struct netif *pppif = ppp_netif(pcb);
        LWIP_UNUSED_ARG(ctx);

 
        switch(err_code) {
                case PPPERR_NONE:               /* No error. */
                        {
#if LWIP_DNS
                                const ip_addr_t *ns;
#endif /* LWIP_DNS */
                                nrc_usr_print("ppp_link_status_cb: PPPERR_NONE\n\r");
#if LWIP_IPV4
                                nrc_usr_print("   our_ip4addr = %s\n\r", ip4addr_ntoa(netif_ip4_addr(pppif)));
                                nrc_usr_print("   his_ipaddr  = %s\n\r", ip4addr_ntoa(netif_ip4_gw(pppif)));
                                nrc_usr_print("   netmask     = %s\n\r", ip4addr_ntoa(netif_ip4_netmask(pppif)));
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
                                nrc_usr_print("   our_ip6addr = %s\n\r", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* LWIP_IPV6 */

#if LWIP_DNS
                                ns = dns_getserver(0);
                                nrc_usr_print("   dns1        = %s\n\r", ipaddr_ntoa(ns));
                                ns = dns_getserver(1);
                                nrc_usr_print("   dns2        = %s\n\r", ipaddr_ntoa(ns));
#endif /* LWIP_DNS */
#if PPP_IPV6_SUPPORT
                                nrc_usr_print("   our6_ipaddr = %s\n\r", ip6addr_ntoa(netif_ip6_addr(pppif, 0)));
#endif /* PPP_IPV6_SUPPORT */
                        }
                        break;

                case PPPERR_PARAM:             /* Invalid parameter. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_PARAM\n");
                        break;

                case PPPERR_OPEN:              /* Unable to open PPP session. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_OPEN\n");
                        break;

                case PPPERR_DEVICE:            /* Invalid I/O device for PPP. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_DEVICE\n");
                        break;

                case PPPERR_ALLOC:             /* Unable to allocate resources. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_ALLOC\n");
                        break;

                case PPPERR_USER:              /* User interrupt. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_USER\n");
                        break;

                case PPPERR_CONNECT:           /* Connection lost. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_CONNECT\n");
                        break;

                case PPPERR_AUTHFAIL:          /* Failed authentication challenge. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_AUTHFAIL\n");
                        break;

                case PPPERR_PROTOCOL:          /* Failed to meet protocol. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_PROTOCOL\n");
                        break;

                case PPPERR_PEERDEAD:          /* Connection timeout. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_PEERDEAD\n");
                        break;

                case PPPERR_IDLETIMEOUT:       /* Idle Timeout. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_IDLETIMEOUT\n");
                        break;

                case PPPERR_CONNECTTIME:       /* PPPERR_CONNECTTIME. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_CONNECTTIME\n");
                        break;

                case PPPERR_LOOPBACK:          /* Connection timeout. */
                        nrc_usr_print("ppp_link_status_cb: PPPERR_LOOPBACK\n");
                        break;

                default:
                        nrc_usr_print("ppp_link_status_cb: unknown errCode %d\n", err_code);
                        break;
        }
}

/**
 * @brief ppp output callback
 * 
 * PPP generic output callback routine
 *
 * @param network interface
 * @param data to output
 * @param len to output
 * @param arbitary ptr
 * @return number of bytes written
 */
static u32_t ppp_output_cb(ppp_pcb *pcb, u8_t *data, u32_t len, void *ctx)
{
        LWIP_UNUSED_ARG(pcb);
        LWIP_UNUSED_ARG(ctx);

        return sio_write(ppp_sio, data, len);
}

#if LWIP_NETIF_STATUS_CALLBACK
/**
 * @brief ppp network status callback
 * 
 * PPP network status callback routine
 *
 * @param network interface
 * @return none
 */
static void netif_status_callback(struct netif *nif)
{
        nrc_usr_print("PPPNETIF: %c%c%d is %s\n", nif->name[0], nif->name[1], nif->num, netif_is_up(nif) ? "UP" : "DOWN");
#if LWIP_IPV4
        nrc_usr_print("IPV4: Host at %s ", ip4addr_ntoa(netif_ip4_addr(nif)));
        nrc_usr_print("mask %s ", ip4addr_ntoa(netif_ip4_netmask(nif)));
        nrc_usr_print("gateway %s\n", ip4addr_ntoa(netif_ip4_gw(nif)));
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
        nrc_usr_print("IPV6: Host at %s\n", ip6addr_ntoa(netif_ip6_addr(nif, 0)));
#endif /* LWIP_IPV6 */
#if LWIP_NETIF_HOSTNAME
        nrc_usr_print("FQDN: %s\n", netif_get_hostname(nif));
#endif /* LWIP_NETIF_HOSTNAME */
}
#endif /* LWIP_NETIF_STATUS_CALLBACK */
#endif

static void pppos_run(void* arg)
{
#if PPPOS_SUPPORT
        ppp_sio = sio_open(PPP_UART);

        if(!ppp_sio)
        {
                nrc_usr_print("PPPOS: Error opening device");
                return;
        }

        ppp = pppos_create(&pppos_netif, ppp_output_cb, ppp_link_status_cb, NULL);
        if (!ppp)
        {
                nrc_usr_print("PPPOS: Could not create PPP control interface");
                return;
        }

#if LWIP_NETIF_STATUS_CALLBACK
        netif_set_status_callback(&pppos_netif, netif_status_callback);
#endif /* LWIP_NETIF_STATUS_CALLBACK */

        xTaskCreate(pppos_rx_thread, "pppos_rx_thread", 4096, NULL, 5, NULL);

        // NOTE: see lib/lwip/port/lwipopts.h to turn on AUTH features
//         ppp_set_auth(ppp, PPPAUTHTYPE_NONE, NULL, NULL);

        ppp_set_default(ppp);

        nrc_usr_print("running ppp_connect\n");
        
        ppp_connect(ppp, 0);

#endif /* PPPOS_SUPPORT */
}

/**
 * @brief start ppp
 * 
 * Start PPP networking and interface
 * 
 * @param none
 * @returns int, 0 on success
 */
int td_start_ppp(void)
{
        ppp_recv_queue = xQueueCreate(128, sizeof(char));

        pppos_run(NULL);

        return 0;
}
 
/**
 * @brief stop ppp
 * 
 * Stop PPP networking & interface
 * 
 * @param none
 * @returns int, 0 on success
 */
int td_stop_ppp(void)
{
        vQueueDelete(ppp_recv_queue);
        return 0;
}
