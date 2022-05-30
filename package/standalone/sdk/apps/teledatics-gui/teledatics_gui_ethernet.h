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
 * @file teledatics_gui_ethernet.h
 * @author James Ewing
 * @date 8 Apr 2022
 * @brief Teledatics Ethernet hAT utilities for Halo TD-XPAH
 */

#include "teledatics_gui.h"

#ifndef TELEDATICS_ETHERNET_H
#define TELEDATICS_ETHERNET_H

// Ethernet hAT
#define ENC28J60_SOFT_RESET 0xFF
#define ADDR_MASK 0x1F
#define BANK_MASK 0x60
#define SPRD_MASK 0x80
#define ECON2 0x1E
#define ECON1 0x1F
#define EIE 0x1B
#define ECON1_BSEL1 0x02
#define ECON1_BSEL0 0x01
#define ECON2_AUTOINC 0x80
#define ECON2_VRPS 0x08
#define EREVID (0x12 | 0x60)
#define ENC28J60_READ_CTRL_REG 0x00
#define ENC28J60_WRITE_CTRL_REG 0x40
#define ENC28J60_BIT_FIELD_SET 0x80
#define ENC28J60_BIT_FIELD_CLR 0xA0
#define ENC28J60_REV_B1 0x2
#define ENC28J60_REV_B7 0x6
#define SPI_OPLEN 1

nrc_err_t td_check_ethernet_hat(void);
uint8_t* get_eth_standalone_macaddr(void);

#endif /* TELEDATICS_ETHERNET_H */
