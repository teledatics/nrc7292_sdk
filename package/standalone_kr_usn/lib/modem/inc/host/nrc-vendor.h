/*
 * Copyright (c) 2016-2019 Newracom, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _NRC_VENDOR_H_
#define _NRC_VENDOR_H_

#define OUI_NRC								0xFCFFAA
#define NRC_SUBCMD_WOWLAN_PATTERN			0xB0
#define NRC_OUI_SUBCMD_ANNOUNCE1			0xF0
#define NRC_OUI_SUBCMD_ANNOUNCE2			0xF1
#define NRC_OUI_SUBCMD_ANNOUNCE3			0xF2
#define NRC_OUI_SUBCMD_ANNOUNCE4			0xF3
#define NRC_OUI_SUBCMD_ANNOUNCE5			0xF4
#define NRC_OUI_SUBCMD_RM_ANNOUNCE1			0xE0 // remove the vendor specific IE which was injected by 0xF0
#define NRC_OUI_SUBCMD_RM_ANNOUNCE2			0xE1 // remove the vendor specific IE which was injected by 0xF1
#define NRC_OUI_SUBCMD_RM_ANNOUNCE3			0xE2 // remove the vendor specific IE which was injected by 0xF2
#define NRC_OUI_SUBCMD_RM_ANNOUNCE4			0xE3 // remove the vendor specific IE which was injected by 0xF3
#define NRC_OUI_SUBCMD_RM_ANNOUNCE5			0xE4 // remove the vendor specific IE which was injected by 0xF4

/*
 * GPIO pin number on Raspberry Pi.
 * This is used to wake up the target in deep-sleep.
 * [direction]: OUTPUT
 */
#define RPI_GPIO_FOR_PS						(20)

/*
 * GPIO pin number on the target device.
 * This is used to designate the GPIO pin on target
 * which will read signal from RPI_GPIO_FOR_RS for waking up target.
 * [direction]: INPUT
 */
#define TARGET_GPIO_FOR_WAKEUP				(11)

/*
 * GPIO pin number on the target device.
 * This is used to designate the GPIO pin on target
 * which will raise signal to wake up host system.
 * [direction]: OUTPUT
 */
#define TARGET_GPIO_FOR_WAKEUP_HOST			(10)

#define RPI_GPIO_FOR_RST					(4)

enum nrc_vendor_event {
	NRC_VENDOR_EVENT_ANNOUNCE = 0,
	NRC_VENDOR_EVENT_WOWLAN,
	NUM_VENDOR_EVENT,
	MAX_VENDOR_EVENT = NUM_VENDOR_EVENT - 1
};

enum nrc_vendor_attributes {
	NRC_VENDOR_ATTR_DATA = 0,
	NUM_VENDOR_ATTR,
	MAX_VENDOR_ATTR = NUM_VENDOR_ATTR - 1
};

#endif /* _NRC_VENDOR_H_ */
