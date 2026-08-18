/*
 * Copyright (c) 2026 Benjamin Lee McQueen <mcq@disroot.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
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

#ifndef _SYS_DEV_DDCVAR_H_
#define _SYS_DEV_DDCVAR_H_

#include <sys/device.h>
#include <dev/i2c/i2cvar.h>
#include <linux/i2c.h>

int ddc_register(struct device *, struct i2c_adapter *, i2c_addr_t);
void ddc_unregister(struct device *);
int ddc_probe_device(struct device *, struct i2c_adapter *);
int ddc_get_caps(struct device *, struct i2c_adapter *);

#define DDC_CMD_CAPS            0xF3
#define DDC_HOST_ADDR_ODD        0x51
#define DDC_HOST_ADDR_EVEN      0x50
#define DDC_DEFAULT_DEVICE_ADDR  0x6E
#define DDC_HOST_REPLY_CAPS      0xE3
#define DDC_MONITOR_ADDR         0x37
#define DDC_PFLAG                0x80
#define DDC_MAX_CAPS_STRING      16384
#define DDC_MAX_CAP_CHUNKS      ((DDC_MAX_CAPS_STRING / 16) + 8)

#endif /* _SYS_DEV_DDCVAR_H_ */
