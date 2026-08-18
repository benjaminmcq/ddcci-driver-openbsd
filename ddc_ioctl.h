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

#ifndef _SYS_DEV_DDC_IOCTL_H_
#define _SYS_DEV_DDC_IOCTL_H_

#include <sys/ioctl.h>

struct ddc_probe_args {
        char                dpa_name[16];
        uint8_t        *dpa_caps_buf;
        unsigned int        dpa_caps_buf_len;
        unsigned int        dpa_caps_len;
};

#define DDCIOCPROBE        _IOW('D', 0, struct ddc_probe_args)
#define DDCIOCREADCAPS        _IOWR('D', 1, struct ddc_probe_args)

#endif /* _SYS_DEV_DDC_IOCTL_H_ */

