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

#include <sys/param.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/systm.h>
#include <sys/rwlock.h>

#include <linux/i2c.h>

#include <dev/i2c/i2cvar.h>
#include <dev/i2c/ddcvar.h>
#include <dev/i2c/ddc_ioctl.h>

#define DDC_DEBUG

#ifdef DDC_DEBUG
#define	DPRINTF(x...)		printf(x)
#else
#define	DPRINTF(x...)
#endif

struct ddc_mapping {
	int			 dm_index;
	TAILQ_ENTRY(ddc_mapping) dm_link;
	struct device		*dm_dev;		/* Parent GPU driver instance (referred to as the GPU) */
	struct i2c_adapter	*dm_adapter;	/* I2C device that initiates transfers from within the GPU (I2C master) */
	i2c_addr_t		 dm_addr;		/* Target monitor slave (I2C slave) */
};

static int ddc_next_index = 0;

TAILQ_HEAD(, ddc_mapping) ddcs = TAILQ_HEAD_INITIALIZER(ddcs);
struct rwlock ddcs_lock = RWLOCK_INITIALIZER("ddclk");

int		ddc_register(struct device *, struct i2c_adapter *, i2c_addr_t);
void		ddc_unregister(struct device *);
int		ddc_probe_device(struct device *, struct i2c_adapter *);
int		ddc_get_vcp(struct device *, struct i2c_adapter *, uint8_t,
    uint16_t *);
int		ddc_set_vcp(struct device *, struct i2c_adapter *, uint8_t,
    uint16_t);
unsigned char	ddc_checksum(uint8_t *, unsigned int, i2c_addr_t);
void		ddcattach(int);
int		ddcclose(dev_t, int, int, struct proc *);
int		ddcioctl(dev_t, u_long, caddr_t, int, struct proc *);
int		ddcopen(dev_t, int, int, struct proc *);

/*
 * Allocate a ddc_mapping for the given GPU driver instance,
 * I2C master, slave and monitor slave address and insert it at the
 * end of the TAILQ.
 */
int
ddc_register(struct device *dev, struct i2c_adapter *adapter, i2c_addr_t addr)
{
	struct ddc_mapping *dm;
	dm = malloc(sizeof(*dm), M_DEVBUF, M_WAITOK);

	if (dm == NULL)
		return ENOMEM;

	dm->dm_dev = dev;
	dm->dm_adapter = adapter;
	dm->dm_addr = addr;

	rw_enter_write(&ddcs_lock);
	dm->dm_index = ddc_next_index++;
	TAILQ_INSERT_TAIL(&ddcs, dm, dm_link);
	rw_exit_write(&ddcs_lock);

	return 0;
}

/*
 * Remove and free every ddc_mapping registered for the given
 * device from the registered device list.
 */
void
ddc_unregister(struct device *dev)
{
	struct ddc_mapping *dm, *next;
	rw_enter_write(&ddcs_lock);

	TAILQ_FOREACH_SAFE(dm, &ddcs, dm_link, next) {
		if (dev == dm->dm_dev) {
			TAILQ_REMOVE(&ddcs, dm, dm_link);
			free(dm, M_DEVBUF, sizeof(*dm));
		}
	}

	rw_exit_write(&ddcs_lock);
}

/*
 * Search the registered list of devices for a ddc_mapping whose device
 * field matches the given device.
 * Returns the matching ddc_mapping or NULL if none is found.
 *
 * Send a DDC/CI capabilities request to the device described by the given
 * ddc_mapping, and validate the response utilizing a checksum.
 * Return zero on success, or a negative error code on failure.
 */
int
ddc_probe_device(struct device *dev, struct i2c_adapter *adapter)
{
	struct ddc_mapping *dm;
	struct i2c_msg msg;
	uint8_t data[32], cmd[6];
	int len, ret;
	rw_enter_read(&ddcs_lock);

	TAILQ_FOREACH(dm, &ddcs, dm_link) {
		if (dm->dm_dev == dev && dm->dm_adapter == adapter)
			break;
	}

	if (dm == NULL || dm->dm_dev == NULL || dm->dm_adapter == NULL ||
	    dm->dm_addr == 0) {
		DPRINTF(
		    "ddc_probe_device: one or more of the members in ddc_mapping was NULL!");
		ret = ENOENT;
		goto out;
	}

	len = sizeof(cmd) - 1;

	cmd[0] = DDC_HOST_ADDR_ODD;
	cmd[1] = DDC_PFLAG | 3;
	cmd[2] = DDC_CMD_CAPS;
	cmd[3] = 0;
	cmd[4] = 0;
	cmd[5] = ddc_checksum(cmd, len, DDC_MONITOR_ADDR << 1);

	msg.addr = dm->dm_addr;
	msg.flags = 0;
	msg.buf = cmd;
	msg.len = sizeof(cmd);

	ret = i2c_transfer(dm->dm_adapter, &msg, 1);

	if (ret < 0) {
		DPRINTF("ddc_probe_device: write failed! return value=%d\n",
		    ret);
		goto out;
	}

	tsleep_nsec(dm, PWAIT, "ddc", MSEC_TO_NSEC(60));

	msg.flags = I2C_M_RD;
	msg.buf = data;
	msg.len = sizeof(data);

	ret = i2c_transfer(dm->dm_adapter, &msg, 1);

	if (ret < 0) {
		DPRINTF("ddc_probe_device: read failed! return value=%d\n",
		    ret);
		goto out;
	}

	if (data[0] != DDC_DEFAULT_DEVICE_ADDR) {
		DPRINTF("ddc_probe_device: invalid response! data[0]=0x%x\n",
		    data[0]);
		ret = EIO;
		goto out;
	}

	/* Strip PFLAG add header and checksum byte to get message length */
	len = 2 + (data[1] & 0x7F) + 1;

	if (len > sizeof(data)) {
		DPRINTF("ddc_probe_device: response too long! len=%d\n", len);
		ret = EIO;
		goto out;
	}

	if (ddc_checksum(data, len, DDC_HOST_ADDR_EVEN) != 0) {
		DPRINTF("ddc_probe_device: checksum failed!\n");
		ret = EIO;
		goto out;
	}

	ret = 0;
out:
	rw_exit_read(&ddcs_lock);

	return ret;
}

int
ddc_get_vcp(struct device *dev, struct i2c_adapter *adapter, uint8_t vcp_code,
    uint16_t *value)
{
	struct ddc_mapping *dm;
	struct i2c_msg msg;
	uint8_t cmd[5], data[32];
	int len, ret;

	rw_enter_read(&ddcs_lock);

	TAILQ_FOREACH(dm, &ddcs, dm_link) {
		if (dm->dm_dev == dev && dm->dm_adapter == adapter)
			break;
	}

	if (dm == NULL || dm->dm_dev == NULL || dm->dm_adapter == NULL ||
	    dm->dm_addr == 0) {
		DPRINTF(
		    "ddc_get_vcp: one or more of the members in ddc_mapping was NULL!");
		ret = ENOENT;
		goto out;
	}

	len = sizeof(cmd) - 1;

	cmd[0] = DDC_HOST_ADDR_ODD;
	cmd[1] = DDC_PFLAG | 2;
	cmd[2] = DDC_CMD_GETVCP;
	cmd[3] = vcp_code;
	cmd[4] = ddc_checksum(cmd, len, DDC_MONITOR_ADDR << 1);

	msg.addr = dm->dm_addr;
	msg.flags = 0;
	msg.buf = cmd;
	msg.len = sizeof(cmd);

	ret = i2c_transfer(dm->dm_adapter, &msg, 1);

	if (ret < 0) {
		DPRINTF("ddc_get_vcp: write failed! return value=%d\n",
		    ret);
		goto out;
	}

	tsleep_nsec(dm, PWAIT, "ddc", MSEC_TO_NSEC(60));

	msg.flags = I2C_M_RD;
	msg.buf = data;
	msg.len = sizeof(data);

	ret = i2c_transfer(dm->dm_adapter, &msg, 1);

	if (ret < 0) {
		DPRINTF("ddc_probe_device: read failed! return value=%d\n",
		    ret);
		goto out;
	}

	if (data[0] != DDC_DEFAULT_DEVICE_ADDR) {
		DPRINTF("ddc_get_vcp: invalid result code! data[0]=0x%x\n",
		    data[0]);
		ret = EIO;
		goto out;
	}

	if (data[2] != 0x02) {
		DPRINTF("ddc_get_vcp: invalid response! data[2]=0x%x\n",
		    data[2]);
		ret = EIO;
		goto out;
	}

	if (data[3] != 0x00) {
		DPRINTF("ddc_get_vcp: invalid response! data[3]=0x%x\n",
		    data[3]);
		ret = EIO;
		goto out;
	}

	if (data[4] != vcp_code) {
		DPRINTF("ddc_get_vcp: invalid response! data[4]=0x%x\n",
		    data[4]);
		ret = EIO;
		goto out;
	}

	len = 2 + 8 + 1;

	if (len > sizeof(data)) {
		DPRINTF("ddc_get_vcp: response too long! len=%d\n", len);
		ret = EIO;
		goto out;
	}

	if (ddc_checksum(data, len, DDC_HOST_ADDR_EVEN) != 0) {
		DPRINTF("ddc_get_vcp: checksum failed!\n");
		ret = EIO;
		goto out;
	}

	if (value != NULL)
		*value = ((uint16_t)data[8] << 8) | data[9];

	ret = 0;
out:
	rw_exit_read(&ddcs_lock);

	return ret;
}

int
ddc_set_vcp(struct device *dev, struct i2c_adapter *adapter, uint8_t vcp_code,
    uint16_t value)
{
	struct ddc_mapping *dm;
	struct i2c_msg msg;
	uint8_t cmd[7];
	int len, ret;

	rw_enter_read(&ddcs_lock);

	TAILQ_FOREACH(dm, &ddcs, dm_link) {
		if (dm->dm_dev == dev && dm->dm_adapter == adapter)
			break;
	}

	if (dm == NULL || dm->dm_dev == NULL || dm->dm_adapter == NULL ||
	    dm->dm_addr == 0) {
		DPRINTF(
		    "ddc_set_vcp: one or more of the members in ddc_mapping was NULL!");
		ret = ENOENT;
		goto out;
	}

	len = sizeof(cmd) - 1;

	cmd[0] = DDC_HOST_ADDR_ODD;
	cmd[1] = DDC_PFLAG | 4;
	cmd[2] = DDC_CMD_SETVCP;
	cmd[3] = vcp_code;
	cmd[4] = (value >> 8) & 0xFF;
	cmd[5] = value & 0xFF;
	cmd[6] = ddc_checksum(cmd, len, DDC_MONITOR_ADDR << 1);

	msg.addr = dm->dm_addr;
	msg.flags = 0;
	msg.buf = cmd;
	msg.len = sizeof(cmd);

	ret = i2c_transfer(dm->dm_adapter, &msg, 1);

	if (ret < 0) {
		DPRINTF("ddc_set_vcp: write failed! return value=%d\n",
		    ret);
		goto out;
	}

	ret = 0;
out:
	rw_exit_read(&ddcs_lock);

	return ret;
}

/*
 * Compute the DDC/CI checksum for the given command payload,
 * starting from a value derived from the monitor's bus address
 * Returns the checksum.
 */
uint8_t
ddc_checksum(uint8_t *cmd, unsigned int len, i2c_addr_t addr)
{
	unsigned int i, sum;
	sum = (unsigned char)(addr);

	for (i = 0; i < len; i++)
		sum ^= cmd[i];

	return (uint8_t)sum;
}

void
ddcattach(int nunits)
{
}

int
ddcopen(dev_t dev, int flags, int mode, struct proc *p)
{
	struct ddc_mapping *dm;

	rw_enter_write(&ddcs_lock);

	TAILQ_FOREACH(dm, &ddcs, dm_link) {
		if (dm->dm_index == minor(dev))
			break;
	}

	if (dm == NULL) {
		rw_exit_write(&ddcs_lock);
		return ENXIO;
	}

	rw_exit_write(&ddcs_lock);

	return 0;
}

int
ddcclose(dev_t dev, int flags, int mode, struct proc *p)
{
	return 0;
}

int
ddcioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	struct ddc_vcp_args *dva = (struct ddc_vcp_args *)data;
	struct ddc_mapping *dm;

	switch (cmd) {
	case DDCIOCPROBE:
		rw_enter_read(&ddcs_lock);

		dm = NULL;

		TAILQ_FOREACH(dm, &ddcs, dm_link) {
			if (minor(dev) == dm->dm_index)
				break;
		}

		rw_exit_read(&ddcs_lock);

		if (dm == NULL)
			return ENOENT;

		return ddc_probe_device(dm->dm_dev, dm->dm_adapter);

	case DDCIOCGETBRIGHTNESS:
		rw_enter_read(&ddcs_lock);

		dm = NULL;

		TAILQ_FOREACH(dm, &ddcs, dm_link) {
			if (minor(dev) == dm->dm_index)
				break;
		}

		rw_exit_read(&ddcs_lock);

		if (dm == NULL)
			return ENOENT;

		return ddc_get_vcp(dm->dm_dev, dm->dm_adapter, 0x10,
		    &dva->dva_value);

	case DDCIOCSETBRIGHTNESS:
		rw_enter_read(&ddcs_lock);

		dm = NULL;

		TAILQ_FOREACH(dm, &ddcs, dm_link) {
			if (minor(dev) == dm->dm_index)
				break;
		}

		rw_exit_read(&ddcs_lock);

		if (dm == NULL)
			return ENOENT;

		return ddc_set_vcp(dm->dm_dev, dm->dm_adapter, 0x10,
		    dva->dva_value);

	default:
		return ENOTTY;
	}
}
