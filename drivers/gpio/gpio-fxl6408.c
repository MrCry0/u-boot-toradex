// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2021 Toradex
 *  Copyright (C) 2016 Broadcom
 */

/**
 * DOC: FXL6408 I2C to GPIO expander.
 *
 * This chip has 8 GPIO lines out of it, and is controlled by an I2C
 * bus (a pair of lines), providing 4x expansion of GPIO lines. It
 * also provides an interrupt line out for notifying of state changes.
 *
 * Any preconfigured state will be left in place until the GPIO lines
 * get activated. At power on, everything is treated as an input,
 * default input is HIGH and pulled-up, all interrupts are masked.
 *
 * Documentation can be found at:
 * https://www.fairchildsemi.com/datasheets/FX/FXL6408.pdf
 *
 * This driver bases on:
 * - the original driver by Eric Anholt <eric@anholt.net>:
 *   https://patchwork.kernel.org/patch/9148419/
 * - the Toradex version by Max Krummenacher <max.krummenacher@toradex.com>:
 *   http://git.toradex.com/cgit/linux-toradex.git/tree/drivers/gpio/gpio-fxl6408.c?h=toradex_5.4-2.3.x-imx
 * - the U-boot PCA953x driver by Peng Fan <van.freenix@gmail.com>:
 *   drivers/gpio/pca953x_gpio.c
 *
 * TODO: Add interrupts support
 */

#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <i2c.h>
#include <log.h>
#include <asm-generic/gpio.h>
#include <asm/global_data.h>
#include <linux/bitops.h>
#include <dt-bindings/gpio/gpio.h>

#define REG_DEVID_CTRL		0x01
# define SW_RST			BIT(0)
# define RST_INT		BIT(1)
/* 0b101 is the Manufacturer's ID assigned to Fairchild by Nokia */
# define MF_ID_FAIRCHILD	5

/* Bits set here indicate that the GPIO is an output */
#define REG_IO_DIR		0x03

/*
 * Bits set here, when the corresponding bit of REG_IO_DIR is set, drive
 * the output high instead of low.
 */
#define REG_OUT_STATE		0x05

/* Bits here make the output High-Z, instead of the OUTPUT value */
#define REG_OUT_HIGH_Z		0x07

/*
 * Bits here define the expected input state of the GPIO.
 * INTERRUPT_STATUS bits will be set when the INPUT transitions away
 * from this value.
 */
#define REG_IN_DEFAULT_STATE	0x09

/*
 * Bits here enable either pull up or pull down according to
 * REG_PULL_MODE.
 */
#define REG_PULL_ENABLE		0x0b

/*
 * Bits set here selects a pull-up/pull-down state of pin, which
 * is configured as Input and the corresponding REG_PULL_ENABLE bit is
 * set.
 */
#define REG_PULL_MODE		0x0d

/* Returns the current status (1 = HIGH) of the input pins */
#define REG_IN_STATUS		0x0f

/* Mask of pins which can generate interrupts */
#define REG_INT_MASK		0x11

/* Mask of pins which have generated an interrupt. Cleared on read */
#define REG_INT_STATUS		0x13

/* Manufacturer's ID getting from Device ID & Ctrl register */
enum {
	MF_ID_MASK = GENMASK(7, 5),
	MF_ID_SHIFT = 5,
};

/* Firmware revision getting from Device ID & Ctrl register */
enum {
	FW_REV_MASK = GENMASK(4, 2),
	FW_REV_SHIFT = 2,
};

enum io_direction {
	DIR_IN,
	DIR_OUT,
};

/*
 * struct fxl6408_info - Data for fxl6408
 *
 * @dev: udevice structure for the device
 * @addr: i2c slave address
 * @reg_io_dir: hold the value of direction register
 * @reg_output: hold the value of output register
 */
struct fxl6408_info {
	struct udevice *dev;
	int addr;
	u8 device_id;
	u8 reg_io_dir;
	u8 reg_output;
};

static inline int fxl6408_write(struct udevice *dev, int reg, u8 val)
{
	return dm_i2c_write(dev, reg, &val, 1);
}

static int fxl6408_read(struct udevice *dev, int reg)
{
	int ret;
	u8 tmp;

	ret = dm_i2c_read(dev, reg, &tmp, 1);
	if (!ret)
		ret = tmp;

	return ret;
}

/*
 * fxl6408_is_output() - check whether the gpio configures as either
 *			 output or input.
 * Return: false - input, true - output.
 */
static bool fxl6408_is_output(struct udevice *dev, int offset)
{
	struct fxl6408_info *info = dev_get_plat(dev);

	return info->reg_io_dir & BIT(offset);
}

static int fxl6408_get_value(struct udevice *dev, uint offset)
{
	int ret, reg = fxl6408_is_output(dev, offset) ? REG_OUT_STATE : REG_IN_STATUS;

	ret = fxl6408_read(dev, reg);
	if (ret < 0)
		return ret;

	return !!(ret & BIT(offset));
}

static int fxl6408_set_value(struct udevice *dev, uint offset, int value)
{
	struct fxl6408_info *info = dev_get_plat(dev);
	u8 val;
	int ret;

	if (value)
		val = info->reg_output | BIT(offset);
	else
		val = info->reg_output & ~BIT(offset);

	ret = fxl6408_write(dev, REG_OUT_STATE, val);
	if (ret < 0)
		return ret;

	info->reg_output = val;

	return 0;
}

static int fxl6408_set_direction(struct udevice *dev, uint offset,
				 enum io_direction dir)
{
	struct fxl6408_info *info = dev_get_plat(dev);
	u8 val;
	int ret;

	if (dir == DIR_IN)
		val = info->reg_io_dir & ~BIT(offset);
	else
		val = info->reg_io_dir | BIT(offset);

	ret = fxl6408_write(dev, REG_IO_DIR, val);
	if (ret < 0)
		return ret;

	info->reg_io_dir = val;

	return 0;
}

static int fxl6408_direction_input(struct udevice *dev, uint offset)
{
	return fxl6408_set_direction(dev, offset, DIR_IN);
}

static int fxl6408_direction_output(struct udevice *dev, uint offset, int value)
{
	int ret;

	/* Configure output value */
	ret = fxl6408_set_value(dev, offset, value);
	if (ret < 0)
		return ret;
	/* Configure direction as output */
	fxl6408_set_direction(dev, offset, DIR_OUT);

	return 0;
}

static int fxl6408_get_function(struct udevice *dev, uint offset)
{
	if (fxl6408_is_output(dev, offset))
		return GPIOF_OUTPUT;
	else
		return GPIOF_INPUT;
}

static int fxl6408_xlate(struct udevice *dev, struct gpio_desc *desc,
			 struct ofnode_phandle_args *args)
{
	desc->offset = args->args[0];
	desc->flags = args->args[1] & GPIO_ACTIVE_LOW ? GPIOD_ACTIVE_LOW : 0;

	return 0;
}

static const struct dm_gpio_ops fxl6408_ops = {
	.direction_input        = fxl6408_direction_input,
	.direction_output       = fxl6408_direction_output,
	.get_value              = fxl6408_get_value,
	.set_value              = fxl6408_set_value,
	.get_function           = fxl6408_get_function,
	.xlate                  = fxl6408_xlate,
};

static int fxl6408_probe(struct udevice *dev)
{
	struct fxl6408_info *info = dev_get_plat(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	char name[32], label[32], *str;
	int addr;
	int ret;
	int size;
	const u8 *tmp;
	u32 val32;

	addr = dev_read_addr(dev);
	if (addr == 0)
		return -EINVAL;
	info->addr = addr;

	/*
	 * Check the device ID register to see if it's responding.
	 * This also clears RST_INT as a side effect, so we won't get
	 * the "we've been power cycled" interrupt once interrupts
	 * being enabled.
	 */
	ret = fxl6408_read(dev, REG_DEVID_CTRL);
	if (ret < 0) {
		dev_err(dev, "FXL6408 probe returned %d\n", ret);
		return ret;
	}

	if ((ret & MF_ID_MASK) >> MF_ID_SHIFT != MF_ID_FAIRCHILD) {
		dev_err(dev, "FXL6408 probe: wrong Manufacturer's ID: 0x%02x\n", ret);
		return -ENXIO;
	}
	info->device_id = ret;

	/*
	 * Disable High-Z of outputs, so that the OUTPUT updates
	 * actually take effect.
	 */
	ret = fxl6408_write(dev, REG_OUT_HIGH_Z, (u8)0);
	if (ret < 0) {
		dev_err(dev, "Error writing High-Z register\n");
		return ret;
	}

	/*
	 * If configured, set initial output state and direction,
	 * otherwise read them from the chip.
	 */
	if (dev_read_u32(dev, "initial_io_dir", &val32)) {
		ret = fxl6408_read(dev, REG_IO_DIR);
		if (ret < 0) {
			dev_err(dev, "Error reading direction register\n");
			return ret;
		}
		info->reg_io_dir = ret;
	} else {
		info->reg_io_dir = val32 & 0xFF;
		ret = fxl6408_write(dev, REG_IO_DIR, info->reg_io_dir);
		if (ret < 0) {
			dev_err(dev, "Error setting direction register\n");
			return ret;
		}
	}

	if (dev_read_u32(dev, "initial_output", &val32)) {
		ret = fxl6408_read(dev, REG_OUT_STATE);
		if (ret < 0) {
			dev_err(dev, "Error reading output register\n");
			return ret;
		}
		info->reg_output = ret;
	} else {
		info->reg_output = val32 & 0xFF;
		ret = fxl6408_write(dev, REG_OUT_STATE, info->reg_output);
		if (ret < 0) {
			dev_err(dev, "Error setting output register\n");
			return ret;
		}
	}

	tmp = dev_read_prop(dev, "label", &size);
	if (tmp) {
		size = min(size, (int)sizeof(label) - 1);
		memcpy(label, tmp, size);
		label[size] = '\0';
		snprintf(name, sizeof(name), "%s@%x_", label, info->addr);
	} else {
		snprintf(name, sizeof(name), "gpio@%x_", info->addr);
	}

	str = strdup(name);
	if (!str)
		return -ENOMEM;

	uc_priv->bank_name = str;
	uc_priv->gpio_count = dev_get_driver_data(dev);
	uc_priv->gpio_base = -1;

	dev_dbg(dev, "%s (FW rev. %ld) is ready\n", str,
		(info->device_id & MF_ID_MASK) >> MF_ID_SHIFT);

	return 0;
}

static const struct udevice_id fxl6408_ids[] = {
	{ .compatible = "fcs,fxl6408", .data = 8 },
	{ }
};

U_BOOT_DRIVER(fxl6408_gpio) = {
	.name = "fxl6408_gpio",
	.id = UCLASS_GPIO,
	.ops = &fxl6408_ops,
	.probe = fxl6408_probe,
	.of_match = fxl6408_ids,
	.plat_auto = sizeof(struct fxl6408_info),
};
