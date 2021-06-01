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

#define FXL6408_DEVID_CTRL		0x01
# define FXL6408_SW_RST			BIT(0)
# define FXL6408_RST_INT		BIT(1)

/* 3-bit manufacturer ID */
# define FXL6408_MF_MASK		GENMASK(7, 5)
# define FXL6408_MF_ID(devid)		((devid & FXL6408_MF_MASK) >> 5)
/* 0b101 is for Fairchild assigned by Nokia */
# define FXL6408_FAIRCHILD_MF		5

/* 3-bit firmware revision */
# define FXL6408_FW_MASK		GENMASK(4, 2)
# define FXL6408_FW_REV(devid)		((devid & FXL6408_FW_MASK) >> 2)

/*
 * Bits set here indicate that the GPIO is an output.
 */
#define FXL6408_DIRECTION	0x03

enum {
	FXL6408_DIRECTION_IN,
	FXL6408_DIRECTION_OUT,
};

/*
 * Bits set here, when the corresponding bit of IO_DIR is set, drive
 * the output high instead of low.
 */
#define FXL6408_OUTPUT		0x05

/*
 * Bits here make the output High-Z, instead of the OUTPUT value.
 */
#define FXL6408_OUTPUT_HIGH_Z	0x07

/*
 * Bits here define the expected input state of the GPIO.
 * INTERRUPT_STATUS bits will be set when the INPUT transitions away
 * from this value.
 */
#define FXL6408_INPUT_DEF_STATE	0x09

/*
 * Bits here enable either pull up or pull down according to
 * INPUT_PULL_STATE.
 */
#define FXL6408_INPUT_PULL_ENABLE	0x0b

/*
 * Bits set here selects a pull-up/pull-down state of pin, which
 * is configured as Input and the corresponding PULL_ENABLE bit is
 * set.
 */
#define FXL6408_INPUT_PULL_STATE	0x0d

/*
 * Returns the current status (1 = HIGH) of the input pins.
 */
#define FXL6408_INPUT		0x0f

/*
 * Mask of pins which can generate interrupts.
 */
#define FXL6408_INTERRUPT_MASK	0x11
/*
 * Mask of pins which have generated an interrupt.
 * Cleared on read.
 */
#define FXL6408_INTERRUPT_STATUS	0x13

/*
 * struct fxl6408_info - Data for fxl6408
 *
 * @dev: udevice structure for the device
 * @addr: i2c slave address
 * @reg_output: hold the value of output register
 * @reg_direction: hold the value of direction register
 */
struct fxl6408_info {
	struct udevice *dev;
	int addr;
	u8 device_id;
	u8 reg_io_dir;
	u8 reg_output;
};

static int fxl6408_write(struct udevice *dev, int reg, u8 val)
{
	int ret;

	ret = dm_i2c_write(dev, reg, &val, 1);
	if (ret)
		dev_err(dev, "%s error: %d\n", __func__, ret);

	return ret;
}

static int fxl6408_read(struct udevice *dev, int reg, u8 *val)
{
	int ret;
	u8 tmp;

	ret = dm_i2c_read(dev, reg, &tmp, 1);
	if (!ret)
		*val = tmp;
	else
		dev_err(dev, "%s error: %d\n", __func__, ret);

	return ret;
}

/*
 * fxl6408_is_output() - check whether the gpio configures as either
 *			 output or input.
 * Return: 0 - input, 1 - output.
 */
static int fxl6408_is_output(struct udevice *dev, int offset)
{
	struct fxl6408_info *info = dev_get_plat(dev);

	return (info->reg_io_dir & BIT(offset));
}

static int fxl6408_get_value(struct udevice *dev, uint offset)
{
	int ret;
	u8 val = 0;

	ret = fxl6408_read(dev, FXL6408_INPUT, &val);
	if (IS_ERR_VALUE(ret))
		return ret;

	return (val >> offset) & 0x1;
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

	ret = fxl6408_write(dev, FXL6408_OUTPUT, val);
	if (!IS_ERR_VALUE(ret))
		info->reg_output = val;

	return ret;
}

static int fxl6408_set_direction(struct udevice *dev, uint offset, int dir)
{
	struct fxl6408_info *info = dev_get_plat(dev);
	u8 val;
	int ret;

	if (dir == FXL6408_DIRECTION_IN)
		val = info->reg_io_dir | BIT(offset);
	else
		val = info->reg_io_dir & ~BIT(offset);

	ret = fxl6408_write(dev, FXL6408_DIRECTION, val);
	if (!IS_ERR_VALUE(ret))
		info->reg_io_dir = val;

	return ret;
}

static int fxl6408_direction_input(struct udevice *dev, uint offset)
{
	return fxl6408_set_direction(dev, offset, FXL6408_DIRECTION_IN);
}

static int fxl6408_direction_output(struct udevice *dev, uint offset, int value)
{
	int ret;

	/* Configure output value. */
	ret = fxl6408_set_value(dev, offset, value);
	if (!IS_ERR_VALUE(ret))
		/* Configure direction as output. */
		fxl6408_set_direction(dev, offset, FXL6408_DIRECTION_OUT);

	return ret;
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
	u8 val;
	u32 val32;

	dev_dbg(dev, "%s: Enter\n", __func__);
	addr = dev_read_addr(dev);
	if (addr == 0)
		return -ENODEV;
	info->addr = addr;

	/* Check the device ID register to see if it's responding.
	 * This also clears RST_INT as a side effect, so we won't get
	 * the "we've been power cycled" interrupt once we enable
	 * interrupts.
	 */
	ret = fxl6408_read(dev, FXL6408_DEVID_CTRL, &val);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "FXL6408 probe returned %d\n", ret);
		return ret;
	}

	if (FXL6408_MF_ID(val) != FXL6408_FAIRCHILD_MF) {
		dev_err(dev, "FXL6408 probe returned DID: 0x%02x\n", val);
		return -ENODEV;
	}
	info->device_id = val;

	/*
	 * Disable High-Z of outputs, so that the OUTPUT updates
	 * actually take effect.
	 */
	ret = fxl6408_write(dev, FXL6408_OUTPUT_HIGH_Z, (u8) 0);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Error writing High-Z register\n");
		return ret;
	}

	/*
	 * If configured, set initial output state and direction,
	 * otherwise read them from the chip.
	 */
	if (IS_ERR_VALUE(dev_read_u32(dev, "inital_io_dir", &val32))) {
		ret = fxl6408_read(dev, FXL6408_DIRECTION, &info->reg_io_dir);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Error reading direction register\n");
			return ret;
		}
	} else {
		info->reg_io_dir = val32 & 0xFF;
		ret = fxl6408_write(dev, FXL6408_DIRECTION, info->reg_io_dir);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Error setting direction register\n");
			return ret;
		}
	}

	if (IS_ERR_VALUE(dev_read_u32(dev, "inital_output", &val32))) {
		ret = fxl6408_read(dev, FXL6408_OUTPUT, &info->reg_output);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Error reading output register\n");
			return ret;
		}
	} else {
		info->reg_output = val32 & 0xFF;
		ret = fxl6408_write(dev, FXL6408_OUTPUT, info->reg_output);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Error setting output register\n");
			return ret;
		}
	}

	tmp = dev_read_prop(dev, "label", &size);
	if (tmp) {
		size = min(size, (int) sizeof(label) - 1);
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
		FXL6408_FW_REV(info->device_id));

	dev_info(dev, "%s (FW rev. %ld) is ready\n", str,
		FXL6408_FW_REV(info->device_id));

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
