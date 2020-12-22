// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018-2019 Toradex
 */
#include <common.h>
#include <cpu_func.h>
#include <env.h>
#include <errno.h>
#include <init.h>
#include <linux/libfdt.h>
#include <fsl_esdhc_imx.h>
#include <fdt_support.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch-imx8/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/snvs_security_sc.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>

#include <power-domain.h>
#include <usb.h>

#include "../common/tdx-cfg-block.h"

DECLARE_GLOBAL_DATA_PTR;

#define ESDHC_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define ESDHC_CLK_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define USB_CDET_GPIO	IMX_GPIO_NR(5, 9)

static iomux_cfg_t uart3_pads[] = {
	SC_P_FLEXCAN2_RX | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_FLEXCAN2_TX | MUX_MODE_ALT(2) | MUX_PAD_CTRL(UART_PAD_CTRL),
	/* Transceiver FORCEOFF# signal, mux to use pullup */
	SC_P_QSPI0B_DQS | MUX_MODE_ALT(4) | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx8_iomux_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
}

int board_ci_udc_phy_mode(void *__iomem phy_base, int phy_off)
{
	switch ((phys_addr_t)phy_base) {
	case 0x5b0d0000:
		if (gpio_get_value(USB_CDET_GPIO))
			return USB_INIT_DEVICE;
		else
			return USB_INIT_HOST;
	case 0x5b110000:
	default:
		return USB_INIT_HOST;
	}
}

void board_mem_get_layout(uint64_t *phys_sdram_1_start,
			  uint64_t *phys_sdram_1_size,
			  uint64_t *phys_sdram_2_start,
			  uint64_t *phys_sdram_2_size)
{
	uint32_t is_dualx = 0, val = 0;
	sc_err_t sciErr = sc_misc_otp_fuse_read(-1, 6, &val);

	if (sciErr == SC_ERR_NONE) {
		/* DX has two A35 cores disabled */
		is_dualx = (val & 0xf) != 0x0;
	}

	*phys_sdram_1_start = PHYS_SDRAM_1;
	if (is_dualx)
		/* Our DX based SKUs only have 1 GB RAM */
		*phys_sdram_1_size = SZ_1G;
	else
		*phys_sdram_1_size = PHYS_SDRAM_1_SIZE;
	*phys_sdram_2_start = PHYS_SDRAM_2;
	*phys_sdram_2_size = PHYS_SDRAM_2_SIZE;
}

int board_early_init_f(void)
{
	sc_pm_clock_rate_t rate = SC_80MHZ;
	int ret;

	/*
	 * This works around that having only UART3 up the baudrate is 1.2M
	 * instead of 115.2k. Set UART0 clock root to 80 MHz and enable it
	 */
	ret = sc_pm_setup_uart(SC_R_UART_0, rate);
	if (ret)
		return ret;

	/* Set UART0 clock root to 80 MHz and enable it */
	ret = sc_pm_setup_uart(SC_R_UART_3, rate);
	if (ret)
		return ret;

	setup_iomux_uart();

	return 0;
}


#ifdef CONFIG_FEC_MXC
#include <miiphy.h>

int board_phy_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#undef CONFIG_MXC_GPIO /* TODO */
#ifdef CONFIG_MXC_GPIO
#define IOEXP_RESET IMX_GPIO_NR(1, 1)

static iomux_cfg_t board_gpios[] = {
	SC_P_SPI2_SDO | MUX_MODE_ALT(4) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	SC_P_ENET0_REFCLK_125M_25M | MUX_MODE_ALT(4) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void board_gpio_init(void)
{
	int ret;
	struct gpio_desc desc;

	ret = dm_gpio_lookup_name("gpio@1a_3", &desc);
	if (ret)
		return;

	ret = dm_gpio_request(&desc, "bb_per_rst_b");
	if (ret)
		return;

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&desc, 0);
	udelay(50);
	dm_gpio_set_value(&desc, 1);

	imx8_iomux_setup_multiple_pads(board_gpios, ARRAY_SIZE(board_gpios));

	/* enable i2c port expander assert reset line */
	gpio_request(IOEXP_RESET, "ioexp_rst");
	gpio_direction_output(IOEXP_RESET, 1);
}
#endif

int board_init(void)
{
#ifdef CONFIG_MXC_GPIO
	board_gpio_init();
#endif
	gpio_request(USB_CDET_GPIO, "usb_cdet");

#ifdef CONFIG_SNVS_SEC_SC_AUTO
	{
		int ret = snvs_security_sc_init();

		if (ret)
			return ret;
	}
#endif

	return 0;
}

/* todo: With that function in ther is no console output in linux, drop for now */
#if 0
void board_quiesce_devices(void)
{
	const char *power_on_devices[] = {
		"dma_lpuart3",

		/* HIFI DSP boot */
		"audio_sai0",
		"audio_ocram",
	};

	power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}
#endif

void detail_board_ddr_info(void)
{
	puts("\nDDR    ");
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(ulong addr)
{
	sc_pm_reboot(-1, SC_PM_RESET_TYPE_COLD);
	while(1);

}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return ft_common_board_setup(blob, bd);
}
#endif
void board_late_mmc_env_init() {}
int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
/* TODO move to common */
	env_set("board_name", "Colibri iMX8QXP");
	env_set("board_rev", "v1.0");
#endif

	build_info();

#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#else
	env_set("sec_boot", "no");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/
