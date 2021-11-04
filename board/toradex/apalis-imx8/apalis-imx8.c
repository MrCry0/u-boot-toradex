// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019 Toradex
 */

#include <common.h>
#include <cpu_func.h>
#include <init.h>

#include <asm/arch/clock.h>
#include <asm/arch/imx8-pins.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <env.h>
#include <errno.h>
#include <linux/libfdt.h>
#include <mmc.h>

#include <power-domain.h>
#include <usb.h>

#include "../common/tdx-cfg-block.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define PCB_VERS_DETECT	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | \
			 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
			 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | \
			 (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define PCB_VERS_DEFAULT	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | \
				 (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) | \
				 (SC_PAD_28FDSOI_PS_PD << PADRING_PULL_SHIFT) | \
				 (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT))

typedef enum {
	PCB_VERSION_1_0,
	PCB_VERSION_1_1
} pcb_rev_t;

static iomux_cfg_t pcb_vers_detect[] = {
	SC_P_MIPI_DSI0_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(PCB_VERS_DETECT),
	SC_P_MIPI_DSI0_GPIO0_01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(PCB_VERS_DETECT),
};

static iomux_cfg_t pcb_vers_default[] = {
	SC_P_MIPI_DSI0_GPIO0_00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(PCB_VERS_DEFAULT),
	SC_P_MIPI_DSI0_GPIO0_01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(PCB_VERS_DEFAULT),
};

static iomux_cfg_t uart1_pads[] = {
	SC_P_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	SC_P_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx8_iomux_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

void board_mem_get_layout(uint64_t *phys_sdram_1_start,
			  uint64_t *phys_sdram_1_size,
			  uint64_t *phys_sdram_2_start,
			  uint64_t *phys_sdram_2_size)
{
	uint32_t is_quadplus = 0, val = 0;
	sc_err_t sciErr = sc_misc_otp_fuse_read(-1, 6, &val);

	if (sciErr == SC_ERR_NONE) {
		/* QP has one A72 core disabled */
		is_quadplus = ((val >> 4) & 0x3) != 0x0;
	}

	*phys_sdram_1_start = PHYS_SDRAM_1;
	*phys_sdram_1_size = PHYS_SDRAM_1_SIZE;
	*phys_sdram_2_start = PHYS_SDRAM_2;
	if (is_quadplus)
		/* Our QP based SKUs only have 2 GB RAM (PHYS_SDRAM_1_SIZE) */
		*phys_sdram_2_size = 0x0UL;
	else
		*phys_sdram_2_size = PHYS_SDRAM_2_SIZE;
}

int board_early_init_f(void)
{
	sc_pm_clock_rate_t rate = SC_80MHZ;
	int ret;

	/**
	 *  Set UART0 clock root to 80 MHz and enable it
	 *  This is needed in order for UART1 to work
	 */
	ret = sc_pm_setup_uart(SC_R_UART_0, rate);
	if (ret)
		return ret;

	/* Set UART1 clock root to 80 MHz and enable it */
	ret = sc_pm_setup_uart(SC_R_UART_1, rate);
	if (ret)
		return ret;

	setup_iomux_uart();

	return 0;
}

#ifdef CONFIG_MXC_GPIO

#define BKL1_GPIO   IMX_GPIO_NR(1, 10)

static iomux_cfg_t board_gpios[] = {
	SC_P_LVDS1_GPIO00 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void board_gpio_init(void)
{
	imx8_iomux_setup_multiple_pads(board_gpios, ARRAY_SIZE(board_gpios));

	gpio_request(BKL1_GPIO, "BKL1_GPIO");
	gpio_direction_output(BKL1_GPIO, 1);
}
#endif

#if IS_ENABLED(CONFIG_FEC_MXC)
#include <miiphy.h>

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	return ft_common_board_setup(blob, bd);
}
#endif

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_dev();

	if (!check_mmc_autodetect())
		return;

	env_set_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

static pcb_rev_t get_pcb_revision(void)
{
	unsigned int pcb_vers = 0;

	imx8_iomux_setup_multiple_pads(pcb_vers_detect,
				       ARRAY_SIZE(pcb_vers_detect));

	gpio_request(IMX_GPIO_NR(1, 18), \
		     "PCB version detection on PAD SC_P_MIPI_DSI0_GPIO0_00");
	gpio_request(IMX_GPIO_NR(1, 19), \
		     "PCB version detection on PAD SC_P_MIPI_DSI0_GPIO0_01");
	gpio_direction_input(IMX_GPIO_NR(1, 18));
	gpio_direction_input(IMX_GPIO_NR(1, 19));

	udelay(1000);

	pcb_vers = gpio_get_value(IMX_GPIO_NR(1, 18));
	pcb_vers |= gpio_get_value(IMX_GPIO_NR(1, 19)) << 1;

	/* Set muxing back to default values for saving energy */
	imx8_iomux_setup_multiple_pads(pcb_vers_default,
				       ARRAY_SIZE(pcb_vers_default));

	switch(pcb_vers) {
		case 0b11:
			return PCB_VERSION_1_0;
			break;
		case 0b10:
			return PCB_VERSION_1_1;
			break;
		default:
			return -ENODEV;
			break;
	}
}

static void select_dt_from_module_version(void)
{
	env_set("soc", "imx8qm");
	env_set("variant", "-v1.1");

	switch (tdx_hw_tag.prodid) {
		/* Select Apalis iMX8QM device trees */
		case APALIS_IMX8QM_IT:
		case APALIS_IMX8QM_WIFI_BT_IT:
			if (get_pcb_revision() == PCB_VERSION_1_0)
				env_set("variant", "");
			break;

		/* Select Apalis iMX8QP device trees */
		case APALIS_IMX8QP_WIFI_BT:
		case APALIS_IMX8QP:
			env_set("soc", "imx8qp");
			break;
	default:
		printf("Unknown Apalis iMX8 module\n");
		return;
	}
}

static int do_select_dt_from_module_version(cmd_tbl_t *cmdtp, int flag, int argc,
		       char * const argv[]) {
	select_dt_from_module_version();
	return 0;
}

U_BOOT_CMD(
	select_dt_from_module_version, CONFIG_SYS_MAXARGS, 1, do_select_dt_from_module_version,
	"\n", "    - select devicetree from module version"
);

int board_init(void)
{
#ifdef CONFIG_MXC_GPIO
	board_gpio_init();
#endif

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
		"dma_lpuart1",
	};

	power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}
#endif

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(ulong addr)
{
	sc_pm_reboot(-1, SC_PM_RESET_TYPE_COLD);
	while(1);
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
/* TODO move to common */
	env_set("board_name", "Apalis iMX8QM");
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

#if defined(CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX) || defined(CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX)
	char *end_of_uboot;
	char command[256];
	end_of_uboot = (char *)(ulong)(CONFIG_SYS_TEXT_BASE + _end_ofs + fdt_totalsize(gd->fdt_blob));
	end_of_uboot += 9;

	/* load hdmitxfw.bin and hdmirxfw.bin*/
	memcpy((void *)IMX_HDMI_FIRMWARE_LOAD_ADDR, end_of_uboot,
			IMX_HDMITX_FIRMWARE_SIZE + IMX_HDMIRX_FIRMWARE_SIZE);

#ifdef CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX
	sprintf(command, "hdp load 0x%x", IMX_HDMI_FIRMWARE_LOAD_ADDR);
	run_command(command, 0);
#endif
#ifdef CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX
	sprintf(command, "hdprx load 0x%x",
			IMX_HDMI_FIRMWARE_LOAD_ADDR + IMX_HDMITX_FIRMWARE_SIZE);
	run_command(command, 0);
#endif
#endif /* CONFIG_IMX_LOAD_HDMI_FIMRWARE_RX || CONFIG_IMX_LOAD_HDMI_FIMRWARE_TX */

	select_dt_from_module_version();

	return 0;
}
