/*
 *  EASYLINK EL-M300 board support
 *
 *  Copyright (C) 2017 huangfc <huangfangcheng@163.com>
 *  Copyright (C) 2017 L. D. Pinney <ldpinney@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/ar71xx_regs.h>

#include "common.h"
#include "dev-eth.h"
#include "dev-ap9x-pci.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-spi.h"
#include "dev-usb.h"
#include "dev-wmac.h"
#include "machtypes.h"
#include "pci.h"

#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/concat.h>


#define EL_M300_GPIO_LED_STATUS	13

#define EL_M300_GPIO_LED_WAN		2
#define EL_M300_GPIO_LED_LAN1		3
#define EL_M300_GPIO_LED_LAN2		4

#define EL_M300_GPIO_BTN_RESET		12

#define EL_M300_KEYS_POLL_INTERVAL	20	/* msecs */
#define EL_M300_KEYS_DEBOUNCE_INTERVAL	(3 * EL_M300_KEYS_POLL_INTERVAL)

static const char *el_m300_part_probes[] = {
	"tp-link",
	NULL,
};

static struct flash_platform_data el_m300_flash_data = {
	.part_probes	= el_m300_part_probes,
};

static struct spi_board_info ath79_spi_info[] = {
	{

		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 25000000,
		.modalias	= "m25p80",

	}
};

static struct ath79_spi_platform_data ath79_spi_data;

void __init ath79_register_el_m300_m25p80(struct flash_platform_data *pdata)
{
	ath79_spi_data.bus_num = 0;
	ath79_spi_data.num_chipselect = 3;
	ath79_spi_info[0].platform_data = pdata;
	ath79_register_spi(&ath79_spi_data, ath79_spi_info, 1);
}

static struct gpio_led el_m300_leds_gpio[] __initdata = {
	{

		.name		= "easylink:green:system",
		.gpio		= EL_M300_GPIO_LED_STATUS,
		.active_low	= 1,
	}
};

static struct gpio_keys_button el_m300_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = EL_M300_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= EL_M300_GPIO_BTN_RESET,
		.active_low	= 1,
	},
};

static void __init el_m300_setup(void)
{
	u8 *mac = (u8 *) KSEG1ADDR(0x1f01fc00);
	u8 *ee = (u8 *) KSEG1ADDR(0x1fff1000);
	u8 tmpmac[ETH_ALEN];

	/* disable PHY_SWAP and PHY_ADDR_SWAP bits */

	ath79_register_el_m300_m25p80(&el_m300_flash_data);
	ath79_gpio_direction_select(EL_M300_GPIO_LED_STATUS, true);
	ath79_gpio_direction_select(EL_M300_GPIO_LED_WAN, true);
	ath79_gpio_direction_select(EL_M300_GPIO_LED_LAN1, true);
	ath79_gpio_direction_select(EL_M300_GPIO_LED_LAN2, true);


	ath79_gpio_output_select(EL_M300_GPIO_LED_WAN,
			QCA953X_GPIO_OUT_MUX_LED_LINK5);
	ath79_gpio_output_select(EL_M300_GPIO_LED_LAN1,
			QCA953X_GPIO_OUT_MUX_LED_LINK3);
	ath79_gpio_output_select(EL_M300_GPIO_LED_LAN2,
			QCA953X_GPIO_OUT_MUX_LED_LINK1);

	ath79_register_leds_gpio(-1, ARRAY_SIZE(el_m300_leds_gpio),
			el_m300_leds_gpio);
	ath79_register_gpio_keys_polled(-1, EL_M300_KEYS_POLL_INTERVAL,
			ARRAY_SIZE(el_m300_gpio_keys),
			el_m300_gpio_keys);

	ath79_register_usb();
	ath79_register_pci();

	ath79_setup_ar933x_phy4_switch(false, false);

	ath79_register_mdio(0, 0x0);

	/* LAN */
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_GMII;
	ath79_eth1_data.duplex = DUPLEX_FULL;
	ath79_switch_data.phy_poll_mask |= BIT(4);
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);
	ath79_register_eth(1);

	/* WAN port */
	ath79_switch_data.phy4_mii_en = 1;
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_MII;
	ath79_eth0_data.duplex = DUPLEX_FULL;
	ath79_eth0_data.speed = SPEED_100;
	ath79_eth0_data.phy_mask = BIT(4);
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 1);
	ath79_register_eth(0);

	ath79_init_mac(tmpmac, mac, 0);
	ath79_register_wmac(ee,tmpmac);
}

MIPS_MACHINE(ATH79_MACH_EL_M300, "EL-M300", "EasyLink EL-M300",
	     el_m300_setup);
