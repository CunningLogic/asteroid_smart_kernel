/*
 * linux/arch/arm/mach-omap2/board-fc6100.c
 *
 * Copyright (C) 2012 Parrot SA
 * Christian ROSALIE <christian.rosalie@parrot.com>
 *
 * Extract from linux/arch/arm/mach-omap2/board-fc6100.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <plat/mux.h>
#include <mach/board-fc6100.h>
#include <../drivers/parrot/usb/otg/dummy-smsc-usb43340.h>
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>

#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#include "parrot-common.h"
#include "board-fc6100.h"
#include "omap3-opp.h"
#include "smartreflex-class3.h"
#include "hsmmc.h"

#include <linux/mmc/host.h>
#include <linux/i2c/rtc-pca8565.h>

#include <linux/serial_max3100.h>
#include "board-fc6100-camera.h"


#if ( defined(CONFIG_OMAP2_DSS_VENC) \
      && (  (defined(CONFIG_REGULATOR)  && !defined(CONFIG_REGULATOR_TPS65023))  \
      ||  (!defined(CONFIG_REGULATOR)  && defined(CONFIG_REGULATOR_TPS65023)) )  )
#error Set 'Voltage and Current Regulator Support' and 'TI TPS65023 Power regulators' to make the system boots
#endif

#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE) \
     || defined(CONFIG_SND_SOC_FC6100_I2S_CODECS) || defined(CONFIG_SND_SOC_FC6100_I2S_CODECS_MODULE)
#error Do not use anymore WAU8822 I2S codec and Nuvoton NAU8820 and other I2S slave codecs use PALCODEC for init
#endif

/* Use to force the settings for production test */
static int parrot_force_usb_device;

static int __init parrot_force_usbd(char *str)
{
	parrot_force_usb_device = 1;
	return 1;
}
__setup("parrot_force_usbd", parrot_force_usbd);

/* Use to set composite tv instead of s-video */
static int parrot_force_tv_composite;


int __init parrot_force_composite(char *str)
{
	parrot_force_tv_composite = 1;
	return 1;
}

__setup("parrot_force_composite", parrot_force_composite);

/* Board revision (module revision + mother board) */
int board_rev = 0;

/* Board settings ( for submodules initialisation ) */
unsigned int board_config = 0;

#define GPIO_BOARD_REV_NBR_BIT	(10)

#define PCB_VERSION_BIT_0	(34)
#define PCB_VERSION_BIT_1	(35)
#define PCB_VERSION_BIT_2	(36)
#define PCB_VERSION_BIT_3	(37)

#define DDR_MANUFCTURER_BIT_4	(38)
#define DDR_SIZE_BIT_5		(39)
#define NAND_SIZE_BIT_6		(40)
#define RTC_BT_ANTENNA_BIT_7	(41)

#define USB_PHY_0_BIT_8		(42)
#define RESERVED_BIT_9		(43)

#define MOTHERBOARD_ID_BIT0 	(99)
#define MOTHERBOARD_ID_BIT7	(105)


static void fc6100_mod_print_board_rev(void)
{
	int i;

	// CUS package is used for OMAP3630 FC6100
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);

	// Get revision of FC6100 module
	board_rev = 0;

	for (i = 0; i < GPIO_BOARD_REV_NBR_BIT; i++) {

		parrot_gpio_in_init(PCB_VERSION_BIT_0 + i, 0);

		if (gpio_get_value(PCB_VERSION_BIT_0 + i)) {
			board_rev |= (1 << i);
		}
	}

	//Disable camera
	parrot_gpio_out_init(FC_CAM_EN_N, 1);

	// Get revision of board
	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW02) {

		for (i = MOTHERBOARD_ID_BIT0; i <= MOTHERBOARD_ID_BIT7; i++) {

			parrot_gpio_in_init(i, 0);

			//The bits from 10 to 15 are used for the mother board ID
			if (gpio_get_value(i))
				board_rev |= (1 << (i - MOTHERBOARD_ID_BIT0
					+ GPIO_BOARD_REV_NBR_BIT ) );
		}
	}

	//Save the board revision
	system_rev = board_rev;

	printk(KERN_INFO "FC6100 module revision : 0x%x\n", fc6100_board_rev());
	printk(KERN_INFO "FC6100 pcb revision : 0x%x\n", fc6100_mod_get_pcb_revision());
	printk(KERN_INFO "FC6100 moth revision : 0x%x\n", fc6100_mod_get_motherboard_revision());
	printk(KERN_INFO "DDR MANUFACTURER : %d\t\\0 (Micron) / 1 (Other)\n", (board_rev & (1<<4))>>4);

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW07) {
		/* USB PHY 1 : 0 (not used) / 1 (assembled) */
		printk(KERN_INFO "USB 0 PHY : %d\t\\0 (not used) / 1 (assembled)\n", (board_rev & (1<<8))>>8);
	}

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW05) {
		/* 	VERSION_BIT_5	DDR Size bit 0
			DDR Size : 0 (1Gbbits / 128MB) / 1 (2Gbits / 256MB) */
		printk(KERN_INFO "DDR SIZE : %d MB\n", (board_rev & (1<<5))? 256 : 128);

		/*	VERSION_BIT_6	DDR Size bit 1 (MSB)
			Flash Size : 0 (2Gbits / 256MB) / 1 (4Gbits / 512MB) */
		printk(KERN_INFO "NAND SIZE : %d MB\n", (board_rev & (1<<6))? 512 : 256);

	}else if (fc6100_mod_get_pcb_revision() < FC6100_0_HW07) {
		/*	VERSION_BIT_5	DDR Size bit 0
			DDR Size : 0 (2Gbits / 256MB) / 1 (4Gbits / 512MB) */
		printk(KERN_INFO "DDR SIZE : %d MB\n", (board_rev & (1<<5))? 512 : 256);

		/*	VERSION_BIT_6	DDR Size bit 1 (MSB)
			Flash Size : 0 (4Gbits / 512MB) / 1 (8Gbits / 1024MB) */
		printk(KERN_INFO "NAND SIZE : %d MB\n", (board_rev & (1<<6))? 1024 : 512);
	}else{
		int memory_size[] = { 64, 128, 256, 512, 1024};

		/*	VERSION_BIT_5	DDR Size bit 0 (LSB)
			VERSION_BIT_6	DDR Size bit 1 (MSB)
			DDR Size :	0x0 (512Mbits / 64MB) / 0x1 (1Gbbits / 128MB)
					0x2 (2Gbits / 256MB) / 0x3 (4Gbits / 512MB)
		*/
		int temp = (board_rev & ((1<<5) | (1<<6)) ) >> 5;
		printk(KERN_INFO "DDR SIZE : %d MB\n", memory_size[temp] );

		/*	VERSION_BIT_8	Flash Size bit 0 (LSBits)
			VERSION_BIT_9	Flash size bit 1 (MSBits)
			Flash Size :	0x0 (1Gbbits / 128MB) / 0x1 (2Gbits / 256MB)
					0x2 (4Gbits / 512MB)  / 0x3 (8Gbits / 1024MB)
		*/
		temp =  (board_rev & ((1<<8) | (1<<9) )) >> 8;
		printk(KERN_INFO "NAND SIZE : %d MB\n", memory_size[temp + 1]);

	}

	/* PROTO HW02 and followings */
	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03)
		printk(KERN_INFO "BT Antenna : %d\t\\0 (not used) / 1 (detected)\n", (board_rev & (1<<7))>>7);
	else
		printk(KERN_INFO "RTC : %d\t\\0 (not used) / 1 (assembled)\n", (board_rev & (1<<7))>>7);


	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW06) {
		parrot_gpio_user_in_init(48, OMAP_PIN_INPUT_PULLUP, "uart-dbg");
		if (gpio_get_value(48) == 0) {
			printk(KERN_INFO "dbg version\n");
			console_loglevel = 15;
		}
	}
	return;
}


/* 
 * UARTs 
 *
 * UART 1  for Console trace - AT
 * UART 3  BT 2.1 from Marvell 88w8688 (inside the module)
 */
static void __init fc6100_mod_uart_mux_init(int use_rts_cts)
{

	/* UART 1 for Console trace - AT*/
	omap_mux_init_signal("uart1_tx.uart1_tx",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart1_rx.uart1_rx",
			OMAP_PIN_INPUT);

	if (use_rts_cts) {
		omap_mux_init_signal("uart1_rts.uart1_rts", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("uart1_cts.uart1_cts", OMAP_PIN_INPUT);
	}

	/* UART 3 BT 2.1 from Marvell 88w8688 */
	omap_mux_init_signal("uart3_cts_rctx.uart3_cts_rctx",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("uart3_rts_sd.uart3_rts_sd",
			OMAP_PIN_OUTPUT);
	omap_mux_init_signal("uart3_rx_irrx.uart3_rx_irrx",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("uart3_tx_irtx.uart3_tx_irtx",
			OMAP_PIN_OUTPUT);

	parrot_omap_serial_init();
}


/* I2C Settings/Data
 *
 * i2c_1 does not go out of the module
 *	- PMIC	tps65023
 *	- RTC	pca8565
 *
 * i2c_2 can be used outside the module
 *	- NAU8822 (mandatory)
 *
 * i2c_3 can be used outside the module
 *
 */

/* PMU TPS65023 */
static struct regulator_consumer_supply vdcdc1_supply[] = {
	{
		.supply		= "vdcdc1_1v2_mpu_iva",
	}
};

static struct regulator_consumer_supply vdcdc2_supply[] = {
	{
		.supply		= "vdcdc2_1v8",
	}
};

static struct regulator_consumer_supply vdcdc3_supply[] = {
	{
		.supply		= "vdcdc3_1v2_c",
	}
};

static struct regulator_consumer_supply vldo1_supply[] = {
	{
		.supply		= "ldo1_1v8_a_t",
	}
};

static struct regulator_consumer_supply vldo2_supply[] = {
	{
		.supply		= "ldo2_1v8_pll",
	},
};

static struct regulator_init_data regulator_init_data[] = {
	/* VDCDC1 /  1V2_MPU_IVA / OMAP 3630 MPU and IVA power supply */
	{
		.constraints = {
			.name			= "VDCDC1 / 1V2_MPU_IVA",
			.min_uV			= 1200000,
			.max_uV			= 1200000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= 0,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vdcdc1_supply),
		.consumer_supplies      = vdcdc1_supply,
        },
	/* VDCDC2 /  1V8 / Board prower supply */
	{
		.constraints = {
			.name			= "VDCDC2 / 1V8",
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= 0,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vdcdc2_supply),
		.consumer_supplies      = vdcdc2_supply,
        },
	/* VDCDC3 /  1V2_C / Omap 3630 Core power supply */
	{
		.constraints = {
			.name			= "VDCDC3 / 1V2_C",
			.min_uV			= 1200000,
			.max_uV			= 1200000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= 0,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vdcdc3_supply),
		.consumer_supplies      = vdcdc3_supply,
        },
	/* LDO1 / 1V8_A_T */
	{
		.constraints = {
			.name			= "LDO1 / 1V8_A_T",
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,
			.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
			.apply_uV		= true,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vldo1_supply),
		.consumer_supplies      = vldo1_supply,
        },
	/* LDO2 / 1V8_PLL  */
	{
		.constraints = {
			.name			= "LDO2 / 1V8_PLL",
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL,

			.always_on = 1, /* never turn this off */
		},
		.num_consumer_supplies  = ARRAY_SIZE(vldo2_supply),
		.consumer_supplies      = vldo2_supply,
        }
};

static struct pca8565_platform_data rtc_pca8565_info = {
	.clock_output	=	TIMER_CLOCK_OUT_32_KHZ, /* Set clock out 32kHz clock out for the OMAP processor */
	.timer_source	=	0
};
static struct i2c_board_info __initdata fc6100_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &regulator_init_data,
	},
	{
		I2C_BOARD_INFO("pca8565", 0x51),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &rtc_pca8565_info,
	},
};

#if defined(CONFIG_OMAP2_DSS_VENC)
//Mandatory : provide power supply to the tv out (see vdda_dac for more details)
static struct regulator_consumer_supply vldo1_dac_supply[] = {
	{
		.supply		= "vdda_dac",
	},
};
#endif

static void __init fc6100_mod_early_i2c_init(unsigned int i2c_set)
{
	// Muxing of I2C[1..3]]
	// i2c1_scl and i2c1_sda not muxable
	if( i2c_set & FC6100_USE_I2C2 ){
		omap_mux_init_signal("i2c2_scl.i2c2_scl",OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("i2c2_sda.i2c2_sda",OMAP_PIN_INPUT_PULLUP);
		parrot_omap_i2c_init(2);
	}

	if( i2c_set & FC6100_USE_I2C3 ){
		omap_mux_init_signal("i2c3_scl.i2c3_scl",OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_signal("i2c3_sda.i2c3_sda",OMAP_PIN_INPUT_PULLUP);
		parrot_omap_i2c_init(3);
	}

	parrot_omap_i2c_init(1);

	/* PMIC */
	omap_register_i2c_bus(1, 100, NULL, fc6100_i2c_bus1_info,
			ARRAY_SIZE(fc6100_i2c_bus1_info));

#if defined(CONFIG_OMAP2_DSS_VENC)
	//Mandatory : provide power supply to the tv out (see vdda_dac for more details)
	regulator_init_data[3].consumer_supplies = vldo1_dac_supply;
	regulator_init_data[3].num_consumer_supplies =  ARRAY_SIZE(vldo1_dac_supply);
#endif
}

/* 
 * USB Settings/Data
 */
#define	PHY0_PRESENT				(1<<8)

static struct smsc43340_usb_data fc6100_usb_otg_data = {
	/* pins must be set before being used */
	.gpio_reset		= ULPI_0_RST_N,
	.gpio_overcurrent	= USB_0_OC_N_HW02,
};

static struct platform_device fc6100_usb_otg_device = {
	.name		= "smsc43340_usbotg",
	.id		= -1,
	.dev		= {
		.platform_data = &fc6100_usb_otg_data,
	},
};

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = ULPI_1_RST_N,
	.reset_gpio_port[2]  = -EINVAL
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 500,
	//.extvbus 		= 1,
};

static int __init fc6100_mod_usb_init(int config_mask)
{

	if (fc6100_board_rev() & PHY0_PRESENT) {

		if (parrot_force_usb_device)
			musb_board_data.mode = MUSB_PERIPHERAL;
		//else if (config_mask & FC6100_USE_USB0_HOST)
		/* XXX setting HOST mode crash for an unknow reason */
		//else
		//	musb_board_data.mode = MUSB_HOST;

		/* Mux initialisation */
		omap_mux_init_signal("hsusb0_clk.hsusb0_clk", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_stp.hsusb0_stp", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("hsusb0_dir.hsusb0_dir", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_nxt.hsusb0_nxt", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data0.hsusb0_data0", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data1.hsusb0_data1", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data2.hsusb0_data2", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data3.hsusb0_data3", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data4.hsusb0_data4", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data5.hsusb0_data5", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data6.hsusb0_data6", OMAP_PIN_INPUT);
		omap_mux_init_signal("hsusb0_data7.hsusb0_data7", OMAP_PIN_INPUT);

		//Reset
		if (gpio_is_valid(fc6100_usb_otg_data.gpio_reset)){

			parrot_gpio_out_init(fc6100_usb_otg_data.gpio_reset, 0);
			mdelay(20);
			gpio_set_value(fc6100_usb_otg_data.gpio_reset, 1);
		}

		//Overcurrent and Pin id for USB 0
		if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW06)
			fc6100_usb_otg_data.gpio_overcurrent = USB_0_OC_N;

		if(fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {

			/* PROTO HW03 and following */
			if (0 && 
					!parrot_force_usb_device)
				/* force host mode : pin id to 0 */
				parrot_gpio_out_init(USB_0_ID, 1);
			else
				/* keep the pin id value */
				parrot_gpio_out_init(USB_0_ID, 0);
		}

		usb_musb_init(&musb_board_data);
	}

	// USB1
	if( gpio_is_valid(ehci_pdata.reset_gpio_port[1]) ){
		omap_mux_init_gpio(ehci_pdata.reset_gpio_port[1], OMAP_PIN_INPUT);
	}else{
		printk(KERN_ERR "%s: invalid data for usb\n", __func__);
		return -EINVAL;
	}

	// USB 1 overcurrent
	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW05)
		omap_mux_init_gpio(USB_1_OC_N,OMAP_PIN_INPUT_PULLUP);
	else
		//Gpio 126 for over current
		omap_mux_init_gpio(USB_1_OC_N_HW04,OMAP_PIN_INPUT_PULLUP);

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) {
		/* host only */
		parrot_gpio_out_init(USB_1_ID, 1);
	}

	usb_ehci_init(&ehci_pdata);

	return 0;
}

/* SPI Settings/Data
 *
 * i2c_1 does not go out of the module
 *	- PMIC	tps65023
 *	- RTC	pca8565
 *
 * i2c_2 can be used outside the module
 *	- NAU8822 (mandatory)
 *
 * i2c_3 can be used outside the module
 *
 */

struct omap2_mcspi_device_config fc6100_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
};

static struct omap2_mcspi_platform_config fc6100_mcspi3_platform_data = {
	.num_cs = 2,
	.mode = OMAP2_MCSPI_MASTER,
	.dma_mode = 0,
	.force_cs_mode = 1,
	.fifo_depth = 0,
	.regs_data = 0
};

static struct omap2_mcspi_platform_config fc6100_mcspi1_platform_data = {
	.num_cs = 1,
	.mode = OMAP2_MCSPI_MASTER,
	.dma_mode = 0,
	.force_cs_mode = 1,
	.fifo_depth = 0,
	.regs_data = 0
};

//Mux and Controler init
static void __init fc6100_mod_mux_ctrl_spi_init( unsigned int ctlr_set  )
{
	if ( ctlr_set & FC6100_USE_SPI1_CS0 ){
		/* MCSPI1 */
		omap_mux_init_signal("mcspi1_clk.mcspi1_clk", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcspi1_simo.mcspi1_simo", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcspi1_somi.mcspi1_somi", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcspi1_cs0.mcspi1_cs0", OMAP_PIN_INPUT);

		omap2_init_mcspi(&fc6100_mcspi1_platform_data, 1);
	}

	if ( ctlr_set & (FC6100_USE_SPI3_CS1 | FC6100_USE_SPI3_CS0)  ){
		/* MCSPI2 */
		omap_mux_init_signal("dss_data18.mcspi3_clk", OMAP_PIN_INPUT);
		omap_mux_init_signal("dss_data19.mcspi3_simo", OMAP_PIN_INPUT);
		omap_mux_init_signal("dss_data20.mcspi3_somi", OMAP_PIN_INPUT);

		if ( ctlr_set & FC6100_USE_SPI3_CS0 ){
			omap_mux_init_signal("dss_data21.mcspi3_cs0", OMAP_PIN_INPUT);
		}

		if ( (fc6100_mod_get_pcb_revision() < FC6100_0_HW05)
			&& ( ctlr_set & FC6100_USE_SPI3_CS1 ) ){
			omap_mux_init_signal("dss_data22.mcspi3_cs1", OMAP_PIN_INPUT);
		}

		omap2_init_mcspi(&fc6100_mcspi3_platform_data, 3);
	}
}

/*
 * Display settings
 *
 * LCD    : This parameter is board specific and must be register by the board
 *
 * TV_OUT : Common to all module, tv settings can be changed (composite or S-Video )
 *
 */

#if defined(CONFIG_OMAP2_DSS_VENC)
static int fc6100_panel_enable_tv(struct omap_dss_device *dssdev)
{
	struct regulator *vdac_reg;

	vdac_reg = regulator_get(NULL, "vdda_dac");
	if (IS_ERR(vdac_reg)) {
		printk(KERN_ERR "Unable to get vdac regulator\n");
		return PTR_ERR(vdac_reg);
	}
	return regulator_enable(vdac_reg);
}

static void fc6100_panel_disable_tv(struct omap_dss_device *dssdev)
{
	struct regulator *vdac_reg;

	vdac_reg = regulator_get(NULL, "vdda_dac");
	if (IS_ERR(vdac_reg)) {
		printk(KERN_ERR "Unable to get vpll2 regulator\n");
		return;
	}
	regulator_disable(vdac_reg);
}

static struct omap_dss_device fc6100_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.channel 		= OMAP_DSS_CHANNEL_DIGIT,
	.platform_enable        = fc6100_panel_enable_tv,
	.platform_disable       = fc6100_panel_disable_tv,
};
#endif



static struct omap_dss_device *fc6100_dss_devices[] = {
	NULL,
#if defined(CONFIG_OMAP2_DSS_VENC)
	&fc6100_tv_device,
#endif
};


static struct omap_dss_board_info fc6100_dss_data = {
	.num_devices = ARRAY_SIZE(fc6100_dss_devices),
	.devices = fc6100_dss_devices,
	.default_device = NULL,
};


int __init fc6100_mod_display_init(struct omap_dss_device *lcd_device)
{
	int loop, var;
	char mux_name[25];

#if defined(CONFIG_OMAP2_DSS_VENC)
	if (parrot_force_tv_composite 
		|| (board_config & FC6100_USE_COMPOSITE_TV) )
		fc6100_tv_device.phy.venc.type
				= OMAP_DSS_VENC_TYPE_COMPOSITE;

	if( fc6100_mod_get_config() & FC6100_USE_TVOUT_ONLY ){

		fc6100_dss_devices[0] = &fc6100_tv_device;
		fc6100_dss_data.num_devices = 1;

		omap_display_init(&fc6100_dss_data);
		return 0;
	}
#endif

	if(!lcd_device){
		printk(KERN_ERR "%s: not lcd device provided\n", __func__);
		return -EINVAL; 
	}

	fc6100_dss_devices[0] = lcd_device;
	fc6100_dss_data.default_device = lcd_device;

	omap_display_init(&fc6100_dss_data);

	omap_mux_init_signal("dss_acbias.dss_acbias", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("dss_pclk.dss_pclk", OMAP_PIN_OUTPUT);

	if(fc6100_mod_get_config() & FC6100_USE_LCD_HSYNC_VSYNC ){
		omap_mux_init_signal("dss_hsync.dss_hsync", OMAP_PIN_OUTPUT);
		omap_mux_init_signal("dss_vsync.dss_vsync", OMAP_PIN_OUTPUT);
	}
	else {
		/* force hsync/vsync to ground */
		parrot_gpio_out_init(67 /* dss_hsync.gpio_67 */, 0);
		parrot_gpio_out_init(68 /* dss_vsync.gpio_68 */, 0);
	}

	for (loop = 0; loop < 18; loop++) {
		sprintf(mux_name, "dss_data%i.dss_data%i", loop, loop);
		omap_mux_init_signal(mux_name, OMAP_PIN_OUTPUT);
	}

	for (loop = 0; loop < 7; loop++) {
		if ( loop != 2 ) {
			var = (loop > 2)? 0:1;
			sprintf(mux_name, "sys_boot%i.dss_data%i", loop,
				var + loop + 17 );
			omap_mux_init_signal(mux_name, OMAP_PIN_OUTPUT);
		}
	}

	return 0; 
}


/*
 * Ipod Init
 */
static void __init fc6100_mod_ipod_init(void)
{
	/* IPOD chip */
	/* Disable the reset of Ipod chip */
	parrot_gpio_user_out_init(FC_IPOD_RESET_N, 1, "ipod-rst");
}

/* 
 * Wifi : features common to all fc6100 products
 * Init GPIO and detect antenna
 */

static void __init fc6100_mod_mux_wifi_init(void)
{
	int val = 0;
	/*  RF reset released */
	parrot_gpio_user_out_init(RF_RST_N, 1, "bt-rst");
	/* Detection of wifi antenna	gpio_167 */
	parrot_gpio_in_init(WIFI_ANT_DET_N, 0);
	val = gpio_get_value(WIFI_ANT_DET_N);

	if (val != 0x00) {
		printk(KERN_ERR "Failed to detect wifi antenna\n");
	}

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW07){
		parrot_gpio_in_init(BT_EXT_ANT_DET_N, OMAP_PIN_INPUT_PULLUP);
		val = gpio_get_value(BT_EXT_ANT_DET_N);
		if (val != 0x00) {
			printk(KERN_ERR "Failed to detect BT antenna\n");
		}
	}
}

/* 
 * Camera Init
 */
static void __init fc6100_mod_mux_cam_init(int config)
{
	int loop;
	char mux_name[25];

	int max_pin = 10;


	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW05)
		// cam_d8 and cam_d9 are not used on HW05
		max_pin = 8;

	if (config & FC6100_USE_CAM_HSYNC_VSYNC) {
		omap_mux_init_signal("cam_hs.cam_hs", OMAP_PIN_INPUT);
		omap_mux_init_signal("cam_vs.cam_vs",OMAP_PIN_INPUT);
		if( fc6100_mod_get_pcb_revision() >= FC6100_0_HW06)
			omap_mux_init_signal("cam_fld.cam_fld", OMAP_PIN_INPUT);
	}
	else {
		parrot_gpio_in_init(94 /* cam_hs */, 0);
		parrot_gpio_in_init(95 /* cam_vs */, 0);
		if( fc6100_mod_get_pcb_revision() >= FC6100_0_HW06)
			parrot_gpio_in_init(98 /* cam_fld */, 0);
	}


	omap_mux_init_signal("cam_xclka.cam_xclka", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_pclk.cam_pclk", OMAP_PIN_INPUT);

	for (loop = 0; loop < max_pin; loop++) {
		sprintf(mux_name, "cam_d%i.cam_d%i", loop, loop);
		omap_mux_init_signal(mux_name, OMAP_PIN_INPUT);
	}
}

/* 
 * MMC/SD Settings/Data
 */

static struct omap2_hsmmc_info mmc_settings[4] __initdata  = {
	{
		.name		= "mmc3-SDIORF",
		.mmc		= 3,
		.ext_clock	= 0,
		.no_off		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34, /* force OCR */
		.power_saving   = false,
		.max_freq	= 24000000,

		.gpio_cd = -EINVAL,
		.gpio_wp = -EINVAL,
	},
};

struct omap2_hsmmc_info mmc1_settings __initdata = {
	.name		= "mmc1-SDcard",
	.mmc		= 1,
	.ext_clock	= 0,
	.no_off		= 1,
	.caps		= MMC_CAP_4_BIT_DATA,
	.power_saving   = false,

	.gpio_wp = FC_SDIO_0_WP_N,
	.gpio_cd = FC_SDIO_0_CD_N,
};

struct omap2_hsmmc_info mmc2_settings __initdata = {
	.name		= "mmc2-SDcard",
	.mmc		= 2,
	.ext_clock	= 1,	// Use of mmc2_clkin to synchronize data
				// mmc2_clkin : Input clock from MMC/SD/SDIO card
				// mmc2_clkin is used to resample the clock when using level shifter
	.no_off		= 1,

	.caps		= MMC_CAP_4_BIT_DATA,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,  /* force OCR */
	.power_saving   = false,
	.max_freq	= 24000000, /* Limitation of the maximum frequency of the second SDIO controller
					due the distance between the SDIO controller and connector */

	.gpio_cd = FC_SDIO_1_CD_N,
	.gpio_wp = FC_SDIO_1_WP_N,
};

//MMC settings can be changed before calling the common init
static void __init fc6100_mod_mmc_init(unsigned int mmc_options)
{
	int mmc_set = 1;

	/* MMC3 */
	omap_mux_init_signal("etk_clk.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_ctl.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d3.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc2_dat6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP);

	if (mmc_options & FC6100_USE_MMC1) {

		if( mmc_options & FC6100_DISABLE_MMC1_CD ){
			/* disable the gpio used to check "card-detect" feature */
			mmc1_settings.gpio_cd = -EINVAL;
		}else{
			/* MMC1 CD and WP */
			omap_mux_init_gpio(FC_SDIO_0_CD_N,OMAP_PIN_INPUT_PULLUP);
		}

		if( mmc_options & FC6100_DISABLE_MMC1_WP ){
			/* disable the gpio used to check "write-protect" feature */
			mmc1_settings.gpio_wp = -EINVAL;
		}else{
			if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW04) {
				omap_mux_init_gpio(FC_SDIO_0_WP_N,OMAP_PIN_INPUT_PULLUP);
			}else{
				//omap_mux_init_gpio(SDIO_0_WP_N_HW03_HW02,OMAP_PIN_INPUT_PULLUP); /* TODO, correct mistake in omap muxing cf  */
				//TODO omap_mux_init_signal("sim_rst.gpio_129", OMAP_PIN_INPUT_PULLUP); /* Must be checked */
				mmc1_settings.gpio_wp = FC_SDIO_0_WP_N_HW03_HW02;
				omap_writew(OMAP_PIN_INPUT_PULLUP|OMAP_MUX_MODE4, 0x48002a5a);
			}
		}

		memcpy(&mmc_settings[mmc_set++], &mmc1_settings, sizeof(struct omap2_hsmmc_info) );
	}

	if (mmc_options & FC6100_USE_MMC2) {

		if( mmc_options & FC6100_DISABLE_MMC2_CD ){
			/* disable the gpio used to check "card-detect" feature */
			mmc2_settings.gpio_cd = -EINVAL;
		}else{
			/* MMC2 CD */
			omap_mux_init_gpio(FC_SDIO_1_CD_N,OMAP_PIN_INPUT_PULLUP);
		}


		if( mmc_options & FC6100_DISABLE_MMC2_WP ){
			/* disable the gpio used to check "write-protect" feature */
			mmc2_settings.gpio_wp = -EINVAL;
		}else{
			/* MMC2 WP */
			omap_mux_init_gpio(FC_SDIO_1_WP_N,OMAP_PIN_INPUT_PULLUP);
		}

		memcpy(&mmc_settings[mmc_set], &mmc2_settings, sizeof(struct omap2_hsmmc_info) );
	}

	omap2_hsmmc_init(mmc_settings);

	/* Correct mmc output settings due to the use of level shifters
	 *  This change must not been done before the call to omap2_hsmmc_init
	 *  omap2_hsmmc_init make a default configuration that is good without
	 *  the use of level shifters */

	if (mmc_options & FC6100_USE_MMC2) {
		/* MMC1 and MMC2 muxing is done by omap2_mmc_mux */
		omap_mux_init_signal("sdmmc2_clkin", OMAP_PIN_INPUT);
		omap_mux_init_signal("sdmmc2_clk",OMAP_PIN_INPUT);
		omap_mux_init_signal("sdmmc2_cmd",OMAP_PIN_INPUT);
		omap_mux_init_signal("sdmmc2_dat0",OMAP_PIN_INPUT);

		omap_mux_init_signal("sdmmc2_dat1",OMAP_PIN_INPUT);
		omap_mux_init_signal("sdmmc2_dat2",OMAP_PIN_INPUT);
		omap_mux_init_signal("sdmmc2_dat3",OMAP_PIN_INPUT);
	}

}


/* 
 * Audio Settings/Data
 */

static void __init fc6100_mod_omap_audio_mux( unsigned int mcbsp_set )
{
	// muxing of
	omap_mux_init_signal("sys_clkout1", OMAP_PIN_INPUT);

	// Mandatory
	// muxing of the McBSP1
	omap_mux_init_signal("mcbsp1_dx",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("mcbsp1_dr", OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp1_fsx",
			OMAP_PIN_INPUT);
	omap_mux_init_signal("mcbsp1_clkx",
			OMAP_PIN_INPUT);

	// config PCM BT
	// PROTO HW02  and followings
	omap_mux_init_signal("mcbsp3_dx.mcbsp3_dx",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcbsp3_dr.mcbsp3_dr",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcbsp3_fsx.mcbsp3_fsx",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcbsp3_clkx.mcbsp3_clkx",
			OMAP_PIN_INPUT_PULLUP);

	if ( mcbsp_set & FC6100_USE_MCBSP2 ){
		// muxing of the McBSP2
		omap_mux_init_signal("mcbsp2_fsx.mcbsp2_fsx", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp2_clkx.mcbsp2_clkx", OMAP_PIN_INPUT);

		omap_mux_init_signal("mcbsp2_dr.mcbsp2_dr", OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp2_dx.mcbsp2_dx", OMAP_PIN_OUTPUT);
	}

	if ( (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03) 
		&& (mcbsp_set & FC6100_USE_MCBSP5) ) {
		omap_mux_init_signal("etk_d6.mcbsp5_dx",
						OMAP_PIN_INPUT);

		omap_mux_init_signal("etk_d5.mcbsp5_fsx",
				OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp_clks.mcbsp_clks",
				OMAP_PIN_INPUT);
	} else {
		omap_mux_init_signal("mcbsp_clks.safe_mode",
				OMAP_PIN_INPUT);
	}
}

/*
 * Miscellaneous
 */
static void __init fc6100_mod_miscellaneous(void)
{
	/* PROTO HW02 and followings */
	/* Output sys_clkreq */
	omap_mux_init_signal("sys_clkreq.sys_clkreq",
			 OMAP_PIN_INPUT);

	/* pins in safe mode */
	omap_mux_init_signal("gpmc_wait3.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("gpmc_clk.safe_mode", OMAP_PIN_INPUT);
	omap_mux_init_signal("sdmmc1_dat4.safe_mode", OMAP_PIN_INPUT);

	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW05) {
		/* was USB_1_OC_N_HW04 */
		omap_mux_init_signal("cam_strobe.safe_mode", OMAP_PIN_INPUT);
	}
	else if (fc6100_mod_get_pcb_revision() == FC6100_0_HW02) {
		omap_mux_init_signal("cam_d10.safe_mode", OMAP_PIN_INPUT);
		omap_mux_init_signal("cam_d11.safe_mode", OMAP_PIN_INPUT);
	}
}


/* 
 * Platform devices 
 */
struct platform_device fc6100_gpio = {
	.name		= "gpio",
	.id		= -1,
	.num_resources	= 0,
	.resource	= NULL,
};

//XXX TODO Check that it works with p6-acpower
struct platform_device fc6100_acpower_device = {
	.name		= "acpower",
	.id		= -1,
};

static struct platform_device *fc6100_devices[] __initdata = {
	&fc6100_gpio,
	&fc6100_acpower_device,
	&fc6100_usb_otg_device, /* usb otg */
	&omap_33,
};


/* NAND
 * NAND chip access: 8 bit */
#define FC6100_NAND_CS 0
static struct omap_nand_platform_data fc6100_nand_data = {
	.nand_setup	= NULL,
	.dma_channel	= -1,	/* disable DMA in OMAP NAND driver */
	.dev_ready	= (void *)1,
	.devsize	= 0,	/* '0' for 8-bit, '1' for 16-bit device */
	.cs		= FC6100_NAND_CS,
	.ecc_opt	= 0x2, /* HW ECC in romcode layout */
	.gpmc_baseaddr	= (void *) OMAP34XX_GPMC_VIRT,
	.gpmc_cs_baseaddr = (void *) (OMAP34XX_GPMC_VIRT+GPMC_CS0_BASE+FC6100_NAND_CS*GPMC_CS_SIZE),
};

void __init fc6100_mod_init_irq(void)
{
	parrot_omap_init_irq(166);
}

/**
 * FC6100 module common init
 * WARNING: this funtion must be called in board init before board specific settings
 * (I2C, MMC/SDIO, GPIO, etc...)
 * MMC settings can be changed before calling the common init
 */

void __init fc6100_mod_common_init(unsigned int mod_settings)
{

	fc6100_mod_print_board_rev();

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW02) {
		panic("this board is not supported anymore");
	}

	board_config = mod_settings;

	platform_device_register(&user_gpio);

	/* Signal info for measure */
	if (fc6100_mod_get_pcb_revision() >= FC6100_0_HW05) {
		//Sleep Request Input
		parrot_gpio_in_init(SLEEP_REQUEST_FLAG_HW05, 0);
		// Sleep Mode
		parrot_gpio_out_init(SLEEPOUT_N, 0);
	}

	/* Serial init */
	fc6100_mod_uart_mux_init(board_config & FC6100_USE_UART1_RTS_CTS);

	/* I2C (internal init) */
	fc6100_mod_early_i2c_init(board_config & FC6100_USE_ALL_I2C);
	
	/* mmc init */
	fc6100_mod_mmc_init(board_config & (FC6100_USE_ALL_MMC
					| FC6100_DISABLE_MMC1_WP_CD
					| FC6100_DISABLE_MMC2_WP_CD) );

	/* USB */
	fc6100_mod_usb_init(board_config);

	/* SPI */
	fc6100_mod_mux_ctrl_spi_init( board_config & FC6100_USE_SPI_ALL );

	/* Audio init */
	fc6100_mod_omap_audio_mux( board_config & FC6100_USE_ALL_MCBSP );

	/* Wifi gpio init **/
	fc6100_mod_mux_wifi_init();

	/* Camera init */
	fc6100_mod_mux_cam_init(board_config);

	/* Register OMAP3 camera devices (tvp5151) */
	fc6100_camera_init();

	/* Ipod init */
	fc6100_mod_ipod_init();

	/* Micellaneous */
	fc6100_mod_miscellaneous();

	/* no smartreflex with TPS65023 PMIC */
	/* SmartReflex */
	//sr_class3_init();
	parrot_omap_voltage_init();

	/* Init NAND Flash */
	parrot_omap_gpmc_nand_config(&fc6100_nand_data);
	gpmc_nand_init(&fc6100_nand_data);

	/* Register devices */
	platform_add_devices(fc6100_devices, ARRAY_SIZE(fc6100_devices));
}
