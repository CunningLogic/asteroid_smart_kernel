#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/tsc2007.h>
#include <../drivers/parrot/input/touchscreen/atmel_mxt_ts.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mmc/host.h>

#include <linux/switch.h>
#include <plat/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/timer-gp.h>

#include <plat/usb.h>
#include <plat/mux.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <plat/control.h>

#include <plat/display.h>
//#include <plat/hdq.h>

#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#include <asm/mach/flash.h>
#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/opp_twl_tps.h>

#include "hsmmc.h"

#include "parrot-common.h"

#ifdef CONFIG_OMAP_MUX
struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif


/* Parrot_installer : force nand gpmc config */
static int parrot_installer_setup;

static int __init parrot_installer(char *str)
{
	parrot_installer_setup = 1;
	return 1;
}

__setup("parrot_installer", parrot_installer);



static struct regulator_consumer_supply omap_33_consumers[] = {
	{
		.supply		= "vmmc", /* Fake, not really link to this regulator */
		.dev_name	= "mmci-omap-hs.0",
	}
};

static struct regulator_init_data omap_33_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(omap_33_consumers),
	.consumer_supplies = omap_33_consumers,
};

static struct fixed_voltage_config omap_33_pdata = {
	.supply_name = "B_PWR_33V",
	.microvolts = 3300000,
	.init_data = &omap_33_data,
	.gpio = -EINVAL,
};

struct platform_device omap_33 = {
	.name          = "reg-fixed-voltage",
	.id            = -1,
	.dev = {
		.platform_data = &omap_33_pdata,
	},
};

struct platform_device user_gpio = {
	.name          = "user_gpio",
	.id            = -1,
	.dev = {
		.platform_data = NULL,
	},
};

/* UART1 DEBUG, UART2 AT, UART3 BT Marvell */
//TOOD : Clarify the muxing of the serial ports
// rts, cts according to the muxing

/* Seems to be UART 1,2,3,4 in this order */
static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.flags		= 0
	}
};


void parrot_omap_serial_init()
{
	omap_serial_init(omap_serial_platform_data);
}

void parrot_omap_i2c_init(int i2c_num)
{
	/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		if (i2c_num == 1) {
			prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
			/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
			prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
			omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);
		} else if (i2c_num == 2) {
			prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
			/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
			prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
			omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);
		} else if (i2c_num == 3) {
			prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
			/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
			prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
			omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);
		} else if (i2c_num == 4) {
			prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
			/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
			prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
			omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
		}
	}
}

static void parrot_omap_mux_init_gpio(int gpio, int direction)
{
	if (gpio == 131) {
		omap_mux_init_signal("sdmmc2_cmd.gpio_131", direction);
		return;
	}
	omap_mux_init_gpio(gpio, direction);
}

int parrot_gpio_out_init(int gpio, int val)
{
	int r;

	parrot_omap_mux_init_gpio(gpio, OMAP_PIN_INPUT);

	r = gpio_request(gpio, "parrot gpio");
	if (r) {
		printk(KERN_WARNING "Could not request GPIO %d\n", gpio);
		return r;
	}
	r = gpio_direction_output(gpio, val);
	if (r)
		printk(KERN_WARNING "Could not set GPIO %d\n", gpio);
	return r;
}

void parrot_gpio_user_out_init(int gpio, int val, char *alias)
{
	int r;
	r = parrot_gpio_out_init(gpio, val);
	if (r)
	  return;

	gpio_export(gpio, 0);
	if (alias != NULL) {
		printk(KERN_ERR "Export user out GPIO %d with alias '%s'\n", gpio, alias);
		r = gpio_export_link(&user_gpio.dev, alias, gpio);
		if (r)
			printk(KERN_ERR "   can not export link %s : %d\n", alias, r);
	}
	else {
		printk(KERN_ERR "Export user out GPIO %d without alias\n", gpio);
	}
}


int parrot_gpio_in_init(int gpio, int val)
{
	int r;
	parrot_omap_mux_init_gpio(gpio, OMAP_PIN_INPUT|val);

	r = gpio_request(gpio, "parrot gpio");
	if (r) {
		printk(KERN_WARNING "Could not request GPIO %d\n", gpio);
		return r;
	}
	r = gpio_direction_input(gpio);
	if (r)
		printk(KERN_WARNING "Could not set GPIO %d\n", gpio);
	return r;
}

void parrot_gpio_user_in_init(int gpio, int val, char *alias)
{
	int r;
	r = parrot_gpio_in_init(gpio, val);
	if (r)
	  return;

	gpio_export(gpio, 0);
	if (alias != NULL) {
		printk(KERN_ERR "Export user in GPIO %d with alias '%s'\n", gpio, alias);
		gpio_export_link(&user_gpio.dev, alias, gpio);
	}
	else {
		printk(KERN_ERR "Export user in GPIO %d without alias\n", gpio);
	}
}


static int tsc2007_irq;
/* Touchscreen TSC2007 */
static int ts_init(void)
{
	gpio_request(tsc2007_irq, "pen irq");
	gpio_direction_input(tsc2007_irq);

	return 0;
}

static int ts_get_pendown_state(void)
{
	int val = 0;

	val = gpio_get_value(tsc2007_irq);

	return val ? 0 : 1;
}

static struct tsc2007_platform_data tsc2007_info = {
	.model			= 2007,
	.x_plate_ohms		= 800,
	.get_pendown_state	= ts_get_pendown_state,
	.init_platform_hw	= ts_init,
};

void tsc2007_init(int irq, struct i2c_board_info *info)
{
	tsc2007_irq = irq;
	info->platform_data = &tsc2007_info;
	info->irq = gpio_to_irq(irq);
	omap_mux_init_gpio(irq, OMAP_PIN_INPUT_PULLUP|OMAP_PIN_OFF_WAKEUPENABLE);
}

static int mxt224e_irq;
/* Touchscreen MXT224E */
u8 mxt224e_read_irq(void)
{
	return gpio_get_value(mxt224e_irq);
}

void mxt224e_init(int irq, int hw_reset, struct i2c_board_info *info, int reset_value)
{
	if (gpio_is_valid(irq)) {
		omap_mux_init_gpio(irq, OMAP_PIN_INPUT_PULLUP|OMAP_PIN_OFF_WAKEUPENABLE);
		if (gpio_request(irq, "ATMEL MX224E_message") < 0) {
			printk(KERN_ERR "Failed to request GPIO %d for ""ATMEL MX224E_message IRQ\n", irq);
			return;
		}
		mxt224e_irq = irq;
		info->irq = gpio_to_irq(irq);
		gpio_direction_input(irq);
	}
	else
		info->irq = -1;

	if (gpio_is_valid(hw_reset)) {
               /* Reset Hard */
	       parrot_gpio_out_init(hw_reset, reset_value);
	       mdelay(10);
	       gpio_set_value(hw_reset, !reset_value);

	}

}

/* fake power management for pmu */
static struct omap_volt_vc_data vc_config = {
	/* MPU */
	.vdd0_on	= 1200000, /* 1.2v */
	.vdd0_onlp	= 1000000, /* 1.0v */
	.vdd0_ret	=  975000, /* 0.975v */
	.vdd0_off	=  600000, /* 0.6v */
	/* CORE */
	.vdd1_on	= 1150000, /* 1.15v */
	.vdd1_onlp	= 1150000, /* 1.15v */
	.vdd1_ret	=  975000, /* 0.975v */
	.vdd1_off	=  600000, /* 0.6v */

	.clksetup	= 0xff,
	.voltoffset	= 0xff,
	.voltsetup2	= 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
};

/* PMIC IC information */
/**
 * omap_tps65023_uv_to_vsel - convert TPS65023 VSEL value to microvolts DC
 * @vsel: TPS65023 VSEL value to convert
 *
 * Returns the microvolts DC that the TPS65023 PMIC should
 * generate when programmed with @vsel.
 */
static unsigned long omap_tps65023_vsel_to_uv(const u8 vsel)
{
	return ((vsel * 2500) + 800000);
}

static u8 omap_tps65023_uv_to_vsel(unsigned long uV)
{
	return DIV_ROUND_UP(uV - 800000, 2500);
}

#define omap_tps65023_onforce_cmd 	not_used_on_omap3
#define omap_tps65023_on_cmd 		not_used_on_omap3
#define omap_tps65023_sleepforce_cmd 	not_used_on_omap3
#define omap_tps65023_sleep_cmd 	not_used_on_omap3
#define omap_tps65023_onforce_cmd 	not_used_on_omap3

static u8 not_used_on_omap3(const u8 vsel)
{
	return 0; /* not used on OMAP3, not clear for OMAP4  */
}



/* Link between PMIC / MPU */
static struct omap_volt_pmic_info omap_pmic_mpu = { /* and iva */
	.name = "mpu_info",
	.slew_rate = 14400, /* maximum 14.4mV/us */
	.step_size = 2500, /* 25 mV */ 
	.i2c_addr = 0x48,
	.i2c_vreg = 0x6, /* (vdd0) VDCDC1 -> VDD1_CORE -> VDD_MPU */
	.vsel_to_uv = omap_tps65023_vsel_to_uv,
	.uv_to_vsel = omap_tps65023_uv_to_vsel,
	.onforce_cmd = omap_tps65023_onforce_cmd,
	.on_cmd = omap_tps65023_on_cmd,
	.sleepforce_cmd = omap_tps65023_sleepforce_cmd,
	.sleep_cmd = omap_tps65023_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x0, /* means 0.8 V */
	.vp_vlimitto_vddmax = 0x20, /* means 1.6 V */
};

/* TODO : Fixed voltage on VDCDC2 see datasheet ELECTRICAL CHARACTERISTICS p 8  */
static struct omap_volt_pmic_info omap_pmic_core = { 
	.name = "core_info",
	.slew_rate = 14400, /* maximum 14.4mV/us */
	.step_size = 2500, /* 25 mV */ 
	.i2c_addr = 0x48,
	.i2c_vreg = 0x6, /* (vdd1) VDD2 -> VDD2_CORE -> VDD_CORE */
	.vsel_to_uv = omap_tps65023_vsel_to_uv,
	.uv_to_vsel = omap_tps65023_uv_to_vsel,
	.onforce_cmd = omap_tps65023_onforce_cmd,
	.on_cmd = omap_tps65023_on_cmd,
	.sleepforce_cmd = omap_tps65023_sleepforce_cmd,
	.sleep_cmd = omap_tps65023_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x0, /* means 0.8 V */
	.vp_vlimitto_vddmax = 0x20, /* means 1.6 V */
/*	.vp_vstepmin_vstepmin = 0x0,
	.vp_vstepmax_vstepmax = 0x0,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x0, 
	.vp_vlimitto_vddmax = 0x0, */
};

void parrot_omap_voltage_init(void)
{
#ifdef CONFIG_PM
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_init_vc(&vc_config);
#endif
}

/* this is ugly, very ugly : there are fuse for that ... */

/* OPP MPU/IVA Clock Frequency */
struct opp_frequencies {
	unsigned long mpu;
	unsigned long iva;
};

static struct opp_frequencies opp_freq_add_table[] __initdata = {
  {
	.mpu = 800000000,
	.iva = 660000000,
  },
  /* even if some 3630 on fidji/fc6100 support 1Ghz, set the same freq
	 for everybody
	*/
#if 0
  {
	.mpu = 1000000000,
	.iva =  800000000,
  },
#endif
#if 0
  /*1.2GHz has been observed to cause issues on ES1.1 boards and requires
  further investigation.*/
  {
	.mpu = 1200000000,
	.iva =   65000000,
  },
#endif

  { 0, 0 },
};

/* must be called after omap2_common_pm_init() */
static int __init zoom3_opp_init(void)
{
	struct omap_hwmod *mh, *dh;
	struct omap_opp *mopp, *dopp;
	struct device *mdev, *ddev;
	struct opp_frequencies *opp_freq;


	if (!cpu_is_omap3630())
		return 0;

	mh = omap_hwmod_lookup("mpu");
	if (!mh || !mh->od) {
		pr_err("%s: no MPU hwmod device.\n", __func__);
		return 0;
	}

	dh = omap_hwmod_lookup("iva");
	if (!dh || !dh->od) {
		pr_err("%s: no DSP hwmod device.\n", __func__);
		return 0;
	}

	mdev = &mh->od->pdev.dev;
	ddev = &dh->od->pdev.dev;

	/* add MPU and IVA clock frequencies */
	for (opp_freq = opp_freq_add_table; opp_freq->mpu; opp_freq++) {
		/* check enable/disable status of MPU frequecy setting */
		mopp = opp_find_freq_exact(mdev, opp_freq->mpu, false);
		if (IS_ERR(mopp))
			mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		if (IS_ERR(mopp)) {
			pr_err("%s: MPU does not support %lu MHz\n", __func__, opp_freq->mpu / 1000000);
			continue;
		}

		/* check enable/disable status of IVA frequency setting */
		dopp = opp_find_freq_exact(ddev, opp_freq->iva, false);
		if (IS_ERR(dopp))
			dopp = opp_find_freq_exact(ddev, opp_freq->iva, true);
		if (IS_ERR(dopp)) {
			pr_err("%s: DSP does not support %lu MHz\n", __func__, opp_freq->iva / 1000000);
			continue;
		}

		/* try to enable MPU frequency setting */
		if (opp_enable(mopp)) {
			pr_err("%s: OPP cannot enable MPU:%lu MHz\n", __func__, opp_freq->mpu / 1000000);
			continue;
		}

		/* try to enable IVA frequency setting */
		if (opp_enable(dopp)) {
			pr_err("%s: OPP cannot enable DSP:%lu MHz\n", __func__, opp_freq->iva / 1000000);
			opp_disable(mopp);
			continue;
		}

		/* verify that MPU and IVA frequency settings are available */
		mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		dopp = opp_find_freq_exact(ddev, opp_freq->iva, true);
		if (!mopp || !dopp) {
			pr_err("%s: OPP requested MPU: %lu MHz and DSP: %lu MHz not found\n",
				__func__, opp_freq->mpu / 1000000, opp_freq->iva / 1000000);
			continue;
		}

		dev_info(mdev, "OPP enabled %lu MHz\n", opp_freq->mpu / 1000000);
		dev_info(ddev, "OPP enabled %lu MHz\n", opp_freq->iva / 1000000);
	}

	return 0;
}
device_initcall(zoom3_opp_init);

#include "sdrc.h"
#include <plat/sdrc.h>
#include "cm.h"
static struct omap_sdrc_params parrot_sdrc_params_cs0[3] = { };
static struct omap_sdrc_params parrot_sdrc_params_cs1[3] = { };


void __init parrot_omap_init_irq(int rate)
{
	omap_board_config = NULL;
	omap_board_config_size = 0;

	/* get the bootloader config */

	/* XXX we can't read the clock here, the clk framework is not
	   init here (done by omap2_init_common_hw)...
	 */
	parrot_sdrc_params_cs0[0].rate = rate * 1000000;

	parrot_sdrc_params_cs0[0].actim_ctrla = sdrc_read_reg(SDRC_ACTIM_CTRL_A_0);
	parrot_sdrc_params_cs0[0].actim_ctrlb = sdrc_read_reg(SDRC_ACTIM_CTRL_B_0);
	parrot_sdrc_params_cs0[0].rfr_ctrl = sdrc_read_reg(SDRC_RFR_CTRL_0);
	parrot_sdrc_params_cs0[0].mr = sdrc_read_reg(SDRC_MR_0);

	parrot_sdrc_params_cs0[1].rate = 0 * 1000000;

	parrot_sdrc_params_cs0[1].actim_ctrla = sdrc_read_reg(SDRC_ACTIM_CTRL_A_0);
	parrot_sdrc_params_cs0[1].actim_ctrlb = sdrc_read_reg(SDRC_ACTIM_CTRL_B_0);
	parrot_sdrc_params_cs0[1].rfr_ctrl = sdrc_read_reg(SDRC_RFR_CTRL_0);
	parrot_sdrc_params_cs0[1].mr = sdrc_read_reg(SDRC_MR_0);

	parrot_sdrc_params_cs0[2].rate = 0;

	/* if SDRC_RFR_CTRL_1 = 0 (reset value), CS1 is not used */
	parrot_sdrc_params_cs1[0].rate = rate * 1000000;

	parrot_sdrc_params_cs1[0].actim_ctrla = sdrc_read_reg(SDRC_ACTIM_CTRL_A_1);
	parrot_sdrc_params_cs1[0].actim_ctrlb = sdrc_read_reg(SDRC_ACTIM_CTRL_B_1);
	parrot_sdrc_params_cs1[0].rfr_ctrl = sdrc_read_reg(SDRC_RFR_CTRL_1);
	parrot_sdrc_params_cs1[0].mr = sdrc_read_reg(SDRC_MR_1);

	parrot_sdrc_params_cs1[1].rate = 0 * 1000000;
	parrot_sdrc_params_cs1[1].actim_ctrla = sdrc_read_reg(SDRC_ACTIM_CTRL_A_1);
	parrot_sdrc_params_cs1[1].actim_ctrlb = sdrc_read_reg(SDRC_ACTIM_CTRL_B_1);
	parrot_sdrc_params_cs1[1].rfr_ctrl = sdrc_read_reg(SDRC_RFR_CTRL_1);
	parrot_sdrc_params_cs1[1].mr = sdrc_read_reg(SDRC_MR_1);

	parrot_sdrc_params_cs1[2].rate = 0;

	omap2_init_common_hw( parrot_sdrc_params_cs0, parrot_sdrc_params_cs1 );
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif

	omap_init_irq();
}

void __init parrot_omap_map_io(void)
{
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}

static int parrot_pm_begin(suspend_state_t state)
{
	return 0;
}

static int parrot_never_suspend(suspend_state_t state)
{
	return 0;
}

struct platform_suspend_ops parrot_pm_ops = {
	.begin		= parrot_pm_begin,
	.valid		= parrot_never_suspend,
};

static int __init parrot_pm_init(void)
{
	if (!cpu_is_omap34xx())
		return -ENODEV;

	printk(KERN_ERR "Power Management for Parrot OMAP3.\n");

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&parrot_pm_ops);
#endif /* CONFIG_SUSPEND */

	return(0);
}

late_initcall(parrot_pm_init);

#ifndef CONFIG_PM_RUNTIME
/* CONFIG_PM_RUNTIME need to be set because
   if not the sgx_fck won't be disable by skip_setup_idle

PVR_K:(Error): EnableSystemClocks: Couldn't set SGX parent clock (-16) [489, t/f
c6100_droid/kernel/omap/drivers/gpu/pvr/omap3/sysutils_linux_wqueue_compat.c]   
PVR_K:(Error): SysInitialise: Failed to Enable system clocks (176) [514, t/fc610
0_droid/kernel/omap/drivers/gpu/pvr/omap3/sysconfig.c]
 */
#error gpu will crash
#endif

#ifndef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
#error please enable dma on nand
#endif

#ifndef CONFIG_OMAP_WATCHDOG
#error please enable CONFIG_OMAP_WATCHDOG
#endif

#ifndef CONFIG_REGULATOR_FIXED_VOLTAGE
/* CONFIG_REGULATOR_FIXED_VOLTAGE need to be set to avoid kernel crashing
 */
#error kernel will crash without this option
#endif


void parrot_omap_gpmc_nand_config(struct omap_nand_platform_data *_nand_data)
{
	return;
}

