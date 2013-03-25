/*
 * fc6100.c  --  SoC audio for FC6100
 *
 * Author: Christian ROSALIE <christian.rosalie@parrot.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <plat/hardware.h>
#include <plat/gpio.h>
#include <plat/mcbsp.h>
#include <plat/mux.h>
#include <plat/control.h>
#include <mach/board-fc6100.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/wau8822.h"


#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE) \
     || defined(CONFIG_SND_SOC_FC6100_I2S_CODECS) || defined(CONFIG_SND_SOC_FC6100_I2S_CODECS_MODULE)
#error Do not use anymore WAU8822 I2S codec and Nuvoton NAU8820 and other I2S slave codecs use PALCODEC for init
#endif

static struct clk *sys_clkout1;
static int sys_clkout1_state;

static ssize_t fc6100_audio_clk_get(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", sys_clkout1_state?"on":"off");
}


static ssize_t fc6100_audio_clk_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	if (!strncmp(buf, "on", size) && !sys_clkout1_state) {
		sys_clkout1_state = 1;
        clk_enable(sys_clkout1);
	}
	else if (!strncmp(buf, "off", size) && sys_clkout1_state) {
        clk_disable(sys_clkout1);
		sys_clkout1_state = 0;
	}

	return size;
}

static DEVICE_ATTR(fc6100_audio_clk, 0644,
		   fc6100_audio_clk_get, fc6100_audio_clk_set);

static struct snd_soc_dai_driver null_dai[4] = {
/* Suitable for PALCODEC */
        {
                .name = "wau8822-hifi",
                .playback = {
                        .stream_name  = "Playback",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates = SNDRV_PCM_RATE_48000,
                        .formats = SNDRV_PCM_FMTBIT_S16_LE
                                 | SNDRV_PCM_FMTBIT_S32_LE,
                },
                .capture = {
                        .stream_name  = "Capture",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates = SNDRV_PCM_RATE_48000,
                        .formats = SNDRV_PCM_FMTBIT_S16_LE
                                 | SNDRV_PCM_FMTBIT_S32_LE,
                 },
        },
        {
                .name = "PCM",
                .playback = {
                        .stream_name = "Playback",
                        .channels_min = 1,
                        .channels_max = 1,
                        .rates = SNDRV_PCM_RATE_8000,
                        .formats = SNDRV_PCM_FMTBIT_S16_LE
                                 | SNDRV_PCM_FMTBIT_U8
                                 | SNDRV_PCM_FMTBIT_S8,
                },
                .capture = {
                        .stream_name = "Capture",
                        .channels_min = 1,
                        .channels_max = 1,
                        .rates = SNDRV_PCM_RATE_8000,
                        .formats = SNDRV_PCM_FMTBIT_S16_LE
                                 | SNDRV_PCM_FMTBIT_U8
                                 | SNDRV_PCM_FMTBIT_S8,
                },
        },
};

/*
 * External codec connected to I2S OMAP MCBSP2 and MCBSP5
 * The goal of this structure is to initialize the soc controller (MCBSP) of the OMAP 3630
 * It register as a Digital Audio Interfaces (DAI) and can be used with linux system but
 * controlled by PAL codec
 */
static struct snd_soc_dai_driver mcbsp2_dai __initdata = {
	.name = "codc_48k_S16LE",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static struct snd_soc_dai_driver mcbsp5_dai __initdata = {
	.name = "codc2_48k_S16LE",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = { 0 }
};

static int fc6100_pcm_hw_params(struct snd_pcm_substream *substream,
        struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
        int ret;

        /* Set cpu DAI configuration */
        ret = snd_soc_dai_set_fmt( cpu_dai, SND_SOC_DAIFMT_DSP_A
					| SND_SOC_DAIFMT_IB_NF
					| SND_SOC_DAIFMT_CBM_CFM );

        if (ret < 0) {
                printk(KERN_ERR "can't set cpu DAI configuration\n");
                return ret;
        }

       	/* Use external clock for mcBSP3 */
       	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT,
               	        0, SND_SOC_CLOCK_OUT);

       	if (ret < 0) {
               	printk(KERN_ERR "can't use external clock pin clkx\n");
               	return ret;
       	}

        return 0;
}

static struct snd_soc_ops fc6100_pcm_ops = {
        .hw_params = fc6100_pcm_hw_params,
};

static int fc6100_i2s_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
        int ret;

        /* Set cpu DAI configuration */
        ret = snd_soc_dai_set_fmt(cpu_dai,
                                  SND_SOC_DAIFMT_I2S |
                                  SND_SOC_DAIFMT_NB_NF |
                                  SND_SOC_DAIFMT_CBM_CFM);

        if (ret < 0) {
                printk(KERN_ERR "can't set cpu DAI configuration\n");
                return ret;
        }

        if( !strcmp(cpu_dai->name, "omap-mcbsp-dai.0") ){
                ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT,
                                0, SND_SOC_CLOCK_OUT);

                if (ret < 0) {
                        printk(KERN_ERR "can't use external clock pin clkx\n");
                        return ret;
                }

                /* to use CLKR from CLKX and FSR from FSX , only for MCBSP1 */
                ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_FSR_SRC_FSX, 0, SND_SOC_CLOCK_OUT);
                if (ret < 0) {
                        printk(KERN_ERR "can't use FSR from FSX\n");
                        return ret;
                }

                ret=snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_CLKR_SRC_CLKX, 0, SND_SOC_CLOCK_OUT);
                if (ret < 0) {
                        printk(KERN_ERR "can't use CLKR from CLKX\n");
                        return ret;
                }

        }else if( !strcmp(cpu_dai->name, "omap-mcbsp-dai.1") ){

                ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKX_EXT,
                                0, SND_SOC_CLOCK_OUT);

                if (ret < 0) {
                        printk(KERN_ERR "can't use external clock pin clkx\n");
                        return ret;
                }

        }else if( !strcmp(cpu_dai->name, "omap-mcbsp-dai.4") ){
		struct omap_mcbsp_data *mcbsp_data = snd_soc_dai_get_drvdata(cpu_dai);
		struct omap_mcbsp_reg_cfg *regs = &mcbsp_data->regs;

                ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 1);
                if (ret < 0) {
                        printk(KERN_ERR "can't set codec clock divisor\n");
                        return ret;
                }

                ret=snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_EXT, 0, SND_SOC_CLOCK_OUT);
                if (ret < 0) {
                        printk(KERN_ERR "can't use CLKS for MCBSP5\n");
                        return ret;
                }
 
		regs->srgr2 |= CLKSP;
        } 

        return 0;
}

static int fc6100_i2s_startup(struct snd_pcm_substream *substream)
{
	    if (!sys_clkout1_state)
		        printk(KERN_WARNING, "fc6100 i2s MCLK is not enabled!\n");

        return 0;
}

static struct snd_soc_ops fc6100_i2s_ops = {
        .hw_params = fc6100_i2s_hw_params,
        .startup = fc6100_i2s_startup,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link fc6100_dai[4] = {
        {/* Nuvoton nau8820/wau8822 codec */
                .name = "I2S_CODEC",
                .stream_name = "I2S_CODEC_STRM",
                .cpu_dai_name = "omap-mcbsp-dai.0", //For MCBSP1
                .codec_dai_name = "wau8822-hifi",
                .platform_name = "omap-pcm-audio",
                /* Configuration for Pal codec */
                .no_codec = 1, /* TODO: have a dummy CODEC */
                .ops = &fc6100_i2s_ops,
        },
        {/* Marvel Bluetooth chip */
                .name = "BT_PCM",
                .stream_name = "BT",
                .cpu_dai_name = "omap-mcbsp-dai.2", //For MCBSP3
                .platform_name = "omap-pcm-audio",

                .codec_dai_name = "PCM",
                .no_codec = 1, /* TODO: have a dummy CODEC */
                .ops = &fc6100_pcm_ops,
        },
};

static struct snd_soc_dai_link mcbsp2_dai_link __initdata = {
/* I2S codec not defined */
	.name = "I2S_EXT_CODEC",
	.stream_name = "I2S_EXT_CODEC_STRM",
	.cpu_dai_name = "omap-mcbsp-dai.1", //For MCBSP2
	.platform_name = "omap-pcm-audio",

	.codec_dai_name = "codc_48k_S16LE",
	.no_codec = 1, /* TODO: have a dummy CODEC */
	.ops = &fc6100_i2s_ops,
};

static struct snd_soc_dai_link mcbsp5_dai_link  __initdata = {
/* I2S codec not defined */
	.name = "I2S_EXT_CODEC2",
	.stream_name = "I2S_EXT_CODEC_STRM2",
	.cpu_dai_name = "omap-mcbsp-dai.4", //For MCBSP5
	.platform_name = "omap-pcm-audio",

	.codec_dai_name = "codc2_48k_S16LE",
	.no_codec = 1, /* TODO: have a dummy CODEC */
	.ops = &fc6100_i2s_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_card_fc6100 = {
        .name = "FC6100",
        .long_name = "FC6100 (wau8822)",
        .dai_link = fc6100_dai,
        .num_links = 2, /* MCBSP1 and MCBSP3 are the default configuration  */
};


static struct platform_device *fc6100_snd_device;

static int __init fc6100_soc_init(void)
{
        int ret;
	unsigned int mcbsp_set = fc6100_mod_get_audio_config();

        if (!machine_is_fc6100_module())
        {
                pr_debug("Not FC6100!\n");
                return -ENODEV;
        }

	if (fc6100_mod_get_pcb_revision() < FC6100_0_HW02) {
		panic("this board is not supported anymore");
	}

        printk(KERN_INFO "FC6100 SoC init\n");

        fc6100_snd_device = platform_device_alloc("soc-audio", -1);
        if (!fc6100_snd_device) {
                printk(KERN_ERR "%s:%d Platform device allocation failed\n",  __FILE__, __LINE__);
                return -ENOMEM;
        }

	/* snd_soc_card_fc6100.num_links is used to register the correct number of dais
         * according to the proto revisions (HW00, HW02 or HW03 and followings)
         *
         * List of MCBSP used
         * Common  to all proto :
         * - MCBSP1 : wau8822/nau8820 codec
         *
         * On Proto HW00 (no more supported)
         * - MCBSP4 : Marvel Bluetooth chip
         *
         * On Proto HW02
         * - MCBSP3 : Marvel Bluetooth chip
         * - MCBSP2 : 2nd I2S codec (not defined)
         *
         * On Proto HW03 and followings
         * - MCBSP3 : Marvel Bluetooth chip
         * - MCBSP2 : 2nd I2S codec (not defined)
         * - MCBSP5 : 3rd I2S codec (not defined, only playback)
         * */

	if ( mcbsp_set & FC6100_USE_MCBSP2 ){
		memcpy(&null_dai[snd_soc_card_fc6100.num_links], &mcbsp2_dai, sizeof(struct snd_soc_dai_driver) );
		memcpy(&fc6100_dai[snd_soc_card_fc6100.num_links++], &mcbsp2_dai_link, sizeof(struct snd_soc_dai_link) );
	}

	if ( (fc6100_mod_get_pcb_revision() >= FC6100_0_HW03)
		&& (mcbsp_set & FC6100_USE_MCBSP5) ) {

		memcpy(&null_dai[snd_soc_card_fc6100.num_links], &mcbsp5_dai, sizeof(struct snd_soc_dai_driver) );
		memcpy(&fc6100_dai[snd_soc_card_fc6100.num_links++], &mcbsp5_dai_link, sizeof(struct snd_soc_dai_link) );
	}

	snd_soc_register_dais(&fc6100_snd_device->dev, null_dai, snd_soc_card_fc6100.num_links);

        platform_set_drvdata(fc6100_snd_device, &snd_soc_card_fc6100);

        ret = platform_device_add(fc6100_snd_device);
        if (ret){
        	printk(KERN_ERR "%s:%d Unable to add platform device\n",  __FILE__, __LINE__);
                goto err1;
	}

        // set and enable the clock
        sys_clkout1 = clk_get(NULL, "sys_clkout1");
        if (IS_ERR(sys_clkout1)){
        	printk(KERN_ERR "%s: Could not get sys_clkout1.\n",  __FILE__);
                goto clk_err;
	}
	device_create_file(&fc6100_snd_device->dev,
			   &dev_attr_fc6100_audio_clk);


        return 0;

clk_err:
        clk_put(sys_clkout1);
err1:
        platform_device_put(fc6100_snd_device);

        return ret;
}
module_init(fc6100_soc_init);

static void __exit fc6100_soc_exit(void)
{
        platform_device_unregister(fc6100_snd_device);
        if (sys_clkout1_state) {
                clk_disable(sys_clkout1);
                sys_clkout1_state = 0;
        }
        clk_put(sys_clkout1);
}
module_exit(fc6100_soc_exit);

MODULE_AUTHOR("Christian ROSALIE <christian.rosalie@parrot.com>");
MODULE_DESCRIPTION("ALSA SoC FC6100");
MODULE_LICENSE("GPL");

