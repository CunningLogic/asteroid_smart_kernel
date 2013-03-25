/*
 * fidji.c  --  SoC audio for Fidji
 *
 * Author: François MULLER <francois.muller@parrot.com>
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

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/pmb8763.h"

static int fidji_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM);

	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	return 0;
}

int fidji_pcm_hw_free(struct snd_pcm_substream *substream)
{
	//omap_mux_config("MCBSP3_TRISTATE");
	return 0;
}

static struct snd_soc_ops fidji_pcm_ops = {
	.hw_params = fidji_pcm_hw_params,
	.hw_free = fidji_pcm_hw_free,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link fidji_dai[] = {
    {
        .name = "BT_PCM",
        .stream_name = "BT",
        .platform_name = "omap-pcm-audio",
        .cpu_dai_name = "omap-mcbsp-dai.2", /* PCM on MCBSP3 */
        .codec_name = "pmb8763-codec",
        .codec_dai_name = "pmb8763-bt",
        .ops = &fidji_pcm_ops,
    },
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_fidji = {
	.name = "Fidji",
	.long_name = "Fidji (pmb8763)",
	.dai_link = fidji_dai,
	.num_links = ARRAY_SIZE(fidji_dai),
};

struct platform_device pmb8763_codec_device = {
		.name = "pmb8763-codec",
		.id = -1,
};

static struct platform_device *fidji_snd_device;

static int __init fidji_soc_init(void)
{
	int ret;

	if (!machine_is_omap_fidji())
	{
		pr_debug("Not Fidji!\n");
		return -ENODEV;
	}

	printk(KERN_INFO "Fidji SoC init\n");

	platform_device_register(&pmb8763_codec_device);
	fidji_snd_device = platform_device_alloc("soc-audio", -1);
	if (!fidji_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(fidji_snd_device, &snd_soc_fidji);
	ret = platform_device_add(fidji_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(fidji_snd_device);

	return ret;
}
module_init(fidji_soc_init);

static void __exit fidji_soc_exit(void)
{
	platform_device_unregister(fidji_snd_device);
	platform_device_unregister(&pmb8763_codec_device);
}
module_exit(fidji_soc_exit);

MODULE_AUTHOR("François MULLER <francois.muller@parrot.com>");
MODULE_DESCRIPTION("ALSA SoC Fidji");
MODULE_LICENSE("GPL");

