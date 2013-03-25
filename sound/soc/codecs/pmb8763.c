/*
 * ALSA Soc PMB8763 BT codec support
 *
 * Author:  Francois MULLER
 * Copyright 2010 PARROT SA
 *
 * Based on PCM3008 Soc codec, original copyright follow:
 * Copyright 2008 Lyrtech inc
 * Copyright 2005 Wolfson Microelectronics PLC.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * Generic PMB8763 support.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "pmb8763.h"

#define PMB8763_VERSION "0.1"

static struct snd_soc_dai_driver pmb8763_dai[] = {
    {
        .name = "pmb8763-bt",
        .playback = {
            .stream_name = "input",
            .channels_min = 1,
            .channels_max = 1,
            .rates = SNDRV_PCM_RATE_8000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
        .capture = {
            .stream_name = "output",
            .channels_min = 1,
            .channels_max = 1,
            .rates = SNDRV_PCM_RATE_8000,
            .formats = SNDRV_PCM_FMTBIT_S16_LE,
        },
    },
};

static int pmb8763_soc_probe(struct snd_soc_codec *codec)
{
    int ret = 0;

    printk(KERN_INFO "PMB8763 SoC Audio Codec %s\n", PMB8763_VERSION);

    return ret;
}

static int pmb8763_soc_remove(struct snd_soc_codec *codec)
{
    return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_pmb8763 = {
    .probe =    pmb8763_soc_probe,
    .remove =   pmb8763_soc_remove,
    .suspend =  NULL,
    .resume =   NULL,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_pmb8763);

static int __devinit pmb8763_codec_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_pmb8763, pmb8763_dai, ARRAY_SIZE(pmb8763_dai));
}

static int __devexit pmb8763_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

MODULE_ALIAS("platform:pmb8763-codec");

static struct platform_driver pmb8763_codec_driver = {
	.probe		= pmb8763_codec_probe,
	.remove		= __devexit_p(pmb8763_codec_remove),
	.driver		= {
		.name	= "pmb8763-codec",
		.owner	= THIS_MODULE,
	},
};

static int __init pmb8763_modinit(void)
{
	return platform_driver_register(&pmb8763_codec_driver);
}
module_init(pmb8763_modinit);

static void __exit pmb8763_exit(void)
{
	platform_driver_unregister(&pmb8763_codec_driver);
}
module_exit(pmb8763_exit);

MODULE_DESCRIPTION("Soc PMB8763 driver");
MODULE_AUTHOR("Francois MULLER");
MODULE_LICENSE("GPL");
