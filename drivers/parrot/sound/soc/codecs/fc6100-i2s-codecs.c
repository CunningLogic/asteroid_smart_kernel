/*
 * fc6100-i2s-codecs.h  --  WAU8822 Soc Audio driver and I2S slave codec (based on wau8822.c)
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 *
 * Authors: Liam Girdwood <lg@opensource.wolfsonmicro.com>
 * 2008.12.05 Henry Lo <CYLo@nuvoton.com> modified for WAU8822
 * 2010.07.01 Merge WAU8822 and WAU 8 8 2 2 and Wm8974 for WAU 8 8 2 2 CF Yang3
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "fc6100-i2s-codecs.h"

#define AUDIO_NAME "fc6100-i2s-codecs"
#define WAU8822_VERSION "0.7"

#define WAU8822_POWER1_BIASEN  0x08
#define WAU8822_POWER1_BUFIOEN 0x04

#define wau8822_reset(c) wau8822_write(c, WAU8822_RESET, 0)

 enum{
        MASTER_I2S_CODEC,
        SND_I2S_CODEC,
        TRD_I2S_CODEC,
        NO_MORE_CODEC
 };

enum{
        MASTER_I2S_CODEC_PLAYBACK,
        MASTER_I2S_CODEC_CAPTURE,
        SND_I2S_CODEC_PLAYBACK,
        SND_I2S_CODEC_CAPTURE,
        TRD_I2S_CODEC_PLAYBACK,
        MAX_SUBSTREAM
};

struct dai_state{
        atomic_t playback_active;
        atomic_t capture_active;
};

struct wau8822_priv {
        enum snd_soc_control_type control_type;
        void *control_data;
        struct snd_soc_codec codec;

        /* Manage the I2S substreams (Capture, Playback) in use
        * in order to deactivate or not the clocks
        * (Master, Bits and Frame clocks of the master I2S Codec)
        */
        atomic_t substream;		// number of substream (set/unset)
        atomic_t fmt_set;		// dai fmt (set/unset)
        atomic_t pll_set;		// dai pll (set/unset)
        atomic_t dai_clkdiv_set;	// dai clkdiv (set/unset)
        atomic_t hw_params_set;		// dai hardware params (set/unset)
        atomic_t bias_level_set;	// dai bias level (set/unset)
        atomic_t mute_set;		// dai mute (set/unset)
};

static const u16 wau8822_reg[WAU8822_CACHEREGNUM] = {
        0x0000, 0x0000, 0x0000, 0x0000,
        0x0050, 0x0000, 0x0140, 0x0000,
        0x0000, 0x0080, 0x0000, 0x00ff,
        0x00ff, 0x0021, 0x0100, 0x00ff,
        0x00ff, 0x0000, 0x012c, 0x002c,
        0x002c, 0x002c, 0x002c, 0x0000,
        0x0032, 0x0000, 0x0000, 0x0000,
        0x0000, 0x0000, 0x0000, 0x0000,
        0x0038, 0x000b, 0x0032, 0x0000,
        0x0008, 0x000c, 0x0093, 0x00e9,
        0x0000, 0x0000, 0x0000, 0x0000,
        0x0033, 0x0010, 0x0010, 0x0100,
        0x0100, 0x0002, 0x0001, 0x0001,
        0x0039, 0x0039, 0x0039, 0x0039,
        0x0001, 0x0001, 0x0   , 0x0   ,
        0x20  , 0x0   , 0x0   , 0x1A  ,
};


static u16 Set_Codec_Reg_Init[][2]={
        {CFG_POWER_MANAGEMENT1,         0x001f},  //  1   1f 
        {CFG_POWER_MANAGEMENT2,         0x01bf},  //  2  1bf
        {CFG_POWER_MANAGEMENT3,         0x007f},  //  3   7f
        {CFG_AUDIO_INTERFACE,           0x0010},  //  4   10
        {CFG_COMPANDING_CTRL,           0x0000},  //  5    0
        {CFG_CLK_GEN_CTRL,              0x0000},  //  6    0
        {CFG_GPIO,                      0x0004},  //  8    4
        {CFG_LEFT_ADC_DIGITAL_VOL,      0x01ff},  //  f  1ff
        {CFG_RIGHT_ADC_DIGITAL_VOL,     0x01ff},  // 10  1ff
        {CFG_DAC_CTRL,                  0x0008},  //  a    8
        {CFG_BEEP_CONTROL,              0x0   },  // 2b   0
        {CFG_LEFT_MIXER_CTRL,           0x0001},  // 32    1
        {CFG_RIGHT_MIXER_CTRL,          0x0001},  // 33    1
        {CFG_OUTPUT_CTRL,               0x0002},  // 31    2
        {CFG_LOUT2_SPKR_VOLUME_CTRL,    0x0139},  // 36  139
        {CFG_JACK_DETECT_CTRL2,         0x0000},  //  d   0
        {CFG_JACK_DETECT_CTRL1,         0x0000},  //  9   0

        {CFG_LEFT_ADC_BOOST_CTRL,       0x55 },  // 2f  55
        {CFG_RIGHT_ADC_BOOST_CTRL,      0x55 },  // 30  55
        {CFG_LEFT_INP_PGA_GAIN_CTRL,    0x110},  // 2d  110
        {CFG_RIGHT_INP_PGA_GAIN_CTRL,   0x110},  // 2e  110
};
#define SET_CODEC_REG_INIT_NUM  ARRAY_SIZE(Set_Codec_Reg_Init)

/*
 * read wau8822 register cache
 */
static inline unsigned int wau8822_read_reg_cache(struct snd_soc_codec  *codec,
  unsigned int reg)
{
        u16 *cache = codec->reg_cache;
        if (reg == WAU8822_RESET)
                return 0;
        if (reg >= WAU8822_CACHEREGNUM)
               return -1;
        return cache[reg];
}

/*
 * write wau8822 register cache
 */
static inline void wau8822_write_reg_cache(struct snd_soc_codec  *codec,
  u16 reg, unsigned int value)
{
        u16 *cache = codec->reg_cache;
        if (reg >= WAU8822_CACHEREGNUM)
                return;
        cache[reg] = value;
}

/*
 * write to the WAU8822 register space
 */
static int wau8822_write(struct snd_soc_codec  *codec, unsigned int reg,
  unsigned int value)
{
        wau8822_write_reg_cache (codec, reg, value);

        if (!snd_soc_write(codec, reg, value)) {
                dev_dbg(codec->dev, "%s=> reg:0x%08x,value:0x%08x success\n", __FUNCTION__, reg, value);
                return 0;
        }
        else {
                dev_err(codec->dev, "%s=> reg:0x%08x,value:0x%08x error\n", __FUNCTION__, reg, value);
                return -EIO;
        }
}

void Wau8822DumpRegister(struct snd_soc_codec *codec)
{
        int i;

        dev_dbg(codec->dev, "\n\n>>>>>>>>>> Dump Reg Cache <<<<<<<<<<<<\n");
        for(i=0; i<WAU8822_CACHEREGNUM; i++) {
                if((i%8 == 0) && i)
                        dev_dbg(codec->dev, "\n");

                dev_dbg(codec->dev, "R%d:%04x  ", i, ((u16 *)codec->reg_cache)[i]);
        }
        printk("\n\n");
}


static const char *wau8822_companding[] = {"Off", "NC", "u-law", "A-law" };
static const char *wau8822_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz" };
static const char *wau8822_eqmode[] = {"Capture", "Playback" };
static const char *wau8822_bw[] = {"Narrow", "Wide" };
static const char *wau8822_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz" };
static const char *wau8822_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz" };
static const char *wau8822_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz" };
static const char *wau8822_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz" };
static const char *wau8822_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz" };
static const char *wau8822_alc[] =
    {"ALC both on", "Limiter", "ALC left only", "ALC right only"};

static const struct soc_enum wau8822_enum[] = {
  SOC_ENUM_SINGLE(WAU8822_COMP, 1, 4, wau8822_companding), /* adc */
  SOC_ENUM_SINGLE(WAU8822_COMP, 3, 4, wau8822_companding), /* dac */
  SOC_ENUM_SINGLE(WAU8822_DAC,  4, 4, wau8822_deemp),
  SOC_ENUM_SINGLE(WAU8822_EQ1,  8, 2, wau8822_eqmode),

  SOC_ENUM_SINGLE(WAU8822_EQ1,  5, 4, wau8822_eq1),
  SOC_ENUM_SINGLE(WAU8822_EQ2,  8, 2, wau8822_bw),
  SOC_ENUM_SINGLE(WAU8822_EQ2,  5, 4, wau8822_eq2),
  SOC_ENUM_SINGLE(WAU8822_EQ3,  8, 2, wau8822_bw),

  SOC_ENUM_SINGLE(WAU8822_EQ3,  5, 4, wau8822_eq3),
  SOC_ENUM_SINGLE(WAU8822_EQ4,  8, 2, wau8822_bw),
  SOC_ENUM_SINGLE(WAU8822_EQ4,  5, 4, wau8822_eq4),
  SOC_ENUM_SINGLE(WAU8822_EQ5,  8, 2, wau8822_bw),

  SOC_ENUM_SINGLE(WAU8822_EQ5,  5, 4, wau8822_eq5),
  SOC_ENUM_SINGLE(WAU8822_ALC3,  8, 2, wau8822_alc),
};

//2010.8.18 snd_soc_put_volsw_with_update
int update(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
        unsigned int reg = mc->reg;
        unsigned int shift = mc->shift;
        unsigned int rshift = mc->rshift;
        int max = mc->max;
        unsigned int mask = (1 << fls(max)) - 1;
        unsigned int invert = mc->invert;
        unsigned int val, val2, val_mask;
//snd_soc_update_bits_locked declare
        int change;
        unsigned int old, new;
//snd_soc_update_bits_locked declare end //

        val = (ucontrol->value.integer.value[0] & mask);
        if (invert)
                val = max - val;
        val_mask = mask << shift;
        val = val << shift;

        if (shift != rshift) {
                val2 = (ucontrol->value.integer.value[1] & mask);
                if (invert)
                        val2 = max - val2;
                val_mask |= mask << rshift;
                val |= val2 << rshift;
        }
//return snd_soc_update_bits_locked(codec, reg, val_mask, val);
        old = snd_soc_read(codec, reg);
        new = (old & ~mask) | val;
        change = old != new;

        new |= 0x100; //add update bit for  b,c,2d,2e,34,35,36,37
        if (change)
                wau8822_write(codec, reg, new);

        return change;
}


//#define SOC_SINGLE(xname, reg, shift, mask, invert)
//Defines a single control as follows:-
//  xname = Control name e.g. "Playback Volume"
//  reg = codec register
//  shift = control bit(s) offset in register
//  mask = control bit size(s) e.g. mask of 7 = 3 bits
//  invert = the control is inverted
static const struct snd_kcontrol_new wau8822_snd_controls[] = {
//x5
SOC_SINGLE("Digital Loopback Switch", WAU8822_COMP, 0, 1, 0),
//SOC_ENUM("ADC Companding", wau8822_enum[0]),
//SOC_ENUM("DAC Companding", wau8822_enum[1]),
//x9
SOC_SINGLE("Jack Detection Enable", WAU8822_JACK1, 6, 1, 0),
//a
SOC_SINGLE("DAC Right Inversion Switch", WAU8822_DAC, 1, 1, 0),
SOC_SINGLE("DAC Left Inversion Switch",  WAU8822_DAC, 0, 1, 0),
//b,c
SOC_SINGLE_EXT("Left Playback Volume",  WAU8822_DACVOLL, 0, 255, 0, snd_soc_get_volsw, update),
SOC_SINGLE_EXT("Right Playback Volume", WAU8822_DACVOLR, 0, 255, 0, snd_soc_get_volsw, update),
//xe 
SOC_SINGLE("High Pass Filter Switch",    WAU8822_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Cut Off",          WAU8822_ADC, 4, 7, 0),
SOC_SINGLE("Right ADC Inversion Switch", WAU8822_ADC, 1, 1, 0),
SOC_SINGLE("Left ADC Inversion Switch",  WAU8822_ADC, 0, 1, 0),
//xf,x10
SOC_SINGLE_EXT("Left Capture Volume",  WAU8822_ADCVOLL,  0, 255, 0, snd_soc_get_volsw, update),
SOC_SINGLE_EXT("Right Capture Volume", WAU8822_ADCVOLR,  0, 255, 0, snd_soc_get_volsw, update),
//x11
//SOC_ENUM("Equaliser Function", wau8822_enum[3]),
//SOC_ENUM("EQ1 Cut Off", wau8822_enum[4]),
//SOC_SINGLE("EQ1 Volume", WAU8822_EQ1,  0, 31, 1),
//x13
//SOC_ENUM("Equaliser EQ2 Bandwith", wau8822_enum[5]),
//SOC_ENUM("EQ2 Cut Off", wau8822_enum[6]),
//SOC_SINGLE("EQ2 Volume", WAU8822_EQ2,  0, 31, 1),
//x14
//SOC_ENUM("Equaliser EQ3 Bandwith", wau8822_enum[7]),
//SOC_ENUM("EQ3 Cut Off", wau8822_enum[8]),
//SOC_SINGLE("EQ3 Volume", WAU8822_EQ3,  0, 31, 1),
//x15
//SOC_ENUM("Equaliser EQ4 Bandwith", wau8822_enum[9]),
//SOC_ENUM("EQ4 Cut Off", wau8822_enum[10]),
//SOC_SINGLE("EQ4 Volume", WAU8822_EQ4,  0, 31, 1),
//x16
//SOC_ENUM("Equaliser EQ5 Bandwith", wau8822_enum[11]),
//SOC_ENUM("EQ5 Cut Off", wau8822_enum[12]),
//SOC_SINGLE("EQ5 Volume", WAU8822_EQ5,  0, 31, 1),
//x18
SOC_SINGLE("DAC Playback Limiter Switch", WAU8822_DACLIM1,  8, 1, 0),
SOC_SINGLE("DAC Playback Limiter Decay", WAU8822_DACLIM1,  4, 15, 0),
SOC_SINGLE("DAC Playback Limiter Attack", WAU8822_DACLIM1,  0, 15, 0),
//x19
SOC_SINGLE("DAC Playback Limiter Threshold", WAU8822_DACLIM2,  4, 7, 0),
SOC_SINGLE("DAC Playback Limiter Boost", WAU8822_DACLIM2,  0, 15, 0),
//x20
SOC_SINGLE("ALC Enable Switch", WAU8822_ALC1,  8, 1, 0),
SOC_SINGLE("ALC Capture Max Gain", WAU8822_ALC1,  3, 7, 0),
SOC_SINGLE("ALC Capture Min Gain", WAU8822_ALC1,  0, 7, 0),
//x21
//SOC_SINGLE("ALC Capture ZC Switch", WAU8822_ALC2,  8, 1, 0),
SOC_SINGLE("ALC Capture Hold", WAU8822_ALC2,  4, 7, 0),
SOC_SINGLE("ALC Capture Target", WAU8822_ALC2,  0, 15, 0),
//x22
SOC_ENUM("ALC Capture Mode",     wau8822_enum[13]),
SOC_SINGLE("ALC Capture Decay",  WAU8822_ALC3,  4, 15, 0),
SOC_SINGLE("ALC Capture Attack", WAU8822_ALC3,  0, 15, 0),
//x23
SOC_SINGLE("ALC Capture Noise Gate Switch",      WAU8822_NGATE,  3, 1, 0),
SOC_SINGLE("ALC Capture Noise Gate Threshold",   WAU8822_NGATE,  0, 7, 0),
//x2d
SOC_SINGLE_EXT("Left Capture PGA ZC Switch",     WAU8822_INPPGAL,  7,  1, 0, snd_soc_get_volsw, update),
SOC_SINGLE_EXT("Left Capture PGA Volume",        WAU8822_INPPGAL,  0, 63, 0, snd_soc_get_volsw, update),
//x2e
SOC_SINGLE_EXT("Right Capture PGA ZC Switch",    WAU8822_INPPGAR,  7,  1, 0, snd_soc_get_volsw, update),
SOC_SINGLE_EXT("Right Capture PGA Volume",       WAU8822_INPPGAR,  0, 63, 0, snd_soc_get_volsw, update),
//x34
SOC_SINGLE("Left Headphone Playback ZC Switch",  WAU8822_HPVOLL,  7,  1, 0),
SOC_SINGLE("Left Headphone Playback Switch",     WAU8822_HPVOLL,  6,  1, 1),
SOC_SINGLE_EXT("Left Headphone Playback Volume", WAU8822_HPVOLL,  0, 63, 0, snd_soc_get_volsw, update),
//x35
SOC_SINGLE("Right Headphone Playback ZC Switch", WAU8822_HPVOLR,  7,  1, 0),
SOC_SINGLE("Right Headphone Playback Switch",    WAU8822_HPVOLR,  6,  1, 1),
SOC_SINGLE_EXT("Right Headphone Playback Volume",WAU8822_HPVOLR,  0, 63, 0, snd_soc_get_volsw, update),
//x36
SOC_SINGLE("Left Speaker Playback ZC Switch",    WAU8822_SPKVOLL, 7,  1, 0),
SOC_SINGLE("Left Speaker Playback Switch",       WAU8822_SPKVOLL, 6,  1, 1),
SOC_SINGLE_EXT("Left Speaker Playback Volume",   WAU8822_SPKVOLL, 0, 63, 0, snd_soc_get_volsw, update),
//x37
SOC_SINGLE("Right Speaker Playback ZC Switch",   WAU8822_SPKVOLR, 7,  1, 0),
SOC_SINGLE("Right Speaker Playback Switch",      WAU8822_SPKVOLR, 6,  1, 1),
SOC_SINGLE_EXT("Right Speaker Playback Volume",  WAU8822_SPKVOLR, 0, 63, 0, snd_soc_get_volsw, update),
//0x2f,0x30
SOC_DOUBLE_R("Capture Boost(+20dB)", WAU8822_ADCBOOSTL, WAU8822_ADCBOOSTR,  8, 1, 0),
};


/* Left Output Mixer */
static const struct snd_kcontrol_new wau8822_left_mixer_controls[] = {
        SOC_DAPM_SINGLE("Right PCM Playback Switch", WAU8822_OUTPUT, 6, 1, 1),
        SOC_DAPM_SINGLE("Left PCM Playback Switch", WAU8822_MIXL, 0, 1, 1),
        SOC_DAPM_SINGLE("Line Bypass Switch", WAU8822_MIXL, 1, 1, 0),
        SOC_DAPM_SINGLE("Aux Playback Switch", WAU8822_MIXL, 5, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wau8822_right_mixer_controls[] = {
        SOC_DAPM_SINGLE("Left PCM Playback Switch", WAU8822_OUTPUT, 5, 1, 1),
        SOC_DAPM_SINGLE("Right PCM Playback Switch", WAU8822_MIXR, 0, 1, 1),
        SOC_DAPM_SINGLE("Line Bypass Switch", WAU8822_MIXR, 1, 1, 0),
        SOC_DAPM_SINGLE("Aux Playback Switch", WAU8822_MIXR, 5, 1, 0),
};


/* BIASGEN control */
static const struct snd_kcontrol_new wau8822_biasg_en_controls =
SOC_DAPM_SINGLE("BIAS Gen", WAU8822_BIASGEN, 4, WAU8822_5V_MODE?1:0, 0);

/* Speaker boost control */
static const struct snd_kcontrol_new wau8822_speaker_boost_controls =
SOC_DAPM_SINGLE("Speaker Boost", WAU8822_SPKBOOST, 2, WAU8822_5V_MODE?0:1, 0);
/* Aux1 boost control */
static const struct snd_kcontrol_new wau8822_out3_boost_controls =
SOC_DAPM_SINGLE("Out3 Boost", WAU8822_AUX1BOOST, 3, WAU8822_5V_MODE?0:1, 0);
/* Aux2 boost control */
static const struct snd_kcontrol_new wau8822_out4_boost_controls =
SOC_DAPM_SINGLE("Out4 Boost", WAU8822_AUX2BOOST, 4, WAU8822_5V_MODE?0:1, 0);

/* Left AUX Input boost vol */
static const struct snd_kcontrol_new wau8822_laux_boost_controls =
SOC_DAPM_SINGLE("Left Aux Volume", WAU8822_ADCBOOSTL, 0, 3, 0);

/* Right AUX Input boost vol */
static const struct snd_kcontrol_new wau8822_raux_boost_controls =
SOC_DAPM_SINGLE("Right Aux Volume", WAU8822_ADCBOOSTR, 0, 3, 0);

/* Left Input boost vol */
static const struct snd_kcontrol_new wau8822_lmic_boost_controls =
SOC_DAPM_SINGLE("Left Input Volume", WAU8822_ADCBOOSTL, 4, 3, 0);

/* Right Input boost vol */
static const struct snd_kcontrol_new wau8822_rmic_boost_controls =
SOC_DAPM_SINGLE("Right Input Volume", WAU8822_ADCBOOSTR, 4, 3, 0);

/* Left Aux In to PGA */
static const struct snd_kcontrol_new wau8822_laux_capture_boost_controls =
SOC_DAPM_SINGLE("Left Capture Switch", WAU8822_ADCBOOSTL,  8, 1, 0);

/* Right  Aux In to PGA */
static const struct snd_kcontrol_new wau8822_raux_capture_boost_controls =
SOC_DAPM_SINGLE("Right Capture Switch", WAU8822_ADCBOOSTR,  8, 1, 0);

/* Left Input P In to PGA */
static const struct snd_kcontrol_new wau8822_lmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Left Input P Capture Boost Switch", WAU8822_INPUT,  0, 1, 0);

/* Right Input P In to PGA */
static const struct snd_kcontrol_new wau8822_rmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Right Input P Capture Boost Switch", WAU8822_INPUT,  4, 1, 0);

/* Left Input N In to PGA */
static const struct snd_kcontrol_new wau8822_lmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Left Input N Capture Boost Switch", WAU8822_INPUT,  1, 1, 0);

/* Right Input N In to PGA */
static const struct snd_kcontrol_new wau8822_rmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Right Input N Capture Boost Switch", WAU8822_INPUT,  5, 1, 0);

// TODO Widgets
static const struct snd_soc_dapm_widget wau8822_dapm_widgets[] = {
#if 0
        //SND_SOC_DAPM_MUTE("Mono Mute", WAU8822_MONOMIX, 6, 0),
        //SND_SOC_DAPM_MUTE("Speaker Mute", WAU8822_SPKMIX, 6, 0),

        SND_SOC_DAPM_MIXER("Speaker Mixer", WAU8822_POWER3, 2, 0,
                  &wau8822_speaker_mixer_controls[0],
        ARRAY_SIZE(wau8822_speaker_mixer_controls)),
        SND_SOC_DAPM_MIXER("Mono Mixer", WAU8822_POWER3, 3, 0,
                &wau8822_mono_mixer_controls[0],
        ARRAY_SIZE(wau8822_mono_mixer_controls)),
        SND_SOC_DAPM_DAC("DAC", "HiFi Playback", WAU8822_POWER3, 0, 0),
        SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WAU8822_POWER3, 0, 0),
        SND_SOC_DAPM_PGA("Aux Input", WAU8822_POWER1, 6, 0, NULL, 0),
        SND_SOC_DAPM_PGA("SpkN Out", WAU8822_POWER3, 5, 0, NULL, 0),
        SND_SOC_DAPM_PGA("SpkP Out", WAU8822_POWER3, 6, 0, NULL, 0),
        SND_SOC_DAPM_PGA("Mono Out", WAU8822_POWER3, 7, 0, NULL, 0),
        SND_SOC_DAPM_PGA("Mic PGA", WAU8822_POWER2, 2, 0, NULL, 0),

        SND_SOC_DAPM_PGA("Aux Boost", SND_SOC_NOPM, 0, 0,
          &wau8822_aux_boost_controls, 1),
        SND_SOC_DAPM_PGA("Mic Boost", SND_SOC_NOPM, 0, 0,
          &wau8822_mic_boost_controls, 1),
        SND_SOC_DAPM_SWITCH("Capture Boost", SND_SOC_NOPM, 0, 0,
          &wau8822_capture_boost_controls),

        SND_SOC_DAPM_MIXER("Boost Mixer", WAU8822_POWER2, 4, 0, NULL, 0),

        SND_SOC_DAPM_MICBIAS("Mic Bias", WAU8822_POWER1, 4, 0),

        SND_SOC_DAPM_INPUT("MICN"),
        SND_SOC_DAPM_INPUT("MICP"),
        SND_SOC_DAPM_INPUT("AUX"),
        SND_SOC_DAPM_OUTPUT("MONOOUT"),
        SND_SOC_DAPM_OUTPUT("SPKOUTP"),
        SND_SOC_DAPM_OUTPUT("SPKOUTN"),
#endif
};

static const struct snd_soc_dapm_route audio_map[] = {
        /* Mono output mixer */
        {"Mono Mixer", "PCM Playback Switch", "DAC"},
        {"Mono Mixer", "Aux Playback Switch", "Aux Input"},
        {"Mono Mixer", "Line Bypass Switch", "Boost Mixer"},

        /* Speaker output mixer */
        {"Speaker Mixer", "PCM Playback Switch", "DAC"},
        {"Speaker Mixer", "Aux Playback Switch", "Aux Input"},
        {"Speaker Mixer", "Line Bypass Switch", "Boost Mixer"},

        /* Outputs */
        {"Mono Out", NULL, "Mono Mixer"},
        {"MONOOUT", NULL, "Mono Out"},
        {"SpkN Out", NULL, "Speaker Mixer"},
        {"SpkP Out", NULL, "Speaker Mixer"},
        {"SPKOUTN", NULL, "SpkN Out"},
        {"SPKOUTP", NULL, "SpkP Out"},

        /* Boost Mixer */
        {"Boost Mixer", NULL, "ADC"},
        {"Capture Boost Switch", "Aux Capture Boost Switch", "AUX"},
        {"Aux Boost", "Aux Volume", "Boost Mixer"},
        {"Capture Boost", "Capture Switch", "Boost Mixer"},
        {"Mic Boost", "Mic Volume", "Boost Mixer"},

        /* Inputs */
        {"MICP", NULL, "Mic Boost"},
        {"MICN", NULL, "Mic PGA"},
        {"Mic PGA", NULL, "Capture Boost"},
        {"AUX", NULL, "Aux Input"},
};

struct pll_ {
        unsigned int pre_div:4; /* prescale - 1 */
        unsigned int n:4;
        unsigned int k;
};

static struct pll_ pll_div;

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static void pll_factors(unsigned int target, unsigned int source)
{
        unsigned long long Kpart;
        unsigned int K, Ndiv, Nmod;

        Ndiv = target / source;
        if (Ndiv < 6) {
                source >>= 1;
                pll_div.pre_div = 1;
                Ndiv = target / source;
        } else
                pll_div.pre_div = 0;

        if ((Ndiv < 6) || (Ndiv > 12))
                printk(KERN_WARNING
                        "WAU8822 N value %d outwith recommended range!d\n",
                        Ndiv);

        pll_div.n = Ndiv;
        Nmod = target % source;
        Kpart = FIXED_PLL_SIZE * (long long)Nmod;

        do_div(Kpart, source);

        K = Kpart & 0xFFFFFFFF;

        /* Check if we need to round */
        if ((K % 10) >= 5)
                K += 5;

        /* Move down to proper range now rounding is done */
        K /= 10;

        pll_div.k = K;

#if defined(DEBUG)
       printk(KERN_DEBUG "%s : pll_factor, freq_in %d, freq_out %d, pre_div %d, Ndiv %d, K %d \n",
              __FUNCTION__,source, target,pll_div.pre_div,pll_div.n,pll_div.k ); 
#endif

}


static int wau8822_set_dai_pll(struct snd_soc_dai *codec_dai,
        int pll_id, int source, unsigned int freq_in, unsigned int freq_out)
{
        struct snd_soc_codec *codec = codec_dai->codec;
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);
        u16 reg;

        dev_dbg(codec->dev, "%s, pll_id %d, freq_in %d, frq_out %d\n", __FUNCTION__, pll_id,freq_in,freq_out);

        if(freq_in == 0 || freq_out == 0) {

                if( (atomic_read(&wau8822->substream) > 1)
			&& atomic_read(&wau8822->pll_set) ){
                	/* One DAI has allready been initialized */
                        return 0;
                }

		// Unset pll
        	atomic_dec(&wau8822->pll_set);

                /* Clock CODEC directly from MCLK */
                reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK);
                wau8822_write(codec, WAU8822_CLOCK, reg & 0x0ff);

                /* Turn off PLL */
                reg = wau8822_read_reg_cache(codec, WAU8822_POWER1);
                wau8822_write(codec, WAU8822_POWER1, reg & 0x1df);
                return 0;
        }

        if( atomic_read(&wau8822->pll_set) ){
                /* One DAI has allready been initialized */
                return 0;
        }/*else*/


        atomic_inc(&wau8822->pll_set);
        pll_factors(freq_out*8, freq_in);

        wau8822_write(codec, WAU8822_PLLN, (pll_div.pre_div << 4) | pll_div.n);
        wau8822_write(codec, WAU8822_PLLK1, pll_div.k >> 18);
        wau8822_write(codec, WAU8822_PLLK2, (pll_div.k >> 9) & 0x1ff);
        wau8822_write(codec, WAU8822_PLLK3, pll_div.k & 0x1ff);

        reg = wau8822_read_reg_cache(codec, WAU8822_POWER1);
        wau8822_write(codec, WAU8822_POWER1, reg | 0x020);

        /* Run CODEC from PLL instead of MCLK */
        reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x11f;
        wau8822_write(codec, WAU8822_CLOCK, reg | 0x100);

        /* MATT: adapt the MCLK */
        reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK);
        wau8822_write(codec, WAU8822_CLOCK, reg | (0x020<<pll_div.pre_div));

        /* MATT to try to put the out in aiz */
        wau8822_write(codec,0x3c,0x0);

        /* MATT: to adapt bclk - McBSP driver currently only support 16bits data  */
        reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x1e3;

        /* MATT: if divided by 8 -> 16BCLK per channel
             if divided by 4 -> 32BCLK per channel */
//       wau8822_write(codec, WAU8822_CLOCK, reg | WAU8822_BCLKDIV_8);
        wau8822_write(codec, WAU8822_CLOCK, reg | WAU8822_BCLKDIV_4);

         return 0;
}

/*
 * Configure WAU8822 clock dividers.
 */
static int wau8822_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
                int div_id, int div)
{
        struct snd_soc_codec *codec = codec_dai->codec;
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);
        u16 reg;

        dev_dbg(codec->dev, "%s : div_id %d, div %d \n", __FUNCTION__,  div_id,div);

        if( atomic_read(&wau8822->dai_clkdiv_set) )
                return 0; /* One DAI has allready been initialized */

	/* Initialization */
        atomic_inc(&wau8822->dai_clkdiv_set);

        switch (div_id) {
        case WAU8822_OPCLKDIV:
                reg = wau8822_read_reg_cache(codec, WAU8822_GPIO) & 0x1cf;
                wau8822_write(codec, WAU8822_GPIO, reg | div);
                break;
        case WAU8822_MCLKDIV:
                reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x11f;
                wau8822_write(codec, WAU8822_CLOCK, reg | div);
                break;
        case WAU8822_ADCCLK:
                reg = wau8822_read_reg_cache(codec, WAU8822_ADC) & 0x1f7;
                wau8822_write(codec, WAU8822_ADC, reg | div);
                break;
        case WAU8822_DACCLK:
                reg = wau8822_read_reg_cache(codec, WAU8822_DAC) & 0x1f7;
                wau8822_write(codec, WAU8822_DAC, reg | div);
                break;
        case WAU8822_BCLKDIV:
                reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x1e3;
                wau8822_write(codec, WAU8822_CLOCK, reg | div);
                break;
        default:
        return -EINVAL;
        }

        return 0;
}

static int wau8822_set_dai_fmt(struct snd_soc_dai *codec_dai,
    unsigned int fmt)
{
        struct snd_soc_codec *codec = codec_dai->codec;
        u16 iface = 0;
        u16 clk = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x1fe;

        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

        dev_dbg(codec->dev, "%s\n", __FUNCTION__);

        /* set master/slave audio interface */
        switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
                case SND_SOC_DAIFMT_CBM_CFM:
                        clk |= 0x0001;
                break;

                case SND_SOC_DAIFMT_CBS_CFS:
                break;

                default:
                return -EINVAL;
        }

        /* interface format */
        switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
                case SND_SOC_DAIFMT_I2S:
                        iface |= 0x0010;
                break;

                case SND_SOC_DAIFMT_RIGHT_J:
                break;

                case SND_SOC_DAIFMT_LEFT_J:
                        iface |= 0x0008;
                break;

                case SND_SOC_DAIFMT_DSP_A:
                        iface |= 0x00018;
                break;

                default:
                return -EINVAL;
        }

        /* clock inversion */
        switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
        case SND_SOC_DAIFMT_NB_NF:
        break;

        case SND_SOC_DAIFMT_IB_IF:
                iface |= 0x0180;
        break;

        case SND_SOC_DAIFMT_IB_NF:
                iface |= 0x0100;
        break;

        case SND_SOC_DAIFMT_NB_IF:
                iface |= 0x0080;
        break;

        default:
        return -EINVAL;
        }

	if( atomic_read(&wau8822->fmt_set) ){
                if( ((wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0x198)
			!= (iface & 0x198))
                    || ((wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x1) 
			!= (clk & 0x1)) ){

			dev_err(codec->dev,  "%s : fmt cannot be set\n", __func__);
                        return -EINVAL;
                }

		dev_dbg(codec->dev, "%s : fmt allready set\n", __func__);
                return 0;
        }

	atomic_inc(&wau8822->fmt_set);
        wau8822_write(codec, WAU8822_IFACE, iface);
        wau8822_write(codec, WAU8822_CLOCK, clk);
  return 0;
}

static int wau8822_pcm_hw_params(struct snd_pcm_substream *substream,
                                 struct snd_pcm_hw_params *params,
                                 struct snd_soc_dai *dai)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_codec *codec = rtd->codec;
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

        u16 iface = wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0x19f;
        u16 adn = wau8822_read_reg_cache(codec, WAU8822_ADD) & 0x1f1;

        dev_dbg(codec->dev, "%s: rate=>%d format=>%d\n", __FUNCTION__, params_rate(params), params_format(params));

        /* bit size */
        switch (params_format(params)) {
                case SNDRV_PCM_FORMAT_S16_LE:
                break;
                case SNDRV_PCM_FORMAT_S20_3LE:
                        iface |= 0x0020;
                break;
                case SNDRV_PCM_FORMAT_S24_LE:
                        iface |= 0x0040;
                break;
                case SNDRV_PCM_FORMAT_S32_LE:
                        iface |= 0x0060;
                break;
        }

        /* filter coefficient */
        switch (params_rate(params)) {
                case SNDRV_PCM_RATE_8000:
                        adn |= 0x5 << 1;
                break;
                case SNDRV_PCM_RATE_11025:
                        adn |= 0x4 << 1;
                break;
                case SNDRV_PCM_RATE_16000:
                        adn |= 0x3 << 1;
                break;
                case SNDRV_PCM_RATE_22050:
                        adn |= 0x2 << 1;
                break;
                case SNDRV_PCM_RATE_32000:
                        adn |= 0x1 << 1;
                break;
                case SNDRV_PCM_RATE_44100:
                case SNDRV_PCM_RATE_48000:
                break;
        }

	if( atomic_read(&wau8822->hw_params_set) ){
        /* One DAI has allready been initialized */

                if( ((wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0x70)
			!= (iface & 0x70) )
                    || ((wau8822_read_reg_cache(codec, WAU8822_ADD) & 0xA)
			!= (adn & 0xA)  ) ){

			dev_err(codec->dev,  "%s : pcm hardware cannot be set\n", __func__);
                        return -EINVAL;
                }

		dev_dbg(codec->dev, "%s : pcm hardware allready set\n", __func__);
                return 0;
        }/*else*/

	atomic_inc(&wau8822->hw_params_set);
        wau8822_write(codec, WAU8822_IFACE, iface);
        wau8822_write(codec, WAU8822_ADD, adn);
        return 0;
}

static int wau8822_mute(struct snd_soc_dai *codec_dai, int mute)
{
        struct snd_soc_codec *codec = codec_dai->codec;
        u16 mute_reg = wau8822_read_reg_cache(codec, WAU8822_DAC) & 0xffbf;

        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

        dev_dbg(codec->dev, "%s : muting/unmuting %d \n",__FUNCTION__,  mute);

        /* Only on DAI in use */
        if(mute) {
        	if( (atomic_read(&wau8822->substream) == 1) 
			&& atomic_read(&wau8822->mute_set) ){

        		atomic_dec(&wau8822->mute_set);
                	wau8822_write(codec, WAU8822_DAC, mute_reg | 0x40);
		}
        }
        else {
        	if( atomic_read(&wau8822->substream) 
			&& atomic_read(&wau8822->mute_set) ){

        		atomic_dec(&wau8822->mute_set);
                	wau8822_write(codec, WAU8822_DAC, mute_reg);
                	Wau8822DumpRegister(codec);
		}
        }

        return 0;
}

static int wau8822_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
        struct snd_soc_codec *codec = dai->codec;
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

        dev_dbg(codec->dev, "%s\n",__func__);

        if( atomic_read(&wau8822->substream) > MAX_SUBSTREAM )
                return -EINVAL;

        atomic_inc(&wau8822->substream);

        return 0;
}

static void wau8822_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
        struct snd_soc_codec *codec = dai->codec;
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

        dev_dbg(codec->dev, "%s\n",__func__);

        if( atomic_read(&wau8822->substream) > MAX_SUBSTREAM )
                return;

        if (atomic_read(&wau8822->substream))
        	atomic_dec(&wau8822->substream);

        if( !atomic_read(&wau8822->substream) ){
		if (atomic_read(&wau8822->fmt_set))
                	atomic_dec(&wau8822->fmt_set);

		if (atomic_read(&wau8822->dai_clkdiv_set))
        		atomic_dec(&wau8822->dai_clkdiv_set);

		if (atomic_read(&wau8822->hw_params_set))
        		atomic_dec(&wau8822->hw_params_set);
        }

}

#define WAU8822_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
                SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define WAU8822_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
        SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops wau8822_dai_ops_playback = {
        .startup      = wau8822_startup,
        .shutdown     = wau8822_shutdown,
        .hw_params    = wau8822_pcm_hw_params,
        .set_fmt      = wau8822_set_dai_fmt,
        .set_clkdiv   = wau8822_set_dai_clkdiv,
        .set_pll      = wau8822_set_dai_pll,
        .digital_mute = wau8822_mute,
};

static struct snd_soc_dai_driver wau8822_dai[NO_MORE_CODEC] = {
       {
                .name     = "wau8822-hifi",
                .id          = MASTER_I2S_CODEC,
                .playback = {
                        .stream_name  = "Playback",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates        = WAU8822_RATES,
                        .formats      = WAU8822_FORMATS,
                },
                .capture = {
                        .stream_name  = "Capture",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates        = WAU8822_RATES,
                        .formats      = WAU8822_FORMATS,
                 },
                .ops = &wau8822_dai_ops_playback,
       },
/* I2S Slave codecs */
       {
                .name     = "codc_48k_S16LE",
                .id          = SND_I2S_CODEC,
                .playback = {
                        .stream_name  = "Playback",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates        = WAU8822_RATES,
                        .formats      = WAU8822_FORMATS,
                },
                .capture = {
                        .stream_name  = "Capture",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates        = WAU8822_RATES,
                        .formats      = WAU8822_FORMATS,
                 },
                .ops = &wau8822_dai_ops_playback,
       },
       {
                .name     = "codc2_48k_S16LE",
                .id          = TRD_I2S_CODEC,
                .playback = {
                        .stream_name  = "Playback",
                        .channels_min = 2,
                        .channels_max = 2,
                        .rates        = WAU8822_RATES,
                        .formats      = WAU8822_FORMATS,
                },
                .capture = { 0 },
                .ops = &wau8822_dai_ops_playback,
       },
};

/* liam need to make this lower power with dapm */
static int wau8822_set_bias_level(struct snd_soc_codec *codec,
  enum snd_soc_bias_level level)
{
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);
        u16 power1 = wau8822_read_reg_cache(codec, WAU8822_POWER1) & ~0x3;

        dev_dbg(codec->dev, "%s level %d \n", __FUNCTION__, level);

        if( atomic_read(&wau8822->substream)
		&& atomic_read(&wau8822->bias_level_set) ){
                codec->dapm->bias_level = level;
                return 0;
        }

        switch (level) {
        case SND_SOC_BIAS_ON:
        case SND_SOC_BIAS_PREPARE:

        	if( atomic_read(&wau8822->substream)
			&& !atomic_read(&wau8822->bias_level_set) ){

			atomic_inc(&wau8822->bias_level_set);

                	power1 |= 0x1;  /* VMID 50k */
                	wau8822_write(codec, WAU8822_POWER1, power1);
		}
                break;

        case SND_SOC_BIAS_STANDBY:

        	if( (atomic_read(&wau8822->substream) == 1)
			&& atomic_read(&wau8822->bias_level_set) ){

			atomic_dec(&wau8822->bias_level_set);

                	power1 |= WAU8822_POWER1_BIASEN | WAU8822_POWER1_BUFIOEN;

	                if (codec->dapm->bias_level == SND_SOC_BIAS_OFF) {
        	                /* Initial cap charge at VMID 5k */
                	        wau8822_write(codec, WAU8822_POWER1, power1 | 0x3);
                        	mdelay(100);
                	}

                	power1 |= 0x2;  /* VMID 500k */
                	wau8822_write(codec, WAU8822_POWER1, power1);
		}
                break;

        case SND_SOC_BIAS_OFF:

        	if( (atomic_read(&wau8822->substream) == 1)
			&& atomic_read(&wau8822->bias_level_set) ){

                	wau8822_write(codec, WAU8822_POWER1, 0);
                	wau8822_write(codec, WAU8822_POWER2, 0);
                	wau8822_write(codec, WAU8822_POWER3, 0);
		}
                break;
        }

        codec->dapm->bias_level = level;
        return 0;
}

static int wau8822_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
        /* we only need to suspend if we are a valid card */
        if(!codec->card)
                return 0;

        wau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
        return 0;
}

static int wau8822_resume(struct snd_soc_codec *codec )
{
        int i;
        u8 data[2];
        u16 *cache = codec->reg_cache;

        /* we only need to resume if we are a valid card */
        if(!codec->card)
                return 0;
        /* Sync reg_cache with the hardware */

        for (i = 0; i < ARRAY_SIZE(wau8822_reg); i++) {
                if (i == WAU8822_RESET)
                        continue;
                data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
                data[1] = cache[i] & 0x00ff;
                wau8822_write(codec->control_data, data, 2);
        }
        wau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
            wau8822_set_bias_level(codec, codec->dapm->suspend_bias_level);
        return 0;
}

static int wau8822_probe(struct snd_soc_codec *codec)
{
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);
        int ret = 0, i;
        unsigned int value = 0;

        dev_dbg(codec->dev, "%s\n", __FUNCTION__);
        dev_dbg(codec->dev,"WAU8822 Audio Codec %s",WAU8822_VERSION);

        mutex_init(&codec->mutex);

        codec->control_data = wau8822->control_data;
        ret = snd_soc_codec_set_cache_io(codec, 7, 9, wau8822->control_type);

        if (ret < 0) {
                dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
                return ret;
        }

        /* Reset  */
        ret = wau8822_reset(codec);
        if (ret != 0) {
                dev_err(codec->dev, "Failed to reset codec: %d\n", ret);
                goto err_init;
        }

        /* power on device */
        codec->dapm->bias_level = SND_SOC_BIAS_OFF;
        wau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

        /* initialize codec register and update cache */
        dev_dbg(codec->dev, "initialize codec\n");
        for(i=0;i<SET_CODEC_REG_INIT_NUM;i++)
        {
                wau8822_write(codec, Set_Codec_Reg_Init[i][0],Set_Codec_Reg_Init[i][1]);
        }
        /* update cache */
        for(i=0;i<WAU8822_CACHEREGNUM;i++)
        {
                value = snd_soc_read(codec, i);
                wau8822_write_reg_cache(codec, i, value);
        }

        /* init widgets and controls */
        dev_dbg(codec->dev, "init widgets and controls\n");
        snd_soc_add_controls(codec, wau8822_snd_controls,
                              ARRAY_SIZE(wau8822_snd_controls));

        //snd_soc_dapm_new_controls(codec->dapm, wau8822_dapm_widgets, 
        //                            ARRAY_SIZE(wau8822_dapm_widgets));

        //snd_soc_dapm_add_routes(codec->dapm, audio_map, ARRAY_SIZE(audio_map));

err_init:
        return ret;
}

/* power down chip */
static int wau8822_remove(struct snd_soc_codec * codec)
{
        struct wau8822_priv *wau8822 = snd_soc_codec_get_drvdata(codec);

        dev_dbg(codec->dev, "%s\n", __FUNCTION__);

        wau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);

        kfree(wau8822);
        return 0;
}

struct snd_soc_codec_driver soc_codec_dev_wau8822 = {
        .probe             = wau8822_probe,
        .remove            = wau8822_remove,
        .suspend           = wau8822_suspend,
        .resume            = wau8822_resume,
        .set_bias_level    = wau8822_set_bias_level,
        .reg_cache_size    = sizeof(wau8822_reg),
        .reg_word_size     = sizeof(u16),
        .reg_cache_default = &wau8822_reg,
};
//EXPORT_SYMBOL_GPL(soc_codec_dev_wau8822);

///////////////////////////////////////////////////////

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

static int wau8822_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
        struct wau8822_priv *wau8822;
        int ret;

        pr_debug(AUDIO_NAME ": %s\n", __FUNCTION__ );
        wau8822 = kzalloc(sizeof(struct wau8822_priv), GFP_KERNEL);
        if (wau8822 == NULL)
                return -ENOMEM;

        i2c_set_clientdata(i2c, wau8822);
        wau8822->control_data = i2c;
        wau8822->control_type = SND_SOC_I2C;

        atomic_set(&wau8822->substream, 0);
        atomic_set(&wau8822->fmt_set, 0);
        atomic_set(&wau8822->pll_set, 0);
        atomic_set(&wau8822->dai_clkdiv_set, 0);
        atomic_set(&wau8822->hw_params_set, 0);
        atomic_set(&wau8822->bias_level_set, 0);
        atomic_set(&wau8822->mute_set, 1);


        ret =  snd_soc_register_codec(&i2c->dev,
                        &soc_codec_dev_wau8822, wau8822_dai, ARRAY_SIZE(wau8822_dai));

        if (ret < 0){
                kfree(wau8822);
                pr_err(AUDIO_NAME ": %s failed: ret = %d\n",
                                       __FUNCTION__, ret);
        }

        return ret;
}

static int wau8822_i2c_remove(struct i2c_client *client)
{
        struct snd_soc_codec *codec = i2c_get_clientdata(client);

        pr_debug(AUDIO_NAME ": %s\n", __FUNCTION__ );
        kfree(codec->reg_cache);
        kfree(client);
        return 0;
}


/*
 * WAU8822 2 wire address is determined by A1 pin
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
static const struct i2c_device_id wau8822_i2c_table[] = {
        {"wau8822", 0},
        {}
};

MODULE_DEVICE_TABLE(i2c, wau8822_i2c_table);

/*  i2c codec control layer */
static struct i2c_driver wau8822_i2c_driver = {
        .driver = {
                .name  = "wau8822-codec",
                .owner = THIS_MODULE,
        },
        .probe    = wau8822_i2c_probe,
        .remove   = wau8822_i2c_remove,
        .id_table = wau8822_i2c_table,
};


#endif

static int __init wau8822_modinit(void)
{
        int ret = 0;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
        pr_debug(AUDIO_NAME ": module init \n");
        ret = i2c_add_driver(&wau8822_i2c_driver);
        if (ret != 0) {
                pr_err("Failed to register WM8580 I2C driver: %d\n", ret);
        }
#endif

        return ret;
}
module_init(wau8822_modinit);

static void __exit wau8822_modexit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
        pr_debug(AUDIO_NAME ": module exit \n");
        i2c_del_driver(&wau8822_i2c_driver);
#endif
}
module_exit(wau8822_modexit);

MODULE_DESCRIPTION("ASoC WAU8822 driver");
MODULE_AUTHOR("flove , Ethan");
MODULE_LICENSE("GPL");
