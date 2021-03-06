/*
 * nau8822.c  --  NAU8822 ALSA SoC Audio Codec driver
 *
 * cfyang3 for nau8822
 * Copyright (C) 2009-2010 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 * Copyright (C) 2007 Carlos Munoz <carlos@kenati.com>
 * Copyright 2006-2009 Wolfson Microelectronics PLC.
 * Based on wm8974 and wm8990 by Liam Girdwood <lrg@slimlogic.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
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
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include "nau8822.h"

/* nau8822 register cache. Note that register 0 is not included in the cache. */
static const u16 nau8822_reg[NAU8822_CACHEREGNUM] = {
    0x0000, 0x01dd, 0x0000, 0x01ff,    /* 0x00...0x03 */
    0x0012, 0x0000, 0x0140, 0x0000,    /* 0x04...0x07 */
    0x0000, 0x0000, 0x000c, 0x01ff,    /* 0x08...0x0b */
    0x01ff, 0x0000, 0x0108, 0x01ff,    /* 0x0c...0x0f */
    0x01ff, 0x0006, 0x0128, 0x0028,    /* 0x10...0x13 */
    0x0029, 0x0028, 0x0028, 0x0000,    /* 0x14...0x17 */
    0x0032, 0x0000, 0x0100, 0x0100,    /* 0x18...0x1b */
    0x0100, 0x0100, 0x0100, 0x0000,    /* 0x1c...0x1f */
    0x0038, 0x000b, 0x0132, 0x0010,    /* 0x20...0x23 */
    0x0008, 0x000c, 0x0093, 0x00e9,    /* 0x24...0x27 */
    0x0000, 0x0000, 0x0000, 0x0010,    /* 0x28...0x2b */
    0x0030, 0x0178, 0x0178, 0x0100,    /* 0x2c...0x2f */
    0x0100, 0x0043, 0x001d, 0x0000,    /* 0x30...0x33 */
    0x0165, 0x0165, 0x003f, 0x003f,    /* 0x34...0x37 */
    0x0140, 0x0112, 0x0000, 0x0000,    /* 0x38...0x3b */
    0x0020, 0x0020, 0x00fd,            /* 0x3c...0x3f */
};

static u16 Set_Codec_Reg_Init[][2]={
#if 1 //AUXIN 2 AUXOUT
//[aux2aux/Power Management] 
{0x1, 0x1dd},
{0x2, 0x0},
{0x3, 0x1ff},
//[aux2aux/Audio Control] 
//{0x4, 0x50},
{0x4, 0x12},
{0x5, 0x0},
{0x6, 0x0},
{0x7, 0x0},
{0x8, 0x0},
{0x9, 0x0},
{0xa, 0xc},
{0xb, 0x1ff},
{0xc, 0x1ff},
{0xd, 0x0},
{0xe, 0x108},
{0xf, 0x1ff},
{0x10, 0x1ff},
{0x11, 0x6},
//[EXT_I2S/Equalizer]     
{0x12, 0x128},
{0x13, 0x28},
{0x14, 0x29},
{0x15, 0x28},
{0x16, 0x28},
//[EXT_I2S/DAC Limiter] 
{0x18, 0x32},
{0x19, 0x0},
//[EXT_I2S/Notch Filter]     
{0x1b, 0x100},
{0x1c, 0x100},
{0x1d, 0x100},
{0x1e, 0x100},
//[EXT_I2S/ALC Control] 
{0x20, 0x38},
{0x21, 0xb},
{0x22, 0x132},
{0x23, 0x10},
//[EXT_I2S/PLL Control] 
{0x24, 0x8},
{0x25, 0xc},
{0x26, 0x93},
{0x27, 0xe9},
//[EXT_I2S/Mic Bias Mode]     
{0x28, 0x0},
//[EXT_I2S/3D Control]     
{0x29, 0x0},
//[EXT_I2S/Input Output Mixer]
{0x2b, 0x10},
{0x2c, 0x30},
//{0x2d, 0x150},
{0x2d, 0x178},
//{0x2e, 0x110},
{0x2e, 0x178},
{0x2f, 0x100},
{0x30, 0x100},
{0x31, 0x43},
//{0x32, 0x1},
{0x32, 0x1D},
{0x33, 0x0},
{0x34, 0x165},
{0x35, 0x165},
{0x36, 0x3f},
{0x37, 0x3f},
{0x38, 0x140},
{0x39, 0x112},
{0x3a, 0x0},
//[EXT_I2S/PCM Timeslot] 
{0x3b, 0x0},
{0x3c, 0x20},
{0x3d, 0x20},
{0x3e, 0xfd}
#endif
};
#define SET_CODEC_REG_INIT_NUM  ARRAY_SIZE(Set_Codec_Reg_Init)

/* codec private data */
struct nau8822_priv {
    enum snd_soc_control_type control_type;
    void *control_data;
    unsigned int f_pllout;
    unsigned int f_mclk;
    unsigned int f_256fs;
    unsigned int f_opclk;
    int mclk_idx;
    enum nau8822_sysclk_src sysclk;
};

static const char *nau8822_companding[] = {"Off", "NC", "u-law", "A-law"};
static const char *nau8822_eqmode[] = {"Capture", "Playback"};
static const char *nau8822_bw[] = {"Narrow", "Wide"};
static const char *nau8822_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz"};
static const char *nau8822_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz"};
static const char *nau8822_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz"};
static const char *nau8822_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz"};
static const char *nau8822_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz"};
static const char *nau8822_alc3[] = {"ALC", "Limiter"};
static const char *nau8822_alc1[] = {"Off", "Right", "Left", "Both"};

static const SOC_ENUM_SINGLE_DECL(adc_compand, NAU8822_COMPANDING_CONTROL, 1,
                                  nau8822_companding);
static const SOC_ENUM_SINGLE_DECL(dac_compand, NAU8822_COMPANDING_CONTROL, 3,
                                  nau8822_companding);
static const SOC_ENUM_SINGLE_DECL(eqmode, NAU8822_EQ1, 8, nau8822_eqmode);
static const SOC_ENUM_SINGLE_DECL(eq1, NAU8822_EQ1, 5, nau8822_eq1);
static const SOC_ENUM_SINGLE_DECL(eq2bw, NAU8822_EQ2, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq2, NAU8822_EQ2, 5, nau8822_eq2);
static const SOC_ENUM_SINGLE_DECL(eq3bw, NAU8822_EQ3, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq3, NAU8822_EQ3, 5, nau8822_eq3);
static const SOC_ENUM_SINGLE_DECL(eq4bw, NAU8822_EQ4, 8, nau8822_bw);
static const SOC_ENUM_SINGLE_DECL(eq4, NAU8822_EQ4, 5, nau8822_eq4);
static const SOC_ENUM_SINGLE_DECL(eq5, NAU8822_EQ5, 5, nau8822_eq5);
static const SOC_ENUM_SINGLE_DECL(alc3, NAU8822_ALC_CONTROL_3, 8, nau8822_alc3);
static const SOC_ENUM_SINGLE_DECL(alc1, NAU8822_ALC_CONTROL_1, 7, nau8822_alc1);

static const DECLARE_TLV_DB_SCALE(digital_tlv, -12750, 50, 1);
static const DECLARE_TLV_DB_SCALE(eq_tlv, -1200, 100, 0);
static const DECLARE_TLV_DB_SCALE(inpga_tlv, -1200, 75, 0);
static const DECLARE_TLV_DB_SCALE(spk_tlv, -5700, 100, 0);
static const DECLARE_TLV_DB_SCALE(boost_tlv, -1500, 300, 1);
static const DECLARE_TLV_DB_SCALE(limiter_tlv, 0, 100, 0);

static const struct snd_kcontrol_new nau8822_snd_controls[] = {

    SOC_SINGLE("Digital Loopback Switch",
               NAU8822_COMPANDING_CONTROL, 0, 1, 0),

    SOC_ENUM("ADC Companding", adc_compand),
    SOC_ENUM("DAC Companding", dac_compand),

    SOC_DOUBLE("DAC Inversion Switch", NAU8822_DAC_CONTROL, 0, 1, 1, 0),

    SOC_DOUBLE_R_TLV("PCM Volume",
                     NAU8822_LEFT_DAC_DIGITAL_VOLUME, NAU8822_RIGHT_DAC_DIGITAL_VOLUME,
                     0, 255, 0, digital_tlv),

    SOC_SINGLE("High Pass Filter Switch", NAU8822_ADC_CONTROL, 8, 1, 0),
    SOC_SINGLE("High Pass Cut Off", NAU8822_ADC_CONTROL, 4, 7, 0),
    SOC_DOUBLE("ADC Inversion Switch", NAU8822_ADC_CONTROL, 0, 1, 1, 0),

    SOC_DOUBLE_R_TLV("ADC Volume",
                     NAU8822_LEFT_ADC_DIGITAL_VOLUME, NAU8822_RIGHT_ADC_DIGITAL_VOLUME,
                     0, 255, 0, digital_tlv),

    SOC_ENUM("Equaliser Function", eqmode),
    SOC_ENUM("EQ1 Cut Off", eq1),
    SOC_SINGLE_TLV("EQ1 Volume", NAU8822_EQ1,  0, 24, 1, eq_tlv),

    SOC_ENUM("Equaliser EQ2 Bandwith", eq2bw),
    SOC_ENUM("EQ2 Cut Off", eq2),
    SOC_SINGLE_TLV("EQ2 Volume", NAU8822_EQ2,  0, 24, 1, eq_tlv),

    SOC_ENUM("Equaliser EQ3 Bandwith", eq3bw),
    SOC_ENUM("EQ3 Cut Off", eq3),
    SOC_SINGLE_TLV("EQ3 Volume", NAU8822_EQ3,  0, 24, 1, eq_tlv),

    SOC_ENUM("Equaliser EQ4 Bandwith", eq4bw),
    SOC_ENUM("EQ4 Cut Off", eq4),
    SOC_SINGLE_TLV("EQ4 Volume", NAU8822_EQ4,  0, 24, 1, eq_tlv),

    SOC_ENUM("EQ5 Cut Off", eq5),
    SOC_SINGLE_TLV("EQ5 Volume", NAU8822_EQ5, 0, 24, 1, eq_tlv),

    SOC_SINGLE("DAC Playback Limiter Switch",
               NAU8822_DAC_LIMITER_1, 8, 1, 0),
    SOC_SINGLE("DAC Playback Limiter Decay",
               NAU8822_DAC_LIMITER_1, 4, 15, 0),
    SOC_SINGLE("DAC Playback Limiter Attack",
               NAU8822_DAC_LIMITER_1, 0, 15, 0),

    SOC_SINGLE("DAC Playback Limiter Threshold",
               NAU8822_DAC_LIMITER_2, 4, 7, 0),
    SOC_SINGLE_TLV("DAC Playback Limiter Volume",
                   NAU8822_DAC_LIMITER_2, 0, 12, 0, limiter_tlv),

    SOC_ENUM("ALC Enable Switch", alc1),
    SOC_SINGLE("ALC Capture Min Gain", NAU8822_ALC_CONTROL_1, 0, 7, 0),
    SOC_SINGLE("ALC Capture Max Gain", NAU8822_ALC_CONTROL_1, 3, 7, 0),

    SOC_SINGLE("ALC Capture Hold", NAU8822_ALC_CONTROL_2, 4, 10, 0),
    SOC_SINGLE("ALC Capture Target", NAU8822_ALC_CONTROL_2, 0, 15, 0),

    SOC_ENUM("ALC Capture Mode", alc3),
    SOC_SINGLE("ALC Capture Decay", NAU8822_ALC_CONTROL_3, 4, 10, 0),
    SOC_SINGLE("ALC Capture Attack", NAU8822_ALC_CONTROL_3, 0, 10, 0),

    SOC_SINGLE("ALC Capture Noise Gate Switch", NAU8822_NOISE_GATE, 3, 1, 0),
    SOC_SINGLE("ALC Capture Noise Gate Threshold",
               NAU8822_NOISE_GATE, 0, 7, 0),

    SOC_DOUBLE_R("Capture PGA ZC Switch",
                 NAU8822_LEFT_INP_PGA_CONTROL, NAU8822_RIGHT_INP_PGA_CONTROL,
                 7, 1, 0),

    /* OUT1 - Headphones */
    SOC_DOUBLE_R("Headphone Playback ZC Switch",
                 NAU8822_LOUT1_HP_CONTROL, NAU8822_ROUT1_HP_CONTROL, 7, 1, 0),

    SOC_DOUBLE_R_TLV("Headphone Playback Volume",
                     NAU8822_LOUT1_HP_CONTROL, NAU8822_ROUT1_HP_CONTROL,
                     0, 63, 0, spk_tlv),

    /* OUT2 - Speakers */
    SOC_DOUBLE_R("Speaker Playback ZC Switch",
                 NAU8822_LOUT2_SPK_CONTROL, NAU8822_ROUT2_SPK_CONTROL, 7, 1, 0),

    SOC_DOUBLE_R_TLV("Speaker Playback Volume",
                     NAU8822_LOUT2_SPK_CONTROL, NAU8822_ROUT2_SPK_CONTROL,
                     0, 63, 0, spk_tlv),

    /* OUT3/4 - Line Output */
    SOC_DOUBLE_R("Line Playback Switch",
                 NAU8822_OUT3_MIXER_CONTROL, NAU8822_OUT4_MIXER_CONTROL, 6, 1, 1),

    /* Mixer #3: Boost (Input) mixer */
    SOC_DOUBLE_R("PGA Boost (+20dB)",
                 NAU8822_LEFT_ADC_BOOST_CONTROL, NAU8822_RIGHT_ADC_BOOST_CONTROL,
                 8, 1, 0),
    SOC_DOUBLE_R_TLV("L2/R2 Boost Volume",
                     NAU8822_LEFT_ADC_BOOST_CONTROL, NAU8822_RIGHT_ADC_BOOST_CONTROL,
                     4, 7, 0, boost_tlv),
    SOC_DOUBLE_R_TLV("Aux Boost Volume",
                     NAU8822_LEFT_ADC_BOOST_CONTROL, NAU8822_RIGHT_ADC_BOOST_CONTROL,
                     0, 7, 0, boost_tlv),

    /* Input PGA volume */
    SOC_DOUBLE_R_TLV("Input PGA Volume",
                     NAU8822_LEFT_INP_PGA_CONTROL, NAU8822_RIGHT_INP_PGA_CONTROL,
                     0, 63, 0, inpga_tlv),

    /* Headphone */
    SOC_DOUBLE_R("Headphone Switch",
                 NAU8822_LOUT1_HP_CONTROL, NAU8822_ROUT1_HP_CONTROL, 6, 1, 1),

    /* Speaker */
    SOC_DOUBLE_R("Speaker Switch",
                 NAU8822_LOUT2_SPK_CONTROL, NAU8822_ROUT2_SPK_CONTROL, 6, 1, 1),

    /* DAC / ADC oversampling */
    SOC_SINGLE("DAC 128x Oversampling Switch", NAU8822_DAC_CONTROL,
               5, 1, 0),
    SOC_SINGLE("ADC 128x Oversampling Switch", NAU8822_ADC_CONTROL,
               5, 1, 0),
};

/* Mixer #1: Output (OUT1, OUT2) Mixer: mix AUX, Input mixer output and DAC */
static const struct snd_kcontrol_new nau8822_left_out_mixer[] = {
    SOC_DAPM_SINGLE("Line Bypass Switch", NAU8822_LEFT_MIXER_CONTROL, 1, 1, 0),
    SOC_DAPM_SINGLE("Aux Playback Switch", NAU8822_LEFT_MIXER_CONTROL, 5, 1, 0),
    SOC_DAPM_SINGLE("PCM Playback Switch", NAU8822_LEFT_MIXER_CONTROL, 0, 1, 0),
};

static const struct snd_kcontrol_new nau8822_right_out_mixer[] = {
    SOC_DAPM_SINGLE("Line Bypass Switch", NAU8822_RIGHT_MIXER_CONTROL, 1, 1, 0),
    SOC_DAPM_SINGLE("Aux Playback Switch", NAU8822_RIGHT_MIXER_CONTROL, 5, 1, 0),
    SOC_DAPM_SINGLE("PCM Playback Switch", NAU8822_RIGHT_MIXER_CONTROL, 0, 1, 0),
};

/* OUT3/OUT4 Mixer not implemented */
static const struct snd_kcontrol_new nau8822_aux1_mixer[] = {
    SOC_DAPM_SINGLE("RDAC Switch", NAU8822_OUT4_MIXER_CONTROL, 0, 1, 0),
    SOC_DAPM_SINGLE("RMIX Switch", NAU8822_OUT4_MIXER_CONTROL, 1, 1, 0),
    SOC_DAPM_SINGLE("RADC Switch", NAU8822_OUT4_MIXER_CONTROL, 2, 1, 0),
    SOC_DAPM_SINGLE("LDAC Switch", NAU8822_OUT4_MIXER_CONTROL, 3, 1, 0),
    SOC_DAPM_SINGLE("LMIX Switch", NAU8822_OUT4_MIXER_CONTROL, 4, 1, 0),
};

static const struct snd_kcontrol_new nau8822_aux2_mixer[] = {
    SOC_DAPM_SINGLE("LDAC Switch", NAU8822_OUT3_MIXER_CONTROL, 0, 1, 0),
    SOC_DAPM_SINGLE("LMIX Switch", NAU8822_OUT3_MIXER_CONTROL, 1, 1, 0),
    SOC_DAPM_SINGLE("LADC Switch", NAU8822_OUT3_MIXER_CONTROL, 2, 1, 0),
    SOC_DAPM_SINGLE("AUX1 Switch", NAU8822_OUT3_MIXER_CONTROL, 3, 1, 0),
};

/* Mixer #2: Input PGA Mute */
static const struct snd_kcontrol_new nau8822_left_input_mixer[] = {
    SOC_DAPM_SINGLE("L2 Switch", NAU8822_INPUT_CONTROL, 2, 1, 0),
    SOC_DAPM_SINGLE("MicN Switch", NAU8822_INPUT_CONTROL, 1, 1, 0),
    SOC_DAPM_SINGLE("MicP Switch", NAU8822_INPUT_CONTROL, 0, 1, 0),
};
static const struct snd_kcontrol_new nau8822_right_input_mixer[] = {
    SOC_DAPM_SINGLE("R2 Switch", NAU8822_INPUT_CONTROL, 6, 1, 0),
    SOC_DAPM_SINGLE("MicN Switch", NAU8822_INPUT_CONTROL, 5, 1, 0),
    SOC_DAPM_SINGLE("MicP Switch", NAU8822_INPUT_CONTROL, 4, 1, 0),
};

static const struct snd_soc_dapm_widget nau8822_dapm_widgets[] = {
    SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback",
                     NAU8822_POWER_MANAGEMENT_3, 0, 0),
    SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback",
                     NAU8822_POWER_MANAGEMENT_3, 1, 0),
    SND_SOC_DAPM_ADC("Left ADC", "Left HiFi Capture",
                     NAU8822_POWER_MANAGEMENT_2, 0, 0),
    SND_SOC_DAPM_ADC("Right ADC", "Right HiFi Capture",
                     NAU8822_POWER_MANAGEMENT_2, 1, 0),

    /* Mixer #1: OUT1,2 */
    SOC_MIXER_ARRAY("Left Output Mixer", NAU8822_POWER_MANAGEMENT_3,
                    2, 0, nau8822_left_out_mixer),
    SOC_MIXER_ARRAY("Right Output Mixer", NAU8822_POWER_MANAGEMENT_3,
                    3, 0, nau8822_right_out_mixer),

    SOC_MIXER_ARRAY("Left Input Mixer", NAU8822_POWER_MANAGEMENT_2,
                    2, 0, nau8822_left_input_mixer),
    SOC_MIXER_ARRAY("Right Input Mixer", NAU8822_POWER_MANAGEMENT_2,
                    3, 0, nau8822_right_input_mixer),

    /* Mixer AUX1/AUX2 Mixer */
    SOC_MIXER_ARRAY("AUX1 Output Mixer", NAU8822_POWER_MANAGEMENT_3,
                    8, 0, nau8822_aux1_mixer),
    SOC_MIXER_ARRAY("AUX2 Output Mixer", NAU8822_POWER_MANAGEMENT_3,
                    7, 0, nau8822_aux2_mixer),

    SND_SOC_DAPM_PGA("Left Boost Mixer", NAU8822_POWER_MANAGEMENT_2,
                     4, 0, NULL, 0),
    SND_SOC_DAPM_PGA("Right Boost Mixer", NAU8822_POWER_MANAGEMENT_2,
                     5, 0, NULL, 0),

    SND_SOC_DAPM_PGA("Left Capture PGA", NAU8822_LEFT_INP_PGA_CONTROL,
                     6, 1, NULL, 0),
    SND_SOC_DAPM_PGA("Right Capture PGA", NAU8822_RIGHT_INP_PGA_CONTROL,
                     6, 1, NULL, 0),

    SND_SOC_DAPM_PGA("Left Headphone Out", NAU8822_POWER_MANAGEMENT_2,
                     7, 0, NULL, 0),
    SND_SOC_DAPM_PGA("Right Headphone Out", NAU8822_POWER_MANAGEMENT_2,
                     8, 0, NULL, 0),

    SND_SOC_DAPM_PGA("Left Speaker Out", NAU8822_POWER_MANAGEMENT_3,
                     6, 0, NULL, 0),
    SND_SOC_DAPM_PGA("Right Speaker Out", NAU8822_POWER_MANAGEMENT_3,
                     5, 0, NULL, 0),

    SND_SOC_DAPM_PGA("AUX1 Out", NAU8822_POWER_MANAGEMENT_3,
                     8, 0, NULL, 0),
    
    SND_SOC_DAPM_PGA("AUX2 Out", NAU8822_POWER_MANAGEMENT_3,
                     7, 0, NULL, 0),
    
    SND_SOC_DAPM_MIXER("OUT4 VMID", NAU8822_POWER_MANAGEMENT_3,
                       8, 0, NULL, 0),

    SND_SOC_DAPM_MICBIAS("Mic Bias", NAU8822_POWER_MANAGEMENT_1, 4, 1),

    SND_SOC_DAPM_INPUT("LMICN"),
    SND_SOC_DAPM_INPUT("LMICP"),
    SND_SOC_DAPM_INPUT("RMICN"),
    SND_SOC_DAPM_INPUT("RMICP"),
    SND_SOC_DAPM_INPUT("LAUX"),
    SND_SOC_DAPM_INPUT("RAUX"),
    SND_SOC_DAPM_INPUT("L2"),
    SND_SOC_DAPM_INPUT("R2"),
    SND_SOC_DAPM_OUTPUT("LHP"),
    SND_SOC_DAPM_OUTPUT("RHP"),
    SND_SOC_DAPM_OUTPUT("LSPK"),
    SND_SOC_DAPM_OUTPUT("RSPK"),
    SND_SOC_DAPM_OUTPUT("AUXOUT1"),
    SND_SOC_DAPM_OUTPUT("AUXOUT2"),
};

static const struct snd_soc_dapm_route nau8822_dapm_routes[] = {
    /* Output mixer */
    {"Right Output Mixer", "PCM Playback Switch", "Right DAC"},
    {"Right Output Mixer", "Aux Playback Switch", "RAUX"},
    {"Right Output Mixer", "Line Bypass Switch", "Right Boost Mixer"},

    {"Left Output Mixer", "PCM Playback Switch", "Left DAC"},
    {"Left Output Mixer", "Aux Playback Switch", "LAUX"},
    {"Left Output Mixer", "Line Bypass Switch", "Left Boost Mixer"},

    {"AUX1 Output Mixer", "RDAC Switch", "Right DAC"},
    {"AUX1 Output Mixer", "RMIX Switch", "Right Output Mixer"},
    {"AUX1 Output Mixer", "RADC Switch", "Right ADC"},
    {"AUX1 Output Mixer", "LDAC Switch", "Left DAC"},
    {"AUX1 Output Mixer", "LMIX Switch", "Left Output Mixer"},

    {"AUX2 Output Mixer", "LDAC Switch", "Left DAC"},
    {"AUX2 Output Mixer", "LMIX Switch", "Left Output Mixer"},
    {"AUX2 Output Mixer", "LADC Switch", "Left ADC"},
    {"AUX2 Output Mixer", "AUX1 Switch", "AUX1 Output Mixer"},
    
    /* Outputs */
    {"Right Headphone Out", NULL, "Right Output Mixer"},
    {"RHP", NULL, "Right Headphone Out"},

    {"Left Headphone Out", NULL, "Left Output Mixer"},
    {"LHP", NULL, "Left Headphone Out"},

    {"Right Speaker Out", NULL, "Right Output Mixer"},
    {"RSPK", NULL, "Right Speaker Out"},

    {"Left Speaker Out", NULL, "Left Output Mixer"},
    {"LSPK", NULL, "Left Speaker Out"},

    {"AUX1 Out", NULL, "AUX1 Output Mixer"},
    {"AUXOUT1", NULL, "AUX1 Out"},

    {"AUX2 Out", NULL, "AUX2 Output Mixer"},
    {"AUXOUT2", NULL, "AUX2 Out"},
    
    /* Boost Mixer */
    {"Right ADC", NULL, "Right Boost Mixer"},

    {"Right Boost Mixer", NULL, "RAUX"},
    {"Right Boost Mixer", NULL, "Right Capture PGA"},
    {"Right Boost Mixer", NULL, "R2"},

    {"Left ADC", NULL, "Left Boost Mixer"},

    {"Left Boost Mixer", NULL, "LAUX"},
    {"Left Boost Mixer", NULL, "Left Capture PGA"},
    {"Left Boost Mixer", NULL, "L2"},

    /* Input PGA */
    {"Right Capture PGA", NULL, "Right Input Mixer"},
    {"Left Capture PGA", NULL, "Left Input Mixer"},

    {"Right Input Mixer", "R2 Switch", "R2"},
    {"Right Input Mixer", "MicN Switch", "RMICN"},
    {"Right Input Mixer", "MicP Switch", "RMICP"},

    {"Left Input Mixer", "L2 Switch", "L2"},
    {"Left Input Mixer", "MicN Switch", "LMICN"},
    {"Left Input Mixer", "MicP Switch", "LMICP"},
};

/* PLL divisors */
struct nau8822_pll_div {
    u32 k;
    u8 n;
    u8 div2;
};

#define FIXED_PLL_SIZE (1 << 24)

static void pll_factors(struct snd_soc_codec *codec,
                        struct nau8822_pll_div *pll_div, unsigned int target, unsigned int source)
{
    u64 k_part;
    unsigned int k, n_div, n_mod;

    n_div = target / source;
    if (n_div < 6) {
        source >>= 1;
        pll_div->div2 = 1;
        n_div = target / source;
    } else {
        pll_div->div2 = 0;
    }

    if (n_div < 6 || n_div > 12)
        dev_warn(codec->dev,
                 "NAU8822 N value exceeds recommended range! N = %u\n",
                 n_div);

    pll_div->n = n_div;
    n_mod = target - source * n_div;
    k_part = FIXED_PLL_SIZE * (long long)n_mod + source / 2;

    do_div(k_part, source);

    k = k_part & 0xFFFFFFFF;

    pll_div->k = k;
}

/* MCLK dividers */
static const int mclk_numerator[]    = {1, 3, 2, 3, 4, 6, 8, 12};
static const int mclk_denominator[]    = {1, 2, 1, 1, 1, 1, 1, 1};

/*
 * find index >= idx, such that, for a given f_out,
 * 3 * f_mclk / 4 <= f_PLLOUT < 13 * f_mclk / 4
 * f_out can be f_256fs or f_opclk, currently only used for f_256fs. Can be
 * generalised for f_opclk with suitable coefficient arrays, but currently
 * the OPCLK divisor is calculated directly, not iteratively.
 */
static int nau8822_enum_mclk(unsigned int f_out, unsigned int f_mclk,
                             unsigned int *f_pllout)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(mclk_numerator); i++) {
        unsigned int f_pllout_x4 = 4 * f_out * mclk_numerator[i] /
            mclk_denominator[i];
        if (3 * f_mclk <= f_pllout_x4 && f_pllout_x4 < 13 * f_mclk) {
            *f_pllout = f_pllout_x4 / 4;
            return i;
        }
    }

    return -EINVAL;
}

/*
 * Calculate internal frequencies and dividers, according to Figure 40
 * "PLL and Clock Select Circuit" in NAU8822 datasheet Rev. 2.6
 */
static int nau8822_configure_pll(struct snd_soc_codec *codec)
{
    struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
    struct nau8822_pll_div pll_div;
    unsigned int f_opclk = nau8822->f_opclk, f_mclk = nau8822->f_mclk,
        f_256fs = nau8822->f_256fs;
    unsigned int f2;

    printk("f_mclk:%d f_opclk:%d\n",f_mclk,f_opclk);
    if (!f_mclk)
        return -EINVAL;

    if (f_opclk) {
        unsigned int opclk_div;
        /* Cannot set up MCLK divider now, do later */
        nau8822->mclk_idx = -1;

        /*
         * The user needs OPCLK. Choose OPCLKDIV to put
         * 6 <= R = f2 / f1 < 13, 1 <= OPCLKDIV <= 4.
         * f_opclk = f_mclk * prescale * R / 4 / OPCLKDIV, where
         * prescale = 1, or prescale = 2. Prescale is calculated inside
         * pll_factors(). We have to select f_PLLOUT, such that
         * f_mclk * 3 / 4 <= f_PLLOUT < f_mclk * 13 / 4. Must be
         * f_mclk * 3 / 16 <= f_opclk < f_mclk * 13 / 4.
         */
        if (16 * f_opclk < 3 * f_mclk || 4 * f_opclk >= 13 * f_mclk)
            return -EINVAL;

        if (4 * f_opclk < 3 * f_mclk)
            /* Have to use OPCLKDIV */
            opclk_div = (3 * f_mclk / 4 + f_opclk - 1) / f_opclk;
        else
            opclk_div = 1;

        dev_dbg(codec->dev, "%s: OPCLKDIV=%d\n", __func__, opclk_div);
        printk("%s: OPCLKDIV=%d\n", __func__, opclk_div);

        snd_soc_update_bits(codec, NAU8822_GPIO_CONTROL, 0x30,
                            (opclk_div - 1) << 4);

        nau8822->f_pllout = f_opclk * opclk_div;
    } else if (f_256fs) {
        /*
         * Not using OPCLK, but PLL is used for the codec, choose R:
         * 6 <= R = f2 / f1 < 13, to put 1 <= MCLKDIV <= 12.
         * f_256fs = f_mclk * prescale * R / 4 / MCLKDIV, where
         * prescale = 1, or prescale = 2. Prescale is calculated inside
         * pll_factors(). We have to select f_PLLOUT, such that
         * f_mclk * 3 / 4 <= f_PLLOUT < f_mclk * 13 / 4. Must be
         * f_mclk * 3 / 48 <= f_256fs < f_mclk * 13 / 4. This means MCLK
         * must be 3.781MHz <= f_MCLK <= 32.768MHz
         */
        int idx = nau8822_enum_mclk(f_256fs, f_mclk, &nau8822->f_pllout);
        if (idx < 0)
            return idx;

        nau8822->mclk_idx = idx;

        /* GPIO1 into default mode as input - before configuring PLL */
        snd_soc_update_bits(codec, NAU8822_GPIO_CONTROL, 7, 0);
    } else {
        return -EINVAL;
    }

    f2 = nau8822->f_pllout * 4;

    dev_dbg(codec->dev, "%s: f_MCLK=%uHz, f_PLLOUT=%uHz\n", __func__,
            nau8822->f_mclk, nau8822->f_pllout);

    pll_factors(codec, &pll_div, f2, nau8822->f_mclk);

    dev_dbg(codec->dev, "%s: calculated PLL N=0x%x, K=0x%x, div2=%d\n",
            __func__, pll_div.n, pll_div.k, pll_div.div2);

    /* Turn PLL off for configuration... */
    snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0);

    snd_soc_write(codec, NAU8822_PLL_N, (pll_div.div2 << 4) | pll_div.n);
    snd_soc_write(codec, NAU8822_PLL_K1, pll_div.k >> 18);
    snd_soc_write(codec, NAU8822_PLL_K2, (pll_div.k >> 9) & 0x1ff);
    snd_soc_write(codec, NAU8822_PLL_K3, pll_div.k & 0x1ff);

    /* ...and on again */
    snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0x20);

    if (f_opclk)
        /* Output PLL (OPCLK) to GPIO1 */
        snd_soc_update_bits(codec, NAU8822_GPIO_CONTROL, 7, 4);

    return 0;
}

/*
 * Configure NAU8822 clock dividers.
 */
static int nau8822_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
                                  int div_id, int div)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;

    switch (div_id) {
    case NAU8822_OPCLKRATE:
        nau8822->f_opclk = div;

        if (nau8822->f_mclk)
            /*
             * We know the MCLK frequency, the user has requested
             * OPCLK, configure the PLL based on that and start it
             * and OPCLK immediately. We will configure PLL to match
             * user-requested OPCLK frquency as good as possible.
             * In fact, it is likely, that matching the sampling
             * rate, when it becomes known, is more important, and
             * we will not be reconfiguring PLL then, because we
             * must not interrupt OPCLK. But it should be fine,
             * because typically the user will request OPCLK to run
             * at 256fs or 512fs, and for these cases we will also
             * find an exact MCLK divider configuration - it will
             * be equal to or double the OPCLK divisor.
             */
            ret = nau8822_configure_pll(codec);
        break;
    case NAU8822_BCLKDIV:
        if (div & ~0x1c)
            return -EINVAL;
        snd_soc_update_bits(codec, NAU8822_CLOCKING, 0x1c, div);
        break;
    case NAU8822_256FSRATE:
        nau8822->f_256fs = div;

        if (nau8822->f_mclk)
            ret = nau8822_configure_pll(codec);
        break;
    default:
        return -EINVAL;
    }

    dev_dbg(codec->dev, "%s: ID %d, value %u\n", __func__, div_id, div);

    return ret;
}

/*
 * @freq:    when .set_pll() us not used, freq is codec MCLK input frequency
 */
static int nau8822_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
                                  unsigned int freq, int dir)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
    int ret = 0;

    dev_dbg(codec->dev, "%s: ID %d, freq %u\n", __func__, clk_id, freq);

    if (freq) {
        nau8822->f_mclk = freq;

        /* Even if MCLK is used for system clock, might have to drive OPCLK */
        if (nau8822->f_opclk)
            ret = nau8822_configure_pll(codec);

        /* Our sysclk is fixed to 256 * fs, will configure in .hw_params()  */

        if (!ret)
            nau8822->sysclk = clk_id;
    }

    if (nau8822->sysclk == NAU8822_PLL && (!freq || clk_id == NAU8822_MCLK)) {
        /* Clock CODEC directly from MCLK */
        snd_soc_update_bits(codec, NAU8822_CLOCKING, 0x100, 0);

        /* GPIO1 into default mode as input - before configuring PLL */
        snd_soc_update_bits(codec, NAU8822_GPIO_CONTROL, 7, 0);

        /* Turn off PLL */
        snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0);
        nau8822->sysclk = NAU8822_MCLK;
        nau8822->f_pllout = 0;
        nau8822->f_opclk = 0;
    }

    return ret;
}

/*
 * Set ADC and Voice DAC format.
 */
static int nau8822_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
    struct snd_soc_codec *codec = codec_dai->codec;
    /*
     * BCLK polarity mask = 0x100, LRC clock polarity mask = 0x80,
     * Data Format mask = 0x18: all will be calculated anew
     */
    u16 iface = snd_soc_read(codec, NAU8822_AUDIO_INTERFACE) & ~0x198;
    u16 clk = snd_soc_read(codec, NAU8822_CLOCKING);

    dev_dbg(codec->dev, "%s\n", __func__);

    /* set master/slave audio interface */
    switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
    case SND_SOC_DAIFMT_CBM_CFM:
        clk |= 1;
        break;
    case SND_SOC_DAIFMT_CBS_CFS:
        clk &= ~1;
        break;
    default:
        return -EINVAL;
    }

    /* interface format */
    switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
    case SND_SOC_DAIFMT_I2S:
        iface |= 0x10;
        break;
    case SND_SOC_DAIFMT_RIGHT_J:
        break;
    case SND_SOC_DAIFMT_LEFT_J:
        iface |= 0x8;
        break;
    case SND_SOC_DAIFMT_DSP_A:
        iface |= 0x18;
        break;
    default:
        return -EINVAL;
    }

    /* clock inversion */
    switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
    case SND_SOC_DAIFMT_NB_NF:
        break;
    case SND_SOC_DAIFMT_IB_IF:
        iface |= 0x180;
        break;
    case SND_SOC_DAIFMT_IB_NF:
        iface |= 0x100;
        break;
    case SND_SOC_DAIFMT_NB_IF:
        iface |= 0x80;
        break;
    default:
        return -EINVAL;
    }

    snd_soc_write(codec, NAU8822_AUDIO_INTERFACE, iface);
    snd_soc_write(codec, NAU8822_CLOCKING, clk);

    return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int nau8822_hw_params(struct snd_pcm_substream *substream,
                             struct snd_pcm_hw_params *params,
                             struct snd_soc_dai *dai)
{
    struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_codec *codec = rtd->codec;
    struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
    /* Word length mask = 0x60 */
    u16 iface_ctl = snd_soc_read(codec, NAU8822_AUDIO_INTERFACE) & ~0x60;
    /* Sampling rate mask = 0xe (for filters) */
    u16 add_ctl = snd_soc_read(codec, NAU8822_ADDITIONAL_CONTROL) & ~0xe;
    u16 clking = snd_soc_read(codec, NAU8822_CLOCKING);
    enum nau8822_sysclk_src current_clk_id = clking & 0x100 ?
        NAU8822_PLL : NAU8822_MCLK;
    unsigned int f_sel, diff, diff_best = INT_MAX;
    int i, best = 0;

    if (!nau8822->f_mclk)
        return -EINVAL;

    /* bit size */
    switch (params_format(params)) {
    case SNDRV_PCM_FORMAT_S16_LE:
        break;
    case SNDRV_PCM_FORMAT_S20_3LE:
        iface_ctl |= 0x20;
        break;
    case SNDRV_PCM_FORMAT_S24_LE:
        iface_ctl |= 0x40;
        break;
    case SNDRV_PCM_FORMAT_S32_LE:
        iface_ctl |= 0x60;
        break;
    }

    /* filter coefficient */
    switch (params_rate(params)) {
    case 8000:
        add_ctl |= 0x5 << 1;
        break;
    case 11025:
        add_ctl |= 0x4 << 1;
        break;
    case 16000:
        add_ctl |= 0x3 << 1;
        break;
    case 22050:
        add_ctl |= 0x2 << 1;
        break;
    case 32000:
        add_ctl |= 0x1 << 1;
        break;
    case 44100:
    case 48000:
        break;
    }

    /* Sampling rate is known now, can configure the MCLK divider */
    nau8822->f_256fs = params_rate(params) * 256;

    if (nau8822->sysclk == NAU8822_MCLK) {
        nau8822->mclk_idx = -1;
        f_sel = nau8822->f_mclk;
    } else {
        if (!nau8822->f_pllout) {
            /* We only enter here, if OPCLK is not used */
            int ret = nau8822_configure_pll(codec);
            if (ret < 0)
                return ret;
        }
        f_sel = nau8822->f_pllout;
    }

    if (nau8822->mclk_idx < 0) {
        /* Either MCLK is used directly, or OPCLK is used */
        if (f_sel < nau8822->f_256fs || f_sel > 12 * nau8822->f_256fs)
            return -EINVAL;

        for (i = 0; i < ARRAY_SIZE(mclk_numerator); i++) {
            diff = abs(nau8822->f_256fs * 3 -
                       f_sel * 3 * mclk_denominator[i] / mclk_numerator[i]);

            if (diff < diff_best) {
                diff_best = diff;
                best = i;
            }

            if (!diff)
                break;
        }
    } else {
        /* OPCLK not used, codec driven by PLL */
        best = nau8822->mclk_idx;
        diff = 0;
    }

    if (diff)
        dev_warn(codec->dev, "Imprecise sampling rate: %uHz%s\n",
                 f_sel * mclk_denominator[best] / mclk_numerator[best] / 256,
                 nau8822->sysclk == NAU8822_MCLK ?
                 ", consider using PLL" : "");

    dev_dbg(codec->dev, "%s: fmt %d, rate %u, MCLK divisor #%d\n", __func__,
            params_format(params), params_rate(params), best);

    /* MCLK divisor mask = 0xe0 */
    snd_soc_update_bits(codec, NAU8822_CLOCKING, 0xe0, best << 5);

    snd_soc_write(codec, NAU8822_AUDIO_INTERFACE, iface_ctl);
    snd_soc_write(codec, NAU8822_ADDITIONAL_CONTROL, add_ctl);

    if (nau8822->sysclk != current_clk_id) {
        if (nau8822->sysclk == NAU8822_PLL)
            /* Run CODEC from PLL instead of MCLK */
            snd_soc_update_bits(codec, NAU8822_CLOCKING,
                                0x100, 0x100);
        else
            /* Clock CODEC directly from MCLK */
            snd_soc_update_bits(codec, NAU8822_CLOCKING, 0x100, 0);
    }

    return 0;
}

static int nau8822_mute(struct snd_soc_dai *dai, int mute)
{
    struct snd_soc_codec *codec = dai->codec;

    dev_dbg(codec->dev, "%s: %d\n", __func__, mute);

    if (mute)
        snd_soc_update_bits(codec, NAU8822_DAC_CONTROL, 0x40, 0x40);
    else
        snd_soc_update_bits(codec, NAU8822_DAC_CONTROL, 0x40, 0);

    return 0;
}

static int nau8822_set_bias_level(struct snd_soc_codec *codec,
                                  enum snd_soc_bias_level level)
{
    u16 power1 = snd_soc_read(codec, NAU8822_POWER_MANAGEMENT_1) & ~3;

    switch (level) {
    case SND_SOC_BIAS_ON:
    case SND_SOC_BIAS_PREPARE:
        power1 |= 1;  /* VMID 75k */
        snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1, power1);
        break;
    case SND_SOC_BIAS_STANDBY:
        /* bit8,7,6 enable,bit 4: enable bias, bit 2: enable I/O tie off buffer */
        power1 |= 0x1dc;

        if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
            /* Initial cap charge at VMID 5k */
            snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1,
                          power1 | 0x3);
            mdelay(100);
        }

        power1 |= 0x2;  /* VMID 500k */
        snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1, power1);
        break;
    case SND_SOC_BIAS_OFF:
        /* Preserve PLL - OPCLK may be used by someone */
        snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, ~0x20, 0);
        snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_2, 0);
        /*snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_3, 0);*/
        break;
    }

    dev_dbg(codec->dev, "%s: %d, %x\n", __func__, level, power1);

    codec->dapm.bias_level = level;
    return 0;
}

#define NAU8822_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
                         SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops nau8822_dai_ops = {
    .hw_params    = nau8822_hw_params,
    /*.digital_mute    = nau8822_mute,*/
    .set_fmt    = nau8822_set_dai_fmt,
    .set_clkdiv    = nau8822_set_dai_clkdiv,
    .set_sysclk    = nau8822_set_dai_sysclk,
};

/* Also supports 12kHz */
static struct snd_soc_dai_driver nau8822_dai = {
    .name = "nau8822",
    .playback = {
        .stream_name = "Playback",
        .channels_min = 1,
        .channels_max = 2,
        .rates = SNDRV_PCM_RATE_8000_48000,
        .formats = NAU8822_FORMATS,
    },
    .capture = {
        .stream_name = "Capture",
        .channels_min = 1,
        .channels_max = 2,
        .rates = SNDRV_PCM_RATE_8000_48000,
        .formats = NAU8822_FORMATS,
    },
    .ops = &nau8822_dai_ops,
};

static int nau8822_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
    nau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
    /* Also switch PLL off */
    snd_soc_write(codec, NAU8822_POWER_MANAGEMENT_1, 0);

    return 0;
}

static int nau8822_resume(struct snd_soc_codec *codec)
{
    struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
    int i;
    u16 *cache = codec->reg_cache;

    /* Sync reg_cache with the hardware */
    for (i = 0; i < ARRAY_SIZE(nau8822_reg); i++) {
        if (i == NAU8822_RESET)
            continue;
        if (cache[i] != nau8822_reg[i])
            snd_soc_write(codec, i, cache[i]);
    }

    nau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

    if (nau8822->f_pllout)
        /* Switch PLL on */
        snd_soc_update_bits(codec, NAU8822_POWER_MANAGEMENT_1, 0x20, 0x20);

    return 0;
}

/*
 * These registers contain an "update" bit - bit 8. This means, for example,
 * that one can write new DAC digital volume for both channels, but only when
 * the update bit is set, will also the volume be updated - simultaneously for
 * both channels.
 */
static const int update_reg[] = {
    NAU8822_LEFT_DAC_DIGITAL_VOLUME,
    NAU8822_RIGHT_DAC_DIGITAL_VOLUME,
    NAU8822_LEFT_ADC_DIGITAL_VOLUME,
    NAU8822_RIGHT_ADC_DIGITAL_VOLUME,
    NAU8822_LEFT_INP_PGA_CONTROL,
    NAU8822_RIGHT_INP_PGA_CONTROL,
    NAU8822_LOUT1_HP_CONTROL,
    NAU8822_ROUT1_HP_CONTROL,
    NAU8822_LOUT2_SPK_CONTROL,
    NAU8822_ROUT2_SPK_CONTROL,
};

static int nau8822_probe(struct snd_soc_codec *codec)
{
    struct nau8822_priv *nau8822 = snd_soc_codec_get_drvdata(codec);
    int ret = 0, i;
    /*
     * Set default system clock to PLL, it is more precise, this is also the
     * default hardware setting
     */
    nau8822->sysclk = NAU8822_PLL;
    ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_I2C);
    if (ret < 0) {
        dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
        return ret;
    }

    /*
     * Set the update bit in all registers, that have one. This way all
     * writes to those registers will also cause the update bit to be
     * written.
     */
    for (i = 0; i < ARRAY_SIZE(update_reg); i++)
        snd_soc_update_bits(codec, update_reg[i], 0x100, 0x100);

    /* Reset the codec */
    ret = snd_soc_write(codec, NAU8822_RESET, 0);
    if (ret < 0) {
        dev_err(codec->dev, "Failed to issue reset\n");
        return ret;
    }

    nau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

    /*initize codec register*/
    mdelay(25); //25000 us
    for(i=0;i<SET_CODEC_REG_INIT_NUM;i++)
    {
        snd_soc_write(codec,Set_Codec_Reg_Init[i][0],Set_Codec_Reg_Init[i][1]);
    }

    return 0;
}

/* power down chip */
static int nau8822_remove(struct snd_soc_codec *codec)
{
    nau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
    return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_nau8822 = {
    .probe =    nau8822_probe,
    .remove =    nau8822_remove,
    .suspend =    nau8822_suspend,
    .resume =    nau8822_resume,
    .set_bias_level = nau8822_set_bias_level,
    .reg_cache_size = ARRAY_SIZE(nau8822_reg),
    .reg_word_size = sizeof(u16),
    .reg_cache_default = nau8822_reg,
    .controls = nau8822_snd_controls,
    .num_controls = ARRAY_SIZE(nau8822_snd_controls),
    /*
     *.dapm_widgets = nau8822_dapm_widgets,
     *.num_dapm_widgets = ARRAY_SIZE(nau8822_dapm_widgets),
     *.dapm_routes = nau8822_dapm_routes,
     *.num_dapm_routes = ARRAY_SIZE(nau8822_dapm_routes),
     */
};

static int nau8822_i2c_probe(struct i2c_client *i2c,
                                       const struct i2c_device_id *id)
{
    struct nau8822_priv *nau8822;
    int ret;

    nau8822 = devm_kzalloc(&i2c->dev, sizeof(struct nau8822_priv), GFP_KERNEL);
    if (nau8822 == NULL)
        return -ENOMEM;

    i2c_set_clientdata(i2c, nau8822);

    ret = snd_soc_register_codec(&i2c->dev,
                                 &soc_codec_dev_nau8822, &nau8822_dai, 1);

    if (ret < 0)
        devm_kfree(&i2c->dev, nau8822);
    return ret;
}

static int nau8822_i2c_remove(struct i2c_client *client)
{
    snd_soc_unregister_codec(&client->dev);
    kfree(i2c_get_clientdata(client));
    return 0;
}


static const struct i2c_device_id nau8822_i2c_id[] = {
    { "nau8822", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, nau8822_i2c_id);

static const struct of_device_id nau8822_dt_ids[] = {
    { .compatible = "nuvoton,nau8822", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nau8822_dt_ids);

static struct i2c_driver nau8822_i2c_driver = {
    .driver = {
        .name = "nau8822",
        .owner = THIS_MODULE,
        .of_match_table = nau8822_dt_ids,
    },
    .probe =    nau8822_i2c_probe,
    .remove =   nau8822_i2c_remove,
    .id_table = nau8822_i2c_id,
};

module_i2c_driver(nau8822_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU8822 codec driver");
MODULE_AUTHOR("Guennadi Liakhovetski <g.liakhovetski@gmx.de>");
MODULE_LICENSE("GPL");
