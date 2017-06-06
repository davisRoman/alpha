/* 
 * nau8812.h            Based on nau8812.h
 * Copyright (c) 2010 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Changelog:
 *
 */
/*
 * wm8978.h		--  codec driver for WM8978
 *
 * Copyright 2009 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __NAU8812_H__
#define __NAU8812_H__

/*
 * Register values.
 */
#define NAU8812_RESET				0x00
#define NAU8812_POWER_MANAGEMENT_1		0x01
#define NAU8812_POWER_MANAGEMENT_2		0x02
#define NAU8812_POWER_MANAGEMENT_3		0x03
#define NAU8812_AUDIO_INTERFACE			0x04
#define NAU8812_COMPANDING_CONTROL		0x05
#define NAU8812_CLOCKING				0x06
#define NAU8812_ADDITIONAL_CONTROL		0x07
#define NAU8812_GPIO_CONTROL			0x08
//#define NAU8812_JACK_DETECT_CONTROL_1		0x09
#define NAU8812_DAC_CONTROL			0x0A
#define NAU8812_LEFT_DAC_DIGITAL_VOLUME		0x0B
//#define NAU8812_RIGHT_DAC_DIGITAL_VOLUME		0x0C
//#define NAU8812_JACK_DETECT_CONTROL_2		0x0D
#define NAU8812_ADC_CONTROL			0x0E
#define NAU8812_LEFT_ADC_DIGITAL_VOLUME		0x0F
//#define NAU8812_RIGHT_ADC_DIGITAL_VOLUME		0x10
//#define NAU8812_EQ1				0x12
//#define NAU8812_EQ2				0x13
//#define NAU8812_EQ3				0x14
//#define NAU8812_EQ4				0x15
//#define NAU8812_EQ5				0x16
#define NAU8812_DAC_LIMITER_1			0x18
#define NAU8812_DAC_LIMITER_2			0x19
#define NAU8812_NOTCH_FILTER_1			0x1b
#define NAU8812_NOTCH_FILTER_2			0x1c
#define NAU8812_NOTCH_FILTER_3			0x1d
#define NAU8812_NOTCH_FILTER_4			0x1e
#define NAU8812_ALC_CONTROL_1			0x20
#define NAU8812_ALC_CONTROL_2			0x21
#define NAU8812_ALC_CONTROL_3			0x22
#define NAU8812_NOISE_GATE			0x23
#define NAU8812_PLL_N				0x24
#define NAU8812_PLL_K1				0x25
#define NAU8812_PLL_K2				0x26
#define NAU8812_PLL_K3				0x27
#define NAU8812_ATTENUATION_CTRL	0x28
//#define NAU8812_3D_CONTROL			0x29
//#define NAU8812_BEEP_CONTROL			0x2b
#define NAU8812_INPUT_CONTROL			0x2c
#define NAU8812_LEFT_INP_PGA_CONTROL		0x2d
//#define NAU8812_RIGHT_INP_PGA_CONTROL		0x2e
#define NAU8812_LEFT_ADC_BOOST_CONTROL		0x2f
//#define NAU8812_RIGHT_ADC_BOOST_CONTROL		0x30
#define NAU8812_OUTPUT_CONTROL			0x31
#define NAU8812_LEFT_MIXER_CONTROL		0x32
//#define NAU8812_RIGHT_MIXER_CONTROL		0x33
//#define NAU8812_LOUT1_HP_CONTROL			0x34
//#define NAU8812_ROUT1_HP_CONTROL			0x35
#define NAU8812_LOUT2_SPK_CONTROL		0x36
//#define NAU8812_ROUT2_SPK_CONTROL		0x37
#define NAU8812_OUT3_MIXER_CONTROL		0x38
//#define NAU8812_OUT4_MIXER_CONTROL		0x39

#define NAU8812_CACHEREGNUM			58

/* Clock divider Id's */
enum nau8812_clk_id {
	NAU8812_OPCLKRATE,
	NAU8812_BCLKDIV,
  NAU8812_256FSRATE,
};

enum nau8812_sysclk_src {
	NAU8812_PLL,
	NAU8812_MCLK
};

//extern struct snd_soc_dai nau8812_dai;
//extern struct snd_soc_codec_device soc_codec_dev_nau8812;

#endif	/* __NAU8812_H__ */
