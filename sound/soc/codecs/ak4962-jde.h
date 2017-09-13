/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef AK4962_CODEC_JDE_H
#define AK4962_CODEC_JDE_H

#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/switch.h>
#include <linux/input.h>

#define NR_SCANCODES 2

#define AK4962_JACK_BUTTON_MASK (SND_JACK_BTN_0 | SND_JACK_BTN_1 | \
				SND_JACK_BTN_2 | SND_JACK_BTN_3 | \
				SND_JACK_BTN_4)
#define MAX_MIC_DET_TRY	68	/* rev0.11--three section eraphone detect time,unit=10ms */
#define MAX_MIC_DET_RPT	13	/* rev0.10 */
#define MAX_KEY_DET_RPT	1	/* rev0.10 */
#define MIC_DETECTION_LEVEL_VALID	85	/* rev0.10 */
/* #define MIC_DETECTION_LEVEL_VALID	11	//rev0.9 */
#define LRMG_MAX_MIC_DET_TRY	9	/* rev0.9 */
#define LRMG_MAX_MIC_DET_RPT	5	/* rev0.9 */
#define LRMG_MIC_DETECTION_LEVEL_VALID	11	/* rev0.9 */
#define MAX_DUAL_MIC_DET_TRY	15	/* rev0.13 */
#define MAX_DUAL_MIC_DET_RPT	13	/* rev0.13 */

#define IMP_SHORT	13
#define IMP_16OHM	30
#define IMP_32OHM	52
#define IMP_OVER_45OHM	128

#define HP_DVOL1_OFFSET_9			1		/* rev0.7 */
#define HP_DVOL1_OFFSET_16			2		/* rev0.7 */
#define HP_DVOL1_OFFSET_32			0		/* rev0.7 */
#define HP_DVOL1_OFFSET_45_114	4		/* rev0.7 */
#define HP_DVOL1_OFFSET_H			8		/* rev0.7 */
#define HP_AVOL_OFFSET_9			-3		/* rev0.7 */
#define HP_AVOL_OFFSET_16			-2		/* rev0.7 */
#define HP_AVOL_OFFSET_32			0		/* rev0.7 */
#define HP_AVOL_OFFSET_45_114		1		/* rev0.7 */
#define HP_AVOL_OFFSET_H			0		/* rev0.7 */

#define HIGH_PERFORMANCE_VOL_OFFSET	1	/* rev0.20 */
#define LOW_POWER_VOL_OFFSET	-1		/* rev0.20 */

#define KEY_MEDIA_THRESHOLD			8
#define KEY_VOICECOMMAND_THRESHOLD	19
#define KEY_VOLUMEUP_THRESHOLD		30
#define KEY_VOLUMEDOWN_THRESHOLD	60

/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
	BIT_HEADSET_DUAL_MIC = (1 << 6),	/* test4 */
	BIT_HEADSET_UNSUPPORTED = (1 << 7),	/* rev0.9 */
};

static const unsigned short ak4962_keycode[NR_SCANCODES] = {
	KEY_VOLUMEDOWN, KEY_VOLUMEUP
};

struct ak4962_mbhc_config {
	struct snd_soc_jack *headset_jack;
	struct snd_soc_jack *button_jack;
#ifdef CONFIG_SWITCH
	struct switch_dev	*h2w_sdev;
	struct input_dev	*btn_idev;
	unsigned short keycode[NR_SCANCODES];
#endif
	int (*mclk_cb_fn)(struct snd_soc_codec*, int, bool);
	unsigned int mclk_rate;
	int gpio_level_insert;
	/* swap_gnd_mic returns true if extern GND/MIC swap switch toggled */
	bool (*swap_gnd_mic)(struct snd_soc_codec *);
};

extern int ak4962_hs_detect(struct snd_soc_codec *codec,
			 const struct ak4962_mbhc_config *cfg);

/* extern int akm4962_enable_ext_clk(int mclk_enable); */
extern int msm8952_enable_ext_codec_mclk(int enable, int dapm);
extern int ak4962_mclk_enable(struct snd_soc_codec *codec, int mclk_enable,
			     bool dapm);

#endif
