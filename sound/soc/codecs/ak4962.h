/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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
#ifndef AK4962_CODEC_H
#define AK4962_CODEC_H

#include "ak4962-jde.h"
#include <sound/apr_audio-v2.h>
#include <linux/mfd/ak49xx/ak49xx-slimslave.h>

#define AK4962_NUM_REGISTERS 421
#define AK4962_MAX_REGISTER (AK4962_NUM_REGISTERS-1)
#define AK4962_CACHE_SIZE AK4962_NUM_REGISTERS

#define MAX_ARRAY_SIZE		800

#define AK4962_REG_VAL(reg, val)		{reg, 0, val}

extern const u8 ak4962_reg_readable[AK4962_CACHE_SIZE];
extern const u8 ak4962_reg_defaults[AK4962_CACHE_SIZE];

enum ak4962_state {
	AK4962_IDLE = 0,
	AK4962_DSPRSTNON,
};

enum ak4962_slimbus_stream_state {
	AK4962_SLIMBUS_STREAM_NA = 0,
	AK4962_SLIMBUS_STREAM_OFF,
	AK4962_SLIMBUS_STREAM_ON,
};

enum ak4962_pid_current {
	AK4962_PID_MIC_2P5_UA,
	AK4962_PID_MIC_5_UA,
	AK4962_PID_MIC_10_UA,
	AK4962_PID_MIC_20_UA,
};

struct ak4962_reg_mask_val {
	u16	reg;
	u8	mask;
	u8	val;
};

struct ak4962_reg_val {
	u16 reg;
	u8	val;
};

enum ak4962_mbhc_clk_freq {
	AK4962_MCLK_12P2MHZ = 0,
	AK4962_MCLK_9P6MHZ,
	AK4962_NUM_CLK_FREQS,
};

enum ak49xx_codec_event {
	AK49XX_CODEC_EVENT_CODEC_UP = 0,
};

#define	SIZE_OF_PRAM_PHONE		1000

enum ak4962_dsp_mode_enum {
	DSP_MODE_OFF = 0,
	DSP_MODE_NARROW_HANDSET,
	DSP_MODE_NARROW_HEADSET,
	DSP_MODE_NARROW_HANDSFREE,
	DSP_MODE_NARROW_HEADPHONE,
	DSP_MODE_WIDE_HANDSET,
	DSP_MODE_WIDE_HEADSET,
	DSP_MODE_WIDE_HANDSFREE,
	DSP_MODE_WIDE_HEADPHONE,
	DSP_MODE_VOICE_RECOGNITION,
	DSP_MODE_SOUND_RECORD,
	DSP_MODE_VOICE_RECORD,
	DSP_MODE_INTERVIEW_RECORD,
	DSP_MODE_MUSIC_SPEAKER,
	DSP_MODE_BARGE_IN,
	DSP_MODE_KARAOKE_CAVE,
	DSP_MODE_KARAOKE_CLOISTER,
	DSP_MODE_KARAOKE_LIVE,
	DSP_MODE_KARAOKE_ROOM,
	DSP_MODE_KARAOKE_THEATER,
	DSP_MODE_KARAOKE_VALLEY,
	DSP_MODE_KARAOKE_BYPASS,
	DSP_MODE_KARAOKE_SPK_ROOM,
	DSP_MODE_KARAOKE_SPK_THEATER,
	DSP_MODE_KARAOKE_SPK_LIVE,
	DSP_MODE_BEAM_RECORD,
	DSP_MODE_SMART_RINGTONE
};

/* Number of input and output Slimbus port */
enum {
	AK4962_RX1 = 0,
	AK4962_RX2,
	AK4962_RX3,
	AK4962_RX4,
	AK4962_RX5,
	AK4962_RX6,
	AK4962_RX_MAX,
};

enum {
	AK4962_TX1 = 0,
	AK4962_TX2,
	AK4962_TX3,
	AK4962_TX4,
	AK4962_TX5,
	AK4962_TX6,
	AK4962_TX7,
	AK4962_TX8,
	AK4962_TX9,
	AK4962_TX10,
	AK4962_TX_MAX,
};

enum {
	disable_cp1 = 0,
	enable_cp1,
};

enum {
	disable_cp2 = 0,
	enable_cp2,
};

enum {
	disable_cp3 = 0,
	enable_cp3,
};

enum {
	disable_ldo3 = 0,
	enable_ldo3,
};

enum {
	button_error = 48,
	push_key_media,
	push_key_volumeup,
	push_key_volumedown,
	push_key_voicecommand,
	release_key_media,
	release_key_volumeup,
	release_key_volumedown,
	release_key_voicecommand,
};
/* rev0.12 end */
/* extern int zte_get_board_id(void); */

extern void *ak4962_get_afe_config(struct snd_soc_codec *codec,
				  enum afe_config_type config_type);

extern void ak4962_event_register(
	int (*machine_event_cb)(struct snd_soc_codec *codec,
				enum ak49xx_codec_event),
	struct snd_soc_codec *codec);

int ak4962_codec_mclk_enable(struct snd_soc_codec *codec,
			     int enable, bool dapm);

#endif
