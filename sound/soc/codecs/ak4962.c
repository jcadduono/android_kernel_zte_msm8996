/* Copyright (c) 2011-2015, Asahi Kasei Microdevices Corporation.
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/wait.h>
#include <linux/mfd/ak49xx/core.h>
#include <linux/mfd/ak49xx/ak496x_registers.h>
#include <linux/mfd/ak49xx/ak4962_registers.h>
#include <linux/mfd/ak49xx/pdata.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/kfifo.h>
#include <linux/jiffies.h>
#include <linux/clk.h>
#include "ak4962.h"
/* #include "core.h" */
/* zte jjp 2015-03-30 start */
#include <linux/time.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
/* zte jjp 2015-03-30   end*/

/* weizj add for audience es804 */
#if defined(CONFIG_SND_SOC_ES705) || defined(CONFIG_SND_SOC_ES705_ESCORE) || \
	defined(CONFIG_SND_SOC_ES804_ESCORE)
#include "audience/es705-export.h"
#endif

#if defined(CONFIG_SND_SOC_ES705)

#define CONFIG_SND_SOC_ESXXX
#define REMOTE_ROUTE_ENABLE_CB  es705_remote_route_enable
#define SLIM_GET_CHANNEL_MAP_CB es705_slim_get_channel_map
#define SLIM_SET_CHANNEL_MAP_CB es705_slim_set_channel_map
#define SLIM_HW_PARAMS_CB       es705_slim_hw_params
#define REMOTE_CFG_SLIM_RX_CB	es705_remote_cfg_slim_rx
#define REMOTE_CLOSE_SLIM_RX_CB	es705_remote_close_slim_rx
#define REMOTE_CFG_SLIM_TX_CB	es705_remote_cfg_slim_tx
#define REMOTE_CLOSE_SLIM_TX_CB	es705_remote_close_slim_tx
#define REMOTE_ADD_CODEC_CONTROLS_CB	es705_remote_add_codec_controls

#elif defined(CONFIG_SND_SOC_ES705_ESCORE) || \
	defined(CONFIG_SND_SOC_ES804_ESCORE)

#if defined(CONFIG_SND_SOC_ES_SLIM)
#define CONFIG_SND_SOC_ESXXX
#define REMOTE_ROUTE_ENABLE_CB		es705_remote_route_enable
#define SLIM_GET_CHANNEL_MAP_CB		escore_slim_get_channel_map
#define SLIM_SET_CHANNEL_MAP_CB		escore_slim_set_channel_map
#define SLIM_HW_PARAMS_CB		escore_slim_hw_params
#define REMOTE_CFG_SLIM_RX_CB		escore_remote_cfg_slim_rx
#define REMOTE_CLOSE_SLIM_RX_CB		escore_remote_close_slim_rx
#define REMOTE_CFG_SLIM_TX_CB		escore_remote_cfg_slim_tx
#define REMOTE_CLOSE_SLIM_TX_CB		escore_remote_close_slim_tx
#define REMOTE_ADD_CODEC_CONTROLS_CB	es705_remote_add_codec_controls
#endif

#endif

/* weizj add for audience es804 */

#if 0
#define AK4962_NUM_PRAM			9
#define AK4962_NUM_CRAM			22
#define AK4962_NUM_ORAM			4
#else
#define AK4962_NUM_PRAM			10 /* 9 */
#define AK4962_NUM_CRAM			25 /* 24 */
#define AK4962_NUM_ARAM			4
#endif

#define JACK_ALWAYS_OPEN_WHEN_PLUGOUT
#define ANC_JACK_NO_REMOTE_CONTROL	/* test4 */
#define READ_ERROR_LENGTH 10

#define WB_NOT_SET	0
#define NB_SET		1
#define WB_SET		2

static struct afe_param_slimbus_slave_port_cfg ak4962_slimbus_slave_port_cfg = {
	.minor_version = 1,
	.slimbus_dev_id = AFE_SLIMBUS_DEVICE_1,
	.slave_dev_pgd_la = 0,
	.slave_dev_intfdev_la = 0,
	.bit_width = 16,
	.data_format = 0,
	.num_channels = 1
};

#if 0
static const char * const AK4962_PRAM_FIRMWARES[] = {
	"ak4962_pram_narrow.bin",
	"ak4962_pram_wide.bin",
	"ak4962_pram_voice_recognition.bin",
	"ak4962_pram_sound_record.bin",
	"ak4962_pram_music_speaker.bin",
	"ak4962_pram_barge_in.bin",
	"ak4962_pram_karaoke.bin",
	"ak4962_pram_beam_record.bin",
	"ak4962_pram_smart_ringtone.bin"
};

static const char * const AK4962_CRAM_FIRMWARES[] = {
	"ak4962_cram_null.bin",
	"ak4962_cram_narrow_hs.bin",
	"ak4962_cram_narrow_hp.bin",
	"ak4962_cram_narrow_hf.bin",
	"ak4962_cram_narrow_headphone.bin",
	"ak4962_cram_wide_hs.bin",
	"ak4962_cram_wide_hp.bin",
	"ak4962_cram_wide_hf.bin",
	"ak4962_cram_wide_headphone.bin",
	"ak4962_cram_voice_recognition.bin",
	"ak4962_cram_sound_record.bin",
	"ak4962_cram_voice_record.bin",
	"ak4962_cram_music_speaker.bin",
	"ak4962_cram_karaoke_cave.bin",
	"ak4962_cram_karaoke_cloister.bin",
	"ak4962_cram_karaoke_live.bin",
	"ak4962_cram_karaoke_room.bin",
	"ak4962_cram_karaoke_theater.bin",
	"ak4962_cram_karaoke_valley.bin",
	"ak4962_cram_karaoke_bypass.bin",
	"ak4962_cram_beam_record.bin",
	"ak4962_cram_smart_ringtone.bin"
};

static const char * const AK4962_ORAM_FIRMWARES[] = {
	"ak4962_oram_null.bin",
	"ak4962_oram_karaoke_heavy.bin",
	"ak4962_oram_karaoke_light.bin",
	"ak4962_oram_karaoke_middle.bin"
};
#else
static const char * const AK4962_PRAM_FIRMWARES[] = {
	"ak4962_pram_narrow.bin",
	"ak4962_pram_wide.bin",
	"ak4962_pram_voice_recognition.bin",
	"ak4962_pram_sound_record.bin",
	"ak4962_pram_interview_record.bin",
	"ak4962_pram_music_speaker.bin",
	"ak4962_pram_barge_in.bin",
	"ak4962_pram_karaoke.bin",
	"ak4962_pram_karaoke_spk.bin",
	"ak4962_pram_beam_record.bin",
};
static const char * const AK4962_CRAM_FIRMWARES[] = {
	"ak4962_cram_narrow_handset.bin",
	"ak4962_cram_narrow_headset.bin",
	"ak4962_cram_narrow_headphone.bin",
	"ak4962_cram_narrow_handfree.bin",
	"ak4962_cram_wide_handset.bin",
	"ak4962_cram_wide_headset.bin",
	"ak4962_cram_wide_headphone.bin",
	"ak4962_cram_wide_handfree.bin",
	"ak4962_cram_voice_recognition.bin",
	"ak4962_cram_sound_record.bin",
	"ak4962_cram_voice_record.bin",
	"ak4962_cram_interview_record.bin",
	"ak4962_cram_music_speaker.bin",
	"ak4962_cram_barge_in.bin",
	"ak4962_cram_karaoke_cave.bin",
	"ak4962_cram_karaoke_cloister.bin",
	"ak4962_cram_karaoke_live.bin",
	"ak4962_cram_karaoke_room.bin",
	"ak4962_cram_karaoke_theater.bin",
	"ak4962_cram_karaoke_valley.bin",
	"ak4962_cram_karaoke_bypass.bin",
	"ak4962_cram_karaoke_spk_room.bin",
	"ak4962_cram_karaoke_spk_theater.bin",
	"ak4962_cram_karaoke_spk_live.bin",
	"ak4962_cram_beam_record.bin",
};
static const char * const AK4962_ARAM_FIRMWARES[] = {
	"ak4962_aram_karaoke_heavy.bin",
	"ak4962_aram_karaoke_light.bin",
	"ak4962_aram_karaoke_middle.bin",
	"ak4962_aram_karaoke_spk.bin"
};
#endif

#define CONFIG_DEBUG_FS_CODEC

#define AK4962_DMIC1_CLK_ENABLE (1 << 3)
#define AK4962_DMIC1_CHANEL_LRP (0 << 1)	/* 0: L->Lch, H->Rch; 1: L->Rch, H->Lch */
#define AK4962_DMIC2_CLK_ENABLE (1 << 3)
#define AK4962_DMIC2_CHANEL_LRP (0 << 1)	/* 0: L->Lch, H->Rch; 1: L->Rch, H->Lch */

#define AK4962_RATES SNDRV_PCM_RATE_8000_192000

/* #define BITS_PER_REG 8 */
/* #define SLIM_CLOSE_TIMEOUT	1000 */

#define SLIM_CLOSE_TIMEOUT	1000
#define SRC_RESET_TIMEOUT	600000	/* msecs */

#define AK4962_JACK_MASK (SND_JACK_HEADSET | SND_JACK_OC_HPHL | \
			 SND_JACK_OC_HPHR | SND_JACK_UNSUPPORTED)

#define AK4962_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE)

#define AK4962_FORMATS_AIF (SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE | \
		SNDRV_PCM_FORMAT_MU_LAW | SNDRV_PCM_FORMAT_A_LAW)

/* static int tfa_spk_rcv_switch_gpio = -1;   //for smartPA be lvrongguo */

enum {
	SB1_PB = 0,
	SB1_CAP,
	SB2_PB,
	SB2_CAP,
	SB3_PB,
	SB3_CAP,
	SB4_VIFEED,
	NUM_CODEC_DAIS,
};

enum {
	AIF_PORT1 = 0,
	AIF_PORT2,
	AIF_PORT3,
	AIF_PORT4,
	NUM_AIF_DAIS,
};

#define AK4962_MCLK_RATE_12288KHZ 12288000
#define AK4962_MCLK_RATE_9600KHZ 9600000

#define AK4962_RX_PORT_START_NUMBER	10

static int slim_ssr = 0;/* added as qcom suggest */

static int last_crc;
static struct snd_soc_codec *ak4962_codec;/* add by shengguanghui for volume set at 20160120 */

static struct snd_soc_dai_driver ak4962_dai[];
static int ak4962_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event);
static int ak4962_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event);

static int ak49xx_bandswitch_set(struct snd_soc_codec *codec, int howtochange);

static int	ak4962_dsp_mode = 0;
static int	internal_rx_gain = 0;
static int	internal_mic_gain = 0;
static int	hpfeedback_mic_gain = 0;
static int	internal_ahpf_sensitivity = 0;
static bool share_wait_time = 0;
static u8	dac_status = 0;
static int	pll_status = 0;
static int	hp_status = 0;
static int g_ak4962_band_mode = 0;
static bool ak4962_band_set = 0;
static long	d0, d1;
static long	d2_1, d2_2, d3_1, d3_2, d4;
static int br_work = 0;
static int br_extra_mode = 0;
static int karaoke_mic_gain = 0;	/* add by shengguanghui for karaoke 20150606 */
static int ak4962_gain_set_switch = 0;
int ak4962_hp_impedance_value = 0;

/* from 0dB ~ +20dB */
static const u32 mic_gain_table[21] = {
	0x008000,
	0x008FA0, 0x00A120, 0x00B4D0, 0x00CAE0, 0x00E3A0,
	0x00FF60, 0x011E90, 0x014180, 0x0168C0, 0x0194C0,
	0x01C630, 0x01FD90, 0x023BC0, 0x028180, 0x02CFD0,
	0x0327A0, 0x038A30, 0x03F8C0, 0x0474D0, 0x050000,
};

/* from 0dB ~ -20dB */
static const u32 hp_mic_gain_table[21] = {
	0x008000,
	0x007210, 0x0065B0, 0x005AA0, 0x0050C0, 0x004800,
	0x004020, 0x003930, 0x0032F0, 0x002D70, 0x002880,
	0x002410, 0x002020, 0x001CA0, 0x001990, 0x0016C0,
	0x001450, 0x001210, 0x001020, 0x000E60, 0x000CD0,
};

static const struct ak49xx_ch ak4962_rx_chs[AK4962_RX_MAX] = {
	AK49XX_CH(AK4962_RX_PORT_START_NUMBER, 0),
	AK49XX_CH(AK4962_RX_PORT_START_NUMBER + 1, 1),
	AK49XX_CH(AK4962_RX_PORT_START_NUMBER + 2, 2),
	AK49XX_CH(AK4962_RX_PORT_START_NUMBER + 3, 3),
	AK49XX_CH(AK4962_RX_PORT_START_NUMBER + 4, 4),
	AK49XX_CH(AK4962_RX_PORT_START_NUMBER + 5, 5)
};

static const struct ak49xx_ch ak4962_tx_chs[AK4962_TX_MAX] = {
	AK49XX_CH(0, 0),
	AK49XX_CH(1, 1),
	AK49XX_CH(2, 2),
	AK49XX_CH(3, 3),
	AK49XX_CH(4, 4),
	AK49XX_CH(5, 5),
	AK49XX_CH(6, 6),
	AK49XX_CH(7, 7),
	AK49XX_CH(8, 8),
	AK49XX_CH(9, 9)
};

static const u32 vport_check_table[NUM_CODEC_DAIS] = {
	0,					/* SB1_PB */
	(1 << SB2_CAP) | (1 << SB3_CAP),	/* SB1_CAP */
	0,					/* SB2_PB */
	(1 << SB1_CAP) | (1 << SB3_CAP),	/* SB2_CAP */
	0,					/* SB2_PB */
	(1 << SB1_CAP) | (1 << SB2_CAP),	/* SB2_CAP */
};

static const u32 vport_i2s_check_table[NUM_CODEC_DAIS] = {
	0, /* SB1_PB */
	0, /* SB1_CAP */
	0, /* SB2_PB */
	0, /* SB2_CAP */
};

struct ak4962_priv {
	struct mutex mutex;
	struct workqueue_struct *workqueue;
	struct workqueue_struct *workqueue_br;
	struct work_struct work;
	struct delayed_work work_br;
	struct kfifo fifo_br;
	enum   ak4962_state state;
	enum   ak4962_slimbus_stream_state stream_state;
	enum   ak4962_slimbus_stream_state pcm_state; /* lhs */

	struct snd_soc_codec *codec;

	struct ak4962_mbhc_config mbhc_cfg;

	struct ak49xx_pdata *pdata;

	int ext_clk_users;
	struct clk *cdc_ext_clk;

	/*track ak4962 interface type*/
	u8 intf_type;

	/* num of slim ports required */
	struct ak49xx_codec_dai_data dai[NUM_CODEC_DAIS];

	struct afe_param_cdc_slimbus_slave_cfg slimbus_slave_cfg;

	const struct firmware *pram_firmware[AK4962_NUM_PRAM];
	const struct firmware *cram_firmware[AK4962_NUM_CRAM];
	const struct firmware *aram_firmware[AK4962_NUM_ARAM];

	u8 pram_load_index;
	u8 cram_load_index;
	u8 aram_load_index;

	struct clk *akm_mclk; /* add for mclk */
	bool tfa_spk_rcv_switch;

	bool ak4962_dsp_downlink_status;
	bool ak4962_dsp_uplink_status;
	bool ak4962_voice_sync_switch;

	int low_power_mode;				/* rev0.16 */
	int power_select_vol_offset;	/* rev0.16 */

	u8 ak4962_hp_anc_status;
	u8 ak4962_band_mode;

	u8 lch_bargein_sel;
	u8 rch_bargein_sel;
	u8 slim_inter_regs;

	bool ak4962_uplink_ns_control;

	int (*machine_codec_event_cb)(struct snd_soc_codec *codec,
			enum ak49xx_codec_event);

	struct timer_list timer;
	int mono_jack;

	int hp_dvol1_org;
	int hp_avol_org;
	int hp_dvol1_offset;
	int hp_avol_offset;
	/* Port values for Rx and Tx codec_dai */
	unsigned int rx_port_value;
	unsigned int tx_port_value;
};

#ifdef CONFIG_DEBUG_FS_CODEC
struct ak49xx *debugak49xx;

static ssize_t reg_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, i, j, fpt;
	int rx[256];

	for (i = 0; i < VIRTUAL_ADDRESS_CONTROL; i++) {
		ret = ak49xx_reg_read(&debugak49xx->core_res, i);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

	for (j = CRC_RESULT_H8; j < MIC_DETECTION_LEVEL; i++, j++) {
		ret = ak49xx_reg_read(&debugak49xx->core_res, j);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}
/*
	for (j = ANC_FILTER_ENABLE; j <= SRC_STATUS_3; i++, j++) {
		ret = ak49xx_reg_read(&debugak49xx->core_res, j);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}
*/
	if (i < 256) {

		ret = fpt = 0;

		for (i = 0; i < VIRTUAL_ADDRESS_CONTROL; i++, fpt += 6) {

			ret += snprintf(buf + fpt, sizeof(buf), "%02x,%02x\n", i, rx[i]);
			/*ret += sprintf(buf + fpt, "%02x,%02x\n", i, rx[i]);*/
		}

		for (j = CRC_RESULT_H8; j <= MIC_DETECTION_LEVEL; i++, j++, fpt += 6) {

			ret += snprintf(buf + fpt, sizeof(buf), "%02x,%02x\n", j, rx[i]);
			/*ret += sprintf(buf + fpt, "%02x,%02x\n", j, rx[i]);*/
		}
/*
*		for (j = ANC_FILTER_ENABLE; j <= SRC_STATUS_3; i++, j++, fpt +=7) {
*
*			ret += snprintf(buf + fpt, sizeof(buf), "%03x,%02x\n", j, rx[i]);
*			ret += sprintf(buf + fpt, "%03x,%02x\n", j, rx[i]);
*		}
*/
		return ret;

	} else {

		return snprintf(buf, READ_ERROR_LENGTH, "read error\n");
		/*return sprintf(buf, "read error");*/
	}
}

static ssize_t reg_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	*ptr_data = (char *)buf;
	char	*p;
	int		i, pt_count = 0;
	long int val[20];
	int error;

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 20)
			break;

		/*val[pt_count] = simple_strtoul(p, NULL, 16);*/
		error = kstrtoul(p, 16, &val[pt_count]);
		if (error)
			pr_err("error %d\n", error);

		pt_count++;
	}

	for (i = 0; i < pt_count; i += 2) {
		ak49xx_reg_write(&debugak49xx->core_res, val[i], val[i+1]);
		pr_debug("%s: write add=%ld, val=%ld\n", __func__, val[i], val[i+1]);
	}

	return count;
}

static ssize_t ram_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	u8		val[MAX_ARRAY_SIZE];
	int		i, j, ret;

	ak49xx_reg_write(&debugak49xx->core_res, 0x02, 0x00);

	ret = ak49xx_cram_read(0x0400, 768, val);

	ak49xx_reg_write(&debugak49xx->core_res, 0x02, 0x01);

	if (!ret) {
		for (i = 0x400, j = 0; i < 0x500; i++, j += 3) {
			snprintf(buf, 13, "%04X, %02X%02X%02X\n", i,
					val[j], val[j+1], val[j+2]);
		}
	}

	return 0;
}

static ssize_t ram_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	*ptr_data = (char *)buf;
	char	*p;
	int		pt_count = 0;
	u8		val[9];
	u16		temp;
	int error;

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (pt_count >= 16)
			break;

		/*temp = simple_strtoul(p, NULL, 16);*/
		error = kstrtoul(p, 16, (long *)&temp);
		if (error)
			pr_err("error %d\n", error);
		pt_count++;

		if (pt_count % 3 == 1) {
			val[4] = temp >> 8;
			val[5] = temp & 0xFF;

		} else if (pt_count % 3 == 2) {
			val[6] = temp >> 8;
			val[7] = temp & 0xFF;

		} else {
			val[8] = temp;
			ak49xx_run_ram_write(debugak49xx, val);
			pr_debug("%s: add = %02x%02x, val=%02x%02x%02x.\n",
					__func__, val[4], val[5], val[6], val[7], val[8]);
		}
	}

	return count;
}

static ssize_t ram_load_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int	ret;

	ret = snprintf(buf, 5, "%04X\n", last_crc);

	return ret;
}

static ssize_t ram_load_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char	buf_val[9];
	u8		val[MAX_ARRAY_SIZE];
	u8		vat = 0;
	u8		page = 0;
	u16		ptr_count = 0;
	u16		pt_count = 0;
	u16		start = 0;
	u32		val_tmp;
	int		ret = 0;
	int		error = 0;

	if (!strncmp(buf, "PRAM", 4)) {
		vat = 0x04;
	} else if (!strncmp(buf, "CRAM", 4)) {
		vat = 0x05;
	}
	ptr_count = 4;

	if (*(buf + ptr_count) != '\0') {
		memcpy(buf_val, buf + ptr_count, 4);
		buf_val[4] = '\0';
		/*start = simple_strtoul(buf_val, NULL, 16);*/
		error = kstrtoul(buf_val, 16, (long *)&start);
		if (error)
			pr_err("error %d\n", error);

		ptr_count += 4;
		page += start >> 8;
		start = start & 0x00FF;
	}

	while (*(buf + ptr_count) != '\0') {
		memcpy(buf_val, buf + ptr_count, 8);
		buf_val[8] = '\0';
		/*val_tmp = simple_strtoul(buf_val, NULL, 16);*/
		error = kstrtoul(buf_val, 16, (long *)&val_tmp);
		if (error)
			pr_err("error %d\n", error);

		val[pt_count++] = (val_tmp & 0xFF000000) >> 24;
		val[pt_count++] = (val_tmp & 0x00FF0000) >> 16;
		val[pt_count++] = (val_tmp & 0x0000FF00) >> 8;
		val[pt_count++] = val_tmp & 0x000000FF;
		ptr_count += 8;
	}

	ret += ak49xx_ram_write(debugak49xx, vat, page, start, pt_count, val);
	if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SPI ||
		ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		ret += ak49xx_bulk_read(&debugak49xx->core_res, CRC_RESULT_H8, 2, buf_val);
		last_crc = (buf_val[0] << 8) + buf_val[1];
	} else if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS) {
		if (ret == 0) {
			last_crc = 0;
		} else {
			last_crc = 1;
		}
	}

	if (ret) {
		pr_err("%s: failed!\n", __func__);
	} else {
		pr_info("%s: succeeded!\n", __func__);
	}

	return count;
}

static ssize_t mir_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, i;
	int rx[16];

	for (i = 0; i < 16; i++) {

		ret = ak49xx_reg_read(&debugak49xx->core_res, MIR1_REGISTER_1 + i);

		if (ret < 0) {
			pr_err("%s: read register error.\n", __func__);
			break;

		} else {
			rx[i] = ret;
		}
	}

	if (i == 16) {

		return snprintf(buf, 24, "%02x%02x%x,%02x%02x%x,%02x%02x%x,%02x%02x%x\n",
				rx[0], rx[1], rx[2]>>4, rx[4], rx[5], rx[6]>>4,
				rx[8], rx[9], rx[10]>>4, rx[12], rx[13], rx[14]>>4);
	} else {
		return snprintf(buf, READ_ERROR_LENGTH, "read error");
		/*return sprintf(buf, "read error");*/
	}
}

static ssize_t mir_data_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(reg_data, 0644, reg_data_show, reg_data_store);
static DEVICE_ATTR(ram_data, 0644, ram_data_show, ram_data_store);
static DEVICE_ATTR(ram_load, 0644, ram_load_show, ram_load_store);
static DEVICE_ATTR(mir_data, 0644, mir_data_show, mir_data_store);

#endif

static void ak4962_slim_inter_regs(struct ak49xx *ak49xx)
{
	int ret = 0;

	ret = ak49xx_interface_reg_read(ak49xx, AK496X_SLIM_PGD_PORT0_ARRAY);
	pr_info("%s : reg[%x] = %x\n", __func__, AK496X_SLIM_PGD_PORT0_ARRAY, ret);

	ret = ak49xx_interface_reg_read(ak49xx, AK496X_SLIM_PGD_PORT1_ARRAY);
	pr_info("%s : reg[%x] = %x\n", __func__, AK496X_SLIM_PGD_PORT1_ARRAY, ret);

	ret = ak49xx_interface_reg_read(ak49xx, AK496X_SLIM_PGD_PORT2_ARRAY);
	pr_info("%s : reg[%x] = %x\n", __func__, AK496X_SLIM_PGD_PORT2_ARRAY, ret);

	ret = ak49xx_interface_reg_read(ak49xx, AK496X_SLIM_PGD_PORT3_ARRAY);
	pr_info("%s : reg[%x] = %x\n", __func__, AK496X_SLIM_PGD_PORT3_ARRAY, ret);

	ret = ak49xx_interface_reg_read(ak49xx, AK496X_SLIM_PGD_PORT4_ARRAY);
	pr_info("%s : reg[%x] = %x\n", __func__, AK496X_SLIM_PGD_PORT4_ARRAY, ret);

	ret = ak49xx_interface_reg_read(ak49xx, AK496X_SLIM_PGD_PORT5_ARRAY);
	pr_info("%s : reg[%x] = %x\n", __func__, AK496X_SLIM_PGD_PORT5_ARRAY, ret);

	ret = ak49xx_interface_reg_read(ak49xx, 0x100);
	pr_info("%s : reg[%x] = %x\n", __func__, 0x100, ret);
}

static void ak4962_bring_up(struct ak49xx *ak49xx)
{
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT0_ARRAY, 0x0A);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT1_ARRAY, 0x0B);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT2_ARRAY, 0x0C);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT3_ARRAY, 0x0D);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT4_ARRAY, 0x0E);
	ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT5_ARRAY, 0x0F);

	/* ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT8_ARRAY, 0x02); */
	/* ak49xx_interface_reg_write(ak49xx, AK496X_SLIM_PGD_PORT9_ARRAY, 0x04); */
}

static int ak4962_get_slim_inter_regs(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->slim_inter_regs;

	return 0;
}

static int ak4962_set_slim_inter_regs(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct ak49xx *core;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];

	core = dev_get_drvdata(codec->dev->parent);

	pr_err("%s: status = %d\n", __func__, status);

	switch (status) {
	case 0:
		ak4962_slim_inter_regs(core);
		break;

	case 1:
		ak4962_bring_up(core);
		break;

	default:
		break;
	}

	ak4962->slim_inter_regs = status;
	return 0;
}

static void ak4962_work(struct work_struct *work)
{
	struct snd_soc_codec *codec;
	struct ak4962_priv *ak4962;

	ak4962 = container_of(work, struct ak4962_priv, work);
	codec = ak4962->codec;

	mutex_lock(&ak4962->mutex);
	switch (ak4962->state) {
	case AK4962_DSPRSTNON:
		if (ak4962->pcm_state == AK4962_SLIMBUS_STREAM_ON) {
			snd_soc_write(codec, FLOW_CONTROL_3, 0x01);	/* DSPRSTN On */
			if ((ak4962_dsp_mode >= DSP_MODE_NARROW_HANDSET) &&
				(ak4962_dsp_mode <= DSP_MODE_WIDE_HEADPHONE)) {
				usleep_range(1000, 1100);
				if (g_ak4962_band_mode)
					ak49xx_bandswitch_set(codec, g_ak4962_band_mode);
			}
		}
		break;
	default:
		break;
	}

	if (ak4962->stream_state != AK4962_SLIMBUS_STREAM_NA) {

		if (snd_soc_read(codec, JACK_DETECTION_STATUS) & 0x02) {
			/* JDS == 1 */
			if (ak4962->stream_state == AK4962_SLIMBUS_STREAM_OFF) {
				snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x00);	/* HP Off */
			}

			if (ak4962->stream_state == AK4962_SLIMBUS_STREAM_ON) {
				if (snd_soc_read(codec, POWER_MANAGEMENT_8) & 0x01) {
				/* PMDA1 == 1 */
					snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03);	/* HP On */
				}
			}
		}
	}
	mutex_unlock(&ak4962->mutex);
}

static inline long get_dtime(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	return ((long)(tv.tv_sec) * 1000 + (long)(tv.tv_usec) / 1000);
}

static void ak4962_work_br(struct work_struct *work)
{
	struct snd_soc_codec *codec;
	struct ak4962_priv *ak4962;
	int val, ret;
	/* int val; */
	long fifo_len = 0;
	unsigned char fifo_val;	/* ASCII */
	int report;
	struct delayed_work *dw = to_delayed_work(work);

	pr_info("\t----------[K4962] %s(%d)ENTER work_br\n", __func__, __LINE__);
	ak4962 = container_of(dw, struct ak4962_priv, work_br);
	codec = ak4962->codec;

	fifo_len = kfifo_len(&ak4962->fifo_br);
	pr_info("\t----------[K4962] %s(%d)1: fifo_len=%ld\n", __func__, __LINE__, fifo_len);

       /* while (0) { */
	while (fifo_len > 0) {
	/* ret = kfifo_out(&ak4962->fifo_br, &fifo_val,sizeof(fifo_val)); */
		ret = kfifo_get(&ak4962->fifo_br, &fifo_val);
		pr_info("\t----------[K4962] %s(%d)fifo_val=%d\n", __func__, __LINE__, fifo_val);
		if (ret == 0) {
			pr_info("\t----------[K4962] %s(%d)kfifo is empty\n", __func__, __LINE__);
		}

		switch (fifo_val) {
		case push_key_media:
		case release_key_media:
			report = KEY_MEDIA;
			break;
		case push_key_volumeup:
		case release_key_volumeup:
			report = KEY_VOLUMEUP;
			break;
		case push_key_volumedown:
		case release_key_volumedown:
			report = KEY_VOLUMEDOWN;
			break;
		case push_key_voicecommand:
		case release_key_voicecommand:
			report = KEY_VOICECOMMAND;
			break;
		default:
			break;
		}

		pr_info("\t-------[K4962] %s(%d)report=%d\n", __func__, __LINE__, report);

		val = snd_soc_read(codec, JACK_DETECTION_STATUS);	/* JDS read */

		if (val & 0x01) {
			/* JDS=1 */
			if (fifo_val < release_key_media) {
				/* push */
				if (report == KEY_VOLUMEUP) {
					input_event(ak4962->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 1);
				}
				if (report == KEY_VOLUMEDOWN) {
					input_event(ak4962->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 0);
				}
				pr_err("[LHS] %s line %d : <<<<<<<<<<<<<< report =%d press!\n",
						__func__, __LINE__, report);
				input_report_key(ak4962->mbhc_cfg.btn_idev, report, 1);
				input_sync(ak4962->mbhc_cfg.btn_idev);
				dev_info(codec->dev, "%s: report %d\n", __func__, report);

				d2_2 = get_dtime();
				pr_info("\t*******[AK4962] %s(%d)(push_report)d2_2=%lu\n", __func__,
						__LINE__, d2_2);

			} else {	/* release */
				if (report == KEY_VOLUMEUP) {
					input_event(ak4962->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 1);
				}
				if (report == KEY_VOLUMEDOWN) {
					input_event(ak4962->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 0);
				}
				pr_err("[LHS] %s line %d : >>>>>>>>>>>>>> report =%d release!\n",
					__func__, __LINE__, report);
				input_report_key(ak4962->mbhc_cfg.btn_idev, report, 0);
				input_sync(ak4962->mbhc_cfg.btn_idev);
				dev_info(codec->dev, "%s: report 0\n", __func__);

				d3_2 = get_dtime();
				pr_info("\t*******[AK4962] %s(%d)(release_report)d3_2=%lu\n", __func__,
						__LINE__, d3_2);
			}
		} else {
			kfifo_reset_out(&ak4962->fifo_br);
			pr_info("\t*******[AK4962] %s(%d)JDS=0 loop\n", __func__, __LINE__);
		}

		fifo_len = kfifo_len(&ak4962->fifo_br);
		pr_info("\t----------[K4962] %s(%d)while loop: fifo_len=%ld\n", __func__, __LINE__, fifo_len);
	}	/* while close; fifo_len == 0 */

	br_work = 0;
	if (val == 1) {
		br_extra_mode = 1;	/* 300ms */
	}
	d4 = get_dtime();

	pr_err("[LHS] %s line %d : out!\n", __func__, __LINE__);
}

#if 0
int ak4962_mclk_enable(struct snd_soc_codec *codec, int mclk_enable, bool dapm)
{
/* struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec); */

	pr_debug("%s: mclk_enable = %u, dapm = %d\n", __func__, mclk_enable,
		 dapm);

	return 0;
}
#endif

static int __ak4962_mclk_enable(struct ak4962_priv *ak4962, int mclk_enable)
{
	int ret = 0;

	if (mclk_enable) {
		ak4962->ext_clk_users++;
		if (ak4962->ext_clk_users > 1)
			goto exit;
		ret = clk_prepare_enable(ak4962->cdc_ext_clk);
		if (ret) {
			pr_err("%s: ext clk enable failed\n",
				__func__);
			ak4962->ext_clk_users--;
			goto exit;
		}
	} else {
		ak4962->ext_clk_users--;
		if (ak4962->ext_clk_users == 0) {
			clk_disable_unprepare(ak4962->cdc_ext_clk);
		}
	}
exit:
	return ret;
}

int ak4962_codec_mclk_enable(struct snd_soc_codec *codec,
			     int enable, bool dapm)
{
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	if (ak4962->cdc_ext_clk) {
		dev_dbg(codec->dev, "%s: mclk_enable = %u, dapm = %d\n",
			__func__, enable, dapm);
		return __ak4962_mclk_enable(ak4962, enable);
	}
	dev_err(codec->dev, "%s: Cannot turn on MCLK\n", __func__);
	return -EINVAL;
}
EXPORT_SYMBOL(ak4962_codec_mclk_enable);

static int ak4962_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct ak49xx *ak4962_core = dev_get_drvdata(dai->codec->dev->parent);

	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	if ((ak4962_core != NULL) &&
	    (ak4962_core->dev != NULL) &&
	    (ak4962_core->dev->parent != NULL))
		pm_runtime_get_sync(ak4962_core->dev->parent);

	pr_debug("%s(): ak4962_dsp_mode = %d  dai_id = %d\n", __func__,
				ak4962_dsp_mode, dai->id);
	return 0;
}

static void ak4962_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
/* struct snd_soc_codec *codec = dai->codec; */
	struct ak49xx *ak4962_core = dev_get_drvdata(dai->codec->dev->parent);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(dai->codec);

	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	if (ak4962->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS &&
		ak4962->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		return;

	if ((ak4962_core != NULL) &&
	    (ak4962_core->dev != NULL) &&
	    (ak4962_core->dev->parent != NULL)) {
		pm_runtime_mark_last_busy(ak4962_core->dev->parent);
		pm_runtime_put(ak4962_core->dev->parent);
	}

	pr_debug("%s(): ak4962_dsp_mode = %d  dai_id = %d\n", __func__,
			ak4962_dsp_mode, dai->id);

/* if (ak4962_dsp_mode != 0 && dai->id == SB3_CAP) { */
/* ak4962->state = AK4962_IDLE; */
/* snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00); */
/* ak4962_dsp_mode = 0; */
/* } */
/* switch (dai->id) { */
/* case SB1_PB: */
/* case SB1_CAP: */
/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x70, 0x00); */
/* break; */
/* case SB2_PB: */
/* case SB2_CAP: */
/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x07, 0x00); */
/* break; */
/* case SB3_PB: */
/* case SB3_CAP: */
/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x70, 0x00); */
/* break; */
/* default: */
/* pr_err("%s: Invalid dai id %d\n", __func__, dai->id); */
/* break; */
/* } */
}

static int ak4962_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(dai->codec);
	u8 MDV, BDV, SDV, CMF;

	pr_debug("%s: dai_name = %s DAI-ID %x rate %d num_ch %d\n", __func__,
			dai->name, dai->id, params_rate(params),
			params_channels(params));

	switch (params_rate(params)) {
	case 8000:
		MDV = 0x0E;
		BDV = 0xEF;
		SDV = 0x3F;
		CMF = 0x40;
		break;
	case 16000:
		MDV = 0x0E;
		BDV = 0x77;
		SDV = 0x3F;
		CMF = 0x24;
		break;
	case 32000:
		MDV = 0x0E;
		BDV = 0x3B;
		SDV = 0x3F;
		CMF = 0x08;
		break;
	case 48000:
		MDV = 0x09;
		BDV = 0x27;
		SDV = 0x3F;
		CMF = 0x0A;
		break;
	case 96000:
		MDV = 0x04;
		BDV = 0x13;
		SDV = 0x3F;
		CMF = 0x0E;
		break;
	case 192000:
		MDV = 0x04;
		BDV = 0x09;
		SDV = 0x3F;
		CMF = 0xF2;
		break;
	default:
		pr_err("%s: Invalid sampling rate %d\n", __func__,
				params_rate(params));
		return -EINVAL;
	}
	ak4962->dai[dai->id].rate = params_rate(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ak4962->dai[dai->id].bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ak4962->dai[dai->id].bit_width = 24;
		snd_soc_update_bits(codec, CODEC_AIF_FORMAT, 0x07, 0x00);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		ak4962->dai[dai->id].bit_width = 32;
		snd_soc_update_bits(codec, CODEC_AIF_FORMAT, 0x07, 0x04);
		break;
	default:
		pr_err("%s: Invalid RX format %u\n", __func__,
				params_format(params));
		return -EINVAL;
	}

	snd_soc_update_bits(codec, CDCMCLK_DIVIDER, 0xFF, MDV);
	snd_soc_update_bits(codec, MSYNC5_BDV, 0xFF, BDV);
	snd_soc_update_bits(codec, MSYNC5_SDV, 0xFF, SDV);
	snd_soc_update_bits(codec, CLOCK_MODE_SELECT, 0x7F, CMF);

	if (ak4962->dai[dai->id].rate == 192000) {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x20, 0x20);
	} else {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x20, 0x00);
	}

	switch (dai->id) {
	case SB1_PB:
	case SB1_CAP:
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x50, 0x50);
		break;
	case SB2_PB:
	case SB2_CAP:
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR4, 0x05, 0x05);
		break;
	case SB3_PB:
	case SB3_CAP:
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x50, 0x50);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	return 0;
}

static int ak4962_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot);
static int ak4962_get_channel_map(struct snd_soc_dai *dai,
				unsigned int *tx_num, unsigned int *tx_slot,
				unsigned int *rx_num, unsigned int *rx_slot);

#ifdef CONFIG_SND_SOC_ESXXX
int (*remote_route_enable)(struct snd_soc_dai *dai) = REMOTE_ROUTE_ENABLE_CB;
int (*slim_get_channel_map)(struct snd_soc_dai *dai,
	unsigned int *tx_num, unsigned int *tx_slot,
	unsigned int *rx_num, unsigned int *rx_slot)
	= SLIM_GET_CHANNEL_MAP_CB;
int (*slim_set_channel_map)(struct snd_soc_dai *dai,
	unsigned int tx_num, unsigned int *tx_slot,
	unsigned int rx_num, unsigned int *rx_slot)
	= SLIM_SET_CHANNEL_MAP_CB;
int (*slim_hw_params)(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
	= SLIM_HW_PARAMS_CB;
int (*remote_cfg_slim_rx)(int dai_id)	=	REMOTE_CFG_SLIM_RX_CB;
int (*remote_close_slim_rx)(int dai_id)	=	REMOTE_CLOSE_SLIM_RX_CB;
int (*remote_cfg_slim_tx)(int dai_id)	=	REMOTE_CFG_SLIM_TX_CB;
int (*remote_close_slim_tx)(int dai_id)	=	REMOTE_CLOSE_SLIM_TX_CB;
int (*remote_add_codec_controls)(struct snd_soc_codec *codec)
	= REMOTE_ADD_CODEC_CONTROLS_CB;

static int ak4962_esxxx_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	ak4962_startup(substream, dai);
	return 0;
}

static void ak4962_esxxx_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	ak4962_shutdown(substream, dai);
}

static int ak4962_esxxx_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	int rc = 0;

	pr_err("%s: dai_name = %s DAI-ID %x rate %d num_ch %d\n", __func__,
		dai->name, dai->id, params_rate(params),
		params_channels(params));
	rc = ak4962_hw_params(substream, params, dai);
	if (remote_route_enable(dai))
		rc = slim_hw_params(substream, params, dai);
	else
		pr_info("%s: route not enabled\n", __func__);
	return rc;
}

static int ak4962_esxxx_set_sysclk(struct snd_soc_dai *dai,
	int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_esxxx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_esxxx_set_fll(struct snd_soc_dai *dai, int id, int src,
	unsigned int freq_in, unsigned int freq_out)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_esxxx_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	int mute_reg;
	int reg;

	switch (codec_dai->id) {
	case 1:
		mute_reg = 0x00;
		break;
	case 2:
		mute_reg = 0x01;
		break;
	default:
		return -EINVAL;
	}

	if (mute)
		reg = 1;
	else
		reg = 0;

	/* snd_soc_update_bits(codec, mute_reg, 0x00, reg); */
	return 0;
}

static int ak4962_esxxx_set_channel_map(struct snd_soc_dai *dai,
	unsigned int tx_num, unsigned int *tx_slot,
	unsigned int rx_num, unsigned int *rx_slot)
{
	unsigned int taiko_tx_num = 0;
	unsigned int taiko_tx_slot[6];
	unsigned int taiko_rx_num = 0;
	unsigned int taiko_rx_slot[6];
	int rc = 0;

	pr_err("%s(): dai_name = %s DAI-ID %x tx_ch %d rx_ch %d\n",
		__func__, dai->name, dai->id, tx_num, rx_num);
	if (remote_route_enable(dai)) {
		rc = ak4962_get_channel_map(dai, &taiko_tx_num, taiko_tx_slot,
			&taiko_rx_num, taiko_rx_slot);
		rc = ak4962_set_channel_map(dai, tx_num, taiko_tx_slot, rx_num, taiko_rx_slot);
		rc = slim_set_channel_map(dai, tx_num, tx_slot, rx_num,
			rx_slot);
	} else
		rc = ak4962_set_channel_map(dai, tx_num, tx_slot, rx_num, rx_slot);

	return rc;
}
static int ak4962_esxxx_get_channel_map(struct snd_soc_dai *dai,
	unsigned int *tx_num, unsigned int *tx_slot,
	unsigned int *rx_num, unsigned int *rx_slot)
{
	int rc = 0;

	pr_err("%s(): dai_name = %s DAI-ID %d tx_ch %d rx_ch %d\n",
		__func__, dai->name, dai->id, *tx_num, *rx_num);
	if (remote_route_enable(dai))
		rc = slim_get_channel_map(dai, tx_num, tx_slot, rx_num,
		rx_slot);
	else {
		pr_info("%s: route not enabled\n", __func__);
		rc = ak4962_get_channel_map(dai, tx_num, tx_slot, rx_num, rx_slot);
	}

	return rc;
}

static int ak4962_esxxx_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	pr_debug("%s cmd: %d\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* ak4961->stream_state = AK4961_SLIMBUS_STREAM_ON; */
		ak4962->pcm_state = AK4962_SLIMBUS_STREAM_ON;
		queue_work(ak4962->workqueue, &ak4962->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* ak4961->stream_state = AK4961_SLIMBUS_STREAM_OFF; */
		ak4962->pcm_state = AK4962_SLIMBUS_STREAM_OFF;
		queue_work(ak4962->workqueue, &ak4962->work);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#else
static int ak4962_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_set_fll(struct snd_soc_dai *dai, int id, int src,
			  unsigned int freq_in, unsigned int freq_out)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
/* struct snd_soc_codec *codec = codec_dai->codec; */

#if 0   /* temp del by lvrongguo @20151203 begin */
	int mute_reg;
	int reg;

	switch (codec_dai->id) {
	case 1:
		mute_reg = 0x00;
		break;
	case 2:
		mute_reg = 0x01;
		break;
	default:
		return -EINVAL;
	}

	if (mute)
		reg = 1;
	else
		reg = 0;
#endif   /* temp del by lvrongguo @20151203 end */

	/* snd_soc_update_bits(codec, mute_reg, 0x00, reg); */

	return 0;
}

#endif
static int ak4962_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)

{
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(dai->codec);
	struct ak49xx *core = dev_get_drvdata(dai->codec->dev->parent);

	if (!tx_slot && !rx_slot) {
		pr_err("%s: Invalid\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s(): dai_name = %s DAI-ID %x tx_ch %d rx_ch %d\n"
		 "ak4962->intf_type %d\n",
		 __func__, dai->name, dai->id, tx_num, rx_num,
		 ak4962->intf_type);

	if (ak4962->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak4962->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		ak49xx_init_slimslave(core, core->slim->laddr,
				       tx_num, tx_slot, rx_num, rx_slot);
	return 0;
}

static int ak4962_get_channel_map(struct snd_soc_dai *dai,
				unsigned int *tx_num, unsigned int *tx_slot,
				unsigned int *rx_num, unsigned int *rx_slot)

{
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(dai->codec);
	u32 i = 0;
	struct ak49xx_ch *ch;

	switch (dai->id) {
	case SB1_PB:
	case SB2_PB:
	case SB3_PB:
		if (!rx_slot || !rx_num) {
			pr_err("%s: Invalid rx_slot %p or rx_num %p\n",
				 __func__, rx_slot, rx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &ak4962_p->dai[dai->id].ak49xx_ch_list,
				    list) {
			rx_slot[i++] = ch->ch_num;
		}
		pr_debug("%s: rx_num %d\n", __func__, i);
		*rx_num = i;
		break;
	case SB1_CAP:
	case SB2_CAP:
	case SB3_CAP:
		if (!tx_slot || !tx_num) {
			pr_err("%s: Invalid tx_slot %p or tx_num %p\n",
				 __func__, tx_slot, tx_num);
			return -EINVAL;
		}
		list_for_each_entry(ch, &ak4962_p->dai[dai->id].ak49xx_ch_list,
				    list) {
			tx_slot[i++] = ch->ch_num;
		}
		pr_debug("%s: tx_num %d\n", __func__, i);
		*tx_num = i;
		break;

	default:
		pr_err("%s: Invalid DAI ID %x\n", __func__, dai->id);
		break;
	}
	return 0;
}

static int ak4962_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	pr_debug("%s cmd: %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* ak4962->stream_state = AK4962_SLIMBUS_STREAM_ON; */
		ak4962->pcm_state = AK4962_SLIMBUS_STREAM_ON;
		queue_work(ak4962->workqueue, &ak4962->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* ak4962->stream_state = AK4962_SLIMBUS_STREAM_OFF; */
		ak4962->pcm_state = AK4962_SLIMBUS_STREAM_OFF;
		queue_work(ak4962->workqueue, &ak4962->work);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
#ifdef CONFIG_SND_SOC_ESXXX
static struct snd_soc_dai_ops ak4962_dai_ops = {
	.startup = ak4962_esxxx_startup, /* taiko_startup, */
	.shutdown = ak4962_esxxx_shutdown, /* taiko_shutdown, */
	.hw_params = ak4962_esxxx_hw_params, /* taiko_hw_params, */
	.set_sysclk = ak4962_esxxx_set_sysclk,
	.set_fmt = ak4962_esxxx_set_fmt,
	.set_pll = ak4962_esxxx_set_fll,
	.digital_mute	= ak4962_esxxx_digital_mute,
	.set_channel_map = ak4962_esxxx_set_channel_map,
	/* taiko_set_channel_map, */
	.get_channel_map = ak4962_esxxx_get_channel_map,
	/* taiko_get_channel_map, */
	.trigger = ak4962_esxxx_trigger,
};
#else
static struct snd_soc_dai_ops ak4962_dai_ops = {
	.startup = ak4962_startup,
	.shutdown = ak4962_shutdown,
	.hw_params = ak4962_hw_params,
	.set_sysclk = ak4962_set_sysclk,
	.set_fmt = ak4962_set_fmt,
	.set_pll = ak4962_set_fll,
	.digital_mute	= ak4962_digital_mute,
	.set_channel_map = ak4962_set_channel_map,
	.get_channel_map = ak4962_get_channel_map,
	.trigger = ak4962_trigger,
};
#endif

static struct snd_soc_dai_driver ak4962_dai[] = {
	{
		.name = "ak4962_rx1",
		.id = SB1_PB,
		.playback = {
			.stream_name = "SB1 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_dai_ops,
	},
	{
		.name = "ak4962_tx1",
		.id = SB1_CAP,
		.capture = {
			.stream_name = "SB1 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &ak4962_dai_ops,
	},
	{
		.name = "ak4962_rx2",
		.id = SB2_PB,
		.playback = {
			.stream_name = "SB2 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_dai_ops,
	},
	{
		.name = "ak4962_tx2",
		.id = SB2_CAP,
		.capture = {
			.stream_name = "SB2 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &ak4962_dai_ops,
	},
	{
		.name = "ak4962_tx3",
		.id = SB3_CAP,
		.capture = {
			.stream_name = "SB3 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_dai_ops,
	},
	{
		.name = "ak4962_rx3",
		.id = SB3_PB,
		.playback = {
			.stream_name = "SB3 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_dai_ops,
	},
	{
		.name = "ak4962_vifeedback",
		.id = SB4_VIFEED,
		.capture = {
			.stream_name = "VIfeed",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = AK4962_FORMATS,
			.rate_max = 48000,
			.rate_min = 48000,
			.channels_min = 2,
			.channels_max = 2,
	 },
		.ops = &ak4962_dai_ops,
	},
};

static void ak4962_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	switch (dai->id) {
	case AIF_PORT1:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x10, 0x00);
		/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x70, 0x00); */
		break;
	case AIF_PORT2:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x20, 0x00);
		/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x07, 0x00); */
		break;
	case AIF_PORT3:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x40, 0x00);
		/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x70, 0x00); */
		break;
	case AIF_PORT4:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x80, 0x00);
		/* snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x07, 0x00); */
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		break;
	}
}

static int ak4962_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(dai->codec);
	u8 MDV, BDV, SDV, DLC, CMF, PRD, PFD, PL1S;

	pr_debug("%s: dai_name = %s DAI-ID %x rate %d num_ch %d\n", __func__,
			dai->name, dai->id, params_rate(params),
			params_channels(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		DLC = 1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		DLC = 0;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		DLC = 4;
		break;
	case SNDRV_PCM_FORMAT_A_LAW:
		DLC = 2;
		break;
	case SNDRV_PCM_FORMAT_MU_LAW:
		DLC = 3;
		break;
	default:
		pr_err("%s: invalid RX format %u\n", __func__,
				params_format(params));
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 8000:
		MDV = 0x0E;
		BDV = 0xEF;
		SDV = 0x3F;
		CMF = 0x40;
		PRD = 0x00;
		PFD = 0xEF;
		PL1S = 0x08;		/* Reference clock: BICK*/
		break;
	case 16000:
		MDV = 0x0E;
		BDV = 0x77;
		SDV = 0x3F;
		CMF = 0x24;
		PRD = 0x00;
		PFD = 0xEF;
		PL1S = 0x08;		/* Reference clock: BICK*/
		break;
	case 32000:
		MDV = 0x0E;
		BDV = 0x3B;
		SDV = 0x3F;
		CMF = 0x08;
		PRD = 0x00;
		PFD = 0x00;
		PL1S = 0x00;		/* Reference clock: BICK*/
		break;
	case 44100:
		MDV = 0x09;
		BDV = 0x4F;
		SDV = 0x1F;		/* 32fs */
		CMF = 0x09;
		PRD = 0x01;
		PFD = 0x27;
		PL1S = 0x04;		/* Reference clock: BICK*/
		break;
	case 48000:
		MDV = 0x09;
		BDV = 0x27;
		SDV = 0x3F;
		CMF = 0x0A;
		PRD = 0x01;
		PFD = 0x27;
		PL1S = 0x04;		/* Reference clock: BICK*/
		break;
	case 96000:
		MDV = 0x04;
		BDV = 0x13;
		SDV = 0x3F;
		CMF = 0x0E;
		PRD = 0x03;
		PFD = 0x27;
		PL1S = 0x04;		/* Reference clock: BICK*/
		break;
	case 192000:
		MDV = 0x04;
		BDV = 0x09;
		SDV = 0x3F;
		CMF = 0xF2;
		PRD = 0x07;
		PFD = 0x27;
		PL1S = 0x04;		/* Reference clock: BICK*/
		break;
	default:
		pr_err("%s: Invalid sampling rate %d\n", __func__,
				params_rate(params));
		return -EINVAL;
	}

	switch (dai->id) {
	case AIF_PORT1:
		snd_soc_update_bits(codec, CDCMCLK_DIVIDER, 0xFF, MDV);
		snd_soc_update_bits(codec, MSYNC1_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC1_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, AIF1_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, CODEC_AIF_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x70, 0x10);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x10, 0x10);
		snd_soc_update_bits(codec, CLOCK_MODE_SELECT, 0x7F, CMF);
		snd_soc_update_bits(codec, PLL1_SOURCE_SELECTOR, 0x1F, PL1S);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_L8, 0xFF, PRD);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_L8, 0xFF, PFD);
		if (params_rate(params) == 192000) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x20, 0x20);
		} else {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x20, 0x00);
		}
		break;
	case AIF_PORT2:
		snd_soc_update_bits(codec, MSYNC2_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC2_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, AIF2_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR1, 0x07, 0x02);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x20, 0x20);
		break;
	case AIF_PORT3:
		snd_soc_update_bits(codec, MSYNC3_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC3_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, AIF3_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x70, 0x30);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x30, 0x30);
		break;
	case AIF_PORT4:
		snd_soc_update_bits(codec, MSYNC4_BDV, 0xFF, BDV);
		snd_soc_update_bits(codec, MSYNC4_SDV, 0xFF, SDV);
		snd_soc_update_bits(codec, AIF4_FORMAT, 0x07, DLC);
		snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR2, 0x07, 0x04);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x40, 0x40);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	ak4962->dai[dai->id].rate = params_rate(params);
	return 0;
}

static int ak4962_i2s_set_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(dai->codec); */
	u8 msn, dif, bckp, thrnc;

	pr_debug("%s\n", __func__);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		msn = 0x20;		/* CODEC is master */
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		msn = 0x00;		/* CODEC is slave */
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_B:
		dif = 0x30;		/* PCM Long Frame */
		thrnc = 0x00;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		dif = 0x20;		/* PCM Short Frame */
		thrnc = 0x00;
		break;
	case SND_SOC_DAIFMT_I2S:
		dif = 0x00;
		thrnc = 0x00;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dif = 0x10;
		thrnc = 0x40;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		bckp = 0x00;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		bckp = 0x08;
		break;
	default:
		return -EINVAL;
	}

	switch (dai->id) {
	case AIF_PORT1:
		snd_soc_update_bits(dai->codec, MSYNC1_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, AIF1_FORMAT, 0x38, dif | bckp);
		snd_soc_update_bits(dai->codec, CODEC_AIF_FORMAT, 0x40, thrnc);
		break;
	case AIF_PORT2:
		snd_soc_update_bits(dai->codec, MSYNC2_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, AIF2_FORMAT, 0x38, dif | bckp);
		snd_soc_update_bits(dai->codec, CODEC_AIF_FORMAT, 0x40, thrnc);
		break;
	case AIF_PORT3:
		snd_soc_update_bits(dai->codec, MSYNC3_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, AIF3_FORMAT, 0x38, dif | bckp);
		snd_soc_update_bits(dai->codec, CODEC_AIF_FORMAT, 0x40, thrnc);
		break;
	case AIF_PORT4:
		snd_soc_update_bits(dai->codec, MSYNC4_MSN_CKS, 0x20, msn);
		snd_soc_update_bits(dai->codec, AIF4_FORMAT, 0x38, dif | bckp);
		snd_soc_update_bits(dai->codec, CODEC_AIF_FORMAT, 0x40, thrnc);
		break;
	default:
		pr_err("%s: Invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	return 0;
}

static int ak4962_i2s_set_fll(struct snd_soc_dai *dai, int id, int src,
			  unsigned int freq_in, unsigned int freq_out)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int ak4962_i2s_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
/* struct snd_soc_codec *codec = codec_dai->codec; */

#if 0   /* temp del by lvrongguo @20151203 begin */
	int mute_reg;
	int reg;

	switch (codec_dai->id) {
	case 1:
		mute_reg = 0x00;
		break;
	case 2:
		mute_reg = 0x01;
		break;
	default:
		return -EINVAL;
	}

	if (mute)
		reg = 1;
	else
		reg = 0;
#endif    /* temp del by lvrongguo @20151203 end */

/* snd_soc_update_bits(codec, mute_reg, 0x00, reg); */

	return 0;
}

static struct snd_soc_dai_ops ak4962_i2s_dai_ops = {
	.shutdown = ak4962_i2s_shutdown,
	.hw_params = ak4962_i2s_hw_params,
	.set_sysclk = ak4962_i2s_set_sysclk,
	.set_fmt = ak4962_i2s_set_fmt,
	.set_pll = ak4962_i2s_set_fll,
	.digital_mute	= ak4962_i2s_digital_mute,
	.trigger = ak4962_trigger,
};

static struct snd_soc_dai_driver ak4962_i2s_dai[] = {
	{
		.name = "ak4962_aif1",
		.id = AIF_PORT1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 8,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &ak4962_i2s_dai_ops,
	},
	{
		.name = "ak4962_aif2",
		.id = AIF_PORT2,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF2 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_i2s_dai_ops,
	},
	{
		.name = "ak4962_aif3",
		.id = AIF_PORT3,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_i2s_dai_ops,
	},
	{
		.name = "ak4962_aif4",
		.id = AIF_PORT4,
		.playback = {
			.stream_name = "AIF4 Playback",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.capture = {
			.stream_name = "AIF4 Capture",
			.rates = AK4962_RATES,
			.formats = AK4962_FORMATS_AIF,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4962_i2s_dai_ops,
	},
};

static const struct ak4962_reg_val anc_room_val[] = {
		{	0x18E, 0x19	},
		{	0x18F, 0xDE	},
		{	0x18D, 0x1F	},
		{	0x191, 0x19	},
		{	0x192, 0xDE	},
		{	0x190, 0x1F	},
		{	0x194, 0x07	},
		{	0x195, 0x4F	},
		{	0x193, 0x04	},
		{	0x197, 0xA8	},
		{	0x198, 0xA8	},
		{	0x100, 0x08	},
		{	0x101, 0x00	},
		{	0x102, 0xC4	},
		{	0x103, 0x80	},
		{	0x104, 0x99	},
		{	0x105, 0xFF	},
		{	0x106, 0x94	},
		{	0x107, 0xF9	},
		{	0x108, 0xE8	},
		{	0x109, 0xFF	},
		{	0x10A, 0xEC	},
		{	0x10B, 0x32	},
		{	0x10C, 0x21	},
		{	0x10D, 0x37	},
		{	0x10E, 0x47	},
		{	0x10F, 0x42	},
		{	0x110, 0xB2	},
		{	0x111, 0xE8	},
		{	0x112, 0x73	},
		{	0x113, 0x10	},
		{	0x114, 0xAB	},
		{	0x115, 0x1E	},
		{	0x116, 0x01	},
		{	0x117, 0xE5	},
		{	0x118, 0x77	},
		{	0x119, 0xE1	},
		{	0x11A, 0xFE	},
		{	0x11B, 0x1A	},
		{	0x11C, 0x89	},
		{	0x11D, 0x00	},
		{	0x11E, 0x00	},
		{	0x11F, 0x00	},
		{	0x120, 0x00	},
		{	0x121, 0x1F	},
		{	0x122, 0xFB	},
		{	0x123, 0x4F	},
		{	0x124, 0x55	},
		{	0x125, 0x00	},
		{	0x126, 0x00	},
		{	0x127, 0x00	},
		{	0x128, 0x00	},
		{	0x129, 0x1F	},
		{	0x12A, 0xFF	},
		{	0x12B, 0xC8	},
		{	0x12C, 0xBA	},
		{	0x12D, 0xC0	},
		{	0x12E, 0x12	},
		{	0x12F, 0x27	},
		{	0x130, 0xFF	},
		{	0x131, 0x1F	},
		{	0x132, 0xEE	},
		{	0x133, 0x25	},
		{	0x134, 0x2D	},
		{	0x135, 0x3F	},
		{	0x136, 0xED	},
		{	0x137, 0xD7	},
		{	0x138, 0x76	},
		{	0x139, 0xE0	},
		{	0x13A, 0x12	},
		{	0x13B, 0x11	},
		{	0x13C, 0x8F	},
		{	0x13D, 0x1F	},
		{	0x13E, 0xA5	},
		{	0x13F, 0x22	},
		{	0x140, 0x8B	},
		{	0x141, 0xC1	},
		{	0x142, 0x27	},
		{	0x143, 0x4C	},
		{	0x144, 0x57	},
		{	0x145, 0x1F	},
		{	0x146, 0x43	},
		{	0x147, 0x47	},
		{	0x148, 0xBE	},
		{	0x149, 0x3E	},
		{	0x14A, 0xD8	},
		{	0x14B, 0xB3	},
		{	0x14C, 0xA9	},
		{	0x14D, 0xE1	},
		{	0x14E, 0x17	},
		{	0x14F, 0x95	},
		{	0x150, 0xB8	},
		{	0x151, 0x20	},
		{	0x152, 0x00	},
		{	0x153, 0x00	},
		{	0x154, 0x00	},
		{	0x155, 0x00	},
		{	0x156, 0x00	},
		{	0x157, 0x00	},
		{	0x158, 0x00	},
		{	0x159, 0x00	},
		{	0x15A, 0x00	},
		{	0x15B, 0x00	},
		{	0x15C, 0x00	},
		{	0x15D, 0x00	},
		{	0x15E, 0x00	},
		{	0x15F, 0x00	},
		{	0x160, 0x20	},
		{	0x161, 0x00	},
		{	0x162, 0x00	},
		{	0x163, 0x00	},
		{	0x164, 0x00	},
		{	0x165, 0x00	},
		{	0x166, 0x00	},
		{	0x167, 0x00	},
		{	0x168, 0x00	},
		{	0x169, 0x00	},
		{	0x16A, 0x00	},
		{	0x16B, 0x00	},
		{	0x16C, 0x00	},
		{	0x16D, 0x00	},
		{	0x16E, 0x00	},
		{	0x16F, 0x20	},
		{	0x170, 0x00	},
		{	0x171, 0x00	},
		{	0x172, 0x00	},
		{	0x173, 0x00	},
		{	0x174, 0x00	},
		{	0x175, 0x00	},
		{	0x176, 0x00	},
		{	0x177, 0x00	},
		{	0x178, 0x00	},
		{	0x179, 0x00	},
		{	0x17A, 0x00	},
		{	0x17B, 0x00	},
		{	0x17C, 0x00	},
		{	0x17D, 0x00	},
		{	0x17E, 0x20	},
		{	0x17F, 0x00	},
		{	0x180, 0x00	},
		{	0x181, 0x00	},
		{	0x182, 0x00	},
		{	0x183, 0x00	},
		{	0x184, 0x00	},
		{	0x185, 0x00	},
		{	0x186, 0x00	},
		{	0x187, 0x00	},
		{	0x188, 0x00	},
		{	0x189, 0x00	},
		{	0x18A, 0x00	},
		{	0x18B, 0x00	},
		{	0x18C, 0x00	},

		{	0x199, 0x1B	},
		{	0x199, 0x0B	},
		{	0x196, 0x08	},
		{	0x199, 0x1B	},
		{	0x199, 0x0B	},
		{	0x199, 0x0A	},
		{	0x199, 0x1A	},
		{	0x199, 0x0A	},
};

static const struct ak4962_reg_val anc_airplane_val[] = {
		{	0x18E, 0x19	},
		{	0x18F, 0xDE	},
		{	0x18D, 0x1F	},
		{	0x191, 0x19	},
		{	0x192, 0xDE	},
		{	0x190, 0x1F	},
		{	0x194, 0x07	},
		{	0x195, 0x4F	},
		{	0x193, 0x04	},
		{	0x197, 0xA8	},
		{	0x198, 0xA8	},
		{	0x100, 0x08	},
		{	0x101, 0x00	},
		{	0x102, 0xB3	},
		{	0x103, 0x7D	},
		{	0x104, 0x0F	},
		{	0x105, 0xFF	},
		{	0x106, 0x9F	},
		{	0x107, 0x57	},
		{	0x108, 0xB3	},
		{	0x109, 0xFF	},
		{	0x10A, 0xEC	},
		{	0x10B, 0x6D	},
		{	0x10C, 0x77	},
		{	0x10D, 0x37	},
		{	0x10E, 0x8E	},
		{	0x10F, 0x74	},
		{	0x110, 0x7C	},
		{	0x111, 0xE8	},
		{	0x112, 0x32	},
		{	0x113, 0x49	},
		{	0x114, 0x4B	},
		{	0x115, 0x1F	},
		{	0x116, 0xFE	},
		{	0x117, 0xD0	},
		{	0x118, 0xFB	},
		{	0x119, 0xC0	},
		{	0x11A, 0x1A	},
		{	0x11B, 0x5A	},
		{	0x11C, 0x6C	},
		{	0x11D, 0x1F	},
		{	0x11E, 0xE6	},
		{	0x11F, 0xE1	},
		{	0x120, 0x36	},
		{	0x121, 0x3F	},
		{	0x122, 0xE5	},
		{	0x123, 0xA5	},
		{	0x124, 0x94	},
		{	0x125, 0xE0	},
		{	0x126, 0x1A	},
		{	0x127, 0x4D	},
		{	0x128, 0xCF	},
		{	0x129, 0x1F	},
		{	0x12A, 0xFD	},
		{	0x12B, 0x2C	},
		{	0x12C, 0x52	},
		{	0x12D, 0xC0	},
		{	0x12E, 0x7E	},
		{	0x12F, 0x6C	},
		{	0x130, 0x2D	},
		{	0x131, 0x1F	},
		{	0x132, 0x85	},
		{	0x133, 0x30	},
		{	0x134, 0x16	},
		{	0x135, 0x3F	},
		{	0x136, 0x81	},
		{	0x137, 0x93	},
		{	0x138, 0xD3	},
		{	0x139, 0xE0	},
		{	0x13A, 0x7D	},
		{	0x13B, 0xA3	},
		{	0x13C, 0x99	},
		{	0x13D, 0x1F	},
		{	0x13E, 0x9B	},
		{	0x13F, 0x86	},
		{	0x140, 0x58	},
		{	0x141, 0xC1	},
		{	0x142, 0x40	},
		{	0x143, 0x28	},
		{	0x144, 0xD1	},
		{	0x145, 0x1F	},
		{	0x146, 0x34	},
		{	0x147, 0x01	},
		{	0x148, 0x3F	},
		{	0x149, 0x3E	},
		{	0x14A, 0xBF	},
		{	0x14B, 0xD7	},
		{	0x14C, 0x2F	},
		{	0x14D, 0xE1	},
		{	0x14E, 0x30	},
		{	0x14F, 0x78	},
		{	0x150, 0x69	},
		{	0x151, 0x20	},
		{	0x152, 0x00	},
		{	0x153, 0x00	},
		{	0x154, 0x00	},
		{	0x155, 0x00	},
		{	0x156, 0x00	},
		{	0x157, 0x00	},
		{	0x158, 0x00	},
		{	0x159, 0x00	},
		{	0x15A, 0x00	},
		{	0x15B, 0x00	},
		{	0x15C, 0x00	},
		{	0x15D, 0x00	},
		{	0x15E, 0x00	},
		{	0x15F, 0x00	},
		{	0x160, 0x20	},
		{	0x161, 0x00	},
		{	0x162, 0x00	},
		{	0x163, 0x00	},
		{	0x164, 0x00	},
		{	0x165, 0x00	},
		{	0x166, 0x00	},
		{	0x167, 0x00	},
		{	0x168, 0x00	},
		{	0x169, 0x00	},
		{	0x16A, 0x00	},
		{	0x16B, 0x00	},
		{	0x16C, 0x00	},
		{	0x16D, 0x00	},
		{	0x16E, 0x00	},
		{	0x16F, 0x20	},
		{	0x170, 0x00	},
		{	0x171, 0x00	},
		{	0x172, 0x00	},
		{	0x173, 0x00	},
		{	0x174, 0x00	},
		{	0x175, 0x00	},
		{	0x176, 0x00	},
		{	0x177, 0x00	},
		{	0x178, 0x00	},
		{	0x179, 0x00	},
		{	0x17A, 0x00	},
		{	0x17B, 0x00	},
		{	0x17C, 0x00	},
		{	0x17D, 0x00	},
		{	0x17E, 0x20	},
		{	0x17F, 0x00	},
		{	0x180, 0x00	},
		{	0x181, 0x00	},
		{	0x182, 0x00	},
		{	0x183, 0x00	},
		{	0x184, 0x00	},
		{	0x185, 0x00	},
		{	0x186, 0x00	},
		{	0x187, 0x00	},
		{	0x188, 0x00	},
		{	0x189, 0x00	},
		{	0x18A, 0x00	},
		{	0x18B, 0x00	},
		{	0x18C, 0x00	},

		{	0x199, 0x1B	},
		{	0x199, 0x0B	},
		{	0x196, 0x08	},
		{	0x199, 0x1B	},
		{	0x199, 0x0B	},
		{	0x199, 0x0A	},
		{	0x199, 0x1A	},
		{	0x199, 0x0A	},
};

static const struct ak4962_reg_val anc_trainbus_val[] = {
		{	0x18E, 0x19	},
		{	0x18F, 0xDE	},
		{	0x18D, 0x1F	},
		{	0x191, 0x19	},
		{	0x192, 0xDE	},
		{	0x190, 0x1F	},
		{	0x194, 0x07	},
		{	0x195, 0x4F	},
		{	0x193, 0x04	},
		{	0x197, 0xA8	},
		{	0x198, 0xA8	},
		{	0x100, 0x08	},
		{	0x101, 0x00	},
		{	0x102, 0x7A	},
		{	0x103, 0x38	},
		{	0x104, 0x3B	},
		{	0x105, 0xFF	},
		{	0x106, 0xC9	},
		{	0x107, 0x7F	},
		{	0x108, 0x2C	},
		{	0x109, 0xFF	},
		{	0x10A, 0xEE	},
		{	0x10B, 0x80	},
		{	0x10C, 0x58	},
		{	0x10D, 0x38	},
		{	0x10E, 0x71	},
		{	0x10F, 0xD2	},
		{	0x110, 0x35	},
		{	0x111, 0xE7	},
		{	0x112, 0x5B	},
		{	0x113, 0xF6	},
		{	0x114, 0x0C	},
		{	0x115, 0x1F	},
		{	0x116, 0xFF	},
		{	0x117, 0x43	},
		{	0x118, 0x32	},
		{	0x119, 0xC0	},
		{	0x11A, 0x06	},
		{	0x11B, 0x6B	},
		{	0x11C, 0xFF	},
		{	0x11D, 0x1F	},
		{	0x11E, 0xFA	},
		{	0x11F, 0x53	},
		{	0x120, 0x09	},
		{	0x121, 0x3F	},
		{	0x122, 0xF9	},
		{	0x123, 0x94	},
		{	0x124, 0x01	},
		{	0x125, 0xE0	},
		{	0x126, 0x06	},
		{	0x127, 0x69	},
		{	0x128, 0xC5	},
		{	0x129, 0x1F	},
		{	0x12A, 0xBE	},
		{	0x12B, 0x73	},
		{	0x12C, 0xD4	},
		{	0x12D, 0xC2	},
		{	0x12E, 0x0C	},
		{	0x12F, 0xA3	},
		{	0x130, 0xF6	},
		{	0x131, 0x1E	},
		{	0x132, 0x35	},
		{	0x133, 0x2A	},
		{	0x134, 0xCA	},
		{	0x135, 0x3D	},
		{	0x136, 0xF3	},
		{	0x137, 0x5C	},
		{	0x138, 0x0A	},
		{	0x139, 0xE2	},
		{	0x13A, 0x0C	},
		{	0x13B, 0x61	},
		{	0x13C, 0x62	},
		{	0x13D, 0x1F	},
		{	0x13E, 0x61	},
		{	0x13F, 0x03	},
		{	0x140, 0x6C	},
		{	0x141, 0xC1	},
		{	0x142, 0xAF	},
		{	0x143, 0x3B	},
		{	0x144, 0x5F	},
		{	0x145, 0x1E	},
		{	0x146, 0xFF	},
		{	0x147, 0x55	},
		{	0x148, 0xD7	},
		{	0x149, 0x3E	},
		{	0x14A, 0x50	},
		{	0x14B, 0xC4	},
		{	0x14C, 0xA1	},
		{	0x14D, 0xE1	},
		{	0x14E, 0x9F	},
		{	0x14F, 0xA6	},
		{	0x150, 0xBD	},
		{	0x151, 0x20	},
		{	0x152, 0x00	},
		{	0x153, 0x00	},
		{	0x154, 0x00	},
		{	0x155, 0x00	},
		{	0x156, 0x00	},
		{	0x157, 0x00	},
		{	0x158, 0x00	},
		{	0x159, 0x00	},
		{	0x15A, 0x00	},
		{	0x15B, 0x00	},
		{	0x15C, 0x00	},
		{	0x15D, 0x00	},
		{	0x15E, 0x00	},
		{	0x15F, 0x00	},
		{	0x160, 0x20	},
		{	0x161, 0x00	},
		{	0x162, 0x00	},
		{	0x163, 0x00	},
		{	0x164, 0x00	},
		{	0x165, 0x00	},
		{	0x166, 0x00	},
		{	0x167, 0x00	},
		{	0x168, 0x00	},
		{	0x169, 0x00	},
		{	0x16A, 0x00	},
		{	0x16B, 0x00	},
		{	0x16C, 0x00	},
		{	0x16D, 0x00	},
		{	0x16E, 0x00	},
		{	0x16F, 0x20	},
		{	0x170, 0x00	},
		{	0x171, 0x00	},
		{	0x172, 0x00	},
		{	0x173, 0x00	},
		{	0x174, 0x00	},
		{	0x175, 0x00	},
		{	0x176, 0x00	},
		{	0x177, 0x00	},
		{	0x178, 0x00	},
		{	0x179, 0x00	},
		{	0x17A, 0x00	},
		{	0x17B, 0x00	},
		{	0x17C, 0x00	},
		{	0x17D, 0x00	},
		{	0x17E, 0x20	},
		{	0x17F, 0x00	},
		{	0x180, 0x00	},
		{	0x181, 0x00	},
		{	0x182, 0x00	},
		{	0x183, 0x00	},
		{	0x184, 0x00	},
		{	0x185, 0x00	},
		{	0x186, 0x00	},
		{	0x187, 0x00	},
		{	0x188, 0x00	},
		{	0x189, 0x00	},
		{	0x18A, 0x00	},
		{	0x18B, 0x00	},
		{	0x18C, 0x00	},

		{	0x199, 0x1B	},
		{	0x199, 0x0B	},
		{	0x196, 0x08	},
		{	0x199, 0x1B	},
		{	0x199, 0x0B	},
		{	0x199, 0x0A	},
		{	0x199, 0x1A	},
		{	0x199, 0x0A	},
};

static const char * const mic1_power_level[] = {
	"V2.8", "V2.5", "V1.8", "AVDD1"
};

static const char * const mic2_power_level[] = {
	"V2.8", "V2.5", "V1.8", "N/A"
};

static const struct soc_enum mic_1_power_level =
	SOC_ENUM_SINGLE(MIC_POWER_LEVEL, 0, 4, mic1_power_level);

static const struct soc_enum mic_2_power_level =
	SOC_ENUM_SINGLE(MIC_POWER_LEVEL, 2, 4, mic2_power_level);

/*
 * MIC-Amp gain control:
 * from 0 to 30 dB in 3 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_gain_tlv, 0, 300, 0);

/*
 * MIC-Amp gain control:
 * from 8 to 20 dB in 4 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic3_gain_tlv, 800, 400, 0);

/*
 * MIC-Amp gain control:
 * from -6 to 0 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(ain_att_tlv, -600, 600, 0);

/*
 * Digital output volume control:
 * from -12.5 to 3 dB in 0.5 dB steps (mute instead of -12.5 dB)
 */
static DECLARE_TLV_DB_SCALE(dout_vol_tlv, -1250, 50, 1);

/*
 * HP-Amp volume control:
 * from -22 to 6 dB in 2 dB steps (mute instead of -22 dB)
 */
static DECLARE_TLV_DB_SCALE(hp_out_tlv, -2200, 200, 1);

/*
 * Lineout1 volume control:
 * from -7.5 to 3 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(lineout1_tlv, -750, 150, 0);

/*
 * Lineout2 volume control:
 * from -7.5 to 3 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(lineout2_tlv, -750, 150, 0);

/*
 * Receiver volume control:
 * from -7.5 to 3 dB in 1.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(receiver_tlv, -750, 150, 0);

/* ADC HPF Filter Enable */
static const char * const adc_hpf_enable[] = {
	"hpf_on", "hpf_off"
};

static const struct soc_enum ad1_hpf_enable =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 0, 2, adc_hpf_enable);

static const struct soc_enum ad2_hpf_enable =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 0, 2, adc_hpf_enable);

/* cut of frequency for high pass filter*/
static const char * const hp_cf_text[] = {
	"fs@3.4Hz=44.1kHz", "fs@0.92Hz=44.1kHz"
};

static const struct soc_enum ad1_hp_cf =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 1, 2, hp_cf_text);

static const struct soc_enum ad2_hp_cf =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 1, 2, hp_cf_text);

/* ADC Digital Filter Setting */
static const char * const adc_df_text[] = {
	"sharp_roll_off", "short_delay_sharp_roll_off"
};

static const struct soc_enum ad1_digital_filter =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 4, 2, adc_df_text);

static const struct soc_enum ad2_digital_filter =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 4, 2, adc_df_text);

/* DAC Filter Bypass */
static const char * const dac_df_thr[] = {
	"df_filter", "df_thr"
};

static const struct soc_enum da1_filter_through =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 3, 2, dac_df_thr);

static const struct soc_enum da2_filter_through =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_2, 3, 2, dac_df_thr);

/* DAC Digital Filter Setting */
static const char * const dac_df_text[] = {
	"sharp_roll_off", "slow_roll_off",
	"short_delay_sharp_roll_off", "short_delay_slow_roll_off"
};

static const struct soc_enum da1_digital_filter =
	SOC_ENUM_SINGLE(DIGITAL_FILTER_SELECT_1, 6, 4, dac_df_text);

/* DAC Mixer PGA Setting */
static const char * const dac_mix_pga_text[] = {
	"bypass", "half", "minus", "half_minus"
};

static const struct soc_enum da1_lch_mix_pga =
	SOC_ENUM_SINGLE(DAC1_MONO_MIXING, 2, 4, dac_mix_pga_text);

static const struct soc_enum da1_rch_mix_pga =
	SOC_ENUM_SINGLE(DAC1_MONO_MIXING, 6, 4, dac_mix_pga_text);

static const struct soc_enum da2_lch_mix_pga =
	SOC_ENUM_SINGLE(DAC2_MONO_MIXING, 2, 4, dac_mix_pga_text);

static const struct soc_enum da2_rch_mix_pga =
	SOC_ENUM_SINGLE(DAC2_MONO_MIXING, 6, 4, dac_mix_pga_text);

/* Charge Pump1 clock setting */
static const char * const cp1_clock_sel_text[] = {
	"500kHz", "250kHz", "125kHz", "62.5kHz"
};

static const struct soc_enum cp1_clock_sel =
	SOC_ENUM_SINGLE(CHARGE_PUMP_1_SETTING_1, 0, 4, cp1_clock_sel_text);

/*
 * Mixer A PGA gain control:
 * from -6 to 0 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(mixer_gain_tlv, -600, 600, 0);

/* Mixer A SWAP Setting */
static const char * const mixer_swap_text[] = {
	"through", "lin", "rin", "swap"
};

static const struct soc_enum mixer_a1_swap =
	SOC_ENUM_SINGLE(MIXER_A_CONTROL, 0, 4, mixer_swap_text);

static const struct soc_enum mixer_a2_swap =
	SOC_ENUM_SINGLE(MIXER_A_CONTROL, 2, 4, mixer_swap_text);

static const struct soc_enum mixer_b1_swap =
	SOC_ENUM_SINGLE(MIXER_B_CONTROL, 0, 4, mixer_swap_text);

static const struct soc_enum mixer_b2_swap =
	SOC_ENUM_SINGLE(MIXER_B_CONTROL, 2, 4, mixer_swap_text);

static const struct soc_enum mixer_c1_swap =
	SOC_ENUM_SINGLE(MIXER_C_CONTROL, 0, 4, mixer_swap_text);

static const struct soc_enum mixer_c2_swap =
	SOC_ENUM_SINGLE(MIXER_C_CONTROL, 2, 4, mixer_swap_text);

static const struct soc_enum mixer_d1_swap =
	SOC_ENUM_SINGLE(MIXER_D_CONTROL, 0, 4, mixer_swap_text);

static const struct soc_enum mixer_d2_swap =
	SOC_ENUM_SINGLE(MIXER_D_CONTROL, 2, 4, mixer_swap_text);

/* SRC Output Sync Domain Setting */
static const char * const sync_domain_text[] = {
	"n/a", "aif1", "aif2", "aif3", "aif4", "slimbus",
	"dsp16k", "dsp24k", "dsp32k", "dsp96k", "dsp192k",
};

static const struct soc_enum srcao_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR7, 4, 11, sync_domain_text);

static const struct soc_enum srcbo_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR7, 0, 11, sync_domain_text);

static const struct soc_enum srcco_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR8, 4, 11, sync_domain_text);

static const struct soc_enum srcdo_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR8, 0, 11, sync_domain_text);

static const struct soc_enum dsp_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR5, 0, 11, sync_domain_text);

static const struct soc_enum dspo2_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR6, 4, 11, sync_domain_text);

static const struct soc_enum dspo4_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR6, 0, 11, sync_domain_text);

static const struct soc_enum cdc_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR3, 0, 11, sync_domain_text);

static const struct soc_enum port2_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR1, 0, 11, sync_domain_text);

static const struct soc_enum port3_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR2, 4, 11, sync_domain_text);

static const struct soc_enum port4_sync_domain =
	SOC_ENUM_SINGLE(SYNC_DOMAIN_SELECTOR2, 0, 11, sync_domain_text);

static const char * const hp_power_mode_text[] = {
	"HiFi", "LowPower",
};

static const struct soc_enum hp_power_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_power_mode_text), hp_power_mode_text);	/* rev0.16 */

/* rev0.16 */
static int ak4962_get_hp_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->low_power_mode;

	return 0;
}

static int ak4962_set_hp_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	int val, hp_gain;
	int i;

	pr_info("%s: [ak4962] 1. power_select_vol_offset = %d\n", __func__, ak4962->power_select_vol_offset);

	ak4962->low_power_mode = ucontrol->value.enumerated.item[0];

	if (ak4962->low_power_mode == 0) {
		/*High Performance mode*/
		ak4962->power_select_vol_offset = HIGH_PERFORMANCE_VOL_OFFSET;
		val = snd_soc_read(codec, POWER_MANAGEMENT_9);	/* PMHPL/R read */
		if (val & 0x03) {
			/*playing*/
			hp_gain = snd_soc_read(codec, HP_VOLUME_CONTROL);
			pr_info("%s: hp_gain = %d\n", __func__, hp_gain);
			for (i = hp_gain; i >= 0; i--) {
				pr_info("%s: down: i = %d\n", __func__, i);
				snd_soc_update_bits(codec, HP_VOLUME_CONTROL, 0x0F, i);
			}
			snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x00);	/* PMHPL/R=0 */
			snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x10, 0x00);	/* LPMODE=0 */
			snd_soc_update_bits(codec, MODE_CONTROL, 0x40, 0x00);		/* DSMLP1=0 */
			snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03);	/* PMHPL/R=1 */
			usleep_range(26000, 27000);
			for (i = 0;
				i <= ak4962->hp_avol_org +
					ak4962->hp_avol_offset +
					ak4962->power_select_vol_offset;
				i++) {
				pr_info("%s: up: i = %d\n", __func__, i);
				snd_soc_update_bits(codec, HP_VOLUME_CONTROL, 0x0F, i);
			}
		}
	} else {
	/* Low Power mode */
		ak4962->power_select_vol_offset = LOW_POWER_VOL_OFFSET;
		val = snd_soc_read(codec, POWER_MANAGEMENT_9);	/* PMHPL/R read */
		if (val & 0x03) {
			/* playing */
			hp_gain = snd_soc_read(codec, HP_VOLUME_CONTROL);
			pr_info("%s: hp_gain = %d\n", __func__, hp_gain);
			for (i = hp_gain; i >= 0; i--) {
				pr_info("%s: down: i = %d\n", __func__, i);
				snd_soc_update_bits(codec, HP_VOLUME_CONTROL, 0x0F, i);
			}
			snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x00);	/* PMHPL/R=0 */
			snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x10, 0x10);	/* LPMODE=1 */
			snd_soc_update_bits(codec, MODE_CONTROL, 0x40, 0x40);		/* DSMLP1=1 */
			snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03);	/* PMHPL/R=1 */
			usleep_range(26000, 27000);
			for (i = 0;
				i <= ak4962->hp_avol_org +
					ak4962->hp_avol_offset +
					ak4962->power_select_vol_offset;
				i++) {
				pr_info("%s: up: i = %d\n", __func__, i);
				snd_soc_update_bits(codec, HP_VOLUME_CONTROL, 0x0F, i);
			}
		}
	}

	pr_info("%s: [ak4962] 2. power_select_vol_offset = %d\n", __func__, ak4962->power_select_vol_offset);

	return 0;
}
/* rev0.16 end */

static const char *const dsp_status_text[] = {"off", "on"};

static const struct soc_enum dsp_downlink_status_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dsp_status_text), dsp_status_text);

static int ak4962_get_dsp_downlink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->ak4962_dsp_downlink_status;

	return 0;
}

static int ak4962_set_dsp_downlink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret = 0;

	pr_info("%s: status = %d\n", __func__, status);

	if (ak4962_dsp_mode >= DSP_MODE_SOUND_RECORD &&
			ak4962_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {

		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x01;
		val[5] = 0x25; /* 0x1d; */
		val[6] = 0x00;
		val[7] = status;
		val[8] = 0;

		ret = ak49xx_run_ram_write(codec->control_data, val);
	}
	if (ret == 0) {
		ak4962->ak4962_dsp_downlink_status = status;
	}

	if (ak4962->ak4962_dsp_uplink_status == 0 &&
		ak4962->ak4962_dsp_downlink_status == 0) {

		ak4962_dsp_mode = DSP_MODE_OFF;
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
		ak4962->state = AK4962_IDLE;
	}

	return ret;
}

static const struct soc_enum dsp_uplink_status_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dsp_status_text), dsp_status_text);

static int ak4962_get_dsp_uplink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->ak4962_dsp_uplink_status;

	return 0;
}

static int ak4962_set_dsp_uplink_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret = 0;

	pr_info("%s: status = %d\n", __func__, status);

	if (ak4962_dsp_mode >= DSP_MODE_SOUND_RECORD &&
			ak4962_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {

		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x01;
		val[5] = 0x13; /* 0x10; */
		val[6] = 0x00;
		val[7] = status;
		val[8] = 0;

		ret = ak49xx_run_ram_write(codec->control_data, val);
	}
	if (ret == 0) {
		ak4962->ak4962_dsp_uplink_status = status;
	}

	if (ak4962->ak4962_dsp_uplink_status == 0 &&
		ak4962->ak4962_dsp_downlink_status == 0) {

		ak4962_dsp_mode = DSP_MODE_OFF;
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
		ak4962->state = AK4962_IDLE;
	}

	return ret;
}

static const char * const dsp_mode_text[] = {
	"off",
	"narrow_handset", "narrow_headset", "narrow_headphone", "narrow_handsfree",
	"wide_handset", "wide_headset", "wide_headphone", "wide_handsfree",
	"voice_recognition",
	"sound_record", "voice_record",
	"interview_record",
	"music_speaker",
	"barge_in",
	"karaoke_cave", "karaoke_cloister", "karaoke_live", "karaoke_room",
	"karaoke_theater", "karaoke_valley", "karaoke_bypass",
	"karaoke_spk_room", "karaoke_spk_theater", "karaoke_spk_live",
	"beam_record",
	"smart_ringtone"
};
#if 0
static const u16 ram_table[] = {
	0x0010, 0x0020, 0x0030, 0x0040,	/* NARROW_MODE */
	0x1050, 0x1060, 0x1070,	0x0080,	/* WIDE_MODE */
	0x2090,							/* Voice Recognition */
	0x30A0, 0x30B0,					/* Sound Voice Record */
	0x40C0,							/* Music Speaker */
	0x5000,							/* Barge-in */
	0x60D0, 0x60E0, 0x60F0,		/* Karaoke */
	0x6100, 0x6110, 0x6120, 0x6130,	/* Karaoke */
	0x7140,							/* Beam Record */
	0x8150							/* Smart Ringtone */
};
#endif
static const u16 ram_table[] = {
	0x0000, 0x0010, 0x0020, 0x0030,	/* NARROW_MODE */
	0x1040, 0x1050, 0x1060, 0x1070,	       /* WIDE_MODE */
	0x2080,							/* Voice Recognition */
	0x3090, 0x30A0,			      /* Sound Voice Record */
	0x40B0,                         /* Interview Record */
	0x50C0,						/* Music Speaker */
	0x60D0,							/* Barge-in */
	0x70E0, 0x70F0, 0x7100, 0x7110,
	0x7120, 0x7130, 0x7140,		/* Karaoke */
	0x8153, 0x8163, 0x8173,			/* Karaoke SPK */
	0x9180,							/* Beam Record */
	0xA190						/* Smart Ringtone */
};

static const struct soc_enum dsp_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dsp_mode_text), dsp_mode_text),
};

static int ak4962_get_dsp_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ak4962_dsp_mode;
	return 0;
}

static int ak4962_set_dsp_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *ak49xx = codec->control_data;
	struct ak49xx_core_resource *core_res = &ak49xx->core_res;
	int new_dsp_mode = ucontrol->value.integer.value[0];
	int interface = ak49xx_get_intf_type();
	int ret = 0;
	int dsp_sync_domain = 0;
	int reg_srcab, reg_srccd;

	u8  pram_index, cram_index, ex_pram_index, ex_cram_index;
	u8	aram_index, ex_aram_index;
	size_t pram_size = 0, cram_size = 0, aram_size = 0;
	u8	crc[2];
	u8 *pram_fwbuf = 0, *cram_fwbuf = 0, *aram_fwbuf = 0;
	u8	val[9];


	if (ak4962_dsp_mode == new_dsp_mode) {
		return 0;
	}

	if (new_dsp_mode == DSP_MODE_OFF &&
		(ak4962->ak4962_dsp_downlink_status == 1 ||
		 ak4962->ak4962_dsp_uplink_status == 1)
		) {
		return 0;
	}

	if (new_dsp_mode != DSP_MODE_OFF) {
		pram_index = ram_table[new_dsp_mode - 1] >> 12;
		cram_index = (ram_table[new_dsp_mode - 1] >> 4) % 0x100;
		aram_index = ram_table[new_dsp_mode - 1] % 0x10;
		pr_err("%s: pram_index = %d, cram_index =%d, aram_index = %d\n",
				__func__, pram_index, cram_index, aram_index);

		if (ak4962_dsp_mode != DSP_MODE_OFF) {
			ex_pram_index = ram_table[ak4962_dsp_mode - 1] >> 12;
			ex_cram_index = (ram_table[ak4962_dsp_mode - 1] >> 4) % 0x100;
			ex_aram_index = ram_table[ak4962_dsp_mode - 1] % 0x10;
			dsp_sync_domain = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR5);
			if (dsp_sync_domain & 0x07) {
				snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x07, 0x00);
				usleep_range(16000, 17000);
			}

			snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x00);
			snd_soc_update_bits(codec, FLOW_CONTROL_2, 0x01, 0x01);

		} else {
			ex_pram_index = ex_cram_index = ex_aram_index = 0xff;

			reg_srcab = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR7);
			reg_srccd = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR8);
			snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x01, 0x01);
			snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x03);
			snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, reg_srcab);
			snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, reg_srccd);
			snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x00);
			snd_soc_update_bits(codec, FLOW_CONTROL_2, 0x01, 0x01);
		}

		if (pram_index != ex_pram_index &&
				ak4962->pram_firmware[pram_index] != NULL) {
			pram_fwbuf = kmemdup(ak4962->pram_firmware[pram_index]->data,
					ak4962->pram_firmware[pram_index]->size, GFP_KERNEL);
			if (!pram_fwbuf) {
				pr_err("%s: MEM Allocation for PRAM failed\n", __func__);
				return -ENOMEM;
			}

			if (interface == AK49XX_INTERFACE_TYPE_SLIMBUS) {
				pram_size = ak4962->pram_firmware[pram_index]->size;
			} else if (interface == AK49XX_INTERFACE_TYPE_SPI ||
					   interface == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
				pram_size = ak4962->pram_firmware[pram_index]->size - 2;
			}
		} else {
			pram_fwbuf = 0;
		}

		if (cram_index != ex_cram_index &&
				ak4962->cram_firmware[cram_index] != NULL) {
			cram_fwbuf = kmemdup(ak4962->cram_firmware[cram_index]->data,
					ak4962->cram_firmware[cram_index]->size, GFP_KERNEL);
			if (!cram_fwbuf) {
				pr_err("%s: MEM Allocation for CRAM failed\n", __func__);
				return -ENOMEM;
			}

			if (interface == AK49XX_INTERFACE_TYPE_SLIMBUS) {
				cram_size = ak4962->cram_firmware[cram_index]->size;
			} else if (interface == AK49XX_INTERFACE_TYPE_SPI ||
					   interface == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
				cram_size = ak4962->cram_firmware[cram_index]->size - 2;
			}
		} else {
			cram_fwbuf = 0;
		}

		if (aram_index != ex_aram_index &&
				ak4962->aram_firmware[aram_index] != NULL) {

			aram_fwbuf = kmemdup(ak4962->aram_firmware[aram_index]->data,
					ak4962->aram_firmware[aram_index]->size, GFP_KERNEL);
			if (!aram_fwbuf) {
				pr_err("%s: MEM Allocation for ARAM failed\n", __func__);
				return -ENOMEM;
			}

			if (interface == AK49XX_INTERFACE_TYPE_SLIMBUS) {
				aram_size = ak4962->aram_firmware[aram_index]->size;
			} else if (interface == AK49XX_INTERFACE_TYPE_SPI ||
					   interface == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
				aram_size = ak4962->aram_firmware[aram_index]->size - 2;
			}
		} else {
			aram_fwbuf = 0;
		}

		if (pram_fwbuf) {
			snd_soc_update_bits(codec, PRAM_READY, 0x01, 0x01);

			ret += ak49xx_ram_write(codec->control_data, 0x04, 0x00, 0x00,
					pram_size, pram_fwbuf);

			if (interface != AK49XX_INTERFACE_TYPE_SLIMBUS) {
				ret += ak49xx_bulk_read(core_res, CRC_RESULT_H8, 2, crc);
				if (((pram_fwbuf[pram_size] << 8) + pram_fwbuf[pram_size + 1])
						!= ((crc[0] << 8) + crc[1])) {
					ret = -EIO;
					pr_err("%s: PRAM download CRC failed\n", __func__);
				}
			}
			if (ret) {
				pr_err("%s: PRAM download failed\n", __func__);
			} else {
				pr_err("%s: PRAM download succeed\n", __func__);
			}

			snd_soc_update_bits(codec, PRAM_READY, 0x01, 0x00);
		}

		if (cram_fwbuf) {

			ret += ak49xx_ram_write(codec->control_data, 0x05, 0x00, 0x00,
					cram_size, cram_fwbuf);

			if (interface != AK49XX_INTERFACE_TYPE_SLIMBUS) {
				ret += ak49xx_bulk_read(core_res, CRC_RESULT_H8, 2, crc);
				if (((cram_fwbuf[cram_size] << 8) + cram_fwbuf[cram_size + 1])
						!= ((crc[0] << 8) + crc[1])) {
					ret = -EIO;
					pr_err("%s: CRAM download CRC failed\n", __func__);
				}
			}
			if (ret) {
				pr_err("%s: CRAM download failed\n", __func__);
			} else {
				pr_err("%s: CRAM download succeed\n", __func__);
			}
		}

		if (aram_fwbuf) {

			ret += ak49xx_ram_write(codec->control_data, 0x07, 0x00, 0x00,
					aram_size, aram_fwbuf);

			if (interface != AK49XX_INTERFACE_TYPE_SLIMBUS) {
				ret += ak49xx_bulk_read(core_res, CRC_RESULT_H8, 2, crc);
				if (((aram_fwbuf[aram_size] << 8) + aram_fwbuf[aram_size + 1])
						!= ((crc[0] << 8) + crc[1])) {
					ret = -EIO;
					pr_err("%s: ARAM download CRC failed\n", __func__);
				}
			}
			if (ret) {
				pr_err("%s: ARAM download failed\n", __func__);
			} else {
				pr_err("%s: ARAM download succeed\n", __func__);
			}
		}

		snd_soc_update_bits(codec, FLOW_CONTROL_2, 0x01, 0x00);
	}

	if (ret == 0) {
		ak4962_dsp_mode = new_dsp_mode;
		pr_info("%s: new mode = %d\n", __func__, new_dsp_mode);
		ak4962_band_set = 0;

		switch (ak4962_dsp_mode) {
		case DSP_MODE_OFF:
			break;
		case DSP_MODE_NARROW_HANDSET:
		case DSP_MODE_NARROW_HEADSET:
		case DSP_MODE_NARROW_HANDSFREE:
		case DSP_MODE_NARROW_HEADPHONE:
			snd_soc_write(codec, DSP_SETTING1, 0x68);
			snd_soc_write(codec, DSP_SETTING2, 0x3C);
			snd_soc_write(codec, DSP_SETTING3, 0x09);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_WIDE_HANDSET:
		case DSP_MODE_WIDE_HEADSET:
		case DSP_MODE_WIDE_HANDSFREE:
		case DSP_MODE_WIDE_HEADPHONE:
		case DSP_MODE_BEAM_RECORD:
			snd_soc_write(codec, DSP_SETTING1, 0x68);
			snd_soc_write(codec, DSP_SETTING2, 0x3C);
			snd_soc_write(codec, DSP_SETTING3, 0x0A);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_VOICE_RECOGNITION:
		case DSP_MODE_SOUND_RECORD:
		case DSP_MODE_VOICE_RECORD:
		case DSP_MODE_INTERVIEW_RECORD:
		case DSP_MODE_MUSIC_SPEAKER:
			snd_soc_write(codec, DSP_SETTING1, 0x22);
			snd_soc_write(codec, DSP_SETTING2, 0x00);
			snd_soc_write(codec, DSP_SETTING3, 0x0A);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_KARAOKE_CAVE:
		case DSP_MODE_KARAOKE_CLOISTER:
		case DSP_MODE_KARAOKE_LIVE:
		case DSP_MODE_KARAOKE_ROOM:
		case DSP_MODE_KARAOKE_THEATER:
		case DSP_MODE_KARAOKE_VALLEY:
		case DSP_MODE_KARAOKE_BYPASS:
		case DSP_MODE_BARGE_IN:
			snd_soc_write(codec, DSP_SETTING1, 0x21);
			snd_soc_write(codec, DSP_SETTING2, 0x00);
			snd_soc_write(codec, DSP_SETTING3, 0x09);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_KARAOKE_SPK_ROOM:
		case DSP_MODE_KARAOKE_SPK_THEATER:
		case DSP_MODE_KARAOKE_SPK_LIVE:
			snd_soc_write(codec, DSP_SETTING1, 0x62);
			snd_soc_write(codec, DSP_SETTING2, 0x73);
			snd_soc_write(codec, DSP_SETTING3, 0x09);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		case DSP_MODE_SMART_RINGTONE:
			snd_soc_write(codec, DSP_SETTING1, 0x20);
			snd_soc_write(codec, DSP_SETTING2, 0x00);
			snd_soc_write(codec, DSP_SETTING3, 0x08);
			snd_soc_write(codec, DSP_SETTING5, 0x01);
			break;
		default:
			break;
		}

		if (ak4962_dsp_mode == DSP_MODE_OFF) {
/* snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00); */
			ak4962->state = AK4962_IDLE;
		} else {
/* snd_soc_write(codec, FLOW_CONTROL_3, 0x01); */
			if (dsp_sync_domain & 0x07) {
				snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x07,
						dsp_sync_domain);
			}
			ak4962->state = AK4962_DSPRSTNON;
		}
	}

	if (ak4962_dsp_mode >= DSP_MODE_SOUND_RECORD &&
		ak4962_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {
		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x01;
		val[6] = 0x00;
		val[7] = 0x01;
		val[8] = 0;

		if (ak4962->ak4962_dsp_uplink_status) {
			val[5] = 0x12;
			ret = ak49xx_run_ram_write(codec->control_data, val);
		}

		if (ak4962->ak4962_dsp_downlink_status) {
			val[5] = 0x1c;
			ret = ak49xx_run_ram_write(codec->control_data, val);
		}

		if (ret) {
			pr_err("%s: ram_write_err %d\n", __func__, ret);
		}
	}

	kfree(pram_fwbuf);
	kfree(cram_fwbuf);
	kfree(aram_fwbuf);

	return ret;
}
 /* rev0.7 */

static int dout1_lch_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = ak4962->hp_dvol1_org;

	return 0;
}

static int dout1_lch_gain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	int dvol1;

	ak4962->hp_dvol1_org = ucontrol->value.integer.value[0];

	if (ak4962->hp_dvol1_org == 100) {	/* magic number: don't change volume */
		return 0;
	}

	dvol1 = ak4962->hp_dvol1_org + ak4962->hp_dvol1_offset;
	pr_info("%s:�@ hp_dvol1_org=%d, hp_dvol1_offset=%d, dvol1=%d\n", __func__,
		ak4962->hp_dvol1_org, ak4962->hp_dvol1_offset, dvol1);

	if (dvol1 > 0x1F)
		dvol1 = 0x1F;

	snd_soc_update_bits(codec, LCH_OUTPUT_VOLUME_1, 0x1F, dvol1);

	return 0;
}

static int hp_out_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = ak4962->hp_avol_org;

	pr_info("%s: ak4962->hp_avol_org=%d\n", __func__, ak4962->hp_avol_org);

	return 0;
}

static int hp_out_gain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	int avol;

	ak4962->hp_avol_org = ucontrol->value.integer.value[0];

	if (ak4962->hp_avol_org == 100) {	/* magic number: don't change volume */
		return 0;
	}

	if (ak4962->low_power_mode == 1) {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x10, 0x10);	/* LPMODE=1 */
		snd_soc_update_bits(codec, MODE_CONTROL, 0x40, 0x40);		/* DSMLP1=1 */
	} else {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x10, 0x00);	/* LPMODE=0 */
		snd_soc_update_bits(codec, MODE_CONTROL, 0x40, 0x00);		/* DSMLP1=0 */
	}

	/* rev0.16 */
	avol = ak4962->hp_avol_org + ak4962->hp_avol_offset + ak4962->power_select_vol_offset;
	pr_info("%s: hp_avol_org=%d, hp_avol_offset=%d, power_select_vol_offset=%d, avol=%d\n", __func__,
		ak4962->hp_avol_org, ak4962->hp_avol_offset, ak4962->power_select_vol_offset, avol);
	/* rev0.16 end */
	if (avol > 0x0F)
		avol = 0x0F;

	snd_soc_update_bits(codec, HP_VOLUME_CONTROL, 0x0F, avol);

	return 0;
}

static int ak49xx_bandswitch_set(struct snd_soc_codec *codec, int howtochange)
{
	u8 val[9];
	int mir3_2;

	if (ak4962_band_set == 1) {
		pr_err("[LHS]%s the band have set !\n", __func__);
		return 0;
	}

	mir3_2 = snd_soc_read(codec, MIR3_REGISTER_2);
	pr_info("[LHS] %s mir3_2 voice mode =%x howtochange=%d\n", __func__, mir3_2, howtochange);

	if (howtochange == 1) {
		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x0a;
		val[5] = 0x95;
		val[6] = 0x00;
		val[7] = 0x00;
		val[8] = 0;
		ak4962_band_set = 1;
		return ak49xx_run_ram_write(codec->control_data, val);
	} else if (howtochange == 2) {
		/* wide band to norrow band */
		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x0a;
		val[5] = 0x95;
		val[6] = 0x00;
		val[7] = 0x01;
		val[8] = 0;
		ak4962_band_set = 1;
		return ak49xx_run_ram_write(codec->control_data, val);
	}
	pr_err("[LHS] %s set akm codec band switch mode failed!\n", __func__);

	return 0;
}

static int akm_internal_rx_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = internal_rx_gain;
	return 0;
}

int akm_internal_rx_gain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	u8	val[9];

	internal_rx_gain = ucontrol->value.integer.value[0];
	pr_info("%s internal_rx_gain=%d\n", __func__, internal_rx_gain);
	codec = ak4962_codec;
	if (codec == NULL) {
		pr_info("%s no codec!\n", __func__);
		return -EINVAL;
	}
/*part boundary*/
	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x06;
	val[5] = 0x0c;	/* change address 9 -> c */
	val[6] = 0x00;
	val[7] = internal_rx_gain;
	val[8] = 0;

	return ak49xx_run_ram_write(codec->control_data, val);
/*if use below part, won`t use upper part*/
#if 0
	codec = ak4961_codec;
	if (codec == NULL) {
		pr_info("%s no codec!\n", __func__);
		return -EINVAL;
	}

	snd_soc_write(codec, RUN_STATE_DATA_LENGTH, 0x00);
	snd_soc_write(codec, RUN_STATE_START_ADDR1, 0x06);
	snd_soc_write(codec, RUN_STATE_START_ADDR2, 0x0C);
	snd_soc_write(codec, RUN_STATE_DATA_1, 0x00);
	snd_soc_write(codec, RUN_STATE_DATA_2, internal_rx_gain);
	snd_soc_write(codec, RUN_STATE_DATA_3, 0x00);
	snd_soc_write(codec, CRAM_RUN_EXE, 0x01);
	return 0;
#endif
}

/*add by shengguanghui for karaoke 20150606 begin*/
static DECLARE_TLV_DB_SCALE(karaoke_mic_gain_tlv, 000, 100, 0);
static int akm_karaoke_mic_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = karaoke_mic_gain;
	return 0;
}

static int akm_karaoke_mic_gain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);

	karaoke_mic_gain = ucontrol->value.integer.value[0];

	pr_info("%s karaoke_mic_gain=%d\n", __func__, karaoke_mic_gain);

	codec = ak4962_codec;
	if (codec == NULL) {
		pr_info("%s no codec!\n", __func__);
		return -EINVAL;
	}

	snd_soc_write(codec, RUN_STATE_DATA_LENGTH, 0x01);
	snd_soc_write(codec, RUN_STATE_START_ADDR1, 0x01);
	snd_soc_write(codec, RUN_STATE_START_ADDR2, 0x00);
	snd_soc_write(codec, RUN_STATE_DATA_1, mic_gain_table[karaoke_mic_gain]>>16);
	snd_soc_write(codec, RUN_STATE_DATA_2, (mic_gain_table[karaoke_mic_gain]>>8)%0x100);
	snd_soc_write(codec, RUN_STATE_DATA_3, mic_gain_table[karaoke_mic_gain]%0x100);
	snd_soc_write(codec, RUN_STATE_DATA_4, mic_gain_table[karaoke_mic_gain]>>16);
	snd_soc_write(codec, RUN_STATE_DATA_5, (mic_gain_table[karaoke_mic_gain]>>8)%0x100);
	snd_soc_write(codec, RUN_STATE_DATA_6, mic_gain_table[karaoke_mic_gain]%0x100);
	snd_soc_write(codec, CRAM_RUN_EXE, 0x01);

	return 0;

}
/*add by shengguanghui for karaoke 20150606 end*/

/*
 * Internal Rx Out Volume control:
 * from -18 to 0 dB in 3 dB steps
 */
static DECLARE_TLV_DB_SCALE(rx_out_gain_tlv, -1800, 300, 0);
static int akm_internal_mic_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = internal_mic_gain;
	return 0;
}

static int akm_internal_mic_gain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	u8	val[12];

	internal_mic_gain = ucontrol->value.integer.value[0];

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x01;
	val[4] = 0x01;
	val[5] = 0x04;
	val[6] = mic_gain_table[internal_mic_gain] >> 16;
	val[7] = (mic_gain_table[internal_mic_gain] >> 8) % 0x100;
	val[8] = mic_gain_table[internal_mic_gain] % 0x100;
	val[9] = val[6];
	val[10] = val[7];
	val[11] = val[8];

	return ak49xx_run_ram_write(codec->control_data, val);
}

/*
 * Internal MIC Out Volume control:
 * from 0 to +20 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_out_gain_tlv, 000, 100, 0);

static int akm_hpfeedback_mic_gain_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hpfeedback_mic_gain;
	return 0;
}

static int akm_hpfeedback_mic_gain_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	u8	val[9];

	hpfeedback_mic_gain = ucontrol->value.integer.value[0];

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x03;
	val[6] = hp_mic_gain_table[hpfeedback_mic_gain] >> 16;
	val[7] = (hp_mic_gain_table[hpfeedback_mic_gain] >> 8) % 0x100;
	val[8] = hp_mic_gain_table[hpfeedback_mic_gain] % 0x100;

	return ak49xx_run_ram_write(codec->control_data, val);
}
/*
 * HP Feedback MIC Volume control:
 * from -20 to 0 dB in 1 dB steps
 */

static DECLARE_TLV_DB_SCALE(hpfeedback_mic_gain_tlv, -2000, 100, 0);

static int akm_internal_ahpf_sensitivity_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = internal_ahpf_sensitivity;
	return 0;
}

static int akm_internal_ahpf_sensitivity_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	u8	val[12];

	internal_ahpf_sensitivity = ucontrol->value.integer.value[0];

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x01;
	val[4] = 0x01;
	val[5] = 0x44;
	val[6] = internal_ahpf_sensitivity;
	val[7] = 0x00;
	val[8] = 0x00;
	val[9] = internal_ahpf_sensitivity >> 1;
	if ((internal_ahpf_sensitivity % 2) != 0)
		val[10] = 0x80;
	else
		val[10] = 0x00;
	val[11] = 0x00;

	return ak49xx_run_ram_write(codec->control_data, val);
}
/*
 * Internal MIC Out Volume control:
 * from 0 to +13 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(ahpf_sensitivity_tlv, 000, 100, 0);

/* bargein path selector */
static const char * const bargein_sel_text[] = {
	"n/a", "output1_l", "output1_r", "output3_l", "output3_r",
	"input5_l", "input5_r", "input5_m"
};

static const char * const slim_inter_regs[] = {
	"r", "w"
};

static const struct soc_enum lch_bargein_sel_text_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bargein_sel_text), bargein_sel_text);

static const struct soc_enum rch_bargein_sel_text_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bargein_sel_text), bargein_sel_text);

static const struct soc_enum slim_inter_reg_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_inter_regs), slim_inter_regs);

static int ak4962_get_lch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->lch_bargein_sel;

	return 0;
}

static int ak4962_set_lch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret;

	pr_debug("%s: status = %d\n", __func__, status);

	snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x01);

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x23;
	val[6] = 0x00;
	val[7] = status;
	val[8] = 0;

	ret = ak49xx_run_ram_write(codec->control_data, val);
	if (ret == 0) {
		ak4962->lch_bargein_sel = status;
	}

	return ret;
}

static int ak4962_get_rch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->rch_bargein_sel;

	return 0;
}

static int ak4962_set_rch_bargein_sel(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret;

	pr_debug("%s: status = %d\n", __func__, status);

	snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x01);

	val[0] = 0x81;
	val[1] = RUN_STATE_DATA_LENGTH >> 8;
	val[2] = RUN_STATE_DATA_LENGTH & 0xff;
	val[3] = 0x00;
	val[4] = 0x01;
	val[5] = 0x24;
	val[6] = 0x00;
	val[7] = status;
	val[8] = 0;

	ret = ak49xx_run_ram_write(codec->control_data, val);
	if (ret == 0) {
		ak4962->rch_bargein_sel = status;
	}

	return ret;
}

static const char * const voice_sync_switch_text[] = {
	"off", "on"
};

static const struct soc_enum voice_sync_switch_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_sync_switch_text), voice_sync_switch_text);


static int ak4962_get_voice_sync_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->ak4962_voice_sync_switch;

	return 0;
}

static int ak4962_set_voice_sync_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret = 0;

	pr_info("%s: status = %d\n", __func__, status);

	if (ak4962_dsp_mode >= DSP_MODE_SOUND_RECORD &&
			ak4962_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {

		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x01;
		val[5] = 0x18;
		val[6] = 0x00;
		val[7] = status;
		val[8] = 0;

		ret = ak49xx_run_ram_write(codec->control_data, val);
	}
	if (ret == 0) {
		ak4962->ak4962_voice_sync_switch = status;
	}

	return ret;
}

static const char * const hp_anc_status_text[] = {
	"off", "airplane", "room", "trainbus",
};

static const struct soc_enum hp_anc_status_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_anc_status_text), hp_anc_status_text);

static int ak4962_get_hp_anc_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->ak4962_hp_anc_status;

	return 0;
}

static int ak4962_set_hp_anc_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u32 i;

	pr_info("%s: status = %d\n", __func__, status);

	switch (status) {
	case 0:
		snd_soc_update_bits(codec, 0x199, 0x01, 0x01);	/* ANCSMUTE=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x10);	/* ANCWR=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x00);	/* ANCWR=0 */
		msleep(230);	/* SMUTE wait */

		snd_soc_update_bits(codec, 0x196, 0x08, 0x00);	/* ANCE=0 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x10);	/* ANCWR=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x00);	/* ANCWR=0 */

		snd_soc_update_bits(codec, 0x199, 0x01, 0x01);	/* ANCSMUTE=0 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x10);	/* ANCWR=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x00);	/* ANCWR=0 */

		/* ak4962->mbhc_cfg.mclk_cb_fn(codec, 0, false);	//test4 */
		break;

	case 1:
		/* ak4962->mbhc_cfg.mclk_cb_fn(codec, 1, false);	//test4 */
		snd_soc_write(codec, ANC_FILTER_SETTING_1, 0x00);	/*test4 ANCE,etc=0*/
		snd_soc_update_bits(codec, 0x199, 0x0e, 0x0a);	/* test4 ANCSMT=5(0.228s) */
		snd_soc_update_bits(codec, 0x199, 0x01, 0x01);	/* test4 ANCSMUTE=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x10);	/* ANCWR=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x00);	/* ANCWR=0 */
		msleep(230);	/* SMUTE wait */
		for (i = 0; i < ARRAY_SIZE(anc_airplane_val); i++)
			snd_soc_write(codec, anc_airplane_val[i].reg,	anc_airplane_val[i].val);
		break;

	case 2:
		/* ak4962->mbhc_cfg.mclk_cb_fn(codec, 1, false);	//test4 */
		snd_soc_write(codec, ANC_FILTER_SETTING_1, 0x00);	/*test4 ANCE,etc=0*/
		snd_soc_update_bits(codec, 0x199, 0x0e, 0x0a);	/* test4 ANCSMT=5(0.228s) */
		snd_soc_update_bits(codec, 0x199, 0x01, 0x01);	/* test4 ANCSMUTE=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x10);	/* ANCWR=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x00);	/* ANCWR=0 */
		msleep(230);	/* SMUTE wait */
		for (i = 0; i < ARRAY_SIZE(anc_room_val); i++)
			snd_soc_write(codec, anc_room_val[i].reg,	anc_room_val[i].val);
		break;

	case 3:
		/* ak4962->mbhc_cfg.mclk_cb_fn(codec, 1, false);	//test4 */
		snd_soc_write(codec, ANC_FILTER_SETTING_1, 0x00);	/*test4 ANCE,etc=0*/
		snd_soc_update_bits(codec, 0x199, 0x0e, 0x0a);	/* test4 ANCSMT=5(0.228s) */
		snd_soc_update_bits(codec, 0x199, 0x01, 0x01);	/* test4 ANCSMUTE=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x10);	/* ANCWR=1 */
		snd_soc_update_bits(codec, 0x199, 0x10, 0x00);	/* ANCWR=0 */
		msleep(230);	/* SMUTE wait */
		for (i = 0; i < ARRAY_SIZE(anc_trainbus_val); i++)
			snd_soc_write(codec, anc_trainbus_val[i].reg,	anc_trainbus_val[i].val);
		break;
	}

	ak4962->ak4962_hp_anc_status = status;
	return 0;
}

/*modify by shengguanghui for impedance of hp at 201600607 begin*/
static const char * const hp_impedance_value_text[] = {
	"0", "1", "2"
};

static const struct soc_enum hp_impedance_value_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_impedance_value_text), hp_impedance_value_text);

static int ak4962_hp_impedance_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = ak4962_hp_impedance_value;
	pr_err("%s: ak4962_hp_impedance_value(%d)", __func__, ak4962_hp_impedance_value);
	return 0;
}

static int ak4962_hp_impedance_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ak4962_hp_impedance_value = ucontrol->value.integer.value[0];
	pr_err("%s: ak4962_hp_impedance_value(%d)", __func__, ak4962_hp_impedance_value);
	return 0;
}
/*modify by shengguanghui for impedance of hp at 201600607 end*/

/*modify by shengguanghui for voice gain set switch at 20160428 begin*/
static const char * const voice_gain_set_switch_text[] = {
	"0", "1", "2"
};

static const struct soc_enum voice_gain_set_switch_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_gain_set_switch_text), voice_gain_set_switch_text);

static int ak4962_gain_switch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = ak4962_gain_set_switch;
	pr_err("%s: ak4962_gain_set_switch(%d)", __func__, ak4962_gain_set_switch);
	return 0;
}

static int ak4962_gain_switch_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ak4962_gain_set_switch = ucontrol->value.integer.value[0];
	pr_err("%s: ak4962_gain_set_switch(%d)", __func__, ak4962_gain_set_switch);
	return 0;
}
/*modify by shengguanghui for voice gain set switch at 20160428 end*/

static const char * const voice_band_switch_text[] = {
	"off", "nb", "wb"
};

static const struct soc_enum voice_band_switch_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voice_band_switch_text), voice_band_switch_text);

static int ak4962_bandswitch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->ak4962_band_mode;
	return 0;
}

static int ak4962_bandswitch_set(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	mode =  ucontrol->value.integer.value[0];

	g_ak4962_band_mode = mode;
	ak4962_band_set = 0;

	if (codec == NULL) {
		pr_err("[LHS] %s no codec!\n", __func__);
		return -EINVAL;
	}

	if (mode < WB_NOT_SET || mode > WB_SET) {
		pr_err("%s: howtochange = %d out of scope(ret must 0~2)\n", __func__, mode);
		return -EINVAL;
	}

	if (snd_soc_read(codec, FLOW_CONTROL_3) & 0x01) {
		pr_err("[LHS] %s DSP working\n", __func__);
		ak49xx_bandswitch_set(codec, g_ak4962_band_mode);
	} else{
		pr_err("[LHS] %s DSP not working , set band later!\n", __func__);
	}

	ak4962->ak4962_band_mode = mode;
	return 0;
}

static const char * const uplink_ns_control_text[] = {
	"off", "on"
};

static const struct soc_enum uplink_ns_control_enum =
		SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(uplink_ns_control_text), uplink_ns_control_text);

static int ak4962_get_uplink_ns_control(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962->ak4962_uplink_ns_control;

	return 0;
}

static int ak4962_set_uplink_ns_control(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);
	u8	status = ucontrol->value.integer.value[0];
	u8	val[9];
	int	ret = 0;

	pr_info("%s: status = %d\n", __func__, status);

	if (ak4962_dsp_mode >= DSP_MODE_SOUND_RECORD &&
			ak4962_dsp_mode <= DSP_MODE_MUSIC_SPEAKER) {

		val[0] = 0x81;
		val[1] = RUN_STATE_DATA_LENGTH >> 8;
		val[2] = RUN_STATE_DATA_LENGTH & 0xff;
		val[3] = 0x00;
		val[4] = 0x01;
		val[5] = 0x16;
		val[6] = 0x00;
		val[7] = status;
		val[8] = 0;

		ret = ak49xx_run_ram_write(codec->control_data, val);
	}
		if (ret == 0) {
			ak4962->ak4962_uplink_ns_control = status;
	}

	return ret;
}

static const struct snd_kcontrol_new ak4962_snd_controls[] = {

	SOC_ENUM("MIC1 Power Level", mic_1_power_level),

	SOC_ENUM("MIC2 Power Level", mic_2_power_level),

	SOC_SINGLE_TLV("MIC Gain 1L", MIC_AMP_1_LCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC Gain 1R", MIC_AMP_1_RCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC Gain 2L", MIC_AMP_2_LCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC Gain 2R", MIC_AMP_2_RCH_GAIN, 0, 10, 0, mic_gain_tlv),

	SOC_SINGLE_TLV("MIC3 Gain", MIC_AMP_3_GAIN, 0, 3, 0, mic3_gain_tlv),

	SOC_SINGLE_TLV("AIN1 Attenuator", MIC_INPUT_ATT, 0, 1, 0, ain_att_tlv),

	SOC_SINGLE_TLV("AIN2 Attenuator", MIC_INPUT_ATT, 1, 1, 0, ain_att_tlv),

	SOC_SINGLE_TLV("AIN3 Attenuator", MIC_INPUT_ATT, 2, 1, 0, ain_att_tlv),

	SOC_SINGLE_TLV("AIN4 Attenuator", MIC_INPUT_ATT, 3, 1, 0, ain_att_tlv),

	SOC_SINGLE_TLV("AIN5 Attenuator", MIC_INPUT_ATT, 4, 1, 0, ain_att_tlv),

	SOC_SINGLE_TLV("AIN6 Attenuator", MIC_INPUT_ATT, 5, 1, 0, ain_att_tlv),

	/* SOC_SINGLE_TLV("DOut1 Lch Volume",
			LCH_OUTPUT_VOLUME_1, 0, 0x1F, 0, dout_vol_tlv),	//rev0.7 */
	SOC_SINGLE_EXT_TLV("DOut1 Lch Volume", SND_SOC_NOPM, 0, 0x1F, 0,
			dout1_lch_gain_get, dout1_lch_gain_set, dout_vol_tlv),	/* rev0.7, */

	SOC_SINGLE_TLV("DOut1 Rch Volume", RCH_OUTPUT_VOLUME_1, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE_TLV("DOut2 Lch Volume", LCH_OUTPUT_VOLUME_2, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE_TLV("DOut2 Rch Volume", RCH_OUTPUT_VOLUME_2, 0, 0x1F, 0, dout_vol_tlv),

	SOC_SINGLE("DOut1 Volume Independent", OUTPUT_VOLUME_SETTING, 0, 1, 0),

	SOC_SINGLE("DOut2 Volume Independent", OUTPUT_VOLUME_SETTING, 1, 1, 0),

	/* SOC_SINGLE_TLV("HP Out Volume",
			HP_VOLUME_CONTROL, 0, 0x0E, 0, hp_out_tlv),		//rev0.7 */
	SOC_SINGLE_EXT_TLV("HP Out Volume", SND_SOC_NOPM, 0, 0x0F, 0,
			hp_out_gain_get, hp_out_gain_set, hp_out_tlv),		/* rev0.7 */

	SOC_SINGLE_TLV("Lineout1 Volume", LINEOUT1_VOLUME_CONTROL, 0, 0x07, 0, lineout1_tlv),

	SOC_SINGLE_TLV("Lineout2 Volume", LINEOUT2_VOLUME_CONTROL, 4, 0x07, 0, lineout2_tlv),

	SOC_SINGLE_TLV("Receiver Volume", LINEOUT2_VOLUME_CONTROL, 0, 0x07, 0, receiver_tlv),

	SOC_ENUM("ADC1 HPF Enable", ad1_hpf_enable),

	SOC_ENUM("ADC2 HPF Enable", ad2_hpf_enable),

	SOC_ENUM("ADC1 HPF cut-off", ad1_hp_cf),

	SOC_ENUM("ADC2 HPF cut-off", ad2_hp_cf),

	SOC_ENUM("ADC1 Digital Filter", ad1_digital_filter),

	SOC_ENUM("ADC2 Digital Filter", ad2_digital_filter),

	SOC_ENUM("DAC1 Digital Filter", da1_digital_filter),

	SOC_ENUM("DAC1 Digital Filter Through", da1_filter_through),

	SOC_ENUM("DAC2 Digital Filter Through", da2_filter_through),

	SOC_ENUM("DAC1 Lch Mixer PGA", da1_lch_mix_pga),

	SOC_ENUM("DAC1 Rch Mixer PGA", da1_rch_mix_pga),

	SOC_ENUM("DAC2 Lch Mixer PGA", da2_lch_mix_pga),

	SOC_ENUM("DAC2 Rch Mixer PGA", da2_rch_mix_pga),

	SOC_ENUM("CP1 Clock Mode", cp1_clock_sel),

	SOC_SINGLE_TLV("MIXERA Input1 Rch Gain", MIXER_A_CONTROL, 4, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERA Input1 Lch Gain", MIXER_A_CONTROL, 5, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERA Input2 Rch Gain", MIXER_A_CONTROL, 6, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERA Input2 Lch Gain", MIXER_A_CONTROL, 7, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERB Input1 Rch Gain", MIXER_B_CONTROL, 4, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERB Input1 Lch Gain", MIXER_B_CONTROL, 5, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERB Input2 Rch Gain", MIXER_B_CONTROL, 6, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERB Input2 Lch Gain", MIXER_B_CONTROL, 7, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERC Input1 Rch Gain", MIXER_C_CONTROL, 4, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERC Input1 Lch Gain", MIXER_C_CONTROL, 5, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERC Input2 Rch Gain", MIXER_C_CONTROL, 6, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERC Input2 Lch Gain", MIXER_C_CONTROL, 7, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERD Input1 Rch Gain", MIXER_D_CONTROL, 4, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERD Input1 Lch Gain", MIXER_D_CONTROL, 5, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERD Input2 Rch Gain", MIXER_D_CONTROL, 6, 1, 0, mixer_gain_tlv),

	SOC_SINGLE_TLV("MIXERD Input2 Lch Gain", MIXER_D_CONTROL, 7, 1, 0, mixer_gain_tlv),

	SOC_ENUM("MIXERA Input1 SWAP", mixer_a1_swap),

	SOC_ENUM("MIXERA Input2 SWAP", mixer_a2_swap),

	SOC_ENUM("MIXERB Input1 SWAP", mixer_b1_swap),

	SOC_ENUM("MIXERB Input2 SWAP", mixer_b2_swap),

	SOC_ENUM("MIXERC Input1 SWAP", mixer_c1_swap),

	SOC_ENUM("MIXERC Input2 SWAP", mixer_c2_swap),

	SOC_ENUM("MIXERD Input1 SWAP", mixer_d1_swap),

	SOC_ENUM("MIXERD Input2 SWAP", mixer_d2_swap),

	SOC_ENUM("SRCAO Sync Domain", srcao_sync_domain),

	SOC_ENUM("SRCBO Sync Domain", srcbo_sync_domain),

	SOC_ENUM("SRCCO Sync Domain", srcco_sync_domain),

	SOC_ENUM("SRCDO Sync Domain", srcdo_sync_domain),

	SOC_ENUM("DSP Sync Domain", dsp_sync_domain),

	SOC_ENUM("DSPO2 Sync Domain", dspo2_sync_domain),

	SOC_ENUM("DSPO4 Sync Domain", dspo4_sync_domain),

	SOC_ENUM("CODEC Sync Domain", cdc_sync_domain),

	SOC_ENUM("PORT2 Sync Domain", port2_sync_domain),

	SOC_ENUM("PORT3 Sync Domain", port3_sync_domain),

	SOC_ENUM("PORT4 Sync Domain", port4_sync_domain),

	SOC_ENUM_EXT("HP Power Mode", hp_power_mode_enum,
			ak4962_get_hp_power_mode, ak4962_set_hp_power_mode),	/*rev0.16*/

	SOC_ENUM_EXT("DSP Downlink Status", dsp_downlink_status_enum,
			ak4962_get_dsp_downlink_status, ak4962_set_dsp_downlink_status),

	SOC_ENUM_EXT("DSP Uplink Status", dsp_uplink_status_enum,
			ak4962_get_dsp_uplink_status, ak4962_set_dsp_uplink_status),

	SOC_ENUM_EXT("DSP Mode", dsp_mode_enum[0],
			ak4962_get_dsp_mode, ak4962_set_dsp_mode),

	SOC_SINGLE("DSP CLK Adjust", DSP_SETTING5, 0, 255, 0),

	SOC_SINGLE_EXT_TLV("Internal RX Out Volume", SND_SOC_NOPM, 0, 6, 0,
			akm_internal_rx_gain_get, akm_internal_rx_gain_set, rx_out_gain_tlv),

	SOC_SINGLE_EXT_TLV("Internal MIC Out Volume", SND_SOC_NOPM, 0, 20, 0,
			akm_internal_mic_gain_get, akm_internal_mic_gain_set, mic_out_gain_tlv),

	SOC_SINGLE_EXT_TLV("HP Feedback MIC Volume", SND_SOC_NOPM, 0, 20, 0,
			akm_hpfeedback_mic_gain_get, akm_hpfeedback_mic_gain_set, hpfeedback_mic_gain_tlv),

	/*add by shengguanghui for karaoke 20150606 begin*/
	SOC_SINGLE_EXT_TLV("KARAOKE MIC Volume", SND_SOC_NOPM, 0, 20, 0,
			akm_karaoke_mic_gain_get, akm_karaoke_mic_gain_set, karaoke_mic_gain_tlv),
	/*add by shengguanghui for karaoke 20150606 end*/

	SOC_SINGLE_EXT_TLV("Internal AHPF Sensitivity", SND_SOC_NOPM, 0, 13, 0,
			akm_internal_ahpf_sensitivity_get, akm_internal_ahpf_sensitivity_set,
			ahpf_sensitivity_tlv),

	SOC_ENUM_EXT("Bargein Lch Sel", lch_bargein_sel_text_enum,
			ak4962_get_lch_bargein_sel, ak4962_set_lch_bargein_sel),

	SOC_ENUM_EXT("Bargein Rch Sel", rch_bargein_sel_text_enum,
			ak4962_get_rch_bargein_sel, ak4962_set_rch_bargein_sel),

	SOC_ENUM_EXT("Voice Sync Switch", voice_sync_switch_enum,
			ak4962_get_voice_sync_switch, ak4962_set_voice_sync_switch),

	SOC_ENUM_EXT("HP ANC Status", hp_anc_status_enum,
			ak4962_get_hp_anc_status, ak4962_set_hp_anc_status),

	SOC_ENUM_EXT("Voice Band Switch", voice_band_switch_enum,
			ak4962_bandswitch_get, ak4962_bandswitch_set),

	SOC_ENUM_EXT("Uplink NS Control", uplink_ns_control_enum,
			ak4962_get_uplink_ns_control, ak4962_set_uplink_ns_control),

	SOC_ENUM_EXT("Slim Interface Regs", slim_inter_reg_enum,
			ak4962_get_slim_inter_regs, ak4962_set_slim_inter_regs),
	/*modify by shengguanghui for Gain set Switch at 20160316 begin*/
	SOC_ENUM_EXT("Voice Gain Set Switch", voice_gain_set_switch_enum,
			ak4962_gain_switch_get, ak4962_gain_switch_set),
	/*modify by shengguanghui for Gain set Switch at 20160316 end*/

	/*modify by shengguanghui for impedance of hp at 201600607 begin*/
	SOC_ENUM_EXT("Hp impedance value", hp_impedance_value_enum,
			ak4962_hp_impedance_get, ak4962_hp_impedance_set),
	/*modify by shengguanghui for impedance of hp at 201600607 end*/
};

/* virtual port entries */
static int slim_tx_mixer_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(codec);
	struct soc_multi_mixer_control *mixer =
		((struct soc_multi_mixer_control *)kcontrol->private_value);
	u32 port_id = mixer->shift;

	ucontrol->value.integer.value[0] = (ak4962_p->tx_port_value & 1 << port_id);
	return 0;
}

static int slim_tx_mixer_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *core = dev_get_drvdata(codec->dev->parent);
	struct soc_multi_mixer_control *mixer =
		((struct soc_multi_mixer_control *)kcontrol->private_value);
	struct snd_soc_dapm_update *update = NULL;
	u32 dai_id = widget->shift;
	u32 port_id = mixer->shift;
	u32 enable = ucontrol->value.integer.value[0];
	u32 vtable = vport_check_table[dai_id];

	pr_debug("%s: wname %s cname %s value %u shift %d item %ld\n", __func__,
		widget->name, ucontrol->id.name, ak4962_p->tx_port_value, widget->shift,
		ucontrol->value.integer.value[0]);

	mutex_lock(&codec->mutex);
	if (ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS &&
		ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		if (dai_id != SB1_CAP) {
			dev_err(codec->dev, "%s: invalid AIF for I2S mode\n",
				__func__);
			mutex_unlock(&codec->mutex);
			return -EINVAL;
		}
	}
	switch (dai_id) {
	case SB1_CAP:
	case SB2_CAP:
	case SB3_CAP:
		/* only add to the list if value not set
		 */
		if (enable && !(ak4962_p->tx_port_value & 1 << port_id)) {

			if (ak4962_p->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
				ak4962_p->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
				vtable = vport_check_table[dai_id];
			if (ak4962_p->intf_type == AK49XX_INTERFACE_TYPE_I2C ||
					ak4962_p->intf_type == AK49XX_INTERFACE_TYPE_SPI)
				vtable = vport_i2s_check_table[dai_id];

			if (ak49xx_tx_vport_validation(
						vtable,
						port_id,
						ak4962_p->dai, NUM_CODEC_DAIS)) {
				dev_dbg(codec->dev, "%s: TX%u is used by other virtual port\n",
					__func__, port_id + 1);
				mutex_unlock(&codec->mutex);
				return 0;
			}
			ak4962_p->tx_port_value |= 1 << port_id;
			list_add_tail(&core->tx_chs[port_id].list,
				      &ak4962_p->dai[dai_id].ak49xx_ch_list
				      );
		} else if (!enable && (ak4962_p->tx_port_value & 1 << port_id)) {
			ak4962_p->tx_port_value &= ~(1 << port_id);
			list_del_init(&core->tx_chs[port_id].list);
		} else {
			if (enable)
				dev_dbg(codec->dev, "%s: TX%u port is used by this virtual port\n",
					__func__, port_id + 1);
			else
				dev_dbg(codec->dev, "%s: TX%u port is not used by this virtual port\n",
					__func__, port_id + 1);
			/* avoid update power function */
			mutex_unlock(&codec->mutex);
			return 0;
		}
		break;
	default:
		pr_err("Unknown AIF %d\n", dai_id);
		mutex_unlock(&codec->mutex);
		return -EINVAL;
	}
	pr_debug("%s: name %s sname %s updated value %u shift %d\n", __func__,
		widget->name, widget->sname, ak4962_p->tx_port_value, widget->shift);

	mutex_unlock(&codec->mutex);
	snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, enable, update);

	return 0;
}

static int slim_rx_mux_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = ak4962_p->rx_port_value;
	return 0;
}

static const char *const slim_rx_mux_text[] = {
	"ZERO", "SB1_PB", "SB2_PB", "SB3_PB"
};

static int slim_rx_mux_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = dapm_kcontrol_get_wlist(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_codec *codec = widget->codec;
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(codec);
	struct ak49xx *core = dev_get_drvdata(codec->dev->parent);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;
	u32 port_id = widget->shift;

	pr_debug("%s: wname %s cname %s value %u shift %d item %u\n", __func__,
		widget->name, ucontrol->id.name, ak4962_p->rx_port_value, widget->shift,
		ucontrol->value.enumerated.item[0]);

	ak4962_p->rx_port_value = ucontrol->value.enumerated.item[0];

	mutex_lock(&codec->mutex);

	if (ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS &&
		ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		if (ak4962_p->rx_port_value > 2) {
			dev_err(codec->dev, "%s: invalid AIF for I2S mode\n",
				__func__);
			goto err;
		}
	}
	/* value need to match the Virtual port and AIF number
	 */
	switch (ak4962_p->rx_port_value) {
	case 0:
		list_del_init(&core->rx_chs[port_id].list);
	break;
	case 1:
		if (ak49xx_rx_vport_validation(port_id +
			AK4962_RX_PORT_START_NUMBER,
			&ak4962_p->dai[SB1_PB].ak49xx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id + 1);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &ak4962_p->dai[SB1_PB].ak49xx_ch_list);
	break;
	case 2:
		if (ak49xx_rx_vport_validation(port_id +
			AK4962_RX_PORT_START_NUMBER,
			&ak4962_p->dai[SB2_PB].ak49xx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id + 1);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &ak4962_p->dai[SB2_PB].ak49xx_ch_list);
	break;
	case 3:
		if (ak49xx_rx_vport_validation(port_id +
			AK4962_RX_PORT_START_NUMBER,
			&ak4962_p->dai[SB3_PB].ak49xx_ch_list)) {
			dev_dbg(codec->dev, "%s: RX%u is used by current requesting AIF_PB itself\n",
				__func__, port_id + 1);
			goto rtn;
		}
		list_add_tail(&core->rx_chs[port_id].list,
			      &ak4962_p->dai[SB3_PB].ak49xx_ch_list);
	break;
	default:
		pr_err("Unknown AIF %d\n", ak4962_p->rx_port_value);
		goto err;
	}
rtn:
	mutex_unlock(&codec->mutex);
	snd_soc_dapm_mux_update_power(widget->dapm, kcontrol, ak4962_p->rx_port_value, e, update);
	return 0;
err:
	mutex_unlock(&codec->mutex);
	return -EINVAL;
}

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);

static const struct snd_kcontrol_new slim_rx_mux[AK4962_RX_MAX] = {
	SOC_DAPM_ENUM_EXT("SLIM RX1 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX2 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX3 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX4 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX5 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX6 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
};

static const struct snd_kcontrol_new slim_cap1_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, AK4962_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, AK4962_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, AK4962_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, AK4962_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, AK4962_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, AK4962_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, AK4962_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, AK4962_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, AK4962_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, AK4962_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static const struct snd_kcontrol_new slim_cap2_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, AK4962_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, AK4962_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, AK4962_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, AK4962_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, AK4962_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, AK4962_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, AK4962_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, AK4962_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, AK4962_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, AK4962_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static const struct snd_kcontrol_new slim_cap3_mixer[] = {
	SOC_SINGLE_EXT("SLIM TX1", SND_SOC_NOPM, AK4962_TX1, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX2", SND_SOC_NOPM, AK4962_TX2, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX3", SND_SOC_NOPM, AK4962_TX3, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX4", SND_SOC_NOPM, AK4962_TX4, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX5", SND_SOC_NOPM, AK4962_TX5, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX6", SND_SOC_NOPM, AK4962_TX6, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX7", SND_SOC_NOPM, AK4962_TX7, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX8", SND_SOC_NOPM, AK4962_TX8, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX9", SND_SOC_NOPM, AK4962_TX9, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
	SOC_SINGLE_EXT("SLIM TX10", SND_SOC_NOPM, AK4962_TX10, 1, 0,
			slim_tx_mixer_get, slim_tx_mixer_put),
};

static int ak4962_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct ak49xx *core;
	struct snd_soc_codec *codec = w->codec;
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	struct ak49xx_codec_dai_data *dai;

	core = dev_get_drvdata(codec->dev->parent);

	pr_debug("%s: event called! codec name %s num_dai %d\n"
		"stream name %s event %d\n",
		__func__, w->codec->component.name, w->codec->component.num_dai, w->sname, event);

	/* Execute the callback only if interface type is slimbus */
	if (ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS &&
		ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		return 0;

	dai = &ak4962_p->dai[w->shift];
	pr_debug("%s: w->name %s w->shift %d event %d\n",
		 __func__, w->name, w->shift, event);
	pr_err("[lhs]%s: w->name %s w->shift %d event %d\n",
		 __func__, w->name, w->shift, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ak4962_bring_up(core);
		if (slim_ssr) {
			pr_err("YCH post SSR err\n");
			ret = ak49xx_close_slim_sch_rx(core, &dai->ak49xx_ch_list, dai->grph);
			slim_ssr = 0;
		}

		#ifdef CONFIG_SND_SOC_ESXXX
		if ((w->shift == SB1_PB) || (w->shift == SB2_PB))
			ret = remote_cfg_slim_rx(w->shift);
		#endif
		ret = ak49xx_cfg_slim_sch_rx(core, &dai->ak49xx_ch_list,
					      dai->rate, dai->bit_width,
					      &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		#ifdef CONFIG_SND_SOC_ESXXX
		if ((w->shift == SB1_PB) || (w->shift == SB2_PB))
			ret = remote_close_slim_rx(w->shift);
		#endif
		ret = ak49xx_close_slim_sch_rx(core, &dai->ak49xx_ch_list,
						dai->grph);
		if (ret < 0)
			slim_ssr = 1;

		if (ret < 0) {
			ret = ak49xx_disconnect_port(core,
						      &dai->ak49xx_ch_list,
						      dai->grph);
			pr_debug("%s: Disconnect RX port, ret = %d\n",
				 __func__, ret);
		}
		break;
	}
	return ret;
}

static int ak4962_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct ak49xx *core;
	struct snd_soc_codec *codec = w->codec;
	struct ak4962_priv *ak4962_p = snd_soc_codec_get_drvdata(codec);
	u32  ret = 0;
	struct ak49xx_codec_dai_data *dai;

	core = dev_get_drvdata(codec->dev->parent);

	pr_debug("%s: event called! codec name %s num_dai %d stream name %s\n",
		__func__, w->codec->component.name, w->codec->component.num_dai, w->sname);

	/* Execute the callback only if interface type is slimbus */
	if (ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS &&
		ak4962_p->intf_type != AK49XX_INTERFACE_TYPE_SLIMBUS_SPI)
		return 0;

	pr_debug("%s(): w->name %s event %d w->shift %d\n",
		__func__, w->name, event, w->shift);

	dai = &ak4962_p->dai[w->shift];
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ak4962_bring_up(core);
		#ifdef CONFIG_SND_SOC_ESXXX
		if (w->shift == SB1_CAP)
			ret = remote_cfg_slim_tx(w->shift);
		#endif
		ret = ak49xx_cfg_slim_sch_tx(core, &dai->ak49xx_ch_list,
						dai->rate, dai->bit_width,
					    &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		#ifdef CONFIG_SND_SOC_ESXXX
		if (w->shift == SB1_CAP)
			ret = remote_close_slim_tx(w->shift);
		#endif
		ret = ak49xx_close_slim_sch_tx(core, &dai->ak49xx_ch_list,
						dai->grph);
		if (ret < 0) {
			ret = ak49xx_disconnect_port(core,
						&dai->ak49xx_ch_list,
						dai->grph);
			pr_debug("%s: Disconnect RX port ret = %d\n",
				__func__, ret);
		}
		break;
	}
	return ret;
}

/* DAC1 Lch Digital Mixing Setting */
static const struct snd_kcontrol_new dac1_lch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC1_MONO_MIXING, 0, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC1_MONO_MIXING, 1, 1, 0),

	SOC_DAPM_SINGLE("Anc_Switch", POWER_MANAGEMENT_8, 4, 1, 0),
};

/* DAC1 Rch Digital Mixing Setting */
static const struct snd_kcontrol_new dac1_rch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC1_MONO_MIXING, 4, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC1_MONO_MIXING, 5, 1, 0),

	SOC_DAPM_SINGLE("Anc_Switch", POWER_MANAGEMENT_8, 5, 1, 0),
};

/* DAC2 Lch Digital Mixing Setting */
static const struct snd_kcontrol_new dac2_lch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC2_MONO_MIXING, 0, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC2_MONO_MIXING, 1, 1, 0),
};

/* DAC2 Rch Digital Mixing Setting */
static const struct snd_kcontrol_new dac2_rch_mixer_kctrl[] = {

	SOC_DAPM_SINGLE("Lch_Switch", DAC2_MONO_MIXING, 4, 1, 0),

	SOC_DAPM_SINGLE("Rch_Switch", DAC2_MONO_MIXING, 5, 1, 0),
};

/* Mic-Amp Input Signal Select */
static const char * const mic_input_select_text[] = {
	"AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "ZERO"
};

static const struct soc_enum mic1L_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_1, 0, 7, mic_input_select_text);

static const struct soc_enum mic1R_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_1, 4, 7, mic_input_select_text);

static const struct soc_enum mic2L_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_2, 0, 7, mic_input_select_text);

static const struct soc_enum mic2R_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_2, 4, 7, mic_input_select_text);

static const struct soc_enum mic3_input_select_enum =
	SOC_ENUM_SINGLE(MIC_INPUT_SELECTOR_3, 0, 7, mic_input_select_text);

static const struct snd_kcontrol_new mic1L_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-1L input Selector Mux", mic1L_input_select_enum);

static const struct snd_kcontrol_new mic1R_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-1R input Selector Mux", mic1R_input_select_enum);

static const struct snd_kcontrol_new mic2L_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-2L input Selector Mux", mic2L_input_select_enum);

static const struct snd_kcontrol_new mic2R_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-2R input Selector Mux", mic2R_input_select_enum);

static const struct snd_kcontrol_new mic3_input_select_kctrl =
	SOC_DAPM_ENUM("Mic-3 input Selector Mux", mic3_input_select_enum);

/* ANC Mic-Amp Input Signal Select */
static const char * const anc_mic_input_select_text[] = {
	"AIN", "ANC"
};

static const struct soc_enum ain3_input_select_enum =
	SOC_ENUM_SINGLE(POWER_MANAGEMENT_8, 6, 2, anc_mic_input_select_text);

static const struct soc_enum ain4_input_select_enum =
	SOC_ENUM_SINGLE(POWER_MANAGEMENT_8, 7, 2, anc_mic_input_select_text);

static const struct snd_kcontrol_new ain3_input_select_kctrl =
	SOC_DAPM_ENUM("AIN3 input Selector Mux", ain3_input_select_enum);

static const struct snd_kcontrol_new ain4_input_select_kctrl =
	SOC_DAPM_ENUM("AIN4 input Selector Mux", ain4_input_select_enum);

/* rev0.15 */
static u8 charge_pump2_status;
static u8 ldo3_status;

static int ak4962_change_charge_pump_2(struct snd_soc_codec *codec,
		int event)
{
	pr_info("\t********[AK4962] %s(%d) event=%d\n", __func__, __LINE__, event);
	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case enable_cp2:
		if (charge_pump2_status == 1 || charge_pump2_status == 2) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x02, 0x02);	/* PMCP2=1 */
		}
		break;
	case disable_cp2:
		if (charge_pump2_status == 0) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x02, 0x00);	/* PMCP2=0 */
		}
		break;
	}
	return 0;
}

static int ak4962_change_ldo_3(struct snd_soc_codec *codec,
		int event)
{
	pr_info("\t********[AK4962] %s(%d) event=%d\n", __func__, __LINE__, event);
	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case enable_ldo3:
		if (ldo3_status == 1 || ldo3_status == 2) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0xC0, 0xC0);	/* PMCLDO3P/N on */
		}
		break;
	case disable_ldo3:
		if (ldo3_status == 0) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0xC0, 0x00);	/* PMLDO3P/N=0 */
		}
		break;
	}
	return 0;
}
/* rev0.15 end */

static int ak4962_codec_enable_dac(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u8 dac_setting;
	unsigned int dac;
	int ret;

	ret = kstrtouint(strpbrk(w->name, "12"), 10, &dac);
	if (ret < 0) {
		pr_err("%s: Invalid AMIC line on the codec\n", __func__);
		return -EINVAL;
	}

	dac_setting = (1 << ((dac - 1) << 1));
	pr_err("[LHS][AK4962] %s 1:dac_status=%d\n", __func__, dac_status);
	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
	if (dac_setting == 4) {
	    snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x10, 0x10);	/* LPMODE=1 */
	}

		if (share_wait_time) {
			share_wait_time = 0;
		} else {
			usleep_range(6500, 6600);	/* wait for charge pump*/
		}
		ldo3_status |= 0x01;						/* rev0.15 */
		ak4962_change_ldo_3(codec, enable_ldo3);	/* rev0.15 */
		usleep_range(1000, 1100);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_8, dac_setting, dac_setting);
		dac_status += dac;
		break;
	case SND_SOC_DAPM_POST_PMD:
		dac_status -= dac;
		if (dac_status == 0) {
			ldo3_status &= 0xFE;						/* rev0.15 */
			ak4962_change_ldo_3(codec, disable_ldo3);	/* rev0.15 */
		}
		snd_soc_update_bits(codec, POWER_MANAGEMENT_8, dac_setting, 0x00);
		share_wait_time = 0;
		break;
	}
	pr_err("[LHS][AK4962] %s 2:dac_status=%d\n", __func__, dac_status);
	return 0;
}

static int ak4962_codec_enable_hp(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	pr_err("[LHS][AK4962] %s 1:hp_status=%d, event = %d\n", __func__, hp_status, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	  if (hp_status == 0) {
		charge_pump2_status |= 0x01;					/* rev0.15 */
		ak4962_change_charge_pump_2(codec, enable_cp2);	/* rev0.15 */
		usleep_range(4500, 4600);

	/* rev0.15 */
		while ((snd_soc_read(codec, IMPEDANCE_DETECTION) & 0x80) > 0) {
		       pr_info("\t[AK4962] %s(%d)*****IMP DET wait loop******\n", __func__, __LINE__);
			usleep_range(10000, 11000);
		}
	/* rev0.15 end */

		if (ak4962->mono_jack == 0)	{
			snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03);
		} else if (ak4962->mono_jack == 1)	{
			snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x01, 0x01);
			snd_soc_update_bits(codec, DAC1_MONO_MIXING, 0x06, 0x06);
		}
/* snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x03); */
		usleep_range(21000, 22000);
	  }
		hp_status++;
		break;
	case SND_SOC_DAPM_PRE_PMD:
	if (hp_status == 1) {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x03, 0x00);
		charge_pump2_status &= 0xFE;						/* rev0.15 */
		ak4962_change_charge_pump_2(codec, disable_cp2);	/* rev0.15 */
	}

	if (ak4962->mono_jack == 1)	{
		snd_soc_update_bits(codec, DAC1_MONO_MIXING, 0x06, 0x00);
	}

	hp_status--;
	break;
	}

	pr_err("[LHS][AK4962] %s 2:hp_status=%d\n", __func__, hp_status);

	return 0;
}

static int ak4962_codec_enable_lout1l(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x10, 0x10);
		usleep_range(4000, 4100);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x10, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_enable_lout1r(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x20, 0x20);
		usleep_range(4000, 4100);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_9, 0x20, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_enable_lout2l(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x01);
		usleep_range(4000, 4100);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x00);
		break;
	}
	return 0;
}
/*
static int ak4962_codec_enable_lout2ld(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x03, 0x03);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x03, 0x00);
		break;
	}
	return 0;
}
*/
static int ak4962_codec_enable_lout2ldp(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x01);
/* usleep_range(4000, 4000); */
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x01, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_enable_lout2ldn(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x02, 0x02);
/* usleep_range(4000, 4000); */
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x02, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_enable_lout2r(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x10);
		usleep_range(4000, 4100);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x00);
		break;
	}
	return 0;
}
/*
static int ak4962_codec_enable_lout2rd(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x30, 0x30);
		usleep_range(4000, 4000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x30, 0x00);
		break;
	}
	return 0;
}
*/
static int ak4962_codec_enable_lout2rdp(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x10);
/* usleep_range(4000, 4000); */
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x10, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_enable_lout2rdn(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, LINEOUT2_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x20, 0x20);
/* usleep_range(4000, 4000); */
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x20, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_enable_rcv(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		charge_pump2_status |= 0x01;						/* rev0.15 */
		ak4962_change_charge_pump_2(codec, enable_cp2);		/* rev0.15 */
		usleep_range(4500, 4600);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x80, 0x80);
		usleep_range(15000, 16000);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_10, 0x80, 0x00);
		charge_pump2_status &= 0xFE;						/* rev0.15 */
		ak4962_change_charge_pump_2(codec, disable_cp2);	/* rev0.15 */
		break;
	}
	return 0;
}

static int ak4962_codec_enable_micbias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
/* struct snd_soc_codec *codec = w->codec; */

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(14000, 15000);	/* 48kHz wait time*/
		break;
	}
	return 0;
}

static int ak4962_codec_enable_pmsw(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int	reg_srcab, reg_srccd;

	if (ak4962_dsp_mode != DSP_MODE_OFF) {
		return 0;
	}

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		reg_srcab = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR7);
		reg_srccd = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR8);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x01, 0x01);
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x02, 0x02);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, reg_srcab);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, reg_srccd);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);
		break;
	}
	return 0;
}

static u8 charge_pump1_status;

static int ak4962_change_charge_pump_1(struct snd_soc_codec *codec,
		int event)
{
	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case enable_cp1:
		if (charge_pump1_status == 1 || charge_pump1_status == 2) {
/* charge_pump1_status == 3) { */
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x01, 0x01);
			usleep_range(6500, 6600);
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x10, 0x10);
			usleep_range(500, 600);
		}
		if (snd_soc_read(codec, POWER_MANAGEMENT_8) & 0x10) {
			snd_soc_update_bits(codec, 0x199, 0x10, 0x10);
			snd_soc_update_bits(codec, 0x199, 0x10, 0x00);
		}
		break;
	case disable_cp1:
		if (charge_pump1_status == 0) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x11, 0x00);
		}
		break;
	}
	return 0;
}

static int ak4962_codec_enable_charge_pump_1(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x80, 0x80);

		charge_pump1_status |= 0x01;
		ak4962_change_charge_pump_1(codec, enable_cp1);

		if (snd_soc_read(codec, POWER_MANAGEMENT_3) & 0x04) {
			share_wait_time = 1;
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x80, 0x00);

		charge_pump1_status &= 0xFE;
		ak4962_change_charge_pump_1(codec, disable_cp1);
		break;
	}
	return 0;
}

/* rev0.15 */
static u8 charge_pump3_status;

static int ak4962_change_charge_pump_3(struct snd_soc_codec *codec,
		int event)
{
	pr_info("\t********[AK4962] %s(%d) event=%d\n", __func__, __LINE__, event);
	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case enable_cp3:
		if (charge_pump3_status == 1 || charge_pump3_status == 2) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x04, 0x04);	/* PMCP3=1 */
		}
		break;
	case disable_cp3:
		if (charge_pump3_status == 0) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_3, 0x04, 0x00);	/* PMCP3=0 */
		}
		break;
	}
	return 0;
}

static int ak4962_codec_enable_charge_pump_3(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_info("\t********[AK4962] %s(%d) event=%d\n", __func__, __LINE__, event);
	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		charge_pump3_status |= 0x01;
		ak4962_change_charge_pump_3(codec, enable_cp3);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		charge_pump3_status &= 0xFE;
		ak4962_change_charge_pump_3(codec, disable_cp3);
		break;
	}

	return 0;
}
/* rev0.15 end */

static const char * const mic_select_text[] = {"Analog", "Digital"};

static const struct soc_enum mic1l_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_1, 0, 2, mic_select_text);

static const struct soc_enum mic2l_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_2, 0, 2, mic_select_text);

static const struct soc_enum mic1r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mic_select_text);

static const struct soc_enum mic2r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mic_select_text);

static const struct snd_kcontrol_new mic1l_select_kctrl =
	SOC_DAPM_ENUM("MIC1L Selector Mux", mic1l_select_enum);

static const struct snd_kcontrol_new mic2l_select_kctrl =
	SOC_DAPM_ENUM("MIC2L Selector Mux", mic2l_select_enum);

static const struct snd_kcontrol_new mic1r_select_kctrl =
	SOC_DAPM_ENUM("MIC1R Selector Mux", mic1r_select_enum);

static const struct snd_kcontrol_new mic2r_select_kctrl =
	SOC_DAPM_ENUM("MIC2R Selector Mux", mic2r_select_enum);

static const char * const dmic_select_text[] = {"dmdat1l", "dmdat1r", "dmdat2l", "dmdat2r"};

static const struct soc_enum dmic1l_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_3, 0, 4, dmic_select_text);

static const struct soc_enum dmic1r_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_3, 2, 4, dmic_select_text);

static const struct soc_enum dmic2l_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_3, 4, 4, dmic_select_text);

static const struct soc_enum dmic2r_select_enum =
	SOC_ENUM_SINGLE(DIGITAL_MIC_3, 6, 4, dmic_select_text);

static const struct snd_kcontrol_new dmic1l_select_kctrl =
	SOC_DAPM_ENUM("DMIC1L Selector Mux", dmic1l_select_enum);

static const struct snd_kcontrol_new dmic1r_select_kctrl =
	SOC_DAPM_ENUM("DMIC1R Selector Mux", dmic1r_select_enum);

static const struct snd_kcontrol_new dmic2l_select_kctrl =
	SOC_DAPM_ENUM("DMIC2L Selector Mux", dmic2l_select_enum);

static const struct snd_kcontrol_new dmic2r_select_kctrl =
	SOC_DAPM_ENUM("DMIC2R Selector Mux", dmic2r_select_enum);

static const char * const pmmp1_cp1_switch_text[] = {"Off", "On"};

static const struct soc_enum pmmp1_cp1_switch_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, pmmp1_cp1_switch_text);

static const struct snd_kcontrol_new pmmp1_cp1_switch_kctrl =
	SOC_DAPM_ENUM("PMMP1 CP1 Switch Mux", pmmp1_cp1_switch_num);

static const char * const ain_pmmp_select_text[] = {
	"NONE", "MPWR1A", "MPWR1B", "MPWR1C", "MPWR2A", "MPWR2B"};

static const struct soc_enum ain1_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum ain2_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum ain3_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum ain4_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum ain5_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum ain6_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum dmic1l_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum dmic1r_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum dmic2l_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct soc_enum dmic2r_pmmp_select_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 6, ain_pmmp_select_text);

static const struct snd_kcontrol_new ain1_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN1 PMMP Selector Mux", ain1_pmmp_select_num);

static const struct snd_kcontrol_new ain2_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN2 PMMP Selector Mux", ain2_pmmp_select_num);

static const struct snd_kcontrol_new ain3_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN3 PMMP Selector Mux", ain3_pmmp_select_num);

static const struct snd_kcontrol_new ain4_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN4 PMMP Selector Mux", ain4_pmmp_select_num);

static const struct snd_kcontrol_new ain5_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN5 PMMP Selector Mux", ain5_pmmp_select_num);

static const struct snd_kcontrol_new ain6_pmmp_select_kctrl =
	SOC_DAPM_ENUM("AIN6 PMMP Selector Mux", ain6_pmmp_select_num);

static const struct snd_kcontrol_new dmic1l_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC1L PMMP Selector Mux", dmic1l_pmmp_select_num);

static const struct snd_kcontrol_new dmic1r_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC1R PMMP Selector Mux", dmic1r_pmmp_select_num);

static const struct snd_kcontrol_new dmic2l_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC2L PMMP Selector Mux", dmic2l_pmmp_select_num);

static const struct snd_kcontrol_new dmic2r_pmmp_select_kctrl =
	SOC_DAPM_ENUM("DMIC2R PMMP Selector Mux", dmic2r_pmmp_select_num);

static const char * const smart_pa_init_switch_text[] = {"Off", "On"};

static const struct soc_enum smart_pa_init_switch_num =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, smart_pa_init_switch_text);

static const struct snd_kcontrol_new smart_pa_init_switch_kctrl =
	SOC_DAPM_ENUM("smart pa init Switch Mux", smart_pa_init_switch_num);

static int ak4962_codec_enable_dmic(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u8 dmic_ctl_reg, dmic_setting;
	unsigned int dmic;
	int ret;

	ret = kstrtouint(strpbrk(w->name, "12"), 10, &dmic);
	if (ret < 0) {
		pr_err("%s: Invalid DMIC line on the codec\n", __func__);
		return -EINVAL;
	}

	dmic_ctl_reg = DIGITAL_MIC_1 + (dmic - 1);

	switch (dmic) {
	case 1:
		dmic_setting = 0x30 | AK4962_DMIC1_CLK_ENABLE | AK4962_DMIC1_CHANEL_LRP;
		break;
	case 2:
		dmic_setting = 0x30 | AK4962_DMIC2_CLK_ENABLE | AK4962_DMIC2_CHANEL_LRP;
		break;

	default:
		pr_err("%s: Invalid DMIC Selection\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, dmic_ctl_reg, 0x3A, dmic_setting);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, dmic_ctl_reg, 0x3A, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_pll_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	pr_info("\t[AK4962] %s(%d) 1:pll_status=%d\n", __func__, __LINE__, pll_status);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	  if (pll_status == 0) {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x01); /* PMPLL On */
/* pll_status++; */
		usleep_range(2000, 2100);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_2, 0x10, 0x10); /* PMAIF On */
	  }
		pll_status++;
		break;

	case SND_SOC_DAPM_PRE_PMD:
		/*
		if (snd_soc_read(codec, SYNC_DOMAIN_SELECTOR5) & 0x07) {
			snd_soc_update_bits(codec, SYNC_DOMAIN_SELECTOR5, 0x07, 0x00);
			usleep_range(16000, 16000);
		}
		*/
		if (pll_status == 1) {
		ak4962->state = AK4962_IDLE;
		ak4962_dsp_mode = DSP_MODE_OFF;
		snd_soc_update_bits(codec, FLOW_CONTROL_3, 0x01, 0x00);     /* DSPRSTN Off */
		snd_soc_update_bits(codec, FLOW_CONTROL_1, 0x03, 0x00);		/* PMSW Off */
		snd_soc_update_bits(codec, POWER_MANAGEMENT_2, 0x10, 0x00); /* PMAIF Off */
	/* rev0.15 */
	    if ((snd_soc_read(codec, IMPEDANCE_DETECTION) & 0x80) == 0) { /* IDST check */
		snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x00); /* PMPLL Off */
		}
	/* rev0.15 end */
	}
		pll_status--;
		break;
	}

	pr_info("\t[AK4962] %s(%d) 2:pll_status=%d\n", __func__, __LINE__, pll_status);

	return 0;
}

static int ak4962_codec_srca_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x10, 0x10);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR7);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x01, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x11, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_srcb_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x20, 0x20);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR7);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR7, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x02, 0x02);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x22, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_srcc_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x40, 0x40);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR8);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x04, 0x04);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x44, 0x00);
		break;
	}
	return 0;
}

static int ak4962_codec_srcd_setup(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, SRC_CLK_SETTING, 0x01, 0x01);
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x80, 0x80);
		val = snd_soc_read(codec, SYNC_DOMAIN_SELECTOR8);
		snd_soc_write(codec, SYNC_DOMAIN_SELECTOR8, val);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x08, 0x08);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, SRC_MUTE_CONTROL, 0x88, 0x00);
		break;
	}
	return 0;
}

/* Audio Sink Port: Source Select */
static const char * const audio_sink_source_select_text[] = {
	"ZERO", "SDTI1A", "SDTI1B", "SDTI1C", "SDTI1D", "SDTI2", "SDTI3", "SDTI4", "reserved1",
	"ADC1", "ADC2", "SBI1",	"SBI2", "SBI3", "DSPO1", "DSPO2", "DSPO3", "DSPO4",
	"DSPO5", "MIXAO", "MIXBO", "SRCAO", "SRCBO", "SRCCO", "SRCDO", "reserved2",
	"MIXCO", "MIXDO"
};

static const struct soc_enum dac1_source_select_enum =
	SOC_ENUM_SINGLE(DAC1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum dac2_source_select_enum =
	SOC_ENUM_SINGLE(DAC2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new dac1_source_select_kctrl =
	SOC_DAPM_ENUM("DAC1 Source Selector Mux", dac1_source_select_enum);

static const struct snd_kcontrol_new dac2_source_select_kctrl =
	SOC_DAPM_ENUM("DAC2 Source Selector Mux", dac2_source_select_enum);

static const struct soc_enum mixai1_source_select_enum =
	SOC_ENUM_SINGLE(MIXAI1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum mixai2_source_select_enum =
	SOC_ENUM_SINGLE(MIXAI2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new mixai1_source_select_kctrl =
	SOC_DAPM_ENUM("MIXAI1 Source Selector Mux", mixai1_source_select_enum);

static const struct snd_kcontrol_new mixai2_source_select_kctrl =
	SOC_DAPM_ENUM("MIXAI2 Source Selector Mux", mixai2_source_select_enum);

static const struct soc_enum mixbi1_source_select_enum =
	SOC_ENUM_SINGLE(MIXBI1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum mixbi2_source_select_enum =
	SOC_ENUM_SINGLE(MIXBI2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new mixbi1_source_select_kctrl =
	SOC_DAPM_ENUM("MIXBI1 Source Selector Mux", mixbi1_source_select_enum);

static const struct snd_kcontrol_new mixbi2_source_select_kctrl =
	SOC_DAPM_ENUM("MIXBI2 Source Selector Mux", mixbi2_source_select_enum);

static const struct soc_enum mixci1_source_select_enum =
	SOC_ENUM_SINGLE(MIXCI1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum mixci2_source_select_enum =
	SOC_ENUM_SINGLE(MIXCI2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new mixci1_source_select_kctrl =
	SOC_DAPM_ENUM("MIXCI1 Source Selector Mux", mixci1_source_select_enum);

static const struct snd_kcontrol_new mixci2_source_select_kctrl =
	SOC_DAPM_ENUM("MIXCI2 Source Selector Mux", mixci2_source_select_enum);

static const struct soc_enum mixdi1_source_select_enum =
	SOC_ENUM_SINGLE(MIXDI1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum mixdi2_source_select_enum =
	SOC_ENUM_SINGLE(MIXDI2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new mixdi1_source_select_kctrl =
	SOC_DAPM_ENUM("MIXDI1 Source Selector Mux", mixdi1_source_select_enum);

static const struct snd_kcontrol_new mixdi2_source_select_kctrl =
	SOC_DAPM_ENUM("MIXDI2 Source Selector Mux", mixdi2_source_select_enum);

static const struct soc_enum srcai_source_select_enum =
	SOC_ENUM_SINGLE(SRCAI_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum srcbi_source_select_enum =
	SOC_ENUM_SINGLE(SRCBI_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum srcci_source_select_enum =
	SOC_ENUM_SINGLE(SRCCI_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum srcdi_source_select_enum =
	SOC_ENUM_SINGLE(SRCDI_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new srcai_source_select_kctrl =
	SOC_DAPM_ENUM("SRCAI Source Selector Mux", srcai_source_select_enum);

static const struct snd_kcontrol_new srcbi_source_select_kctrl =
	SOC_DAPM_ENUM("SRCBI Source Selector Mux", srcbi_source_select_enum);

static const struct snd_kcontrol_new srcci_source_select_kctrl =
	SOC_DAPM_ENUM("SRCCI Source Selector Mux", srcci_source_select_enum);

static const struct snd_kcontrol_new srcdi_source_select_kctrl =
	SOC_DAPM_ENUM("SRCDI Source Selector Mux", srcdi_source_select_enum);

static const struct soc_enum dspi1_source_select_enum =
	SOC_ENUM_SINGLE(DSPI1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum dspi2_source_select_enum =
	SOC_ENUM_SINGLE(DSPI2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum dspi3_source_select_enum =
	SOC_ENUM_SINGLE(DSPI3_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum dspi4_source_select_enum =
	SOC_ENUM_SINGLE(DSPI4_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum dspi5_source_select_enum =
	SOC_ENUM_SINGLE(DSPI5_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new dspi1_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI1 Source Selector Mux", dspi1_source_select_enum);

static const struct snd_kcontrol_new dspi2_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI2 Source Selector Mux", dspi2_source_select_enum);

static const struct snd_kcontrol_new dspi3_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI3 Source Selector Mux", dspi3_source_select_enum);

static const struct snd_kcontrol_new dspi4_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI4 Source Selector Mux", dspi4_source_select_enum);

static const struct snd_kcontrol_new dspi5_source_select_kctrl =
	SOC_DAPM_ENUM("DSPI5 Source Selector Mux", dspi5_source_select_enum);

static const struct soc_enum sbo1l_source_select_enum =
	SOC_ENUM_SINGLE(SBO1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sbo1r_source_select_enum =
	SOC_ENUM_SINGLE(SBO1_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sbo2l_source_select_enum =
	SOC_ENUM_SINGLE(SBO2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sbo2r_source_select_enum =
	SOC_ENUM_SINGLE(SBO2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new sbo1l_source_select_kctrl =
	SOC_DAPM_ENUM("SBO1L Source Selector Mux", sbo1l_source_select_enum);

static const struct snd_kcontrol_new sbo1r_source_select_kctrl =
	SOC_DAPM_ENUM("SBO1R Source Selector Mux", sbo1r_source_select_enum);

static const struct snd_kcontrol_new sbo2l_source_select_kctrl =
	SOC_DAPM_ENUM("SBO2L Source Selector Mux", sbo2l_source_select_enum);

static const struct snd_kcontrol_new sbo2r_source_select_kctrl =
	SOC_DAPM_ENUM("SBO2R Source Selector Mux", sbo2r_source_select_enum);

static const char * const dac_select_text[] = {"Off", "On"};

static const struct soc_enum hp_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout1l_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout1r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2l_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2ld_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2r_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum lout2rd_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct soc_enum rcv_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, dac_select_text);

static const struct snd_kcontrol_new hp_select_kctrl =
	SOC_DAPM_ENUM("HP Virt Switch Mux", hp_select_enum);

static const struct snd_kcontrol_new hp_anc_select_kctrl =
	SOC_DAPM_ENUM("HP ANC Virt Switch Mux", hp_select_enum);

static const struct snd_kcontrol_new lout1l_select_kctrl =
	SOC_DAPM_ENUM("LOUT1L Virt Switch Mux", lout1l_select_enum);

static const struct snd_kcontrol_new lout1r_select_kctrl =
	SOC_DAPM_ENUM("LOUT1R Virt Switch Mux", lout1r_select_enum);

static const struct snd_kcontrol_new lout2l_select_kctrl =
	SOC_DAPM_ENUM("LOUT2L Virt Switch Mux", lout2l_select_enum);

static const struct snd_kcontrol_new lout2ld_select_kctrl =
	SOC_DAPM_ENUM("LOUT2LD Virt Switch Mux", lout2ld_select_enum);

static const struct snd_kcontrol_new lout2r_select_kctrl =
	SOC_DAPM_ENUM("LOUT2R Virt Switch Mux", lout2r_select_enum);

static const struct snd_kcontrol_new lout2rd_select_kctrl =
	SOC_DAPM_ENUM("LOUT2RD Virt Switch Mux", lout2rd_select_enum);

static const struct snd_kcontrol_new rcv_select_kctrl =
	SOC_DAPM_ENUM("RCV Virt Switch Mux", rcv_select_enum);

static const char * const mixer_select_text[] = {"Off", "On"};

static const struct soc_enum mixera_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mixer_select_text);

static const struct soc_enum mixerb_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mixer_select_text);

static const struct soc_enum mixerc_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mixer_select_text);

static const struct soc_enum mixerd_select_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 2, mixer_select_text);

static const struct snd_kcontrol_new mixera_select_kctrl =
	SOC_DAPM_ENUM("MIXAO Virt Switch Mux", mixera_select_enum);

static const struct snd_kcontrol_new mixerb_select_kctrl =
	SOC_DAPM_ENUM("MIXBO Virt Switch Mux", mixerb_select_enum);

static const struct snd_kcontrol_new mixerc_select_kctrl =
	SOC_DAPM_ENUM("MIXCO Virt Switch Mux", mixerc_select_enum);

static const struct snd_kcontrol_new mixerd_select_kctrl =
	SOC_DAPM_ENUM("MIXDO Virt Switch Mux", mixerd_select_enum);

static const struct snd_kcontrol_new dspo1_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI1_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI1_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI1_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI1_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI1_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo2_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI2_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI2_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI2_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI2_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI2_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo3_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI3_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI3_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI3_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI3_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI3_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo4_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI4_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI4_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI4_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI4_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI4_VIRT_MIXER, 4, 1, 0),
};

static const struct snd_kcontrol_new dspo5_mixer_kctrl[] = {
	SOC_DAPM_SINGLE("DSPI1_Switch", DSPI5_VIRT_MIXER, 0, 1, 0),
	SOC_DAPM_SINGLE("DSPI2_Switch", DSPI5_VIRT_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("DSPI3_Switch", DSPI5_VIRT_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("DSPI4_Switch", DSPI5_VIRT_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("DSPI5_Switch", DSPI5_VIRT_MIXER, 4, 1, 0),
};


/*
*Todo: Have separate dapm widgets for I2S and Slimbus.
 */
static const struct snd_soc_dapm_widget ak4962_dapm_widgets[] = {

	SND_SOC_DAPM_AIF_IN_E("SB1 PB", "SB1 Playback", 0, SND_SOC_NOPM, SB1_PB,
				0, ak4962_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SB2 PB", "SB2 Playback", 0, SND_SOC_NOPM, SB2_PB,
				0, ak4962_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("SB3 PB", "SB3 Playback", 0, SND_SOC_NOPM, SB3_PB,
				0, ak4962_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, AK4962_RX1, 0,
				&slim_rx_mux[AK4962_RX1]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, AK4962_RX2, 0,
				&slim_rx_mux[AK4962_RX2]),
	SND_SOC_DAPM_MUX("SLIM RX3 MUX", SND_SOC_NOPM, AK4962_RX3, 0,
				&slim_rx_mux[AK4962_RX3]),
	SND_SOC_DAPM_MUX("SLIM RX4 MUX", SND_SOC_NOPM, AK4962_RX4, 0,
				&slim_rx_mux[AK4962_RX4]),
	SND_SOC_DAPM_MUX("SLIM RX5 MUX", SND_SOC_NOPM, AK4962_RX5, 0,
				&slim_rx_mux[AK4962_RX5]),
	SND_SOC_DAPM_MUX("SLIM RX6 MUX", SND_SOC_NOPM, AK4962_RX6, 0,
				&slim_rx_mux[AK4962_RX6]),

	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX3", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX4", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX5", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX6", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_AIF_OUT_E("SB1 CAP", "SB1 Capture", 0, SND_SOC_NOPM, SB1_CAP,
				0, ak4962_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SB2 CAP", "SB2 Capture", 0, SND_SOC_NOPM, SB2_CAP,
				0, ak4962_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("SB3 CAP", "SB3 Capture", 0, SND_SOC_NOPM, SB3_CAP,
				0, ak4962_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER("SB1_CAP Mixer", SND_SOC_NOPM, SB1_CAP, 0,
		slim_cap1_mixer, ARRAY_SIZE(slim_cap1_mixer)),

	SND_SOC_DAPM_MIXER("SB2_CAP Mixer", SND_SOC_NOPM, SB2_CAP, 0,
		slim_cap2_mixer, ARRAY_SIZE(slim_cap2_mixer)),

	SND_SOC_DAPM_MIXER("SB3_CAP Mixer", SND_SOC_NOPM, SB3_CAP, 0,
		slim_cap3_mixer, ARRAY_SIZE(slim_cap3_mixer)),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_INPUT("DMIC1L"),

	SND_SOC_DAPM_INPUT("DMIC1R"),

	SND_SOC_DAPM_INPUT("DMIC2L"),

	SND_SOC_DAPM_INPUT("DMIC2R"),

	SND_SOC_DAPM_ADC_E("ADC DMIC1", NULL, SND_SOC_NOPM, 0, 0,
		ak4962_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("MIC1L Selector", SND_SOC_NOPM, 0, 0, &mic1l_select_kctrl),

	SND_SOC_DAPM_MUX("MIC1R Selector", SND_SOC_NOPM, 0, 0, &mic1r_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC1L Selector", SND_SOC_NOPM, 0, 0, &dmic1l_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC1R Selector", SND_SOC_NOPM, 0, 0, &dmic1r_select_kctrl),

	SND_SOC_DAPM_INPUT("DMIC2"),

	SND_SOC_DAPM_ADC_E("ADC DMIC2", NULL, SND_SOC_NOPM, 0, 0,
		ak4962_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("MIC2L Selector", SND_SOC_NOPM, 0, 0, &mic2l_select_kctrl),

	SND_SOC_DAPM_MUX("MIC2R Selector", SND_SOC_NOPM, 0, 0, &mic2r_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC2L Selector", SND_SOC_NOPM, 0, 0, &dmic2l_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC2R Selector", SND_SOC_NOPM, 0, 0, &dmic2r_select_kctrl),

	/* Smart PA */
	SND_SOC_DAPM_INPUT("Smart PA Input"),

	SND_SOC_DAPM_OUTPUT("Smart PA Output"),

	SND_SOC_DAPM_MUX("Smart PA Init Switch", SND_SOC_NOPM, 0, 0, &smart_pa_init_switch_kctrl),

	/* MIC Power */
	SND_SOC_DAPM_INPUT("MRF1"),

	SND_SOC_DAPM_INPUT("MRF2"),

	SND_SOC_DAPM_MUX("PMMP1 CP1 Switch", SND_SOC_NOPM, 0, 0, &pmmp1_cp1_switch_kctrl),

	SND_SOC_DAPM_MICBIAS_E("PMMP1A", POWER_MANAGEMENT_4, 0, 0,
		ak4962_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP1B", POWER_MANAGEMENT_4, 1, 0,
		ak4962_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP1C", POWER_MANAGEMENT_4, 2, 0,
		ak4962_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP2A", POWER_MANAGEMENT_4, 3, 0,
		ak4962_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MICBIAS_E("PMMP2B", POWER_MANAGEMENT_4, 4, 0,
		ak4962_codec_enable_micbias, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MUX("AIN1 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain1_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN2 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain2_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN3 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain3_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN4 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain4_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN5 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain5_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("AIN6 PMMP Selector", SND_SOC_NOPM, 0, 0, &ain6_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC1L PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic1l_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC1R PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic1r_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC2L PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic2l_pmmp_select_kctrl),

	SND_SOC_DAPM_MUX("DMIC2R PMMP Selector", SND_SOC_NOPM, 0, 0, &dmic2r_pmmp_select_kctrl),

	/* Analog Mic Inputs */
	SND_SOC_DAPM_INPUT("AIN1"),

	SND_SOC_DAPM_INPUT("AIN2"),

	SND_SOC_DAPM_INPUT("AIN3"),

	SND_SOC_DAPM_INPUT("AIN4"),

	SND_SOC_DAPM_INPUT("AIN5"),

	SND_SOC_DAPM_INPUT("AIN6"),

	SND_SOC_DAPM_MICBIAS("PMAIN1", POWER_MANAGEMENT_5, 0, 0),

	SND_SOC_DAPM_MICBIAS("PMAIN2", POWER_MANAGEMENT_5, 1, 0),

	SND_SOC_DAPM_MICBIAS("PMAIN3", POWER_MANAGEMENT_5, 2, 0),

	SND_SOC_DAPM_MICBIAS("PMAIN4", POWER_MANAGEMENT_5, 3, 0),

	SND_SOC_DAPM_MICBIAS("PMAIN5", POWER_MANAGEMENT_5, 4, 0),

	SND_SOC_DAPM_MICBIAS("PMAIN6", POWER_MANAGEMENT_5, 5, 0),

	SND_SOC_DAPM_MUX("Mic_1L input Selector", SND_SOC_NOPM, 0, 0,
		&mic1L_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_1R input Selector", SND_SOC_NOPM, 0, 0,
		&mic1R_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_2L input Selector", SND_SOC_NOPM, 0, 0,
		&mic2L_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_2R input Selector", SND_SOC_NOPM, 0, 0,
		&mic2R_input_select_kctrl),

	SND_SOC_DAPM_MUX("Mic_3 input Selector", SND_SOC_NOPM, 0, 0,
		&mic3_input_select_kctrl),

	SND_SOC_DAPM_MUX("AIN3 input Selector", SND_SOC_NOPM, 0, 0,
		&ain3_input_select_kctrl),

	SND_SOC_DAPM_MUX("AIN4 input Selector", SND_SOC_NOPM, 0, 0,
		&ain4_input_select_kctrl),

	/* ADC stuff */
	SND_SOC_DAPM_ADC("ADC_1L", NULL, POWER_MANAGEMENT_6, 0, 0),

	SND_SOC_DAPM_ADC("ADC_1R", NULL, POWER_MANAGEMENT_6, 1, 0),

	SND_SOC_DAPM_ADC("ADC_2L", NULL, POWER_MANAGEMENT_6, 2, 0),

	SND_SOC_DAPM_ADC("ADC_2R", NULL, POWER_MANAGEMENT_6, 3, 0),

	SND_SOC_DAPM_ADC("ADC_3", NULL, POWER_MANAGEMENT_6, 4, 0),

	SND_SOC_DAPM_OUTPUT("DMIC3_Output"),

	/* DAC stuff */
	SND_SOC_DAPM_MIXER("DAC1 Lch Mixer", SND_SOC_NOPM, 0, 0,
		dac1_lch_mixer_kctrl, ARRAY_SIZE(dac1_lch_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DAC1 Rch Mixer", SND_SOC_NOPM, 0, 0,
		dac1_rch_mixer_kctrl, ARRAY_SIZE(dac1_rch_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DAC2 Lch Mixer", SND_SOC_NOPM, 0, 0,
		dac2_lch_mixer_kctrl, ARRAY_SIZE(dac2_lch_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DAC2 Rch Mixer", SND_SOC_NOPM, 0, 0,
		dac2_rch_mixer_kctrl, ARRAY_SIZE(dac2_rch_mixer_kctrl)),

	SND_SOC_DAPM_MUX("HP Virt Switch", SND_SOC_NOPM, 0, 0, &hp_select_kctrl),

	SND_SOC_DAPM_MUX("HP ANC Virt Switch", SND_SOC_NOPM, 0, 0, &hp_anc_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT1L Virt Switch", SND_SOC_NOPM, 0, 0, &lout1l_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT1R Virt Switch", SND_SOC_NOPM, 0, 0, &lout1r_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT2L Virt Switch", SND_SOC_NOPM, 0, 0, &lout2l_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT2LD Virt Switch", SND_SOC_NOPM, 0, 0, &lout2ld_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT2R Virt Switch", SND_SOC_NOPM, 0, 0, &lout2r_select_kctrl),

	SND_SOC_DAPM_MUX("LOUT2RD Virt Switch", SND_SOC_NOPM, 0, 0, &lout2rd_select_kctrl),

	SND_SOC_DAPM_MUX("RCV Virt Switch", SND_SOC_NOPM, 0, 0, &rcv_select_kctrl),

	SND_SOC_DAPM_DAC_E("DAC1", NULL, SND_SOC_NOPM, 0, 0,
		ak4962_codec_enable_dac, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("DAC2", NULL, SND_SOC_NOPM, 0, 0,
		ak4962_codec_enable_dac, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* HP outputs */
	SND_SOC_DAPM_HP("HP", ak4962_codec_enable_hp),

	SND_SOC_DAPM_HP("HP ANC", ak4962_codec_enable_hp),

	SND_SOC_DAPM_HP("LOUT1L", ak4962_codec_enable_lout1l),

	SND_SOC_DAPM_HP("LOUT1R", ak4962_codec_enable_lout1r),

	SND_SOC_DAPM_HP("LOUT2L", ak4962_codec_enable_lout2l),

/* SND_SOC_DAPM_HP("LOUT2LD", ak4962_codec_enable_lout2ld), */

	SND_SOC_DAPM_HP("LINEOUT1", ak4962_codec_enable_lout2ldp),

	SND_SOC_DAPM_HP("LINEOUT3", ak4962_codec_enable_lout2ldn),

	SND_SOC_DAPM_HP("LOUT2R", ak4962_codec_enable_lout2r),

/* SND_SOC_DAPM_HP("LOUT2RD", ak4962_codec_enable_lout2rd), */

	SND_SOC_DAPM_HP("LINEOUT2", ak4962_codec_enable_lout2rdp),

	SND_SOC_DAPM_HP("LINEOUT4", ak4962_codec_enable_lout2rdn),

	SND_SOC_DAPM_HP("RCV", ak4962_codec_enable_rcv),

	/* Power Supply */
	SND_SOC_DAPM_SUPPLY_S("Charge Pump1", 1, SND_SOC_NOPM, 0, 0,
		ak4962_codec_enable_charge_pump_1, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY_S("Charge Pump3", 1, SND_SOC_NOPM, 0, 0,
		ak4962_codec_enable_charge_pump_3, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),	/* rev0.15 */

	SND_SOC_DAPM_SUPPLY("PMSW", SND_SOC_NOPM, 0, 0, ak4962_codec_enable_pmsw,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* PLL stuff */
	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL CLK", SND_SOC_NOPM, 0, 0,
		ak4962_codec_pll_setup,	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("PLL CLK ANC", SND_SOC_NOPM, 0, 0,
		ak4962_codec_pll_setup,	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* MIXER stuff */
	SND_SOC_DAPM_MUX("MIXAO Virt Switch", SND_SOC_NOPM, 0, 0, &mixera_select_kctrl),

	SND_SOC_DAPM_MUX("MIXBO Virt Switch", SND_SOC_NOPM, 0, 0, &mixerb_select_kctrl),

	SND_SOC_DAPM_MUX("MIXCO Virt Switch", SND_SOC_NOPM, 0, 0, &mixerc_select_kctrl),

	SND_SOC_DAPM_MUX("MIXDO Virt Switch", SND_SOC_NOPM, 0, 0, &mixerd_select_kctrl),

	/* SRC stuff */
	SND_SOC_DAPM_PGA_E("SRCA", POWER_MANAGEMENT_2, 0, 0, NULL, 0,
		ak4962_codec_srca_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SRCB", POWER_MANAGEMENT_2, 1, 0, NULL, 0,
		ak4962_codec_srcb_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SRCC", POWER_MANAGEMENT_2, 2, 0, NULL, 0,
		ak4962_codec_srcc_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SRCD", POWER_MANAGEMENT_2, 3, 0, NULL, 0,
		ak4962_codec_srcd_setup, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* DSP stuff */
	SND_SOC_DAPM_MIXER("DSPO1 Mixer", SND_SOC_NOPM, 0, 0,
		dspo1_mixer_kctrl, ARRAY_SIZE(dspo1_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO2 Mixer", SND_SOC_NOPM, 0, 0,
		dspo2_mixer_kctrl, ARRAY_SIZE(dspo2_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO3 Mixer", SND_SOC_NOPM, 0, 0,
		dspo3_mixer_kctrl, ARRAY_SIZE(dspo3_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO4 Mixer", SND_SOC_NOPM, 0, 0,
		dspo4_mixer_kctrl, ARRAY_SIZE(dspo4_mixer_kctrl)),

	SND_SOC_DAPM_MIXER("DSPO5 Mixer", SND_SOC_NOPM, 0, 0,
		dspo5_mixer_kctrl, ARRAY_SIZE(dspo5_mixer_kctrl)),

	/* Digital Path Staff */
	SND_SOC_DAPM_MUX("SBO1L Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo1l_source_select_kctrl),

	SND_SOC_DAPM_MUX("SBO1R Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo1r_source_select_kctrl),

	SND_SOC_DAPM_MUX("SBO2L Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo2l_source_select_kctrl),

	SND_SOC_DAPM_MUX("SBO2R Source Selector", SND_SOC_NOPM, 0, 0,
		&sbo2r_source_select_kctrl),

	SND_SOC_DAPM_MUX("DAC1 Source Selector", SND_SOC_NOPM, 0, 0,
		&dac1_source_select_kctrl),

	SND_SOC_DAPM_MUX("DAC2 Source Selector", SND_SOC_NOPM, 0, 0,
		&dac2_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi1_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi2_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI3 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi3_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI4 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi4_source_select_kctrl),

	SND_SOC_DAPM_MUX("DSPI5 Source Selector", SND_SOC_NOPM, 0, 0,
		&dspi5_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXAI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixai1_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXAI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixai2_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXBI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixbi1_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXBI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixbi2_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXCI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixci1_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXCI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixci2_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXDI1 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixdi1_source_select_kctrl),

	SND_SOC_DAPM_MUX("MIXDI2 Source Selector", SND_SOC_NOPM, 0, 0,
		&mixdi2_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCAI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcai_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCBI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcbi_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCCI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcci_source_select_kctrl),

	SND_SOC_DAPM_MUX("SRCDI Source Selector", SND_SOC_NOPM, 0, 0,
		&srcdi_source_select_kctrl),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* DMIC path */
	{"DMIC1L Selector", "dmdat1l", "DMIC1L"},
	{"DMIC1L Selector", "dmdat1r", "DMIC1R"},
	{"DMIC1L Selector", "dmdat2l", "DMIC2L"},
	{"DMIC1L Selector", "dmdat2r", "DMIC2R"},
	{"DMIC1R Selector", "dmdat1l", "DMIC1L"},
	{"DMIC1R Selector", "dmdat1r", "DMIC1R"},
	{"DMIC1R Selector", "dmdat2l", "DMIC2L"},
	{"DMIC1R Selector", "dmdat2r", "DMIC2R"},

	{"ADC DMIC1", NULL, "DMIC1L Selector"},
	{"ADC DMIC1", NULL, "DMIC1R Selector"},
/* {"ADC DMIC1", NULL, "Charge Pump1"}, */
	{"MIC1L Selector", "Digital", "ADC DMIC1"},
	{"MIC1R Selector", "Digital", "ADC DMIC1"},

	{"DMIC2L Selector", "dmdat1l", "DMIC1L"},
	{"DMIC2L Selector", "dmdat1r", "DMIC1R"},
	{"DMIC2L Selector", "dmdat2l", "DMIC2L"},
	{"DMIC2L Selector", "dmdat2r", "DMIC2R"},
	{"DMIC2R Selector", "dmdat1l", "DMIC1L"},
	{"DMIC2R Selector", "dmdat1r", "DMIC1R"},
	{"DMIC2R Selector", "dmdat2l", "DMIC2L"},
	{"DMIC2R Selector", "dmdat2r", "DMIC2R"},

	{"ADC DMIC2", NULL, "DMIC2L Selector"},
	{"ADC DMIC2", NULL, "DMIC2R Selector"},
/* {"ADC DMIC2", NULL, "Charge Pump1"}, */
	{"MIC2L Selector", "Digital", "ADC DMIC2"},
	{"MIC2R Selector", "Digital", "ADC DMIC2"},

	/* MIC power path */
	{"PMMP2A", NULL, "Charge Pump1"},
	{"PMMP2B", NULL, "Charge Pump1"},
	{"PMMP1 CP1 Switch", "On", "Charge Pump1"},
	{"PMMP1A", NULL, "PMMP1 CP1 Switch"},
	{"PMMP1B", NULL, "PMMP1 CP1 Switch"},
	{"PMMP1C", NULL, "PMMP1 CP1 Switch"},

	{"PMMP1A", NULL, "MRF1"},
	{"PMMP1B", NULL, "MRF1"},
	{"PMMP1C", NULL, "MRF1"},
	{"PMMP2A", NULL, "MRF2"},
	{"PMMP2B", NULL, "MRF2"},

	{"DMIC1L PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC1L PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC1L PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC1L PMMP Selector", "MPWR2A", "PMMP2A"},
	{"DMIC1L PMMP Selector", "MPWR2B", "PMMP2B"},

	{"DMIC1R PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC1R PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC1R PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC1R PMMP Selector", "MPWR2A", "PMMP2A"},
	{"DMIC1R PMMP Selector", "MPWR2B", "PMMP2B"},

	{"DMIC2L PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC2L PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC2L PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC2L PMMP Selector", "MPWR2A", "PMMP2A"},
	{"DMIC2L PMMP Selector", "MPWR2B", "PMMP2B"},

	{"DMIC2R PMMP Selector", "MPWR1A", "PMMP1A"},
	{"DMIC2R PMMP Selector", "MPWR1B", "PMMP1B"},
	{"DMIC2R PMMP Selector", "MPWR1C", "PMMP1C"},
	{"DMIC2R PMMP Selector", "MPWR2A", "PMMP2A"},
	{"DMIC2R PMMP Selector", "MPWR2B", "PMMP2B"},

	{"ADC DMIC1", NULL, "DMIC1L PMMP Selector"},
	{"ADC DMIC1", NULL, "DMIC1R PMMP Selector"},
	{"ADC DMIC2", NULL, "DMIC2L PMMP Selector"},
	{"ADC DMIC2", NULL, "DMIC2R PMMP Selector"},

	{"AIN1 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN1 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN1 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN1 PMMP Selector", "MPWR2A", "PMMP2A"},
	{"AIN1 PMMP Selector", "MPWR2B", "PMMP2B"},

	{"AIN2 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN2 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN2 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN2 PMMP Selector", "MPWR2A", "PMMP2A"},
	{"AIN2 PMMP Selector", "MPWR2B", "PMMP2B"},

	{"AIN3 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN3 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN3 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN3 PMMP Selector", "MPWR2A", "PMMP2A"},
	{"AIN3 PMMP Selector", "MPWR2B", "PMMP2B"},

	{"AIN4 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN4 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN4 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN4 PMMP Selector", "MPWR2A", "PMMP2A"},
	{"AIN4 PMMP Selector", "MPWR2B", "PMMP2B"},

	{"AIN5 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN5 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN5 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN5 PMMP Selector", "MPWR2A", "PMMP2A"},
	{"AIN5 PMMP Selector", "MPWR2B", "PMMP2B"},

	{"AIN6 PMMP Selector", "MPWR1A", "PMMP1A"},
	{"AIN6 PMMP Selector", "MPWR1B", "PMMP1B"},
	{"AIN6 PMMP Selector", "MPWR1C", "PMMP1C"},
	{"AIN6 PMMP Selector", "MPWR2A", "PMMP2A"},
	{"AIN6 PMMP Selector", "MPWR2B", "PMMP2B"},

	{"AIN1", NULL, "AIN1 PMMP Selector"},
	{"AIN2", NULL, "AIN2 PMMP Selector"},
	{"PMAIN3", NULL, "AIN3 PMMP Selector"},
	{"PMAIN4", NULL, "AIN4 PMMP Selector"},
	{"AIN5", NULL, "AIN5 PMMP Selector"},
	{"AIN6", NULL, "AIN6 PMMP Selector"},

	/* AMIC path */
	{"PMAIN1", NULL, "AIN1"},
	{"PMAIN2", NULL, "AIN2"},
	{"PMAIN5", NULL, "AIN5"},
	{"PMAIN6", NULL, "AIN6"},

	{"AIN3 input Selector", "AIN", "AIN3"},
	{"AIN3 input Selector", "ANC", "SLIM RX1"},
	{"PMAIN3", NULL, "AIN3 input Selector"},

	{"AIN4 input Selector", "AIN", "AIN4"},
	{"AIN4 input Selector", "ANC", "SLIM RX1"},
	{"PMAIN4", NULL, "AIN4 input Selector"},

	{"Mic_1L input Selector", "AIN1", "PMAIN1"},
	{"Mic_1L input Selector", "AIN2", "PMAIN2"},
	{"Mic_1L input Selector", "AIN3", "PMAIN3"},
	{"Mic_1L input Selector", "AIN4", "PMAIN4"},
	{"Mic_1L input Selector", "AIN5", "PMAIN5"},
	{"Mic_1L input Selector", "AIN6", "PMAIN6"},

	{"Mic_1R input Selector", "AIN1", "PMAIN1"},
	{"Mic_1R input Selector", "AIN2", "PMAIN2"},
	{"Mic_1R input Selector", "AIN3", "PMAIN3"},
	{"Mic_1R input Selector", "AIN4", "PMAIN4"},
	{"Mic_1R input Selector", "AIN5", "PMAIN5"},
	{"Mic_1R input Selector", "AIN6", "PMAIN6"},

	{"Mic_2L input Selector", "AIN1", "PMAIN1"},
	{"Mic_2L input Selector", "AIN2", "PMAIN2"},
	{"Mic_2L input Selector", "AIN3", "PMAIN3"},
	{"Mic_2L input Selector", "AIN4", "PMAIN4"},
	{"Mic_2L input Selector", "AIN5", "PMAIN5"},
	{"Mic_2L input Selector", "AIN6", "PMAIN6"},

	{"Mic_2R input Selector", "AIN1", "PMAIN1"},
	{"Mic_2R input Selector", "AIN2", "PMAIN2"},
	{"Mic_2R input Selector", "AIN3", "PMAIN3"},
	{"Mic_2R input Selector", "AIN4", "PMAIN4"},
	{"Mic_2R input Selector", "AIN5", "PMAIN5"},
	{"Mic_2R input Selector", "AIN6", "PMAIN6"},

	{"Mic_3 input Selector", "AIN1", "PMAIN1"},
	{"Mic_3 input Selector", "AIN2", "PMAIN2"},
	{"Mic_3 input Selector", "AIN3", "PMAIN3"},
	{"Mic_3 input Selector", "AIN4", "PMAIN4"},
	{"Mic_3 input Selector", "AIN5", "PMAIN5"},
	{"Mic_3 input Selector", "AIN6", "PMAIN6"},

	{"ADC_1L", NULL, "Mic_1L input Selector"},
	{"ADC_1R", NULL, "Mic_1R input Selector"},
	{"ADC_2L", NULL, "Mic_2L input Selector"},
	{"ADC_2R", NULL, "Mic_2R input Selector"},
	{"ADC_3", NULL, "Mic_3 input Selector"},
	{"DMIC3_Output", NULL, "ADC_3"},

	{"ADC_1L", NULL, "Charge Pump1"},
	{"ADC_1R", NULL, "Charge Pump1"},
	{"ADC_2L", NULL, "Charge Pump1"},
	{"ADC_2R", NULL, "Charge Pump1"},

	{"MIC1L Selector", "Analog", "ADC_1L"},
	{"MIC1R Selector", "Analog", "ADC_1R"},
	{"MIC2L Selector", "Analog", "ADC_2L"},
	{"MIC2R Selector", "Analog", "ADC_2R"},

	/* DAC path */
	{"DAC1 Lch Mixer", "Lch_Switch", "DAC1 Source Selector"},
	{"DAC1 Lch Mixer", "Rch_Switch", "DAC1 Source Selector"},
	{"DAC1 Lch Mixer", "Anc_Switch", "MIC1L Selector"},

	{"DAC1 Rch Mixer", "Lch_Switch", "DAC1 Source Selector"},
	{"DAC1 Rch Mixer", "Rch_Switch", "DAC1 Source Selector"},
	{"DAC1 Rch Mixer", "Anc_Switch", "MIC1R Selector"},

	{"DAC2 Lch Mixer", "Lch_Switch", "DAC2 Source Selector"},
	{"DAC2 Lch Mixer", "Rch_Switch", "DAC2 Source Selector"},
	{"DAC2 Rch Mixer", "Lch_Switch", "DAC2 Source Selector"},
	{"DAC2 Rch Mixer", "Rch_Switch", "DAC2 Source Selector"},

	{"DAC1", NULL, "DAC1 Lch Mixer"},
	{"DAC1", NULL, "DAC1 Rch Mixer"},
	{"DAC2", NULL, "DAC2 Lch Mixer"},
	{"DAC2", NULL, "DAC2 Rch Mixer"},

	{"DAC1", NULL, "Charge Pump3"},
	{"DAC2", NULL, "Charge Pump3"},

	/* Output path */
	{"HP Virt Switch", "On", "DAC1"},
	{"HP ANC Virt Switch", "On", "DAC1"},
	{"LOUT1L Virt Switch", "On", "DAC2"},
	{"LOUT1R Virt Switch", "On", "DAC2"},
	{"LOUT2L Virt Switch", "On", "DAC2"},
	{"LOUT2R Virt Switch", "On", "DAC2"},
	{"LOUT2LD Virt Switch", "On", "DAC2"},
	{"LOUT2RD Virt Switch", "On", "DAC2"},
	{"RCV Virt Switch", "On", "DAC2"},

	{"HP", NULL, "HP Virt Switch"},
	{"HP ANC", NULL, "HP ANC Virt Switch"},
	{"LOUT1L", NULL, "LOUT1L Virt Switch"},
	{"LOUT1R", NULL, "LOUT1R Virt Switch"},
	{"LOUT2L", NULL, "LOUT2L Virt Switch"},
	{"LOUT2R", NULL, "LOUT2R Virt Switch"},
/* {"LOUT2LD", NULL, "LOUT2LD Virt Switch"}, */
	{"LINEOUT1", NULL, "LOUT2LD Virt Switch"},
	{"LINEOUT3", NULL, "LOUT2LD Virt Switch"},
/* {"LOUT2RD", NULL, "LOUT2RD Virt Switch"}, */
	{"LINEOUT2", NULL, "LOUT2RD Virt Switch"},
	{"LINEOUT4", NULL, "LOUT2RD Virt Switch"},
	{"RCV", NULL, "RCV Virt Switch"},

	/* PLL CLK */
	{"PLL CLK", NULL, "RX_BIAS"},

	/* SLIM_MUX("SB1_PB", "SB1 PB") */
	{"SLIM RX1 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX2 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX3 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX4 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX5 MUX", "SB1_PB", "SB1 PB"},
	{"SLIM RX6 MUX", "SB1_PB", "SB1 PB"},

	/* SLIM_MUX("SB2_PB", "SB2 PB") */
	{"SLIM RX1 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX2 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX3 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX4 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX5 MUX", "SB2_PB", "SB2 PB"},
	{"SLIM RX6 MUX", "SB2_PB", "SB2 PB"},

	/* SLIM_MUX("SB3_PB", "SB3 PB") */
	{"SLIM RX1 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX2 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX3 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX4 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX5 MUX", "SB3_PB", "SB3 PB"},
	{"SLIM RX6 MUX", "SB3_PB", "SB3 PB"},

	{"SLIM RX1", NULL, "SLIM RX1 MUX"},
	{"SLIM RX2", NULL, "SLIM RX2 MUX"},
	{"SLIM RX3", NULL, "SLIM RX3 MUX"},
	{"SLIM RX4", NULL, "SLIM RX4 MUX"},
	{"SLIM RX5", NULL, "SLIM RX5 MUX"},
	{"SLIM RX6", NULL, "SLIM RX6 MUX"},

	{"SB1_CAP Mixer", "SLIM TX7",  "SBO1L Source Selector"},
	{"SB1_CAP Mixer", "SLIM TX8",  "SBO1R Source Selector"},
	{"SB1_CAP Mixer", "SLIM TX9",  "SBO2L Source Selector"},
	{"SB1_CAP Mixer", "SLIM TX10", "SBO2R Source Selector"},

	{"SB2_CAP Mixer", "SLIM TX7",  "SBO1L Source Selector"},
	{"SB2_CAP Mixer", "SLIM TX8",  "SBO1R Source Selector"},
	{"SB2_CAP Mixer", "SLIM TX9",  "SBO2L Source Selector"},
	{"SB2_CAP Mixer", "SLIM TX10", "SBO2R Source Selector"},

	{"SB3_CAP Mixer", "SLIM TX7",  "SBO1L Source Selector"},
	{"SB3_CAP Mixer", "SLIM TX8",  "SBO1R Source Selector"},
	{"SB3_CAP Mixer", "SLIM TX9",  "SBO2L Source Selector"},
	{"SB3_CAP Mixer", "SLIM TX10", "SBO2R Source Selector"},

	{"SB1 CAP", NULL, "SB1_CAP Mixer"},
	{"SB2 CAP", NULL, "SB2_CAP Mixer"},
	{"SB3 CAP", NULL, "SB3_CAP Mixer"},

	{"SB1 PB", NULL, "PLL CLK"},
	{"SB2 PB", NULL, "PLL CLK"},
	{"SB3 PB", NULL, "PLL CLK"},
	{"SB1 CAP", NULL, "PLL CLK"},
	{"SB2 CAP", NULL, "PLL CLK"},
	{"SB3 CAP", NULL, "PLL CLK"},

	{"HP ANC", NULL, "PLL CLK ANC"},

	/* Smart PA Init Path */
	{"Smart PA Init Switch", "On", "Smart PA Input"},
	{"Smart PA Output", NULL, "Smart PA Init Switch"},
	{"Smart PA Output", NULL, "PLL CLK"},
	{"Smart PA Output", NULL, "SDTO2 Source Selector"},
	{"Smart PA Output", NULL, "SDTO3 Source Selector"},
	{"Smart PA Output", NULL, "SDTO4 Source Selector"},

	/* SRC path */
	{"SRCA", NULL, "PMSW"},
	{"SRCB", NULL, "PMSW"},
	{"SRCC", NULL, "PMSW"},
	{"SRCD", NULL, "PMSW"},

	{"SRCA", NULL, "SRCAI Source Selector"},
	{"SRCB", NULL, "SRCBI Source Selector"},
	{"SRCC", NULL, "SRCCI Source Selector"},
	{"SRCD", NULL, "SRCDI Source Selector"},

	/* MIXER path */
	{"MIXAO Virt Switch", "On", "MIXAI1 Source Selector"},
	{"MIXAO Virt Switch", "On", "MIXAI2 Source Selector"},
	{"MIXBO Virt Switch", "On", "MIXBI1 Source Selector"},
	{"MIXBO Virt Switch", "On", "MIXBI2 Source Selector"},
	{"MIXCO Virt Switch", "On", "MIXCI1 Source Selector"},
	{"MIXCO Virt Switch", "On", "MIXCI2 Source Selector"},
	{"MIXDO Virt Switch", "On", "MIXDI1 Source Selector"},
	{"MIXDO Virt Switch", "On", "MIXDI2 Source Selector"},

	/* DSP path */
	{"DSPO1 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO1 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO1 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO1 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO1 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO2 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO2 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO2 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO2 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO2 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO3 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO3 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO3 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO3 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO3 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO4 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO4 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO4 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO4 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO4 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	{"DSPO5 Mixer", "DSPI1_Switch", "DSPI1 Source Selector"},
	{"DSPO5 Mixer", "DSPI2_Switch", "DSPI2 Source Selector"},
	{"DSPO5 Mixer", "DSPI3_Switch", "DSPI3 Source Selector"},
	{"DSPO5 Mixer", "DSPI4_Switch", "DSPI4 Source Selector"},
	{"DSPO5 Mixer", "DSPI5_Switch", "DSPI5 Source Selector"},

	/* Digital path */
	{"SBO1L Source Selector", "ADC1", "MIC1L Selector"},
	{"SBO1R Source Selector", "ADC1", "MIC1R Selector"},
	{"SBO1L Source Selector", "ADC2", "MIC2L Selector"},
	{"SBO1R Source Selector", "ADC2", "MIC2R Selector"},
	{"SBO1L Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO1R Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO1L Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO1R Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO1L Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SBO1R Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SBO1L Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SBO1R Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SBO1L Source Selector", "SRCAO", "SRCA"},
	{"SBO1R Source Selector", "SRCAO", "SRCA"},
	{"SBO1L Source Selector", "SRCBO", "SRCB"},
	{"SBO1R Source Selector", "SRCBO", "SRCB"},
	{"SBO1L Source Selector", "SRCCO", "SRCC"},
	{"SBO1R Source Selector", "SRCCO", "SRCC"},
	{"SBO1L Source Selector", "SRCDO", "SRCD"},
	{"SBO1R Source Selector", "SRCDO", "SRCD"},
	{"SBO1L Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO1R Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO1L Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO1R Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO1L Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO1R Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO1L Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO1R Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO1L Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO1R Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO1L Source Selector", "SBI1", "SLIM RX1"},
	{"SBO1R Source Selector", "SBI1", "SLIM RX1"},
	{"SBO1L Source Selector", "SBI1", "SLIM RX2"},
	{"SBO1R Source Selector", "SBI1", "SLIM RX2"},
	{"SBO1L Source Selector", "SBI2", "SLIM RX3"},
	{"SBO1R Source Selector", "SBI2", "SLIM RX3"},
	{"SBO1L Source Selector", "SBI2", "SLIM RX4"},
	{"SBO1R Source Selector", "SBI2", "SLIM RX4"},
	{"SBO1L Source Selector", "SBI3", "SLIM RX5"},
	{"SBO1R Source Selector", "SBI3", "SLIM RX5"},
	{"SBO1L Source Selector", "SBI3", "SLIM RX6"},
	{"SBO1R Source Selector", "SBI3", "SLIM RX6"},

	{"SBO2L Source Selector", "ADC1", "MIC1L Selector"},
	{"SBO2R Source Selector", "ADC1", "MIC1R Selector"},
	{"SBO2L Source Selector", "ADC2", "MIC2L Selector"},
	{"SBO2R Source Selector", "ADC2", "MIC2R Selector"},
	{"SBO2L Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO2R Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SBO2L Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO2R Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SBO2L Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SBO2R Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SBO2L Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SBO2R Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SBO2L Source Selector", "SRCAO", "SRCA"},
	{"SBO2R Source Selector", "SRCAO", "SRCA"},
	{"SBO2L Source Selector", "SRCBO", "SRCB"},
	{"SBO2R Source Selector", "SRCBO", "SRCB"},
	{"SBO2L Source Selector", "SRCCO", "SRCC"},
	{"SBO2R Source Selector", "SRCCO", "SRCC"},
	{"SBO2L Source Selector", "SRCDO", "SRCD"},
	{"SBO2R Source Selector", "SRCDO", "SRCD"},
	{"SBO2L Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO2R Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SBO2L Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO2R Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SBO2L Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO2R Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SBO2L Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO2R Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SBO2L Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO2R Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SBO2L Source Selector", "SBI1", "SLIM RX1"},
	{"SBO2R Source Selector", "SBI1", "SLIM RX1"},
	{"SBO2L Source Selector", "SBI1", "SLIM RX2"},
	{"SBO2R Source Selector", "SBI1", "SLIM RX2"},
	{"SBO2L Source Selector", "SBI2", "SLIM RX3"},
	{"SBO2R Source Selector", "SBI2", "SLIM RX3"},
	{"SBO2L Source Selector", "SBI2", "SLIM RX4"},
	{"SBO2R Source Selector", "SBI2", "SLIM RX4"},
	{"SBO2L Source Selector", "SBI3", "SLIM RX5"},
	{"SBO2R Source Selector", "SBI3", "SLIM RX5"},
	{"SBO2L Source Selector", "SBI3", "SLIM RX6"},
	{"SBO2R Source Selector", "SBI3", "SLIM RX6"},

	{"DAC1 Source Selector", "SBI1", "SLIM RX1"},
	{"DAC1 Source Selector", "SBI1", "SLIM RX2"},
	{"DAC1 Source Selector", "SBI2", "SLIM RX3"},
	{"DAC1 Source Selector", "SBI2", "SLIM RX4"},
	{"DAC1 Source Selector", "SBI3", "SLIM RX5"},
	{"DAC1 Source Selector", "SBI3", "SLIM RX6"},
	{"DAC1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DAC1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DAC1 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DAC1 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DAC1 Source Selector", "SRCAO", "SRCA"},
	{"DAC1 Source Selector", "SRCBO", "SRCB"},
	{"DAC1 Source Selector", "SRCCO", "SRCC"},
	{"DAC1 Source Selector", "SRCDO", "SRCD"},
	{"DAC1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DAC1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DAC1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DAC1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DAC1 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DAC1 Source Selector", "ADC1", "MIC1L Selector"},
	{"DAC1 Source Selector", "ADC1", "MIC1R Selector"},
	{"DAC1 Source Selector", "ADC2", "MIC2L Selector"},
	{"DAC1 Source Selector", "ADC2", "MIC2R Selector"},

	{"DAC2 Source Selector", "SBI1", "SLIM RX1"},
	{"DAC2 Source Selector", "SBI1", "SLIM RX2"},
	{"DAC2 Source Selector", "SBI2", "SLIM RX3"},
	{"DAC2 Source Selector", "SBI2", "SLIM RX4"},
	{"DAC2 Source Selector", "SBI3", "SLIM RX5"},
	{"DAC2 Source Selector", "SBI3", "SLIM RX6"},
	{"DAC2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DAC2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DAC2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DAC2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DAC2 Source Selector", "SRCAO", "SRCA"},
	{"DAC2 Source Selector", "SRCBO", "SRCB"},
	{"DAC2 Source Selector", "SRCCO", "SRCC"},
	{"DAC2 Source Selector", "SRCDO", "SRCD"},
	{"DAC2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DAC2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DAC2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DAC2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DAC2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"DAC2 Source Selector", "ADC1", "MIC1L Selector"},
	{"DAC2 Source Selector", "ADC1", "MIC1R Selector"},
	{"DAC2 Source Selector", "ADC2", "MIC2L Selector"},
	{"DAC2 Source Selector", "ADC2", "MIC2R Selector"},

	{"MIXAI1 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXAI1 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXAI1 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXAI1 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXAI1 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXAI1 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXAI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXAI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXAI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXAI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXAI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXAI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXAI1 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXAI1 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXAI1 Source Selector", "SRCAO", "SRCA"},
	{"MIXAI1 Source Selector", "SRCBO", "SRCB"},
	{"MIXAI1 Source Selector", "SRCCO", "SRCC"},
	{"MIXAI1 Source Selector", "SRCDO", "SRCD"},
	{"MIXAI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXAI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXAI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXAI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXAI1 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXAI2 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXAI2 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXAI2 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXAI2 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXAI2 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXAI2 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXAI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXAI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXAI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXAI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXAI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXAI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXAI2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXAI2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXAI2 Source Selector", "SRCAO", "SRCA"},
	{"MIXAI2 Source Selector", "SRCBO", "SRCB"},
	{"MIXAI2 Source Selector", "SRCCO", "SRCC"},
	{"MIXAI2 Source Selector", "SRCDO", "SRCD"},
	{"MIXAI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXAI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXAI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXAI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXAI2 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXBI1 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXBI1 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXBI1 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXBI1 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXBI1 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXBI1 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXBI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXBI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXBI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXBI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXBI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXBI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXBI1 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXBI1 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXBI1 Source Selector", "SRCAO", "SRCA"},
	{"MIXBI1 Source Selector", "SRCBO", "SRCB"},
	{"MIXBI1 Source Selector", "SRCCO", "SRCC"},
	{"MIXBI1 Source Selector", "SRCDO", "SRCD"},
	{"MIXBI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXBI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXBI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXBI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXBI1 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXBI2 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXBI2 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXBI2 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXBI2 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXBI2 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXBI2 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXBI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXBI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXBI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXBI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXBI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXBI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXBI2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXBI2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXBI2 Source Selector", "SRCAO", "SRCA"},
	{"MIXBI2 Source Selector", "SRCBO", "SRCB"},
	{"MIXBI2 Source Selector", "SRCCO", "SRCC"},
	{"MIXBI2 Source Selector", "SRCDO", "SRCD"},
	{"MIXBI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXBI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXBI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXBI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXBI2 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXCI1 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXCI1 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXCI1 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXCI1 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXCI1 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXCI1 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXCI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXCI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXCI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXCI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXCI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXCI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXCI1 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXCI1 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXCI1 Source Selector", "SRCAO", "SRCA"},
	{"MIXCI1 Source Selector", "SRCBO", "SRCB"},
	{"MIXCI1 Source Selector", "SRCCO", "SRCC"},
	{"MIXCI1 Source Selector", "SRCDO", "SRCD"},
	{"MIXCI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXCI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXCI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXCI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXCI1 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXCI2 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXCI2 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXCI2 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXCI2 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXCI2 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXCI2 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXCI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXCI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXCI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXCI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXCI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXCI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXCI2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXCI2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXCI2 Source Selector", "SRCAO", "SRCA"},
	{"MIXCI2 Source Selector", "SRCBO", "SRCB"},
	{"MIXCI2 Source Selector", "SRCCO", "SRCC"},
	{"MIXCI2 Source Selector", "SRCDO", "SRCD"},
	{"MIXCI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXCI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXCI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXCI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXCI2 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXDI1 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXDI1 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXDI1 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXDI1 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXDI1 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXDI1 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXDI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXDI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXDI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXDI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXDI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXDI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXDI1 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXDI1 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXDI1 Source Selector", "SRCAO", "SRCA"},
	{"MIXDI1 Source Selector", "SRCBO", "SRCB"},
	{"MIXDI1 Source Selector", "SRCCO", "SRCC"},
	{"MIXDI1 Source Selector", "SRCDO", "SRCD"},
	{"MIXDI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXDI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXDI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXDI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXDI1 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"MIXDI2 Source Selector", "SBI1", "SLIM RX1"},
	{"MIXDI2 Source Selector", "SBI1", "SLIM RX2"},
	{"MIXDI2 Source Selector", "SBI2", "SLIM RX3"},
	{"MIXDI2 Source Selector", "SBI2", "SLIM RX4"},
	{"MIXDI2 Source Selector", "SBI3", "SLIM RX5"},
	{"MIXDI2 Source Selector", "SBI3", "SLIM RX6"},
	{"MIXDI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"MIXDI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"MIXDI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"MIXDI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"MIXDI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"MIXDI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"MIXDI2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"MIXDI2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"MIXDI2 Source Selector", "SRCAO", "SRCA"},
	{"MIXDI2 Source Selector", "SRCBO", "SRCB"},
	{"MIXDI2 Source Selector", "SRCCO", "SRCC"},
	{"MIXDI2 Source Selector", "SRCDO", "SRCD"},
	{"MIXDI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"MIXDI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"MIXDI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"MIXDI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"MIXDI2 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"SRCAI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCAI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCAI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCAI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCAI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCAI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCAI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCAI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCAI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCAI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCAI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCAI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCAI Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SRCAI Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SRCAI Source Selector", "SRCAO", "SRCA"},
	{"SRCAI Source Selector", "SRCBO", "SRCB"},
	{"SRCAI Source Selector", "SRCCO", "SRCC"},
	{"SRCAI Source Selector", "SRCDO", "SRCD"},
	{"SRCAI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCAI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCAI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCAI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCAI Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"SRCBI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCBI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCBI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCBI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCBI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCBI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCBI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCBI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCBI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCBI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCBI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCBI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCBI Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SRCBI Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SRCBI Source Selector", "SRCAO", "SRCA"},
	{"SRCBI Source Selector", "SRCBO", "SRCB"},
	{"SRCBI Source Selector", "SRCCO", "SRCC"},
	{"SRCBI Source Selector", "SRCDO", "SRCD"},
	{"SRCBI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCBI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCBI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCBI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCBI Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"SRCCI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCCI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCCI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCCI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCCI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCCI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCCI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCCI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCCI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCCI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCCI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCCI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCCI Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SRCCI Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SRCCI Source Selector", "SRCAO", "SRCA"},
	{"SRCCI Source Selector", "SRCBO", "SRCB"},
	{"SRCCI Source Selector", "SRCCO", "SRCC"},
	{"SRCCI Source Selector", "SRCDO", "SRCD"},
	{"SRCCI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCCI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCCI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCCI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCCI Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"SRCDI Source Selector", "SBI1", "SLIM RX1"},
	{"SRCDI Source Selector", "SBI1", "SLIM RX2"},
	{"SRCDI Source Selector", "SBI2", "SLIM RX3"},
	{"SRCDI Source Selector", "SBI2", "SLIM RX4"},
	{"SRCDI Source Selector", "SBI3", "SLIM RX5"},
	{"SRCDI Source Selector", "SBI3", "SLIM RX6"},
	{"SRCDI Source Selector", "ADC1", "MIC1L Selector"},
	{"SRCDI Source Selector", "ADC1", "MIC1R Selector"},
	{"SRCDI Source Selector", "ADC2", "MIC2L Selector"},
	{"SRCDI Source Selector", "ADC2", "MIC2R Selector"},
	{"SRCDI Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SRCDI Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SRCDI Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SRCDI Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SRCDI Source Selector", "SRCAO", "SRCA"},
	{"SRCDI Source Selector", "SRCBO", "SRCB"},
	{"SRCDI Source Selector", "SRCCO", "SRCC"},
	{"SRCDI Source Selector", "SRCDO", "SRCD"},
	{"SRCDI Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SRCDI Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SRCDI Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SRCDI Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SRCDI Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"DSPI1 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI1 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI1 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI1 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI1 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI1 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI1 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI1 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI1 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI1 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI1 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI1 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI1 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DSPI1 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DSPI1 Source Selector", "SRCAO", "SRCA"},
	{"DSPI1 Source Selector", "SRCBO", "SRCB"},
	{"DSPI1 Source Selector", "SRCCO", "SRCC"},
	{"DSPI1 Source Selector", "SRCDO", "SRCD"},
	{"DSPI1 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI1 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI1 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI1 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI1 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"DSPI2 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI2 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI2 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI2 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI2 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI2 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI2 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI2 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI2 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI2 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DSPI2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DSPI2 Source Selector", "SRCAO", "SRCA"},
	{"DSPI2 Source Selector", "SRCBO", "SRCB"},
	{"DSPI2 Source Selector", "SRCCO", "SRCC"},
	{"DSPI2 Source Selector", "SRCDO", "SRCD"},
	{"DSPI2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI2 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"DSPI3 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI3 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI3 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI3 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI3 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI3 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI3 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI3 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI3 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI3 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI3 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI3 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI3 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DSPI3 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DSPI3 Source Selector", "SRCAO", "SRCA"},
	{"DSPI3 Source Selector", "SRCBO", "SRCB"},
	{"DSPI3 Source Selector", "SRCCO", "SRCC"},
	{"DSPI3 Source Selector", "SRCDO", "SRCD"},
	{"DSPI3 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI3 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI3 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI3 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI3 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"DSPI4 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI4 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI4 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI4 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI4 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI4 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI4 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI4 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI4 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI4 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI4 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI4 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI4 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DSPI4 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DSPI4 Source Selector", "SRCAO", "SRCA"},
	{"DSPI4 Source Selector", "SRCBO", "SRCB"},
	{"DSPI4 Source Selector", "SRCCO", "SRCC"},
	{"DSPI4 Source Selector", "SRCDO", "SRCD"},
	{"DSPI4 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI4 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI4 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI4 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI4 Source Selector", "DSPO5", "DSPO5 Mixer"},

	{"DSPI5 Source Selector", "SBI1", "SLIM RX1"},
	{"DSPI5 Source Selector", "SBI1", "SLIM RX2"},
	{"DSPI5 Source Selector", "SBI2", "SLIM RX3"},
	{"DSPI5 Source Selector", "SBI2", "SLIM RX4"},
	{"DSPI5 Source Selector", "SBI3", "SLIM RX5"},
	{"DSPI5 Source Selector", "SBI3", "SLIM RX6"},
	{"DSPI5 Source Selector", "ADC1", "MIC1L Selector"},
	{"DSPI5 Source Selector", "ADC1", "MIC1R Selector"},
	{"DSPI5 Source Selector", "ADC2", "MIC2L Selector"},
	{"DSPI5 Source Selector", "ADC2", "MIC2R Selector"},
	{"DSPI5 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"DSPI5 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"DSPI5 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"DSPI5 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"DSPI5 Source Selector", "SRCAO", "SRCA"},
	{"DSPI5 Source Selector", "SRCBO", "SRCB"},
	{"DSPI5 Source Selector", "SRCCO", "SRCC"},
	{"DSPI5 Source Selector", "SRCDO", "SRCD"},
	{"DSPI5 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"DSPI5 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"DSPI5 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"DSPI5 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"DSPI5 Source Selector", "DSPO5", "DSPO5 Mixer"},
};

static const struct soc_enum sdto1a_source_select_enum =
	SOC_ENUM_SINGLE(SDTO1A_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sdto1b_source_select_enum =
	SOC_ENUM_SINGLE(SDTO1B_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sdto2_source_select_enum =
	SOC_ENUM_SINGLE(SDTO2_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sdto3_source_select_enum =
	SOC_ENUM_SINGLE(SDTO3_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct soc_enum sdto4_source_select_enum =
	SOC_ENUM_SINGLE(SDTO4_SOURCE_SELECTOR, 0, 28, audio_sink_source_select_text);

static const struct snd_kcontrol_new sdto1a_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO1A Source Selector Mux", sdto1a_source_select_enum);

static const struct snd_kcontrol_new sdto1b_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO1B Source Selector Mux", sdto1b_source_select_enum);

static const struct snd_kcontrol_new sdto2_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO2 Source Selector Mux", sdto2_source_select_enum);

static const struct snd_kcontrol_new sdto3_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO3 Source Selector Mux", sdto3_source_select_enum);

static const struct snd_kcontrol_new sdto4_source_select_kctrl =
	SOC_DAPM_ENUM("SDTO4 Source Selector Mux", sdto4_source_select_enum);

static const struct snd_soc_dapm_widget ak4962_dapm_i2s_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF1 SDTIA", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF1 SDTIB", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF1 SDTIC", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF1 SDTID", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1 SDTOA", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1 SDTOB", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("AIF2 SDTI", "AIF2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF2 SDTO", "AIF2 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("AIF3 SDTI", "AIF3 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF3 SDTO", "AIF3 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_AIF_IN("AIF4 SDTI", "AIF4 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF4 SDTO", "AIF4 Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("SDTO1A Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto1a_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO1B Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto1b_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO2 Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto2_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO3 Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto3_source_select_kctrl),

	SND_SOC_DAPM_MUX("SDTO4 Source Selector", SND_SOC_NOPM, 0, 0,
			&sdto4_source_select_kctrl),
};

static const struct snd_soc_dapm_route audio_i2s_map[] = {
	{"AIF1 SDTIA", NULL, "PLL CLK"},
	{"AIF1 SDTIB", NULL, "PLL CLK"},
	{"AIF1 SDTIC", NULL, "PLL CLK"},
	{"AIF1 SDTID", NULL, "PLL CLK"},
	{"AIF2 SDTI", NULL, "PLL CLK"},
	{"AIF3 SDTI", NULL, "PLL CLK"},
	{"AIF4 SDTI", NULL, "PLL CLK"},
	{"AIF1 SDTOA", NULL, "PLL CLK"},
	{"AIF1 SDTOB", NULL, "PLL CLK"},
	{"AIF2 SDTO", NULL, "PLL CLK"},
	{"AIF3 SDTO", NULL, "PLL CLK"},
	{"AIF4 SDTO", NULL, "PLL CLK"},

	{"AIF1 SDTOA", NULL, "SDTO1A Source Selector"},
	{"AIF1 SDTOB", NULL, "SDTO1B Source Selector"},
	{"AIF2 SDTO", NULL, "SDTO2 Source Selector"},
	{"AIF3 SDTO", NULL, "SDTO3 Source Selector"},
	{"AIF4 SDTO", NULL, "SDTO4 Source Selector"},

	{"SDTO1A Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO1A Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO1A Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO1A Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO1A Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO1A Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO1A Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SDTO1A Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SDTO1A Source Selector", "SRCAO", "SRCA"},
	{"SDTO1A Source Selector", "SRCBO", "SRCB"},
	{"SDTO1A Source Selector", "SRCCO", "SRCC"},
	{"SDTO1A Source Selector", "SRCDO", "SRCD"},
	{"SDTO1A Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO1A Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO1A Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO1A Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO1A Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO1A Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO1A Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO1A Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO1A Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO1A Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO1A Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO1A Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO1A Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SDTO1A Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO1B Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO1B Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO1B Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO1B Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO1B Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO1B Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO1B Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SDTO1B Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SDTO1B Source Selector", "SRCAO", "SRCA"},
	{"SDTO1B Source Selector", "SRCBO", "SRCB"},
	{"SDTO1B Source Selector", "SRCCO", "SRCC"},
	{"SDTO1B Source Selector", "SRCDO", "SRCD"},
	{"SDTO1B Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO1B Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO1B Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO1B Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO1B Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO1B Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO1B Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO1B Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO1B Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO1B Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO1B Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO1B Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO1B Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SDTO1B Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO2 Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO2 Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO2 Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO2 Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO2 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO2 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO2 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SDTO2 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SDTO2 Source Selector", "SRCAO", "SRCA"},
	{"SDTO2 Source Selector", "SRCBO", "SRCB"},
	{"SDTO2 Source Selector", "SRCCO", "SRCC"},
	{"SDTO2 Source Selector", "SRCDO", "SRCD"},
	{"SDTO2 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO2 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO2 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO2 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO2 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO2 Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO2 Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO2 Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO2 Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO2 Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO2 Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SDTO2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SDTO2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SDTO2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO3 Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO3 Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO3 Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO3 Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO3 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO3 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO3 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SDTO3 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SDTO3 Source Selector", "SRCAO", "SRCA"},
	{"SDTO3 Source Selector", "SRCBO", "SRCB"},
	{"SDTO3 Source Selector", "SRCCO", "SRCC"},
	{"SDTO3 Source Selector", "SRCDO", "SRCD"},
	{"SDTO3 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO3 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO3 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO3 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO3 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO3 Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO3 Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO3 Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO3 Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO3 Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO3 Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO3 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SDTO3 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SDTO3 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO3 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SDTO4 Source Selector", "ADC1", "MIC1L Selector"},
	{"SDTO4 Source Selector", "ADC1", "MIC1R Selector"},
	{"SDTO4 Source Selector", "ADC2", "MIC2L Selector"},
	{"SDTO4 Source Selector", "ADC2", "MIC2R Selector"},
	{"SDTO4 Source Selector", "MIXAO", "MIXAO Virt Switch"},
	{"SDTO4 Source Selector", "MIXBO", "MIXBO Virt Switch"},
	{"SDTO4 Source Selector", "MIXCO", "MIXCO Virt Switch"},
	{"SDTO4 Source Selector", "MIXDO", "MIXDO Virt Switch"},
	{"SDTO4 Source Selector", "SRCAO", "SRCA"},
	{"SDTO4 Source Selector", "SRCBO", "SRCB"},
	{"SDTO4 Source Selector", "SRCCO", "SRCC"},
	{"SDTO4 Source Selector", "SRCDO", "SRCD"},
	{"SDTO4 Source Selector", "DSPO1", "DSPO1 Mixer"},
	{"SDTO4 Source Selector", "DSPO2", "DSPO2 Mixer"},
	{"SDTO4 Source Selector", "DSPO3", "DSPO3 Mixer"},
	{"SDTO4 Source Selector", "DSPO4", "DSPO4 Mixer"},
	{"SDTO4 Source Selector", "DSPO5", "DSPO5 Mixer"},
	{"SDTO4 Source Selector", "SBI1", "SLIM RX1"},
	{"SDTO4 Source Selector", "SBI1", "SLIM RX2"},
	{"SDTO4 Source Selector", "SBI2", "SLIM RX3"},
	{"SDTO4 Source Selector", "SBI2", "SLIM RX4"},
	{"SDTO4 Source Selector", "SBI3", "SLIM RX5"},
	{"SDTO4 Source Selector", "SBI3", "SLIM RX6"},
	{"SDTO4 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SDTO4 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SDTO4 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SDTO4 Source Selector", "SDTI3", "AIF3 SDTI"},

	{"DAC1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DAC1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DAC1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DAC1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DAC1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DAC1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DAC1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DAC2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DAC2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DAC2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DAC2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DAC2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DAC2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DAC2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXAI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXAI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXAI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXAI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXAI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXAI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXAI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXAI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXAI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXAI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXAI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXAI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXAI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXAI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXBI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXBI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXBI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXBI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXBI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXBI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXBI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXBI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXBI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXBI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXBI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXBI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXBI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXBI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXCI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXCI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXCI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXCI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXCI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXCI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXCI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXCI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXCI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXCI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXCI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXCI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXCI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXCI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXDI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXDI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXDI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXDI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXDI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXDI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXDI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"MIXDI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"MIXDI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"MIXDI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"MIXDI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"MIXDI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"MIXDI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"MIXDI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCAI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCAI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCAI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCAI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCAI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCAI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCAI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCBI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCBI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCBI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCBI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCBI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCBI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCBI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCCI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCCI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCCI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCCI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCCI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCCI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCCI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SRCDI Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SRCDI Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SRCDI Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SRCDI Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SRCDI Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SRCDI Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SRCDI Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI1 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI1 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI1 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI1 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI1 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI1 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI1 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI2 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI2 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI2 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI2 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI2 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI2 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI2 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI3 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI3 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI3 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI3 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI3 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI3 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI3 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI4 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI4 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI4 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI4 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI4 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI4 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI4 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"DSPI5 Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"DSPI5 Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"DSPI5 Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"DSPI5 Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"DSPI5 Source Selector", "SDTI2", "AIF2 SDTI"},
	{"DSPI5 Source Selector", "SDTI3", "AIF3 SDTI"},
	{"DSPI5 Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SBO1L Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO1R Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO1L Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO1R Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO1L Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO1R Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO1L Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO1R Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO1L Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO1R Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO1L Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO1R Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO1L Source Selector", "SDTI4", "AIF4 SDTI"},
	{"SBO1R Source Selector", "SDTI4", "AIF4 SDTI"},

	{"SBO2L Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO2R Source Selector", "SDTI1A", "AIF1 SDTIA"},
	{"SBO2L Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO2R Source Selector", "SDTI1B", "AIF1 SDTIB"},
	{"SBO2L Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO2R Source Selector", "SDTI1C", "AIF1 SDTIC"},
	{"SBO2L Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO2R Source Selector", "SDTI1D", "AIF1 SDTID"},
	{"SBO2L Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO2R Source Selector", "SDTI2", "AIF2 SDTI"},
	{"SBO2L Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO2R Source Selector", "SDTI3", "AIF3 SDTI"},
	{"SBO2L Source Selector", "SDTI4", "AIF4 SDTI"},
	{"SBO2R Source Selector", "SDTI4", "AIF4 SDTI"},
};

#ifdef CONFIG_SWITCH
static ssize_t headset_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case BIT_NO_HEADSET:
		return snprintf(buf, 10, "No Device\n");
	case BIT_HEADSET:
		return snprintf(buf, 8, "Headset\n");
	case BIT_HEADSET_NO_MIC:
		return snprintf(buf, 15, "Headset_no_mic\n");
	}
	return -EINVAL;
}
#endif
static int last_report_key;
static int last_detect_key;
static int last_report_sw;
static int mic_det_counter;
static int mic_det_repeat;
static int dual_mic_det_counter;
static int dual_mic_det_repeat;
static int dual_mic_det;
static int key_det_repeat;
static int imp_det_repeat;
static int mclk_control;
/* static int pll_control; */
/* static int cp23_control; */
static int lrmg_det;
static int lrmg_mic_det_repeat;
static int lrmg_mic_det_counter;
static int det_pm;
static int hp_check;

/* zte jjp  add headset detect /proc/hs 2015-03-30 start */
int g_hs_state = 0;
static ssize_t hs_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count = 0;

	if (*offset > 0) {
		count = 0;
	} else {
		pr_err("%s: enter %d\n", __func__, g_hs_state);
		count = snprintf(buf, 2, "%d\n", g_hs_state);
		*offset += count;
	}
	return count;
}
static const struct file_operations hs_proc_fops = {
	.owner = THIS_MODULE,
	.read = hs_read,
};
/* zte jjp add headset detect /proc/hs 2015-03-30 end */

static irqreturn_t ak4962_jde_irq(int irq, void *data)
{
	struct ak4962_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	int val;
	int report = last_report_sw;

	val = snd_soc_read(codec, JACK_DETECTION_STATUS);	/* JDS read */
	dev_err(codec->dev, "%s: val %d\n", __func__, val);
	if (val < 0) {
		snd_soc_update_bits(codec, DETECTION_EVENT, 0x01, 0x01);
		dev_err(codec->dev, "Failed to read JACK_DETECTION_STATUS: %d\n", val);
		return IRQ_HANDLED;
	}

	if (val & 0x01) {
		pr_err("%s: CLRJDE=H  last_report_sw=%x\n ", __func__, last_report_sw);
		snd_soc_update_bits(codec, DETECTION_EVENT, 0x01, 0x01);
		if (!last_report_sw) {
			pr_err("%s: last_report_sw == 0\n", __func__);
			mic_det_repeat = 0;
			mic_det_counter = 0;

			dual_mic_det_repeat = 0;
			dual_mic_det_counter = 0;

			lrmg_mic_det_repeat = 0;
			lrmg_mic_det_counter = 0;
			det_pm = 0;

			br_work = 0;
			br_extra_mode = 0;

			snd_soc_update_bits(codec, CDCMCLK_DIVIDER, 0xFF, 0x09);	/* 20151216 */
			snd_soc_update_bits(codec, CLOCK_MODE_SELECT, 0x7F, 0x0A);	/* 20151216 */

			charge_pump1_status |= 0x02;
			ak4962_change_charge_pump_1(codec, enable_cp1);
			snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x08, 0x08);	/* PMMP2A on */
			usleep_range(14000, 15000);	/* 48kHz wait time */

			/* command mode, SARSEL=L */
			snd_soc_update_bits(codec, DETECTION_SETTING_2, 0xE0, 0x60);
			/* PMSAR on */
			snd_soc_update_bits(codec, DETECTION_PM, 0x20, 0x20);

#ifdef JACK_ALWAYS_OPEN_WHEN_PLUGOUT
			snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x1F, 0x1F);
#else
			snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x1F, 0x00);
#endif
			pr_err("%s: test insert_1\n", __func__);
		}
	} else {
		pr_err("%s: test4 reject\n", __func__);
		snd_soc_update_bits(codec, DETECTION_EVENT, 0x01, 0x01);
		imp_det_repeat = 0;

		snd_soc_update_bits(codec, OUTPUT_MODE_SETTING, 0x03, 0x00);

#ifdef JACK_ALWAYS_OPEN_WHEN_PLUGOUT
		/* lhs for test */
		snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x1F, 0x00);
#else
		snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x1F, 0x1F);
#endif

		snd_soc_update_bits(codec, DETECTION_SETTING_2, 0xE0, 0x00);
		snd_soc_update_bits(codec, DETECTION_PM, 0x20, 0x00);
		snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x18, 0x00);

		snd_soc_update_bits(codec, CODEC_DEBUG_6, 0x80, 0x00);	/* test4 rev.B only PMIREF=0 */
		snd_soc_update_bits(codec, CODEC_DEBUG_1, 0x30, 0x00);	/* test4 rev.B only DG_CALISEL_IDET=0 */

		charge_pump1_status &= 0xFD;
		ak4962_change_charge_pump_1(codec, disable_cp1);

		charge_pump2_status &= 0xFD;						/* rev0.15 */
		ak4962_change_charge_pump_2(codec, disable_cp2);	/* rev0.15 */
		ldo3_status &= 0xFD;								/* rev0.15 */
		ak4962_change_ldo_3(codec, disable_ldo3);			/* rev0.15 */
		charge_pump3_status &= 0xFD;						/* rev0.15 */
		ak4962_change_charge_pump_3(codec, disable_cp3);	/* rev0.15 */

/* snd_soc_update_bits(codec, CODEC_DEBUG_1, 0x03, 0x00); */
		dual_mic_det = 0;
		lrmg_det = 0;
		priv->mono_jack = 0;
/* snd_soc_update_bits(codec, DAC1_MONO_MIXING, 0x33, 0x21); */

		hp_check = 0;		/* rev0.15 */

		priv->hp_dvol1_offset = 0;
		priv->hp_avol_offset = 0;

		if (mclk_control == 1) {
			/* priv->mbhc_cfg.mclk_cb_fn(codec, 0, false); */
			mclk_control = 0;
		}

		if (pll_status == 0) {
			snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x00);
		}

		if (last_report_sw != 0) {
			report = 0;

#ifdef CONFIG_SWITCH
			if (last_report_sw) {
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 0);
			}
			if (last_report_sw == BIT_HEADSET) {
				input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 0);
			}
			if (last_report_sw == BIT_HEADSET_DUAL_MIC) {
				input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE2_INSERT, 0);
			}
			input_sync(priv->mbhc_cfg.btn_idev);
#endif
		}
	}

	if (report != last_report_sw) {
	       pr_info("\t[AK4962] %s(%d)\n", __func__, __LINE__);
#ifdef CONFIG_SWITCH
		switch_set_state(priv->mbhc_cfg.h2w_sdev, report);
#else
		snd_soc_jack_report(priv->mbhc_cfg.headset_jack, report,
				    SND_JACK_HEADSET);
#endif
		last_report_sw = report;
		dev_info(codec->dev, "%s: report %d\n", __func__, report);
/* zte jjp add headset detect /proc/hs 2015-03-30 start */
	     g_hs_state = report;
/* zte jjp add headset detect /proc/hs 2015-03-30 end*/
	}

	return IRQ_HANDLED;
}

static irqreturn_t ak4962_sare_irq(int irq, void *data)
{
	struct ak4962_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	int val;
	int val2, val3, val4;		/* rev0.15 */
	int report = last_report_sw;

	pr_err("%s: mic_det_repeat=%d, mic_det_counter=%d\n",
			__func__, mic_det_repeat, mic_det_counter);
	pr_err("%s: dual_mic_det_repeat=%d, dual_mic_det_counter=%d\n",
			__func__, dual_mic_det_repeat, dual_mic_det_counter);
	pr_err("%s: lrmg_det=%d, lrmg_mic_det_repeat=%d, lrmg_mic_det_counter=%d\n",
			__func__, lrmg_det, lrmg_mic_det_repeat, lrmg_mic_det_counter);

	val = snd_soc_read(codec, SAR_DATA_VALUE);
	if (val < 0) {
		/* PMSAR off */
		snd_soc_update_bits(codec, DETECTION_PM, 0x20, 0x00);
		/* stop mode, SARSEL=L */
		snd_soc_update_bits(codec, DETECTION_SETTING_2, 0xE0, 0x00);
		/* CLRMSE=H */
		snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
		dev_err(codec->dev, "Failed to read SAR_DATA_VALUE: %d\n", val);
		return IRQ_HANDLED;
	}

	pr_err("%s: SARD = 0x%x\n", __func__, val);

	if (lrmg_det == 1) {
		if (val > LRMG_MIC_DETECTION_LEVEL_VALID) {
			lrmg_mic_det_repeat++;
			if (lrmg_mic_det_repeat >= LRMG_MAX_MIC_DET_RPT) {
				lrmg_det = 2;
#ifdef CONFIG_SWITCH
				/* pr_err("[LHS] %s line %d : BIT_HEADSET_UNSUPPORTED\n",__func__,__LINE__); */
				report = BIT_HEADSET_UNSUPPORTED;
				input_report_switch(priv->mbhc_cfg.btn_idev, SW_UNSUPPORT_INSERT, 1);
				input_sync(priv->mbhc_cfg.btn_idev);
#else
				report = SND_JACK_UNSUPPORTED;
#endif
				pr_info("%s: Unsupported headset (LRMG) is detected\n", __func__);
			}
		} else {
			lrmg_mic_det_repeat = 0;
			if (lrmg_mic_det_counter >= LRMG_MAX_MIC_DET_TRY) {
				lrmg_det = 2;
#ifdef CONFIG_SWITCH
				pr_err("[LHS] %s line %d : BIT_HEADSET_NO_MIC\n", __func__, __LINE__);
				report = BIT_HEADSET_NO_MIC;
				input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
				input_sync(priv->mbhc_cfg.btn_idev);
#else
				report = SND_JACK_HEADPHONE;
#endif
				pr_info("%s: headphonoe is detected\n", __func__);
			} else {
				lrmg_mic_det_counter++;
			}
		}

		if (lrmg_det == 2) {
		pr_info("%s: LRGM test, lrmg_det=%d\n", __func__, lrmg_det);
		dual_mic_det = 3;
		/* stop mode, SARSEL=L */
		snd_soc_update_bits(codec, DETECTION_SETTING_2, 0xE0, 0x00);
		/* PMSAR off */
		snd_soc_update_bits(codec, DETECTION_PM, 0x20, 0x00);
		det_pm = 1;
		/* PMMP2A off */
		snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x08, 0x00);
		charge_pump1_status &= 0xFD;
		/* test4, PMLDO1,PMCP1=L */
		ak4962_change_charge_pump_1(codec, disable_cp1);
		/* HPLHZ/HPRHZ=0 */
		snd_soc_update_bits(codec, OUTPUT_MODE_SETTING, 0x03, 0x00);
		}
	} else {	/* lrmg_det == 0 */
		if (dual_mic_det == 0) {
			if (val > MIC_DETECTION_LEVEL_VALID) {
				mic_det_repeat++;
				if (mic_det_repeat >= MAX_MIC_DET_RPT) {
#ifdef TWO_MIC_DETECTION
					dual_mic_det = 1;
					snd_soc_update_bits(codec, DETECTION_SETTING_2, 0x80, 0x80);
					snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x10, 0x10);
					usleep_range(14000, 15000);
#else

#ifdef CONFIG_SWITCH
					pr_err("[LHS] %s line %d : BIT_HEADSET\n", __func__, __LINE__);
					report = BIT_HEADSET;
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 1);
					input_sync(priv->mbhc_cfg.btn_idev);
#else
					report = SND_JACK_HEADSET;
#endif
					dual_mic_det = 2;
					pr_info("%s: 1-mic headset is detected\n", __func__);
#endif
				}
			} else {
				mic_det_repeat = 0;
				if (mic_det_counter >= MAX_MIC_DET_TRY) {
					snd_soc_update_bits(codec, OUTPUT_MODE_SETTING, 0x03, 0x03);
					lrmg_det = 1;
#if 0
#ifdef CONFIG_SWITCH
					report = BIT_HEADSET_NO_MIC;
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
					input_sync(priv->mbhc_cfg.btn_idev);
#else
					report = SND_JACK_HEADPHONE;	/* include/sound/jack.hで定義；1 */
#endif
					pr_info("%s: headphone is detected\n", __func__);

					dual_mic_det = 3;
					snd_soc_update_bits(codec, DETECTION_SETTING_2, 0xE0, 0x00);
					/* stop mode, SARSEL=L */
					snd_soc_update_bits(codec, DETECTION_PM, 0x20, 0x00);
					/* PMSAR off */
					snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x08, 0x00);
					/* PMMP2A off */
					charge_pump1_status &= 0xFD;
					ak4962_change_charge_pump_1(codec, disable_cp1);	/* PMLDO1,PMCP1=L */
#endif
				} else {
					if (mic_det_counter == (MAX_MIC_DET_TRY >> 1)) {
						if (priv->mbhc_cfg.swap_gnd_mic) {
							priv->mbhc_cfg.swap_gnd_mic(codec);
						}
					}
					mic_det_counter++;
				}
			}
		} else if (dual_mic_det == 1) {
			if ((val > MIC_DETECTION_LEVEL_VALID) && (val != 0xff)) {
				dual_mic_det_repeat++;
				if (dual_mic_det_repeat >= MAX_DUAL_MIC_DET_RPT) {
#ifdef CONFIG_SWITCH
					pr_err("[LHS] %s line %d : BIT_HEADSET_DUAL_MIC\n", __func__, __LINE__);
					report = BIT_HEADSET_DUAL_MIC;	/* need to change */
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
					/* input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 1); */
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE2_INSERT, 1);
					input_sync(priv->mbhc_cfg.btn_idev);
#else
					report = SND_JACK_ANC_HEADPHONE;
#endif
					dual_mic_det = 2;
					pr_info("%s: 2-mic headset is detected\n", __func__);

#ifdef ANC_JACK_NO_REMOTE_CONTROL	/* test4 */
					snd_soc_update_bits(codec, DETECTION_PM, 0x20, 0x00);
					/* PMSAR off */
					det_pm = 1;
#endif
				}
			} else {
				dual_mic_det_repeat = 0;
				if (dual_mic_det_counter >= MAX_DUAL_MIC_DET_TRY) {
					snd_soc_update_bits(codec, POWER_MANAGEMENT_4, 0x10, 0x00);
#ifdef CONFIG_SWITCH
					pr_err("[LHS] %s line %d : BIT_HEADSET\n", __func__, __LINE__);
					report = BIT_HEADSET;
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_HEADPHONE_INSERT, 1);
					input_report_switch(priv->mbhc_cfg.btn_idev, SW_MICROPHONE_INSERT, 1);
					input_sync(priv->mbhc_cfg.btn_idev);
#else
					report = SND_JACK_HEADSET;
#endif
					dual_mic_det = 2;
					pr_info("%s: 1-mic headset is detected\n", __func__);
				} else {
					dual_mic_det_counter++;
				}
			}
		}
	}

	/* pr_err("%s: dual_mic_det=%d\n",__func__,dual_mic_det); */

	if (dual_mic_det == 2) {
			last_detect_key = key_det_repeat = 0;
			snd_soc_update_bits(codec, MIC_DETECTION_LEVEL, 0xFF, 0x44);	/* MDLVL < 26.6% */
			snd_soc_update_bits(codec, DETECTION_SETTING_2, 0xE0, 0x00);	/* SARSEL=L, stop mode */
			snd_soc_update_bits(codec, DETECTION_SETTING_2, 0x60, 0x40);	/* interval mode */
			dual_mic_det = 3;
	}

	/* pr_err("%s: dual_mic_det=%d, det_pm=%d\n",__func__,dual_mic_det,det_pm); */

	if ((report != last_report_sw) && (dual_mic_det == 3)) {
		val2 = snd_soc_read(codec, POWER_MANAGEMENT_8);		/* rev0.15, DAC1/2 check */
		val2 &= 0x5;										/* rev0.15 */
		val3 = snd_soc_read(codec, POWER_MANAGEMENT_6);		/* rev0.15, ADC1/2/3 check */
		val3 &= 0x1F;										/* rev0.15 */
		val4 = snd_soc_read(codec, SDTO2_SOURCE_SELECTOR);	/* rev0.15, SDTO2 (Speaker) check */
		priv->hp_dvol1_offset = HP_DVOL1_OFFSET_32;			/* rev0.15 */
		priv->hp_avol_offset = HP_AVOL_OFFSET_32;			/* rev0.15 */

#ifdef CONFIG_SWITCH
		switch_set_state(priv->mbhc_cfg.h2w_sdev, report);
#else
		snd_soc_jack_report(priv->mbhc_cfg.headset_jack, report,
				    SND_JACK_HEADSET);
#endif
		last_report_sw = report;

/* zte jjp add headset detect /proc/hs 2015-03-30 start */
	     g_hs_state = report;
/* zte jjp add headset detect /proc/hs 2015-03-30 end*/
		dev_info(codec->dev, "%s: report %d\n", __func__, report);
		dual_mic_det = 0;
		lrmg_det = 0;

		d0 = get_dtime();
		if (d0 < 0) {
			d0 = 0;
		}
		pr_err("%s: ******d0=%ld\n", __func__, d0);

		pr_err("\t[AK4962] %s(%d)DAC1/2,SDTO check: val2=%d, val3=%d, val4=%d\n",
				__func__, __LINE__, val2, val3, val4);	/*rev0.15*/

		if ((val2 == 0) && (val3 == 0) && (val4 == 0)) {
			/* rev0.15 DAC1/2, ADC1/2/3, SDTO2 off -> imp det */
			pr_err("%s: mclk_enable on\n", __func__);
			/* priv->mbhc_cfg.mclk_cb_fn(codec, 1, false); */
			mclk_control = 1;

			pr_err("%s: pll_status=%d\n", __func__, pll_status);
			if (pll_status == 0) {
				snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x01);
				usleep_range(2000, 2100);
			}

			snd_soc_update_bits(codec, POWER_MANAGEMENT_7, 0x10, 0x00);	/* LPMODE=0	//rev0.16 */

			charge_pump3_status |= 0x02;						/* rev0.15 */
			ak4962_change_charge_pump_3(codec, enable_cp3);		/* rev0.15 */
			snd_soc_update_bits(codec, CODEC_DEBUG_6, 0x80, 0x80);	/* rev.B */
			usleep_range(6500, 6600);
			ldo3_status |= 0x02;								/* rev0.15 */
			ak4962_change_ldo_3(codec, enable_ldo3);			/* rev0.15 */
			usleep_range(1000, 1100);
			charge_pump2_status |= 0x02;						/* rev0.15 */
			ak4962_change_charge_pump_2(codec, enable_cp2);		/* rev0.15 */
			usleep_range(4500, 4600);

			snd_soc_update_bits(codec, CODEC_DEBUG_1, 0x30, 0x10);
			usleep_range(24000, 25000);
			snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x80, 0x80);
	}
	}
#if 0
#ifdef ANC_JACK_NO_REMOTE_CONTROL
 #ifdef CONFIG_SWITCH
	if ((report != BIT_HEADSET_NO_MIC) && (report != BIT_HEADSET_DUAL_MIC)) {
	snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
	}
 #else
	if ((report != SND_JACK_HEADPHONE) && (report != SND_JACK_ANC_HEADPHONE)) {
	snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
	}
 #endif
#else
#ifdef CONFIG_SWITCH
	if (report != BIT_HEADSET_NO_MIC) {
	snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
	}
#else
	if (report != SND_JACK_HEADPHONE) {
	snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
	}
#endif
#endif
#endif
	/* snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04); */

	if (det_pm == 0) {
		snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ak4962_ide_irq(int irq, void *data)
{
	struct ak4962_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	int lval, rval;
	int val1;
	int err_det;

	lval = snd_soc_read(codec, LCH_IMPEDANCE_VALUE);
	rval = snd_soc_read(codec, RCH_IMPEDANCE_VALUE);
	err_det = snd_soc_read(codec, JACK_DETECTION_STATUS);
/* cap = snd_soc_read(codec, POWER_MANAGEMENT_3); */

	pr_err("%s: ide_irq, lval=0x%x, rval=0x%x, err_det=0x%x\n", __func__, lval, rval, err_det);
	pr_err("%s: imp_det_repeat=%d\n", __func__, imp_det_repeat);

	snd_soc_update_bits(codec, DETECTION_EVENT, 0x02, 0x02);
	err_det &= 0x0a;

	if ((lval < 0) || (rval < 0)) {
		dev_err(codec->dev, "Failed to read IMPEDANCE_VALUE: lval=0x%x, rval=0x%x\n", lval, rval);
		ldo3_status &= 0xFD;								/* rev0.15 */
		ak4962_change_ldo_3(codec, disable_ldo3);			/* rev0.15 */
		charge_pump3_status &= 0xFD;						/* rev0.15 */
		ak4962_change_charge_pump_3(codec, disable_cp3);	/* rev0.15 */
		charge_pump2_status &= 0xFD;						/* rev0.15 */
		ak4962_change_charge_pump_2(codec, disable_cp2);	/* rev0.15 */
		return IRQ_HANDLED;
	}

	if (imp_det_repeat == 0) {	/* //1st time imp detect */
		if (err_det > 0) {
			dev_err(codec->dev, "Failed to read IMPEDANCE_VALUE (analog error), err_det=%d\n", err_det);
			val1 = snd_soc_read(codec, JACK_DETECTION_STATUS);	/* JDS read */
			if (val1 & 0x01) {
				snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x80, 0x80);
				imp_det_repeat = 1;
			}
			return IRQ_HANDLED;
		} else if ((lval >= IMP_SHORT) && (lval < IMP_16OHM) &&
				(rval >= IMP_SHORT) && (rval < IMP_16OHM)) {
			/*16ohm*/
			pr_info("\t[AK4962] %s(%d) IMP=16ohm\n", __func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_16;
			priv->hp_avol_offset = HP_AVOL_OFFSET_16;
		} else if ((lval >= IMP_16OHM) && (lval < IMP_32OHM) &&
				(rval >= IMP_16OHM) && (rval < IMP_32OHM)) {
			/*32ohm*/
			pr_info("\t[AK4962] %s(%d) IMP=32ohm\n", __func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_32;
			priv->hp_avol_offset = HP_AVOL_OFFSET_32;
		} else if ((lval >= IMP_32OHM) && (lval < IMP_OVER_45OHM) &&
				(rval >= IMP_32OHM) && (rval < IMP_OVER_45OHM)) {
			/*45~114ohm*/
			pr_info("\t[AK4962] %s(%d) IMP=45~114ohm\n", __func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_45_114;
			priv->hp_avol_offset = HP_AVOL_OFFSET_45_114;
		} else {	/*go to 2nd time detection*/
			pr_info("\t[AK4962] %s(%d) lval&rval: not same value\n",
					__func__, __LINE__);
			val1 = snd_soc_read(codec, JACK_DETECTION_STATUS);	/*JDS read*/
			if (val1 & 0x01) {
				snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x80, 0x80);
				imp_det_repeat = 1;
			}
			return IRQ_HANDLED;
		}
	} else {
		/*imp_det_repeat == 1*/
		pr_info("\t[AK4962] %s(%d) 2nd Impedance Detection\n", __func__, __LINE__);

		if (err_det > 0) {
			pr_info("\t[AK4962] %s(%d) IMP=default setting(32ohm), err_det=%d\n",
					__func__, __LINE__, err_det);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_32;
			priv->hp_avol_offset = HP_AVOL_OFFSET_32;
		} else if (rval < 3) {		/*mono*/
			priv->mono_jack = 1;
			snd_soc_update_bits(codec, DAC1_MONO_MIXING, 0x77, 0x07);
			/*DAC1 Rch:mute,Lch:L+R/2*/
			dev_info(codec->dev, "%s: mono_jack %d\n", __func__, priv->mono_jack);

			if (lval < IMP_SHORT) {
				/*(mono)9ohm*/
				pr_info("\t[AK4962] %s(%d) (mono)IMP=9ohm\n",
						__func__, __LINE__);
				priv->hp_dvol1_offset = HP_DVOL1_OFFSET_9;
				priv->hp_avol_offset = HP_AVOL_OFFSET_9;
			} else if (lval < IMP_16OHM) {
				/*(mono)16ohm*/
				pr_info("\t[AK4962] %s(%d) (mono)IMP=16ohm\n",
						__func__, __LINE__);
				priv->hp_dvol1_offset = HP_DVOL1_OFFSET_16;
				priv->hp_avol_offset = HP_AVOL_OFFSET_16;
			} else if (lval < IMP_32OHM) {
				/*(mono)32ohm*/
				pr_info("\t[AK4962] %s(%d) (mono)IMP=32ohm\n",
						__func__, __LINE__);
				priv->hp_dvol1_offset = HP_DVOL1_OFFSET_32;
				priv->hp_avol_offset = HP_AVOL_OFFSET_32;
			} else if (lval < IMP_OVER_45OHM) {
				/*(mono)45~114ohm*/
				pr_info("\t[AK4962] %s(%d) (mono)IMP=45~114ohm\n",
						__func__, __LINE__);
				priv->hp_dvol1_offset = HP_DVOL1_OFFSET_45_114;
				priv->hp_avol_offset = HP_AVOL_OFFSET_45_114;
			} else if (lval >= IMP_OVER_45OHM) {
				/*(mono)Hi-impedance*/
				pr_info("\t[AK4962] %s(%d) (mono)IMP=Hi-impedance(over 114ohm)\n",
						__func__, __LINE__);
				priv->hp_dvol1_offset = HP_DVOL1_OFFSET_H;
				priv->hp_avol_offset = HP_AVOL_OFFSET_H;
			}
		} else if ((lval < IMP_SHORT) && (rval < IMP_SHORT)) {
			/*9ohm stereo*/
			pr_info("\t[AK4962] %s(%d) IMP=9ohm\n",
					__func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_9;
			priv->hp_avol_offset = HP_AVOL_OFFSET_9;
			ak4962_hp_impedance_value = 1;
		} else if ((lval >= IMP_SHORT) && (lval < IMP_16OHM) &&
				(rval >= IMP_SHORT) && (rval < IMP_16OHM)) {
			/*16ohm*/
			pr_info("\t[AK4962] %s(%d) IMP=16ohm\n",
					__func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_16;
			priv->hp_avol_offset = HP_AVOL_OFFSET_16;
			ak4962_hp_impedance_value = 1;
		} else if ((lval >= IMP_16OHM) && (lval < IMP_32OHM) &&
				(rval >= IMP_16OHM) && (rval < IMP_32OHM)) {
			/*32ohm*/
			pr_info("\t[AK4962] %s(%d) IMP=32ohm\n",
					__func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_32;
			priv->hp_avol_offset = HP_AVOL_OFFSET_32;
			ak4962_hp_impedance_value = 1;
		} else if ((lval >= IMP_32OHM) && (lval < IMP_OVER_45OHM) &&
				(rval >= IMP_32OHM) && (rval < IMP_OVER_45OHM)) {
			/*45~114ohm*/
			pr_info("\t[AK4962] %s(%d) IMP=45~114ohm\n",
					__func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_45_114;
			priv->hp_avol_offset = HP_AVOL_OFFSET_45_114;
			ak4962_hp_impedance_value = 2;
		} else if ((lval >= IMP_OVER_45OHM) && (rval >= IMP_OVER_45OHM)) {
			/*Hi-impedance*/
			pr_info("\t[AK4962] %s(%d) IMP=Hi-impedance(over 114ohm)\n",
					__func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_H;
			priv->hp_avol_offset = HP_AVOL_OFFSET_H;
			ak4962_hp_impedance_value = 3;
		} else {
			/*set to default(32ohm)*/
			pr_info("\t[AK4962] %s(%d) IMP=default setting(32ohm)\n",
					__func__, __LINE__);
			priv->hp_dvol1_offset = HP_DVOL1_OFFSET_32;
			priv->hp_avol_offset = HP_AVOL_OFFSET_32;
			ak4962_hp_impedance_value = 1;
		}
	}

	snd_soc_update_bits(codec, POWER_MANAGEMENT_7,
				0x10, priv->low_power_mode << 4);	/*rev0.16*/

	hp_check = snd_soc_read(codec, POWER_MANAGEMENT_8);
	/*rev0.15; DAC1 check*/
	hp_check &= 0x1;
	pr_err("%s: ****** hp_check=%d\n", __func__, hp_check);
	if (hp_check != 0x1) {
		charge_pump2_status &= 0xFD;						/* rev0.15 */
		ak4962_change_charge_pump_2(codec, disable_cp2);	/* rev0.15 */
		ldo3_status &= 0xFD;								/* rev0.15 */
		ak4962_change_ldo_3(codec, disable_ldo3);			/* rev0.15 */
		charge_pump3_status &= 0xFD;						/* rev0.15 */
		ak4962_change_charge_pump_3(codec, disable_cp3);	/* rev0.15 */
	}

	if (pll_status == 0) {
		snd_soc_update_bits(codec, POWER_MANAGEMENT_1, 0x01, 0x00);
	}

	if (mclk_control == 1) {
		/* priv->mbhc_cfg.mclk_cb_fn(codec, 0, false); */
		mclk_control = 0;
	}

	snd_soc_update_bits(codec, CODEC_DEBUG_6, 0x80, 0x00);	/* rev.B only PMIREF=0 */
	snd_soc_update_bits(codec, CODEC_DEBUG_1, 0x30, 0x00);	/* rev.B only DG_CALISEL_IDET=0 */

	return IRQ_HANDLED;
}

static irqreturn_t ak4962_mice_irq(int irq, void *data)
{
	struct ak4962_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	int mic_level;
	int report = last_report_key;
	int test_0x3, test_0x5, test_0x6;
	int val;

	test_0x3 = snd_soc_read(codec, POWER_MANAGEMENT_1);
	test_0x5 = snd_soc_read(codec, POWER_MANAGEMENT_3);
	test_0x6 = snd_soc_read(codec, POWER_MANAGEMENT_4);
	pr_err("%s: AKM test: rev0.16: 0x3=0x%x, 0x5=0x%x, 0x6=0x%x\n",
			__func__, test_0x3, test_0x5, test_0x6);

	d1 = get_dtime();
	/* pr_err("%s: ****** erapsed=%ld[ms]\n",__func__,d1-d0); */

	if (d1-d0 < 2000) {     /* four section earphone remote key protect time */
		snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);
		return IRQ_HANDLED;
	}

	pr_err("%s: Entry\n", __func__);
	if (last_report_sw != 0) {
		mic_level = snd_soc_read(codec, SAR_DATA_VALUE);	/* SARD read */
		if (mic_level < 0) {
			snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);	/* CLRMSE=H */
			dev_err(codec->dev, "Failed to read SAR_DATA_VALUE: %d\n", mic_level);
			return IRQ_HANDLED;
		}

		pr_err("%s: mic_level=%d\n", __func__, mic_level);

		if (mic_level < KEY_MEDIA_THRESHOLD) {
			if (last_detect_key == KEY_MEDIA) {
				key_det_repeat++;
			} else {
				last_detect_key = KEY_MEDIA;
				key_det_repeat = 0;
			}

			if (key_det_repeat > MAX_KEY_DET_RPT) {
#ifdef CONFIG_SWITCH
				report = KEY_MEDIA;
#else
				report = SND_JACK_BTN_0;
#endif
				key_det_repeat = 0;
			}
		} else if (mic_level < KEY_VOICECOMMAND_THRESHOLD) {
			if (last_detect_key == KEY_VOICECOMMAND) {
				key_det_repeat++;
			} else {
				last_detect_key = KEY_VOICECOMMAND;
				key_det_repeat = 0;
			}

			if (key_det_repeat > MAX_KEY_DET_RPT) {
#ifdef CONFIG_SWITCH
				report = KEY_VOICECOMMAND;
#else
				report = SND_JACK_BTN_3;
#endif
				key_det_repeat = 0;
			}
		} else if (mic_level < KEY_VOLUMEUP_THRESHOLD) {
			if (last_detect_key == KEY_VOLUMEUP) {
				key_det_repeat++;
			} else {
				last_detect_key = KEY_VOLUMEUP;
				key_det_repeat = 0;
			}

			if (key_det_repeat > MAX_KEY_DET_RPT) {
#ifdef CONFIG_SWITCH
				report = KEY_VOLUMEUP;
#else
				report = SND_JACK_BTN_1;
#endif
				key_det_repeat = 0;
			}
		} else if (mic_level < KEY_VOLUMEDOWN_THRESHOLD) {
			if (last_detect_key == KEY_VOLUMEDOWN) {
				key_det_repeat++;
			} else {
				last_detect_key = KEY_VOLUMEDOWN;
				key_det_repeat = 0;
			}

			if (key_det_repeat > MAX_KEY_DET_RPT) {
#ifdef CONFIG_SWITCH
				report = KEY_VOLUMEDOWN;
#else
				report = SND_JACK_BTN_2;
#endif
				key_det_repeat = 0;
			}
		} else {
			if (last_detect_key == 0) {
				key_det_repeat++;
			} else {
				last_detect_key = 0;
				key_det_repeat = 0;
			}

			if (key_det_repeat > MAX_KEY_DET_RPT) {
				report = 0;
				key_det_repeat = 0;
			}
		}
	} else {
		report = 0;
	}

	pr_err("[LHS] %s line %d : last_detect = %d, key_det_repeat = %d\n",
			__func__, __LINE__, last_detect_key, key_det_repeat);
#ifdef CONFIG_SWITCH
	if (report != 0 && last_report_key == 0) {
		last_report_key = report;

		if (report == KEY_MEDIA) {
			kfifo_in(&priv->fifo_br, "1", 1);	/* ASCII: 1=49 */
		} else if (report == KEY_VOLUMEUP) {
			kfifo_in(&priv->fifo_br, "2", 1);
		} else if (report == KEY_VOLUMEDOWN) {
			kfifo_in(&priv->fifo_br, "3", 1);
		} else if (report == KEY_VOICECOMMAND) {
			kfifo_in(&priv->fifo_br, "4", 1);
		} else {
			kfifo_in(&priv->fifo_br, "0", 1);
			dev_err(codec->dev, "Failed to put kfifo: %d\n", mic_level);
			return IRQ_HANDLED;
		}

		d2_1 = get_dtime();
		pr_info("\t*******[AK4962] %s(%d)(push_key time)d2_1=%ld\n",
				__func__, __LINE__, d2_1);
		pr_info("\t*******[AK4962] %s(%d)push: fifo_len: %u\n", __func__,
				__LINE__, kfifo_len(&priv->fifo_br));
		pr_info("\t*******[AK4962] %s(%d)test d4=%ld\n",
				__func__, __LINE__, d4);

		if ((d2_1 - d4) > 300) {
			br_extra_mode = 0; /* lhs */
			pr_info("\t[AK4962] %s(%d) br_work=%d br_extra_mod = %d\n",
					__func__, __LINE__, br_work, br_extra_mode);
	}

		if ((br_work == 0) && (br_extra_mode == 0)) {
			pr_err("[LHS] %s line %d : queue_delayed_work!\n", __func__, __LINE__);
			queue_delayed_work(priv->workqueue_br, &priv->work_br, msecs_to_jiffies(300));
			br_work = 1;
		}

		if (br_extra_mode) {
			pr_info("\t*******[AK4962] %s(%d)EXTRA MODE\n", __func__, __LINE__);
			val = snd_soc_read(codec, JACK_DETECTION_STATUS);	/* JDS read */

			if (val & 0x01) {
				if (report == KEY_VOLUMEUP) {
					input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 1);
				}
				if (report == KEY_VOLUMEDOWN) {
					input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 0);
				}
				pr_err("[LHS] %s line %d : <<<< report =%d press!\n",
						__func__, __LINE__, report);
				input_report_key(priv->mbhc_cfg.btn_idev, report, 1);
				input_sync(priv->mbhc_cfg.btn_idev);
				dev_info(codec->dev, "%s: report %d\n", __func__, report);
								kfifo_reset_out(&priv->fifo_br);
								cancel_delayed_work(&priv->work_br);

				d4 = get_dtime();
				pr_info("\t--[K4962] %s(%d) d4 update at EXTRA_MODE: d4=%ld\n",
						__func__, __LINE__, d4);
			}
		}
		snd_soc_update_bits(codec, MIC_DETECTION_LEVEL, 0xFF, 0xFF);
	} else if (report == 0 && last_report_key != 0) {
		if (last_report_key == KEY_MEDIA) {
			kfifo_in(&priv->fifo_br, "5", 1);
		} else if (last_report_key == KEY_VOLUMEUP) {
			kfifo_in(&priv->fifo_br, "6", 1);
		} else if (last_report_key == KEY_VOLUMEDOWN) {
			kfifo_in(&priv->fifo_br, "7", 1);
		} else if (last_report_key == KEY_VOICECOMMAND) {
			kfifo_in(&priv->fifo_br, "8", 1);
		} else {
			kfifo_in(&priv->fifo_br, "0", 1);
			dev_err(codec->dev, "Failed to put kfifo: %d\n", mic_level);
			return IRQ_HANDLED;
		}

		d3_1 = get_dtime();
		pr_info("\t*******[AK4962] %s(%d)(release_bt)d3_1=%ld\n",
				__func__, __LINE__, d3_1);
		pr_info("\t*******[AK4962] %s(%d)release: fifo_len: %u\n",
				__func__, __LINE__, kfifo_len(&priv->fifo_br));

		if ((br_work == 0) || br_extra_mode) {
			val = snd_soc_read(codec, JACK_DETECTION_STATUS);	/* JDS read */

			if (val & 0x01) {
				if (last_report_key == KEY_VOLUMEUP) {
					input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 1);
				}
				if (last_report_key == KEY_VOLUMEDOWN) {
					input_event(priv->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN, 0);
				}
				pr_err("[LHS] %s line %d : >>>> report =%d release!\n",
						__func__, __LINE__, last_report_key);
				input_report_key(priv->mbhc_cfg.btn_idev, last_report_key, 0);
				input_sync(priv->mbhc_cfg.btn_idev);
				dev_info(codec->dev, "%s: report %d\n", __func__, report);
				kfifo_reset_out(&priv->fifo_br);
			}
		}
		last_report_key = report;

		snd_soc_update_bits(codec, MIC_DETECTION_LEVEL, 0xFF, 0x44);	/* MDLVL < 26.6% */
	}
#else
	snd_soc_jack_report(priv->mbhc_cfg.button_jack, report, AK4962_JACK_BUTTON_MASK);
	dev_dbg(codec->dev, "%s: report %d\n", __func__, report);
#endif

	snd_soc_update_bits(codec, DETECTION_EVENT, 0x04, 0x04);

	return IRQ_HANDLED;
}

int ak4962_hs_detect(struct snd_soc_codec *codec,
		    const struct ak4962_mbhc_config *cfg)
{
	struct ak4962_priv *ak4962;
	struct ak49xx *ak49xx;
	struct ak49xx_core_resource *core_res;
	int rc = 0;

	if (!codec) {
		pr_err("Error: no codec\n");
		return -EINVAL;
	}
	ak49xx = codec->control_data;
	core_res = &ak49xx->core_res;
	pr_err("[lhs] %s in cfg->mclk_rate=%d\n", __func__, cfg->mclk_rate);

	switch (cfg->mclk_rate) {
	case AK4962_MCLK_RATE_12288KHZ:
		snd_soc_update_bits(codec, PLL1_SOURCE_SELECTOR, 0x1F, 0x04);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_L8, 0xFF, 0x03);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_L8, 0xFF, 0x27);
		break;
	case AK4962_MCLK_RATE_9600KHZ:
		snd_soc_update_bits(codec, PLL1_SOURCE_SELECTOR, 0x1F, 0x04);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_REF_DIVISOR_L8, 0xFF, 0x04);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_H8, 0xFF, 0x00);
		snd_soc_update_bits(codec, PLL1_FB_DIVISOR_L8, 0xFF, 0x3F);
		break;
	default:
		pr_err("Error: unsupported clock rate %d\n", cfg->mclk_rate);
		return -EINVAL;
	}
	pr_info("%s: MCLK: clock rate using %dHz\n", __func__, cfg->mclk_rate);

	ak4962 = snd_soc_codec_get_drvdata(codec);
	ak4962->mbhc_cfg = *cfg;

#ifdef CONFIG_SWITCH
	ak4962->mbhc_cfg.h2w_sdev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	ak4962->mbhc_cfg.h2w_sdev->name = "h2w";
	ak4962->mbhc_cfg.h2w_sdev->print_name = headset_print_name;
	rc = switch_dev_register(ak4962->mbhc_cfg.h2w_sdev);
	if (rc < 0) {
		pr_err("%d: Error in switch_dev_register\n", rc);
		goto error_switch_register;
	}

	ak4962->mbhc_cfg.btn_idev = input_allocate_device();
	memcpy(ak4962->mbhc_cfg.keycode, ak4962_keycode,
			sizeof(ak4962->mbhc_cfg.keycode));

	ak4962->mbhc_cfg.btn_idev->name = "hs_detect";
	ak4962->mbhc_cfg.btn_idev->id.vendor = 0x0001;
	ak4962->mbhc_cfg.btn_idev->id.product = 1;
	ak4962->mbhc_cfg.btn_idev->id.version = 1;
	ak4962->mbhc_cfg.btn_idev->keycode = ak4962->mbhc_cfg.keycode;
	ak4962->mbhc_cfg.btn_idev->keycodesize = sizeof(unsigned short);
	ak4962->mbhc_cfg.btn_idev->keycodemax = ARRAY_SIZE(ak4962->mbhc_cfg.keycode);

	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_KEY, KEY_MEDIA);
	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_MSC, MSC_SCAN);
	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_SW, SW_MICROPHONE_INSERT);
	input_set_capability(ak4962->mbhc_cfg.btn_idev, EV_SW, SW_MICROPHONE2_INSERT);

	rc = input_register_device(ak4962->mbhc_cfg.btn_idev);
	if (rc != 0) {
		pr_err("%d: Error in input_register_device\n", rc);
		goto error_input_register;
	} else {
		pr_info("%s: input_register_device finished\n", __func__);
		goto request_virq;
	}

error_input_register:
	if (ak4962->mbhc_cfg.h2w_sdev != NULL) {
		switch_dev_unregister(ak4962->mbhc_cfg.h2w_sdev);
		kfree(ak4962->mbhc_cfg.h2w_sdev);
		ak4962->mbhc_cfg.h2w_sdev = NULL;
	}
error_switch_register:
	return rc;
#endif

request_virq:
	rc = ak49xx_request_irq(core_res, AK4962_IRQ_JDE,
		ak4962_jde_irq, "Headset detect", ak4962);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4962_IRQ_JDE);
		goto err_jde_irq;
	}

	rc = ak49xx_request_irq(core_res, AK4962_IRQ_IDE,
		ak4962_ide_irq, "Impedance detect", ak4962);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4962_IRQ_IDE);
		goto err_ide_irq;
	}

	rc = ak49xx_request_irq(core_res, AK4962_IRQ_MICE,
		ak4962_mice_irq, "Button detect", ak4962);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4962_IRQ_MICE);
		goto err_mice_irq;
	}

	rc = ak49xx_request_irq(core_res, AK4962_IRQ_SARE,
		ak4962_sare_irq, "Voice active detect", ak4962);
	if (rc) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			AK4962_IRQ_SARE);
		goto err_sare_irq;
	}
	pr_info("%s: ak49xx_request_irq finished\n", __func__);


#ifdef JACK_ALWAYS_OPEN_WHEN_PLUGOUT
	snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x1F, 0x00);	/* < 12.5% AVDD1 */
	snd_soc_update_bits(codec, DETECTION_PM, 0x82, 0x80);
#else
	snd_soc_update_bits(codec, IMPEDANCE_DETECTION, 0x1F, 0x1F);	/* > 90% AVDD1 */
	snd_soc_update_bits(codec, DETECTION_PM, 0x82, 0x82);
#endif

	return rc;

err_sare_irq:
	ak49xx_free_irq(core_res, AK4962_IRQ_MICE, ak4962);

err_mice_irq:
	ak49xx_free_irq(core_res, AK4962_IRQ_IDE, ak4962);

err_ide_irq:
	ak49xx_free_irq(core_res, AK4962_IRQ_JDE, ak4962);

err_jde_irq:
	return rc;
}
EXPORT_SYMBOL_GPL(ak4962_hs_detect);

static const struct ak4962_reg_mask_val ak4962s_reg_defaults[] = {

	/* ak4962s changes */
	/* AK4962_REG_VAL(AK4962s_BiQuad1_A1, 0x24), */
};

static void ak4962_update_reg_defaults(struct snd_soc_codec *codec)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ak4962s_reg_defaults); i++)
		snd_soc_write(codec, ak4962s_reg_defaults[i].reg,
				ak4962s_reg_defaults[i].val);
}

static const struct ak4962_reg_mask_val ak4962_codec_reg_init_val[] = {
	/*set the MCLK 9.6M */
	{FLOW_CONTROL_1, 0x04, 0x04},/* add by shengguanghui for spi6 at 21060411 begin */
	{PLL1_SOURCE_SELECTOR, 0x1F, 0x04},
	{PLL1_REF_DIVISOR_H8, 0xFF, 0x00},
	{PLL1_REF_DIVISOR_L8, 0xFF, 0x04},
	{PLL1_FB_DIVISOR_H8, 0xFF, 0x00},
	{PLL1_FB_DIVISOR_L8, 0xFF, 0x3F},

	/* set Sync Domain 1 source: Tie Low */
	{MSYNC1_MSN_CKS, 0x1F, 0x00},

	/* set Sync Domain 2 source: PLLCLK1 */
	{MSYNC2_MSN_CKS, 0x3F, 0x21},
	{MSYNC2_BDV, 0xFF, 0x27},
	{MSYNC2_SDV, 0xFF, 0x3F},

	/* set Sync Domain 3 source: PLLCLK1 */
	/* {MSYNC3_MSN_CKS, 0x1F, 0x01}, */
	{MSYNC3_MSN_CKS, 0x3F, 0x21},   /* for i2s clk by lvrg */
	{MSYNC3_BDV, 0xFF, 0x27},
	{MSYNC3_SDV, 0xFF, 0x3F},

	/* set Sync Domain 4 source: PLLCLK1 */
	{MSYNC4_MSN_CKS, 0x1F, 0x01},

	/* set Sync Domain 5 source: PLLCLK1 for slimbus */
	{MSYNC5_MSN_CKS, 0x1F, 0x01},
	{MSYNC5_BDV, 0xFF, 0x27},/* add by shengguanghui for pllclk form slimbus to i2s_ws at 20160706 */
	{MSYNC5_SDV, 0xFF, 0x3F},

	/* set Sync Domain 6 source: PLLCLK1 -> fs16kHz, bclk=64fs */
	{MSYNC6_MSN_CKS, 0x1F, 0x01},
	{MSYNC6_BDV, 0xFF, 0x77},
	{MSYNC6_SDV, 0xFF, 0x3F},

	/* set Sync Domain 7 source: PLLCLK1 -> fs24kHz, bclk=64fs */
	{MSYNC7_MSN_CKS, 0x1F, 0x01},
	{MSYNC7_BDV, 0xFF, 0x4F},
	{MSYNC7_SDV, 0xFF, 0x3F},

	/* set Sync Domain 8 source: PLLCLK1 -> fs32kHz, bclk=64fs */
	{MSYNC8_MSN_CKS, 0x1F, 0x01},
	{MSYNC8_BDV, 0xFF, 0x3B},
	{MSYNC8_SDV, 0xFF, 0x3F},

	/* set Sync Domain 9 source: PLLCLK1 -> fs96kHz, bclk=64fs */
	{MSYNC9_MSN_CKS, 0x1F, 0x01},
	{MSYNC9_BDV, 0xFF, 0x13},
	{MSYNC9_SDV, 0xFF, 0x3F},

	/* set Sync Domain 10 source: PLLCLK1 -> fs192kHz, bclk=64fs */
	{MSYNC10_MSN_CKS, 0x1F, 0x01},
	{MSYNC10_BDV, 0xFF, 0x09},
	{MSYNC10_SDV, 0xFF, 0x3F},

	/* set CODEC Clock Source -> PLLCLK1 */
	{CDCMCLK_SOURCE_SELECTOR, 0x1F, 0x01},

	/* set DSP Clock Source -> PLLCLK1 */
	{DSPMCLK_SOURCE_SELECTOR, 0x1F, 0x01},
	/* set Bus Clock -> 24.576MHz (Max Sync 192kHz) */
	/* {BUSMCLK_DIVIDER, 0xFF, 0x03}, */

	/* set AIF1/2 Sync Domain to select SYNC1 */
	{SYNC_DOMAIN_SELECTOR1, 0x77, 0x11},

	/* set AIF3/4 Sync Domain to select SYNC1 */
	{SYNC_DOMAIN_SELECTOR2, 0x77, 0x11},

	/* set CODEC Sync Domain to select SYNC7 */
	{SYNC_DOMAIN_SELECTOR3, 0x07, 0x05},

	/* set DSP Sync Domain to select SYNC5 */
	{SYNC_DOMAIN_SELECTOR5, 0x07, 0x06},

	/* set Soft SRC Sync Domain to select SYNC7 */
	{SYNC_DOMAIN_SELECTOR6, 0x77, 0x55},

	/* set SRC A/B Sync Domain to select SYNC5 */
	{SYNC_DOMAIN_SELECTOR7, 0x77, 0x66},

	/* set SRC C/D Sync Domain to select SYNC7 */
	{SYNC_DOMAIN_SELECTOR8, 0x77, 0x66},

	/* jack detection power on */
	{DETECTION_SETTING_1, 0xDF, 0xDF},	/* JCNT = 11 (70ms), INTNTM = 10ms */
	{DETECTION_SETTING_2, 0x1B, 0x10},	/* M1AVGE = 0, M23AVGE = 1, GSWCNTL = 1us */
	{MIC_DETECTION_LEVEL, 0xFF, 0x44},	/* < 68/255 */

	/* analog input start-up time */
	{MODE_CONTROL, 0x03, 0x02},

	/* dac setting */
	{CODEC_DEBUG_2, 0xff, 0x60},
	/* {CODEC_DEBUG_3, 0xff, 0x43}, */
	{CODEC_DEBUG_3, 0xff, 0x3b},
	{CODEC_DEBUG_6, 0xff, 0x07},
	/* {CODEC_DEBUG_7, 0xff, 0x70}, */
	{CODEC_DEBUG_7, 0xff, 0x00},
};

static int ak4962_find_mpwrsetting(unsigned int mpwr)
{
	int rc;

	switch (mpwr) {
	case 2800:
		rc = 0;	break;
	case 2500:
		rc = 1;	break;
	case 1800:
		rc = 2;	break;
	case 0:
		rc = 3;	break;
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int ak4962_handle_pdata(struct ak4962_priv *ak4962)
{
	struct snd_soc_codec *codec = ak4962->codec;
	struct ak49xx_pdata *pdata = ak4962->pdata;
	int k1, k2, rc = 0;

	if (!pdata) {
		rc = -ENODEV;
		goto done;
	}

	/* figure out MIC-Power value */
	k1 = ak4962_find_mpwrsetting(pdata->micbias.mpwr1_mv);
	k2 = ak4962_find_mpwrsetting(pdata->micbias.mpwr2_mv);

	if (IS_ERR_VALUE(k1) || IS_ERR_VALUE(k2)) {
		rc = -EINVAL;
		goto done;
	}

	/* Set MIC-Power level */
	snd_soc_update_bits(codec, MIC_POWER_LEVEL, 0x03, k1);
	snd_soc_update_bits(codec, MIC_POWER_LEVEL, 0x0C, k2 << 2);

done:
	return rc;
}

static void ak4962_codec_init_reg(struct snd_soc_codec *codec)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ak4962_codec_reg_init_val); i++)
		snd_soc_update_bits(codec, ak4962_codec_reg_init_val[i].reg,
				ak4962_codec_reg_init_val[i].mask,
				ak4962_codec_reg_init_val[i].val);
	pr_err("%s : [lvrg] init ak4962 register finished!\n ", __func__);
}

void ak4962_event_register(
	int (*machine_event_cb)(struct snd_soc_codec *codec,
				enum ak49xx_codec_event),
	struct snd_soc_codec *codec)
{
	struct ak4962_priv *priv = snd_soc_codec_get_drvdata(codec);

	priv->machine_codec_event_cb = machine_event_cb;
}
EXPORT_SYMBOL_GPL(ak4962_event_register);

static void ak4962_init_slim_slave_cfg(struct snd_soc_codec *codec)
{
	struct ak4962_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct afe_param_cdc_slimbus_slave_cfg *cfg;
	struct ak49xx *ak49xx = codec->control_data;
	uint64_t eaddr = 0;

	cfg = &priv->slimbus_slave_cfg;
	cfg->minor_version = 1;
	cfg->tx_slave_port_offset = 0;
	cfg->rx_slave_port_offset = 10;

	memcpy(&eaddr, &ak49xx->slim->e_addr, sizeof(ak49xx->slim->e_addr));
	WARN_ON(sizeof(ak49xx->slim->e_addr) != 6);
	cfg->device_enum_addr_lsw = eaddr & 0xFFFFFFFF;
	cfg->device_enum_addr_msw = eaddr >> 32;

	pr_info("%s: slimbus logical address 0x%llx\n", __func__, eaddr);
}

static int ak4962_device_down(struct ak49xx *ak49xx)
{
	struct snd_soc_codec *codec;

	codec = (struct snd_soc_codec *)(ak49xx->ssr_priv);
	snd_soc_card_change_online_state(codec->component.card, 0);

	return 0;
}

static int ak4962_post_reset_cb(struct ak49xx *ak49xx)
{
	int ret = 0;
	struct snd_soc_codec *codec;
	struct ak4962_priv *ak4962;

	codec = (struct snd_soc_codec *)(ak49xx->ssr_priv);
	ak4962 = snd_soc_codec_get_drvdata(codec);

	snd_soc_card_change_online_state(codec->component.card, 1);

	mutex_lock(&codec->mutex);

	ak4962_update_reg_defaults(codec);
	ak4962_codec_init_reg(codec);

	codec->cache_sync = true;
	snd_soc_cache_sync(codec);
	codec->cache_sync = false;

	ret = ak4962_handle_pdata(ak4962);
	if (IS_ERR_VALUE(ret))
		pr_err("%s: bad pdata\n", __func__);

	ak4962_init_slim_slave_cfg(codec);

	ak4962->machine_codec_event_cb(codec, AK49XX_CODEC_EVENT_CODEC_UP);

	mutex_unlock(&codec->mutex);
	return ret;
}

void *ak4962_get_afe_config(struct snd_soc_codec *codec,
			   enum afe_config_type config_type)
{
	struct ak4962_priv *priv = snd_soc_codec_get_drvdata(codec);
	/* struct ak49xx *ak4962_core = dev_get_drvdata(codec->dev->parent); */

	switch (config_type) {
	case AFE_SLIMBUS_SLAVE_CONFIG:
		return &priv->slimbus_slave_cfg;
	default:
		pr_err("%s: Unknown config_type 0x%x\n", __func__, config_type);
		return NULL;
	}
}

static int ak49xx_ssr_register(struct ak49xx *control,
				int (*device_down_cb)(struct ak49xx *ak49xx),
				int (*device_up_cb)(struct ak49xx *ak49xx),
				void *priv)
{
	control->dev_down = device_down_cb;
	control->post_reset = device_up_cb;
	control->ssr_priv = priv;
	return 0;
}

static void ak4962_aram_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	if (!fw) {
		dev_err(codec->dev, "ARAM firmware request failed\n");
		ak4962->aram_firmware[ak4962->aram_load_index] = NULL;
	} else {
		ak4962->aram_firmware[ak4962->aram_load_index] = fw;
		dev_err(codec->dev, "ARAM firmware request succeed\n");
	}
	ak4962->aram_load_index++;

	if (ak4962->aram_load_index < AK4962_NUM_ARAM) {
			request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		AK4962_ARAM_FIRMWARES[ak4962->aram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4962_aram_ready);
	}
}

static void ak4962_pram_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	if (!fw) {
		dev_err(codec->dev, "PRAM firmware request failed\n");
		ak4962->pram_firmware[ak4962->pram_load_index] = NULL;
	} else {
		ak4962->pram_firmware[ak4962->pram_load_index] = fw;
		dev_dbg(codec->dev, "PRAM firmware request succeed\n");
	}
	ak4962->pram_load_index++;

	if (ak4962->pram_load_index < AK4962_NUM_PRAM) {
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4962_PRAM_FIRMWARES[ak4962->pram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4962_pram_ready);
	} else {
			request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4962_ARAM_FIRMWARES[ak4962->aram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4962_aram_ready);
	}
}

static void ak4962_cram_ready(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	if (!fw) {
		dev_err(codec->dev, "CRAM firmware request failed\n");
		ak4962->cram_firmware[ak4962->cram_load_index] = NULL;
	} else {
		ak4962->cram_firmware[ak4962->cram_load_index] = fw;

		dev_err(codec->dev, "CRAM firmware request succeed\n");
	}
	ak4962->cram_load_index++;

	if (ak4962->cram_load_index < AK4962_NUM_CRAM) {
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4962_CRAM_FIRMWARES[ak4962->cram_load_index],
				codec->dev, GFP_KERNEL,	codec, ak4962_cram_ready);
	} else {
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				AK4962_PRAM_FIRMWARES[ak4962->pram_load_index],
				codec->dev, GFP_KERNEL, codec, ak4962_pram_ready);
	}
}

static void ak4962_timer(unsigned long data)
{
	struct ak4962_priv *ak4962 = (struct ak4962_priv *)data;

	mod_timer(&ak4962->timer, jiffies + msecs_to_jiffies(SRC_RESET_TIMEOUT));
}
/* extern int zte_get_board_id(); */
static int ak4962_codec_probe(struct snd_soc_codec *codec)
{
	struct ak49xx *control;
	struct ak4962_priv *ak4962;
	struct ak49xx *ak49xx;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	int i;
	/* int bd_id = -1; */
	void *ptr = NULL;
	struct clk *cdc_ext_clk = NULL;


	pr_err("%s Entry!\n", __func__);

	/* weizj add for audience es804 */
#ifdef CONFIG_SND_SOC_ESXXX
	dev_info(codec->dev, "%s: weizj reg es804", __func__);
	remote_add_codec_controls(codec);
#endif

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;

	ak49xx_ssr_register(control, ak4962_device_down,
			     ak4962_post_reset_cb, (void *)codec);

	/* dev_info(codec->dev, "%s()\n", __func__); */

	ak4962 = kzalloc(sizeof(struct ak4962_priv), GFP_KERNEL);
	if (!ak4962) {
		return -ENOMEM;
	}

	snd_soc_codec_set_drvdata(codec, ak4962);


	ak49xx = control;

	if (!of_find_property(ak49xx->dev->of_node, "clock-names", NULL)) {
		dev_dbg(ak49xx->dev, "%s: codec not using audio-ext-clk driver\n",
			__func__);
	} else {
		cdc_ext_clk = clk_get(ak49xx->dev, "cdc_clk");
		if (IS_ERR(cdc_ext_clk)) {
			dev_err(codec->dev, "%s: clk get %s failed\n",
					__func__, "cdc_ext_clk");
			goto err_nomem_slimch;
		}
	}
	ak4962->cdc_ext_clk = cdc_ext_clk;

	ak4962->codec = codec;
	ak4962_codec = codec;

	ak4962->pdata = dev_get_platdata(codec->dev->parent);
	ak4962->intf_type = ak49xx_get_intf_type();

	mutex_init(&ak4962->mutex);

	ak4962_update_reg_defaults(codec);
	ak4962_codec_init_reg(codec);
	pr_err("[LHS] %s LINE %d!", __func__, __LINE__);
	ret = ak4962_handle_pdata(ak4962);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: bad pdata\n", __func__);
		goto err_pdata;
	}
	pr_err("[LHS] %s LINE %d!", __func__, __LINE__);

	ptr = kmalloc((sizeof(ak4962_rx_chs) +
			   sizeof(ak4962_tx_chs)), GFP_KERNEL);
	if (!ptr) {
		ret = -ENOMEM;
		goto err_pdata;
	}
	pr_err("[LHS] %s LINE %d!", __func__, __LINE__);

	if (ak4962->intf_type == AK49XX_INTERFACE_TYPE_SPI ||
		ak4962->intf_type == AK49XX_INTERFACE_TYPE_I2C ||
		ak4962->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak4962->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		snd_soc_dapm_new_controls(dapm, ak4962_dapm_i2s_widgets,
			ARRAY_SIZE(ak4962_dapm_i2s_widgets));
		snd_soc_dapm_add_routes(dapm, audio_i2s_map,
			ARRAY_SIZE(audio_i2s_map));
		for (i = 0; i < ARRAY_SIZE(ak4962_i2s_dai); i++)
			INIT_LIST_HEAD(&ak4962->dai[i].ak49xx_ch_list);
	}
	pr_err("[LHS] %s LINE %d!", __func__, __LINE__);

	if (ak4962->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak4962->intf_type == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		for (i = 0; i < NUM_CODEC_DAIS; i++) {
			INIT_LIST_HEAD(&ak4962->dai[i].ak49xx_ch_list);
			init_waitqueue_head(&ak4962->dai[i].dai_wait);
		}
		ak4962_slimbus_slave_port_cfg.slave_dev_intfdev_la =
			control->slim_slave->laddr;
		ak4962_slimbus_slave_port_cfg.slave_dev_pgd_la =
			control->slim->laddr;
		ak4962_slimbus_slave_port_cfg.slave_port_mapping[0] = 10;

		ak4962_init_slim_slave_cfg(codec);
	}

	ret = kfifo_alloc(&ak4962->fifo_br, 16, GFP_KERNEL);	/* 2^n */
	pr_info("%s: ---------ret = %d\n", __func__, ret);
	if (ret) {
		pr_info("%s: Failed to allocate kfifo\n", __func__);
		kfifo_free(&ak4962->fifo_br);
		goto err_kfifio;
	}

	control->num_rx_port = AK4962_RX_MAX;
	control->rx_chs = ptr;
	memcpy(control->rx_chs, ak4962_rx_chs, sizeof(ak4962_rx_chs));
	control->num_tx_port = AK4962_TX_MAX;
	control->tx_chs = ptr + sizeof(ak4962_rx_chs);
	memcpy(control->tx_chs, ak4962_tx_chs, sizeof(ak4962_tx_chs));

	snd_soc_dapm_sync(dapm);

	ak4962->stream_state = AK4962_SLIMBUS_STREAM_NA;
	ak4962->pcm_state = AK4962_SLIMBUS_STREAM_NA;

	ak4962->workqueue = create_singlethread_workqueue("ak4962_wq");
	if (ak4962->workqueue == NULL) {
		goto err_wk_irq;
	}
	INIT_WORK(&ak4962->work, ak4962_work);

	ak4962->workqueue_br = create_singlethread_workqueue("ak4962_br");
	if (ak4962->workqueue_br == NULL) {
		pr_info("\t[AK4962] %s(%d)******wq error******\n", __func__, __LINE__);
		goto err_wk_irq;
	}
	INIT_DELAYED_WORK(&ak4962->work_br, ak4962_work_br);

	init_timer(&ak4962->timer);
	ak4962->timer.data = (unsigned long)ak4962;
	ak4962->timer.function = ak4962_timer;

#ifdef CONFIG_DEBUG_FS_CODEC
	debugak49xx = control;
#endif

	ak4962->cram_load_index = 0;
	ak4962->pram_load_index = 0;
	ak4962->aram_load_index = 0;

	ak4962->hp_dvol1_offset = 0;
	ak4962->hp_avol_offset = 0;

	ak4962->low_power_mode = 0;	/* rev0.16 */
	ak4962->power_select_vol_offset = HIGH_PERFORMANCE_VOL_OFFSET;
	/* rev0.16 */

	/* bd_id = zte_get_board_id(); */
	/* pr_info("zte_get_board_id board_id  bd_id=%d\n",bd_id); */
	/* if(bd_id == 16){ */
		/* AK4962_CRAM_FIRMWARES[4] = "ak4962_cram_wide_handset_new.bin"; */
		/* pr_info("zte_get_board_id board_id  AK4962_CRAM_FIRMWARES[4]= %s\n",AK4962_CRAM_FIRMWARES[4]); */
	/* } */

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			AK4962_CRAM_FIRMWARES[0], codec->dev, GFP_KERNEL,
			codec, ak4962_cram_ready);

	ak4962->ak4962_dsp_downlink_status = 0;
	ak4962->ak4962_dsp_uplink_status = 0;
	ak4962->ak4962_voice_sync_switch = 0;

	codec->component.ignore_pmdown_time = 1;
	pr_err("[LHS] %s finish!", __func__);
	return ret;

err_wk_irq:
err_kfifio:
	kfree(ptr);
err_pdata:
err_nomem_slimch:
	kfree(ak4962);

	return ret;
}

static int ak4962_codec_remove(struct snd_soc_codec *codec)
{
	int i;
	struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec);

	for (i = 0; i < AK4962_NUM_PRAM; i++) {
		if (ak4962->pram_firmware[0])
			release_firmware(ak4962->pram_firmware[0]);
	}
	for (i = 0; i < AK4962_NUM_CRAM; i++) {
		if (ak4962->cram_firmware[0])
			release_firmware(ak4962->cram_firmware[0]);
	}

#ifdef CONFIG_SWITCH
	input_unregister_device(ak4962->mbhc_cfg.btn_idev);

	if (ak4962->mbhc_cfg.h2w_sdev != NULL) {
		switch_dev_unregister(ak4962->mbhc_cfg.h2w_sdev);
		kfree(ak4962->mbhc_cfg.h2w_sdev);
		ak4962->mbhc_cfg.h2w_sdev = NULL;
	}
#endif
	ak49xx_free_irq(codec->control_data, AK4962_IRQ_JDE, ak4962);
	ak49xx_free_irq(codec->control_data, AK4962_IRQ_IDE, ak4962);
	ak49xx_free_irq(codec->control_data, AK4962_IRQ_MICE, ak4962);
	ak49xx_free_irq(codec->control_data, AK4962_IRQ_SARE, ak4962);
	destroy_workqueue(ak4962->workqueue);
	destroy_workqueue(ak4962->workqueue_br);
	del_timer_sync(&ak4962->timer);
	kfifo_free(&ak4962->fifo_br);
	kfree(ak4962);
	return 0;
}

static int ak4962_readable(struct snd_soc_codec *ssc, unsigned int reg)
{
	return ak4962_reg_readable[reg];
}

static int ak4962_volatile(struct snd_soc_codec *ssc, unsigned int reg)
{

	if ((reg >= CRC_RESULT_H8) && (reg <= IMPEDANCE_DETECTION))
		return 1;

	if ((reg >= SRC_CLK_SETTING) && (reg <= SRC_MUTE_CONTROL))
		return 1;

	if (reg == DETECTION_EVENT)
		return 1;

	if ((reg >= DSP_SETTING1) && (reg <= DSP_SETTING5))
		return 1;

	return 0;
}

static int ak4962_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int ret;
	struct ak49xx *control = codec->control_data;

	if (reg == SND_SOC_NOPM)
		return 0;

	WARN_ON(reg > AK4962_MAX_REGISTER);

	if (!ak4962_volatile(codec, reg)) {
		ret = snd_soc_cache_write(codec, reg, value);
		if (ret != 0)
			dev_err(codec->dev, "Cache write to %x failed: %d\n",
				reg, ret);
	}

	return ak49xx_reg_write(&control->core_res, reg, value);
}

static unsigned int ak4962_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int val;
	int ret;
	struct ak49xx *control = codec->control_data;

	if (reg == SND_SOC_NOPM)
		return 0;

	WARN_ON(reg > AK4962_MAX_REGISTER);

	if (!ak4962_volatile(codec, reg) && ak4962_readable(codec, reg) &&
		reg < codec->driver->reg_cache_size) {
		ret = snd_soc_cache_read(codec, reg, &val);
		if (ret >= 0) {
			return val;
		}
		dev_err(codec->dev, "Cache read from %x failed: %d\n", reg, ret);
	}

	val = ak49xx_reg_read(&control->core_res, reg);
	return val;
}

static int ak4962_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	/* struct ak4962_priv *ak4962 = snd_soc_codec_get_drvdata(codec); */
	/* struct ak49xx *control = codec->control_data; */

#if 0   /* del temp by lvrongguo @20151203 begin */
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
		}
		if (codec->dapm.bias_level == SND_SOC_BIAS_ON) {
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
		}

		if (codec->dapm.bias_level >= SND_SOC_BIAS_PREPARE) {
		}
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
#endif   /* del temp by lvrongguo @20151203 end */
	codec->dapm.bias_level = level;
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_ak4962 = {
	.probe	= ak4962_codec_probe,
	.remove	= ak4962_codec_remove,

	.read = ak4962_read,
	.write = ak4962_write,

	.readable_register = ak4962_readable,
	.volatile_register = ak4962_volatile,

	.reg_cache_size = AK4962_CACHE_SIZE,
	.reg_cache_default = ak4962_reg_defaults,
	.reg_word_size = 1,

	.controls = ak4962_snd_controls,
	.num_controls = ARRAY_SIZE(ak4962_snd_controls),
	.dapm_widgets = ak4962_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4962_dapm_widgets),
	.dapm_routes = audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),

	.set_bias_level = ak4962_set_bias_level,
};

#ifdef CONFIG_PM
static int ak4962_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: system suspend\n", __func__);
	return 0;
}

static int ak4962_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ak4962_priv *ak4962 = platform_get_drvdata(pdev);

	dev_dbg(dev, "%s: system resume ak4962 %p\n", __func__, ak4962);
	return 0;
}

static const struct dev_pm_ops ak4962_pm_ops = {
	.suspend	= ak4962_suspend,
	.resume		= ak4962_resume,
};
#endif
#if 0
int ak4962_wn_bandswitch(int howtochange)
{

	struct snd_soc_codec *codec = ak4962_codec;

	pr_info("[LHS] %s  howtochange=%d\n", __func__, howtochange);
	ak49xx_bandswitch_set(codec, howtochange);
	return 0;
}
/*add by shengguanghui for voice_wn switch 20160203 end*/
static ssize_t voice_wn_proc_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	ssize_t count = 0;

	/* pr_err("%s: enter\n", __func__); */
	if (*offset > 0) {
		count = 0;
	} else {
		pr_err("%s: enter %d\n", __func__, g_ak4962_band_mode);
		count = snprintf(buf, 2, "%d\n", g_ak4962_band_mode);
		*offset += count;
	}
	return count;
}

static ssize_t voice_wn_proc_write(struct file *file, const char __user *buffer,
				    size_t count, loff_t *pos)
{
	int ret;
	int rc;
	int err = 0;

	rc = kstrtoint_from_user(buffer, count, 0, &ret);
	if (rc)
		return rc;

	if (ret < 0 || ret > 10) {
		pr_err("%s: ret=%d out of scope(ret must 0~3)\n", __func__, ret);
		return 1;
	}

	/* if (ret == ak_ctrl_pdata) */
		/* return 1; */

	g_ak4962_band_mode = ret;
	ak4962_band_set = 0;
	err = ak4962_wn_bandswitch(g_ak4962_band_mode);

	pr_err("%s: [LHS]current_ce_level = %d\n", __func__, g_ak4962_band_mode);
	return 1;

}

static const struct file_operations voice_wn_proc_fops = {
	.owner  = THIS_MODULE,
	.read		= voice_wn_proc_read,
	.write		= voice_wn_proc_write,
};

static int voice_wn_proc_init(void)
{
	struct proc_dir_entry *res;

	res = proc_create("voice_wn_switch", S_IFREG|S_IRWXUGO, NULL,
			  &voice_wn_proc_fops);
	if (!res) {
		pr_info("failed to create /proc/voice_wn_switch\n");
		return -ENOMEM;
	}

	pr_info("created /proc/voice_wn_switch\n");
	return 0;
}
/*add by shengguanghui for voice_wn switch 20160203 end*/
#endif
static int ak4962_probe(struct platform_device *pdev)
{
	int ret = 0;
/*	int nw = 0;*/

/* zte jjp add headset detect /proc/hs 2015-03-30 start */
	struct proc_dir_entry *proc_hs_type;
/* zte jjp add headset detect /proc/hs 2015-03-30 end */
	pr_info("%s: ak4962_probe Entry!\n", __func__);
	pr_info("------shg------%s: ak4962_probe Entry!\n", __func__);
	/* return ret; */
#ifdef CONFIG_DEBUG_FS_CODEC
	ret = device_create_file(&pdev->dev, &dev_attr_reg_data);
	if (ret) {
		pr_err("%s: Error to create reg_data\n", __func__);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_ram_data);
	if (ret) {
		pr_err("%s: Error to create ram_data\n", __func__);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_ram_load);
	if (ret) {
		pr_err("%s: Error to create ram_load\n", __func__);
	}
	ret = device_create_file(&pdev->dev, &dev_attr_mir_data);
	if (ret) {
		pr_err("%s: Error to create mir_data\n", __func__);
	}
#endif

	if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS ||
		ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SLIMBUS_SPI) {
		pr_err("ak4962_probe: register SLIMbus dai\n");
		pr_info("-------------%s : ak4962_probe: register SLIMbus dai\n", __func__);
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_ak4962,
			ak4962_dai, ARRAY_SIZE(ak4962_dai));
	} else if (ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_SPI ||
			   ak49xx_get_intf_type() == AK49XX_INTERFACE_TYPE_I2C) {
		pr_info("ak4962_probe: register I2S dai\n");
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_ak4962,
			ak4962_i2s_dai, ARRAY_SIZE(ak4962_i2s_dai));
	}
/*
	nw = voice_wn_proc_init();
*/
/* zte jjp add headset detect /proc/hs 2015-03-30 start */
	proc_hs_type = proc_create("hs", S_IFREG | S_IRUGO, NULL, &hs_proc_fops);
	if (!proc_hs_type) {
	    pr_info("[YXS]hs: unable to register '/proc/hs'\n");
	}
/* zte jjp add headset detect /proc/hs 2015-03-30 end */

	return ret;
}

static int ak4962_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS_CODEC
	device_remove_file(&pdev->dev, &dev_attr_reg_data);
	device_remove_file(&pdev->dev, &dev_attr_ram_data);
	device_remove_file(&pdev->dev, &dev_attr_ram_load);
	device_remove_file(&pdev->dev, &dev_attr_mir_data);
#endif
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver ak4962_codec_driver = {
	.probe = ak4962_probe,
	.remove = ak4962_remove,
	.driver = {
		.name = "ak4962_codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &ak4962_pm_ops,
#endif
	},
};

static int __init ak4962_codec_init(void)
{
	int rtn = platform_driver_register(&ak4962_codec_driver);

	/*pr_info("%s : +++++13333333++++-1\n", __func__);*/
	if (rtn != 0) {
		/*pr_info("%s : +++++++2++++\n", __func__);*/
		platform_driver_unregister(&ak4962_codec_driver);
	}
	return rtn;
}

static void __exit ak4962_codec_exit(void)
{
	platform_driver_unregister(&ak4962_codec_driver);
}

module_init(ak4962_codec_init);
module_exit(ak4962_codec_exit);

MODULE_AUTHOR("Haizhen Li <li.kd@om.asahi-kasei.co.jp>");
MODULE_DESCRIPTION("ak4962 codec driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
