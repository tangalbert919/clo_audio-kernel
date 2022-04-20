// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials provided
 *        with the distribution.
 *
 *      * Neither the name of Qualcomm Innovation Center, Inc. nor the
 *        names of its contributors may be used to endorse or promote
 *        products derived from this software without specific prior
 *        written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <sound/hwdep.h>
#include <audio/sound/audio_effects.h>
#include <dsp/msm-dts-srs-tm-config.h>
#include <dsp/q6voice.h>
#include <dsp/q6adm-v2.h>
#include <dsp/q6asm-v2.h>
#include <dsp/q6afe-v2.h>
#include <dsp/q6core.h>
#include <dsp/q6common.h>
#include <dsp/audio_cal_utils.h>

#include "msm-pcm-routing-v2.h"
#include "msm-pcm-routing-devdep.h"
#include "msm-qti-pp-config.h"
#include "msm-dolby-dap-config.h"
#include "msm-ds2-dap-config.h"

#define DRV_NAME "msm-pcm-routing-v2"

#ifndef CONFIG_DOLBY_DAP
#undef DOLBY_ADM_COPP_TOPOLOGY_ID
#define DOLBY_ADM_COPP_TOPOLOGY_ID 0xFFFFFFFE
#endif

#ifndef CONFIG_DOLBY_DS2
#undef DS2_ADM_COPP_TOPOLOGY_ID
#define DS2_ADM_COPP_TOPOLOGY_ID 0xFFFFFFFF
#endif

#define STRING_LENGTH_OF_INT 12
#define MAX_USR_CTRL_CNT 128

static struct mutex routing_lock;

static struct cal_type_data *cal_data[MAX_ROUTING_CAL_TYPES];

#ifndef CONFIG_MI2S_DISABLE
static int pri_mi2s_switch_enable;
static int sec_mi2s_switch_enable;
#endif
static int usb_switch_enable;
static int cdc_dma_rx_switch_enable;
static int msm_route_ec_ref_rx;
static int msm_ec_ref_ch = 4;
static int msm_ec_ref_ch_downmixed = 4;
static int msm_ec_ref_bit_format = SNDRV_PCM_FORMAT_S16_LE;
static int msm_ec_ref_sampling_rate = 48000;
static uint32_t voc_session_id = ALL_SESSION_VSID;
static int msm_route_ext_ec_ref;
static bool is_custom_stereo_on;
static bool ffecns_freeze_event;
static bool swap_ch;
static bool hifi_filter_enabled;
static int msm_ec_ref_port_id;
static int afe_loopback_tx_port_index;
static int afe_loopback_tx_port_id = -1;

#define WEIGHT_0_DB 0x4000
/* all the FEs which can support channel mixer */
static struct msm_pcm_channel_mixer channel_mixer[MSM_FRONTEND_DAI_MM_SIZE];

/* all the FES which can support channel mixer for bidirection */
static struct msm_pcm_channel_mixer
	channel_mixer_v2[MSM_FRONTEND_DAI_MM_SIZE][2];

/* input BE for each FE */
static int channel_input[MSM_FRONTEND_DAI_MM_SIZE][ADM_MAX_CHANNELS];

enum {
	MADNONE,
	MADAUDIO,
	MADBEACON,
	MADULTRASOUND,
	MADSWAUDIO,
};

struct msm_pcm_route_bdai_pp_params {
	u16 port_id; /* AFE port ID */
	unsigned long pp_params_config;
	bool mute_on;
	int latency;
};

static struct msm_pcm_route_bdai_pp_params
	msm_bedais_pp_params[MSM_BACKEND_DAI_PP_PARAMS_REQ_MAX] = {
	{HDMI_RX, 0, 0, 0},
	{HDMI_RX_MS, 0, 0, 0},
	{DISPLAY_PORT_RX, 0, 0, 0},
};

/*
 * The be_dai_name_table is passed to HAL so that it can specify the
 * BE ID for the BE it wants to enable based on the name. Thus there
 * is a matching table and structure in HAL that need to be updated
 * if any changes to these are made.
 */
struct msm_pcm_route_bdai_name {
	unsigned int be_id;
	char be_name[LPASS_BE_NAME_MAX_LENGTH];
};
static struct msm_pcm_route_bdai_name be_dai_name_table[MSM_BACKEND_DAI_MAX];

static int msm_routing_send_device_pp_params(int port_id,  int copp_idx,
					     int fe_id);

static void msm_routing_load_topology(size_t data_size, void *data);
static void msm_routing_unload_topology(uint32_t topology_id);

#ifndef SND_PCM_ADD_VOLUME_CTL
static int pcm_volume_ctl_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x2000;
	return 0;
}

static void pcm_volume_ctl_private_free(struct snd_kcontrol *kcontrol)
{
	struct snd_pcm_volume *info = snd_kcontrol_chip(kcontrol);

	kfree(info);
}

/**
 * snd_pcm_add_volume_ctls - create volume control elements
 * @pcm: the assigned PCM instance
 * @stream: stream direction
 * @max_length: the max length of the volume parameter of stream
 * @private_value: the value passed to each kcontrol's private_value field
 * @info_ret: store struct snd_pcm_volume instance if non-NULL
 *
 * Create volume control elements assigned to the given PCM stream(s).
 * Returns zero if succeed, or a negative error value.
 */
int snd_pcm_add_volume_ctls(struct snd_pcm *pcm, int stream,
			   const struct snd_pcm_volume_elem *volume,
			   int max_length,
			   unsigned long private_value,
			   struct snd_pcm_volume **info_ret)
{
	int err = 0;
	int size = 0;
	struct snd_pcm_volume *info;
	struct snd_kcontrol_new knew = {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pcm_volume_ctl_info,
	};

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->pcm = pcm;
	info->stream = stream;
	info->volume = volume;
	info->max_length = max_length;
	size = sizeof("Playback ") + sizeof(" Volume") +
		STRING_LENGTH_OF_INT*sizeof(char) + 1;
	knew.name = kzalloc(size, GFP_KERNEL);
	if (!knew.name) {
		kfree(info);
		return -ENOMEM;
	}
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		snprintf((char *)knew.name, size, "%s %d %s",
			"Playback", pcm->device, "Volume");
	else
		snprintf((char *)knew.name, size, "%s %d %s",
			"Capture", pcm->device, "Volume");
	knew.device = pcm->device;
	knew.count = pcm->streams[stream].substream_count;
	knew.private_value = private_value;
	info->kctl = snd_ctl_new1(&knew, info);
	if (!info->kctl) {
		kfree(info);
		kfree(knew.name);
		return -ENOMEM;
	}
	info->kctl->private_free = pcm_volume_ctl_private_free;
	err = snd_ctl_add(pcm->card, info->kctl);
	if (err < 0) {
		kfree(info);
		kfree(knew.name);
		return -ENOMEM;
	}
	if (info_ret)
		*info_ret = info;
	kfree(knew.name);
	return 0;
}
EXPORT_SYMBOL(snd_pcm_add_volume_ctls);
#endif

#ifndef SND_PCM_ADD_USR_CTL
static int pcm_usr_ctl_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = MAX_USR_CTRL_CNT;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = INT_MAX;
	return 0;
}

static void pcm_usr_ctl_private_free(struct snd_kcontrol *kcontrol)
{
	struct snd_pcm_usr *info = snd_kcontrol_chip(kcontrol);

	kfree(info);
}

/**
 * snd_pcm_add_usr_ctls - create user control elements
 * @pcm: the assigned PCM instance
 * @stream: stream direction
 * @max_length: the max length of the user parameter of stream
 * @private_value: the value passed to each kcontrol's private_value field
 * @info_ret: store struct snd_pcm_usr instance if non-NULL
 *
 * Create usr control elements assigned to the given PCM stream(s).
 * Returns zero if succeed, or a negative error value.
 */
int snd_pcm_add_usr_ctls(struct snd_pcm *pcm, int stream,
			 const struct snd_pcm_usr_elem *usr,
			 int max_length, int max_kctrl_str_len,
			 unsigned long private_value,
			 struct snd_pcm_usr **info_ret)
{
	int err = 0;
	char *buf = NULL;
	struct snd_pcm_usr *info;
	struct snd_kcontrol_new knew = {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pcm_usr_ctl_info,
	};

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pcm = pcm;
	info->stream = stream;
	info->usr = usr;
	info->max_length = max_length;
	buf = kzalloc(max_kctrl_str_len, GFP_KERNEL);
	if (!buf) {
		pr_err("%s: buffer allocation failed\n", __func__);
		kfree(info);
		return -ENOMEM;
	}
	knew.name = buf;
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		snprintf(buf, max_kctrl_str_len, "%s %d %s",
			"Playback", pcm->device, "User kcontrol");
	else
		snprintf(buf, max_kctrl_str_len, "%s %d %s",
			"Capture", pcm->device, "User kcontrol");
	knew.device = pcm->device;
	knew.count = pcm->streams[stream].substream_count;
	knew.private_value = private_value;
	info->kctl = snd_ctl_new1(&knew, info);
	if (!info->kctl) {
		kfree(info);
		kfree(knew.name);
		pr_err("%s: snd_ctl_new failed\n", __func__);
		return -ENOMEM;
	}
	info->kctl->private_free = pcm_usr_ctl_private_free;
	err = snd_ctl_add(pcm->card, info->kctl);
	if (err < 0) {
		kfree(info);
		kfree(knew.name);
		pr_err("%s: snd_ctl_add failed:%d\n", __func__,
			err);
		return -ENOMEM;
	}
	if (info_ret)
		*info_ret = info;
	kfree(knew.name);
	return 0;
}
EXPORT_SYMBOL(snd_pcm_add_usr_ctls);
#endif

static int msm_routing_get_bit_width(unsigned int format)
{
	int bit_width;

	switch (format) {
	case SNDRV_PCM_FORMAT_S32_LE:
		bit_width = 32;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width = 24;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		bit_width = 16;
	}
	return bit_width;
}

static bool msm_is_resample_needed(int input_sr, int output_sr)
{
	bool rc = false;

	if (input_sr != output_sr)
		rc = true;

	pr_debug("perform resampling (%s) for copp rate (%d)afe rate (%d)",
		(rc ? "oh yes" : "not really"),
		input_sr, output_sr);

	return rc;
}

static void msm_pcm_routng_cfg_matrix_map_pp(struct route_payload payload,
					     int path_type, int perf_mode)
{
	int itr = 0, rc = 0;

	if ((path_type == ADM_PATH_PLAYBACK) &&
	    (perf_mode == LEGACY_PCM_MODE) &&
	    is_custom_stereo_on) {
		for (itr = 0; itr < payload.num_copps; itr++) {
			if ((payload.port_id[itr] != SLIMBUS_0_RX) &&
			    (payload.port_id[itr] != RT_PROXY_PORT_001_RX)) {
				continue;
			}

			rc = msm_qti_pp_send_stereo_to_custom_stereo_cmd(
				payload.port_id[itr],
				payload.copp_idx[itr],
				payload.session_id,
				Q14_GAIN_ZERO_POINT_FIVE,
				Q14_GAIN_ZERO_POINT_FIVE,
				Q14_GAIN_ZERO_POINT_FIVE,
				Q14_GAIN_ZERO_POINT_FIVE);
			if (rc < 0)
				pr_err("%s: err setting custom stereo\n",
					__func__);
		}
	}
}

#define SLIMBUS_EXTPROC_RX AFE_PORT_INVALID
struct msm_pcm_routing_bdai_data msm_bedais[MSM_BACKEND_DAI_MAX] = {
	{ PRIMARY_I2S_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_PRI_I2S_RX},
	{ PRIMARY_I2S_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_PRI_I2S_TX},
	{ SLIMBUS_0_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_0_RX},
	{ SLIMBUS_0_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_0_TX},
	{ HDMI_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_HDMI},
	{ INT_BT_SCO_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_INT_BT_SCO_RX},
	{ INT_BT_SCO_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_INT_BT_SCO_TX},
	{ INT_FM_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_INT_FM_RX},
	{ INT_FM_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_INT_FM_TX},
	{ RT_PROXY_PORT_001_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_AFE_PCM_RX},
	{ RT_PROXY_PORT_001_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_AFE_PCM_TX},
	{ AFE_PORT_ID_PRIMARY_PCM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_AUXPCM_RX},
	{ AFE_PORT_ID_PRIMARY_PCM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_AUXPCM_TX},
	{ VOICE_PLAYBACK_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_VOICE_PLAYBACK_TX},
	{ VOICE2_PLAYBACK_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_VOICE2_PLAYBACK_TX},
	{ VOICE_RECORD_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INCALL_RECORD_RX},
	{ VOICE_RECORD_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INCALL_RECORD_TX},
	{ MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_MI2S_RX},
	{ MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_MI2S_TX},
	{ SECONDARY_I2S_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SEC_I2S_RX},
	{ SLIMBUS_1_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_1_RX},
	{ SLIMBUS_1_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_1_TX},
	{ SLIMBUS_2_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_2_RX},
	{ SLIMBUS_2_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_2_TX},
	{ SLIMBUS_3_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_3_RX},
	{ SLIMBUS_3_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_3_TX},
	{ SLIMBUS_4_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_4_RX},
	{ SLIMBUS_4_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_4_TX},
	{ SLIMBUS_5_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_5_RX},
	{ SLIMBUS_5_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_5_TX},
	{ SLIMBUS_6_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_6_RX},
	{ SLIMBUS_6_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_6_TX},
	{ SLIMBUS_7_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_7_RX},
	{ SLIMBUS_7_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_7_TX},
	{ SLIMBUS_8_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_8_RX},
	{ SLIMBUS_8_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_8_TX},
	{ SLIMBUS_EXTPROC_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_STUB_RX},
	{ SLIMBUS_EXTPROC_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_STUB_TX},
	{ SLIMBUS_EXTPROC_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_STUB_1_TX},
	{ AFE_PORT_ID_QUATERNARY_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_MI2S_RX},
	{ AFE_PORT_ID_QUATERNARY_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_MI2S_TX},
	{ AFE_PORT_ID_SECONDARY_MI2S_RX,  0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_MI2S_RX},
	{ AFE_PORT_ID_SECONDARY_MI2S_TX,  0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_MI2S_TX},
	{ AFE_PORT_ID_PRIMARY_MI2S_RX,    0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_MI2S_RX},
	{ AFE_PORT_ID_PRIMARY_MI2S_TX,    0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_MI2S_TX},
	{ AFE_PORT_ID_TERTIARY_MI2S_RX,   0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_MI2S_RX},
	{ AFE_PORT_ID_TERTIARY_MI2S_TX,   0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_MI2S_TX},
	{ AUDIO_PORT_ID_I2S_RX,           0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_AUDIO_I2S_RX},
	{ AFE_PORT_ID_SECONDARY_PCM_RX,	  0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_AUXPCM_RX},
	{ AFE_PORT_ID_SECONDARY_PCM_TX,   0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_AUXPCM_TX},
	{ AFE_PORT_ID_PRIMARY_TDM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_0},
	{ AFE_PORT_ID_PRIMARY_TDM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_0},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_1},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_1},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_2},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_2},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_3},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_3},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_4},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_4},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_5},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_5},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_6},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_6},
	{ AFE_PORT_ID_PRIMARY_TDM_RX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_RX_7},
	{ AFE_PORT_ID_PRIMARY_TDM_TX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_TDM_TX_7},
	{ AFE_PORT_ID_SECONDARY_TDM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_0},
	{ AFE_PORT_ID_SECONDARY_TDM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_0},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_1},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_1},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_2},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_2},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_3},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_3},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_4},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_4},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_5},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_5},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_6},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_6},
	{ AFE_PORT_ID_SECONDARY_TDM_RX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_RX_7},
	{ AFE_PORT_ID_SECONDARY_TDM_TX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_TDM_TX_7},
	{ AFE_PORT_ID_TERTIARY_TDM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_0},
	{ AFE_PORT_ID_TERTIARY_TDM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_0},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_1},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_1},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_2},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_2},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_3},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_3},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_4},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_4},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_5},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_5},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_6},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_6},
	{ AFE_PORT_ID_TERTIARY_TDM_RX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_RX_7},
	{ AFE_PORT_ID_TERTIARY_TDM_TX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_TDM_TX_7},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_0},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_0},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_1},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_1},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_2},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_2},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_3},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_3},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_4},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_4},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_5},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_5},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_6},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_6},
	{ AFE_PORT_ID_QUATERNARY_TDM_RX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_RX_7},
	{ AFE_PORT_ID_QUATERNARY_TDM_TX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_TDM_TX_7},
	{ AFE_PORT_ID_QUINARY_TDM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_0},
	{ AFE_PORT_ID_QUINARY_TDM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_0},
	{ AFE_PORT_ID_QUINARY_TDM_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_1},
	{ AFE_PORT_ID_QUINARY_TDM_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_1},
	{ AFE_PORT_ID_QUINARY_TDM_RX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_2},
	{ AFE_PORT_ID_QUINARY_TDM_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_2},
	{ AFE_PORT_ID_QUINARY_TDM_RX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_3},
	{ AFE_PORT_ID_QUINARY_TDM_TX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_3},
	{ AFE_PORT_ID_QUINARY_TDM_RX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_4},
	{ AFE_PORT_ID_QUINARY_TDM_TX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_4},
	{ AFE_PORT_ID_QUINARY_TDM_RX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_5},
	{ AFE_PORT_ID_QUINARY_TDM_TX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_5},
	{ AFE_PORT_ID_QUINARY_TDM_RX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_6},
	{ AFE_PORT_ID_QUINARY_TDM_TX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_6},
	{ AFE_PORT_ID_QUINARY_TDM_RX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_RX_7},
	{ AFE_PORT_ID_QUINARY_TDM_TX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_TDM_TX_7},
	{ INT_BT_A2DP_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT_BT_A2DP_RX},
	{ AFE_PORT_ID_USB_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_USB_AUDIO_RX},
	{ AFE_PORT_ID_USB_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_USB_AUDIO_TX},
	{ DISPLAY_PORT_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_DISPLAY_PORT},
	{ DISPLAY_PORT_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_DISPLAY_PORT1},
	{ AFE_PORT_ID_TERTIARY_PCM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_AUXPCM_RX},
	{ AFE_PORT_ID_TERTIARY_PCM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TERT_AUXPCM_TX},
	{ AFE_PORT_ID_QUATERNARY_PCM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_AUXPCM_RX},
	{ AFE_PORT_ID_QUATERNARY_PCM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUAT_AUXPCM_TX},
	{ AFE_PORT_ID_QUINARY_PCM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_AUXPCM_RX},
	{ AFE_PORT_ID_QUINARY_PCM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_QUIN_AUXPCM_TX},
	{ AFE_PORT_ID_INT0_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT0_MI2S_RX},
	{ AFE_PORT_ID_INT0_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT0_MI2S_TX},
	{ AFE_PORT_ID_INT1_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT1_MI2S_RX},
	{ AFE_PORT_ID_INT1_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT1_MI2S_TX},
	{ AFE_PORT_ID_INT2_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT2_MI2S_RX},
	{ AFE_PORT_ID_INT2_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT2_MI2S_TX},
	{ AFE_PORT_ID_INT3_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT3_MI2S_RX},
	{ AFE_PORT_ID_INT3_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT3_MI2S_TX},
	{ AFE_PORT_ID_INT4_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT4_MI2S_RX},
	{ AFE_PORT_ID_INT4_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT4_MI2S_TX},
	{ AFE_PORT_ID_INT5_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT5_MI2S_RX},
	{ AFE_PORT_ID_INT5_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT5_MI2S_TX},
	{ AFE_PORT_ID_INT6_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT6_MI2S_RX},
	{ AFE_PORT_ID_INT6_MI2S_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_INT6_MI2S_TX},
	{ AFE_PORT_ID_SENARY_PCM_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEN_AUXPCM_RX},
	{ AFE_PORT_ID_SENARY_PCM_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEN_AUXPCM_TX},
	{ AFE_PORT_ID_SENARY_MI2S_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SENARY_MI2S_RX},
	{ AFE_PORT_ID_WSA_CODEC_DMA_RX_0, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_WSA_CDC_DMA_RX_0},
	{ AFE_PORT_ID_WSA_CODEC_DMA_TX_0, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_WSA_CDC_DMA_TX_0},
	{ AFE_PORT_ID_WSA_CODEC_DMA_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_WSA_CDC_DMA_RX_1},
	{ AFE_PORT_ID_WSA_CODEC_DMA_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_WSA_CDC_DMA_TX_1},
	{ AFE_PORT_ID_WSA_CODEC_DMA_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_WSA_CDC_DMA_TX_2},
	{ AFE_PORT_ID_VA_CODEC_DMA_TX_0, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_VA_CDC_DMA_TX_0},
	{ AFE_PORT_ID_VA_CODEC_DMA_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_VA_CDC_DMA_TX_1},
	{ AFE_PORT_ID_VA_CODEC_DMA_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_VA_CDC_DMA_TX_2},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_0, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_0},
	{ AFE_PORT_ID_TX_CODEC_DMA_TX_0, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TX_CDC_DMA_TX_0},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_1},
	{ AFE_PORT_ID_TX_CODEC_DMA_TX_1, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TX_CDC_DMA_TX_1},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_2},
	{ AFE_PORT_ID_TX_CODEC_DMA_TX_2, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TX_CDC_DMA_TX_2},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_3},
	{ AFE_PORT_ID_TX_CODEC_DMA_TX_3, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TX_CDC_DMA_TX_3},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_4},
	{ AFE_PORT_ID_TX_CODEC_DMA_TX_4, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TX_CDC_DMA_TX_4},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_5},
	{ AFE_PORT_ID_TX_CODEC_DMA_TX_5, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_TX_CDC_DMA_TX_5},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_6, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_6},
	{ AFE_PORT_ID_RX_CODEC_DMA_RX_7, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_RX_CDC_DMA_RX_7},
	{ AFE_PORT_ID_PRIMARY_SPDIF_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_PRI_SPDIF_TX},
	{ AFE_PORT_ID_SECONDARY_SPDIF_RX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_SPDIF_RX},
	{ AFE_PORT_ID_SECONDARY_SPDIF_TX, 0, {0}, {0}, 0, 0, 0, 0,
	  LPASS_BE_SEC_SPDIF_TX},
	{ SLIMBUS_9_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_9_RX},
	{ SLIMBUS_9_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_SLIMBUS_9_TX},
	{ AFE_LOOPBACK_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_AFE_LOOPBACK_TX},
	{ RT_PROXY_PORT_002_RX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_PROXY_RX},
	{ RT_PROXY_PORT_002_TX, 0, {0}, {0}, 0, 0, 0, 0, LPASS_BE_PROXY_TX},
};

/* Track ASM playback & capture sessions of DAI
 */
static struct msm_pcm_routing_fdai_data
	fe_dai_map[MSM_FRONTEND_DAI_MAX][2] = {
	/* MULTIMEDIA1 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA2 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA3 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA4 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA5 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA6 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA7*/
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA8 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA9 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA10 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA11 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA12 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA13 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA14 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA15 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA16 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA17 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA18 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA19 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA20 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA21 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA22 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA23 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA24 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA25 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA26 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA27 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA28 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA29 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* MULTIMEDIA30 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* VOIP */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* AFE_RX */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* AFE_TX */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* VOICE_STUB */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* DTMF_RX */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* QCHAT */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* VOLTE_STUB */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* VOICE2_STUB */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* VOICEMMODE1 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
	/* VOICEMMODE2 */
	{{0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM},
	 {0, INVALID_SESSION, LEGACY_PCM_MODE, {NULL, NULL}, LEGACY_PCM} },
};

static unsigned long session_copp_map[MSM_FRONTEND_DAI_MAX][2]
				     [MSM_BACKEND_DAI_MAX];
static struct msm_pcm_routing_app_type_data app_type_cfg[MAX_APP_TYPES];
static struct msm_pcm_stream_app_type_cfg
	fe_dai_app_type_cfg[MSM_FRONTEND_DAI_MAX][2][MSM_BACKEND_DAI_MAX];

static int last_be_id_configured[MSM_FRONTEND_DAI_MAX][MAX_SESSION_TYPES];

/* The caller of this should acquire routing lock */
void msm_pcm_routing_get_bedai_info(int be_idx,
				    struct msm_pcm_routing_bdai_data *be_dai)
{
	if (be_idx >= 0 && be_idx < MSM_BACKEND_DAI_MAX)
		memcpy(be_dai, &msm_bedais[be_idx],
			sizeof(struct msm_pcm_routing_bdai_data));
}

/* The caller of this should acquire routing lock */
void msm_pcm_routing_get_fedai_info(int fe_idx, int sess_type,
				    struct msm_pcm_routing_fdai_data *fe_dai)
{
	if ((sess_type == SESSION_TYPE_TX) || (sess_type == SESSION_TYPE_RX))
		memcpy(fe_dai, &fe_dai_map[fe_idx][sess_type],
			sizeof(struct msm_pcm_routing_fdai_data));
}

void msm_pcm_routing_acquire_lock(void)
{
	mutex_lock(&routing_lock);
}

void msm_pcm_routing_release_lock(void)
{
	mutex_unlock(&routing_lock);
}

static int msm_pcm_routing_get_app_type_idx(int app_type)
{
	int idx;

	pr_debug("%s: app_type: %d\n", __func__, app_type);
	for (idx = 0; idx < MAX_APP_TYPES; idx++) {
		if (app_type_cfg[idx].app_type == app_type)
			return idx;
	}
	pr_info("%s: App type not available, fallback to default\n", __func__);
	return 0;
}

static int get_port_id(int port_id)
{
	int ret = port_id;

	if (port_id == AFE_LOOPBACK_TX) {
		/*
		 * Return afe_loopback_tx_port_id if set. Else return
		 * msm_ec_ref_port_id to maintain backward compatibility.
		 */
		if (afe_loopback_tx_port_id != -1)
			ret = afe_loopback_tx_port_id;
		else
			ret = msm_ec_ref_port_id;
	}

	return ret;
}


/**
 * msm_pcm_routing_set_channel_mixer_cfg - cache channel mixer
 * setting before use case start.
 *
 * @fe_id: frontend idx
 * @type: stream direction type
 * @params: parameters of channel mixer setting
 *
 * Return 0 for success
 */
int msm_pcm_routing_set_channel_mixer_cfg(
	int fe_id, int type,
	struct msm_pcm_channel_mixer *params)
{
	int i, j = 0;

	channel_mixer_v2[fe_id][type].enable = params->enable;
	channel_mixer_v2[fe_id][type].rule = params->rule;
	channel_mixer_v2[fe_id][type].input_channel =
		params->input_channel;
	channel_mixer_v2[fe_id][type].output_channel =
		params->output_channel;
	channel_mixer_v2[fe_id][type].port_idx = params->port_idx;

	for (i = 0; i < ADM_MAX_CHANNELS; i++)
		channel_mixer_v2[fe_id][type].in_ch_map[i] =
			params->in_ch_map[i];
	for (i = 0; i < ADM_MAX_CHANNELS; i++)
		channel_mixer_v2[fe_id][type].out_ch_map[i] =
			params->out_ch_map[i];

	for (i = 0; i < ADM_MAX_CHANNELS; i++)
		for (j = 0; j < ADM_MAX_CHANNELS; j++)
			channel_mixer_v2[fe_id][type].channel_weight[i][j] =
				params->channel_weight[i][j];

	channel_mixer_v2[fe_id][type].override_in_ch_map =
			params->override_in_ch_map;
	channel_mixer_v2[fe_id][type].override_out_ch_map =
			params->override_out_ch_map;

	return 0;
}
EXPORT_SYMBOL(msm_pcm_routing_set_channel_mixer_cfg);

int msm_pcm_routing_reg_stream_app_type_cfg(
	int fedai_id, int session_type, int be_id,
	struct msm_pcm_stream_app_type_cfg *cfg_data)
{
	int ret = 0;

	if (cfg_data == NULL) {
		pr_err("%s: Received NULL pointer for cfg_data\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	pr_debug("%s: fedai_id %d, session_type %d, be_id %d, app_type %d, acdb_dev_id %d, sample_rate %d copp_token %d\n",
		__func__, fedai_id, session_type, be_id,
		cfg_data->app_type, cfg_data->acdb_dev_id,
		cfg_data->sample_rate, cfg_data->copp_token);

	if (session_type != SESSION_TYPE_RX &&
		session_type != SESSION_TYPE_TX) {
		pr_err("%s: Invalid session type %d\n",
			__func__, session_type);
		ret = -EINVAL;
		goto done;
	}
	if (be_id < 0 || be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Received out of bounds be_id %d\n",
			__func__, be_id);
		ret = -EINVAL;
		goto done;
	}

	fe_dai_app_type_cfg[fedai_id][session_type][be_id] = *cfg_data;

	/*
	 * Store the BE ID of the configuration information set as the latest so
	 * the get mixer control knows what to return.
	 */
	last_be_id_configured[fedai_id][session_type] = be_id;

done:
	return ret;
}
EXPORT_SYMBOL(msm_pcm_routing_reg_stream_app_type_cfg);

/**
 * msm_pcm_routing_get_stream_app_type_cfg
 *
 * Receives fedai_id, session_type, be_id, and populates app_type,
 * acdb_dev_id, & sample rate. Returns 0 on success. On failure returns
 * -EINVAL and does not alter passed values.
 *
 * fedai_id - Passed value, front end ID for which app type config is wanted
 * session_type - Passed value, session type for which app type config
 *                is wanted
 * be_id - Returned value, back end device id the app type config data is for
 * cfg_data - Returned value, configuration data used by app type config
 */
int msm_pcm_routing_get_stream_app_type_cfg(
	int fedai_id, int session_type, int *bedai_id,
	struct msm_pcm_stream_app_type_cfg *cfg_data)
{
	int be_id;
	int ret = 0;

	if (bedai_id == NULL) {
		pr_err("%s: Received NULL pointer for backend ID\n", __func__);
		ret = -EINVAL;
		goto done;
	} else if (cfg_data == NULL) {
		pr_err("%s: NULL pointer sent for cfg_data\n", __func__);
		ret = -EINVAL;
		goto done;
	} else if (session_type != SESSION_TYPE_RX &&
		   session_type != SESSION_TYPE_TX) {
		pr_err("%s: Invalid session type %d\n", __func__, session_type);
		ret = -EINVAL;
		goto done;
	}

	be_id = last_be_id_configured[fedai_id][session_type];
	if (be_id < 0 || be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Invalid BE ID %d\n", __func__, be_id);
		ret = -EINVAL;
		goto done;
	}

	*bedai_id = be_id;
	*cfg_data = fe_dai_app_type_cfg[fedai_id][session_type][be_id];
	pr_debug("%s: fedai_id %d, session_type %d, be_id %d, app_type %d, acdb_dev_id %d, sample_rate %d copp_token %d\n",
		__func__, fedai_id, session_type, *bedai_id,
		cfg_data->app_type, cfg_data->acdb_dev_id,
		cfg_data->sample_rate, cfg_data->copp_token);
done:
	return ret;
}
EXPORT_SYMBOL(msm_pcm_routing_get_stream_app_type_cfg);

static struct cal_block_data *msm_routing_find_topology_by_path(int path,
								int cal_index)
{
	struct list_head		*ptr, *next;
	struct cal_block_data		*cal_block = NULL;
	pr_debug("%s\n", __func__);

	list_for_each_safe(ptr, next,
		&cal_data[cal_index]->cal_blocks) {

		cal_block = list_entry(ptr,
			struct cal_block_data, list);

		if (cal_utils_is_cal_stale(cal_block))
			continue;

		if (((struct audio_cal_info_adm_top *)cal_block
			->cal_info)->path == path) {
			return cal_block;
		}
	}
	pr_debug("%s: Can't find topology for path %d\n", __func__, path);
	return NULL;
}

static struct cal_block_data *msm_routing_find_topology(int path,
							int app_type,
							int acdb_id,
							int cal_index,
							bool exact)
{
	struct list_head *ptr, *next;
	struct cal_block_data *cal_block = NULL;
	struct audio_cal_info_adm_top *cal_info;

	pr_debug("%s\n", __func__);

	list_for_each_safe(ptr, next,
		&cal_data[cal_index]->cal_blocks) {

		cal_block = list_entry(ptr,
			struct cal_block_data, list);

		if (cal_utils_is_cal_stale(cal_block))
			continue;

		cal_info = (struct audio_cal_info_adm_top *)
			cal_block->cal_info;
		if ((cal_info->path == path)  &&
			(cal_info->app_type == app_type) &&
			(cal_info->acdb_id == acdb_id)) {
			return cal_block;
		}
	}
	pr_debug("%s: Can't find topology for path %d, app %d, "
		 "acdb_id %d %s\n",  __func__, path, app_type, acdb_id,
		 exact ? "fail" : "defaulting to search by path");
	return exact ? NULL : msm_routing_find_topology_by_path(path,
								cal_index);
}

static int msm_routing_find_topology_on_index(int session_type, int app_type,
					      int acdb_dev_id,  int idx,
					      bool exact)
{
	int topology = -EINVAL;
	struct cal_block_data *cal_block = NULL;

	mutex_lock(&cal_data[idx]->lock);
	cal_block = msm_routing_find_topology(session_type, app_type,
					      acdb_dev_id, idx, exact);
	if (cal_block != NULL) {
		topology = ((struct audio_cal_info_adm_top *)
			    cal_block->cal_info)->topology;
	}
	mutex_unlock(&cal_data[idx]->lock);
	return topology;
}

/*
 * Retrieving cal_block will mark cal_block as stale.
 * Hence it cannot be reused or resent unless the flag
 * is reset.
 */
static int msm_routing_get_adm_topology(int fedai_id, int session_type,
					int be_id)
{
	int topology = NULL_COPP_TOPOLOGY;
	int app_type = 0, acdb_dev_id = 0;

	pr_debug("%s: fedai_id %d, session_type %d, be_id %d\n",
		__func__, fedai_id, session_type, be_id);

	if (cal_data == NULL)
		goto done;

	app_type = fe_dai_app_type_cfg[fedai_id][session_type][be_id].app_type;
	acdb_dev_id =
		fe_dai_app_type_cfg[fedai_id][session_type][be_id].acdb_dev_id;

	pr_debug("%s: Check for compatible topology\n", __func__);
	topology = msm_routing_find_topology_on_index(session_type,
						app_type,
						acdb_dev_id,
						ADM_LSM_TOPOLOGY_CAL_TYPE_IDX,
						true /*exact*/);
	if (topology < 0) {
		pr_debug("%s: Check for compatible topology\n", __func__);
		topology = msm_routing_find_topology_on_index(session_type,
							app_type,
							acdb_dev_id,
							ADM_TOPOLOGY_CAL_TYPE_IDX,
							false /*exact*/);
		if (topology < 0)
			topology = NULL_COPP_TOPOLOGY;
	}
done:
	pr_debug("%s: Using topology %d\n", __func__, topology);
	return topology;
}

static void msm_pcm_routing_build_matrix(int fedai_id, int sess_type,
					 int path_type, int perf_mode,
					 uint32_t passthr_mode)
{
	int i, port_type, j, num_copps = 0;
	struct route_payload payload;

	port_type = ((path_type == ADM_PATH_PLAYBACK ||
		      path_type == ADM_PATH_COMPRESSED_RX) ?
		MSM_AFE_PORT_TYPE_RX : MSM_AFE_PORT_TYPE_TX);

	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if ((afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		   (msm_bedais[i].active) &&
		   (test_bit(fedai_id, &msm_bedais[i].fe_sessions[0]))) {
			int port_id = get_port_id(msm_bedais[i].port_id);
			for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
				unsigned long copp =
				      session_copp_map[fedai_id][sess_type][i];
				if (test_bit(j, &copp)) {
					payload.port_id[num_copps] = port_id;
					payload.copp_idx[num_copps] = j;
					payload.app_type[num_copps] =
						fe_dai_app_type_cfg
							[fedai_id][sess_type][i]
								.app_type;
					payload.acdb_dev_id[num_copps] =
						fe_dai_app_type_cfg
							[fedai_id][sess_type][i]
								.acdb_dev_id;
					payload.sample_rate[num_copps] =
						fe_dai_app_type_cfg
							[fedai_id][sess_type][i]
								.sample_rate;
					num_copps++;
				}
			}
		}
	}

	if (num_copps) {
		payload.num_copps = num_copps;
		payload.session_id = fe_dai_map[fedai_id][sess_type].strm_id;
		adm_matrix_map(path_type, payload, perf_mode, passthr_mode);
		msm_pcm_routng_cfg_matrix_map_pp(payload, path_type, perf_mode);
	}
}

void msm_pcm_routing_reg_psthr_stream(int fedai_id, int dspst_id,
				      int stream_type)
{
	int i, session_type, path_type, port_type;
	u32 mode = 0;

	if (fedai_id >= MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID\n", __func__);
		return;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		session_type = SESSION_TYPE_RX;
		path_type = ADM_PATH_PLAYBACK;
		port_type = MSM_AFE_PORT_TYPE_RX;
	} else {
		session_type = SESSION_TYPE_TX;
		path_type = ADM_PATH_LIVE_REC;
		port_type = MSM_AFE_PORT_TYPE_TX;
	}

	mutex_lock(&routing_lock);

	fe_dai_map[fedai_id][session_type].strm_id = dspst_id;
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if ((afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		    (msm_bedais[i].active) &&
		    (test_bit(fedai_id, &msm_bedais[i].fe_sessions[0]))) {
			mode = afe_get_port_type(msm_bedais[i].port_id);
			adm_connect_afe_port(mode, dspst_id,
					     msm_bedais[i].port_id);
			break;
		}
	}
	mutex_unlock(&routing_lock);
}

/*
 * msm_pcm_routing_get_pp_ch_cnt:
 *	Read the processed channel count
 *
 * @fe_id: Front end ID
 * @session_type: Inidicates RX or TX session type
 */
int msm_pcm_routing_reg_phy_compr_stream(int fe_id, int perf_mode,
					  int dspst_id, int stream_type,
					  uint32_t passthr_mode)
{
	int i, j, session_type, path_type, port_type, topology;
	int num_copps = 0;
	struct route_payload payload;
	u32 channels, sample_rate;
	u16 bit_width = 16, be_bit_width;
	uint32_t copp_token = 0;

	pr_debug("%s:fe_id[%d] perf_mode[%d] id[%d] stream_type[%d] passt[%d]",
		 __func__, fe_id, perf_mode, dspst_id,
		 stream_type, passthr_mode);
	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		session_type = SESSION_TYPE_RX;
		if (passthr_mode != LEGACY_PCM)
			path_type = ADM_PATH_COMPRESSED_RX;
		else
			path_type = ADM_PATH_PLAYBACK;
		port_type = MSM_AFE_PORT_TYPE_RX;
	} else if (stream_type == SNDRV_PCM_STREAM_CAPTURE) {
		session_type = SESSION_TYPE_TX;
		if ((passthr_mode != LEGACY_PCM) && (passthr_mode != LISTEN))
			path_type = ADM_PATH_COMPRESSED_TX;
		else
			path_type = ADM_PATH_LIVE_REC;
		port_type = MSM_AFE_PORT_TYPE_TX;
	} else {
		pr_err("%s: invalid stream type %d\n", __func__, stream_type);
		return -EINVAL;
	}

	mutex_lock(&routing_lock);

	fe_dai_map[fe_id][session_type].strm_id = dspst_id;
	fe_dai_map[fe_id][session_type].perf_mode = perf_mode;
	fe_dai_map[fe_id][session_type].passthr_mode = passthr_mode;

	payload.num_copps = 0; /* only RX needs to use payload */
	/* re-enable EQ if active */
	msm_qti_pp_send_eq_values(fe_id);
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if ((afe_get_port_type(msm_bedais[i].port_id) ==
			port_type) &&
			(msm_bedais[i].active) &&
			(test_bit(fe_id, &msm_bedais[i].fe_sessions[0]))) {
			int app_type, app_type_idx, copp_idx, acdb_dev_id;
			int port_id = get_port_id(msm_bedais[i].port_id);

			/*
			 * check if ADM needs to be configured with different
			 * channel mapping than backend
			 */
			if (!msm_bedais[i].adm_override_ch)
				channels = msm_bedais[i].channel;
			else
				channels = msm_bedais[i].adm_override_ch;

			bit_width = msm_routing_get_bit_width(
						msm_bedais[i].format);
			app_type =
			fe_dai_app_type_cfg[fe_id][session_type][i].app_type;
			if (app_type) {
				app_type_idx =
					msm_pcm_routing_get_app_type_idx(
						app_type);
				sample_rate =
			fe_dai_app_type_cfg[fe_id][session_type][i].sample_rate;
				bit_width =
					app_type_cfg[app_type_idx].bit_width;
				copp_token =
			fe_dai_app_type_cfg[fe_id][session_type][i].copp_token;
			} else {
				sample_rate = msm_bedais[i].sample_rate;
			}
			acdb_dev_id =
			fe_dai_app_type_cfg[fe_id][session_type][i].acdb_dev_id;
			topology = msm_routing_get_adm_topology(fe_id,
								session_type,
								i);
			if ((passthr_mode == COMPRESSED_PASSTHROUGH_DSD)
			     || (passthr_mode ==
				COMPRESSED_PASSTHROUGH_GEN)
			     || (passthr_mode ==
				COMPRESSED_PASSTHROUGH_IEC61937))
				topology = COMPRESSED_PASSTHROUGH_NONE_TOPOLOGY;
			pr_debug("%s: Before adm open topology %d\n", __func__,
				topology);

			be_bit_width = msm_routing_get_bit_width(
						      msm_bedais[i].format);
			if (hifi_filter_enabled && (msm_bedais[i].sample_rate
				== 384000 || msm_bedais[i].sample_rate ==
				352800) && be_bit_width == 32)
				bit_width = msm_routing_get_bit_width(
						SNDRV_PCM_FORMAT_S32_LE);

			copp_idx =
				adm_open(port_id, path_type, sample_rate,
					 channels, topology, perf_mode,
					 bit_width, app_type, acdb_dev_id,
					 session_type, passthr_mode, copp_token);
			if ((copp_idx < 0) ||
				(copp_idx >= MAX_COPPS_PER_PORT)) {
				pr_err("%s:adm open failed coppid:%d\n",
				__func__, copp_idx);
				mutex_unlock(&routing_lock);
				return -EINVAL;
			}
			pr_debug("%s: set idx bit of fe:%d, type: %d, be:%d\n",
				 __func__, fe_id, session_type, i);
			set_bit(copp_idx,
				&session_copp_map[fe_id][session_type][i]);

			if (msm_is_resample_needed(
				sample_rate,
				msm_bedais[i].sample_rate))
				adm_copp_mfc_cfg(port_id, copp_idx,
					msm_bedais[i].sample_rate);

			for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
				unsigned long copp =
				session_copp_map[fe_id][session_type][i];
				if (test_bit(j, &copp)) {
					payload.port_id[num_copps] = port_id;
					payload.copp_idx[num_copps] = j;
					payload.app_type[num_copps] =
						fe_dai_app_type_cfg
							[fe_id][session_type][i]
								.app_type;
					payload.acdb_dev_id[num_copps] =
						fe_dai_app_type_cfg
							[fe_id][session_type][i]
								.acdb_dev_id;
					payload.sample_rate[num_copps] =
						fe_dai_app_type_cfg
							[fe_id][session_type][i]
								.sample_rate;
					num_copps++;
				}
			}
			if (passthr_mode != COMPRESSED_PASSTHROUGH_DSD
			    && passthr_mode != COMPRESSED_PASSTHROUGH_GEN
			    && passthr_mode != COMPRESSED_PASSTHROUGH_IEC61937)
				msm_routing_send_device_pp_params(port_id,
				copp_idx, fe_id);
		}
	}
	if (num_copps) {
		payload.num_copps = num_copps;
		payload.session_id = fe_dai_map[fe_id][session_type].strm_id;
		adm_matrix_map(path_type, payload, perf_mode, passthr_mode);
		msm_pcm_routng_cfg_matrix_map_pp(payload, path_type, perf_mode);
	}
	mutex_unlock(&routing_lock);
	return 0;
}

static u32 msm_pcm_routing_get_voc_sessionid(u16 val)
{
	u32 session_id;

	switch (val) {
	case MSM_FRONTEND_DAI_QCHAT:
		session_id = voc_get_session_id(QCHAT_SESSION_NAME);
		break;
	case MSM_FRONTEND_DAI_VOIP:
		session_id = voc_get_session_id(VOIP_SESSION_NAME);
		break;
	case MSM_FRONTEND_DAI_VOICEMMODE1:
		session_id = voc_get_session_id(VOICEMMODE1_NAME);
		break;
	case MSM_FRONTEND_DAI_VOICEMMODE2:
		session_id = voc_get_session_id(VOICEMMODE2_NAME);
		break;
	default:
		session_id = 0;
	}

	pr_debug("%s session_id 0x%x", __func__, session_id);
	return session_id;
}

static int msm_pcm_routing_channel_mixer_v2(int fe_id, bool perf_mode,
				int dspst_id, int stream_type)
{
	int copp_idx = 0;
	int sess_type = 0;
	int j = 0, be_id = 0;
	int ret = 0;

	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return 0;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK)
		sess_type = SESSION_TYPE_RX;
	else
		sess_type = SESSION_TYPE_TX;

	if (!(channel_mixer_v2[fe_id][sess_type].enable)) {
		pr_debug("%s: channel mixer not enabled for FE %d direction %d\n",
			__func__, fe_id, sess_type);
		return 0;
	}

	be_id = channel_mixer_v2[fe_id][sess_type].port_idx - 1;
	if (be_id < 0 || be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Received out of bounds be_id %d\n",
			__func__, be_id);
		return -EINVAL;
	}
	channel_mixer_v2[fe_id][sess_type].input_channels[0] =
		channel_mixer_v2[fe_id][sess_type].input_channel;

	pr_debug("%s sess type %d,fe_id %d,override in:%d out:%d,be active %d\n",
			__func__, sess_type, fe_id,
			channel_mixer_v2[fe_id][sess_type].override_in_ch_map,
			channel_mixer_v2[fe_id][sess_type].override_out_ch_map,
			msm_bedais[be_id].active);

	if ((msm_bedais[be_id].active) &&
		test_bit(fe_id, &msm_bedais[be_id].fe_sessions[0])) {
		unsigned long copp =
			session_copp_map[fe_id][sess_type][be_id];
		for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
			if (test_bit(j, &copp)) {
				copp_idx = j;
				break;
			}
		}

		ret = adm_programable_channel_mixer(
			msm_bedais[be_id].port_id,
			copp_idx, dspst_id, sess_type,
			&channel_mixer_v2[fe_id][sess_type], 0);
	}

	return ret;
}

static int msm_pcm_routing_channel_mixer(int fe_id, bool perf_mode,
				int dspst_id, int stream_type)
{
	int copp_idx = 0;
	int sess_type = 0;
	int i = 0, j = 0, be_id;
	int ret = 0;

	ret = msm_pcm_routing_channel_mixer_v2(fe_id, perf_mode,
				dspst_id, stream_type);
	if (ret) {
		pr_err("%s channel mixer v2 cmd  set failure%d\n", __func__,
				fe_id);
		return ret;
	}

	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return 0;
	}

	if (!(channel_mixer[fe_id].enable)) {
		pr_debug("%s: channel mixer not enabled for FE %d\n",
			__func__, fe_id);
		return 0;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK)
		sess_type = SESSION_TYPE_RX;
	else
		sess_type = SESSION_TYPE_TX;

	for (i = 0; i < ADM_MAX_CHANNELS && channel_input[fe_id][i] > 0;
		++i) {
		be_id = channel_input[fe_id][i] - 1;
		if (be_id < 0 || be_id >= MSM_BACKEND_DAI_MAX) {
			pr_err("%s: Received out of bounds be_id %d\n",
					__func__, be_id);
			return -EINVAL;
		}
		channel_mixer[fe_id].input_channels[i] =
						msm_bedais[be_id].channel;

		if ((msm_bedais[be_id].active) &&
			test_bit(fe_id,
			&msm_bedais[be_id].fe_sessions[0])) {
			unsigned long copp =
				session_copp_map[fe_id][sess_type][be_id];
			for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
				if (test_bit(j, &copp)) {
					copp_idx = j;
					break;
				}
			}

			pr_debug("%s: fe %d, be %d, channel %d, copp %d\n",
				__func__,
				fe_id, be_id, msm_bedais[be_id].channel,
				copp_idx);
			ret = adm_programable_channel_mixer(
					get_port_id(msm_bedais[be_id].port_id),
					copp_idx, dspst_id, sess_type,
					channel_mixer + fe_id, i);
		}
	}

	return ret;
}

/**
 * msm_pcm_routing_set_channel_mixer_runtime - apply channel mixer
 * setting during runtime.
 *
 * @be_id: backend index
 * @session_id: session index
 * @session_type: session type
 * @params: parameters for channel mixer
 *
 * Retuen: 0 for success, else error
 */
int msm_pcm_routing_set_channel_mixer_runtime(int be_id, int session_id,
			int session_type,
			struct msm_pcm_channel_mixer *params)
{
	int rc = 0;
	int port_id, copp_idx = 0;

	be_id--;
	if (be_id < 0 || be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: invalid backend id %d\n", __func__,
				be_id);
		return -EINVAL;
	}

	port_id = msm_bedais[be_id].port_id;
	copp_idx = adm_get_default_copp_idx(port_id);
	pr_debug("%s: port_id - %d, copp_idx %d session id - %d\n",
		 __func__, port_id, copp_idx, session_id);

	if ((params->input_channel < 0) ||
		(params->input_channel > ADM_MAX_CHANNELS)) {
		pr_err("%s: invalid input channel %d\n", __func__,
				params->input_channel);
		return -EINVAL;
	}

	if ((params->output_channel < 0) ||
		(params->output_channel > ADM_MAX_CHANNELS)) {
		pr_err("%s: invalid output channel %d\n", __func__,
				params->output_channel);
		return -EINVAL;
	}

	params->input_channels[0] = params->input_channel;

	pr_debug("%s sess type %d,override in:%d out:%d,be active %d\n",
			__func__, session_type,
			params->override_in_ch_map,
			params->override_out_ch_map,
			msm_bedais[be_id].active);

	rc = adm_programable_channel_mixer(port_id,
					copp_idx,
					session_id,
					session_type,
					params,
					0);
	if (rc) {
		pr_err("%s: send params failed rc=%d\n", __func__, rc);
		rc = -EINVAL;
	}
	return rc;
}
EXPORT_SYMBOL(msm_pcm_routing_set_channel_mixer_runtime);

int msm_pcm_routing_reg_phy_stream(int fedai_id, int perf_mode,
					int dspst_id, int stream_type)
{
	int i, j, session_type, path_type, port_type, topology, num_copps = 0;
	struct route_payload payload;
	u32 channels, sample_rate;
	uint16_t bits_per_sample = 16, be_bit_width;
	uint32_t passthr_mode = LEGACY_PCM;
	int ret = 0;
	uint32_t copp_token = 0;

	if (fedai_id >= MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID %d\n", __func__, fedai_id);
		return -EINVAL;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		session_type = SESSION_TYPE_RX;
		path_type = ADM_PATH_PLAYBACK;
		port_type = MSM_AFE_PORT_TYPE_RX;
	} else {
		session_type = SESSION_TYPE_TX;
		path_type = ADM_PATH_LIVE_REC;
		port_type = MSM_AFE_PORT_TYPE_TX;
	}

	mutex_lock(&routing_lock);

	payload.num_copps = 0; /* only RX needs to use payload */
	fe_dai_map[fedai_id][session_type].strm_id = dspst_id;
	fe_dai_map[fedai_id][session_type].perf_mode = perf_mode;
	fe_dai_map[fedai_id][session_type].passthr_mode = LEGACY_PCM;

	/* re-enable EQ if active */
	msm_qti_pp_send_eq_values(fedai_id);
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if ((afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		   (msm_bedais[i].active) &&
		   (test_bit(fedai_id, &msm_bedais[i].fe_sessions[0]))) {
			int app_type, app_type_idx, copp_idx, acdb_dev_id;
			int port_id = get_port_id(msm_bedais[i].port_id);

			/*
			 * check if ADM needs to be configured with different
			 * channel mapping than backend
			 */
			if (!msm_bedais[i].adm_override_ch)
				channels = msm_bedais[i].channel;
			else
				channels = msm_bedais[i].adm_override_ch;

			bits_per_sample = msm_routing_get_bit_width(
						msm_bedais[i].format);

			app_type =
			fe_dai_app_type_cfg[fedai_id][session_type][i].app_type;
			if (app_type) {
				app_type_idx =
				msm_pcm_routing_get_app_type_idx(app_type);
				sample_rate =
				fe_dai_app_type_cfg[fedai_id][session_type][i]
					.sample_rate;
				bits_per_sample =
					app_type_cfg[app_type_idx].bit_width;
				copp_token =
				fe_dai_app_type_cfg[fedai_id][session_type][i]
					.copp_token;
			} else
				sample_rate = msm_bedais[i].sample_rate;

			acdb_dev_id =
			fe_dai_app_type_cfg[fedai_id][session_type][i]
				.acdb_dev_id;
			topology = msm_routing_get_adm_topology(fedai_id,
								session_type,
								i);
			be_bit_width = msm_routing_get_bit_width(
						      msm_bedais[i].format);

			if (hifi_filter_enabled && (msm_bedais[i].sample_rate ==
				    384000 ||msm_bedais[i].sample_rate == 352800)
				&& be_bit_width == 32)
				bits_per_sample = msm_routing_get_bit_width(
							SNDRV_PCM_FORMAT_S32_LE);
			copp_idx = adm_open(port_id, path_type,
					    sample_rate, channels, topology,
					    perf_mode, bits_per_sample,
					    app_type, acdb_dev_id,
					    session_type, passthr_mode, copp_token);
			if ((copp_idx < 0) ||
				(copp_idx >= MAX_COPPS_PER_PORT)) {
				pr_err("%s: adm open failed copp_idx:%d\n",
					__func__, copp_idx);
				mutex_unlock(&routing_lock);
				return -EINVAL;
			}
			pr_debug("%s: setting idx bit of fe:%d, type: %d, be:%d\n",
				 __func__, fedai_id, session_type, i);
			set_bit(copp_idx,
				&session_copp_map[fedai_id][session_type][i]);

			if (msm_is_resample_needed(
				sample_rate,
				msm_bedais[i].sample_rate))
				adm_copp_mfc_cfg(port_id, copp_idx,
					msm_bedais[i].sample_rate);

			for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
				unsigned long copp =
				    session_copp_map[fedai_id][session_type][i];
				if (test_bit(j, &copp)) {
					payload.port_id[num_copps] = port_id;
					payload.copp_idx[num_copps] = j;
					payload.app_type[num_copps] =
						fe_dai_app_type_cfg
							[fedai_id][session_type]
							[i].app_type;
					payload.acdb_dev_id[num_copps] =
						fe_dai_app_type_cfg
							[fedai_id][session_type]
							[i].acdb_dev_id;
					payload.sample_rate[num_copps] =
						fe_dai_app_type_cfg
							[fedai_id][session_type]
							[i].sample_rate;
					num_copps++;
				}
			}
		}
	}
	if (num_copps) {
		payload.num_copps = num_copps;
		payload.session_id = fe_dai_map[fedai_id][session_type].strm_id;
		adm_matrix_map(path_type, payload, perf_mode, passthr_mode);
		msm_pcm_routng_cfg_matrix_map_pp(payload, path_type, perf_mode);
	}

	ret = msm_pcm_routing_channel_mixer(fedai_id, perf_mode,
				dspst_id, stream_type);
	mutex_unlock(&routing_lock);
	return ret;
}

int msm_pcm_routing_reg_phy_stream_v2(int fedai_id, int perf_mode,
				      int dspst_id, int stream_type,
				      struct msm_pcm_routing_evt event_info)
{
	if (msm_pcm_routing_reg_phy_stream(fedai_id, perf_mode, dspst_id,
					stream_type)) {
		pr_err("%s: failed to reg phy stream\n", __func__);
		return -EINVAL;
	}

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK)
		fe_dai_map[fedai_id][SESSION_TYPE_RX].event_info = event_info;
	else
		fe_dai_map[fedai_id][SESSION_TYPE_TX].event_info = event_info;
	return 0;
}

void msm_pcm_routing_dereg_phy_stream(int fedai_id, int stream_type)
{
	int i, port_type, session_type, path_type, topology, port_id;
	struct msm_pcm_routing_fdai_data *fdai;

	if (stream_type == SNDRV_PCM_STREAM_PLAYBACK) {
		 port_type = MSM_AFE_PORT_TYPE_RX;
		 session_type = SESSION_TYPE_RX;
		 path_type = ADM_PATH_PLAYBACK;
	} else {
		 port_type = MSM_AFE_PORT_TYPE_TX;
		 session_type = SESSION_TYPE_TX;
		 path_type = ADM_PATH_LIVE_REC;
	}

	mutex_lock(&routing_lock);
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		 if ((afe_get_port_type(msm_bedais[i].port_id) == port_type) &&
		    (msm_bedais[i].active) &&
		    (test_bit(fedai_id, &msm_bedais[i].fe_sessions[0]))) {
			  int idx;
			  unsigned long copp =
				   session_copp_map[fedai_id][session_type][i];
			  fdai = &fe_dai_map[fedai_id][session_type];

			  for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++)
				   if (test_bit(idx, &copp))
					    break;

			  if (idx >= MAX_COPPS_PER_PORT || idx < 0) {
				   pr_debug("%s: copp idx is invalid, exiting\n",
									__func__);
				   continue;
			  }
			  port_id = get_port_id(msm_bedais[i].port_id);
			  topology = adm_get_topology_for_port_copp_idx(
					    port_id, idx);
			  msm_routing_unload_topology(topology);
			  adm_close(port_id, fdai->perf_mode, idx);
			  pr_debug("%s:copp:%ld,idx bit fe:%d,type:%d,be:%d\n",
				    __func__, copp, fedai_id, session_type, i);
			  clear_bit(idx,
				     &session_copp_map[fedai_id][session_type][i]);
		 }
	}

	fe_dai_map[fedai_id][session_type].strm_id = INVALID_SESSION;
	fe_dai_map[fedai_id][session_type].be_srate = 0;
	mutex_unlock(&routing_lock);
}
/* Check if FE/BE route is set */
static bool msm_pcm_routing_route_is_set(u16 be_id, u16 fe_id)
{
	bool rc = false;

	if (test_bit(fe_id, &msm_bedais[be_id].fe_sessions[0]))
		rc = true;

	return rc;
}

static void msm_pcm_routing_process_audio(u16 reg, u16 val, int set)
{
	int session_type, path_type, topology;
	u32 channels, sample_rate;
	uint16_t bits_per_sample = 16, be_bit_width;
	struct msm_pcm_routing_fdai_data *fdai;
	uint32_t passthr_mode;
	uint32_t copp_token = 0;

	pr_debug("%s: reg %x val %x set %x\n", __func__, reg, val, set);

	if (val == MSM_FRONTEND_DAI_DTMF_RX &&
		afe_get_port_type(msm_bedais[reg].port_id) ==
			MSM_AFE_PORT_TYPE_RX) {
		pr_debug("%s(): set=%d port id=0x%x for dtmf generation\n",
			__func__, set, msm_bedais[reg].port_id);
		afe_set_dtmf_gen_rx_portid(msm_bedais[reg].port_id, set);
	}

	session_type =
		(afe_get_port_type(msm_bedais[reg].port_id) == MSM_AFE_PORT_TYPE_RX) ?
		SESSION_TYPE_RX : SESSION_TYPE_TX;
	fdai = &fe_dai_map[val][session_type];
	passthr_mode = fdai->passthr_mode;
	if (session_type == SESSION_TYPE_RX) {
		if (passthr_mode != LEGACY_PCM)
			path_type = ADM_PATH_COMPRESSED_RX;
		else
			path_type = ADM_PATH_PLAYBACK;
	} else {
		if ((passthr_mode != LEGACY_PCM) && (passthr_mode != LISTEN))
			path_type = ADM_PATH_COMPRESSED_TX;
		else
			path_type = ADM_PATH_LIVE_REC;
	}
	mutex_lock(&routing_lock);
	if (set) {
		if (!test_bit(val, &msm_bedais[reg].fe_sessions[0]) &&
			((msm_bedais[reg].port_id == VOICE_PLAYBACK_TX) ||
			(msm_bedais[reg].port_id == VOICE2_PLAYBACK_TX)))
			voc_start_playback(set, msm_bedais[reg].port_id);

		set_bit(val, &msm_bedais[reg].fe_sessions[0]);
		if (msm_bedais[reg].active && fdai->strm_id !=
			INVALID_SESSION) {
			int app_type, app_type_idx, copp_idx, acdb_dev_id;
			int port_id = get_port_id(msm_bedais[reg].port_id);
			/*
			 * check if ADM needs to be configured with different
			 * channel mapping than backend
			 */
			if (!msm_bedais[reg].adm_override_ch)
				channels = msm_bedais[reg].channel;
			else
				channels = msm_bedais[reg].adm_override_ch;
			if (session_type == SESSION_TYPE_TX &&
			    fdai->be_srate &&
			    (fdai->be_srate != msm_bedais[reg].sample_rate)) {
				pr_debug("%s: flush strm %d diff BE rates\n",
					__func__, fdai->strm_id);

				if (fdai->event_info.event_func)
					fdai->event_info.event_func(
						MSM_PCM_RT_EVT_BUF_RECFG,
						fdai->event_info.priv_data);
				fdai->be_srate = 0; /* might not need it */
			}

			bits_per_sample = msm_routing_get_bit_width(
						msm_bedais[reg].format);

			app_type =
			fe_dai_app_type_cfg[val][session_type][reg].app_type;
			if (app_type) {
				app_type_idx =
				msm_pcm_routing_get_app_type_idx(app_type);
				sample_rate =
				fe_dai_app_type_cfg[val][session_type][reg]
					.sample_rate;
				bits_per_sample =
					app_type_cfg[app_type_idx].bit_width;
				copp_token =
				fe_dai_app_type_cfg[val][session_type][reg]
					.copp_token;
			} else
				sample_rate = msm_bedais[reg].sample_rate;

			topology = msm_routing_get_adm_topology(val,
								session_type,
								reg);
			acdb_dev_id =
			fe_dai_app_type_cfg[val][session_type][reg].acdb_dev_id;

			be_bit_width = msm_routing_get_bit_width(
						      msm_bedais[reg].format);
			if (hifi_filter_enabled && (msm_bedais[reg].sample_rate
				== 384000 ||msm_bedais[reg].sample_rate ==
				352800) && be_bit_width == 32)
				bits_per_sample = msm_routing_get_bit_width(
							SNDRV_PCM_FORMAT_S32_LE);
			copp_idx = adm_open(port_id, path_type,
					    sample_rate, channels, topology,
					    fdai->perf_mode, bits_per_sample,
					    app_type, acdb_dev_id,
					    session_type, passthr_mode, copp_token);
			if ((copp_idx < 0) ||
			    (copp_idx >= MAX_COPPS_PER_PORT)) {
				pr_err("%s: adm open failed\n", __func__);
				mutex_unlock(&routing_lock);
				return;
			}
			pr_debug("%s: setting idx bit of fe:%d, type: %d, be:%d\n",
				 __func__, val, session_type, reg);
			set_bit(copp_idx,
				&session_copp_map[val][session_type][reg]);

			if (msm_is_resample_needed(
				sample_rate,
				msm_bedais[reg].sample_rate))
				adm_copp_mfc_cfg(port_id, copp_idx,
					msm_bedais[reg].sample_rate);

			if (session_type == SESSION_TYPE_RX &&
			    fdai->event_info.event_func)
				fdai->event_info.event_func(
					MSM_PCM_RT_EVT_DEVSWITCH,
					fdai->event_info.priv_data);

			msm_pcm_routing_build_matrix(val, session_type,
						     path_type,
						     fdai->perf_mode,
						     passthr_mode);
		}
	} else {
		if (test_bit(val, &msm_bedais[reg].fe_sessions[0]) &&
			((msm_bedais[reg].port_id == VOICE_PLAYBACK_TX) ||
			(msm_bedais[reg].port_id == VOICE2_PLAYBACK_TX)))
			voc_start_playback(set, msm_bedais[reg].port_id);
		clear_bit(val, &msm_bedais[reg].fe_sessions[0]);
		if (msm_bedais[reg].active && fdai->strm_id !=
			INVALID_SESSION) {
			int idx;
			int port_id;
			unsigned long copp =
				session_copp_map[val][session_type][reg];
			for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++)
				if (test_bit(idx, &copp))
					break;

			if (idx >= MAX_COPPS_PER_PORT) {
				pr_debug("%s: copp idx is invalid, exiting\n",
								__func__);
				mutex_unlock(&routing_lock);
				return;
			}
			port_id = get_port_id(msm_bedais[reg].port_id);
			topology = adm_get_topology_for_port_copp_idx(port_id,
								      idx);
			msm_routing_unload_topology(topology);
			adm_close(port_id, fdai->perf_mode, idx);
			pr_debug("%s: copp: %ld, reset idx bit fe:%d, type: %d, be:%d topology=0x%x\n",
				 __func__, copp, val, session_type, reg,
				 topology);
			clear_bit(idx,
				  &session_copp_map[val][session_type][reg]);
			msm_pcm_routing_build_matrix(val, session_type,
						     path_type,
						     fdai->perf_mode,
						     passthr_mode);
		}
	}
	if ((msm_bedais[reg].port_id == VOICE_RECORD_RX)
			|| (msm_bedais[reg].port_id == VOICE_RECORD_TX))
		voc_start_record(msm_bedais[reg].port_id, set, voc_session_id);

	mutex_unlock(&routing_lock);
}

static int msm_routing_get_audio_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	(struct soc_mixer_control *)kcontrol->private_value;

	if (test_bit(mc->rshift, &msm_bedais[mc->shift].fe_sessions[0]))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	pr_debug("%s: shift %x rshift %x val %ld\n", __func__, mc->shift, mc->rshift,
	ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_routing_put_audio_mixer(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;

	if (ucontrol->value.integer.value[0] &&
	   msm_pcm_routing_route_is_set(mc->shift, mc->rshift) == false) {
		msm_pcm_routing_process_audio(mc->shift, mc->rshift, 1);
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 1,
			update);
	} else if (!ucontrol->value.integer.value[0] &&
		  msm_pcm_routing_route_is_set(mc->shift, mc->rshift) == true) {
		msm_pcm_routing_process_audio(mc->shift, mc->rshift, 0);
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 0,
			update);
	}

	return 1;
}

static void msm_pcm_routing_process_voice(u16 reg, u16 val, int set)
{
	u32 session_id = 0;
	u32 session_idx = 0;
	u16 path_type;
	struct media_format_info voc_be_media_format;

	pr_debug("%s: reg %x val %x set %x\n", __func__, reg, val, set);

	session_id = msm_pcm_routing_get_voc_sessionid(val);

	if (!session_id) {
		pr_err("%s: Invalid session_id %x\n", __func__, session_id);
		return;
	}

	pr_debug("%s: FE DAI 0x%x session_id 0x%x\n",
		__func__, val, session_id);

	mutex_lock(&routing_lock);

	if (set)
		set_bit(val, &msm_bedais[reg].fe_sessions[0]);
	else
		clear_bit(val, &msm_bedais[reg].fe_sessions[0]);

	if (val == MSM_FRONTEND_DAI_DTMF_RX &&
	    afe_get_port_type(msm_bedais[reg].port_id) ==
						MSM_AFE_PORT_TYPE_RX) {
		pr_debug("%s(): set=%d port id=0x%x for dtmf generation\n",
			 __func__, set, msm_bedais[reg].port_id);
		afe_set_dtmf_gen_rx_portid(msm_bedais[reg].port_id, set);
	}
	if (session_id != 0 && afe_get_port_type(msm_bedais[reg].port_id) ==
						MSM_AFE_PORT_TYPE_RX) {
		pr_err("%s(): set=%d port id=0x%x for dtmf generation\n",
			 __func__, set, msm_bedais[reg].port_id);
		session_idx = voice_get_idx_for_session(session_id);
		afe_set_dtmf_gen_rx_portid_session(msm_bedais[reg].port_id, set, session_idx);
	}

	if (afe_get_port_type(msm_bedais[reg].port_id) ==
						MSM_AFE_PORT_TYPE_RX)
		path_type = RX_PATH;
	else
		path_type = TX_PATH;

	if (set) {
		if (msm_bedais[reg].active) {
			voc_set_route_flag(session_id, path_type, 1);

			memset(&voc_be_media_format, 0,
				sizeof(struct media_format_info));

			voc_be_media_format.port_id = msm_bedais[reg].port_id;
			voc_be_media_format.num_channels =
						msm_bedais[reg].channel;
			voc_be_media_format.sample_rate =
						msm_bedais[reg].sample_rate;
			voc_be_media_format.bits_per_sample =
						msm_bedais[reg].format;
			/* Defaulting this to 1 for voice call usecases */
			voc_be_media_format.channel_mapping[0] = 1;

			voc_set_device_config(session_id, path_type,
					      &voc_be_media_format);

			if (voc_get_route_flag(session_id, TX_PATH) &&
				voc_get_route_flag(session_id, RX_PATH))
				voc_enable_device(session_id);
		} else {
			pr_debug("%s BE is not active\n", __func__);
		}
	} else {
		voc_set_route_flag(session_id, path_type, 0);
		voc_disable_device(session_id);
	}

	mutex_unlock(&routing_lock);

}

static int msm_routing_get_voice_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	(struct soc_mixer_control *)kcontrol->private_value;

	mutex_lock(&routing_lock);

	if (test_bit(mc->rshift, &msm_bedais[mc->shift].fe_sessions[0]))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	mutex_unlock(&routing_lock);

	pr_debug("%s: shift %x rshift %x val %ld\n", __func__, mc->shift, mc->rshift,
			ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_routing_put_voice_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;

	if (ucontrol->value.integer.value[0]) {
		msm_pcm_routing_process_voice(mc->shift, mc->rshift, 1);
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 1,
						update);
	} else {
		msm_pcm_routing_process_voice(mc->shift, mc->rshift, 0);
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 0,
						update);
	}

	return 1;
}

static int msm_routing_get_voice_stub_mixer(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	mutex_lock(&routing_lock);

	if (test_bit(mc->rshift, &msm_bedais[mc->shift].fe_sessions[0]))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	mutex_unlock(&routing_lock);

	pr_debug("%s: shift %x rshift %x val %ld\n", __func__, mc->shift, mc->rshift,
		ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_routing_put_voice_stub_mixer(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;

	if (ucontrol->value.integer.value[0]) {
		mutex_lock(&routing_lock);
		set_bit(mc->rshift, &msm_bedais[mc->shift].fe_sessions[0]);
		mutex_unlock(&routing_lock);

		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 1,
						update);
	} else {
		mutex_lock(&routing_lock);
		clear_bit(mc->rshift, &msm_bedais[mc->shift].fe_sessions[0]);
		mutex_unlock(&routing_lock);

		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 0,
						update);
	}

	pr_debug("%s: shift %x rshift %x val %ld\n", __func__, mc->shift, mc->rshift,
		ucontrol->value.integer.value[0]);

	return 1;
}

/*
 * Return the mapping between port ID and backend ID to enable the AFE callback
 * to determine the acdb_dev_id from the port id
 */
int msm_pcm_get_be_id_from_port_id(int port_id)
{
	int i;
	int be_id = -EINVAL;

	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (msm_bedais[i].port_id == port_id) {
			be_id = i;
			break;
		}
	}

	return be_id;
}

/*
 * Return the registered dev_acdb_id given a port ID to enable identifying the
 * correct AFE calibration information by comparing the header information.
 */
static int msm_pcm_get_dev_acdb_id_by_port_id(int port_id)
{
	int acdb_id = -EINVAL;
	int i = 0;
	int session;
	int port_type = afe_get_port_type(port_id);
	int be_id = msm_pcm_get_be_id_from_port_id(port_id);

	pr_debug("%s:port_id %d be_id %d, port_type 0x%x\n",
		  __func__, port_id, be_id, port_type);

	if (port_type == MSM_AFE_PORT_TYPE_TX) {
		session = SESSION_TYPE_TX;
	} else if (port_type == MSM_AFE_PORT_TYPE_RX) {
		session = SESSION_TYPE_RX;
	} else {
		pr_err("%s: Invalid port type %d\n", __func__, port_type);
		acdb_id = -EINVAL;
		goto exit;
	}

	if (be_id < 0) {
		pr_err("%s: Error getting backend id %d\n", __func__, be_id);
		goto exit;
	}

	mutex_lock(&routing_lock);
	i = find_first_bit(&msm_bedais[be_id].fe_sessions[0],
			   MSM_FRONTEND_DAI_MAX);
	if (i < MSM_FRONTEND_DAI_MAX)
		acdb_id = fe_dai_app_type_cfg[i][session][be_id].acdb_dev_id;

	pr_debug("%s: FE[%d] session[%d] BE[%d] acdb_id(%d)\n",
		 __func__, i, session, be_id, acdb_id);
	mutex_unlock(&routing_lock);
exit:
	return acdb_id;
}
#ifndef CONFIG_MI2S_DISABLE
static int msm_routing_get_pri_mi2s_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = pri_mi2s_switch_enable;
	pr_debug("%s: PRI MI2S Switch enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_routing_put_pri_mi2s_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_dapm_update *update = NULL;

	pr_debug("%s: PRI MI2S Switch enable %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 1,
						update);
	else
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 0,
						update);
	pri_mi2s_switch_enable = ucontrol->value.integer.value[0];
	return 1;
}

static int msm_routing_get_sec_mi2s_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = sec_mi2s_switch_enable;
	pr_debug("%s: SEC MI2S Switch enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_routing_put_sec_mi2s_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_dapm_update *update = NULL;

	pr_debug("%s: SEC MI2S Switch enable %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 1,
						update);
	else
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol, 0,
						update);
	sec_mi2s_switch_enable = ucontrol->value.integer.value[0];
	return 1;
}
#endif

static int msm_routing_get_usb_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = usb_switch_enable;
	pr_debug("%s: HFP Switch enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_routing_put_usb_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_dapm_update *update = NULL;

	pr_debug("%s: USB Switch enable %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol,
						1, update);
	else
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol,
						0, update);
	usb_switch_enable = ucontrol->value.integer.value[0];
	return 1;
}

static int msm_routing_get_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = cdc_dma_rx_switch_enable;
	pr_debug("%s: Switch enable %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_routing_put_switch_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_dapm_update *update = NULL;

	pr_debug("%s: Switch enable %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	if (ucontrol->value.integer.value[0])
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol,
						1, update);
	else
		snd_soc_dapm_mixer_update_power(widget->dapm, kcontrol,
						0, update);
	cdc_dma_rx_switch_enable = ucontrol->value.integer.value[0];
	return 1;
}

static int msm_routing_get_port_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int idx = 0, shift = 0;
	struct soc_mixer_control *mc =
	(struct soc_mixer_control *)kcontrol->private_value;

	idx = mc->rshift/(sizeof(msm_bedais[mc->shift].port_sessions[0]) * 8);
	shift = mc->rshift%(sizeof(msm_bedais[mc->shift].port_sessions[0]) * 8);

	if (idx >= BE_DAI_PORT_SESSIONS_IDX_MAX) {
		pr_err("%s: Invalid idx = %d\n", __func__, idx);
		return -EINVAL;
	}

	if (test_bit(shift,
		(unsigned long *)&msm_bedais[mc->shift].port_sessions[idx]))
		ucontrol->value.integer.value[0] = 1;
	else
		ucontrol->value.integer.value[0] = 0;

	pr_debug("%s: shift %x rshift %x val %ld\n", __func__, mc->shift, mc->rshift,
	ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_routing_put_port_mixer(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int idx = 0, shift = 0;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	idx = mc->rshift/(sizeof(msm_bedais[mc->shift].port_sessions[0]) * 8);
	shift = mc->rshift%(sizeof(msm_bedais[mc->shift].port_sessions[0]) * 8);

	if (idx >= BE_DAI_PORT_SESSIONS_IDX_MAX) {
		pr_err("%s: Invalid idx = %d\n", __func__, idx);
		return -EINVAL;
	}

	pr_debug("%s: shift 0x%x rshift 0x%x val %ld idx %d reminder shift %d\n",
		 __func__, mc->shift, mc->rshift,
		 ucontrol->value.integer.value[0], idx, shift);

	if (ucontrol->value.integer.value[0]) {
		afe_loopback(1, msm_bedais[mc->shift].port_id,
			    msm_bedais[mc->rshift].port_id);
		set_bit(shift,
		(unsigned long *)&msm_bedais[mc->shift].port_sessions[idx]);
	} else {
		afe_loopback(0, msm_bedais[mc->shift].port_id,
			    msm_bedais[mc->rshift].port_id);
		clear_bit(shift,
		(unsigned long *)&msm_bedais[mc->shift].port_sessions[idx]);
	}

	return 1;
}

static int msm_pcm_get_channel_rule_index(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;

	fe_id = ((struct soc_mixer_control *)
			kcontrol->private_value)->shift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = channel_mixer[fe_id].rule;

	return 0;
}

static int msm_pcm_put_channel_rule_index(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;

	fe_id = ((struct soc_mixer_control *)
			kcontrol->private_value)->shift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	channel_mixer[fe_id].rule = ucontrol->value.integer.value[0];

	return 1;
}

static int msm_pcm_get_out_chs(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;

	fe_id = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] =
		channel_mixer[fe_id].output_channel;
	return 0;
}

static int msm_pcm_put_out_chs(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0, out_ch = 0;
	fe_id = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;
	out_ch = ucontrol->value.integer.value[0];
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	pr_debug("%s: fe_id is %d, output channels = %d\n", __func__,
			fe_id,
			(unsigned int)(ucontrol->value.integer.value[0]));
	if (out_ch < 0 ||
		out_ch > ADM_MAX_CHANNELS) {
		pr_err("%s: invalid output channel %d\n", __func__,
				out_ch);
		return -EINVAL;
	}
	channel_mixer[fe_id].output_channel =
			(unsigned int)(ucontrol->value.integer.value[0]);

	return 1;
}

static const char *const ch_mixer[] = {"Disable", "Enable"};

/* If new backend is added, need update this array */
static const char *const be_name[] = {
"ZERO",

"AFE_PCM_RX", "AFE_PCM_TX",
"AUXPCM_RX", "AUXPCM_TX", "VOICE_PLAYBACK_TX", "VOICE2_PLAYBACK_TX",
"INCALL_RECORD_RX", "INCALL_RECORD_TX",
"SECOND_MI2S_RX", "SECOND_MI2S_TX",
"PRI_MI2S_RX", "PRI_MI2S_TX", "SEC_MI2S_RX", "SEC_MI2S_TX",
"TERT_MI2S_RX", "TERT_MI2S_TX", "QUAT_MI2S_RX", "QUAT_MI2S_TX",
"SEC_AUXPCM_RX", "SEC_AUXPCM_TX", "TERT_AUXPCM_RX", "TERT_AUXPCM_TX",
"QUAT_AUXPCM_RX", "QUAT_AUXPCM_TX",
"PRI_TDM_RX_0", "PRI_TDM_TX_0", "PRI_TDM_RX_1", "PRI_TDM_TX_1",
"PRI_TDM_RX_2", "PRI_TDM_TX_2", "PRI_TDM_RX_3", "PRI_TDM_TX_3",
"PRI_TDM_RX_4", "PRI_TDM_TX_4", "PRI_TDM_RX_5", "PRI_TDM_TX_5",
"PRI_TDM_RX_6", "PRI_TDM_TX_6", "PRI_TDM_RX_7", "PRI_TDM_TX_7",
"SEC_TDM_RX_0", "SEC_TDM_TX_0", "SEC_TDM_RX_1", "SEC_TDM_TX_1",
"SEC_TDM_RX_2", "SEC_TDM_TX_2", "SEC_TDM_RX_3", "SEC_TDM_TX_3",
"SEC_TDM_RX_4", "SEC_TDM_TX_4", "SEC_TDM_RX_5", "SEC_TDM_TX_5",
"SEC_TDM_RX_6", "SEC_TDM_TX_6", "SEC_TDM_RX_7", "SEC_TDM_TX_7",
"TERT_TDM_RX_0", "TERT_TDM_TX_0", "TERT_TDM_RX_1", "TERT_TDM_TX_1",
"TERT_TDM_RX_2", "TERT_TDM_TX_2", "TERT_TDM_RX_3", "TERT_TDM_TX_3",
"TERT_TDM_RX_4", "TERT_TDM_TX_4", "TERT_TDM_RX_5", "TERT_TDM_TX_5",
"TERT_TDM_RX_6", "TERT_TDM_TX_6", "TERT_TDM_RX_7", "TERT_TDM_TX_7",
"QUAT_TDM_RX_0", "QUAT_TDM_TX_0", "QUAT_TDM_RX_1", "QUAT_TDM_TX_1",
"QUAT_TDM_RX_2", "QUAT_TDM_TX_2", "QUAT_TDM_RX_3", "QUAT_TDM_TX_3",
"QUAT_TDM_RX_4", "QUAT_TDM_TX_4", "QUAT_TDM_RX_5", "QUAT_TDM_TX_5",
"QUAT_TDM_RX_6", "QUAT_TDM_TX_6", "QUAT_TDM_RX_7", "QUAT_TDM_TX_7",
"USB_RX", "USB_TX",
"WSA_CDC_DMA_RX_0", "WSA_CDC_DMA_TX_0", "WSA_CDC_DMA_RX_1","WSA_CDC_DMA_TX_1",
"WSA_CDC_DMA_TX_2", "VA_CDC_DMA_TX_0", "VA_CDC_DMA_TX_1", "VA_CDC_DMA_TX_2",
"RX_CDC_DMA_RX_0", "TX_CDC_DMA_TX_0", "RX_CDC_DMA_RX_1", "TX_CDC_DMA_TX_1",
"RX_CDC_DMA_RX_2", "TX_CDC_DMA_TX_2", "RX_CDC_DMA_RX_3", "TX_CDC_DMA_TX_3",
"RX_CDC_DMA_RX_4", "TX_CDC_DMA_TX_4", "RX_CDC_DMA_RX_5", "TX_CDC_DMA_TX_5",
"RX_CDC_DMA_RX_6", "RX_CDC_DMA_RX_7",
"AFE_LOOPBACK_TX",
"PROXY_RX", "PROXY_TX"
};

static SOC_ENUM_SINGLE_DECL(mm1_channel_mux,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, ch_mixer);
static SOC_ENUM_SINGLE_DECL(mm2_channel_mux,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA2, ch_mixer);
static SOC_ENUM_SINGLE_DECL(mm3_channel_mux,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA3, ch_mixer);
static SOC_ENUM_SINGLE_DECL(mm4_channel_mux,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA4, ch_mixer);

static SOC_ENUM_DOUBLE_DECL(mm1_ch1_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 0, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch2_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 1, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch3_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 2, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch4_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 3, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch5_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 4, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch6_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 5, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch7_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 6, be_name);
static SOC_ENUM_DOUBLE_DECL(mm1_ch8_enum,
	SND_SOC_NOPM, MSM_FRONTEND_DAI_MULTIMEDIA1, 7, be_name);

static int msm_pcm_get_ctl_enum_info(struct snd_ctl_elem_info *uinfo,
		unsigned int channels,
		unsigned int items, const char *const names[])
{
	if (uinfo->value.enumerated.item >= items)
		uinfo->value.enumerated.item = items - 1;

	WARN(strlen(names[uinfo->value.enumerated.item]) >=
		sizeof(uinfo->value.enumerated.name),
		"ALSA: too long item name '%s'\n",
		names[uinfo->value.enumerated.item]);
	strlcpy(uinfo->value.enumerated.name,
		names[uinfo->value.enumerated.item],
		sizeof(uinfo->value.enumerated.name));
	return 0;
}

static int msm_pcm_channel_mixer_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	uinfo->value.enumerated.items = ARRAY_SIZE(ch_mixer);
	msm_pcm_get_ctl_enum_info(uinfo, 1, e->items, e->texts);

	return 0;
}
static int msm_pcm_channel_mixer_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;

	fe_id = ((struct soc_enum *)
			kcontrol->private_value)->shift_l;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	pr_debug("%s: FE %d %s\n", __func__,
		fe_id,
		channel_mixer[fe_id].enable ? "Enabled" : "Disabled");
	ucontrol->value.enumerated.item[0] = channel_mixer[fe_id].enable;
	return 0;
}

static int msm_pcm_channel_mixer_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;

	fe_id = ((struct soc_enum *)
			kcontrol->private_value)->shift_l;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	channel_mixer[fe_id].enable = ucontrol->value.enumerated.item[0];
	pr_debug("%s: %s FE %d\n", __func__,
		channel_mixer[fe_id].enable ? "Enable" : "Disable",
		fe_id);
	return 0;
}

static int msm_pcm_channel_input_be_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;

	uinfo->value.enumerated.items = ARRAY_SIZE(be_name);
	msm_pcm_get_ctl_enum_info(uinfo, 1, e->items, e->texts);

	return 0;
}

static int msm_pcm_channel_input_be_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	u16 fe_id = 0, in_ch = 0;

	fe_id = e->shift_l;
	in_ch = e->shift_r;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}
	if (in_ch >= ADM_MAX_CHANNELS) {
		pr_err("%s: invalid input channel %d\n", __func__, in_ch);
		return -EINVAL;
	}

	channel_input[fe_id][in_ch] = ucontrol->value.enumerated.item[0];
	return 1;
}

static int msm_pcm_channel_input_be_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	u16 fe_id = 0, in_ch = 0;

	fe_id = e->shift_l;
	in_ch = e->shift_r;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}
	if (in_ch >= ADM_MAX_CHANNELS) {
		pr_err("%s: invalid input channel %d\n", __func__, in_ch);
		return -EINVAL;
	}

	ucontrol->value.enumerated.item[0] = channel_input[fe_id][in_ch];
	return 1;
}


static int msm_pcm_channel_weight_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = ADM_MAX_CHANNELS;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = WEIGHT_0_DB;

	return 0;
}

static int msm_pcm_channel_weight_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0, out_ch = 0;
	int i, weight;

	fe_id = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;
	out_ch = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->rshift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}
	if (out_ch >= ADM_MAX_CHANNELS) {
		pr_err("%s: invalid input channel %d\n", __func__, out_ch);
		return -EINVAL;
	}

	pr_debug("%s: FE_ID: %d, channel weight %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\n",
		__func__, fe_id,
		ucontrol->value.integer.value[0],
		ucontrol->value.integer.value[1],
		ucontrol->value.integer.value[2],
		ucontrol->value.integer.value[3],
		ucontrol->value.integer.value[4],
		ucontrol->value.integer.value[5],
		ucontrol->value.integer.value[6],
		ucontrol->value.integer.value[7]);

	for (i = 0; i < ADM_MAX_CHANNELS; ++i) {
		weight = ucontrol->value.integer.value[i];
		channel_mixer[fe_id].channel_weight[out_ch][i] = weight;
		pr_debug("%s: FE_ID %d, output %d input %d weight %d\n",
			__func__, fe_id, out_ch, i,
			channel_mixer[fe_id].channel_weight[out_ch][i]);
	}

	return 0;
}

static int msm_pcm_channel_weight_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0, out_ch = 0;
	int i;

	fe_id = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;
	out_ch = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->rshift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}
	if (out_ch >= ADM_MAX_CHANNELS) {
		pr_err("%s: invalid input channel %d\n", __func__, out_ch);
		return -EINVAL;
	}

	for (i = 0; i < ADM_MAX_CHANNELS; ++i)
		ucontrol->value.integer.value[i] =
			channel_mixer[fe_id].channel_weight[out_ch][i];

	pr_debug("%s: FE_ID: %d, weight  %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld",
		__func__, fe_id,
		ucontrol->value.integer.value[0],
		ucontrol->value.integer.value[1],
		ucontrol->value.integer.value[2],
		ucontrol->value.integer.value[3],
		ucontrol->value.integer.value[4],
		ucontrol->value.integer.value[5],
		ucontrol->value.integer.value[6],
		ucontrol->value.integer.value[7]);

	return 0;
}

static int msm_pcm_channel_output_map_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = PCM_FORMAT_MAX_NUM_CHANNEL_V8;
	/* Valid channel map value ranges from 1 to 64 */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 64;

	return 0;
}

static int msm_pcm_channel_output_map_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;
	int i, ch_map;

	fe_id = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL_V8; ++i) {
		ch_map = ucontrol->value.integer.value[i];
		channel_mixer[fe_id].out_ch_map[i] = ch_map;
		pr_debug("%s: FE_ID %d, channel %d channel map %d\n",
			__func__, fe_id, i, channel_mixer[fe_id].out_ch_map[i]);
	}

	/* Make override_out_ch_map false if ch map is reset */
	if (channel_mixer[fe_id].out_ch_map[0] == 0)
		channel_mixer[fe_id].override_out_ch_map = false;
	else
		channel_mixer[fe_id].override_out_ch_map = true;

	return 0;
}

static int msm_pcm_channel_output_map_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 fe_id = 0;
	int i;

	fe_id = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;
	if (fe_id >= MSM_FRONTEND_DAI_MM_SIZE) {
		pr_err("%s: invalid FE %d\n", __func__, fe_id);
		return -EINVAL;
	}

	for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL_V8; ++i)
		ucontrol->value.integer.value[i] =
			channel_mixer[fe_id].out_ch_map[i];

	return 0;
}

static const struct snd_kcontrol_new channel_mixer_controls[] = {
	SOC_SINGLE_EXT("MultiMedia1 Channel Rule", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA1, 8, 0,
			msm_pcm_get_channel_rule_index,
			msm_pcm_put_channel_rule_index),
	SOC_SINGLE_EXT("MultiMedia2 Channel Rule", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA2, 8, 0,
			msm_pcm_get_channel_rule_index,
			msm_pcm_put_channel_rule_index),
	SOC_SINGLE_EXT("MultiMedia3 Channel Rule", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA3, 8, 0,
			msm_pcm_get_channel_rule_index,
			msm_pcm_put_channel_rule_index),
	SOC_SINGLE_EXT("MultiMedia4 Channel Rule", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA4, 8, 0,
			msm_pcm_get_channel_rule_index,
			msm_pcm_put_channel_rule_index),

	SOC_SINGLE_EXT("MultiMedia1 Channels", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA1, 14, 0,
			msm_pcm_get_out_chs,
			msm_pcm_put_out_chs),
	SOC_SINGLE_EXT("MultiMedia2 Channels", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA2, 8, 0,
			msm_pcm_get_out_chs,
			msm_pcm_put_out_chs),
	SOC_SINGLE_EXT("MultiMedia3 Channels", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA3, 8, 0,
			msm_pcm_get_out_chs,
			msm_pcm_put_out_chs),
	SOC_SINGLE_EXT("MultiMedia4 Channels", SND_SOC_NOPM,
			MSM_FRONTEND_DAI_MULTIMEDIA4, 8, 0,
			msm_pcm_get_out_chs,
			msm_pcm_put_out_chs),

	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Channel Mixer",
	.info = msm_pcm_channel_mixer_info,
	.get = msm_pcm_channel_mixer_get,
	.put = msm_pcm_channel_mixer_put,
	.private_value = (unsigned long)&(mm1_channel_mux)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia2 Channel Mixer",
	.info = msm_pcm_channel_mixer_info,
	.get = msm_pcm_channel_mixer_get,
	.put = msm_pcm_channel_mixer_put,
	.private_value = (unsigned long)&(mm2_channel_mux)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia3 Channel Mixer",
	.info = msm_pcm_channel_mixer_info,
	.get = msm_pcm_channel_mixer_get,
	.put = msm_pcm_channel_mixer_put,
	.private_value = (unsigned long)&(mm3_channel_mux)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia4 Channel Mixer",
	.info = msm_pcm_channel_mixer_info,
	.get = msm_pcm_channel_mixer_get,
	.put = msm_pcm_channel_mixer_put,
	.private_value = (unsigned long)&(mm4_channel_mux)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel1",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 0,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel2",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 1, }
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel3",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 2,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel4",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 3,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel5",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 4,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel6",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 5,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel7",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 6,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel8",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 7,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel9",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 8,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel10",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 9,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel11",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 10,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel12",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 11,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel13",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 12,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel14",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1, .rshift = 13,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia2 Output Channel1",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{.shift = MSM_FRONTEND_DAI_MULTIMEDIA2, .rshift = 0,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia2 Output Channel2",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{.shift = MSM_FRONTEND_DAI_MULTIMEDIA2, .rshift = 1,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia2 Output Channel3",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{.shift = MSM_FRONTEND_DAI_MULTIMEDIA2, .rshift = 2,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia3 Output Channel1",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{.shift = MSM_FRONTEND_DAI_MULTIMEDIA3, .rshift = 0,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia3 Output Channel2",
	.info = msm_pcm_channel_weight_info,
	.get = msm_pcm_channel_weight_get,
	.put = msm_pcm_channel_weight_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{.shift = MSM_FRONTEND_DAI_MULTIMEDIA3, .rshift = 1,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Channel1",
	.info = msm_pcm_channel_input_be_info,
	.get = msm_pcm_channel_input_be_get,
	.put = msm_pcm_channel_input_be_put,
	.private_value = (unsigned long)&(mm1_ch1_enum)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Channel2",
	.info = msm_pcm_channel_input_be_info,
	.get = msm_pcm_channel_input_be_get,
	.put = msm_pcm_channel_input_be_put,
	.private_value = (unsigned long)&(mm1_ch2_enum)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Channel3",
	.info = msm_pcm_channel_input_be_info,
	.get = msm_pcm_channel_input_be_get,
	.put = msm_pcm_channel_input_be_put,
	.private_value = (unsigned long)&(mm1_ch3_enum)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Channel4",
	.info = msm_pcm_channel_input_be_info,
	.get = msm_pcm_channel_input_be_get,
	.put = msm_pcm_channel_input_be_put,
	.private_value = (unsigned long)&(mm1_ch4_enum)
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia1 Output Channel Map",
	.info = msm_pcm_channel_output_map_info,
	.get = msm_pcm_channel_output_map_get,
	.put = msm_pcm_channel_output_map_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA1,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia2 Output Channel Map",
	.info = msm_pcm_channel_output_map_info,
	.get = msm_pcm_channel_output_map_get,
	.put = msm_pcm_channel_output_map_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA2,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia3 Output Channel Map",
	.info = msm_pcm_channel_output_map_info,
	.get = msm_pcm_channel_output_map_get,
	.put = msm_pcm_channel_output_map_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA3,}
	},
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.name = "MultiMedia4 Output Channel Map",
	.info = msm_pcm_channel_output_map_info,
	.get = msm_pcm_channel_output_map_get,
	.put = msm_pcm_channel_output_map_put,
	.private_value = (unsigned long)&(struct soc_multi_mixer_control)
		{ .shift = MSM_FRONTEND_DAI_MULTIMEDIA4,}
	},
};

static int msm_ec_ref_ch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = msm_ec_ref_ch;
	pr_debug("%s: msm_ec_ref_ch = %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_ec_ref_ch_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	msm_ec_ref_ch = ucontrol->value.integer.value[0];
	pr_debug("%s: msm_ec_ref_ch = %d\n", __func__, msm_ec_ref_ch);
	adm_num_ec_ref_rx_chans(msm_ec_ref_ch);
	return 0;
}

static int msm_ec_ref_ch_downmixed_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = msm_ec_ref_ch_downmixed;
	pr_debug("%s: msm_ec_ref_downmixed_ch = %ld\n", __func__,
		ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_ec_ref_ch_downmixed_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	msm_ec_ref_ch_downmixed = ucontrol->value.integer.value[0];
	pr_debug("%s: msm_ec_ref_downmixed_ch = %d\n",
		__func__, msm_ec_ref_ch_downmixed);
	adm_num_ec_ref_rx_chans_downmixed(msm_ec_ref_ch_downmixed);
	return 0;
}

static int msm_ec_ref_chmixer_weights_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int i = 0, ret = 0;
	uint16_t weights[PCM_FORMAT_MAX_NUM_CHANNEL_V8] = {0};
	int out_channel_idx = ((struct soc_multi_mixer_control *)
				kcontrol->private_value)->shift;

	for (; i < PCM_FORMAT_MAX_NUM_CHANNEL_V8; i++)
		weights[i] = ucontrol->value.integer.value[i];

	ret = adm_ec_ref_chmixer_weights(out_channel_idx,
					weights, PCM_FORMAT_MAX_NUM_CHANNEL_V8);
	pr_debug("%s: ch_index = %d, ret = %d\n", __func__, out_channel_idx, ret);
	return ret;
}

static const char *const ec_ref_ch_text[] = {"Zero", "One", "Two", "Three",
	"Four", "Five", "Six", "Seven", "Eight", "Nine", "Ten", "Eleven",
	"Twelve", "Thirteen", "Fourteen", "Fifteen", "Sixteen"};

static int msm_ec_ref_bit_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	switch (msm_ec_ref_bit_format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 2;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		ucontrol->value.integer.value[0] = 1;
		break;
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}
	pr_debug("%s: msm_ec_ref_bit_format = %ld\n",
		 __func__, ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_ec_ref_bit_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u16 bit_width = 0;

	switch (ucontrol->value.integer.value[0]) {
	case 2:
		msm_ec_ref_bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 1:
		msm_ec_ref_bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	default:
		msm_ec_ref_bit_format = 0;
		break;
	}

	if (msm_ec_ref_bit_format == SNDRV_PCM_FORMAT_S16_LE)
		bit_width = 16;
	else if (msm_ec_ref_bit_format == SNDRV_PCM_FORMAT_S24_LE)
		bit_width = 24;

	pr_debug("%s: msm_ec_ref_bit_format = %d\n",
		 __func__, msm_ec_ref_bit_format);
	adm_ec_ref_rx_bit_width(bit_width);
	return 0;
}

static char const *ec_ref_bit_format_text[] = {"0", "S16_LE", "S24_LE"};

static int msm_ec_ref_rate_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = msm_ec_ref_sampling_rate;
	pr_debug("%s: msm_ec_ref_sampling_rate = %ld\n",
		 __func__, ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_ec_ref_rate_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		msm_ec_ref_sampling_rate = 0;
		break;
	case 1:
		msm_ec_ref_sampling_rate = 8000;
		break;
	case 2:
		msm_ec_ref_sampling_rate = 16000;
		break;
	case 3:
		msm_ec_ref_sampling_rate = 32000;
		break;
	case 4:
		msm_ec_ref_sampling_rate = 44100;
		break;
	case 5:
		msm_ec_ref_sampling_rate = 48000;
		break;
	case 6:
		msm_ec_ref_sampling_rate = 96000;
		break;
	case 7:
		msm_ec_ref_sampling_rate = 192000;
		break;
	case 8:
		msm_ec_ref_sampling_rate = 384000;
		break;
	default:
		msm_ec_ref_sampling_rate = 48000;
		break;
	}
	pr_debug("%s: msm_ec_ref_sampling_rate = %d\n",
		 __func__, msm_ec_ref_sampling_rate);
	adm_ec_ref_rx_sampling_rate(msm_ec_ref_sampling_rate);
	return 0;
}

static int get_ec_ref_port_id(int value, int *index)
{
	int port_id;

	switch (value) {
	case 0:
		*index = 0;
		port_id = AFE_PORT_INVALID;
		break;
	case 1:
		*index = 1;
		port_id = SLIMBUS_0_RX;
		break;
	case 2:
		*index = 2;
		port_id = AFE_PORT_ID_PRIMARY_MI2S_RX;
		break;
	case 3:
		*index = 3;
		port_id = AFE_PORT_ID_PRIMARY_MI2S_TX;
		break;
	case 4:
		*index = 4;
		port_id = AFE_PORT_ID_SECONDARY_MI2S_TX;
		break;
	case 5:
		*index = 5;
		port_id = AFE_PORT_ID_TERTIARY_MI2S_TX;
		break;
	case 6:
		*index = 6;
		port_id = AFE_PORT_ID_QUATERNARY_MI2S_TX;
		break;
	case 7:
		*index = 7;
		port_id = AFE_PORT_ID_SECONDARY_MI2S_RX;
		break;
	case 9:
		*index = 9;
		port_id = SLIMBUS_5_RX;
		break;
	case 10:
		*index = 10;
		port_id = SLIMBUS_1_TX;
		break;
	case 11:
		*index = 11;
		port_id = AFE_PORT_ID_QUATERNARY_TDM_TX_1;
		break;
	case 12:
		*index = 12;
		port_id = AFE_PORT_ID_QUATERNARY_TDM_RX;
		break;
	case 13:
		*index = 13;
		port_id = AFE_PORT_ID_QUATERNARY_TDM_RX_1;
		break;
	case 14:
		*index = 14;
		port_id = AFE_PORT_ID_QUATERNARY_TDM_RX_2;
		break;
	case 15:
		*index = 15;
		port_id = SLIMBUS_6_RX;
		break;
	case 16:
		*index = 16;
		port_id = AFE_PORT_ID_TERTIARY_MI2S_RX;
		break;
	case 17:
		*index = 17;
		port_id = AFE_PORT_ID_QUATERNARY_MI2S_RX;
		break;
	case 18:
		*index = 18;
		port_id = AFE_PORT_ID_TERTIARY_TDM_TX;
		break;
	case 19:
		*index = 19;
		port_id = AFE_PORT_ID_USB_RX;
		break;
	case 20:
		*index = 20;
		port_id = AFE_PORT_ID_INT0_MI2S_RX;
		break;
	case 21:
		*index = 21;
		port_id = AFE_PORT_ID_INT4_MI2S_RX;
		break;
	case 22:
		*index = 22;
		port_id = AFE_PORT_ID_INT3_MI2S_TX;
		break;
	case 23:
		*index = 23;
		port_id = AFE_PORT_ID_HDMI_OVER_DP_RX;
		break;
	case 24:
		*index = 24;
		port_id = AFE_PORT_ID_WSA_CODEC_DMA_RX_0;
		break;
	case 25:
		*index = 25;
		port_id = AFE_PORT_ID_WSA_CODEC_DMA_RX_1;
		break;
	case 26:
		*index = 26;
		port_id = AFE_PORT_ID_WSA_CODEC_DMA_TX_0;
		break;
	case 27:
		*index = 27;
		port_id = AFE_PORT_ID_WSA_CODEC_DMA_TX_1;
		break;
	case 28:
		*index = 28;
		port_id = AFE_PORT_ID_WSA_CODEC_DMA_TX_2;
		break;
	case 29:
		*index = 29;
		port_id = SLIMBUS_7_RX;
		break;
	case 30:
		*index = 30;
		port_id = AFE_PORT_ID_RX_CODEC_DMA_RX_0;
		break;
	case 31:
		*index = 31;
		port_id = AFE_PORT_ID_RX_CODEC_DMA_RX_1;
		break;
	case 32:
		*index = 32;
		port_id = AFE_PORT_ID_RX_CODEC_DMA_RX_2;
		break;
	case 33:
		*index = 33;
		port_id = AFE_PORT_ID_RX_CODEC_DMA_RX_3;
		break;
	case 34:
		*index = 34;
		port_id = AFE_PORT_ID_TX_CODEC_DMA_TX_0;
		break;
	case 35:
		*index = 35;
		port_id = AFE_PORT_ID_TERTIARY_TDM_RX_2;
		break;
	case 36:
		*index = 36;
		port_id = AFE_PORT_ID_SECONDARY_TDM_TX;
		break;
	case 37:
		*index = 37;
		port_id = AFE_PORT_ID_HDMI_OVER_DP_RX;
		break;
	case 38:
		*index = 38;
		port_id = AFE_PORT_ID_SENARY_MI2S_RX;
		break;
	case 39:
		*index = 39;
		port_id = AFE_PORT_ID_QUINARY_TDM_TX;
		break;
	case 40:
		*index = 40;
		port_id = AFE_PORT_ID_SENARY_MI2S_TX;
		break;
	case 41:
		*index = 41;
		port_id = AFE_PORT_ID_PRIMARY_TDM_RX;
		break;
	case 42:
		*index = 42;
		port_id = AFE_PORT_ID_PRIMARY_TDM_TX;
		break;
	default:
		*index = 0; /* NONE */
		pr_err("%s: Invalid value %d\n", __func__, value);
		port_id = AFE_PORT_INVALID;
		break;
	}

	return port_id;
}

static int msm_routing_afe_lb_tx_port_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: port index = %d", __func__, afe_loopback_tx_port_index);
	mutex_lock(&routing_lock);
	ucontrol->value.integer.value[0] = afe_loopback_tx_port_index;
	mutex_unlock(&routing_lock);

	return 0;
}

static int msm_routing_afe_lb_tx_port_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];

	mutex_lock(&routing_lock);
	afe_loopback_tx_port_id = get_ec_ref_port_id(value,
			&afe_loopback_tx_port_index);
	pr_debug("%s: afe_loopback_tx_port_index = %d\n",
	    __func__, afe_loopback_tx_port_index);
	mutex_unlock(&routing_lock);

	return 0;
}

static const char *const ec_ref_rate_text[] = {"0", "8000", "16000",
	"32000", "44100", "48000", "96000", "192000", "384000"};

static const struct soc_enum msm_route_ec_ref_params_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ec_ref_ch_text), ec_ref_ch_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ec_ref_bit_format_text),
				ec_ref_bit_format_text),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ec_ref_rate_text), ec_ref_rate_text),
};

static const char *const ec_ref_rx[] = { "None", "SLIM_RX", "I2S_RX",
	"PRI_MI2S_TX", "SEC_MI2S_TX",
	"TERT_MI2S_TX", "QUAT_MI2S_TX", "SEC_I2S_RX", "PROXY_RX",
	"SLIM_5_RX", "SLIM_1_TX", "QUAT_TDM_TX_1",
	"QUAT_TDM_RX_0", "QUAT_TDM_RX_1", "QUAT_TDM_RX_2", "SLIM_6_RX",
	"TERT_MI2S_RX", "QUAT_MI2S_RX", "TERT_TDM_TX_0", "USB_AUDIO_RX",
	"INT0_MI2S_RX", "INT4_MI2S_RX", "INT3_MI2S_TX", "DISPLAY_PORT",
	"WSA_CDC_DMA_RX_0", "WSA_CDC_DMA_RX_1",
	"WSA_CDC_DMA_TX_0", "WSA_CDC_DMA_TX_1", "WSA_CDC_DMA_TX_2",
	"SLIM_7_RX", "RX_CDC_DMA_RX_0", "RX_CDC_DMA_RX_1", "RX_CDC_DMA_RX_2",
	"RX_CDC_DMA_RX_3", "TX_CDC_DMA_TX_0", "TERT_TDM_RX_2", "SEC_TDM_TX_0",
	"DISPLAY_PORT1", "SEN_MI2S_RX", "QUIN_TDM_TX_0", "SENARY_MI2S_TX",
	"PRI_TDM_RX_0", "PRI_TDM_TX_0",
};

static const struct soc_enum msm_route_ec_ref_rx_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ec_ref_rx), ec_ref_rx),
};

static const struct snd_kcontrol_new ec_ref_param_controls[] = {
	SOC_ENUM_EXT("EC Reference Channels", msm_route_ec_ref_params_enum[0],
		msm_ec_ref_ch_get, msm_ec_ref_ch_put),
	SOC_ENUM_EXT("EC Reference Bit Format", msm_route_ec_ref_params_enum[1],
		msm_ec_ref_bit_format_get, msm_ec_ref_bit_format_put),
	SOC_ENUM_EXT("EC Reference SampleRate", msm_route_ec_ref_params_enum[2],
		msm_ec_ref_rate_get, msm_ec_ref_rate_put),
	SOC_ENUM_EXT("EC Reference Downmixed Channels",
		msm_route_ec_ref_params_enum[0],
		msm_ec_ref_ch_downmixed_get, msm_ec_ref_ch_downmixed_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch1", SND_SOC_NOPM,
		0, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch2", SND_SOC_NOPM,
		1, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch3", SND_SOC_NOPM,
		2, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch4", SND_SOC_NOPM,
		3, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch5", SND_SOC_NOPM,
		4, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch6", SND_SOC_NOPM,
		5, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch7", SND_SOC_NOPM,
		6, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_SINGLE_MULTI_EXT("EC Reference ChMixer Weights Ch8", SND_SOC_NOPM,
		7, 16384, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8, NULL,
		msm_ec_ref_chmixer_weights_put),
	SOC_ENUM_EXT("AFE_LOOPBACK_TX Port", msm_route_ec_ref_rx_enum[0],
		msm_routing_afe_lb_tx_port_get, msm_routing_afe_lb_tx_port_put),
};

static int msm_routing_ec_ref_rx_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: ec_ref_rx  = %d", __func__, msm_route_ec_ref_rx);
	mutex_lock(&routing_lock);
	ucontrol->value.integer.value[0] = msm_route_ec_ref_rx;
	mutex_unlock(&routing_lock);

	return 0;
}

static int msm_routing_ec_ref_rx_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;

	mutex_lock(&routing_lock);
	msm_ec_ref_port_id = get_ec_ref_port_id(value, &msm_route_ec_ref_rx);
	adm_ec_ref_rx_id(msm_ec_ref_port_id);
	pr_debug("%s: msm_route_ec_ref_rx = %d\n",
	    __func__, msm_route_ec_ref_rx);
	mutex_unlock(&routing_lock);

	snd_soc_dapm_mux_update_power(widget->dapm, kcontrol,
					msm_route_ec_ref_rx, e, update);
	return 0;
}

static const struct snd_kcontrol_new ext_ec_ref_mux_ul1 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL1 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul2 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL2 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul3 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL3 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul4 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL4 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul5 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL5 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul6 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL6 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul8 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL8 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul9 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL9 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul16 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL16 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul10 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL10 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul17 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL17 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul18 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL18 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul19 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL19 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul28 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL28 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul29 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL29 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);

static const struct snd_kcontrol_new ext_ec_ref_mux_ul30 =
	SOC_DAPM_ENUM_EXT("AUDIO_REF_EC_UL30 MUX Mux",
		msm_route_ec_ref_rx_enum[0],
		msm_routing_ec_ref_rx_get, msm_routing_ec_ref_rx_put);


static int msm_routing_ext_ec_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: ext_ec_ref_rx  = %x\n", __func__, msm_route_ext_ec_ref);

	mutex_lock(&routing_lock);
	ucontrol->value.integer.value[0] = msm_route_ext_ec_ref;
	mutex_unlock(&routing_lock);
	return 0;
}

static int msm_routing_ext_ec_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget =
		snd_soc_dapm_kcontrol_widget(kcontrol);
	int mux = ucontrol->value.enumerated.item[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	int ret = 1;
	bool state = true;
	uint16_t ext_ec_ref_port_id;
	struct snd_soc_dapm_update *update = NULL;

	if (mux >= e->items) {
		pr_err("%s: Invalid mux value %d\n", __func__, mux);
		return -EINVAL;
	}

	mutex_lock(&routing_lock);
	msm_route_ext_ec_ref = ucontrol->value.integer.value[0];

	switch (msm_route_ext_ec_ref) {
	case EXT_EC_REF_PRI_MI2S_TX:
		ext_ec_ref_port_id = AFE_PORT_ID_PRIMARY_MI2S_TX;
		break;
	case EXT_EC_REF_SEC_MI2S_TX:
		ext_ec_ref_port_id = AFE_PORT_ID_SECONDARY_MI2S_TX;
		break;
	case EXT_EC_REF_PRI_TDM_TX:
		ext_ec_ref_port_id = AFE_PORT_ID_PRIMARY_TDM_TX;
		break;
	case EXT_EC_REF_SEC_TDM_TX:
		ext_ec_ref_port_id = AFE_PORT_ID_SECONDARY_TDM_TX;
		break;
	case EXT_EC_REF_NONE:
	default:
		ext_ec_ref_port_id = AFE_PORT_INVALID;
		state = false;
		break;
	}

	pr_debug("%s: val = %d ext_ec_ref_port_id = 0x%0x state = %d\n",
		 __func__, msm_route_ext_ec_ref, ext_ec_ref_port_id, state);

	if (!voc_set_ext_ec_ref_port_id(ext_ec_ref_port_id, state)) {
		mutex_unlock(&routing_lock);
		snd_soc_dapm_mux_update_power(widget->dapm, kcontrol, mux, e,
						update);
	} else {
		ret = -EINVAL;
		mutex_unlock(&routing_lock);
	}
	return ret;
}

static const char * const ext_ec_ref_rx[] = {"NONE", "PRI_MI2S_TX",
					"SEC_MI2S_TX", "PRI_TDM_TX",
					"SEC_TDM_TX"};

static const struct soc_enum msm_route_ext_ec_ref_rx_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ext_ec_ref_rx), ext_ec_ref_rx),
};

static const struct snd_kcontrol_new voc_ext_ec_mux =
	SOC_DAPM_ENUM_EXT("VOC_EXT_EC MUX Mux", msm_route_ext_ec_ref_rx_enum[0],
			  msm_routing_ext_ec_get, msm_routing_ext_ec_put);


static const struct snd_kcontrol_new secondary_mi2s_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new primary_mi2s_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};


static const struct snd_kcontrol_new primary_mi2s_rx_port_mixer_controls[] = {
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
};


static const struct snd_kcontrol_new sec_mi2s_rx_port_mixer_controls[] = {
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_BACKEND_DAI_AUXPCM_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new sec_mi2s_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

#ifndef CONFIG_MI2S_DISABLE
static const struct snd_kcontrol_new pri_mi2s_rx_switch_mixer_controls =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM,
	0, 1, 0, msm_routing_get_pri_mi2s_switch_mixer,
	msm_routing_put_pri_mi2s_switch_mixer);

static const struct snd_kcontrol_new sec_mi2s_rx_switch_mixer_controls =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM,
	0, 1, 0, msm_routing_get_sec_mi2s_switch_mixer,
	msm_routing_put_sec_mi2s_switch_mixer);
#endif

	/* incall music delivery mixer */
static const struct snd_kcontrol_new incall_music_delivery_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new incall_music2_delivery_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new usb_audio_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new afe_pcm_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

#ifndef CONFIG_AUXPCM_DISABLE
static const struct snd_kcontrol_new auxpcm_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_auxpcm_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

#endif

#ifndef CONFIG_TDM_DISABLE
static const struct snd_kcontrol_new pri_tdm_rx_0_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_1_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_2_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_3_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new pri_tdm_tx_0_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_0_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_1_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_2_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_3_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_tdm_tx_0_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new sec_tdm_tx_1_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};




static const struct snd_kcontrol_new pri_tdm_rx_0_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("VoLTE Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_1_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("VoLTE Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_2_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("VoLTE Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_2,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_3_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("VoLTE Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_RX_3,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_0_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoLTE Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_1_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoLTE Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_RX_1,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_0_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_PRI_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_PRI_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_PRI_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_0,
		MSM_BACKEND_DAI_PRI_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_1_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_PRI_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_PRI_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_PRI_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_1,
		MSM_BACKEND_DAI_PRI_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_2_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_PRI_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_PRI_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_PRI_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_2,
		MSM_BACKEND_DAI_PRI_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new pri_tdm_rx_3_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_PRI_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_PRI_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_PRI_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_RX_3,
		MSM_BACKEND_DAI_PRI_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_0_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_SEC_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_SEC_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_SEC_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_0,
		MSM_BACKEND_DAI_SEC_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_1_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_SEC_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_SEC_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_SEC_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_1,
		MSM_BACKEND_DAI_SEC_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_2_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_SEC_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_SEC_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_SEC_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_2,
		MSM_BACKEND_DAI_SEC_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new sec_tdm_rx_3_port_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_AFE_PCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
#endif
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_SEC_TDM_TX_0, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_SEC_TDM_TX_1, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_SEC_TDM_TX_2, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_RX_3,
		MSM_BACKEND_DAI_SEC_TDM_TX_3, 1, 0,
		msm_routing_get_port_mixer,
		msm_routing_put_port_mixer),
};

#endif

static const struct snd_kcontrol_new mmul1_mixer_controls[] = {
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_MI2S_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
#endif
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_AUXPCM_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_AUXPCM_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_AFE_PCM_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_DL", SND_SOC_NOPM,
		MSM_BACKEND_DAI_INCALL_RECORD_RX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_UL", SND_SOC_NOPM,
		MSM_BACKEND_DAI_INCALL_RECORD_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
#ifndef CONFIG_TDM_DISABLE
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_TX_0,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_TX_1,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_TX_2,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_PRI_TDM_TX_3,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_TX_0,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_TX_1,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_TX_2,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SEC_TDM_TX_3,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
#endif
	SOC_DOUBLE_EXT("USB_AUDIO_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_USB_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("WSA_CDC_DMA_TX_0", SND_SOC_NOPM,
			MSM_BACKEND_DAI_WSA_CDC_DMA_TX_0,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("WSA_CDC_DMA_TX_1", SND_SOC_NOPM,
			MSM_BACKEND_DAI_WSA_CDC_DMA_TX_1,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("WSA_CDC_DMA_TX_2", SND_SOC_NOPM,
			MSM_BACKEND_DAI_WSA_CDC_DMA_TX_2,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_0", SND_SOC_NOPM,
			MSM_BACKEND_DAI_VA_CDC_DMA_TX_0,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_1", SND_SOC_NOPM,
			MSM_BACKEND_DAI_VA_CDC_DMA_TX_1,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_2", SND_SOC_NOPM,
			MSM_BACKEND_DAI_VA_CDC_DMA_TX_2,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_0", SND_SOC_NOPM,
		MSM_BACKEND_DAI_TX_CDC_DMA_TX_0,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_1", SND_SOC_NOPM,
		MSM_BACKEND_DAI_TX_CDC_DMA_TX_1,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_2", SND_SOC_NOPM,
		MSM_BACKEND_DAI_TX_CDC_DMA_TX_2,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_3", SND_SOC_NOPM,
		MSM_BACKEND_DAI_TX_CDC_DMA_TX_3,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_4", SND_SOC_NOPM,
		MSM_BACKEND_DAI_TX_CDC_DMA_TX_4,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_5", SND_SOC_NOPM,
		MSM_BACKEND_DAI_TX_CDC_DMA_TX_5,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SLIM_9_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_SLIMBUS_9_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("AFE_LOOPBACK_TX", SND_SOC_NOPM,
		MSM_BACKEND_DAI_AFE_LOOPBACK_TX,
		MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
		msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new mmul2_mixer_controls[] = {
	SOC_DOUBLE_EXT("VOC_REC_DL", SND_SOC_NOPM,
	MSM_BACKEND_DAI_INCALL_RECORD_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_UL", SND_SOC_NOPM,
	MSM_BACKEND_DAI_INCALL_RECORD_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#ifndef CONFIG_TDM_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
#ifndef CONFIG_TDM_DISABLE
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
	SOC_DOUBLE_EXT("USB_AUDIO_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("AFE_LOOPBACK_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_LOOPBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("SEC_AUX_PCM_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
};

static const struct snd_kcontrol_new mmul3_mixer_controls[] = {
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_DL", SND_SOC_NOPM,
	MSM_BACKEND_DAI_INCALL_RECORD_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_UL", SND_SOC_NOPM,
	MSM_BACKEND_DAI_INCALL_RECORD_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
#ifndef CONFIG_TDM_DISABLE
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_LOOPBACK_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_LOOPBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new mmul4_mixer_controls[] = {
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_DL", SND_SOC_NOPM,
	MSM_BACKEND_DAI_INCALL_RECORD_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("VOC_REC_UL", SND_SOC_NOPM,
	MSM_BACKEND_DAI_INCALL_RECORD_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#ifndef CONFIG_TDM_DISABLE
	SOC_DOUBLE_EXT("PRI_TDM_TX_0", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_0", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
#ifndef CONFIG_MI2S_DISABLE
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
#endif
	SOC_DOUBLE_EXT("USB_AUDIO_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("AFE_LOOPBACK_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_LOOPBACK_TX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new usb_audio_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new wsa_cdc_dma_rx_0_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new wsa_cdc_dma_rx_1_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_WSA_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_0_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_1_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_2_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_2,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_3_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_3,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_4_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_4,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_4,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_4,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_4,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_5_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_5,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_5,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_5,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_5,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_6_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_6,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_6,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_6,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_6,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_7_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_7,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_7,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_7,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_7,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

static const struct snd_kcontrol_new slimbus_7_rx_mixer_controls[] = {
	SOC_DOUBLE_EXT("MultiMedia1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SLIMBUS_7_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA1, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SLIMBUS_7_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA2, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SLIMBUS_7_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA3, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
	SOC_DOUBLE_EXT("MultiMedia4", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SLIMBUS_7_RX,
	MSM_FRONTEND_DAI_MULTIMEDIA4, 1, 0, msm_routing_get_audio_mixer,
	msm_routing_put_audio_mixer),
};

#ifndef CONFIG_MI2S_DISABLE

static const struct snd_kcontrol_new pri_mi2s_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("Voice2 Stub", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};
#endif

static const struct snd_kcontrol_new afe_pcm_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new aux_pcm_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new sec_aux_pcm_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QCHAT", SND_SOC_NOPM,
MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};


static const struct snd_kcontrol_new proxy_rx_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PROXY_RX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PROXY_RX,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new wsa_cdc_dma_rx_0_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
			MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
			MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
			MSM_BACKEND_DAI_WSA_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_0_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("DTMF", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_DTMF_RX, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_1_voice_mixer_controls[] = {
	SOC_DOUBLE_EXT("Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VoiceMMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_1,
	MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new tx_voicemmode1_mixer_controls[] = {
	SOC_DOUBLE_EXT("AFE_PCM_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1,
	0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TERT_AUX_PCM_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TERT_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1,
	0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QUAT_AUX_PCM_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_QUAT_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1,
	0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_MI2S_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TERT_MI2S_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TERTIARY_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QUAT_MI2S_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_QUATERNARY_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("USB_AUDIO_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_TX,
	MSM_FRONTEND_DAI_VOICEMMODE1, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PROXY_TX_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PROXY_TX, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_0_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_0, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_1_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_1, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_2_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_2, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_0_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_0, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_1_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_1, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_2_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_2, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_3_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_3, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_4_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_4, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_5_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_5, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_0_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_0_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1, MSM_FRONTEND_DAI_VOICEMMODE1,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new tx_voicemmode2_mixer_controls[] = {
	SOC_DOUBLE_EXT("AFE_PCM_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1,
	0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TERT_AUX_PCM_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TERT_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1,
	0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QUAT_AUX_PCM_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_QUAT_AUXPCM_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1,
	0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_MI2S_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TERT_MI2S_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TERTIARY_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("QUAT_MI2S_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_QUATERNARY_MI2S_TX, MSM_FRONTEND_DAI_VOICEMMODE2, 1, 0,
	msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PROXY_TX_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PROXY_TX, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_0_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_0, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_1_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_1, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_2_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_2, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_0_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_0, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_1_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_1, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_2_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_2, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_3_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_3, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_4_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_4, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_5_MMode2", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_5, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_0_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_0, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PRI_TDM_TX_1_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_0_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_TDM_TX_0, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_TDM_TX_1_MMode1", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_TDM_TX_1, MSM_FRONTEND_DAI_VOICEMMODE2,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new tx_voip_mixer_controls[] = {
	SOC_DOUBLE_EXT("AFE_PCM_TX_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_TX_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_TX_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
#endif
	SOC_DOUBLE_EXT("PRI_MI2S_TX_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("USB_AUDIO_TX_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_TX,
	MSM_FRONTEND_DAI_VOIP, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_0_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_0, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_1_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_1, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("VA_CDC_DMA_TX_2_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_VA_CDC_DMA_TX_2, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_0_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_0, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_1_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_1, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_2_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_2, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_3_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_3, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_4_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_4, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_5_Voip", SND_SOC_NOPM,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_5, MSM_FRONTEND_DAI_VOIP,
	1, 0, msm_routing_get_voice_mixer, msm_routing_put_voice_mixer),
};

static const struct snd_kcontrol_new tx_voice_stub_mixer_controls[] = {
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOICE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
#endif
};

static const struct snd_kcontrol_new tx_voice2_stub_mixer_controls[] = {
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
#endif
	SOC_DOUBLE_EXT("AFE_PCM_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_VOICE2_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
};

static const struct snd_kcontrol_new tx_volte_stub_mixer_controls[] = {
#ifndef CONFIG_AUXPCM_DISABLE
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_VOLTE_STUB, 1, 0, msm_routing_get_voice_stub_mixer,
	msm_routing_put_voice_stub_mixer),
#endif
};

static const struct snd_kcontrol_new tx_qchat_mixer_controls[] = {
	SOC_DOUBLE_EXT("AFE_PCM_TX_QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AFE_PCM_TX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("AUX_PCM_TX_QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_TX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_TX_QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("PRI_MI2S_TX_QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_PRI_MI2S_TX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
	SOC_DOUBLE_EXT("USB_AUDIO_TX_QCHAT", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_TX,
	MSM_FRONTEND_DAI_QCHAT, 1, 0, msm_routing_get_voice_mixer,
	msm_routing_put_voice_mixer),
};


#ifndef CONFIG_AUXPCM_DISABLE
static const struct snd_kcontrol_new aux_pcm_rx_port_mixer_controls[] = {
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_BACKEND_DAI_AUXPCM_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("SEC_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_AUXPCM_RX,
	MSM_BACKEND_DAI_SECONDARY_MI2S_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new sec_auxpcm_rx_port_mixer_controls[] = {
	SOC_DOUBLE_EXT("SEC_AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_BACKEND_DAI_SEC_AUXPCM_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("AUX_PCM_UL_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_BACKEND_DAI_AUXPCM_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
	SOC_DOUBLE_EXT("PRI_MI2S_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_SEC_AUXPCM_RX,
	MSM_BACKEND_DAI_PRI_MI2S_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
};
#endif

static const struct snd_kcontrol_new usb_rx_port_mixer_controls[] = {
	SOC_DOUBLE_EXT("USB_AUDIO_TX", SND_SOC_NOPM,
	MSM_BACKEND_DAI_USB_RX,
	MSM_BACKEND_DAI_USB_TX, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),
};

static const struct snd_kcontrol_new rx_cdc_dma_rx_0_port_mixer_controls[] = {
	SOC_DOUBLE_EXT("TX_CDC_DMA_TX_3", SND_SOC_NOPM,
	MSM_BACKEND_DAI_RX_CDC_DMA_RX_0,
	MSM_BACKEND_DAI_TX_CDC_DMA_TX_3, 1, 0, msm_routing_get_port_mixer,
	msm_routing_put_port_mixer),

};

static const struct snd_kcontrol_new usb_switch_mixer_controls =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM,
	0, 1, 0, msm_routing_get_usb_switch_mixer,
	msm_routing_put_usb_switch_mixer);

static const struct snd_kcontrol_new cdc_dma_rx_switch_mixer_controls =
	SOC_SINGLE_EXT("Switch", SND_SOC_NOPM,
	0, 1, 0, msm_routing_get_switch_mixer,
	msm_routing_put_switch_mixer);

static int msm_routing_get_stereo_to_custom_stereo_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = is_custom_stereo_on;
	return 0;
}

static int msm_routing_put_stereo_to_custom_stereo_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int flag = 0, i = 0, rc = 0, idx = 0;
	int be_index = 0, port_id, topo_id;
	unsigned int session_id = 0;
	uint16_t op_FL_ip_FL_weight = 0;
	uint16_t op_FL_ip_FR_weight = 0;
	uint16_t op_FR_ip_FL_weight = 0;
	uint16_t op_FR_ip_FR_weight = 0;

	flag = ucontrol->value.integer.value[0];
	pr_debug("%s E flag %d\n", __func__, flag);

	if ((is_custom_stereo_on && flag) || (!is_custom_stereo_on && !flag)) {
		pr_err("%s: is_custom_stereo_on %d, flag %d\n",
			__func__, is_custom_stereo_on, flag);
		return 0;
	}
	is_custom_stereo_on = flag ? true : false;
	pr_debug("%s:is_custom_stereo_on %d\n", __func__, is_custom_stereo_on);
	for (be_index = 0; be_index < MSM_BACKEND_DAI_MAX; be_index++) {
		port_id = msm_bedais[be_index].port_id;
		if (!msm_bedais[be_index].active)
			continue;
		if ((port_id != SLIMBUS_0_RX) &&
		     (port_id != RT_PROXY_PORT_001_RX) &&
			(port_id != AFE_PORT_ID_PRIMARY_MI2S_RX) &&
			(port_id != AFE_PORT_ID_INT4_MI2S_RX))
			continue;

		for_each_set_bit(i, &msm_bedais[be_index].fe_sessions[0],
				MSM_FRONTEND_DAI_MM_SIZE) {
			if (fe_dai_map[i][SESSION_TYPE_RX].perf_mode !=
			    LEGACY_PCM_MODE)
				goto skip_send_custom_stereo;
			session_id =
				fe_dai_map[i][SESSION_TYPE_RX].strm_id;
			if (is_custom_stereo_on) {
				op_FL_ip_FL_weight =
					Q14_GAIN_ZERO_POINT_FIVE;
				op_FL_ip_FR_weight =
					Q14_GAIN_ZERO_POINT_FIVE;
				op_FR_ip_FL_weight =
					Q14_GAIN_ZERO_POINT_FIVE;
				op_FR_ip_FR_weight =
					Q14_GAIN_ZERO_POINT_FIVE;
			} else {
				op_FL_ip_FL_weight = Q14_GAIN_UNITY;
				op_FL_ip_FR_weight = 0;
				op_FR_ip_FL_weight = 0;
				op_FR_ip_FR_weight = Q14_GAIN_UNITY;
			}
			for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
				unsigned long copp =
					session_copp_map[i]
					[SESSION_TYPE_RX][be_index];
				if (!test_bit(idx, &copp))
					goto skip_send_custom_stereo;
				topo_id = adm_get_topology_for_port_copp_idx(
					msm_bedais[be_index].port_id, idx);
				if (topo_id < 0)
					pr_debug("%s:Err:custom stereo topo %d",
						 __func__, topo_id);
					pr_debug("idx %d\n", idx);
				if (topo_id == DS2_ADM_COPP_TOPOLOGY_ID)
					rc = msm_ds2_dap_set_custom_stereo_onoff
						(msm_bedais[be_index].port_id,
						idx, is_custom_stereo_on);
				else if (topo_id == DOLBY_ADM_COPP_TOPOLOGY_ID)
					rc = dolby_dap_set_custom_stereo_onoff(
						msm_bedais[be_index].port_id,
						idx, is_custom_stereo_on);
				else
				rc = msm_qti_pp_send_stereo_to_custom_stereo_cmd
						(msm_bedais[be_index].port_id,
						idx, session_id,
						op_FL_ip_FL_weight,
						op_FL_ip_FR_weight,
						op_FR_ip_FL_weight,
						op_FR_ip_FR_weight);
				if (rc < 0)
skip_send_custom_stereo:
					pr_err("%s: err setting custom stereo\n",
						__func__);
			}

		}
	}
	return 0;
}

static const struct snd_kcontrol_new stereo_to_custom_stereo_controls[] = {
	SOC_SINGLE_EXT("Set Custom Stereo OnOff", SND_SOC_NOPM, 0,
	1, 0, msm_routing_get_stereo_to_custom_stereo_control,
	msm_routing_put_stereo_to_custom_stereo_control),
};

static int msm_routing_get_app_type_cfg_control(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int msm_routing_put_app_type_cfg_control(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	int i = 0, j;
	int num_app_types = ucontrol->value.integer.value[i++];

	pr_debug("%s\n", __func__);

	memset(app_type_cfg, 0, MAX_APP_TYPES*
				sizeof(struct msm_pcm_routing_app_type_data));
	if (num_app_types > MAX_APP_TYPES || num_app_types < 0) {
		pr_err("%s: number of app types %d is invalid\n",
			__func__, num_app_types);
		return -EINVAL;
	}
	for (j = 0; j < num_app_types; j++) {
		app_type_cfg[j].app_type =
				ucontrol->value.integer.value[i++];
		app_type_cfg[j].sample_rate =
				ucontrol->value.integer.value[i++];
		app_type_cfg[j].bit_width =
				ucontrol->value.integer.value[i++];
	}

	return 0;
}

static int msm_routing_put_app_type_gain_control(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	int j, fe_id, be_id, port_type;
	int ret = 0;
	unsigned long copp;
	struct msm_pcm_routing_bdai_data *bedai;
	int dir = ucontrol->value.integer.value[0] ? SESSION_TYPE_TX :
						     SESSION_TYPE_RX;
	int app_type = ucontrol->value.integer.value[1];
	int gain = (ucontrol->value.integer.value[2] +
		    ucontrol->value.integer.value[3])/2;

	port_type = (dir == SESSION_TYPE_RX) ? MSM_AFE_PORT_TYPE_RX :
						MSM_AFE_PORT_TYPE_TX;

	mutex_lock(&routing_lock);
	for (be_id = 0; be_id < MSM_BACKEND_DAI_MAX; be_id++) {
		bedai = &msm_bedais[be_id];
		if (afe_get_port_type(bedai->port_id) != port_type)
			continue;

		if (!bedai->active)
			continue;

		for (fe_id = 0; fe_id < MSM_FRONTEND_DAI_MAX; fe_id++) {
			if (!test_bit(fe_id, &bedai->fe_sessions[0]))
				continue;

			if (app_type !=
			    fe_dai_app_type_cfg[fe_id][dir][be_id].app_type)
				continue;

			copp = session_copp_map[fe_id][dir][be_id];
			for (j = 0; j < MAX_COPPS_PER_PORT; j++) {
				if (!test_bit(j, &copp))
					continue;
				ret |= adm_set_volume(bedai->port_id, j, gain);
			}
		}
	}
	mutex_unlock(&routing_lock);
	return ret ? -EINVAL : 0;
}

static const struct snd_kcontrol_new app_type_cfg_controls[] = {
	SOC_SINGLE_MULTI_EXT("App Type Config", SND_SOC_NOPM, 0,
	0x7FFFFFFF, 0, 128, msm_routing_get_app_type_cfg_control,
	msm_routing_put_app_type_cfg_control),
	SOC_SINGLE_MULTI_EXT("App Type Gain", SND_SOC_NOPM, 0,
	0x2000, 0, 4, NULL, msm_routing_put_app_type_gain_control)
};

static int msm_routing_put_module_cfg_control(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	int copp_idx, fe_id, be_id, port_type;
	int ret = 0;
	unsigned long copp;
	struct msm_pcm_routing_bdai_data *bedai;
	u8 *packed_params = NULL;
	struct param_hdr_v3 param_hdr;
	u32 packed_param_size = (sizeof(struct param_hdr_v3) +
				 sizeof(uint32_t));

	int dir = ucontrol->value.integer.value[0] ? SESSION_TYPE_TX :
						     SESSION_TYPE_RX;
	int app_type = ucontrol->value.integer.value[1];
	int module_id = ucontrol->value.integer.value[2];
	int instance_id = ucontrol->value.integer.value[3];
	int param_id = ucontrol->value.integer.value[4];
	int param_value = ucontrol->value.integer.value[5];

	port_type = (dir == SESSION_TYPE_RX) ? MSM_AFE_PORT_TYPE_RX :
						MSM_AFE_PORT_TYPE_TX;
	pr_debug("%s app_type:%d mod_id:%d instance_id:%d param_id:%d value:%d\n",
		  __func__, app_type, module_id,
		  instance_id, param_id, param_value);

	packed_params = kzalloc(packed_param_size, GFP_KERNEL);
	if (!packed_params)
		return -ENOMEM;

	memset(&param_hdr, 0, sizeof(param_hdr));
	param_hdr.module_id = module_id;
	param_hdr.instance_id = instance_id;
	param_hdr.param_id = param_id;
	param_hdr.param_size = sizeof(uint32_t);

	packed_param_size = 0;

	mutex_lock(&routing_lock);
	for (be_id = 0; be_id < MSM_BACKEND_DAI_MAX; be_id++) {

		bedai = &msm_bedais[be_id];
		if (afe_get_port_type(bedai->port_id) != port_type)
			continue;

		if (!bedai->active)
			continue;

		for (fe_id = 0; fe_id < MSM_FRONTEND_DAI_MAX; fe_id++) {
			if (!test_bit(fe_id, &bedai->fe_sessions[0]))
				continue;

			if (app_type !=
			    fe_dai_app_type_cfg[fe_id][dir][be_id].app_type)
				continue;

			copp = session_copp_map[fe_id][dir][be_id];
			for (copp_idx = 0; copp_idx < MAX_COPPS_PER_PORT;
			     copp_idx++) {
				if (!test_bit(copp_idx, &copp))
					continue;

				ret = q6common_pack_pp_params(packed_params,
							&param_hdr,
							(u8 *) &param_value,
							&packed_param_size);
				if (ret) {
					pr_err("%s: Failed to pack params, error %d\n",
						__func__, ret);
					goto done;
				}

				ret = adm_set_pp_params(bedai->port_id,
							 copp_idx, NULL,
							 packed_params,
							 packed_param_size);
				if (ret) {
					pr_err("%s: Setting param failed with err=%d\n",
						__func__, ret);
					ret = -EINVAL;
					goto done;
				}
			}
		}
	}
done:
	mutex_unlock(&routing_lock);
	kfree(packed_params);
	return ret;
}

static const struct snd_kcontrol_new module_cfg_controls[] = {
	SOC_SINGLE_MULTI_EXT("Audio Effect", SND_SOC_NOPM, 0,
	0x2000, 0, 6, NULL, msm_routing_put_module_cfg_control)
};


static int msm_routing_get_hifi_filter_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hifi_filter_enabled;
	return 0;
}

static int msm_routing_put_hifi_filter_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	hifi_filter_enabled = ucontrol->value.integer.value[0];
	return 0;
}

static const struct snd_kcontrol_new hifi_filter_controls[] = {
	SOC_SINGLE_EXT("HiFi Filter", SND_SOC_NOPM, 0,
		1, 0, msm_routing_get_hifi_filter_control,
		msm_routing_put_hifi_filter_control),
};

static int msm_routing_get_ffecns_freeze_event_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ffecns_freeze_event;
	return 0;
}

static int msm_routing_put_ffecns_freeze_event_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;

	ffecns_freeze_event = ucontrol->value.integer.value[0];

	ret = adm_set_ffecns_freeze_event(ffecns_freeze_event);
	if (ret)
		pr_err("%s: failed to set ffecns imc event to%d\n",
			__func__, ffecns_freeze_event);

	return ret;
}

static const struct snd_kcontrol_new use_ffecns_freeze_event_controls[] = {
	SOC_SINGLE_EXT("FFECNS Freeze Event", SND_SOC_NOPM, 0,
	1, 0, msm_routing_get_ffecns_freeze_event_control,
	msm_routing_put_ffecns_freeze_event_control),
};

int msm_routing_get_rms_value_control(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol) {
	int rc = 0;
	int be_idx = 0;
	char *param_value;
	int *update_param_value;
	uint32_t param_size = (RMS_PAYLOAD_LEN + 1) * sizeof(uint32_t);
	struct param_hdr_v3 param_hdr;

	param_value = kzalloc(param_size, GFP_KERNEL);
	if (!param_value)
		return -ENOMEM;

	memset(&param_hdr, 0, sizeof(param_hdr));
	for (be_idx = 0; be_idx < MSM_BACKEND_DAI_MAX; be_idx++)
		if (msm_bedais[be_idx].port_id == SLIMBUS_0_TX)
			break;
	if ((be_idx < MSM_BACKEND_DAI_MAX) && msm_bedais[be_idx].active) {
		param_hdr.module_id = RMS_MODULEID_APPI_PASSTHRU;
		param_hdr.instance_id = INSTANCE_ID_0;
		param_hdr.param_id = RMS_PARAM_FIRST_SAMPLE;
		param_hdr.param_size = param_size;
		rc = adm_get_pp_params(SLIMBUS_0_TX, 0, ADM_CLIENT_ID_DEFAULT,
					NULL, &param_hdr, (u8 *) param_value);
		if (rc) {
			pr_err("%s: get parameters failed:%d\n", __func__, rc);
			kfree(param_value);
			return -EINVAL;
		}
		update_param_value = (int *)param_value;
		ucontrol->value.integer.value[0] = update_param_value[0];

		pr_debug("%s: FROM DSP value[0] 0x%x\n",
			  __func__, update_param_value[0]);
	}
	kfree(param_value);
	return 0;
}

static int msm_voc_session_id_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	voc_session_id = ucontrol->value.integer.value[0];

	pr_debug("%s: voc_session_id=%u\n", __func__, voc_session_id);

	return 0;
}

static int msm_voc_session_id_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = voc_session_id;

	return 0;
}

static struct snd_kcontrol_new msm_voc_session_controls[] = {
	SOC_SINGLE_MULTI_EXT("Voc VSID", SND_SOC_NOPM, 0,
			     0xFFFFFFFF, 0, 1, msm_voc_session_id_get,
			     msm_voc_session_id_put),
};

static int msm_sound_focus_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct sound_focus_param);

	return 0;
}

static int msm_voice_sound_focus_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct sound_focus_param soundFocusData;

	memcpy((void *)&soundFocusData, ucontrol->value.bytes.data,
		sizeof(struct sound_focus_param));
	ret = voc_set_sound_focus(soundFocusData);
	if (ret) {
		pr_err("%s: Error setting Sound Focus Params, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
	}

	return ret;
}

static int msm_voice_sound_focus_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct sound_focus_param soundFocusData;

	memset(&soundFocusData, 0, sizeof(struct sound_focus_param));

	ret = voc_get_sound_focus(&soundFocusData);
	if (ret) {
		pr_debug("%s: Error getting Sound Focus Params, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}
	memcpy(ucontrol->value.bytes.data, (void *)&soundFocusData,
		sizeof(struct sound_focus_param));

done:
	return ret;
}

static int msm_source_tracking_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct source_tracking_param);

	return 0;
}

static int msm_voice_source_tracking_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct source_tracking_param sourceTrackingData;

	memset(&sourceTrackingData, 0, sizeof(struct source_tracking_param));

	ret = voc_get_source_tracking(&sourceTrackingData);
	if (ret) {
		pr_debug("%s: Error getting Source Tracking Params, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}
	memcpy(ucontrol->value.bytes.data, (void *)&sourceTrackingData,
		sizeof(struct source_tracking_param));

done:
	return ret;
}

static int msm_audio_get_copp_idx_from_port_id(int port_id, int session_type,
					 int *copp_idx)
{
	int i, idx, be_idx;
	int ret = 0;
	unsigned long copp;

	pr_debug("%s: Enter, port_id=%d\n", __func__, port_id);

	ret = q6audio_validate_port(port_id);
	if (ret < 0) {
		pr_err("%s: port validation failed id 0x%x ret %d\n",
			__func__, port_id, ret);

		ret = -EINVAL;
		goto done;
	}

	for (be_idx = 0; be_idx < MSM_BACKEND_DAI_MAX; be_idx++) {
		if (msm_bedais[be_idx].port_id == port_id)
			break;
	}
	if (be_idx >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Invalid be id %d\n", __func__, be_idx);

		ret = -EINVAL;
		goto done;
	}

	for_each_set_bit(i, &msm_bedais[be_idx].fe_sessions[0],
			 MSM_FRONTEND_DAI_MAX) {

		for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
			copp = session_copp_map[i]
				[session_type][be_idx];
			if (test_bit(idx, &copp))
				break;
		}
		if (idx >= MAX_COPPS_PER_PORT)
			continue;
		else
			break;
	}
	if (i >= MSM_FRONTEND_DAI_MAX) {
		pr_debug("%s: Invalid FE, exiting\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	*copp_idx = idx;
	pr_debug("%s: copp_idx=%d\n", __func__, *copp_idx);

done:
	return ret;
}

static int msm_audio_sound_focus_derive_port_id(struct snd_kcontrol *kcontrol,
					    const char *prefix, int *port_id)
{
	int ret = 0;

	pr_debug("%s: Enter, prefix:%s\n", __func__, prefix);

	/*
	 * Mixer control name will be like "Sound Focus Audio Tx SLIMBUS_0"
	 * where the prefix is "Sound Focus Audio Tx ". Skip the prefix
	 * and compare the string with the backend name to derive the port id.
	 */
	if (!strcmp(kcontrol->id.name + strlen(prefix),
					"SLIMBUS_0")) {
		*port_id = SLIMBUS_0_TX;
	} else if (!strcmp(kcontrol->id.name + strlen(prefix),
					"TERT_MI2S")) {
		*port_id = AFE_PORT_ID_TERTIARY_MI2S_TX;
	} else if (!strcmp(kcontrol->id.name + strlen(prefix),
					"INT3_MI2S")) {
		*port_id = AFE_PORT_ID_INT3_MI2S_TX;
	} else if (!strcmp(kcontrol->id.name + strlen(prefix),
					"VA_CDC_DMA_TX_0")) {
		*port_id = AFE_PORT_ID_VA_CODEC_DMA_TX_0;
	} else if (!strcmp(kcontrol->id.name + strlen(prefix),
					"TX_CDC_DMA_TX_3")) {
		*port_id = AFE_PORT_ID_TX_CODEC_DMA_TX_3;
	} else if (!strcmp(kcontrol->id.name + strlen(prefix),
					"QUIN_TDM_TX_0")) {
		*port_id = AFE_PORT_ID_QUINARY_TDM_TX;
	} else if (!strcmp(kcontrol->id.name + strlen(prefix),
					"PRIMARY_TDM")) {
		*port_id = AFE_PORT_ID_PRIMARY_TDM_TX;
	} else {
		pr_err("%s: mixer ctl name=%s, could not derive valid port id\n",
			__func__, kcontrol->id.name);

		ret = -EINVAL;
		goto done;
	}
	pr_debug("%s: mixer ctl name=%s, derived port_id=%d\n",
		  __func__, kcontrol->id.name, *port_id);

done:
	return ret;
}

static int msm_audio_sound_focus_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct sound_focus_param soundFocusData;
	int port_id, copp_idx;

	ret = msm_audio_sound_focus_derive_port_id(kcontrol,
				"Sound Focus Audio Tx ", &port_id);
	if (ret != 0) {
		pr_err("%s: Error in deriving port id, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}

	ret = msm_audio_get_copp_idx_from_port_id(port_id, SESSION_TYPE_TX,
					    &copp_idx);
	if (ret) {
		pr_err("%s: Could not get copp idx for port_id=%d\n",
			__func__, port_id);

		ret = -EINVAL;
		goto done;
	}

	memcpy((void *)&soundFocusData, ucontrol->value.bytes.data,
		sizeof(struct sound_focus_param));

	ret = adm_set_sound_focus(port_id, copp_idx, soundFocusData);
	if (ret) {
		pr_err("%s: Error setting Sound Focus Params, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}

done:
	return ret;
}

static int msm_audio_sound_focus_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct sound_focus_param soundFocusData;
	int port_id, copp_idx;

	ret = msm_audio_sound_focus_derive_port_id(kcontrol,
				"Sound Focus Audio Tx ", &port_id);
	if (ret) {
		pr_err("%s: Error in deriving port id, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}

	ret = msm_audio_get_copp_idx_from_port_id(port_id, SESSION_TYPE_TX,
					    &copp_idx);
	if (ret) {
		pr_debug("%s: Could not get copp idx for port_id=%d\n",
			__func__, port_id);

		ret = -EINVAL;
		goto done;
	}

	ret = adm_get_sound_focus(port_id, copp_idx, &soundFocusData);
	if (ret) {
		pr_err("%s: Error getting Sound Focus Params, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}

	memcpy(ucontrol->value.bytes.data, (void *)&soundFocusData,
		sizeof(struct sound_focus_param));

done:
	return ret;
}

static int msm_audio_source_tracking_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct source_tracking_param sourceTrackingData;
	int port_id, copp_idx;

	ret = msm_audio_sound_focus_derive_port_id(kcontrol,
				"Source Tracking Audio Tx ", &port_id);
	if (ret) {
		pr_err("%s: Error in deriving port id, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}

	ret = msm_audio_get_copp_idx_from_port_id(port_id, SESSION_TYPE_TX,
					    &copp_idx);
	if (ret) {
		pr_debug("%s: Could not get copp idx for port_id=%d\n",
			__func__, port_id);

		ret = -EINVAL;
		goto done;
	}

	ret = adm_get_source_tracking(port_id, copp_idx, &sourceTrackingData);
	if (ret) {
		pr_err("%s: Error getting Source Tracking Params, err=%d\n",
			  __func__, ret);

		ret = -EINVAL;
		goto done;
	}

	memcpy(ucontrol->value.bytes.data, (void *)&sourceTrackingData,
		sizeof(struct source_tracking_param));

done:
	return ret;
}

static int msm_doa_tracking_mon_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct doa_tracking_mon_param);

	return 0;
}

static int msm_doa_tracking_mon_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct doa_tracking_mon_param doa_tracking_data;
	int port_id, copp_idx;

	memset(&doa_tracking_data, 0, sizeof(struct doa_tracking_mon_param));
	ret = msm_audio_sound_focus_derive_port_id(kcontrol,
				"Doa Tracking Monitor Listen ", &port_id);
	if (ret) {
		pr_err("%s: Error in deriving port id, err=%d\n",
			  __func__, ret);
		ret = -EINVAL;
		goto done;
	}

	/*
	 * If copp id exists for given port id, query adm to get doa data.
	 * Else query afe for doa tracking params.
	 * This is to support in cases where LSM directly connects to
	 * AFE for FFNS.
	 */
	ret = msm_audio_get_copp_idx_from_port_id(port_id, SESSION_TYPE_TX,
					    &copp_idx);
	if (!ret)
		ret = adm_get_doa_tracking_mon(port_id, copp_idx,
					&doa_tracking_data);
	else
		ret = afe_get_doa_tracking_mon(port_id, &doa_tracking_data);

	if (ret) {
		pr_err("%s: Error getting Doa Tracking Params, err=%d\n",
			  __func__, ret);
		ret = -EINVAL;
		goto done;
	}

	memcpy(ucontrol->value.bytes.data, (void *)&doa_tracking_data,
		sizeof(struct doa_tracking_mon_param));
done:
	return ret;
}

static const struct snd_kcontrol_new msm_source_tracking_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Voice Tx SLIMBUS_0",
		.info	= msm_sound_focus_info,
		.get	= msm_voice_sound_focus_get,
		.put	= msm_voice_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Voice Tx SLIMBUS_0",
		.info	= msm_source_tracking_info,
		.get	= msm_voice_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Audio Tx SLIMBUS_0",
		.info	= msm_sound_focus_info,
		.get	= msm_audio_sound_focus_get,
		.put	= msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Audio Tx SLIMBUS_0",
		.info	= msm_source_tracking_info,
		.get	= msm_audio_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Voice Tx TERT_MI2S",
		.info	= msm_sound_focus_info,
		.get	= msm_voice_sound_focus_get,
		.put	= msm_voice_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Voice Tx TERT_MI2S",
		.info	= msm_source_tracking_info,
		.get	= msm_voice_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Audio Tx TERT_MI2S",
		.info	= msm_sound_focus_info,
		.get	= msm_audio_sound_focus_get,
		.put	= msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Audio Tx TERT_MI2S",
		.info	= msm_source_tracking_info,
		.get	= msm_audio_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Voice Tx INT3_MI2S",
		.info	= msm_sound_focus_info,
		.get	= msm_voice_sound_focus_get,
		.put	= msm_voice_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Voice Tx INT3_MI2S",
		.info	= msm_source_tracking_info,
		.get	= msm_voice_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Audio Tx INT3_MI2S",
		.info	= msm_sound_focus_info,
		.get	= msm_audio_sound_focus_get,
		.put	= msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Audio Tx INT3_MI2S",
		.info	= msm_source_tracking_info,
		.get	= msm_audio_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Voice Tx VA_CDC_DMA_TX_0",
		.info	= msm_sound_focus_info,
		.get	= msm_voice_sound_focus_get,
		.put	= msm_voice_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Voice Tx VA_CDC_DMA_TX_0",
		.info	= msm_source_tracking_info,
		.get	= msm_voice_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Audio Tx VA_CDC_DMA_TX_0",
		.info	= msm_sound_focus_info,
		.get	= msm_audio_sound_focus_get,
		.put	= msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Audio Tx VA_CDC_DMA_TX_0",
		.info	= msm_source_tracking_info,
		.get	= msm_audio_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Voice Tx TX_CDC_DMA_TX_3",
		.info	= msm_sound_focus_info,
		.get	= msm_voice_sound_focus_get,
		.put	= msm_voice_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Voice Tx TX_CDC_DMA_TX_3",
		.info	= msm_source_tracking_info,
		.get	= msm_voice_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Audio Tx TX_CDC_DMA_TX_3",
		.info	= msm_sound_focus_info,
		.get	= msm_audio_sound_focus_get,
		.put	= msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Audio Tx TX_CDC_DMA_TX_3",
		.info	= msm_source_tracking_info,
		.get	= msm_audio_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Voice Tx QUIN_TDM_TX_0",
		.info	= msm_sound_focus_info,
		.get	= msm_voice_sound_focus_get,
		.put	= msm_voice_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Voice Tx QUIN_TDM_TX_0",
		.info	= msm_source_tracking_info,
		.get	= msm_voice_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Sound Focus Audio Tx QUIN_TDM_TX_0",
		.info	= msm_sound_focus_info,
		.get	= msm_audio_sound_focus_get,
		.put	= msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Source Tracking Audio Tx QUIN_TDM_TX_0",
		.info	= msm_source_tracking_info,
		.get	= msm_audio_source_tracking_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name   = "Sound Focus Audio Tx PRIMARY_TDM",
		.info   = msm_sound_focus_info,
		.get    = msm_audio_sound_focus_get,
		.put    = msm_audio_sound_focus_put,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface  = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name   = "Source Tracking Audio Tx PRIMARY_TDM",
		.info   = msm_source_tracking_info,
		.get    = msm_audio_source_tracking_get,
	},
};

static const struct snd_kcontrol_new msm_source_doa_tracking_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Doa Tracking Monitor Listen VA_CDC_DMA_TX_0",
		.info	= msm_doa_tracking_mon_info,
		.get	= msm_doa_tracking_mon_get,
	},
};

static int spkr_prot_put_vi_lch_port(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	int item;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	pr_debug("%s item is %d\n", __func__,
		   ucontrol->value.enumerated.item[0]);
	mutex_lock(&routing_lock);
	item = ucontrol->value.enumerated.item[0];
	if (item < e->items) {
		pr_debug("%s RX DAI ID %d TX DAI id %d\n",
			__func__, e->shift_l, e->values[item]);
		if (e->shift_l < MSM_BACKEND_DAI_MAX &&
			e->values[item] < MSM_BACKEND_DAI_MAX)
			/* Enable feedback TX path */
			ret = afe_spk_prot_feed_back_cfg(
			   msm_bedais[e->values[item]].port_id,
			   msm_bedais[e->shift_l].port_id, 1, 0, 1);
		else {
			pr_debug("%s values are out of range item %d\n",
			__func__, e->values[item]);
			/* Disable feedback TX path */
			if (e->values[item] == MSM_BACKEND_DAI_MAX)
				ret = afe_spk_prot_feed_back_cfg(0, 0, 0, 0, 0);
			else
				ret = -EINVAL;
		}
	} else {
		pr_err("%s item value is out of range item\n", __func__);
		ret = -EINVAL;
	}
	mutex_unlock(&routing_lock);
	return ret;
}

static int spkr_prot_get_vi_lch_port(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const int mi2s_rx_vi_fb_tx_value[] = {
	MSM_BACKEND_DAI_MAX, MSM_BACKEND_DAI_PRI_MI2S_TX
};

static const char * const mi2s_rx_vi_fb_tx_mux_text[] = {
	"ZERO", "PRI_MI2S_TX"
};

static const struct soc_enum mi2s_rx_vi_fb_mux_enum =
	SOC_VALUE_ENUM_DOUBLE(0, MSM_BACKEND_DAI_PRI_MI2S_RX, 0, 0,
	ARRAY_SIZE(mi2s_rx_vi_fb_tx_mux_text),
	mi2s_rx_vi_fb_tx_mux_text, mi2s_rx_vi_fb_tx_value);

static const struct snd_kcontrol_new mi2s_rx_vi_fb_mux =
	SOC_DAPM_ENUM_EXT("PRI_MI2S_RX_VI_FB_MUX",
	mi2s_rx_vi_fb_mux_enum, spkr_prot_get_vi_lch_port,
	spkr_prot_put_vi_lch_port);

static const struct snd_soc_dapm_widget msm_qdsp6_widgets[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("MM_DL1", "MultiMedia1 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL2", "MultiMedia2 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL3", "MultiMedia3 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("MM_DL4", "MultiMedia4 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOIP_DL", "VoIP Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MM_UL1", "MultiMedia1 Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MM_UL2", "MultiMedia2 Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MM_UL3", "MultiMedia3 Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("MM_UL4", "MultiMedia4 Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOIP_UL", "VoIP Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOICEMMODE1_DL",
		"VoiceMMode1 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOICEMMODE1_UL",
		"VoiceMMode1 Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOICEMMODE2_DL",
		"VoiceMMode2 Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOICEMMODE2_UL",
		"VoiceMMode2 Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("USBAUDIO_DL_HL", "USBAUDIO_HOSTLESS Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("USBAUDIO_UL_HL", "USBAUDIO_HOSTLESS Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("DTMF_DL_HL", "DTMF_RX_HOSTLESS Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QCHAT_DL", "QCHAT Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QCHAT_UL", "QCHAT Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("CDC_DMA_DL_HL", "CDC_DMA_HOSTLESS Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("CDC_DMA_UL_HL", "CDC_DMA_HOSTLESS Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TX3_CDC_DMA_UL_HL",
		"TX3_CDC_DMA_HOSTLESS Capture", 0, 0, 0, 0),
	/* Backend AIF */
	/* Stream name equals to backend dai link stream name */
	SND_SOC_DAPM_AIF_OUT("PCM_RX", "AFE Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PCM_TX", "AFE Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("WSA_CDC_DMA_RX_0", "WSA CDC DMA0 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("WSA_CDC_DMA_TX_0", "WSA CDC DMA0 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("WSA_CDC_DMA_RX_1", "WSA CDC DMA1 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("WSA_CDC_DMA_TX_1", "WSA CDC DMA1 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("WSA_CDC_DMA_TX_2", "WSA CDC DMA2 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VA_CDC_DMA_TX_0", "VA CDC DMA0 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VA_CDC_DMA_TX_1", "VA CDC DMA1 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VA_CDC_DMA_TX_2", "VA CDC DMA2 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_0", "RX CDC DMA0 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TX_CDC_DMA_TX_0", "TX CDC DMA0 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_1", "RX CDC DMA1 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TX_CDC_DMA_TX_1", "TX CDC DMA1 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_2", "RX CDC DMA2 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TX_CDC_DMA_TX_2", "TX CDC DMA2 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_3", "RX CDC DMA3 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TX_CDC_DMA_TX_3", "TX CDC DMA3 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_4", "RX CDC DMA4 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TX_CDC_DMA_TX_4", "TX CDC DMA4 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_5", "RX CDC DMA5 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TX_CDC_DMA_TX_5", "TX CDC DMA5 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_6", "RX CDC DMA6 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("RX_CDC_DMA_RX_7", "RX CDC DMA7 Playback",
				0, 0, 0, 0),
	/* incall */
	SND_SOC_DAPM_AIF_OUT("VOICE_PLAYBACK_TX", "Voice Farend Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOICE2_PLAYBACK_TX", "Voice2 Farend Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("INCALL_RECORD_TX", "Voice Uplink Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("INCALL_RECORD_RX", "Voice Downlink Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOICE_STUB_DL", "VOICE_STUB Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOICE_STUB_UL", "VOICE_STUB Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOICE2_STUB_DL", "VOICE2_STUB Playback",
			    0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOICE2_STUB_UL", "VOICE2_STUB Capture",
			    0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("VOLTE_STUB_DL", "VOLTE_STUB Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("VOLTE_STUB_UL", "VOLTE_STUB Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("STUB_RX", "Stub Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("STUB_TX", "Stub Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("STUB_1_TX", "Stub1 Capture", 0, 0, 0, 0),
	/* In- call recording */
	SND_SOC_DAPM_AIF_IN("AFE_LOOPBACK_TX", "AFE Loopback Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("USB_AUDIO_RX", "USB Audio Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("USB_AUDIO_TX", "USB Audio Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PROXY_RX", "Proxy Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PROXY_TX", "Proxy Capture", 0, 0, 0, 0),

	/* Switch Definitions */
	SND_SOC_DAPM_SWITCH("USB_DL_HL", SND_SOC_NOPM, 0, 0,
				&usb_switch_mixer_controls),
	SND_SOC_DAPM_SWITCH("RX_CDC_DMA_RX_0_DL_HL", SND_SOC_NOPM, 0, 0,
				&cdc_dma_rx_switch_mixer_controls),

	/* Mixer definitions */
	SND_SOC_DAPM_MIXER("MultiMedia1 Mixer", SND_SOC_NOPM, 0, 0,
	mmul1_mixer_controls, ARRAY_SIZE(mmul1_mixer_controls)),
	SND_SOC_DAPM_MIXER("MultiMedia2 Mixer", SND_SOC_NOPM, 0, 0,
	mmul2_mixer_controls, ARRAY_SIZE(mmul2_mixer_controls)),
	SND_SOC_DAPM_MIXER("MultiMedia3 Mixer", SND_SOC_NOPM, 0, 0,
	mmul3_mixer_controls, ARRAY_SIZE(mmul3_mixer_controls)),
	SND_SOC_DAPM_MIXER("MultiMedia4 Mixer", SND_SOC_NOPM, 0, 0,
	mmul4_mixer_controls, ARRAY_SIZE(mmul4_mixer_controls)),
	SND_SOC_DAPM_MIXER("WSA_CDC_DMA_RX_0 Audio Mixer", SND_SOC_NOPM, 0, 0,
				wsa_cdc_dma_rx_0_mixer_controls,
				ARRAY_SIZE(wsa_cdc_dma_rx_0_mixer_controls)),
	SND_SOC_DAPM_MIXER("WSA_CDC_DMA_RX_1 Audio Mixer", SND_SOC_NOPM, 0, 0,
				wsa_cdc_dma_rx_1_mixer_controls,
				ARRAY_SIZE(wsa_cdc_dma_rx_1_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_0 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_0_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_0_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_1 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_1_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_1_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_2 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_2_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_2_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_3 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_3_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_3_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_4 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_4_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_4_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_5 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_5_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_5_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_6 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_6_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_6_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_7 Audio Mixer", SND_SOC_NOPM, 0, 0,
				rx_cdc_dma_rx_7_mixer_controls,
				ARRAY_SIZE(rx_cdc_dma_rx_7_mixer_controls)),

	/* incall */
	SND_SOC_DAPM_MIXER("Incall_Music Audio Mixer", SND_SOC_NOPM, 0, 0,
	incall_music_delivery_mixer_controls,
	ARRAY_SIZE(incall_music_delivery_mixer_controls)),
	SND_SOC_DAPM_MIXER("Incall_Music_2 Audio Mixer", SND_SOC_NOPM, 0, 0,
	incall_music2_delivery_mixer_controls,
	ARRAY_SIZE(incall_music2_delivery_mixer_controls)),
	SND_SOC_DAPM_MIXER("USB_AUDIO_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
	usb_audio_rx_mixer_controls,
	ARRAY_SIZE(usb_audio_rx_mixer_controls)),
	/* Voice Mixer */
	SND_SOC_DAPM_MIXER("AFE_PCM_RX_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				afe_pcm_rx_voice_mixer_controls,
				ARRAY_SIZE(afe_pcm_rx_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("AUX_PCM_RX_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				aux_pcm_rx_voice_mixer_controls,
				ARRAY_SIZE(aux_pcm_rx_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("PROXY_RX_Voice Mixer",
			   SND_SOC_NOPM, 0, 0,
			   proxy_rx_voice_mixer_controls,
			   ARRAY_SIZE(proxy_rx_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("WSA_CDC_DMA_RX_0_Voice Mixer",
			   SND_SOC_NOPM, 0, 0,
			   wsa_cdc_dma_rx_0_voice_mixer_controls,
			   ARRAY_SIZE(wsa_cdc_dma_rx_0_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_0_Voice Mixer",
			   SND_SOC_NOPM, 0, 0,
			   rx_cdc_dma_rx_0_voice_mixer_controls,
			   ARRAY_SIZE(rx_cdc_dma_rx_0_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_1_Voice Mixer",
			   SND_SOC_NOPM, 0, 0,
			   rx_cdc_dma_rx_1_voice_mixer_controls,
			   ARRAY_SIZE(rx_cdc_dma_rx_1_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("Voip_Tx Mixer",
				SND_SOC_NOPM, 0, 0, tx_voip_mixer_controls,
				ARRAY_SIZE(tx_voip_mixer_controls)),
	SND_SOC_DAPM_MIXER("VoiceMMode1_Tx Mixer",
			   SND_SOC_NOPM, 0, 0, tx_voicemmode1_mixer_controls,
			   ARRAY_SIZE(tx_voicemmode1_mixer_controls)),
	SND_SOC_DAPM_MIXER("VoiceMMode2_Tx Mixer",
			   SND_SOC_NOPM, 0, 0, tx_voicemmode2_mixer_controls,
			   ARRAY_SIZE(tx_voicemmode2_mixer_controls)),
	SND_SOC_DAPM_MIXER("AFE_PCM_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
	afe_pcm_rx_mixer_controls, ARRAY_SIZE(afe_pcm_rx_mixer_controls)),
	SND_SOC_DAPM_MIXER("Voice Stub Tx Mixer", SND_SOC_NOPM, 0, 0,
	tx_voice_stub_mixer_controls, ARRAY_SIZE(tx_voice_stub_mixer_controls)),
	SND_SOC_DAPM_MIXER("Voice2 Stub Tx Mixer", SND_SOC_NOPM, 0, 0,
			   tx_voice2_stub_mixer_controls,
			   ARRAY_SIZE(tx_voice2_stub_mixer_controls)),
	SND_SOC_DAPM_MIXER("VoLTE Stub Tx Mixer", SND_SOC_NOPM, 0, 0,
	tx_volte_stub_mixer_controls, ARRAY_SIZE(tx_volte_stub_mixer_controls)),
	/* port mixer */
	SND_SOC_DAPM_MIXER("QCHAT_Tx Mixer",
	SND_SOC_NOPM, 0, 0, tx_qchat_mixer_controls,
	ARRAY_SIZE(tx_qchat_mixer_controls)),
	SND_SOC_DAPM_MIXER("USB_AUDIO_RX_Voice Mixer",
	SND_SOC_NOPM, 0, 0, usb_audio_rx_voice_mixer_controls,
	ARRAY_SIZE(usb_audio_rx_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("USB_AUDIO_RX Port Mixer",
	SND_SOC_NOPM, 0, 0, usb_rx_port_mixer_controls,
	ARRAY_SIZE(usb_rx_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("RX_CDC_DMA_RX_0 Port Mixer", SND_SOC_NOPM, 0, 0,
	rx_cdc_dma_rx_0_port_mixer_controls,
	ARRAY_SIZE(rx_cdc_dma_rx_0_port_mixer_controls)),
	/* lsm mixer definitions */
	/* Virtual Pins to force backends ON atm */
	SND_SOC_DAPM_OUTPUT("BE_OUT"),
	SND_SOC_DAPM_INPUT("BE_IN"),

	SND_SOC_DAPM_MUX("VOC_EXT_EC MUX", SND_SOC_NOPM, 0, 0,
			 &voc_ext_ec_mux),
	SND_SOC_DAPM_MUX("AUDIO_REF_EC_UL1 MUX", SND_SOC_NOPM, 0, 0,
		&ext_ec_ref_mux_ul1),
	SND_SOC_DAPM_MUX("AUDIO_REF_EC_UL2 MUX", SND_SOC_NOPM, 0, 0,
		&ext_ec_ref_mux_ul2),
	SND_SOC_DAPM_MUX("AUDIO_REF_EC_UL3 MUX", SND_SOC_NOPM, 0, 0,
		&ext_ec_ref_mux_ul3),
	SND_SOC_DAPM_MUX("AUDIO_REF_EC_UL4 MUX", SND_SOC_NOPM, 0, 0,
		&ext_ec_ref_mux_ul4),
};

#ifndef CONFIG_AUXPCM_DISABLE
static const struct snd_soc_dapm_widget msm_qdsp6_widgets_aux_pcm[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("AUXPCM_DL_HL", "AUXPCM_HOSTLESS Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AUXPCM_UL_HL", "AUXPCM_HOSTLESS Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_AUXPCM_DL_HL", "SEC_AUXPCM_HOSTLESS Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_AUXPCM_UL_HL", "SEC_AUXPCM_HOSTLESS Capture",
		0, 0, 0, 0),
	/* Backend AIF */
	/* Stream name equals to backend dai link stream name */
	/* incall */
	SND_SOC_DAPM_AIF_OUT("AUX_PCM_RX", "AUX PCM Playback", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("AUX_PCM_TX", "AUX PCM Capture", 0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_AUX_PCM_RX", "Sec AUX PCM Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_AUX_PCM_TX", "Sec AUX PCM Capture",
				0, 0, 0, 0),
	/* In- call recording */

	/* Switch Definitions */

	/* Mixer definitions */
	SND_SOC_DAPM_MIXER("AUX_PCM_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
	auxpcm_rx_mixer_controls, ARRAY_SIZE(auxpcm_rx_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_AUX_PCM_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
	sec_auxpcm_rx_mixer_controls, ARRAY_SIZE(sec_auxpcm_rx_mixer_controls)),
	/* incall */
	/* Voice Mixer */
	SND_SOC_DAPM_MIXER("SEC_AUX_PCM_RX_Voice Mixer",
			      SND_SOC_NOPM, 0, 0,
			      sec_aux_pcm_rx_voice_mixer_controls,
			      ARRAY_SIZE(sec_aux_pcm_rx_voice_mixer_controls)),
	/* port mixer */
	SND_SOC_DAPM_MIXER("AUX_PCM_RX Port Mixer",
	SND_SOC_NOPM, 0, 0, aux_pcm_rx_port_mixer_controls,
	ARRAY_SIZE(aux_pcm_rx_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_AUXPCM_RX Port Mixer",
	SND_SOC_NOPM, 0, 0, sec_auxpcm_rx_port_mixer_controls,
	ARRAY_SIZE(sec_auxpcm_rx_port_mixer_controls)),
};
#endif

#ifndef CONFIG_MI2S_DISABLE
static const struct snd_soc_dapm_widget msm_qdsp6_widgets_mi2s[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("PRI_MI2S_DL_HL",
		"Primary MI2S_RX Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_MI2S_DL_HL",
		"Secondary MI2S_RX Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_MI2S_DL_HL",
		"Tertiary MI2S_RX Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_MI2S_DL_HL",
		"Quaternary MI2S_RX Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_MI2S_UL_HL",
		"Primary MI2S_TX Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_MI2S_UL_HL",
		"Secondary MI2S_TX Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_MI2S_UL_HL",
		"Tertiary MI2S_TX Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_MI2S_UL_HL",
		"Quaternary MI2S_TX Hostless Capture",
		0, 0, 0, 0),
	/* Backend AIF */
	/* Stream name equals to backend dai link stream name */
	SND_SOC_DAPM_AIF_OUT("SEC_MI2S_RX", "Secondary MI2S Playback",
			     0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_MI2S_RX", "Primary MI2S Playback",
			     0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_MI2S_TX", "Primary MI2S Capture",
			    0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_MI2S_TX", "Secondary MI2S Capture",
			    0, 0, 0, 0),
	SND_SOC_DAPM_SWITCH("PRI_MI2S_RX_DL_HL", SND_SOC_NOPM, 0, 0,
				&pri_mi2s_rx_switch_mixer_controls),
	SND_SOC_DAPM_SWITCH("SEC_MI2S_RX_DL_HL", SND_SOC_NOPM, 0, 0,
				&sec_mi2s_rx_switch_mixer_controls),
	SND_SOC_DAPM_MIXER("SEC_MI2S_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
			   secondary_mi2s_rx_mixer_controls,
			   ARRAY_SIZE(secondary_mi2s_rx_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_MI2S_RX Audio Mixer", SND_SOC_NOPM, 0, 0,
			   primary_mi2s_rx_mixer_controls,
			   ARRAY_SIZE(primary_mi2s_rx_mixer_controls)),

	/* incall */
	/* Voice Mixer */
	SND_SOC_DAPM_MIXER("SEC_MI2S_RX_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				sec_mi2s_rx_voice_mixer_controls,
				ARRAY_SIZE(sec_mi2s_rx_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_MI2S_RX_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				pri_mi2s_rx_voice_mixer_controls,
				ARRAY_SIZE(pri_mi2s_rx_voice_mixer_controls)),
	/* port mixer */
	SND_SOC_DAPM_MIXER("PRI_MI2S_RX Port Mixer", SND_SOC_NOPM, 0, 0,
	primary_mi2s_rx_port_mixer_controls,
	ARRAY_SIZE(primary_mi2s_rx_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_MI2S_RX Port Mixer", SND_SOC_NOPM, 0, 0,
	sec_mi2s_rx_port_mixer_controls,
	ARRAY_SIZE(sec_mi2s_rx_port_mixer_controls)),
	SND_SOC_DAPM_MUX("PRI_MI2S_RX_VI_FB_MUX", SND_SOC_NOPM, 0, 0,
				&mi2s_rx_vi_fb_mux),
};
#endif

#ifndef CONFIG_TDM_DISABLE
static const struct snd_soc_dapm_widget msm_qdsp6_widgets_tdm[] = {
	/* Frontend AIF */
	/* Widget name equals to Front-End DAI name<Need confirmation>,
	 * Stream name must contains substring of front-end dai name
	 */
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_0_DL_HL",
		"Primary TDM0 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_0_UL_HL",
		"Primary TDM0 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_1_DL_HL",
		"Primary TDM1 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_1_UL_HL",
		"Primary TDM1 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_2_DL_HL",
		"Primary TDM2 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_2_UL_HL",
		"Primary TDM2 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_3_DL_HL",
		"Primary TDM3 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_3_UL_HL",
		"Primary TDM3 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_4_DL_HL",
		"Primary TDM4 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_4_UL_HL",
		"Primary TDM4 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_5_DL_HL",
		"Primary TDM5 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_5_UL_HL",
		"Primary TDM5 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_6_DL_HL",
		"Primary TDM6 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_6_UL_HL",
		"Primary TDM6 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_RX_7_DL_HL",
		"Primary TDM7 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_TX_7_UL_HL",
		"Primary TDM7 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_0_DL_HL",
		"Secondary TDM0 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_0_UL_HL",
		"Secondary TDM0 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_1_DL_HL",
		"Secondary TDM1 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_1_UL_HL",
		"Secondary TDM1 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_2_DL_HL",
		"Secondary TDM2 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_2_UL_HL",
		"Secondary TDM2 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_3_DL_HL",
		"Secondary TDM3 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_3_UL_HL",
		"Secondary TDM3 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_4_DL_HL",
		"Secondary TDM4 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_4_UL_HL",
		"Secondary TDM4 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_5_DL_HL",
		"Secondary TDM5 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_5_UL_HL",
		"Secondary TDM5 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_6_DL_HL",
		"Secondary TDM6 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_6_UL_HL",
		"Secondary TDM6 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_RX_7_DL_HL",
		"Secondary TDM7 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_TX_7_UL_HL",
		"Secondary TDM7 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_0_DL_HL",
		"Tertiary TDM0 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_0_UL_HL",
		"Tertiary TDM0 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_1_DL_HL",
		"Tertiary TDM1 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_1_UL_HL",
		"Tertiary TDM1 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_2_DL_HL",
		"Tertiary TDM2 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_2_UL_HL",
		"Tertiary TDM2 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_3_DL_HL",
		"Tertiary TDM3 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_3_UL_HL",
		"Tertiary TDM3 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_4_DL_HL",
		"Tertiary TDM4 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_4_UL_HL",
		"Tertiary TDM4 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_5_DL_HL",
		"Tertiary TDM5 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_5_UL_HL",
		"Tertiary TDM5 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_6_DL_HL",
		"Tertiary TDM6 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_6_UL_HL",
		"Tertiary TDM6 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("TERT_TDM_RX_7_DL_HL",
		"Tertiary TDM7 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("TERT_TDM_TX_7_UL_HL",
		"Tertiary TDM7 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_0_DL_HL",
		"Quaternary TDM0 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_0_UL_HL",
		"Quaternary TDM0 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_1_DL_HL",
		"Quaternary TDM1 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_1_UL_HL",
		"Quaternary TDM1 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_2_DL_HL",
		"Quaternary TDM2 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_2_UL_HL",
		"Quaternary TDM2 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_3_DL_HL",
		"Quaternary TDM3 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_3_UL_HL",
		"Quaternary TDM3 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_4_DL_HL",
		"Quaternary TDM4 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_4_UL_HL",
		"Quaternary TDM4 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_5_DL_HL",
		"Quaternary TDM5 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_5_UL_HL",
		"Quaternary TDM5 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_6_DL_HL",
		"Quaternary TDM6 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_6_UL_HL",
		"Quaternary TDM6 Hostless Capture",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("QUAT_TDM_RX_7_DL_HL",
		"Quaternary TDM7 Hostless Playback",
		0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("QUAT_TDM_TX_7_UL_HL",
		"Quaternary TDM7 Hostless Capture",
		0, 0, 0, 0),
	/* Backend AIF */
	/* Stream name equals to backend dai link stream name */
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_0", "Primary TDM0 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_0", "Primary TDM0 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_1", "Primary TDM1 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_1", "Primary TDM1 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_2", "Primary TDM2 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_2", "Primary TDM2 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_3", "Primary TDM3 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_3", "Primary TDM3 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_4", "Primary TDM4 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_4", "Primary TDM4 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_5", "Primary TDM5 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_5", "Primary TDM5 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_6", "Primary TDM6 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_6", "Primary TDM6 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("PRI_TDM_RX_7", "Primary TDM7 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("PRI_TDM_TX_7", "Primary TDM7 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_0", "Secondary TDM0 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_0", "Secondary TDM0 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_1", "Secondary TDM1 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_1", "Secondary TDM1 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_2", "Secondary TDM2 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_2", "Secondary TDM2 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_3", "Secondary TDM3 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_3", "Secondary TDM3 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_4", "Secondary TDM4 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_4", "Secondary TDM4 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_5", "Secondary TDM5 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_5", "Secondary TDM5 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_6", "Secondary TDM6 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_6", "Secondary TDM6 Capture",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_OUT("SEC_TDM_RX_7", "Secondary TDM7 Playback",
				0, 0, 0, 0),
	SND_SOC_DAPM_AIF_IN("SEC_TDM_TX_7", "Secondary TDM7 Capture",
				0, 0, 0, 0),

	SND_SOC_DAPM_MIXER("PRI_TDM_RX_0 Audio Mixer", SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_0_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_0_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_1 Audio Mixer", SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_1_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_1_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_2 Audio Mixer", SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_2_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_2_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_3 Audio Mixer", SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_3_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_3_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_TX_0 Audio Mixer", SND_SOC_NOPM, 0, 0,
				pri_tdm_tx_0_mixer_controls,
				ARRAY_SIZE(pri_tdm_tx_0_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_0 Audio Mixer", SND_SOC_NOPM, 0, 0,
				sec_tdm_rx_0_mixer_controls,
				ARRAY_SIZE(sec_tdm_rx_0_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_1 Audio Mixer", SND_SOC_NOPM, 0, 0,
				sec_tdm_rx_1_mixer_controls,
				ARRAY_SIZE(sec_tdm_rx_1_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_2 Audio Mixer", SND_SOC_NOPM, 0, 0,
				sec_tdm_rx_2_mixer_controls,
				ARRAY_SIZE(sec_tdm_rx_2_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_3 Audio Mixer", SND_SOC_NOPM, 0, 0,
				sec_tdm_rx_3_mixer_controls,
				ARRAY_SIZE(sec_tdm_rx_3_mixer_controls)),
 	SND_SOC_DAPM_MIXER("SEC_TDM_TX_0 Audio Mixer", SND_SOC_NOPM, 0, 0,
 				sec_tdm_tx_0_mixer_controls,
 				ARRAY_SIZE(sec_tdm_tx_0_mixer_controls)),
 	SND_SOC_DAPM_MIXER("SEC_TDM_TX_1 Audio Mixer", SND_SOC_NOPM, 0, 0,
 				sec_tdm_tx_1_mixer_controls,
 				ARRAY_SIZE(sec_tdm_tx_1_mixer_controls)),
	/* incall */
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_0_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_0_voice_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_0_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_1_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_1_voice_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_1_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_2_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_2_voice_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_2_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_3_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				pri_tdm_rx_3_voice_mixer_controls,
				ARRAY_SIZE(pri_tdm_rx_3_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_0_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				sec_tdm_rx_0_voice_mixer_controls,
				ARRAY_SIZE(sec_tdm_rx_0_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_1_Voice Mixer",
				SND_SOC_NOPM, 0, 0,
				sec_tdm_rx_1_voice_mixer_controls,
				ARRAY_SIZE(sec_tdm_rx_1_voice_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_0 Port Mixer", SND_SOC_NOPM, 0, 0,
	pri_tdm_rx_0_port_mixer_controls,
	ARRAY_SIZE(pri_tdm_rx_0_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_1 Port Mixer", SND_SOC_NOPM, 0, 0,
	pri_tdm_rx_1_port_mixer_controls,
	ARRAY_SIZE(pri_tdm_rx_1_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_2 Port Mixer", SND_SOC_NOPM, 0, 0,
	pri_tdm_rx_2_port_mixer_controls,
	ARRAY_SIZE(pri_tdm_rx_2_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("PRI_TDM_RX_3 Port Mixer", SND_SOC_NOPM, 0, 0,
	pri_tdm_rx_3_port_mixer_controls,
	ARRAY_SIZE(pri_tdm_rx_3_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_0 Port Mixer", SND_SOC_NOPM, 0, 0,
	sec_tdm_rx_0_port_mixer_controls,
	ARRAY_SIZE(sec_tdm_rx_0_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_1 Port Mixer", SND_SOC_NOPM, 0, 0,
	sec_tdm_rx_1_port_mixer_controls,
	ARRAY_SIZE(sec_tdm_rx_1_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_2 Port Mixer", SND_SOC_NOPM, 0, 0,
	sec_tdm_rx_2_port_mixer_controls,
	ARRAY_SIZE(sec_tdm_rx_2_port_mixer_controls)),
	SND_SOC_DAPM_MIXER("SEC_TDM_RX_3 Port Mixer", SND_SOC_NOPM, 0, 0,
	sec_tdm_rx_3_port_mixer_controls,
	ARRAY_SIZE(sec_tdm_rx_3_port_mixer_controls)),
};
#endif

static const struct snd_soc_dapm_route intercon[] = {

	{"WSA_CDC_DMA_RX_0 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"WSA_CDC_DMA_RX_0 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"WSA_CDC_DMA_RX_0 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"WSA_CDC_DMA_RX_0 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"WSA_CDC_DMA_RX_0", NULL, "WSA_CDC_DMA_RX_0 Audio Mixer"},

	{"WSA_CDC_DMA_RX_1 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"WSA_CDC_DMA_RX_1 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"WSA_CDC_DMA_RX_1 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"WSA_CDC_DMA_RX_1 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"WSA_CDC_DMA_RX_1", NULL, "WSA_CDC_DMA_RX_1 Audio Mixer"},

	{"RX_CDC_DMA_RX_0 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_0 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_0 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_0 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_0", NULL, "RX_CDC_DMA_RX_0 Audio Mixer"},
	{"RX_CDC_DMA_RX_0 Audio Mixer", "DTMF", "DTMF_DL_HL"},

	{"RX_CDC_DMA_RX_1 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_1 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_1 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_1 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_1", NULL, "RX_CDC_DMA_RX_1 Audio Mixer"},

	{"RX_CDC_DMA_RX_2 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_2 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_2 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_2 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_2", NULL, "RX_CDC_DMA_RX_2 Audio Mixer"},

	{"RX_CDC_DMA_RX_3 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_3 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_3 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_3 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_3", NULL, "RX_CDC_DMA_RX_3 Audio Mixer"},

	{"RX_CDC_DMA_RX_4 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_4 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_4 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_4 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_4", NULL, "RX_CDC_DMA_RX_4 Audio Mixer"},

	{"RX_CDC_DMA_RX_5 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_5 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_5 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_5 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_5", NULL, "RX_CDC_DMA_RX_5 Audio Mixer"},

	{"RX_CDC_DMA_RX_6 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_6 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_6 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_6 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_6", NULL, "RX_CDC_DMA_RX_6 Audio Mixer"},

	{"RX_CDC_DMA_RX_7 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"RX_CDC_DMA_RX_7 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"RX_CDC_DMA_RX_7 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"RX_CDC_DMA_RX_7 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"RX_CDC_DMA_RX_7", NULL, "RX_CDC_DMA_RX_7 Audio Mixer"},

	/* incall */
	{"Incall_Music Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"Incall_Music Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"Incall_Music Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"Incall_Music Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"VOICE_PLAYBACK_TX", NULL, "Incall_Music Audio Mixer"},
	{"Incall_Music_2 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"Incall_Music_2 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"VOICE2_PLAYBACK_TX", NULL, "Incall_Music_2 Audio Mixer"},

	{"USB_AUDIO_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"USB_AUDIO_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"USB_AUDIO_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"USB_AUDIO_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"USB_AUDIO_RX", NULL, "USB_AUDIO_RX Audio Mixer"},

	{"MultiMedia1 Mixer", "VOC_REC_UL", "INCALL_RECORD_TX"},
	{"MultiMedia2 Mixer", "VOC_REC_UL", "INCALL_RECORD_TX"},
	{"MultiMedia3 Mixer", "VOC_REC_UL", "INCALL_RECORD_TX"},
	{"MultiMedia4 Mixer", "VOC_REC_UL", "INCALL_RECORD_TX"},
	{"MultiMedia1 Mixer", "VOC_REC_DL", "INCALL_RECORD_RX"},
	{"MultiMedia2 Mixer", "VOC_REC_DL", "INCALL_RECORD_RX"},
	{"MultiMedia3 Mixer", "VOC_REC_DL", "INCALL_RECORD_RX"},
	{"MultiMedia4 Mixer", "VOC_REC_DL", "INCALL_RECORD_RX"},

	{"MultiMedia1 Mixer", "AFE_LOOPBACK_TX", "AFE_LOOPBACK_TX"},
	{"MultiMedia2 Mixer", "AFE_LOOPBACK_TX", "AFE_LOOPBACK_TX"},
	{"MultiMedia3 Mixer", "AFE_LOOPBACK_TX", "AFE_LOOPBACK_TX"},
	{"MultiMedia4 Mixer", "AFE_LOOPBACK_TX", "AFE_LOOPBACK_TX"},

	{"MultiMedia1 Mixer", "USB_AUDIO_TX", "USB_AUDIO_TX"},
	{"MultiMedia2 Mixer", "USB_AUDIO_TX", "USB_AUDIO_TX"},

	{"MultiMedia1 Mixer", "WSA_CDC_DMA_TX_0", "WSA_CDC_DMA_TX_0"},
	{"MultiMedia1 Mixer", "WSA_CDC_DMA_TX_1", "WSA_CDC_DMA_TX_1"},
	{"MultiMedia1 Mixer", "WSA_CDC_DMA_TX_2", "WSA_CDC_DMA_TX_2"},
	{"MultiMedia1 Mixer", "VA_CDC_DMA_TX_0", "VA_CDC_DMA_TX_0"},
	{"MultiMedia1 Mixer", "VA_CDC_DMA_TX_1", "VA_CDC_DMA_TX_1"},
	{"MultiMedia1 Mixer", "VA_CDC_DMA_TX_2", "VA_CDC_DMA_TX_2"},
	{"MultiMedia1 Mixer", "TX_CDC_DMA_TX_0", "TX_CDC_DMA_TX_0"},
	{"MultiMedia1 Mixer", "TX_CDC_DMA_TX_1", "TX_CDC_DMA_TX_1"},
	{"MultiMedia1 Mixer", "TX_CDC_DMA_TX_2", "TX_CDC_DMA_TX_2"},
	{"MultiMedia1 Mixer", "TX_CDC_DMA_TX_3", "TX_CDC_DMA_TX_3"},
	{"MultiMedia1 Mixer", "TX_CDC_DMA_TX_4", "TX_CDC_DMA_TX_4"},
	{"MultiMedia1 Mixer", "TX_CDC_DMA_TX_5", "TX_CDC_DMA_TX_5"},

	{"AFE_PCM_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"AFE_PCM_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"AFE_PCM_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"AFE_PCM_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PCM_RX", NULL, "AFE_PCM_RX Audio Mixer"},

	{"MultiMedia1 Mixer", "AFE_PCM_TX", "PCM_TX"},
	{"MultiMedia3 Mixer", "AFE_PCM_TX", "PCM_TX"},
	{"MM_UL1", NULL, "MultiMedia1 Mixer"},
	{"MM_UL2", NULL, "MultiMedia2 Mixer"},
	{"MM_UL3", NULL, "MultiMedia3 Mixer"},
	{"MM_UL4", NULL, "MultiMedia4 Mixer"},

	{"USB_AUDIO_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"USB_AUDIO_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"USB_AUDIO_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"USB_AUDIO_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"USB_AUDIO_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"USB_AUDIO_RX", NULL, "USB_AUDIO_RX_Voice Mixer"},

	{"AFE_PCM_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"AFE_PCM_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"AFE_PCM_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"AFE_PCM_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"AFE_PCM_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PCM_RX", NULL, "AFE_PCM_RX_Voice Mixer"},

	{"PROXY_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PROXY_RX", NULL, "PROXY_RX_Voice Mixer"},

	{"PROXY_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PROXY_RX", NULL, "PROXY_RX_Voice Mixer"},

	{"WSA_CDC_DMA_RX_0_Voice Mixer", "Voip", "VOIP_DL"},
	{"WSA_CDC_DMA_RX_0_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"WSA_CDC_DMA_RX_0_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"WSA_CDC_DMA_RX_0", NULL, "WSA_CDC_DMA_RX_0_Voice Mixer"},

	{"RX_CDC_DMA_RX_0_Voice Mixer", "Voip", "VOIP_DL"},
	{"RX_CDC_DMA_RX_0_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"RX_CDC_DMA_RX_0_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"RX_CDC_DMA_RX_0_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"RX_CDC_DMA_RX_0", NULL, "RX_CDC_DMA_RX_0_Voice Mixer"},

	{"RX_CDC_DMA_RX_1_Voice Mixer", "Voip", "VOIP_DL"},
	{"RX_CDC_DMA_RX_1_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"RX_CDC_DMA_RX_1_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"RX_CDC_DMA_RX_1", NULL, "RX_CDC_DMA_RX_1_Voice Mixer"},

	{"VOIP_UL", NULL, "VOC_EXT_EC MUX"},
	{"VOICEMMODE1_UL", NULL, "VOC_EXT_EC MUX"},
	{"VOICEMMODE2_UL", NULL, "VOC_EXT_EC MUX"},

	{"MM_UL1", NULL, "AUDIO_REF_EC_UL1 MUX"},
	{"MM_UL2", NULL, "AUDIO_REF_EC_UL2 MUX"},
	{"MM_UL3", NULL, "AUDIO_REF_EC_UL3 MUX"},
	{"MM_UL4", NULL, "AUDIO_REF_EC_UL4 MUX"},

	{"VoiceMMode1_Tx Mixer", "USB_AUDIO_TX_MMode1", "USB_AUDIO_TX"},
	{"VoiceMMode1_Tx Mixer", "AFE_PCM_TX_MMode1", "PCM_TX"},
	{"VoiceMMode1_Tx Mixer", "PROXY_TX_MMode1", "PROXY_TX"},
	{"VoiceMMode1_Tx Mixer", "TX_CDC_DMA_TX_0_MMode1", "TX_CDC_DMA_TX_0"},
	{"VoiceMMode1_Tx Mixer", "TX_CDC_DMA_TX_1_MMode1", "TX_CDC_DMA_TX_1"},
	{"VoiceMMode1_Tx Mixer", "TX_CDC_DMA_TX_2_MMode1", "TX_CDC_DMA_TX_2"},
	{"VoiceMMode1_Tx Mixer", "TX_CDC_DMA_TX_3_MMode1", "TX_CDC_DMA_TX_3"},
	{"VoiceMMode1_Tx Mixer", "TX_CDC_DMA_TX_4_MMode1", "TX_CDC_DMA_TX_4"},
	{"VoiceMMode1_Tx Mixer", "TX_CDC_DMA_TX_5_MMode1", "TX_CDC_DMA_TX_5"},
	{"VOICEMMODE1_UL", NULL, "VoiceMMode1_Tx Mixer"},

	{"VoiceMMode2_Tx Mixer", "PROXY_TX_MMode2", "PROXY_TX"},
	{"VoiceMMode2_Tx Mixer", "TX_CDC_DMA_TX_0_MMode2", "TX_CDC_DMA_TX_0"},
	{"VoiceMMode2_Tx Mixer", "TX_CDC_DMA_TX_1_MMode2", "TX_CDC_DMA_TX_1"},
	{"VoiceMMode2_Tx Mixer", "TX_CDC_DMA_TX_2_MMode2", "TX_CDC_DMA_TX_2"},
	{"VoiceMMode2_Tx Mixer", "TX_CDC_DMA_TX_3_MMode2", "TX_CDC_DMA_TX_3"},
	{"VoiceMMode2_Tx Mixer", "TX_CDC_DMA_TX_4_MMode2", "TX_CDC_DMA_TX_4"},
	{"VoiceMMode2_Tx Mixer", "TX_CDC_DMA_TX_5_MMode2", "TX_CDC_DMA_TX_5"},
	{"VOICEMMODE2_UL", NULL, "VoiceMMode2_Tx Mixer"},

	{"Voip_Tx Mixer", "USB_AUDIO_TX_Voip", "USB_AUDIO_TX"},
	{"Voip_Tx Mixer", "TX_CDC_DMA_TX_0_Voip", "TX_CDC_DMA_TX_0"},
	{"Voip_Tx Mixer", "TX_CDC_DMA_TX_1_Voip", "TX_CDC_DMA_TX_1"},
	{"Voip_Tx Mixer", "TX_CDC_DMA_TX_2_Voip", "TX_CDC_DMA_TX_2"},
	{"Voip_Tx Mixer", "TX_CDC_DMA_TX_3_Voip", "TX_CDC_DMA_TX_3"},
	{"Voip_Tx Mixer", "TX_CDC_DMA_TX_4_Voip", "TX_CDC_DMA_TX_4"},
	{"Voip_Tx Mixer", "TX_CDC_DMA_TX_5_Voip", "TX_CDC_DMA_TX_5"},
	{"Voip_Tx Mixer", "AFE_PCM_TX_Voip", "PCM_TX"},

	{"RX_CDC_DMA_RX_0_DL_HL", "Switch", "CDC_DMA_DL_HL"},
	{"RX_CDC_DMA_RX_0", NULL, "RX_CDC_DMA_RX_0_DL_HL"},
	{"TX3_CDC_DMA_UL_HL", NULL, "TX_CDC_DMA_TX_3"},

	/* connect to INT4_MI2S_DL_HL since same pcm_id */
	{"AFE_PCM_RX Port Mixer", "INTERNAL_FM_TX", "INT_FM_TX"},
	{"AFE_PCM_RX Port Mixer", "SLIM_1_TX", "SLIMBUS_1_TX"},
	{"PCM_RX", NULL, "AFE_PCM_RX Port Mixer"},
	{"USB_AUDIO_RX Port Mixer", "USB_AUDIO_TX", "USB_AUDIO_TX"},
	{"USB_AUDIO_RX", NULL, "USB_AUDIO_RX Port Mixer"},
	{"USB_DL_HL", "Switch", "USBAUDIO_DL_HL"},
	{"USB_AUDIO_RX", NULL, "USB_DL_HL"},
	{"USBAUDIO_UL_HL", NULL, "USB_AUDIO_TX"},

	{"RX_CDC_DMA_RX_0 Port Mixer", "TX_CDC_DMA_TX_3", "TX_CDC_DMA_TX_3"},
	{"RX_CDC_DMA_RX_0", NULL, "RX_CDC_DMA_RX_0 Port Mixer"},

	{"Voice Stub Tx Mixer", "STUB_TX_HL", "STUB_TX"},
	{"Voice Stub Tx Mixer", "INTERNAL_BT_SCO_TX", "INT_BT_SCO_TX"},
	{"Voice Stub Tx Mixer", "STUB_1_TX_HL", "STUB_1_TX"},
	{"Voice Stub Tx Mixer", "AFE_PCM_TX", "PCM_TX"},
	{"VOICE_STUB_UL", NULL, "Voice Stub Tx Mixer"},

	{"VoLTE Stub Tx Mixer", "STUB_TX_HL", "STUB_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"VoLTE Stub Tx Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
#endif
	{"VoLTE Stub Tx Mixer", "AFE_PCM_TX", "PCM_TX"},
	{"VOLTE_STUB_UL", NULL, "VoLTE Stub Tx Mixer"},

	{"Voice2 Stub Tx Mixer", "AFE_PCM_TX", "PCM_TX"},
	{"VOICE2_STUB_UL", NULL, "Voice2 Stub Tx Mixer"},

	{"STUB_RX Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"STUB_RX Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"STUB_RX", NULL, "STUB_RX Mixer"},

	/* Backend Enablement */

	{"BE_OUT", NULL, "USB_AUDIO_RX"},
	{"BE_OUT", NULL, "INT_FM_RX"},
	{"BE_OUT", NULL, "PCM_RX"},
	{"BE_OUT", NULL, "INT_BT_SCO_RX"},
	{"BE_OUT", NULL, "INT_FM_RX"},
	{"BE_OUT", NULL, "VOICE_PLAYBACK_TX"},
	{"BE_OUT", NULL, "VOICE2_PLAYBACK_TX"},
	{"BE_OUT", NULL, "PROXY_RX"},

	{"BE_OUT", NULL, "WSA_CDC_DMA_RX_0"},
	{"BE_OUT", NULL, "WSA_CDC_DMA_RX_1"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_0"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_1"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_2"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_3"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_4"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_5"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_6"},
	{"BE_OUT", NULL, "RX_CDC_DMA_RX_7"},

	{"USB_AUDIO_TX", NULL, "BE_IN" },
	{"INT_BT_SCO_TX", NULL, "BE_IN"},
	{"INT_FM_TX", NULL, "BE_IN"},
	{"PCM_TX", NULL, "BE_IN"},
	{"BE_OUT", NULL, "STUB_RX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"BE_OUT", NULL, "AUX_PCM_RX"},
#endif
	{"INCALL_RECORD_TX", NULL, "BE_IN"},
	{"INCALL_RECORD_RX", NULL, "BE_IN"},
	{"AFE_LOOPBACK_TX", NULL, "BE_IN"},
	{"PROXY_TX", NULL, "BE_IN"},

	{"WSA_CDC_DMA_TX_0", NULL, "BE_IN"},
	{"WSA_CDC_DMA_TX_1", NULL, "BE_IN"},
	{"WSA_CDC_DMA_TX_2", NULL, "BE_IN"},
	{"VA_CDC_DMA_TX_0", NULL, "BE_IN"},
	{"VA_CDC_DMA_TX_1", NULL, "BE_IN"},
	{"VA_CDC_DMA_TX_2", NULL, "BE_IN"},
	{"TX_CDC_DMA_TX_0", NULL, "BE_IN"},
	{"TX_CDC_DMA_TX_1", NULL, "BE_IN"},
	{"TX_CDC_DMA_TX_2", NULL, "BE_IN"},
	{"TX_CDC_DMA_TX_3", NULL, "BE_IN"},
	{"TX_CDC_DMA_TX_4", NULL, "BE_IN"},
	{"TX_CDC_DMA_TX_5", NULL, "BE_IN"},
};

#ifndef CONFIG_AUXPCM_DISABLE
static const struct snd_soc_dapm_route intercon_aux_pcm[] = {
	/* incall */
	{"MultiMedia1 Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"MultiMedia3 Mixer", "AUX_PCM_TX", "AUX_PCM_TX"},
	{"MultiMedia1 Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
	{"MultiMedia2 Mixer", "SEC_AUX_PCM_TX", "SEC_AUX_PCM_TX"},
	{"MultiMedia3 Mixer", "SEC_AUX_PCM_TX", "SEC_AUX_PCM_TX"},


	{"AUX_PCM_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"AUX_PCM_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"AUX_PCM_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"AUX_PCM_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"AUX_PCM_RX", NULL, "AUX_PCM_RX Audio Mixer"},

	{"SEC_AUX_PCM_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_AUX_PCM_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_AUX_PCM_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_AUX_PCM_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_AUX_PCM_RX", NULL, "SEC_AUX_PCM_RX Audio Mixer"},

	{"AUX_PCM_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"AUX_PCM_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"AUX_PCM_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"AUX_PCM_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"AUX_PCM_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"AUX_PCM_RX", NULL, "AUX_PCM_RX_Voice Mixer"},

	{"SEC_AUX_PCM_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"SEC_AUX_PCM_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"SEC_AUX_PCM_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"SEC_AUX_PCM_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"SEC_AUX_PCM_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"SEC_AUX_PCM_RX", NULL, "SEC_AUX_PCM_RX_Voice Mixer"},

	{"VoiceMMode1_Tx Mixer", "AUX_PCM_TX_MMode1", "AUX_PCM_TX"},
	{"VoiceMMode1_Tx Mixer", "SEC_AUX_PCM_TX_MMode1", "SEC_AUX_PCM_TX"},
	{"VoiceMMode1_Tx Mixer", "TERT_AUX_PCM_TX_MMode1", "TERT_AUX_PCM_TX"},
	{"VoiceMMode1_Tx Mixer", "QUAT_AUX_PCM_TX_MMode1", "QUAT_AUX_PCM_TX"},

	{"VoiceMMode2_Tx Mixer", "AUX_PCM_TX_MMode2", "AUX_PCM_TX"},
	{"VoiceMMode2_Tx Mixer", "SEC_AUX_PCM_TX_MMode2", "SEC_AUX_PCM_TX"},
	{"VoiceMMode2_Tx Mixer", "TERT_AUX_PCM_TX_MMode1", "TERT_AUX_PCM_TX"},
	{"VoiceMMode2_Tx Mixer", "QUAT_AUX_PCM_TX_MMode1", "QUAT_AUX_PCM_TX"},

	{"Voip_Tx Mixer", "AUX_PCM_TX_Voip", "AUX_PCM_TX"},
	{"Voip_Tx Mixer", "SEC_AUX_PCM_TX_Voip", "SEC_AUX_PCM_TX"},

	{"QCHAT_Tx Mixer", "AUX_PCM_TX_QCHAT", "AUX_PCM_TX"},
	{"QCHAT_Tx Mixer", "SEC_AUX_PCM_TX_QCHAT", "SEC_AUX_PCM_TX"},

	{"Voice2 Stub Tx Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},

	/* connect to INT4_MI2S_DL_HL since same pcm_id */
	{"INTHFP_UL_HL", NULL, "HFP_PRI_AUX_UL_HL"},
	{"HFP_PRI_AUX_UL_HL", "Switch", "AUX_PCM_TX"},
	{"INTHFP_UL_HL", NULL, "HFP_AUX_UL_HL"},
	{"HFP_AUX_UL_HL", "Switch", "SEC_AUX_PCM_TX"},
	{"AUX_PCM_RX", NULL, "AUXPCM_DL_HL"},
	{"AUX_PCM_RX", NULL, "INTHFP_DL_HL"},
	{"SEC_AUX_PCM_RX", NULL, "SEC_AUXPCM_DL_HL"},
	{"AUXPCM_UL_HL", NULL, "AUX_PCM_TX"},
	{"SEC_AUXPCM_UL_HL", NULL, "SEC_AUX_PCM_TX"},

	{"AUX_PCM_RX Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"AUX_PCM_RX Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
	{"AUX_PCM_RX Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
	{"AUX_PCM_RX", NULL, "AUX_PCM_RX Port Mixer"},

	{"SEC_AUXPCM_RX Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"SEC_AUXPCM_RX Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
	{"SEC_AUXPCM_RX Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"SEC_AUX_PCM_RX", NULL, "SEC_AUXPCM_RX Port Mixer"},

	{"Voice Stub Tx Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"Voice Stub Tx Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},

	/* Backend Enablement */
	{"BE_OUT", NULL, "AUX_PCM_RX"},
	{"BE_OUT", NULL, "SEC_AUX_PCM_RX"},

	{"AUX_PCM_TX", NULL, "BE_IN"},
	{"SEC_AUX_PCM_TX", NULL, "BE_IN"},
};
#endif

#ifndef CONFIG_TDM_DISABLE
static const struct snd_soc_dapm_route intercon_tdm[] = {
	/* incall */
	{"PRI_TDM_RX_0 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_TDM_RX_0 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"PRI_TDM_RX_0 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"PRI_TDM_RX_0 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PRI_TDM_RX_0", NULL, "PRI_TDM_RX_0 Audio Mixer"},

	{"PRI_TDM_RX_1 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_TDM_RX_1 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"PRI_TDM_RX_1 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"PRI_TDM_RX_1 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PRI_TDM_RX_1", NULL, "PRI_TDM_RX_1 Audio Mixer"},

	{"PRI_TDM_RX_2 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_TDM_RX_2 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"PRI_TDM_RX_2 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"PRI_TDM_RX_2 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PRI_TDM_RX_2", NULL, "PRI_TDM_RX_2 Audio Mixer"},

	{"PRI_TDM_RX_3 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_TDM_RX_3 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"PRI_TDM_RX_3 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"PRI_TDM_RX_3 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PRI_TDM_RX_3", NULL, "PRI_TDM_RX_3 Audio Mixer"},

	{"SEC_TDM_RX_0 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_TDM_RX_0 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_TDM_RX_0 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_TDM_RX_0 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_TDM_RX_0", NULL, "SEC_TDM_RX_0 Audio Mixer"},

	{"SEC_TDM_RX_1 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_TDM_RX_1 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_TDM_RX_1 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_TDM_RX_1 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_TDM_RX_1", NULL, "SEC_TDM_RX_1 Audio Mixer"},

	{"SEC_TDM_RX_2 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_TDM_RX_2 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_TDM_RX_2 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_TDM_RX_2 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_TDM_RX_2", NULL, "SEC_TDM_RX_2 Audio Mixer"},

	{"SEC_TDM_RX_3 Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_TDM_RX_3 Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_TDM_RX_3 Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_TDM_RX_3 Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_TDM_RX_3", NULL, "SEC_TDM_RX_3 Audio Mixer"},

	{"MultiMedia1 Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"MultiMedia1 Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"MultiMedia1 Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"MultiMedia1 Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"MultiMedia1 Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"MultiMedia1 Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"MultiMedia1 Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"MultiMedia1 Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},

	{"MultiMedia2 Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"MultiMedia2 Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"MultiMedia2 Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"MultiMedia2 Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"MultiMedia2 Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"MultiMedia2 Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"MultiMedia2 Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"MultiMedia2 Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},

	{"MultiMedia3 Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"MultiMedia3 Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"MultiMedia3 Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"MultiMedia3 Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"MultiMedia3 Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"MultiMedia3 Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"MultiMedia3 Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"MultiMedia3 Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},

	{"MultiMedia4 Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"MultiMedia4 Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"MultiMedia4 Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"MultiMedia4 Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"MultiMedia4 Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"MultiMedia4 Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"MultiMedia4 Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"MultiMedia4 Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},

	{"PRI_TDM_RX_0_Voice Mixer", "Voip", "VOIP_DL"},
	{"PRI_TDM_RX_0_Voice Mixer", "VoLTE Stub", "VOLTE_STUB_DL"},
	{"PRI_TDM_RX_0_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"PRI_TDM_RX_0_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"PRI_TDM_RX_0_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"PRI_TDM_RX_0_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"PRI_TDM_RX_0_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PRI_TDM_RX_0_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PRI_TDM_RX_0", NULL, "PRI_TDM_RX_0_Voice Mixer"},

	{"PRI_TDM_RX_1_Voice Mixer", "Voip", "VOIP_DL"},
	{"PRI_TDM_RX_1_Voice Mixer", "VoLTE Stub", "VOLTE_STUB_DL"},
	{"PRI_TDM_RX_1_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"PRI_TDM_RX_1_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"PRI_TDM_RX_1_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"PRI_TDM_RX_1_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"PRI_TDM_RX_1_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PRI_TDM_RX_1_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PRI_TDM_RX_1", NULL, "PRI_TDM_RX_1_Voice Mixer"},

	{"PRI_TDM_RX_2_Voice Mixer", "Voip", "VOIP_DL"},
	{"PRI_TDM_RX_2_Voice Mixer", "VoLTE Stub", "VOLTE_STUB_DL"},
	{"PRI_TDM_RX_2_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"PRI_TDM_RX_2_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"PRI_TDM_RX_2_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"PRI_TDM_RX_2_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"PRI_TDM_RX_2_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PRI_TDM_RX_2_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PRI_TDM_RX_2", NULL, "PRI_TDM_RX_2_Voice Mixer"},

	{"PRI_TDM_RX_3_Voice Mixer", "Voip", "VOIP_DL"},
	{"PRI_TDM_RX_3_Voice Mixer", "VoLTE Stub", "VOLTE_STUB_DL"},
	{"PRI_TDM_RX_3_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"PRI_TDM_RX_3_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"PRI_TDM_RX_3_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"PRI_TDM_RX_3_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"PRI_TDM_RX_3_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PRI_TDM_RX_3_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PRI_TDM_RX_3", NULL, "PRI_TDM_RX_3_Voice Mixer"},

	{"SEC_TDM_RX_0_Voice Mixer", "Voip", "VOIP_DL"},
	{"SEC_TDM_RX_0_Voice Mixer", "VoLTE Stub", "VOLTE_STUB_DL"},
	{"SEC_TDM_RX_0_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"SEC_TDM_RX_0_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"SEC_TDM_RX_0_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"SEC_TDM_RX_0_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"SEC_TDM_RX_0_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"SEC_TDM_RX_0_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"SEC_TDM_RX_0", NULL, "SEC_TDM_RX_0_Voice Mixer"},

	{"SEC_TDM_RX_1_Voice Mixer", "Voip", "VOIP_DL"},
	{"SEC_TDM_RX_1_Voice Mixer", "VoLTE Stub", "VOLTE_STUB_DL"},
	{"SEC_TDM_RX_1_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"SEC_TDM_RX_1_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"SEC_TDM_RX_1_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"SEC_TDM_RX_1_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"SEC_TDM_RX_1_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"SEC_TDM_RX_1_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"SEC_TDM_RX_1", NULL, "SEC_TDM_RX_1_Voice Mixer"},

	{"VoiceMMode1_Tx Mixer", "PRI_TDM_TX_0_MMode1", "PRI_TDM_TX_0"},
	{"VoiceMMode1_Tx Mixer", "PRI_TDM_TX_1_MMode1", "PRI_TDM_TX_1"},
	{"VoiceMMode1_Tx Mixer", "SEC_TDM_TX_0_MMode1", "SEC_TDM_TX_0"},
	{"VoiceMMode1_Tx Mixer", "SEC_TDM_TX_1_MMode1", "SEC_TDM_TX_1"},

	{"VoiceMMode2_Tx Mixer", "PRI_TDM_TX_0_MMode1", "PRI_TDM_TX_0"},
	{"VoiceMMode2_Tx Mixer", "PRI_TDM_TX_1_MMode1", "PRI_TDM_TX_1"},
	{"VoiceMMode2_Tx Mixer", "SEC_TDM_TX_0_MMode1", "SEC_TDM_TX_0"},
	{"VoiceMMode2_Tx Mixer", "SEC_TDM_TX_1_MMode1", "SEC_TDM_TX_1"},

	{"VOC_EXT_EC MUX", "PRI_TDM_TX",   "PRI_TDM_TX_0"},
	{"VOC_EXT_EC MUX", "SEC_TDM_TX",   "SEC_TDM_TX_0"},

	/* connect to INT4_MI2S_DL_HL since same pcm_id */
	{"PRI_TDM_TX_0_UL_HL", NULL, "PRI_TDM_TX_0"},
	{"PRI_TDM_TX_1_UL_HL", NULL, "PRI_TDM_TX_1"},
	{"PRI_TDM_TX_2_UL_HL", NULL, "PRI_TDM_TX_2"},
	{"PRI_TDM_TX_3_UL_HL", NULL, "PRI_TDM_TX_3"},
	{"PRI_TDM_RX_0", NULL, "PRI_TDM_RX_0_DL_HL"},
	{"PRI_TDM_RX_1", NULL, "PRI_TDM_RX_1_DL_HL"},
	{"PRI_TDM_RX_2", NULL, "PRI_TDM_RX_2_DL_HL"},
	{"PRI_TDM_RX_3", NULL, "PRI_TDM_RX_3_DL_HL"},
	{"SEC_TDM_TX_0_UL_HL", NULL, "SEC_TDM_TX_0"},
	{"SEC_TDM_TX_1_UL_HL", NULL, "SEC_TDM_TX_1"},
	{"SEC_TDM_TX_2_UL_HL", NULL, "SEC_TDM_TX_2"},
	{"SEC_TDM_TX_3_UL_HL", NULL, "SEC_TDM_TX_3"},
	{"SEC_TDM_RX_0", NULL, "SEC_TDM_RX_0_DL_HL"},
	{"SEC_TDM_RX_1", NULL, "SEC_TDM_RX_1_DL_HL"},
	{"SEC_TDM_RX_2", NULL, "SEC_TDM_RX_2_DL_HL"},
	{"SEC_TDM_RX_3", NULL, "SEC_TDM_RX_3_DL_HL"},
	{"SEC_TDM_RX_7", NULL, "SEC_TDM_RX_7_DL_HL"},

#ifndef CONFIG_MI2S_DISABLE
	{"PRI_TDM_RX_0 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"PRI_TDM_RX_0 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"PRI_TDM_RX_0 Port Mixer", "INTERNAL_FM_TX", "INT_FM_TX"},
	{"PRI_TDM_RX_0 Port Mixer", "INTERNAL_BT_SCO_TX", "INT_BT_SCO_TX"},
	{"PRI_TDM_RX_0 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"PRI_TDM_RX_0 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"PRI_TDM_RX_0 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"PRI_TDM_RX_0 Port Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"PRI_TDM_RX_0 Port Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"PRI_TDM_RX_0 Port Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"PRI_TDM_RX_0 Port Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"PRI_TDM_RX_0", NULL, "PRI_TDM_RX_0 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"PRI_TDM_RX_1 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"PRI_TDM_RX_1 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"PRI_TDM_RX_1 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"PRI_TDM_RX_1 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"PRI_TDM_RX_1 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"PRI_TDM_RX_1 Port Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"PRI_TDM_RX_1 Port Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"PRI_TDM_RX_1 Port Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"PRI_TDM_RX_1 Port Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"PRI_TDM_RX_1", NULL, "PRI_TDM_RX_1 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"PRI_TDM_RX_2 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"PRI_TDM_RX_2 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"PRI_TDM_RX_2 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"PRI_TDM_RX_2 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"PRI_TDM_RX_2 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"PRI_TDM_RX_2 Port Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"PRI_TDM_RX_2 Port Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"PRI_TDM_RX_2 Port Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"PRI_TDM_RX_2 Port Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"PRI_TDM_RX_2", NULL, "PRI_TDM_RX_2 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"PRI_TDM_RX_3 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"PRI_TDM_RX_3 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"PRI_TDM_RX_3 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"PRI_TDM_RX_3 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"PRI_TDM_RX_3 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"PRI_TDM_RX_3 Port Mixer", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},
	{"PRI_TDM_RX_3 Port Mixer", "PRI_TDM_TX_1", "PRI_TDM_TX_1"},
	{"PRI_TDM_RX_3 Port Mixer", "PRI_TDM_TX_2", "PRI_TDM_TX_2"},
	{"PRI_TDM_RX_3 Port Mixer", "PRI_TDM_TX_3", "PRI_TDM_TX_3"},
	{"PRI_TDM_RX_3", NULL, "PRI_TDM_RX_3 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"SEC_TDM_RX_0 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"SEC_TDM_RX_0 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"SEC_TDM_RX_0 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"SEC_TDM_RX_0 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"SEC_TDM_RX_0 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"SEC_TDM_RX_0 Port Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"SEC_TDM_RX_0 Port Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"SEC_TDM_RX_0 Port Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"SEC_TDM_RX_0 Port Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},
	{"SEC_TDM_RX_0", NULL, "SEC_TDM_RX_0 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"SEC_TDM_RX_1 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"SEC_TDM_RX_1 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"SEC_TDM_RX_1 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"SEC_TDM_RX_1 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"SEC_TDM_RX_1 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"SEC_TDM_RX_1 Port Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"SEC_TDM_RX_1 Port Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"SEC_TDM_RX_1 Port Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"SEC_TDM_RX_1 Port Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},
	{"SEC_TDM_RX_1", NULL, "SEC_TDM_RX_1 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"SEC_TDM_RX_2 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"SEC_TDM_RX_2 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"SEC_TDM_RX_2 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"SEC_TDM_RX_2 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"SEC_TDM_RX_2 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"SEC_TDM_RX_2 Port Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"SEC_TDM_RX_2 Port Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"SEC_TDM_RX_2 Port Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"SEC_TDM_RX_2 Port Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},
	{"SEC_TDM_RX_2", NULL, "SEC_TDM_RX_2 Port Mixer"},

#ifndef CONFIG_MI2S_DISABLE
	{"SEC_TDM_RX_3 Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"SEC_TDM_RX_3 Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#endif
	{"SEC_TDM_RX_3 Port Mixer", "AFE_PCM_TX", "PCM_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"SEC_TDM_RX_3 Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
	{"SEC_TDM_RX_3 Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"SEC_TDM_RX_3 Port Mixer", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"SEC_TDM_RX_3 Port Mixer", "SEC_TDM_TX_1", "SEC_TDM_TX_1"},
	{"SEC_TDM_RX_3 Port Mixer", "SEC_TDM_TX_2", "SEC_TDM_TX_2"},
	{"SEC_TDM_RX_3 Port Mixer", "SEC_TDM_TX_3", "SEC_TDM_TX_3"},
	{"SEC_TDM_RX_3", NULL, "SEC_TDM_RX_3 Port Mixer"},


	{"AUDIO_REF_EC_UL1 MUX", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"AUDIO_REF_EC_UL1 MUX", "PRI_TDM_RX_0", "PRI_TDM_RX_0"},
	{"AUDIO_REF_EC_UL1 MUX", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},

	{"AUDIO_REF_EC_UL10 MUX", "SEC_TDM_TX_0", "SEC_TDM_TX_0"},
	{"AUDIO_REF_EC_UL10 MUX", "PRI_TDM_RX_0", "PRI_TDM_RX_0"},
	{"AUDIO_REF_EC_UL10 MUX", "PRI_TDM_TX_0", "PRI_TDM_TX_0"},

	/* Backend Enablement */
	{"BE_OUT", NULL, "PRI_TDM_RX_0"},
	{"BE_OUT", NULL, "PRI_TDM_RX_1"},
	{"BE_OUT", NULL, "PRI_TDM_RX_2"},
	{"BE_OUT", NULL, "PRI_TDM_RX_3"},
	{"BE_OUT", NULL, "SEC_TDM_RX_0"},
	{"BE_OUT", NULL, "SEC_TDM_RX_1"},
	{"BE_OUT", NULL, "SEC_TDM_RX_2"},
	{"BE_OUT", NULL, "SEC_TDM_RX_3"},
	{"BE_OUT", NULL, "SEC_TDM_RX_7"},

	{"PRI_TDM_TX_0", NULL, "BE_IN"},
	{"PRI_TDM_TX_1", NULL, "BE_IN"},
	{"PRI_TDM_TX_2", NULL, "BE_IN"},
	{"PRI_TDM_TX_3", NULL, "BE_IN"},
	{"SEC_TDM_TX_0", NULL, "BE_IN"},
	{"SEC_TDM_TX_1", NULL, "BE_IN"},
	{"SEC_TDM_TX_2", NULL, "BE_IN"},
	{"SEC_TDM_TX_3", NULL, "BE_IN"},
};
#endif

#ifndef CONFIG_MI2S_DISABLE
static const struct snd_soc_dapm_route intercon_mi2s[] = {
	{"PRI_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"PRI_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"PRI_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PRI_I2S_RX", NULL, "PRI_RX Audio Mixer"},

	{"SEC_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_I2S_RX", NULL, "SEC_RX Audio Mixer"},

	/* incall */
	{"MultiMedia1 Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"MultiMedia2 Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"MultiMedia3 Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"MultiMedia4 Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},


	{"SEC_MI2S_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"SEC_MI2S_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"SEC_MI2S_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"SEC_MI2S_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"SEC_MI2S_RX", NULL, "SEC_MI2S_RX Audio Mixer"},


	{"SEC_MI2S_RX Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},

	{"PRI_MI2S_RX Audio Mixer", "MultiMedia1", "MM_DL1"},
	{"PRI_MI2S_RX Audio Mixer", "MultiMedia2", "MM_DL2"},
	{"PRI_MI2S_RX Audio Mixer", "MultiMedia3", "MM_DL3"},
	{"PRI_MI2S_RX Audio Mixer", "MultiMedia4", "MM_DL4"},
	{"PRI_MI2S_RX", NULL, "PRI_MI2S_RX Audio Mixer"},
	{"PRI_MI2S_RX Audio Mixer", "DTMF", "DTMF_DL_HL"},

	{"MultiMedia1 Mixer", "PRI_TX", "PRI_I2S_TX"},

	{"PRI_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"PRI_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"PRI_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"PRI_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PRI_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PRI_I2S_RX", NULL, "PRI_RX_Voice Mixer"},

	{"SEC_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"SEC_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"SEC_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"SEC_I2S_RX", NULL, "SEC_RX_Voice Mixer"},

	{"SEC_MI2S_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"SEC_MI2S_RX_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"SEC_MI2S_RX_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"SEC_MI2S_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"SEC_MI2S_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"SEC_MI2S_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"SEC_MI2S_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"SEC_MI2S_RX", NULL, "SEC_MI2S_RX_Voice Mixer"},

	{"PRI_MI2S_RX_Voice Mixer", "Voip", "VOIP_DL"},
	{"PRI_MI2S_RX_Voice Mixer", "Voice Stub", "VOICE_STUB_DL"},
	{"PRI_MI2S_RX_Voice Mixer", "Voice2 Stub", "VOICE2_STUB_DL"},
	{"PRI_MI2S_RX_Voice Mixer", "QCHAT", "QCHAT_DL"},
	{"PRI_MI2S_RX_Voice Mixer", "DTMF", "DTMF_DL_HL"},
	{"PRI_MI2S_RX_Voice Mixer", "VoiceMMode1", "VOICEMMODE1_DL"},
	{"PRI_MI2S_RX_Voice Mixer", "VoiceMMode2", "VOICEMMODE2_DL"},
	{"PRI_MI2S_RX", NULL, "PRI_MI2S_RX_Voice Mixer"},

	{"VOC_EXT_EC MUX", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"VOC_EXT_EC MUX", "SEC_MI2S_TX", "SEC_MI2S_TX"},

	{"AUDIO_REF_EC_UL1 MUX", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"AUDIO_REF_EC_UL1 MUX", "SEC_MI2S_TX", "SEC_MI2S_TX"},

	{"AUDIO_REF_EC_UL2 MUX", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"AUDIO_REF_EC_UL2 MUX", "SEC_MI2S_TX", "SEC_MI2S_TX"},

	{"AUDIO_REF_EC_UL3 MUX", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"AUDIO_REF_EC_UL3 MUX", "SEC_MI2S_TX", "SEC_MI2S_TX"},

	{"AUDIO_REF_EC_UL4 MUX", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"AUDIO_REF_EC_UL4 MUX", "SEC_MI2S_TX", "SEC_MI2S_TX"},

	{"VoiceMMode1_Tx Mixer", "PRI_TX_MMode1", "PRI_I2S_TX"},

	{"VoiceMMode1_Tx Mixer", "PRI_MI2S_TX_MMode1", "PRI_MI2S_TX"},
	{"VoiceMMode1_Tx Mixer", "SEC_MI2S_TX_MMode1", "SEC_MI2S_TX"},
	{"VoiceMMode1_Tx Mixer", "TERT_MI2S_TX_MMode1", "TERT_MI2S_TX"},
	{"VoiceMMode1_Tx Mixer", "QUAT_MI2S_TX_MMode1", "QUAT_MI2S_TX"},

	{"VoiceMMode2_Tx Mixer", "PRI_MI2S_TX_MMode2", "PRI_MI2S_TX"},
	{"VoiceMMode2_Tx Mixer", "SEC_MI2S_TX_MMode2", "SEC_MI2S_TX"},
	{"VoiceMMode2_Tx Mixer", "TERT_MI2S_TX_MMode1", "TERT_MI2S_TX"},
	{"VoiceMMode2_Tx Mixer", "QUAT_MI2S_TX_MMode1", "QUAT_MI2S_TX"},

	{"Voip_Tx Mixer", "PRI_MI2S_TX_Voip", "PRI_MI2S_TX"},
	{"Voip_Tx Mixer", "SEC_MI2S_TX_Voip", "SEC_MI2S_TX"},

	{"QCHAT_Tx Mixer", "PRI_MI2S_TX_QCHAT", "PRI_MI2S_TX"},

	{"PRI_MI2S_RX_DL_HL", "Switch", "PRI_MI2S_DL_HL"},
	{"PRI_MI2S_RX", NULL, "PRI_MI2S_RX_DL_HL"},
	{"SEC_MI2S_RX_DL_HL", "Switch", "SEC_MI2S_DL_HL"},
	{"SEC_MI2S_RX", NULL, "SEC_MI2S_RX_DL_HL"},

	{"SEC_I2S_RX", NULL, "SEC_I2S_DL_HL"},
	{"PRI_MI2S_UL_HL", NULL, "PRI_MI2S_TX"},
	{"SEC_MI2S_UL_HL", NULL, "SEC_MI2S_TX"},
	{"SEC_MI2S_RX", NULL, "SEC_MI2S_DL_HL"},
	{"PRI_MI2S_RX", NULL, "PRI_MI2S_DL_HL"},

	{"Voice Stub Tx Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"Voice2 Stub Tx Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"VoLTE Stub Tx Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},

	{"PRI_MI2S_RX Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"PRI_MI2S_RX Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"PRI_MI2S_RX Port Mixer", "SEC_AUX_PCM_UL_TX", "SEC_AUX_PCM_TX"},
#endif
	{"PRI_MI2S_RX", NULL, "PRI_MI2S_RX Port Mixer"},

	{"SEC_MI2S_RX Port Mixer", "PRI_MI2S_TX", "PRI_MI2S_TX"},
	{"SEC_MI2S_RX Port Mixer", "SEC_MI2S_TX", "SEC_MI2S_TX"},
#ifndef CONFIG_AUXPCM_DISABLE
	{"SEC_MI2S_RX Port Mixer", "AUX_PCM_UL_TX", "AUX_PCM_TX"},
#endif
	{"SEC_MI2S_RX", NULL, "SEC_MI2S_RX Port Mixer"},

	/* Backend Enablement */
	{"BE_OUT", NULL, "PRI_I2S_RX"},
	{"BE_OUT", NULL, "SEC_I2S_RX"},

	{"PRI_MI2S_TX", NULL, "BE_IN"},
	{"SEC_MI2S_TX", NULL, "BE_IN"},

	{"PRI_MI2S_RX", NULL, "PRI_MI2S_RX_VI_FB_MUX"},
};
#endif

static int msm_pcm_routing_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->dai_link->id;

	if (be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: unexpected BE id %d\n", __func__, be_id);
		return -EINVAL;
	}

	mutex_lock(&routing_lock);
	msm_bedais[be_id].sample_rate = params_rate(params);
	msm_bedais[be_id].channel = params_channels(params);
	msm_bedais[be_id].format = params_format(params);
	pr_debug("%s: BE Sample Rate (%d) format (%d) BE id %d\n",
		__func__, msm_bedais[be_id].sample_rate,
		msm_bedais[be_id].format, be_id);
	mutex_unlock(&routing_lock);
	return 0;
}

static int msm_pcm_routing_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->dai_link->id;
	int i, session_type, path_type, topology;
	struct msm_pcm_routing_bdai_data *bedai;
	struct msm_pcm_routing_fdai_data *fdai;

	pr_debug("%s: substream->pcm->id:%s\n",
		 __func__, substream->pcm->id);

	if (be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: unexpected BE id %d\n", __func__, be_id);
		return -EINVAL;
	}

	bedai = &msm_bedais[be_id];
	session_type = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		0 : 1);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		path_type = ADM_PATH_PLAYBACK;
	else
		path_type = ADM_PATH_LIVE_REC;

	mutex_lock(&routing_lock);
	for_each_set_bit(i, &bedai->fe_sessions[0], MSM_FRONTEND_DAI_MAX) {
		fdai = &fe_dai_map[i][session_type];
		if (fdai->strm_id != INVALID_SESSION) {
			int idx;
			int port_id;
			unsigned long copp =
				session_copp_map[i][session_type][be_id];
			for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++)
				if (test_bit(idx, &copp))
					break;

			if (idx >= MAX_COPPS_PER_PORT) {
				pr_debug("%s: copp idx is invalid, exiting\n",
								__func__);
				continue;
			}
			fdai->be_srate = bedai->sample_rate;
			port_id = get_port_id(bedai->port_id);
			topology = adm_get_topology_for_port_copp_idx(port_id,
								     idx);
			msm_routing_unload_topology(topology);
			adm_close(port_id, fdai->perf_mode, idx);
			pr_debug("%s: copp:%ld,idx bit fe:%d, type:%d,be:%d topology=0x%x\n",
				 __func__, copp, i, session_type, be_id,
				 topology);
			clear_bit(idx,
				  &session_copp_map[i][session_type][be_id]);
		}
	}

	bedai->active = 0;
	bedai->sample_rate = 0;
	bedai->channel = 0;
	mutex_unlock(&routing_lock);

	return 0;
}

static int msm_pcm_routing_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int be_id = rtd->dai_link->id;
	int i, path_type, topology;
	int session_type = INVALID_SESSION;
	struct msm_pcm_routing_bdai_data *bedai;
	u32 channels, sample_rate;
	uint16_t bits_per_sample = 16, voc_path_type, be_bit_width;
	struct msm_pcm_routing_fdai_data *fdai;
	u32 session_id;
	struct media_format_info voc_be_media_format;
	uint32_t copp_token = 0;

	pr_debug("%s: substream->pcm->id:%s\n",
		 __func__, substream->pcm->id);

	if (be_id >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: unexpected BE id %d\n", __func__, be_id);
		return -EINVAL;
	}

	bedai = &msm_bedais[be_id];

	mutex_lock(&routing_lock);
	if (bedai->active == 1)
		goto done; /* Ignore prepare if back-end already active */

	/* AFE port is not active at this point. However, still
	 * go ahead setting active flag under the notion that
	 * QDSP6 is able to handle ADM starting before AFE port
	 * is started.
	 */
	bedai->active = 1;

	for_each_set_bit(i, &bedai->fe_sessions[0], MSM_FRONTEND_DAI_MAX) {

		session_type = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
						SESSION_TYPE_RX : SESSION_TYPE_TX;
		fdai = &fe_dai_map[i][session_type];
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (fdai->passthr_mode != LEGACY_PCM)
				path_type = ADM_PATH_COMPRESSED_RX;
			else
				path_type = ADM_PATH_PLAYBACK;
		} else {
			if ((fdai->passthr_mode != LEGACY_PCM) &&
			    (fdai->passthr_mode != LISTEN))
				path_type = ADM_PATH_COMPRESSED_TX;
			else
				path_type = ADM_PATH_LIVE_REC;
		}

		if (fdai->strm_id != INVALID_SESSION) {
			int app_type, app_type_idx, copp_idx, acdb_dev_id;
			int port_id = get_port_id(bedai->port_id);

			if (session_type == SESSION_TYPE_TX &&
			    fdai->be_srate &&
			    (fdai->be_srate != bedai->sample_rate)) {
				pr_debug("%s: flush strm %d diff BE rates\n",
					__func__,
					fdai->strm_id);

				if (fdai->event_info.event_func)
					fdai->event_info.event_func(
						MSM_PCM_RT_EVT_BUF_RECFG,
						fdai->event_info.priv_data);
				fdai->be_srate = 0; /* might not need it */
			}
			bits_per_sample = msm_routing_get_bit_width(
						bedai->format);

			app_type =
			fe_dai_app_type_cfg[i][session_type][be_id].app_type;
			if (app_type) {
				app_type_idx =
				msm_pcm_routing_get_app_type_idx(app_type);
				sample_rate =
					fe_dai_app_type_cfg[i][session_type]
							   [be_id].sample_rate;
				bits_per_sample =
					app_type_cfg[app_type_idx].bit_width;
				copp_token =
					fe_dai_app_type_cfg[i][session_type]
							[be_id].copp_token;
			} else
				sample_rate = bedai->sample_rate;
			/*
			 * check if ADM needs to be configured with different
			 * channel mapping than backend
			 */
			if (!bedai->adm_override_ch)
				channels = bedai->channel;
			else
				channels = bedai->adm_override_ch;
			acdb_dev_id =
			fe_dai_app_type_cfg[i][session_type][be_id].acdb_dev_id;
			topology = msm_routing_get_adm_topology(i, session_type,
								be_id);

			if ((fdai->passthr_mode == COMPRESSED_PASSTHROUGH_DSD)
				|| (fdai->passthr_mode == COMPRESSED_PASSTHROUGH_GEN)
				|| (fdai->passthr_mode == COMPRESSED_PASSTHROUGH_IEC61937))
				topology = COMPRESSED_PASSTHROUGH_NONE_TOPOLOGY;

			be_bit_width = msm_routing_get_bit_width(
						      bedai->format);

			if (hifi_filter_enabled && (bedai->sample_rate == 384000
				|| bedai->sample_rate == 352800) &&
				be_bit_width == 32)
				bits_per_sample = msm_routing_get_bit_width(
							SNDRV_PCM_FORMAT_S32_LE);
			copp_idx = adm_open(port_id, path_type,
					    sample_rate, channels, topology,
					    fdai->perf_mode, bits_per_sample,
					    app_type, acdb_dev_id,
					    session_type, fdai->passthr_mode, copp_token);
			if ((copp_idx < 0) ||
				(copp_idx >= MAX_COPPS_PER_PORT)) {
				pr_err("%s: adm open failed\n", __func__);
				mutex_unlock(&routing_lock);
				return -EINVAL;
			}
			pr_debug("%s: setting idx bit of fe:%d, type: %d, be:%d\n",
				 __func__, i, session_type, be_id);
			set_bit(copp_idx,
				&session_copp_map[i][session_type][be_id]);

			if (msm_is_resample_needed(
				sample_rate,
				bedai->sample_rate))
				adm_copp_mfc_cfg(port_id, copp_idx,
					bedai->sample_rate);

			msm_pcm_routing_build_matrix(i, session_type, path_type,
				fdai->perf_mode, fdai->passthr_mode);
		}
	}

	for_each_set_bit(i, &bedai->fe_sessions[0], MSM_FRONTEND_DAI_MAX) {
		session_id = msm_pcm_routing_get_voc_sessionid(i);
		if (session_id) {
			pr_debug("%s voice session_id: 0x%x\n", __func__,
				 session_id);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				voc_path_type = RX_PATH;
			else
				voc_path_type = TX_PATH;

			voc_set_route_flag(session_id, voc_path_type, 1);

			memset(&voc_be_media_format, 0,
				sizeof(struct media_format_info));

			voc_be_media_format.port_id = bedai->port_id;
			voc_be_media_format.num_channels = bedai->channel;
			voc_be_media_format.sample_rate = bedai->sample_rate;
			voc_be_media_format.bits_per_sample = bedai->format;
			/* Defaulting this to 1 for voice call usecases */
			voc_be_media_format.channel_mapping[0] = 1;

			voc_set_device_config(session_id, voc_path_type,
					      &voc_be_media_format);

			if (voc_get_route_flag(session_id, RX_PATH) &&
			    voc_get_route_flag(session_id, TX_PATH))
				voc_enable_device(session_id);
		}
	}

	/* Check if backend is an external ec ref port and set as needed */
	if (unlikely(bedai->port_id == voc_get_ext_ec_ref_port_id())) {

		memset(&voc_be_media_format, 0,
			sizeof(struct media_format_info));

		/* Get format info for ec ref port from msm_bedais[] */
		voc_be_media_format.port_id = bedai->port_id;
		voc_be_media_format.num_channels = bedai->channel;
		voc_be_media_format.bits_per_sample = bedai->format;
		voc_be_media_format.sample_rate = bedai->sample_rate;
		/* Defaulting this to 1 for voice call usecases */
		voc_be_media_format.channel_mapping[0] = 1;
		voc_set_ext_ec_ref_media_fmt_info(&voc_be_media_format);
		pr_debug("%s: EC Ref media format info set to port_id=%d, num_channels=%d, bits_per_sample=%d, sample_rate=%d\n",
			 __func__, voc_be_media_format.port_id,
			 voc_be_media_format.num_channels,
			 voc_be_media_format.bits_per_sample,
			 voc_be_media_format.sample_rate);
	}

done:
	mutex_unlock(&routing_lock);

	return 0;
}

static int msm_routing_send_device_pp_params(int port_id, int copp_idx,
					     int fe_id)
{
	int index, topo_id, be_idx;
	unsigned long pp_config = 0;
	bool mute_on;
	int latency, session_type;
	bool compr_passthr_mode = true;

	pr_debug("%s: port_id %d, copp_idx %d\n", __func__, port_id, copp_idx);

	if (port_id != HDMI_RX && port_id != DISPLAY_PORT_RX
			&& port_id != HDMI_RX_MS) {
		pr_err("%s: Device pp params on invalid port %d, copp_idx %d, fe_id %d\n",
			__func__, port_id, copp_idx, fe_id);
		return  -EINVAL;
	}

	for (be_idx = 0; be_idx < MSM_BACKEND_DAI_MAX; be_idx++) {
		if (port_id == msm_bedais[be_idx].port_id)
			break;
	}

	if (be_idx >= MSM_BACKEND_DAI_MAX) {
		pr_debug("%s: Invalid be id %d\n", __func__, be_idx);
		return  -EINVAL;
	}

	for (index = 0; index < MSM_BACKEND_DAI_PP_PARAMS_REQ_MAX; index++) {
		if (msm_bedais_pp_params[index].port_id == port_id)
			break;
	}
	if (index >= MSM_BACKEND_DAI_PP_PARAMS_REQ_MAX) {
		pr_err("%s: Invalid backend pp params index %d\n",
			__func__, index);
		return -EINVAL;
	}

	topo_id = adm_get_topology_for_port_copp_idx(port_id, copp_idx);
	if (topo_id != COMPRESSED_PASSTHROUGH_DEFAULT_TOPOLOGY) {
		pr_err("%s: Invalid passthrough topology 0x%x\n",
			__func__, topo_id);
		return -EINVAL;
	}

	session_type =
		(afe_get_port_type(port_id) == MSM_AFE_PORT_TYPE_RX) ?
		SESSION_TYPE_RX : SESSION_TYPE_TX;

	if ((fe_dai_map[fe_id][session_type].passthr_mode == LEGACY_PCM) ||
		(fe_dai_map[fe_id][session_type].passthr_mode == LISTEN))
		compr_passthr_mode = false;

	pp_config = msm_bedais_pp_params[index].pp_params_config;
	if (test_bit(ADM_PP_PARAM_MUTE_BIT, &pp_config)) {
		pr_debug("%s: ADM_PP_PARAM_MUTE\n", __func__);
		clear_bit(ADM_PP_PARAM_MUTE_BIT, &pp_config);
		mute_on = msm_bedais_pp_params[index].mute_on;
		if ((msm_bedais[be_idx].active) && compr_passthr_mode)
			adm_send_compressed_device_mute(port_id,
								copp_idx,
								mute_on);
	}
	if (test_bit(ADM_PP_PARAM_LATENCY_BIT, &pp_config)) {
		pr_debug("%s: ADM_PP_PARAM_LATENCY\n", __func__);
		clear_bit(ADM_PP_PARAM_LATENCY_BIT,
			  &pp_config);
		latency = msm_bedais_pp_params[index].latency;
		if ((msm_bedais[be_idx].active) && compr_passthr_mode)
			adm_send_compressed_device_latency(port_id,
							   copp_idx,
							   latency);
	}
	return 0;
}

static uint32_t msm_routing_get_topology(size_t data_size, void *data)
{
	uint32_t topology = NULL_COPP_TOPOLOGY;
	void *cal_info = NULL;
	uint32_t size = 0;

	/* Retrieve cal_info size from cal data*/
	if (data_size < sizeof(struct audio_cal_type_basic) +
			sizeof(struct audio_cal_info_adm_top)) {
		pr_err("%s: Invalid data size: %zd\n", __func__, data_size);
		goto done;
	}
	size = data_size - sizeof(struct audio_cal_type_basic);
	cal_info = kzalloc(size, GFP_KERNEL);

	if (!cal_info)
		goto done;

	memcpy(cal_info,
		((uint8_t *)data + sizeof(struct audio_cal_type_basic)), size);

	topology = ((struct audio_cal_info_adm_top *)cal_info)->topology;
	kfree(cal_info);
	cal_info = NULL;

done:
	pr_debug("%s: Using topology %d\n", __func__, topology);

	return topology;
}

static void msm_routing_load_topology(size_t data_size, void *data)
{
	uint32_t topology_id;
	int ret = -EINVAL;

	topology_id = msm_routing_get_topology(data_size, data);
	if (topology_id != NULL_COPP_TOPOLOGY)
		ret = q6core_load_unload_topo_modules(topology_id,
			CORE_LOAD_TOPOLOGY);
	if (ret < 0)
		pr_debug("%s %d load topology failed\n",
				 __func__, topology_id);

}

static void msm_routing_unload_topology(uint32_t topology_id)
{
	int ret = -EINVAL;

	if (topology_id != NULL_COPP_TOPOLOGY)
		ret = q6core_load_unload_topo_modules(topology_id,
			CORE_UNLOAD_TOPOLOGY);
	if (ret < 0)
		pr_debug("%s %d unload topology failed\n",
				 __func__, topology_id);

}

static int msm_routing_put_device_pp_params_mixer(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	int pp_id = ucontrol->value.integer.value[0];
	int port_id = 0;
	int index, be_idx, i, topo_id, idx;
	bool mute;
	int latency, session_type;
	bool compr_passthr_mode = true;

	pr_debug("%s: pp_id: 0x%x\n", __func__, pp_id);

	for (be_idx = 0; be_idx < MSM_BACKEND_DAI_MAX; be_idx++) {
		port_id = msm_bedais[be_idx].port_id;
		if (port_id == HDMI_RX || port_id == DISPLAY_PORT_RX
				|| port_id == HDMI_RX_MS)
			break;
	}

	if (be_idx >= MSM_BACKEND_DAI_MAX) {
		pr_debug("%s: Invalid be id %d\n", __func__, be_idx);
		return  -EINVAL;
	}

	for (index = 0; index < MSM_BACKEND_DAI_PP_PARAMS_REQ_MAX; index++) {
		if (msm_bedais_pp_params[index].port_id == port_id)
			break;
	}
	if (index >= MSM_BACKEND_DAI_PP_PARAMS_REQ_MAX) {
		pr_err("%s: Invalid pp params backend index %d\n",
			__func__, index);
		return -EINVAL;
	}

	session_type =
		(afe_get_port_type(port_id) == MSM_AFE_PORT_TYPE_RX) ?
		SESSION_TYPE_RX : SESSION_TYPE_TX;

	for_each_set_bit(i, &msm_bedais[be_idx].fe_sessions[0],
				MSM_FRONTEND_DAI_MM_SIZE) {
		if ((fe_dai_map[i][session_type].passthr_mode == LEGACY_PCM) ||
			(fe_dai_map[i][session_type].passthr_mode == LISTEN))
			compr_passthr_mode = false;

		for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
			unsigned long copp =
				session_copp_map[i]
				[SESSION_TYPE_RX][be_idx];
			if (!test_bit(idx, &copp))
				continue;
			topo_id = adm_get_topology_for_port_copp_idx(port_id,
								     idx);
			if (topo_id != COMPRESSED_PASSTHROUGH_DEFAULT_TOPOLOGY)
				continue;
		pr_debug("%s: port: 0x%x, copp %ld, be active: %d, passt: %d\n",
			 __func__, port_id, copp, msm_bedais[be_idx].active,
			 fe_dai_map[i][session_type].passthr_mode);
		switch (pp_id) {
		case ADM_PP_PARAM_MUTE_ID:
			pr_debug("%s: ADM_PP_PARAM_MUTE\n", __func__);
			mute = ucontrol->value.integer.value[1] ? true : false;
			msm_bedais_pp_params[index].mute_on = mute;
			set_bit(ADM_PP_PARAM_MUTE_BIT,
				&msm_bedais_pp_params[index].pp_params_config);
			if ((msm_bedais[be_idx].active) && compr_passthr_mode)
				adm_send_compressed_device_mute(port_id,
					idx, mute);
			break;
		case ADM_PP_PARAM_LATENCY_ID:
			pr_debug("%s: ADM_PP_PARAM_LATENCY\n", __func__);
			msm_bedais_pp_params[index].latency =
				ucontrol->value.integer.value[1];
			set_bit(ADM_PP_PARAM_LATENCY_BIT,
				&msm_bedais_pp_params[index].pp_params_config);
			latency = msm_bedais_pp_params[index].latency =
				ucontrol->value.integer.value[1];
			if ((msm_bedais[be_idx].active) && compr_passthr_mode)
				adm_send_compressed_device_latency(port_id,
					idx, latency);
			break;
		default:
			pr_info("%s, device pp param %d not supported\n",
				__func__, pp_id);
			break;
		}
		}
	}
	return 0;
}

static int msm_routing_get_device_pp_params_mixer(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s:msm_routing_get_device_pp_params_mixer", __func__);
	return 0;
}

static const struct snd_kcontrol_new device_pp_params_mixer_controls[] = {
	SOC_SINGLE_MULTI_EXT("Device PP Params", SND_SOC_NOPM, 0, 0xFFFFFFFF,
	0, 3, msm_routing_get_device_pp_params_mixer,
	msm_routing_put_device_pp_params_mixer),
};

static int msm_aptx_dec_license_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
			core_get_license_status(ASM_MEDIA_FMT_APTX);
	pr_debug("%s: status %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_aptx_dec_license_control_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int32_t status = 0;

	status = core_set_license(ucontrol->value.integer.value[0],
				APTX_CLASSIC_DEC_LICENSE_ID);
	pr_debug("%s: status %d\n", __func__, status);
	return status;
}

static const struct snd_kcontrol_new aptx_dec_license_controls[] = {
	SOC_SINGLE_EXT("APTX Dec License", SND_SOC_NOPM, 0,
	0xFFFF, 0, msm_aptx_dec_license_control_get,
	msm_aptx_dec_license_control_put),
};

static int msm_routing_get_pll_clk_drift(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int msm_routing_put_pll_clk_drift(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	u16 port_id = 0;
	int32_t clk_drift = 0;
	uint32_t clk_reset = 0;
	int be_idx, ret = -EINVAL;

	be_idx = ucontrol->value.integer.value[0];
	clk_drift = ucontrol->value.integer.value[1];
	clk_reset = ucontrol->value.integer.value[2];

	if (be_idx < 0 || be_idx >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Invalid be id %d\n", __func__, be_idx);
		return -EINVAL;
	}

	if (!msm_bedais[be_idx].active && !clk_reset) {
		pr_err("%s:BE is not active %d, cannot set clock drift\n",
			__func__, be_idx);
		return -EINVAL;
	}

	port_id = msm_bedais[be_idx].port_id;
	pr_debug("%s: clk drift %d be idx %d clk reset %d port id 0x%x\n",
		  __func__, clk_drift, be_idx, clk_reset, port_id);
	ret = afe_set_pll_clk_drift(port_id, clk_drift, clk_reset);
	if (ret < 0)
		pr_err("%s: failed to set pll clk drift\n", __func__);

	return ret;
}

static const struct snd_kcontrol_new pll_clk_drift_controls[] = {
	SOC_SINGLE_MULTI_EXT("PLL config data", SND_SOC_NOPM, 0, 0xFFFFFFFF,
	0, 128, msm_routing_get_pll_clk_drift, msm_routing_put_pll_clk_drift),
};

static int msm_routing_put_port_chmap_mixer(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	uint8_t channel_map[PCM_FORMAT_MAX_NUM_CHANNEL_V8];
	uint32_t be_idx = ucontrol->value.integer.value[0];
	int i;

	if (be_idx >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Invalid Backend index %d\n",  __func__, be_idx);
		return -EINVAL;
	}

	for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL_V8; i++) {
		channel_map[i] = (char)(ucontrol->value.integer.value[i + 1]);
		if (channel_map[i] > PCM_MAX_CHMAP_ID) {
			pr_err("%s: Invalid channel map %d\n",
				__func__, channel_map[i]);
			return -EINVAL;
		}
	}
	adm_set_port_multi_ch_map(channel_map, msm_bedais[be_idx].port_id);

	return 0;
}

static const struct snd_kcontrol_new port_multi_channel_map_mixer_controls[] = {
	SOC_SINGLE_MULTI_EXT("Backend Device Channel Map", SND_SOC_NOPM, 0,
			MSM_BACKEND_DAI_MAX, 0,
			PCM_FORMAT_MAX_NUM_CHANNEL_V8 + 1, NULL,
			msm_routing_put_port_chmap_mixer),
};

static int msm_routing_be_dai_name_table_info(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(be_dai_name_table);
	return 0;
}

static int msm_routing_be_dai_name_table_tlv_get(struct snd_kcontrol *kcontrol,
						 unsigned int __user *bytes,
						 unsigned int size)
{
	int i;
	int ret = 0;

	if (size < sizeof(be_dai_name_table)) {
		pr_err("%s: invalid size %d requested, returning\n",
			__func__, size);
		ret = -EINVAL;
		goto done;
	}

	/*
	 * Fill be_dai_name_table from msm_bedais table to reduce code changes
	 * needed when adding new backends
	 */
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		be_dai_name_table[i].be_id = i;
		strlcpy(be_dai_name_table[i].be_name,
			msm_bedais[i].name,
			LPASS_BE_NAME_MAX_LENGTH);
	}

	ret = copy_to_user(bytes, &be_dai_name_table,
			   sizeof(be_dai_name_table));
	if (ret) {
		pr_err("%s: failed to copy be_dai_name_table\n", __func__);
		ret = -EFAULT;
	}

done:
	return ret;
}

static const struct snd_kcontrol_new
	msm_routing_be_dai_name_table_mixer_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK,
		.info = msm_routing_be_dai_name_table_info,
		.name = "Backend DAI Name Table",
		.tlv.c = snd_soc_bytes_tlv_callback,
		.private_value = (unsigned long) &(struct soc_bytes_ext) {
			.max = sizeof(be_dai_name_table),
			.get = msm_routing_be_dai_name_table_tlv_get,
		}
	},
};

static int msm_routing_put_mclk_src_cfg(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	u16 port_id = 0;
	int32_t mclk_src_id = 0;
	uint32_t mclk_freq = 0;
	int be_idx, ret = -EINVAL;

	be_idx = ucontrol->value.integer.value[0];
	mclk_src_id = ucontrol->value.integer.value[1];
	mclk_freq = ucontrol->value.integer.value[2];

	if (be_idx < 0 || be_idx >= MSM_BACKEND_DAI_MAX) {
		pr_err("%s: Invalid be id %d\n", __func__, be_idx);
		return -EINVAL;
	}

	if (mclk_src_id < MCLK_SRC_INT || mclk_src_id >= MCLK_SRC_MAX) {
		pr_err("%s: Invalid MCLK src %d\n", __func__, mclk_src_id);
		return -EINVAL;
	}

	if (msm_bedais[be_idx].active) {
		pr_err("%s:BE is active %d, cannot set mclk clock src\n",
			__func__, be_idx);
		return -EINVAL;
	}

	port_id = msm_bedais[be_idx].port_id;
	pr_debug("%s: be idx %d mclk_src id %d mclk_freq %d port id 0x%x\n",
		  __func__, be_idx, mclk_src_id, mclk_freq, port_id);
	ret = afe_set_mclk_src_cfg(port_id, mclk_src_id, mclk_freq);
	if (ret < 0)
		pr_err("%s: failed to set mclk src cfg\n", __func__);

	return ret;

}

static const struct snd_kcontrol_new mclk_src_controls[] = {
	SOC_SINGLE_MULTI_EXT("MCLK_SRC CFG", SND_SOC_NOPM, 0, 24576000, 0, 3,
					NULL, msm_routing_put_mclk_src_cfg),
};

static int msm_routing_stereo_channel_reverse_control_get(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = swap_ch;
	pr_debug("%s: Swap channel value: %ld\n", __func__,
				ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_routing_stereo_channel_reverse_control_put(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int i, idx, be_index, port_id;
	int ret = 0;
	unsigned long copp;

	pr_debug("%s Swap channel value:%ld\n", __func__,
				ucontrol->value.integer.value[0]);

	swap_ch = ucontrol->value.integer.value[0];

	mutex_lock(&routing_lock);
	for (be_index = 0; be_index < MSM_BACKEND_DAI_MAX; be_index++) {
		port_id = msm_bedais[be_index].port_id;
		if (!msm_bedais[be_index].active)
			continue;

		for_each_set_bit(i, &msm_bedais[be_index].fe_sessions[0],
				MSM_FRONTEND_DAI_MM_SIZE) {
			copp = session_copp_map[i][SESSION_TYPE_RX][be_index];
			for (idx = 0; idx < MAX_COPPS_PER_PORT; idx++) {
				if (!test_bit(idx, &copp))
					continue;

				pr_debug("%s: swap channel control of portid:%d, coppid:%d\n",
					 __func__, port_id, idx);
				ret = adm_swap_speaker_channels(
					port_id, idx,
					msm_bedais[be_index].sample_rate,
					swap_ch);
				if (ret) {
					pr_err("%s:Swap_channel failed, err=%d\n",
						 __func__, ret);
					goto done;
				}
			}
		}
	}
done:
	mutex_unlock(&routing_lock);
	return ret;
}

static const struct snd_kcontrol_new stereo_channel_reverse_control[] = {
	SOC_SINGLE_EXT("Swap channel", SND_SOC_NOPM, 0,
	1, 0, msm_routing_stereo_channel_reverse_control_get,
	msm_routing_stereo_channel_reverse_control_put),
};

static int msm_routing_instance_id_support_info(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	return 0;
}

static int msm_routing_instance_id_support_put(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	bool supported = ucontrol->value.integer.value[0] ? true : false;

	q6common_update_instance_id_support(supported);
	return 0;
}

static int msm_routing_instance_id_support_get(
	struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	bool supported = false;

	supported = q6common_is_instance_id_supported();
	ucontrol->value.integer.value[0] = supported ? 1 : 0;

	return 0;
}

static const struct snd_kcontrol_new
	msm_routing_feature_support_mixer_controls[] = {
		{
			.access = SNDRV_CTL_ELEM_ACCESS_READ |
				SNDRV_CTL_ELEM_ACCESS_WRITE,
			.info = msm_routing_instance_id_support_info,
			.name = "Instance ID Support",
			.put = msm_routing_instance_id_support_put,
			.get = msm_routing_instance_id_support_get,
		},
};

static const struct snd_pcm_ops msm_routing_pcm_ops = {
	.hw_params	= msm_pcm_routing_hw_params,
	.close          = msm_pcm_routing_close,
	.prepare        = msm_pcm_routing_prepare,
};

#ifndef CONFIG_TDM_DISABLE
static void snd_soc_dapm_new_controls_tdm(struct snd_soc_component *component)
{
	 snd_soc_dapm_new_controls(&component->dapm,
				    msm_qdsp6_widgets_tdm,
				    ARRAY_SIZE(msm_qdsp6_widgets_tdm));
}
static void snd_soc_dapm_add_routes_tdm(struct snd_soc_component *component)
{
	 snd_soc_dapm_add_routes(&component->dapm, intercon_tdm,
		  ARRAY_SIZE(intercon_tdm));
}
#else
static void snd_soc_dapm_new_controls_tdm(struct snd_soc_component *component)
{
	 return;
}

static void snd_soc_dapm_add_routes_tdm(struct snd_soc_component *component)
{
	 return;
}
#endif


#ifndef CONFIG_MI2S_DISABLE
static void snd_soc_dapm_new_controls_mi2s(struct snd_soc_component *component)
{
	 snd_soc_dapm_new_controls(&component->dapm,
				    msm_qdsp6_widgets_mi2s,
				    ARRAY_SIZE(msm_qdsp6_widgets_mi2s));
}
static void snd_soc_dapm_add_routes_mi2s(struct snd_soc_component *component)
{
	 snd_soc_dapm_add_routes(&component->dapm, intercon_mi2s,
		  ARRAY_SIZE(intercon_mi2s));
}
#else
static void snd_soc_dapm_new_controls_mi2s(struct snd_soc_component *component)
{
	 return;
}
static void snd_soc_dapm_add_routes_mi2s(struct snd_soc_component *component)
{
	 return;
}
#endif

#ifndef CONFIG_AUXPCM_DISABLE
static void snd_soc_dapm_new_controls_aux_pcm(struct snd_soc_component *component)
{
	 snd_soc_dapm_new_controls(&component->dapm,
				    msm_qdsp6_widgets_aux_pcm,
				    ARRAY_SIZE(msm_qdsp6_widgets_aux_pcm));
}
static void snd_soc_dapm_add_routes_aux_pcm(struct snd_soc_component *component)
{
	 snd_soc_dapm_add_routes(&component->dapm, intercon_aux_pcm,
		  ARRAY_SIZE(intercon_aux_pcm));
}
#else
static void snd_soc_dapm_new_controls_aux_pcm(struct snd_soc_component *component)
{
	 return;
}
static void snd_soc_dapm_add_routes_aux_pcm(struct snd_soc_component *component)
{
	 return;
}
#endif


/* Not used but frame seems to require it */
static int msm_routing_probe(struct snd_soc_component *component)
{
	snd_soc_dapm_new_controls(&component->dapm, msm_qdsp6_widgets,
			   ARRAY_SIZE(msm_qdsp6_widgets));

	snd_soc_dapm_new_controls_tdm(component);
	snd_soc_dapm_new_controls_mi2s(component);
	snd_soc_dapm_new_controls_aux_pcm(component);

	snd_soc_dapm_ignore_suspend(&component->dapm, "BE_OUT");
	snd_soc_dapm_ignore_suspend(&component->dapm, "BE_IN");
	snd_soc_dapm_add_routes(&component->dapm, intercon,
		ARRAY_SIZE(intercon));

	snd_soc_dapm_add_routes_tdm(component);
	snd_soc_dapm_add_routes_mi2s(component);
	snd_soc_dapm_add_routes_aux_pcm(component);
	snd_soc_dapm_new_widgets(component->dapm.card);

	snd_soc_add_component_controls(component, msm_voc_session_controls,
				      ARRAY_SIZE(msm_voc_session_controls));

	snd_soc_add_component_controls(component, app_type_cfg_controls,
				      ARRAY_SIZE(app_type_cfg_controls));

	snd_soc_add_component_controls(component, module_cfg_controls,
				      ARRAY_SIZE(module_cfg_controls));

	snd_soc_add_component_controls(component,
				stereo_to_custom_stereo_controls,
			ARRAY_SIZE(stereo_to_custom_stereo_controls));

	snd_soc_add_component_controls(component, ec_ref_param_controls,
				ARRAY_SIZE(ec_ref_param_controls));

	snd_soc_add_component_controls(component, channel_mixer_controls,
				ARRAY_SIZE(channel_mixer_controls));

	msm_qti_pp_add_controls(component);

	snd_soc_add_component_controls(component,
			hifi_filter_controls,
			ARRAY_SIZE(hifi_filter_controls));

	snd_soc_add_component_controls(component,
			use_ffecns_freeze_event_controls,
			ARRAY_SIZE(use_ffecns_freeze_event_controls));

	snd_soc_add_component_controls(component,
				device_pp_params_mixer_controls,
				ARRAY_SIZE(device_pp_params_mixer_controls));

	snd_soc_add_component_controls(component,
		msm_routing_be_dai_name_table_mixer_controls,
		ARRAY_SIZE(msm_routing_be_dai_name_table_mixer_controls));

	snd_soc_add_component_controls(component,
				stereo_channel_reverse_control,
				ARRAY_SIZE(stereo_channel_reverse_control));
	snd_soc_add_component_controls(
			component, msm_routing_feature_support_mixer_controls,
			ARRAY_SIZE(msm_routing_feature_support_mixer_controls));
	snd_soc_add_component_controls(component,
			port_multi_channel_map_mixer_controls,
			ARRAY_SIZE(port_multi_channel_map_mixer_controls));

	snd_soc_add_component_controls(component, pll_clk_drift_controls,
				      ARRAY_SIZE(pll_clk_drift_controls));

	snd_soc_add_component_controls(component, mclk_src_controls,
				      ARRAY_SIZE(mclk_src_controls));
	return 0;
}

int msm_routing_pcm_new(struct snd_soc_pcm_runtime *runtime)
{
	return msm_pcm_routing_hwdep_new(runtime, msm_bedais);
}

void msm_routing_pcm_free(struct snd_pcm *pcm)
{
	msm_pcm_routing_hwdep_free(pcm);
}

static struct snd_soc_component_driver msm_soc_routing_component = {
	.name		= DRV_NAME,
	.ops		= &msm_routing_pcm_ops,
	.probe		= msm_routing_probe,
	.pcm_new	= msm_routing_pcm_new,
	.pcm_free	= msm_routing_pcm_free,
};

static int msm_routing_pcm_probe(struct platform_device *pdev)
{

	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));
	return snd_soc_register_component(&pdev->dev,
				&msm_soc_routing_component,
				NULL, 0);
}

static int msm_routing_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_pcm_routing_dt_match[] = {
	{.compatible = "qcom,msm-pcm-routing"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_pcm_routing_dt_match);

static struct platform_driver msm_routing_pcm_driver = {
	.driver = {
		.name = "msm-pcm-routing",
		.owner = THIS_MODULE,
		.of_match_table = msm_pcm_routing_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_routing_pcm_probe,
	.remove = msm_routing_pcm_remove,
};

int msm_routing_check_backend_enabled(int fedai_id)
{
	int i;

	if (fedai_id >= MSM_FRONTEND_DAI_MM_MAX_ID) {
		/* bad ID assigned in machine driver */
		pr_err("%s: bad MM ID\n", __func__);
		return 0;
	}
	for (i = 0; i < MSM_BACKEND_DAI_MAX; i++) {
		if (test_bit(fedai_id, &msm_bedais[i].fe_sessions[0]))
			return msm_bedais[i].active;
	}
	return 0;
}

static int get_cal_type_index(int32_t cal_type)
{
	int ret = -EINVAL;

	switch (cal_type) {
	case ADM_TOPOLOGY_CAL_TYPE:
		ret = ADM_TOPOLOGY_CAL_TYPE_IDX;
		break;
	default:
		pr_err("%s: Invalid cal type %d\n", __func__, cal_type);
	}
	return ret;
}

static int msm_routing_set_cal(int32_t cal_type,
					size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;
	pr_debug("%s\n", __func__);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: Could not get cal index %d\n",
			__func__, cal_index);
		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_set_cal(data_size, data, cal_data[cal_index], 0, NULL);
	if (ret < 0) {
		pr_err("%s: cal_utils_set_cal failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);
		ret = -EINVAL;
		goto done;
	}
	/* Pre-load if it is ADM topology */
	if (cal_index == ADM_TOPOLOGY_CAL_TYPE_IDX)
		msm_routing_load_topology(data_size, data);
done:
	return ret;
}

static void msm_routing_delete_cal_data(void)
{
	pr_debug("%s\n", __func__);

	cal_utils_destroy_cal_types(MAX_ROUTING_CAL_TYPES, &cal_data[0]);
}

static int msm_routing_init_cal_data(void)
{
	int ret = 0;
	struct cal_type_info cal_type_info[] = {
		{{ADM_TOPOLOGY_CAL_TYPE,
		{NULL, NULL, NULL,
		msm_routing_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{ADM_LSM_TOPOLOGY_CAL_TYPE,
		{NULL, NULL, NULL,
		msm_routing_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

	};
	pr_debug("%s\n", __func__);

	ret = cal_utils_create_cal_types(MAX_ROUTING_CAL_TYPES, &cal_data[0],
		&cal_type_info[0]);
	if (ret < 0) {
		pr_err("%s: could not create cal type!\n",
			__func__);
		ret = -EINVAL;
		goto err;
	}

	return ret;
err:
	msm_routing_delete_cal_data();
	return ret;
}

int __init msm_soc_routing_platform_init(void)
{
	mutex_init(&routing_lock);
	if (msm_routing_init_cal_data())
		pr_err("%s: could not init cal data!\n", __func__);

	afe_set_routing_callback(
		(routing_cb)msm_pcm_get_dev_acdb_id_by_port_id);

	memset(&be_dai_name_table, 0, sizeof(be_dai_name_table));
	memset(&last_be_id_configured, 0, sizeof(last_be_id_configured));

	return platform_driver_register(&msm_routing_pcm_driver);
}

void msm_soc_routing_platform_exit(void)
{
	msm_routing_delete_cal_data();
	memset(&be_dai_name_table, 0, sizeof(be_dai_name_table));
	mutex_destroy(&routing_lock);
	platform_driver_unregister(&msm_routing_pcm_driver);
}

MODULE_DESCRIPTION("MSM routing platform driver");
MODULE_LICENSE("GPL v2");
