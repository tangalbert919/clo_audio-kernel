// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "msm-compress-q6-v2.h"

static struct snd_soc_dai_ops msm_fe_dai_ops = {};

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	88200, 96000, 176400, 192000, 352800, 384000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static int multimedia_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE,
		&constraints_sample_rates);
	return 0;
}

static int fe_dai_probe(struct snd_soc_dai *dai)
{
	struct snd_soc_dapm_route intercon;
	struct snd_soc_dapm_context *dapm;

	if (!dai || !dai->driver) {
		pr_err("%s invalid params\n", __func__);
		return -EINVAL;
	}
	dapm = snd_soc_component_get_dapm(dai->component);
	memset(&intercon, 0, sizeof(intercon));
	if (dai->driver->playback.stream_name &&
		dai->driver->playback.aif_name) {
		dev_dbg(dai->dev, "%s add route for widget %s",
			   __func__, dai->driver->playback.stream_name);
		intercon.source = dai->driver->playback.stream_name;
		intercon.sink = dai->driver->playback.aif_name;
		dev_dbg(dai->dev, "%s src %s sink %s\n",
			   __func__, intercon.source, intercon.sink);
		snd_soc_dapm_add_routes(dapm, &intercon, 1);
		snd_soc_dapm_ignore_suspend(dapm, intercon.source);
	}
	if (dai->driver->capture.stream_name &&
	   dai->driver->capture.aif_name) {
		dev_dbg(dai->dev, "%s add route for widget %s",
			   __func__, dai->driver->capture.stream_name);
		intercon.sink = dai->driver->capture.stream_name;
		intercon.source = dai->driver->capture.aif_name;
		dev_dbg(dai->dev, "%s src %s sink %s\n",
			   __func__, intercon.source, intercon.sink);
		snd_soc_dapm_add_routes(dapm, &intercon, 1);
		snd_soc_dapm_ignore_suspend(dapm, intercon.sink);
	}
	return 0;
}

static struct snd_soc_dai_ops msm_fe_Multimedia_dai_ops = {
	.startup	= multimedia_startup,
};

static const struct snd_soc_component_driver msm_fe_dai_component = {
	.name		= "msm-dai-fe",
};

static struct snd_soc_dai_driver msm_fe_dais[] = {
	{
		.playback = {
			.stream_name = "MultiMedia1 Playback",
			.aif_name = "MM_DL1",
			.rates = (SNDRV_PCM_RATE_8000_384000|
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
						SNDRV_PCM_FMTBIT_S24_LE |
						SNDRV_PCM_FMTBIT_S24_3LE |
						SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =     8000,
			.rate_max =	384000,
		},
		.capture = {
			.stream_name = "MultiMedia1 Capture",
			.aif_name = "MM_UL1",
			.rates = (SNDRV_PCM_RATE_8000_384000|
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S24_3LE |
				    SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =     8000,
			.rate_max =	48000,
		},
		.ops = &msm_fe_Multimedia_dai_ops,
		.name = "MultiMedia1",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "MultiMedia2 Playback",
			.aif_name = "MM_DL2",
			.rates = (SNDRV_PCM_RATE_8000_384000|
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
						SNDRV_PCM_FMTBIT_S24_LE |
						SNDRV_PCM_FMTBIT_S24_3LE |
						SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =     8000,
			.rate_max =	384000,
		},
		.capture = {
			.stream_name = "MultiMedia2 Capture",
			.aif_name = "MM_UL2",
			.rates = (SNDRV_PCM_RATE_8000_384000|
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S24_3LE |
				    SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =     8000,
			.rate_max =	48000,
		},
		.ops = &msm_fe_Multimedia_dai_ops,
		.name = "MultiMedia2",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VoIP Playback",
			.aif_name = "VOIP_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_SPECIAL,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =	8000,
			.rate_max = 48000,
		},
		.capture = {
			.stream_name = "VoIP Capture",
			.aif_name = "VOIP_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_SPECIAL,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =	8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VoIP",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "MultiMedia3 Playback",
			.aif_name = "MM_DL3",
			.rates = (SNDRV_PCM_RATE_8000_384000 |
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
						SNDRV_PCM_FMTBIT_S24_LE |
						SNDRV_PCM_FMTBIT_S24_3LE |
						SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =	8000,
			.rate_max = 384000,
		},
		.capture = {
			.stream_name = "MultiMedia3 Capture",
			.aif_name = "MM_UL3",
			.rates = (SNDRV_PCM_RATE_8000_384000|
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
						SNDRV_PCM_FMTBIT_S24_LE |
						SNDRV_PCM_FMTBIT_S24_3LE |
						SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =     8000,
			.rate_max =	48000,
		},
		.ops = &msm_fe_Multimedia_dai_ops,
		.name = "MultiMedia3",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "MultiMedia4 Playback",
			.aif_name = "MM_DL4",
			.rates = (SNDRV_PCM_RATE_8000_384000 |
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
						SNDRV_PCM_FMTBIT_S24_LE |
						SNDRV_PCM_FMTBIT_S24_3LE |
						SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =	8000,
			.rate_max = 384000,
		},
		.capture = {
			.stream_name = "MultiMedia4 Capture",
			.aif_name = "MM_UL4",
			.rates = (SNDRV_PCM_RATE_8000_48000|
					SNDRV_PCM_RATE_KNOT),
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S24_3LE |
				    SNDRV_PCM_FMTBIT_S32_LE),
			.channels_min = 1,
			.channels_max = 32,
			.rate_min =     8000,
			.rate_max =	48000,
		},
		.ops = &msm_fe_Multimedia_dai_ops,
		.name = "MultiMedia4",
		.probe = fe_dai_probe,
	},
	/* FE DAIs created for hostless operation purpose */
	{
		.playback = {
			.stream_name = "USBAUDIO_HOSTLESS Playback",
			.aif_name = "USBAUDIO_DL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min =     8000,
			.rate_max =     384000,
		},
		.capture = {
			.stream_name = "USBAUDIO_HOSTLESS Capture",
			.aif_name = "USBAUDIO_UL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min =     8000,
			.rate_max =     384000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "USBAUDIO_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "AUXPCM_HOSTLESS Playback",
			.aif_name = "AUXPCM_DL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min =     8000,
			.rate_max =     16000,
		},
		.capture = {
			.stream_name = "AUXPCM_HOSTLESS Capture",
			.aif_name = "AUXPCM_UL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min =     8000,
			.rate_max =    16000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "AUXPCM_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "SEC_AUXPCM_HOSTLESS Playback",
			.aif_name = "SEC_AUXPCM_DL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min =     8000,
			.rate_max =     16000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_AUXPCM_RX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "SEC_AUXPCM_HOSTLESS Capture",
			.aif_name = "SEC_AUXPCM_UL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min =     8000,
			.rate_max =    16000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_AUXPCM_TX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VOICE_STUB Playback",
			.aif_name = "VOICE_STUB_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.capture = {
			.stream_name = "VOICE_STUB Capture",
			.aif_name = "VOICE_STUB_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VOICE_STUB",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "SEC_I2S_RX_HOSTLESS Playback",
			.aif_name = "SEC_I2S_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =    48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_I2S_RX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary MI2S_TX Hostless Capture",
			.aif_name = "PRI_MI2S_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_MI2S_TX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary MI2S_RX Hostless Playback",
			.aif_name = "PRI_MI2S_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =    384000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_MI2S_RX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary MI2S_TX Hostless Capture",
			.aif_name = "SEC_MI2S_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_MI2S_TX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary MI2S_RX Hostless Playback",
			.aif_name = "SEC_MI2S_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_MI2S_RX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "CDC_DMA_HOSTLESS Playback",
			.aif_name = "CDC_DMA_DL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
		.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.capture = {
			.stream_name = "CDC_DMA_HOSTLESS Capture",
			.aif_name = "CDC_DMA_UL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "CDC_DMA_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "TX3_CDC_DMA_HOSTLESS Capture",
			.aif_name = "TX3_CDC_DMA_UL_HL",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "TX3_CDC_DMA_HOSTLESS",
		.probe = fe_dai_probe,
	},
	/* TDM Hostless */
	{
		.capture = {
			.stream_name = "Primary TDM0 Hostless Capture",
			.aif_name = "PRI_TDM_TX_0_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_0_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM0 Hostless Playback",
			.aif_name = "PRI_TDM_RX_0_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_0_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM1 Hostless Capture",
			.aif_name = "PRI_TDM_TX_1_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_1_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM1 Hostless Playback",
			.aif_name = "PRI_TDM_RX_1_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_1_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM2 Hostless Capture",
			.aif_name = "PRI_TDM_TX_2_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_2_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM2 Hostless Playback",
			.aif_name = "PRI_TDM_RX_2_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_2_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM3 Hostless Capture",
			.aif_name = "PRI_TDM_TX_3_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_3_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM3 Hostless Playback",
			.aif_name = "PRI_TDM_RX_3_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_3_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM4 Hostless Capture",
			.aif_name = "PRI_TDM_TX_4_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_4_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM4 Hostless Playback",
			.aif_name = "PRI_TDM_RX_4_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_4_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM5 Hostless Capture",
			.aif_name = "PRI_TDM_TX_5_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_5_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM5 Hostless Playback",
			.aif_name = "PRI_TDM_RX_5_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_5_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM6 Hostless Capture",
			.aif_name = "PRI_TDM_TX_6_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_6_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM6 Hostless Playback",
			.aif_name = "PRI_TDM_RX_6_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_6_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Primary TDM7 Hostless Capture",
			.aif_name = "PRI_TDM_TX_7_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_TX_7_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Primary TDM7 Hostless Playback",
			.aif_name = "PRI_TDM_RX_7_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "PRI_TDM_RX_7_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM0 Hostless Capture",
			.aif_name = "SEC_TDM_TX_0_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_0_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM0 Hostless Playback",
			.aif_name = "SEC_TDM_RX_0_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_0_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM1 Hostless Capture",
			.aif_name = "SEC_TDM_TX_1_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_1_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM1 Hostless Playback",
			.aif_name = "SEC_TDM_RX_1_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_1_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM2 Hostless Capture",
			.aif_name = "SEC_TDM_TX_2_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_2_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM2 Hostless Playback",
			.aif_name = "SEC_TDM_RX_2_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_2_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM3 Hostless Capture",
			.aif_name = "SEC_TDM_TX_3_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_3_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM3 Hostless Playback",
			.aif_name = "SEC_TDM_RX_3_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_3_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM4 Hostless Capture",
			.aif_name = "SEC_TDM_TX_4_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_4_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM4 Hostless Playback",
			.aif_name = "SEC_TDM_RX_4_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_4_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM5 Hostless Capture",
			.aif_name = "SEC_TDM_TX_5_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_5_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM5 Hostless Playback",
			.aif_name = "SEC_TDM_RX_5_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_5_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM6 Hostless Capture",
			.aif_name = "SEC_TDM_TX_6_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_6_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM6 Hostless Playback",
			.aif_name = "SEC_TDM_RX_6_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_6_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM7 Hostless Capture",
			.aif_name = "SEC_TDM_TX_7_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_TX_7_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM7 Hostless Playback",
			.aif_name = "SEC_TDM_RX_7_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "SEC_TDM_RX_7_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "DTMF_RX_HOSTLESS Playback",
			.aif_name = "DTMF_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =	8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "DTMF_RX_HOSTLESS",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VoiceMMode1_HOST_RX_PLAYBACK Playback",
			.aif_name = "HPCM_RX_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,

		},
		.ops = &msm_fe_dai_ops,
		.name = "VoiceMMode1_HOST_RX_PLAYBACK",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "VoiceMMode1_HOST_RX_CAPTURE Capture",
			.aif_name = "HPCM_RX_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VoiceMMode1_HOST_RX_CAPTURE",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VoiceMMode1_HOST_TX_PLAYBACK Playback",
			.aif_name = "HPCM_TX_DL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,

		},
		.ops = &msm_fe_dai_ops,
		.name = "VoiceMMode1_HOST_TX_PLAYBACK",
		.probe = fe_dai_probe,
	},
	{
		.capture = {
			.stream_name = "VoiceMMode1_HOST_TX_CAPTURE Capture",
			.aif_name = "HPCM_TX_UL_HL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VoiceMMode1_HOST_TX_CAPTURE",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VOLTE_STUB Playback",
			.aif_name = "VOLTE_STUB_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.capture = {
			.stream_name = "VOLTE_STUB Capture",
			.aif_name = "VOLTE_STUB_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VOLTE_STUB",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VOICE2_STUB Playback",
			.aif_name = "VOICE2_STUB_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.capture = {
			.stream_name = "VOICE2_STUB Capture",
			.aif_name = "VOICE2_STUB_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VOICE2_STUB",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "QCHAT Playback",
			.aif_name = "QCHAT_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.capture = {
			.stream_name = "QCHAT Capture",
			.aif_name = "QCHAT_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "QCHAT",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VoiceMMode1 Playback",
			.aif_name = "VOICEMMODE1_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
	 },
		.capture = {
			.stream_name = "VoiceMMode1 Capture",
			.aif_name = "VOICEMMODE1_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VoiceMMode1",
		.probe = fe_dai_probe,
	},
	{
		.playback = {
			.stream_name = "VoiceMMode2 Playback",
			.aif_name = "VOICEMMODE2_DL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.capture = {
			.stream_name = "VoiceMMode2 Capture",
			.aif_name = "VOICEMMODE2_UL",
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_fe_dai_ops,
		.name = "VoiceMMode2",
		.probe = fe_dai_probe,
	},
};

static int msm_fe_dai_dev_probe(struct platform_device *pdev)
{

	dev_dbg(&pdev->dev, "%s: dev name %s\n", __func__,
		dev_name(&pdev->dev));
	return snd_soc_register_component(&pdev->dev, &msm_fe_dai_component,
		msm_fe_dais, ARRAY_SIZE(msm_fe_dais));
}

static int msm_fe_dai_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_fe_dt_match[] = {
	{.compatible = "qcom,msm-dai-fe"},
	{}
};

static struct platform_driver msm_fe_dai_driver = {
	.probe  = msm_fe_dai_dev_probe,
	.remove = msm_fe_dai_dev_remove,
	.driver = {
		.name = "msm-dai-fe",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_fe_dt_match,
		.suppress_bind_attrs = true,
	},
};

int __init msm_fe_dai_init(void)
{
	return platform_driver_register(&msm_fe_dai_driver);
}

void msm_fe_dai_exit(void)
{
	platform_driver_unregister(&msm_fe_dai_driver);
}

/* Module information */
MODULE_DESCRIPTION("MSM Frontend DAI driver");
MODULE_LICENSE("GPL v2");
