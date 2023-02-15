// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <soc/snd_event.h>
#include <dsp/audio_notifier.h>
#include <dsp/q6afe-v2.h>
#include <dsp/q6core.h>
#include "msm-pcm-routing-v2.h"
#include <asoc/msm-cdc-pinctrl.h>
#include "codecs/wcd934x/wcd934x.h"
#include "codecs/wcd934x/wcd934x-mbhc.h"
#include "codecs/wsa881x.h"
#include "soc/qcom/boot_stats.h"
#include <dt-bindings/sound/audio-codec-port-types.h>
#include "device_event.h"
#include "sa415m_dailink.h"

/* Machine driver Name */
#define DRV_NAME "sdx-asoc-snd"

#define __CHIPSET__ "SDX "
#define SDX_DAILINK_NAME(name) (__CHIPSET__#name)

#define SAMPLE_RATE_8KHZ 8000
#define SAMPLE_RATE_16KHZ 16000
#define SAMPLING_RATE_32KHZ 32000
#define SAMPLE_RATE_48KHZ 48000
#define SAMPLING_RATE_176P4KHZ 176400
#define SAMPLING_RATE_352P8KHZ  352800
#define NUM_OF_BITS_PER_SAMPLE 16
#define DEV_NAME_STR_LEN 32

#define LPAIF_OFFSET 0x01a00000
#define LPAIF_PRI_MODE_MUXSEL (LPAIF_OFFSET + 0x2008)
#define LPAIF_SEC_MODE_MUXSEL (LPAIF_OFFSET + 0x200c)
#define LPASS_CSR_GP_IO_MUX_SPKR_CTL (LPAIF_OFFSET + 0x2004)
#define LPASS_CSR_GP_IO_MUX_MIC_CTL  (LPAIF_OFFSET + 0x2000)

#define I2S_SEL 0
#define PCM_SEL 1
#define I2S_PCM_SEL_OFFSET 0
#define I2S_PCM_MASTER_MODE 1
#define I2S_PCM_SLAVE_MODE 0

#define PRI_TLMM_CLKS_EN_MASTER 0x4
#define SEC_TLMM_CLKS_EN_MASTER 0x2
#define PRI_TLMM_CLKS_EN_SLAVE 0x100000
#define SEC_TLMM_CLKS_EN_SLAVE 0x800000
#define CLOCK_ON  1
#define CLOCK_OFF 0

#define WCD9XXX_MBHC_DEF_BUTTONS 8
#define WCD9XXX_MBHC_DEF_RLOADS 5

#define PERIOD_SIZE_320 320

/* Spk control */
#define SDX_SPK_ON 1
#define SDX_HIFI_ON 1

#define SDX_MCLK_CLK_12P288MHZ 12288000
#define TLV_CLKIN_MCLK 0

enum mi2s_types {
	PRI_MI2S,
	SEC_MI2S,
};

enum {
	TDM_0 = 0,
	TDM_1,
	TDM_2,
	TDM_3,
	TDM_4,
	TDM_5,
	TDM_6,
	TDM_7,
	TDM_PORT_MAX,
};

enum tdm_types {
	TDM_PRI = 0,
	TDM_SEC,
	TDM_INTERFACE_MAX,
};

struct tdm_port {
	u32 mode;
	u32 channel;
};

struct dev_config {
	u32 sample_rate;
	u32 bit_format;
	u32 channels;
};

struct proxy_dev_config {
	u32 sample_rate;
	u32 bit_format;
	u32 channels;
	u32 period_size;
};

struct sdx_machine_data {
	u32 mclk_freq;
	u16 prim_mi2s_mode;
	u16 sec_mi2s_mode;
	u16 prim_tdm_mode;
	u16 sec_tdm_mode;
	u16 prim_auxpcm_mode;
	u32 wsa_max_devs;
	struct device_node *prim_master_p;
	struct device_node *prim_slave_p;
	u16 sec_auxpcm_mode;
	struct device_node *sec_master_p;
	struct device_node *sec_slave_p;
	u32 prim_clk_usrs;
	int hph_en1_gpio;
	int hph_en0_gpio;
	struct snd_info_entry *codec_root;
	void __iomem *lpaif_pri_muxsel_virt_addr;
	void __iomem *lpaif_sec_muxsel_virt_addr;
	void __iomem *lpass_mux_spkr_ctl_virt_addr;
	void __iomem *lpass_mux_mic_ctl_virt_addr;
};

/* TDM default config */
static struct dev_config tdm_rx_cfg[TDM_INTERFACE_MAX][TDM_PORT_MAX] = {
	{ /* PRI TDM */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_0 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_1 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_2 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_3 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_4 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_5 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_6 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_7 */
	},
	{ /* SEC TDM */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_0 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_1 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_2 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_3 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_4 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_5 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_6 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* RX_7 */
	},
};

static struct dev_config tdm_tx_cfg[TDM_INTERFACE_MAX][TDM_PORT_MAX] = {
	{ /* PRI TDM */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_0 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_1 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_2 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_3 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_4 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_5 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_6 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_7 */
	},
	{ /* SEC TDM */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_0 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_1 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_2 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_3 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_4 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_5 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_6 */
		{SAMPLE_RATE_48KHZ, SNDRV_PCM_FORMAT_S16_LE, 1}, /* TX_7 */
	},
};

struct sdx_wsa881x_dev_info {
	struct device_node *of_node;
	u32 index;
};

static void *def_tavil_mbhc_cal(void);

static struct wcd_mbhc_config wcd_mbhc_cfg = {
	.read_fw_bin = false,
	.calibration = NULL,
	.detect_extn_cable = true,
	.mono_stero_detection = false,
	.swap_gnd_mic = NULL,
	.hs_ext_micbias = true,
};

static const struct afe_clk_set lpass_default_v2 = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
	Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	0,
};

static int sdx_auxpcm_rate = 8000;

static struct mutex cdc_mclk_mutex;
static struct mutex snd_card_mutex;
static int sdx_mi2s_rx_ch = 1;
static int sdx_mi2s_tx_ch = 1;
static int sdx_sec_mi2s_rx_ch = 1;
static int sdx_sec_mi2s_tx_ch = 1;
static int sdx_mi2s_bitwidth = SNDRV_PCM_FORMAT_S16_LE;
static int sdx_mi2s_rate = SAMPLE_RATE_48KHZ;
static int sdx_sec_mi2s_rate = SAMPLE_RATE_48KHZ;

static int sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
static int sdx_sec_mi2s_mode = I2S_PCM_MASTER_MODE;
static int sdx_auxpcm_mode = I2S_PCM_MASTER_MODE;
static int sdx_sec_auxpcm_mode = I2S_PCM_MASTER_MODE;
static int sdx_prim_tdm_mode = I2S_PCM_MASTER_MODE;
static int sdx_sec_tdm_mode = I2S_PCM_MASTER_MODE;

static int sdx_spk_control = 1;
static atomic_t mi2s_ref_count;
static atomic_t sec_mi2s_ref_count;

static struct proxy_dev_config proxy_tx_cfg = {
	.sample_rate = SAMPLE_RATE_48KHZ,
	.bit_format = SNDRV_PCM_FORMAT_S16_LE,
	.channels = 2,
	.period_size = PERIOD_SIZE_320,
};

static struct snd_soc_card snd_soc_card_tavil_sdx = {
	.name = "sdx-tavil-i2s-snd-card",
};

static struct snd_soc_card snd_soc_card_auto_sdx = {
	.name = "sdx-auto-i2s-snd-card",
};

static struct snd_info_entry *msm_snd_info_create_subdir(struct module *mod,
				const char *name,
				struct snd_info_entry *parent)
{
	struct snd_info_entry *entry;

	entry = snd_info_create_module_entry(mod, name, parent);
	if (!entry)
		return NULL;
	entry->mode = S_IFDIR | 0555;
	if (snd_info_register(entry) < 0) {
		snd_info_free_entry(entry);
		return NULL;
	}
	return entry;
}

static int sdx_mi2s_clk_ctl(struct snd_soc_pcm_runtime *rtd, bool enable,
			    enum mi2s_types mi2s_type, int rate, u16 mode)
{
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);
	struct afe_clk_set m_clk = lpass_default_v2;
	struct afe_clk_set ibit_clk = lpass_default_v2;
	u16 mi2s_port;
	u16 ibit_clk_id;
	int bit_clk_freq = (rate * 2 * NUM_OF_BITS_PER_SAMPLE);
	int ret = 0;

	dev_dbg(card->dev, "%s: setting lpass clock using v2\n", __func__);

	if (pdata == NULL) {
		dev_err(card->dev, "%s: platform data is null\n", __func__);
		ret = -ENOMEM;
		goto done;
	}

	if (mi2s_type == PRI_MI2S) {
		mi2s_port = AFE_PORT_ID_PRIMARY_MI2S_RX;
		ibit_clk_id = Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT;
	} else {
		mi2s_port = AFE_PORT_ID_SECONDARY_MI2S_RX;
		ibit_clk_id = Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT;
	}

	/* Set both mclk and ibit clocks when using LPASS_CLK_VER_2 */
	m_clk.clk_id = Q6AFE_LPASS_CLK_ID_MCLK_3;
	m_clk.clk_freq_in_hz = pdata->mclk_freq;
	m_clk.enable = enable;
	ret = afe_set_lpass_clock_v2(mi2s_port, &m_clk);
	if (ret < 0) {
		dev_err(card->dev,
			"%s: afe_set_lpass_clock_v2 failed for mclk_3 with ret %d\n",
			__func__, ret);
		goto done;
	}

	if (mode) {
		ibit_clk.clk_id = ibit_clk_id;
		ibit_clk.clk_freq_in_hz = bit_clk_freq;
		ibit_clk.enable = enable;
		ret = afe_set_lpass_clock_v2(mi2s_port, &ibit_clk);
		if (ret < 0) {
			dev_err(card->dev,
				"%s: afe_set_lpass_clock_v2 failed for ibit with ret %d\n",
				__func__, ret);
			goto err_ibit_clk_set;
		}
	}
	ret = 0;

done:
	return ret;

err_ibit_clk_set:
	m_clk.enable = false;
	if (afe_set_lpass_clock_v2(mi2s_port, &m_clk))
		dev_err(card->dev, "%s: afe_set_lpass_clock_v2 failed for mclk_3\n",
			__func__);

	return ret;
}

static void sdx_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&mi2s_ref_count) == 0) {
		ret = sdx_mi2s_clk_ctl(rtd, false, PRI_MI2S, 0,
				       pdata->prim_mi2s_mode);
		if (ret < 0)
			pr_err("%s Clock disable failed\n", __func__);

		if (pdata->prim_mi2s_mode == 1)
			ret = msm_cdc_pinctrl_select_sleep_state
						(pdata->prim_master_p);
		else
			ret = msm_cdc_pinctrl_select_sleep_state
						(pdata->prim_slave_p);
		if (ret)
			pr_err("%s: failed to set pri gpios to sleep: %d\n",
			       __func__, ret);
	}
}

static int sdx_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->prim_mi2s_mode = sdx_mi2s_mode;
	if (atomic_inc_return(&mi2s_ref_count) == 1) {
		if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {
			ret = afe_enable_lpass_core_shared_clock(MI2S_RX,
								 CLOCK_ON);
			if (ret < 0) {
				ret = -EINVAL;
				goto done;
			}
			iowrite32(I2S_SEL << I2S_PCM_SEL_OFFSET,
				  pdata->lpaif_pri_muxsel_virt_addr);
			if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
				if (pdata->prim_mi2s_mode == 1)
					iowrite32(PRI_TLMM_CLKS_EN_MASTER,
					pdata->lpass_mux_spkr_ctl_virt_addr);
				else
					iowrite32(PRI_TLMM_CLKS_EN_SLAVE,
					pdata->lpass_mux_spkr_ctl_virt_addr);
			} else {
				dev_err(card->dev, "%s: mux spkr ctl virt addr is NULL\n",
					__func__);

				ret = -EINVAL;
				goto err;
			}
		} else {
			dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
				__func__);

			ret = -EINVAL;
			goto done;
		}
		/*
		 * This sets the CONFIG PARAMETER WS_SRC.
		 * 1 means internal clock master mode.
		 * 0 means external clock slave mode.
		 */
		if (pdata->prim_mi2s_mode == 1) {
			ret = msm_cdc_pinctrl_select_active_state
					(pdata->prim_master_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto err;
			}
			ret = sdx_mi2s_clk_ctl(rtd, true, PRI_MI2S,
					       sdx_mi2s_rate,
					       pdata->prim_mi2s_mode);
			if (ret < 0) {
				dev_err(card->dev, "%s clock enable failed\n",
					__func__);
				goto err;
			}
			ret = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_CBS_CFS);
			if (ret < 0) {
				sdx_mi2s_clk_ctl(rtd, false, PRI_MI2S,
						 0, pdata->prim_mi2s_mode);
				dev_err(card->dev,
					"%s Set fmt for cpu dai failed\n",
					__func__);
				goto err;
			}
			ret = snd_soc_dai_set_fmt(codec_dai,
						  SND_SOC_DAIFMT_CBS_CFS |
						  SND_SOC_DAIFMT_I2S);
			if (ret < 0) {
				sdx_mi2s_clk_ctl(rtd, false, PRI_MI2S,
						 0, pdata->prim_mi2s_mode);
				dev_err(card->dev,
					"%s Set fmt for codec dai failed\n",
					__func__);
			}
			if (!strcmp(card->name, snd_soc_card_auto_sdx.name)) {
				ret = snd_soc_dai_set_sysclk(codec_dai,
						TLV_CLKIN_MCLK,
						pdata->mclk_freq,
						SND_SOC_CLOCK_OUT);
				if (ret < 0) {
					pr_err("%s Set sysclk for codec dai failed 0x%8x\n",
						__func__, ret);
				}
			}
		} else {
			/*
			 * Disable bit clk in slave mode for QC codec.
			 * Enable only mclk.
			 */
			ret = msm_cdc_pinctrl_select_active_state
					(pdata->prim_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto err;
			}
			ret = sdx_mi2s_clk_ctl(rtd, false, PRI_MI2S, 0,
					       pdata->prim_mi2s_mode);
			if (ret < 0) {
				dev_err(card->dev,
					"%s clock enable failed\n", __func__);
				goto err;
			}
			ret = snd_soc_dai_set_fmt(cpu_dai,
						  SND_SOC_DAIFMT_CBM_CFM);
			if (ret < 0) {
				dev_err(card->dev,
					"%s Set fmt for cpu dai failed\n",
					__func__);
				goto err;
			}
		}
err:
		afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_OFF);
	}
done:
	if (ret)
		atomic_dec_return(&mi2s_ref_count);
	return ret;
}

static void sdx_sec_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&sec_mi2s_ref_count) == 0) {
		ret = sdx_mi2s_clk_ctl(rtd, false, SEC_MI2S,
				       0, pdata->sec_mi2s_mode);
		if (ret < 0)
			pr_err("%s Clock disable failed\n", __func__);

		if (pdata->sec_mi2s_mode == 1)
			ret = msm_cdc_pinctrl_select_sleep_state
					(pdata->sec_master_p);
		else
			ret = msm_cdc_pinctrl_select_sleep_state
					(pdata->sec_slave_p);
		if (ret)
			pr_err("%s: failed to set sec gpios to sleep: %d\n",
			       __func__, ret);
	}
}

static int sdx_sec_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->sec_mi2s_mode = sdx_sec_mi2s_mode;
	if (atomic_inc_return(&sec_mi2s_ref_count) == 1) {
		if (pdata->lpaif_sec_muxsel_virt_addr != NULL) {
			ret = afe_enable_lpass_core_shared_clock(
					SECONDARY_I2S_RX, CLOCK_ON);
			if (ret < 0) {
				ret = -EINVAL;
				goto done;
			}
			iowrite32(I2S_SEL << I2S_PCM_SEL_OFFSET,
				  pdata->lpaif_sec_muxsel_virt_addr);

			if (pdata->lpass_mux_mic_ctl_virt_addr != NULL) {
				if (pdata->sec_mi2s_mode == 1)
					iowrite32(SEC_TLMM_CLKS_EN_MASTER,
					pdata->lpass_mux_mic_ctl_virt_addr);
				else
					iowrite32(SEC_TLMM_CLKS_EN_SLAVE,
					pdata->lpass_mux_mic_ctl_virt_addr);
			} else {
				dev_err(card->dev,
					"%s: mux spkr ctl virt addr is NULL\n",
					__func__);
				ret = -EINVAL;
				goto err;
			}
		} else {
			dev_err(card->dev, "%s lpaif_sec_muxsel_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto done;
		}
		/*
		 * This sets the CONFIG PARAMETER WS_SRC.
		 * 1 means internal clock master mode.
		 * 0 means external clock slave mode.
		 */
		if (pdata->sec_mi2s_mode == 1) {
			ret = msm_cdc_pinctrl_select_active_state
					(pdata->sec_master_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto err;
			}
			ret = sdx_mi2s_clk_ctl(rtd, true, SEC_MI2S,
					       sdx_sec_mi2s_rate,
					       pdata->sec_mi2s_mode);
			if (ret < 0) {
				dev_err(card->dev, "%s clock enable failed\n",
					__func__);
				goto err;
			}
			ret = snd_soc_dai_set_fmt(cpu_dai,
						  SND_SOC_DAIFMT_CBS_CFS);
			if (ret < 0) {
				ret = sdx_mi2s_clk_ctl(rtd, false, SEC_MI2S,
						       0,
						       pdata->sec_mi2s_mode);
				dev_err(card->dev, "%s Set fmt for cpu dai failed\n",
					__func__);
			}
		} else {
			/*
			 * Enable mclk here, if needed for external codecs.
			 * Optional. Refer primary mi2s slave interface.
			 */
			ret = msm_cdc_pinctrl_select_active_state
					(pdata->sec_slave_p);
			if (ret < 0) {
				pr_err("%s pinctrl set failed\n", __func__);
				goto err;
			}
			ret = sdx_mi2s_clk_ctl(rtd, false, SEC_MI2S, 0,
					       pdata->sec_mi2s_mode);
			if (ret < 0) {
				dev_err(card->dev,
					"%s clock enable failed\n", __func__);
				goto err;
			}
			ret = snd_soc_dai_set_fmt(cpu_dai,
						  SND_SOC_DAIFMT_CBM_CFM);
			if (ret < 0)
				dev_err(card->dev, "%s Set fmt for cpu dai failed\n",
					__func__);
		}
err:
		afe_enable_lpass_core_shared_clock(SECONDARY_I2S_RX,
						   CLOCK_OFF);
	}
done:
	if (ret)
		atomic_dec_return(&sec_mi2s_ref_count);
	return ret;
}

static struct snd_soc_ops sdx_mi2s_be_ops = {
	.startup = sdx_mi2s_startup,
	.shutdown = sdx_mi2s_shutdown,
};

static struct snd_soc_ops sdx_sec_mi2s_be_ops = {
	.startup = sdx_sec_mi2s_startup,
	.shutdown = sdx_sec_mi2s_shutdown,
};

static int sdx_mi2s_rate_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: sdx_i2s_rate  = %d", __func__, sdx_mi2s_rate);
	ucontrol->value.integer.value[0] = sdx_mi2s_rate;
	return 0;
}

static int sdx_sec_mi2s_rate_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: sdx_sec_i2s_rate  = %d", __func__, sdx_sec_mi2s_rate);
	ucontrol->value.integer.value[0] = sdx_sec_mi2s_rate;
	return 0;
}

static int sdx_mi2s_bitwidth_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	switch (sdx_mi2s_bitwidth) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ucontrol->value.integer.value[0] = 0;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		ucontrol->value.integer.value[0] = 2;
		break;
	default:
		ucontrol->value.integer.value[0] = 0;
	}
	pr_debug("%s: sdx_i2s_bitwidth = %d ucontrol->value = %d\n",
		 __func__, sdx_mi2s_bitwidth,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
};

static int sdx_mi2s_bitwidth_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_mi2s_bitwidth = SNDRV_PCM_FORMAT_S16_LE;
		break;
	case 1:
		sdx_mi2s_bitwidth = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 2:
		sdx_mi2s_bitwidth = SNDRV_PCM_FORMAT_S32_LE;
		break;
	default:
		sdx_mi2s_bitwidth = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	pr_debug("%s: sdx_i2s_bitwidth = %d ucontrol->value = %d\n",
		 __func__, sdx_mi2s_bitwidth,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_mi2s_rate_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_mi2s_rate = SAMPLE_RATE_8KHZ;
		break;
	case 1:
		sdx_mi2s_rate = SAMPLE_RATE_16KHZ;
		break;
	case 2:
	default:
		sdx_mi2s_rate = SAMPLE_RATE_48KHZ;
		break;
	}
	pr_debug("%s: sdx_i2s_rate = %d ucontrol->value = %d\n",
		 __func__, sdx_mi2s_rate,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_sec_mi2s_rate_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_sec_mi2s_rate = SAMPLE_RATE_8KHZ;
		break;
	case 1:
		sdx_sec_mi2s_rate = SAMPLE_RATE_16KHZ;
		break;
	case 2:
	default:
		sdx_sec_mi2s_rate = SAMPLE_RATE_48KHZ;
		break;
	}
	pr_debug("%s: sdx_sec_mi2s_rate = %d ucontrol->value = %d\n",
		 __func__, sdx_sec_mi2s_rate,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static void param_set_mask(struct snd_pcm_hw_params *p, int n,
			   unsigned int bit)
{
	struct snd_mask *m = NULL;

	if (bit >= SNDRV_MASK_MAX)
		return;
	if ((n >= SNDRV_PCM_HW_PARAM_FIRST_MASK) &&
	    (n <= SNDRV_PCM_HW_PARAM_LAST_MASK)) {
		m = &(p->masks[n - SNDRV_PCM_HW_PARAM_FIRST_MASK]);

		m->bits[0] = 0;
		m->bits[1] = 0;
		m->bits[bit >> 5] |= (1 << (bit & 31));
	}
}

static int sdx_mi2s_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rt,
					  struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT, sdx_mi2s_bitwidth);
	rate->min = rate->max = sdx_mi2s_rate;
	channels->min = channels->max = sdx_mi2s_rx_ch;
	return 0;
}

static int sdx_sec_mi2s_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rt,
					      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
		       SNDRV_PCM_FORMAT_S16_LE);
	rate->min = rate->max = sdx_sec_mi2s_rate;
	channels->min = channels->max = sdx_sec_mi2s_rx_ch;
	return 0;
}

static int sdx_mi2s_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rt,
					  struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT, sdx_mi2s_bitwidth);
	rate->min = rate->max = sdx_mi2s_rate;
	channels->min = channels->max = sdx_mi2s_tx_ch;
	return 0;
}

static int sdx_sec_mi2s_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rt,
					      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
		       SNDRV_PCM_FORMAT_S16_LE);
	rate->min = rate->max = sdx_sec_mi2s_rate;
	channels->min = channels->max = sdx_sec_mi2s_tx_ch;
	return 0;
}

static int sdx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rt,
				  struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
			hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
		       SNDRV_PCM_FORMAT_S16_LE);
	rate->min = rate->max = sdx_mi2s_rate;
	channels->min = sdx_mi2s_rx_ch;
	return 0;
}

static int sdx_mi2s_rx_ch_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_mi2s_rx_ch %d\n", __func__, sdx_mi2s_rx_ch);
	ucontrol->value.integer.value[0] = sdx_mi2s_rx_ch - 1;
	return 0;
}

static int sdx_sec_mi2s_rx_ch_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_sec_mi2s_rx_ch %d\n", __func__, sdx_mi2s_rx_ch);
	ucontrol->value.integer.value[0] = sdx_sec_mi2s_rx_ch - 1;
	return 0;
}

static int sdx_mi2s_rx_ch_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	sdx_mi2s_rx_ch = ucontrol->value.integer.value[0] + 1;
	pr_debug("%s sdx_mi2s_rx_ch %d\n", __func__, sdx_mi2s_rx_ch);
	return 1;
}

static int sdx_sec_mi2s_rx_ch_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	sdx_mi2s_rx_ch = ucontrol->value.integer.value[0] + 1;
	pr_debug("%s sdx_sec_mi2s_rx_ch %d\n", __func__, sdx_sec_mi2s_rx_ch);
	return 1;
}

static int sdx_mi2s_tx_ch_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_mi2s_tx_ch %d\n", __func__, sdx_mi2s_tx_ch);
	ucontrol->value.integer.value[0] = sdx_mi2s_tx_ch - 1;
	return 0;
}

static int sdx_sec_mi2s_tx_ch_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_sec_mi2s_tx_ch %d\n", __func__, sdx_mi2s_tx_ch);
	ucontrol->value.integer.value[0] = sdx_sec_mi2s_tx_ch - 1;
	return 0;
}

static int sdx_mi2s_tx_ch_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	sdx_mi2s_tx_ch = ucontrol->value.integer.value[0] + 1;
	pr_debug("%s sdx_mi2s_tx_ch %d\n", __func__, sdx_mi2s_tx_ch);
	return 1;
}

static int sdx_sec_mi2s_tx_ch_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	sdx_mi2s_tx_ch = ucontrol->value.integer.value[0] + 1;
	pr_debug("%s sdx_sec_mi2s_tx_ch %d\n", __func__, sdx_sec_mi2s_tx_ch);
	return 1;
}

static int sdx_mi2s_mode_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_mi2s_mode %d\n", __func__, sdx_mi2s_mode);
	ucontrol->value.integer.value[0] = sdx_mi2s_mode;
	return 0;
}

static int sdx_mi2s_mode_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	case 1:
		sdx_mi2s_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_mi2s_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_mi2s_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_sec_mi2s_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_sec_mi2s_mode %d\n", __func__, sdx_sec_mi2s_mode);
	ucontrol->value.integer.value[0] = sdx_sec_mi2s_mode;
	return 0;
}

static int sdx_sec_mi2s_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_sec_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	case 1:
		sdx_sec_mi2s_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_sec_mi2s_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_sec_mi2s_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_sec_mi2s_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_auxpcm_mode_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_auxpcm_mode %d\n", __func__, sdx_auxpcm_mode);
	ucontrol->value.integer.value[0] = sdx_auxpcm_mode;
	return 0;
}

static int sdx_auxpcm_mode_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	case 1:
		sdx_auxpcm_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_auxpcm_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_auxpcm_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_sec_auxpcm_mode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_sec_auxpcm_mode %d\n", __func__, sdx_sec_auxpcm_mode);
	ucontrol->value.integer.value[0] = sdx_sec_auxpcm_mode;
	return 0;
}

static int sdx_sec_auxpcm_mode_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		sdx_sec_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	case 1:
		sdx_sec_auxpcm_mode = I2S_PCM_SLAVE_MODE;
		break;
	default:
		sdx_sec_auxpcm_mode = I2S_PCM_MASTER_MODE;
		break;
	}
	pr_debug("%s: sdx_sec_auxpcm_mode = %d ucontrol->value = %d\n",
		 __func__, sdx_sec_auxpcm_mode,
		 (int)ucontrol->value.integer.value[0]);
	return 0;
}

static int sdx_mi2s_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s sdx_spk_control %d", __func__, sdx_spk_control);
	ucontrol->value.integer.value[0] = sdx_spk_control;
	return 0;
}

static void sdx_ext_control(struct snd_soc_dapm_context *dapm)
{
	pr_debug("%s sdx_spk_control %d", __func__, sdx_spk_control);

	if (sdx_spk_control == SDX_SPK_ON) {
		snd_soc_dapm_enable_pin(dapm, "Lineout_1 amp");
		snd_soc_dapm_enable_pin(dapm, "Lineout_2 amp");
	} else {
		snd_soc_dapm_disable_pin(dapm, "Lineout_1 amp");
		snd_soc_dapm_disable_pin(dapm, "Lineout_2 amp");
	}
	snd_soc_dapm_sync(dapm);
}

static int sdx_mi2s_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct snd_soc_dapm_context *dapm;

	dapm = snd_soc_component_get_dapm(component);
	pr_debug("%s()\n", __func__);

	if (sdx_spk_control == ucontrol->value.integer.value[0])
		return 0;
	sdx_spk_control = ucontrol->value.integer.value[0];
	sdx_ext_control(dapm);
	return 1;
}

static int sdx_enable_codec_ext_clk(struct snd_soc_component *component,
					int enable, bool dapm)
{
	int ret = 0;

	if (!strcmp(component->name, "tavil_codec")) {
		ret = tavil_cdc_mclk_enable(component, enable);
	} else {
		dev_err(component->dev, "%s: unknown codec to enable ext clk\n",
			__func__);
		ret = -EINVAL;
	}
	return ret;
}

static int sdx_mclk_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component;

	pr_debug("%s event %d\n", __func__, event);

	component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		sdx_enable_codec_ext_clk(component, 1, true);
		break;
	case SND_SOC_DAPM_POST_PMD:
		sdx_enable_codec_ext_clk(component, 0, true);
		break;
	}
	return 0;
}

static void sdx_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->prim_auxpcm_mode == 1)
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->prim_master_p);
	else
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->prim_slave_p);
	if (ret)
		pr_err("%s: failed to set prim gpios to sleep: %d\n",
		       __func__, ret);
}

static int sdx_auxpcm_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	pdata->prim_auxpcm_mode = sdx_auxpcm_mode;
	if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {
		ret = afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_ON);
		if (ret < 0) {
			ret = -EINVAL;
			goto done;
		}
		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
			  pdata->lpaif_pri_muxsel_virt_addr);
		if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
			if (pdata->prim_auxpcm_mode == 1)
				iowrite32(PRI_TLMM_CLKS_EN_MASTER,
					  pdata->lpass_mux_spkr_ctl_virt_addr);
			else
				iowrite32(PRI_TLMM_CLKS_EN_SLAVE,
					  pdata->lpass_mux_spkr_ctl_virt_addr);
		} else {
			dev_err(card->dev, "%s lpass_mux_spkr_ctl_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
		}
	} else {
		dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
			__func__);
		ret = -EINVAL;
		goto done;
	}

	if (pdata->prim_auxpcm_mode == 1) {
		ret = msm_cdc_pinctrl_select_active_state
						(pdata->prim_master_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
	} else {
		ret = msm_cdc_pinctrl_select_active_state(pdata->prim_slave_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
	}
	afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_OFF);
done:
	return ret;
}

static struct snd_soc_ops sdx_auxpcm_be_ops = {
	.startup = sdx_auxpcm_startup,
	.shutdown = sdx_auxpcm_shutdown,
};

static void sdx_sec_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->sec_auxpcm_mode == 1)
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->sec_master_p);
	else
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->sec_slave_p);
	if (ret)
		pr_err("%s: failed to set sec gpios to sleep: %d\n",
		       __func__, ret);
}

static int sdx_sec_auxpcm_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	pdata->sec_auxpcm_mode = sdx_sec_auxpcm_mode;
	if (pdata->lpaif_sec_muxsel_virt_addr != NULL) {
		ret = afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_ON);
		if (ret < 0) {
			ret = -EINVAL;
			goto done;
		}
		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
				pdata->lpaif_sec_muxsel_virt_addr);
		if (pdata->lpass_mux_mic_ctl_virt_addr != NULL) {
			if (pdata->sec_auxpcm_mode == 1)
				iowrite32(SEC_TLMM_CLKS_EN_MASTER,
					  pdata->lpass_mux_mic_ctl_virt_addr);
			else
				iowrite32(SEC_TLMM_CLKS_EN_SLAVE,
					  pdata->lpass_mux_mic_ctl_virt_addr);
		} else {
			dev_err(card->dev,
				"%s lpass_mux_mic_ctl_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
		}
	} else {
		dev_err(card->dev,
			"%s lpaif_sec_muxsel_virt_addr is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	if (pdata->sec_auxpcm_mode == 1) {
		ret = msm_cdc_pinctrl_select_active_state(pdata->sec_master_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
	} else {
		ret = msm_cdc_pinctrl_select_active_state(pdata->sec_slave_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
	}
	afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_OFF);
done:
	return ret;
}

static struct snd_soc_ops sdx_sec_auxpcm_be_ops = {
	.startup = sdx_sec_auxpcm_startup,
	.shutdown = sdx_sec_auxpcm_shutdown,
};

static int sdx_auxpcm_rate_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = sdx_auxpcm_rate;
	return 0;
}

static int sdx_auxpcm_rate_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		sdx_auxpcm_rate = 16000;
		break;
	case 0:
	default:
		sdx_auxpcm_rate = 8000;
		break;
	}
	return 0;
}

static int sdx_auxpcm_be_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = sdx_auxpcm_rate;
	channels->min = channels->max = 1;

	return 0;
}

static int proxy_tx_format_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	switch (proxy_tx_cfg.bit_format) {
	case SNDRV_PCM_FORMAT_S32_LE:
		ucontrol->value.integer.value[0] = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}

	pr_debug("%s: proxy_tx_format = %d, ucontrol value = %ld\n",
		 __func__, proxy_tx_cfg.bit_format,
		 ucontrol->value.integer.value[0]);
	return 0;
}

static int proxy_tx_format_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;

	switch (ucontrol->value.integer.value[0]) {
	case 2:
		proxy_tx_cfg.bit_format = SNDRV_PCM_FORMAT_S32_LE;
		break;
	case 1:
		proxy_tx_cfg.bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		proxy_tx_cfg.bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	pr_debug("%s: proxy_tx_format = %d, ucontrol value = %ld\n",
		 __func__, proxy_tx_cfg.bit_format,
		 ucontrol->value.integer.value[0]);

	return rc;
}

static int proxy_tx_sample_rate_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int sample_rate_val;

	switch (proxy_tx_cfg.sample_rate) {
	case SAMPLE_RATE_48KHZ:
		sample_rate_val = 3;
		break;
	case SAMPLING_RATE_32KHZ:
		sample_rate_val = 2;
		break;
	case SAMPLE_RATE_16KHZ:
		sample_rate_val = 1;
		break;
	case SAMPLE_RATE_8KHZ:
	default:
		sample_rate_val = 0;
		break;
	}

	ucontrol->value.integer.value[0] = sample_rate_val;
	pr_debug("%s: proxy_tx_sample_rate = %d\n", __func__,
		proxy_tx_cfg.sample_rate);
	return 0;
}

static int proxy_tx_sample_rate_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 3:
		proxy_tx_cfg.sample_rate = SAMPLE_RATE_48KHZ;
		break;
	case 2:
		proxy_tx_cfg.sample_rate = SAMPLING_RATE_32KHZ;
		break;
	case 1:
		proxy_tx_cfg.sample_rate = SAMPLE_RATE_16KHZ;
		break;
	case 0:
		proxy_tx_cfg.sample_rate = SAMPLE_RATE_8KHZ;
		break;
	default:
		proxy_tx_cfg.sample_rate = SAMPLE_RATE_48KHZ;
		break;
	}

	pr_debug("%s: control value = %ld, proxy_tx_sample_rate = %d\n",
		__func__, ucontrol->value.integer.value[0],
		proxy_tx_cfg.sample_rate);
	return 0;
}

static int proxy_tx_psize_get(struct snd_kcontrol *kcontrol,
              struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: proxy_tx period_size = %d\n",
		__func__, proxy_tx_cfg.period_size);
	ucontrol->value.integer.value[0] = proxy_tx_cfg.period_size;

	return 0;
}

static int proxy_tx_psize_put(struct snd_kcontrol *kcontrol,
              struct snd_ctl_elem_value *ucontrol)
{
	proxy_tx_cfg.period_size = ucontrol->value.integer.value[0];

	pr_debug("%s: proxy_tx period_size = %d\n",
	__func__, proxy_tx_cfg.period_size);

	return 0;
}

static int tdm_get_sample_rate(int value)
{
	int sample_rate = 0;

	switch (value) {
	case 0:
		sample_rate = SAMPLE_RATE_8KHZ;
		break;
	case 1:
		sample_rate = SAMPLE_RATE_16KHZ;
		break;
	case 2:
		sample_rate = SAMPLING_RATE_32KHZ;
		break;
	case 3:
		sample_rate = SAMPLE_RATE_48KHZ;
		break;
	case 4:
		sample_rate = SAMPLING_RATE_176P4KHZ;
		break;
	case 5:
		sample_rate = SAMPLING_RATE_352P8KHZ;
		break;
	default:
		sample_rate = SAMPLE_RATE_48KHZ;
		break;
	}
	return sample_rate;
}

static int tdm_get_sample_rate_val(int sample_rate)
{
	int sample_rate_val = 0;

	switch (sample_rate) {
	case SAMPLE_RATE_8KHZ:
		sample_rate_val = 0;
		break;
	case SAMPLE_RATE_16KHZ:
		sample_rate_val = 1;
		break;
	case SAMPLING_RATE_32KHZ:
		sample_rate_val = 2;
		break;
	case SAMPLE_RATE_48KHZ:
		sample_rate_val = 3;
		break;
	case SAMPLING_RATE_176P4KHZ:
		sample_rate_val = 4;
		break;
	case SAMPLING_RATE_352P8KHZ:
		sample_rate_val = 5;
		break;
	default:
		sample_rate_val = 3;
		break;
	}
	return sample_rate_val;
}

static int tdm_get_port_idx(struct snd_kcontrol *kcontrol,
			    struct tdm_port *port)
{
	if (port) {
		if (strnstr(kcontrol->id.name, "PRI",
		    sizeof(kcontrol->id.name))) {
			port->mode = TDM_PRI;
		} else if (strnstr(kcontrol->id.name, "SEC",
		    sizeof(kcontrol->id.name))) {
			port->mode = TDM_SEC;
		} else {
			pr_err("%s: unsupported mode in: %s",
				__func__, kcontrol->id.name);
			return -EINVAL;
		}

		if (strnstr(kcontrol->id.name, "RX_0",
		    sizeof(kcontrol->id.name)) ||
		    strnstr(kcontrol->id.name, "TX_0",
		    sizeof(kcontrol->id.name))) {
			port->channel = TDM_0;
		} else if (strnstr(kcontrol->id.name, "RX_1",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_1",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_1;
		} else if (strnstr(kcontrol->id.name, "RX_2",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_2",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_2;
		} else if (strnstr(kcontrol->id.name, "RX_3",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_3",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_3;
		} else if (strnstr(kcontrol->id.name, "RX_4",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_4",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_4;
		} else if (strnstr(kcontrol->id.name, "RX_5",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_5",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_5;
		} else if (strnstr(kcontrol->id.name, "RX_6",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_6",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_6;
		} else if (strnstr(kcontrol->id.name, "RX_7",
			   sizeof(kcontrol->id.name)) ||
			   strnstr(kcontrol->id.name, "TX_7",
			   sizeof(kcontrol->id.name))) {
			port->channel = TDM_7;
		} else {
			pr_err("%s: unsupported channel in: %s",
				__func__, kcontrol->id.name);
			return -EINVAL;
		}
	} else
		return -EINVAL;
	return 0;
}

static int sdx_tdm_rx_sample_rate_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		ucontrol->value.enumerated.item[0] = tdm_get_sample_rate_val(
			tdm_rx_cfg[port.mode][port.channel].sample_rate);

		pr_debug("%s: tdm_rx_sample_rate = %d, item = %d\n", __func__,
			 tdm_rx_cfg[port.mode][port.channel].sample_rate,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_rx_sample_rate_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		tdm_rx_cfg[port.mode][port.channel].sample_rate =
			tdm_get_sample_rate(ucontrol->value.enumerated.item[0]);

		pr_debug("%s: tdm_rx_sample_rate = %d, item = %d\n", __func__,
			 tdm_rx_cfg[port.mode][port.channel].sample_rate,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_tx_sample_rate_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		ucontrol->value.enumerated.item[0] = tdm_get_sample_rate_val(
			tdm_tx_cfg[port.mode][port.channel].sample_rate);

		pr_debug("%s: tdm_tx_sample_rate = %d, item = %d\n", __func__,
			 tdm_tx_cfg[port.mode][port.channel].sample_rate,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_tx_sample_rate_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		tdm_tx_cfg[port.mode][port.channel].sample_rate =
			tdm_get_sample_rate(ucontrol->value.enumerated.item[0]);

		pr_debug("%s: tdm_tx_sample_rate = %d, item = %d\n", __func__,
			 tdm_tx_cfg[port.mode][port.channel].sample_rate,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int tdm_get_format(int value)
{
	int format = 0;

	switch (value) {
	case 0:
		format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	case 1:
		format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 2:
		format = SNDRV_PCM_FORMAT_S32_LE;
		break;
	default:
		format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	return format;
}

static int tdm_get_format_val(int format)
{
	int value = 0;

	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		value = 0;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		value = 1;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		value = 2;
		break;
	default:
		value = 0;
		break;
	}
	return value;
}

static int sdx_tdm_rx_format_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		ucontrol->value.enumerated.item[0] = tdm_get_format_val(
				tdm_rx_cfg[port.mode][port.channel].bit_format);

		pr_debug("%s: tdm_rx_bit_format = %d, item = %d\n", __func__,
			 tdm_rx_cfg[port.mode][port.channel].bit_format,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_rx_format_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		tdm_rx_cfg[port.mode][port.channel].bit_format =
			tdm_get_format(ucontrol->value.enumerated.item[0]);

		pr_debug("%s: tdm_rx_bit_format = %d, item = %d\n", __func__,
			 tdm_rx_cfg[port.mode][port.channel].bit_format,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_tx_format_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		ucontrol->value.enumerated.item[0] = tdm_get_format_val(
				tdm_tx_cfg[port.mode][port.channel].bit_format);

		pr_debug("%s: tdm_tx_bit_format = %d, item = %d\n", __func__,
			 tdm_tx_cfg[port.mode][port.channel].bit_format,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_tx_format_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		tdm_tx_cfg[port.mode][port.channel].bit_format =
			tdm_get_format(ucontrol->value.enumerated.item[0]);

		pr_debug("%s: tdm_tx_bit_format = %d, item = %d\n", __func__,
			 tdm_tx_cfg[port.mode][port.channel].bit_format,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_rx_ch_get(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {

		ucontrol->value.enumerated.item[0] =
			tdm_rx_cfg[port.mode][port.channel].channels - 1;

		pr_debug("%s: tdm_rx_ch = %d, item = %d\n", __func__,
			 tdm_rx_cfg[port.mode][port.channel].channels - 1,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_rx_ch_put(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		tdm_rx_cfg[port.mode][port.channel].channels =
			ucontrol->value.enumerated.item[0] + 1;

		pr_debug("%s: tdm_rx_ch = %d, item = %d\n", __func__,
			 tdm_rx_cfg[port.mode][port.channel].channels,
			 ucontrol->value.enumerated.item[0] + 1);
	}
	return ret;
}

static int proxy_tx_ch_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: proxy_tx channels = %d\n",
		 __func__, proxy_tx_cfg.channels);
	ucontrol->value.integer.value[0] = proxy_tx_cfg.channels - 1;

	return 0;
}

static int proxy_tx_ch_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	proxy_tx_cfg.channels = ucontrol->value.integer.value[0] + 1;
	pr_debug("%s: proxy_tx channels = %d\n",
		 __func__, proxy_tx_cfg.channels);

	return 1;
}

static int sdx_tdm_tx_ch_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		ucontrol->value.enumerated.item[0] =
			tdm_tx_cfg[port.mode][port.channel].channels - 1;

		pr_debug("%s: tdm_tx_ch = %d, item = %d\n", __func__,
			 tdm_tx_cfg[port.mode][port.channel].channels - 1,
			 ucontrol->value.enumerated.item[0]);
	}
	return ret;
}

static int sdx_tdm_tx_ch_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct tdm_port port;
	int ret = tdm_get_port_idx(kcontrol, &port);

	if (ret) {
		pr_err("%s: unsupported control: %s",
			__func__, kcontrol->id.name);
	} else {
		tdm_tx_cfg[port.mode][port.channel].channels =
			ucontrol->value.enumerated.item[0] + 1;

		pr_debug("%s: tdm_tx_ch = %d, item = %d\n", __func__,
			 tdm_tx_cfg[port.mode][port.channel].channels,
			 ucontrol->value.enumerated.item[0] + 1);
	}
	return ret;
}

static int sdx_tdm_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	switch (cpu_dai->id) {
	case AFE_PORT_ID_PRIMARY_TDM_RX:
		channels->min = channels->max =
				tdm_rx_cfg[TDM_PRI][TDM_0].channels;
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				tdm_rx_cfg[TDM_PRI][TDM_0].bit_format);
		rate->min = rate->max =
				tdm_rx_cfg[TDM_PRI][TDM_0].sample_rate;
		break;

	case AFE_PORT_ID_PRIMARY_TDM_TX:
		channels->min = channels->max =
				tdm_tx_cfg[TDM_PRI][TDM_0].channels;
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				tdm_tx_cfg[TDM_PRI][TDM_0].bit_format);
		rate->min = rate->max =
				tdm_tx_cfg[TDM_PRI][TDM_0].sample_rate;
		break;

	case AFE_PORT_ID_SECONDARY_TDM_RX:
		channels->min = channels->max =
				tdm_rx_cfg[TDM_SEC][TDM_0].channels;
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				tdm_rx_cfg[TDM_SEC][TDM_0].bit_format);
		rate->min = rate->max =
				tdm_rx_cfg[TDM_SEC][TDM_0].sample_rate;
		break;

	case AFE_PORT_ID_SECONDARY_TDM_TX:
		channels->min = channels->max =
				tdm_tx_cfg[TDM_SEC][TDM_0].channels;
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				tdm_tx_cfg[TDM_SEC][TDM_0].bit_format);
		rate->min = rate->max =
				tdm_tx_cfg[TDM_SEC][TDM_0].sample_rate;
		break;

	default:
		pr_err("%s: dai id 0x%x not supported\n",
			__func__, cpu_dai->id);
		return -EINVAL;
	}

	pr_debug("%s: dai id = 0x%x channels = %d rate = %d format = 0x%x\n",
		__func__, cpu_dai->id, channels->max, rate->max,
		params_format(params));

	return 0;
}

static int sdX_tdm_snd_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	int slot_width = 32;
	int channels, slots;
	unsigned int slot_mask, rate, clk_freq;
	unsigned int slot_offset[8] = {0, 4, 8, 12, 16, 20, 24, 28};

	pr_debug("%s: dai id = 0x%x\n", __func__, cpu_dai->id);

	/* currently only supporting TDM_RX_0 and TDM_TX_0 */
	switch (cpu_dai->id) {
	case AFE_PORT_ID_PRIMARY_TDM_RX:
		slots = tdm_rx_cfg[TDM_PRI][TDM_0].channels;
		break;
	case AFE_PORT_ID_SECONDARY_TDM_RX:
		slots = tdm_rx_cfg[TDM_SEC][TDM_0].channels;
		break;
	case AFE_PORT_ID_PRIMARY_TDM_TX:
		slots = tdm_tx_cfg[TDM_PRI][TDM_0].channels;
		break;
	case AFE_PORT_ID_SECONDARY_TDM_TX:
		slots = tdm_tx_cfg[TDM_SEC][TDM_0].channels;
		break;
	default:
		pr_err("%s: dai id 0x%x not supported\n",
			__func__, cpu_dai->id);
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*2 slot config - bits 0 and 1 set for the first two slots */
		slot_mask = 0x0000FFFF >> (16-slots);
		channels = slots;

		pr_debug("%s: tdm rx slot_width %d slots %d\n",
			__func__, slot_width, slots);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, slot_mask,
			slots, slot_width);
		if (ret < 0) {
			pr_err("%s: failed to set tdm rx slot, err:%d\n",
				__func__, ret);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai,
			0, NULL, channels, slot_offset);
		if (ret < 0) {
			pr_err("%s: failed to set tdm rx channel map, err:%d\n",
				__func__, ret);
			goto end;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/*2 slot config - bits 0 and 1 set for the first two slots */
		slot_mask = 0x0000FFFF >> (16-slots);
		channels = slots;

		pr_debug("%s: tdm tx slot_width %d slots %d\n",
			__func__, slot_width, slots);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, slot_mask, 0,
			slots, slot_width);
		if (ret < 0) {
			pr_err("%s: failed to set tdm tx slot, err:%d\n",
				__func__, ret);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai,
			channels, slot_offset, 0, NULL);
		if (ret < 0) {
			pr_err("%s: failed to set tdm tx channel map, err:%d\n",
				__func__, ret);
			goto end;
		}
	} else {
		ret = -EINVAL;
		pr_err("%s: invalid use case, err:%d\n",
			__func__, ret);
		goto end;
	}

	rate = params_rate(params);
	clk_freq = rate * slot_width * slots;
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, clk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		pr_err("%s: failed to set tdm clk, err:%d\n",
			__func__, ret);

end:
	return ret;
}

static void sdx_pri_tdm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->prim_tdm_mode == 1)
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->prim_master_p);
	else
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->prim_slave_p);
	if (ret)
		pr_err("%s: failed to set prim gpios to sleep: %d\n",
		       __func__, ret);
}

static int sdx_pri_tdm_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	pdata->prim_tdm_mode = sdx_prim_tdm_mode;
	if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {
		ret = afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_ON);
		if (ret < 0) {
			ret = -EINVAL;
			goto done;
		}
		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
			  pdata->lpaif_pri_muxsel_virt_addr);
		if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
			if (pdata->prim_tdm_mode == 1)
				iowrite32(PRI_TLMM_CLKS_EN_MASTER,
					  pdata->lpass_mux_spkr_ctl_virt_addr);
			else
				iowrite32(PRI_TLMM_CLKS_EN_SLAVE,
					  pdata->lpass_mux_spkr_ctl_virt_addr);
		} else {
			dev_err(card->dev, "%s lpass_mux_spkr_ctl_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto err;
		}
	} else {
		dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
			__func__);
		ret = -EINVAL;
		goto done;
	}

	if (pdata->prim_tdm_mode == 1) {
		ret = msm_cdc_pinctrl_select_active_state
						(pdata->prim_master_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
			goto err;
	} else {
		ret = msm_cdc_pinctrl_select_active_state(pdata->prim_slave_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
			goto err;
	}
err:
	afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_OFF);
done:
	return ret;
}

static struct snd_soc_ops sdx_pri_tdm_be_ops = {
	.hw_params = sdX_tdm_snd_hw_params,
	.startup = sdx_pri_tdm_startup,
	.shutdown = sdx_pri_tdm_shutdown,
};

static void sdx_sec_tdm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	if (pdata->sec_tdm_mode == 1)
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->sec_master_p);
	else
		ret = msm_cdc_pinctrl_select_sleep_state(pdata->sec_slave_p);
	if (ret)
		pr_err("%s: failed to set sec gpios to sleep: %d\n",
		       __func__, ret);
}

static int sdx_sec_tdm_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	pdata->sec_tdm_mode = sdx_sec_tdm_mode;
	if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {
		ret = afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_ON);
		if (ret < 0) {
			ret = -EINVAL;
			goto done;
		}
		iowrite32(PCM_SEL << I2S_PCM_SEL_OFFSET,
			  pdata->lpaif_pri_muxsel_virt_addr);
		if (pdata->lpass_mux_spkr_ctl_virt_addr != NULL) {
			if (pdata->sec_tdm_mode == 1)
				iowrite32(SEC_TLMM_CLKS_EN_MASTER,
					  pdata->lpass_mux_spkr_ctl_virt_addr);
			else
				iowrite32(SEC_TLMM_CLKS_EN_SLAVE,
					  pdata->lpass_mux_spkr_ctl_virt_addr);
		} else {
			dev_err(card->dev, "%s lpass_mux_spkr_ctl_virt_addr is NULL\n",
				__func__);
			ret = -EINVAL;
			goto err;
		}
	} else {
		dev_err(card->dev, "%s lpaif_pri_muxsel_virt_addr is NULL\n",
			__func__);
		ret = -EINVAL;
		goto done;
	}

	if (pdata->sec_tdm_mode == 1) {
		ret = msm_cdc_pinctrl_select_active_state
						(pdata->prim_master_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
			goto err;
	} else {
		ret = msm_cdc_pinctrl_select_active_state(pdata->prim_slave_p);
		if (ret < 0)
			pr_err("%s pinctrl set failed\n", __func__);
			goto err;
	}
err:
	afe_enable_lpass_core_shared_clock(MI2S_RX, CLOCK_OFF);
done:
	return ret;
}

static struct snd_soc_ops sdx_sec_tdm_be_ops = {
	.hw_params = sdX_tdm_snd_hw_params,
	.startup = sdx_sec_tdm_startup,
	.shutdown = sdx_sec_tdm_shutdown,
};

static const struct snd_soc_dapm_widget sdx_dapm_widgets[] = {

	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
	sdx_mclk_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SPK("Lineout_1 amp", NULL),
	SND_SOC_DAPM_SPK("Lineout_3 amp", NULL),
	SND_SOC_DAPM_SPK("Lineout_2 amp", NULL),
	SND_SOC_DAPM_SPK("Lineout_4 amp", NULL),
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCRight Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCLeft Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Analog Mic4", NULL),
	SND_SOC_DAPM_MIC("Analog Mic6", NULL),
	SND_SOC_DAPM_MIC("Analog Mic7", NULL),
	SND_SOC_DAPM_MIC("Analog Mic8", NULL),

	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("Digital Mic4", NULL),
	SND_SOC_DAPM_MIC("Digital Mic5", NULL),
	SND_SOC_DAPM_MIC("Digital Mic6", NULL),
};

static struct snd_soc_dapm_route wcd_audio_paths[] = {
	{"MIC BIAS1", NULL, "MCLK"},
	{"MIC BIAS2", NULL, "MCLK"},
	{"MIC BIAS3", NULL, "MCLK"},
	{"MIC BIAS4", NULL, "MCLK"},
};

static const char *const spk_function[] = {"Off", "On"};
static const char *const hifi_function[] = {"Off", "On"};
static const char *const mi2s_rx_ch_text[] = {"One", "Two"};
static const char *const mi2s_tx_ch_text[] = {"One", "Two"};
static const char *const auxpcm_rate_text[] = {"rate_8000", "rate_16000"};

static const char *const mi2s_rate_text[] = {"rate_8000",
						"rate_16000", "rate_48000"};
static const char *const mode_text[] = {"master", "slave"};

static char const *proxy_ch_text[] = {"One", "Two", "Three", "Four", "Five",
					"Six", "Seven", "Eight"};
static char const *proxy_bit_format_text[] = {"S16_LE", "S24_LE", "S32_LE"};
static char const *proxy_sample_rate_text[] = {"KHZ_8", "KHZ_16", "KHZ_32",
						"KHZ_48"};

static char const *tdm_ch_text[] = {"One", "Two", "Three", "Four",
				    "Five", "Six", "Seven", "Eight"};
static char const *tdm_bit_format_text[] = {"S16_LE", "S24_LE", "S32_LE"};
static char const *tdm_sample_rate_text[] = {"KHZ_8", "KHZ_16", "KHZ_32",
					     "KHZ_48", "KHZ_176P4",
					     "KHZ_352P8"};
static char const *mi2s_bit_format_text[] = {"S16_LE", "S24_LE", "S32_LE"};

static const struct soc_enum sdx_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, mi2s_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(2, mi2s_tx_ch_text),
	SOC_ENUM_SINGLE_EXT(2, auxpcm_rate_text),
	SOC_ENUM_SINGLE_EXT(3, mi2s_rate_text),
	SOC_ENUM_SINGLE_EXT(2, hifi_function),
	SOC_ENUM_SINGLE_EXT(2, mode_text),
	SOC_ENUM_SINGLE_EXT(3, mi2s_bit_format_text),
};

static SOC_ENUM_SINGLE_EXT_DECL(tdm_tx_chs, tdm_ch_text);
static SOC_ENUM_SINGLE_EXT_DECL(tdm_tx_format, tdm_bit_format_text);
static SOC_ENUM_SINGLE_EXT_DECL(tdm_tx_sample_rate, tdm_sample_rate_text);
static SOC_ENUM_SINGLE_EXT_DECL(tdm_rx_chs, tdm_ch_text);
static SOC_ENUM_SINGLE_EXT_DECL(tdm_rx_format, tdm_bit_format_text);
static SOC_ENUM_SINGLE_EXT_DECL(tdm_rx_sample_rate, tdm_sample_rate_text);

static SOC_ENUM_SINGLE_EXT_DECL(proxy_tx_sample_rate, proxy_sample_rate_text);
static SOC_ENUM_SINGLE_EXT_DECL(proxy_tx_chs, proxy_ch_text);
static SOC_ENUM_SINGLE_EXT_DECL(proxy_tx_format, proxy_bit_format_text);

static const struct snd_kcontrol_new sdx_snd_controls[] = {
	SOC_ENUM_EXT("Speaker Function",   sdx_enum[0],
				 sdx_mi2s_get_spk,
				 sdx_mi2s_set_spk),
	SOC_ENUM_EXT("MI2S_RX Channels",   sdx_enum[1],
				 sdx_mi2s_rx_ch_get,
				 sdx_mi2s_rx_ch_put),
	SOC_ENUM_EXT("MI2S_TX Channels",   sdx_enum[2],
				 sdx_mi2s_tx_ch_get,
				 sdx_mi2s_tx_ch_put),
	SOC_ENUM_EXT("AUX PCM SampleRate", sdx_enum[3],
				 sdx_auxpcm_rate_get,
				 sdx_auxpcm_rate_put),
	SOC_ENUM_EXT("MI2S SampleRate", sdx_enum[4],
				 sdx_mi2s_rate_get,
				 sdx_mi2s_rate_put),
	SOC_ENUM_EXT("MI2S BitWidth", sdx_enum[7],
				 sdx_mi2s_bitwidth_get,
				 sdx_mi2s_bitwidth_put),
	SOC_ENUM_EXT("SEC_MI2S_RX Channels", sdx_enum[1],
				 sdx_sec_mi2s_rx_ch_get,
				 sdx_sec_mi2s_rx_ch_put),
	SOC_ENUM_EXT("SEC_MI2S_TX Channels", sdx_enum[2],
				 sdx_sec_mi2s_tx_ch_get,
				 sdx_sec_mi2s_tx_ch_put),
	SOC_ENUM_EXT("SEC MI2S SampleRate", sdx_enum[4],
				 sdx_sec_mi2s_rate_get,
				 sdx_sec_mi2s_rate_put),
	SOC_ENUM_EXT("MI2S Mode", sdx_enum[6],
				 sdx_mi2s_mode_get,
				 sdx_mi2s_mode_put),
	SOC_ENUM_EXT("SEC_MI2S Mode", sdx_enum[6],
				 sdx_sec_mi2s_mode_get,
				 sdx_sec_mi2s_mode_put),
	SOC_ENUM_EXT("AUXPCM Mode", sdx_enum[6],
				 sdx_auxpcm_mode_get,
				 sdx_auxpcm_mode_put),
	SOC_ENUM_EXT("SEC_AUXPCM Mode", sdx_enum[6],
				 sdx_sec_auxpcm_mode_get,
				 sdx_sec_auxpcm_mode_put),
	SOC_SINGLE_EXT("PROXY_TX PeriodSize", SND_SOC_NOPM, 0, 1, 0,
				proxy_tx_psize_get,
				proxy_tx_psize_put),
	SOC_ENUM_EXT("PROXY_TX SampleRate", proxy_tx_sample_rate,
				proxy_tx_sample_rate_get,
				proxy_tx_sample_rate_put),
	SOC_ENUM_EXT("PROXY_TX Format", proxy_tx_format,
				proxy_tx_format_get,
				proxy_tx_format_put),
	SOC_ENUM_EXT("PROXY_TX Channels", proxy_tx_chs,
				proxy_tx_ch_get,
				proxy_tx_ch_put),
	SOC_ENUM_EXT("PRI_TDM_RX_0 SampleRate", tdm_rx_sample_rate,
			sdx_tdm_rx_sample_rate_get,
			sdx_tdm_rx_sample_rate_put),
	SOC_ENUM_EXT("PRI_TDM_TX_0 SampleRate", tdm_tx_sample_rate,
			sdx_tdm_tx_sample_rate_get,
			sdx_tdm_tx_sample_rate_put),
	SOC_ENUM_EXT("PRI_TDM_RX_0 Format", tdm_rx_format,
			sdx_tdm_rx_format_get,
			sdx_tdm_rx_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_0 Format", tdm_tx_format,
			sdx_tdm_tx_format_get,
			sdx_tdm_tx_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_0 Channels", tdm_rx_chs,
			sdx_tdm_rx_ch_get,
			sdx_tdm_rx_ch_put),
	SOC_ENUM_EXT("PRI_TDM_TX_0 Channels", tdm_tx_chs,
			sdx_tdm_tx_ch_get,
			sdx_tdm_tx_ch_put),
	SOC_ENUM_EXT("SEC_TDM_RX_0 SampleRate", tdm_rx_sample_rate,
			sdx_tdm_rx_sample_rate_get,
			sdx_tdm_rx_sample_rate_put),
	SOC_ENUM_EXT("SEC_TDM_TX_0 SampleRate", tdm_tx_sample_rate,
			sdx_tdm_tx_sample_rate_get,
			sdx_tdm_tx_sample_rate_put),
	SOC_ENUM_EXT("SEC_TDM_RX_0 Format", tdm_rx_format,
			sdx_tdm_rx_format_get,
			sdx_tdm_rx_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_0 Format", tdm_tx_format,
			sdx_tdm_tx_format_get,
			sdx_tdm_tx_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_0 Channels", tdm_rx_chs,
			sdx_tdm_rx_ch_get,
			sdx_tdm_rx_ch_put),
	SOC_ENUM_EXT("SEC_TDM_TX_0 Channels", tdm_tx_chs,
			sdx_tdm_tx_ch_get,
			sdx_tdm_tx_ch_put),

};

static int sdx_mi2s_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_dapm_context *dapm;
	struct snd_card *card;
	struct snd_info_entry *entry;
	struct snd_soc_component *component = NULL;
	struct sdx_machine_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

	u8 spkleft_ports[WSA881X_MAX_SWR_PORTS] = {100, 101, 102, 106};
	u8 spkright_ports[WSA881X_MAX_SWR_PORTS] = {103, 104, 105, 107};
	u8 spkleft_port_types[WSA881X_MAX_SWR_PORTS] = {SPKR_L, SPKR_L_COMP,
				SPKR_L_BOOST, SPKR_L_VI};
	u8 spkright_port_types[WSA881X_MAX_SWR_PORTS] = {SPKR_R, SPKR_R_COMP,
				SPKR_R_BOOST, SPKR_R_VI};
	unsigned int ch_rate[WSA881X_MAX_SWR_PORTS] = {2400, 600, 300, 1200};
	unsigned int ch_mask[WSA881X_MAX_SWR_PORTS] = {0x1, 0xF, 0x3, 0x3};

	component = snd_soc_rtdcom_lookup(rtd, "tavil_codec");
	if (!component) {
		pr_err("%s: tavil_codec component is NULL\n", __func__);
		return -EINVAL;
	}

	rtd->pmdown_time = 0;
	ret = snd_soc_add_component_controls(component, sdx_snd_controls,
					 ARRAY_SIZE(sdx_snd_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, %d\n",
		       __func__, ret);
		goto done;
	}

	dapm = snd_soc_component_get_dapm(component);
	snd_soc_dapm_new_controls(dapm, sdx_dapm_widgets,
				  ARRAY_SIZE(sdx_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, wcd_audio_paths,
				ARRAY_SIZE(wcd_audio_paths));

	/*
	 * After DAPM Enable pins always
	 * DAPM SYNC needs to be called.
	 */
	snd_soc_dapm_enable_pin(dapm, "Lineout_1 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_3 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_2 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_4 amp");

	snd_soc_dapm_ignore_suspend(dapm, "Lineout_1 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Lineout_3 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Lineout_2 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Lineout_4 amp");
	snd_soc_dapm_ignore_suspend(dapm, "Handset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "ANCRight Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "ANCLeft Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic1");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic2");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic3");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic4");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic5");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic6");

	snd_soc_dapm_ignore_suspend(dapm, "MADINPUT");
	snd_soc_dapm_ignore_suspend(dapm, "MAD_CPE_INPUT");
	snd_soc_dapm_ignore_suspend(dapm, "EAR");
	snd_soc_dapm_ignore_suspend(dapm, "LINEOUT1");
	snd_soc_dapm_ignore_suspend(dapm, "LINEOUT2");
	snd_soc_dapm_ignore_suspend(dapm, "ANC EAR");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC4");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC4");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "DMIC0");
	snd_soc_dapm_ignore_suspend(dapm, "SPK1 OUT");
	snd_soc_dapm_ignore_suspend(dapm, "SPK2 OUT");
	snd_soc_dapm_ignore_suspend(dapm, "HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "ANC HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "ANC HPHR");

	snd_soc_dapm_sync(dapm);

	wcd_mbhc_cfg.calibration = def_tavil_mbhc_cal();
	if (wcd_mbhc_cfg.calibration)
		ret = tavil_mbhc_hs_detect(component, &wcd_mbhc_cfg);
	else
		ret = -ENOMEM;

	card = rtd->card->snd_card;
	entry = msm_snd_info_create_subdir(card->module, "codecs",
				       card->proc_root);
	if (!entry) {
		pr_debug("%s: Cannot create codecs module entry\n",
			 __func__);
		ret = 0;
		goto done;
	}
	pdata->codec_root = entry;
	tavil_codec_info_create_codec_entry(pdata->codec_root, component);

	if (pdata->wsa_max_devs > 0) {
		component = snd_soc_rtdcom_lookup(rtd, "wsa-codec.1");
		if (!component) {
			pr_err("%s: wsa-codec.1 component is NULL\n", __func__);
			return -EINVAL;
		}

		dapm = snd_soc_component_get_dapm(component);

		wsa881x_set_channel_map(component, &spkleft_ports[0],
				WSA881X_MAX_SWR_PORTS, &ch_mask[0],
				&ch_rate[0], &spkleft_port_types[0]);
		if (dapm->component)
			snd_soc_dapm_ignore_suspend(dapm, "SpkrLeft IN");
	}

		/* If current platform has more than one WSA */
	if (pdata->wsa_max_devs > 1) {
		component = snd_soc_rtdcom_lookup(rtd, "wsa-codec.2");
		if (!component) {
			pr_err("%s: wsa-codec.2 component is NULL\n", __func__);
			return -EINVAL;
		}

		dapm = snd_soc_component_get_dapm(component);

		wsa881x_set_channel_map(component, &spkright_ports[0],
				WSA881X_MAX_SWR_PORTS, &ch_mask[0],
				&ch_rate[0], &spkright_port_types[0]);
		if (dapm->component) {
			snd_soc_dapm_ignore_suspend(dapm, "SpkrRight IN");
			snd_soc_dapm_ignore_suspend(dapm, "SpkrRight SPKR");
		}
	}
	pdata = snd_soc_card_get_drvdata(component->card);
	if (pdata && pdata->codec_root)
		wsa881x_codec_info_create_codec_entry(pdata->codec_root, component);

	snd_soc_dapm_sync(dapm);
done:
	return ret;
}

static int sdx_mi2s_audrx_init_auto(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_component *component = NULL;

	component = snd_soc_rtdcom_lookup(rtd, "msm-pcm-routing-v2");
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	rtd->pmdown_time = 0;
	ret = snd_soc_add_component_controls(component, sdx_snd_controls,
					 ARRAY_SIZE(sdx_snd_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, %d\n",
		       __func__, ret);
		goto done;
	}
done:
	return ret;
}

static void *def_tavil_mbhc_cal(void)
{
	void *tavil_wcd_cal;
	struct wcd_mbhc_btn_detect_cfg *btn_cfg;
	u16 *btn_high;

	tavil_wcd_cal = kzalloc(WCD_MBHC_CAL_SIZE(WCD_MBHC_DEF_BUTTONS,
				WCD9XXX_MBHC_DEF_RLOADS), GFP_KERNEL);
	if (!tavil_wcd_cal)
		return NULL;

#define S(X, Y) ((WCD_MBHC_CAL_PLUG_TYPE_PTR(tavil_wcd_cal)->X) = (Y))
	S(v_hs_max, 1600);
#undef S
#define S(X, Y) ((WCD_MBHC_CAL_BTN_DET_PTR(tavil_wcd_cal)->X) = (Y))
	S(num_btn, WCD_MBHC_DEF_BUTTONS);
#undef S

	btn_cfg = WCD_MBHC_CAL_BTN_DET_PTR(tavil_wcd_cal);
	btn_high = ((void *)&btn_cfg->_v_btn_low) +
		   (sizeof(btn_cfg->_v_btn_low[0]) * btn_cfg->num_btn);

	btn_high[0] = 75;
	btn_high[1] = 150;
	btn_high[2] = 237;
	btn_high[3] = 500;
	btn_high[4] = 500;
	btn_high[5] = 500;
	btn_high[6] = 500;
	btn_high[7] = 500;

	return tavil_wcd_cal;
}

/* Digital audio interface connects codec <---> CPU */
static struct snd_soc_dai_link sdx_common_dai_links[] = {
	/* FrontEnd DAI Links */
	{
	/*hw:x,0*/
		.name = SDX_DAILINK_NAME(Media1),
		.stream_name = "MultiMedia1",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* This dainlink has playback support */
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_MULTIMEDIA1,
		SND_SOC_DAILINK_REG(multimedia1),
	},
	{
	/*hw:x,1*/
		.name = SDX_DAILINK_NAME(Media2),
		.stream_name = "MultiMedia2",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		/* this dainlink has playback support */
		.id = MSM_FRONTEND_DAI_MULTIMEDIA2,
		SND_SOC_DAILINK_REG(multimedia2),
	},
	{
	/*hw:x,2*/
		.name = "SDX VoiceMMode1",
		.stream_name = "VoiceMMode1",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* This dainlink has Voice support */
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_VOICEMMODE1,
		SND_SOC_DAILINK_REG(voicemmode1),
	},
	{
	/*hw:x,3*/
		.name = "MSM VoIP",
		.stream_name = "VoIP",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_VOIP,
		SND_SOC_DAILINK_REG(msmvoip),
	},
	{
	/*hw:x,4*/
		.name = SDX_DAILINK_NAME(Media3),
		.stream_name = "MultiMedia3",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_MULTIMEDIA3,
		SND_SOC_DAILINK_REG(multimedia3),
	},
	{
	/*hw:x,5*/
		.name = "Primary MI2S RX Hostless",
		.stream_name = "Primary MI2S_RX Hostless Playback",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_mi2s_rx_hostless),
	},
	{
	/*hw:x,6*/
		.name = SDX_DAILINK_NAME(Compress2),
		.stream_name = "COMPR2",
		.dynamic = 1,
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		.async_ops = ASYNC_DPCM_SND_SOC_HW_PARAMS,
#endif /* CONFIG_AUDIO_QGKI */
		.dpcm_playback = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_MULTIMEDIA7,
		SND_SOC_DAILINK_REG(multimedia7),
	},
	{
	/*hw:x,7*/
		.name = "MSM AFE-PCM RX",
		.stream_name = "AFE-PROXY RX",
		.dpcm_playback = 1,
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(afepcm_rx),
	},
	{
	/*hw:x,8*/
		.name = "MSM AFE-PCM TX",
		.stream_name = "AFE-PROXY TX",
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(afepcm_tx),
	},
	{
	/*hw:x,9*/
		.name = SDX_DAILINK_NAME(Compress1),
		.stream_name = "COMPR",
		.dynamic = 1,
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
		.async_ops = ASYNC_DPCM_SND_SOC_HW_PARAMS,
#endif /* CONFIG_AUDIO_QGKI */
		.dpcm_playback = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_MULTIMEDIA4,
		SND_SOC_DAILINK_REG(multimedia4),
	},
	{
	/*hw:x,10*/
		.name = "DTMF RX Hostless",
		.stream_name = "DTMF RX Hostless",
		.dynamic = 1,
		.dpcm_playback = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_DTMF_RX,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		SND_SOC_DAILINK_REG(dtmf_rx_hostless),
	},
	{
	/*hw:x,11*/
		.name = "DTMF TX",
		.stream_name = "DTMF TX",
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(dtmf_tx),
	},
	{
	/*hw:x,12*/
		.name = "Primary MI2S TX Hostless",
		.stream_name = "Primary MI2S_TX Hostless Playback",
		.dynamic = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_mi2s_tx_hostless),
	},
	{
	/*hw:x,13*/
		.name = SDX_DAILINK_NAME(LowLatency),
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_MULTIMEDIA5,
		SND_SOC_DAILINK_REG(multimedia5),
	},
	{
	/*hw:x,14*/
		.name = "SDX VoiceMMode2",
		.stream_name = "VoiceMMode2",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* This dainlink has Voice support */
		.ignore_pmdown_time = 1,
		.id = MSM_FRONTEND_DAI_VOICEMMODE2,
		SND_SOC_DAILINK_REG(voicemmode2),
	},
	{
		.name = "VoiceMMode1 HOST RX CAPTURE",
		.stream_name = "VoiceMMode1 HOST RX CAPTURE",
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		SND_SOC_DAILINK_REG(voicemmode1_host_rx_capture),
	},
	{
	/*hw:x,16*/
		.name = "VoiceMMode1 HOST RX PLAYBACK",
		.stream_name = "VoiceMMode1 HOST RX PLAYBACK",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(voicemmode1_host_rx_playback),
	},
	{
	/*hw:x,17*/
		.name = "VoiceMMode1 HOST TX CAPTURE",
		.stream_name = "VoiceMMode1 HOST TX CAPTURE",
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		SND_SOC_DAILINK_REG(voicemmode1_host_tx_capture),
	},
	{
	/*hw:x,18*/
		.name = "VoiceMMode1 HOST TX PLAYBACK",
		.stream_name = "VoiceMMode1 HOST TX PLAYBACK",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(voicemmode1_host_tx_playback),
	},
	{
	/*hw:x,19*/
		.name = "VoiceMMode2 HOST RX CAPTURE",
		.stream_name = "VoiceMMode2 HOST RX CAPTURE",
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		SND_SOC_DAILINK_REG(voicemmode2_host_rx_capture),
	},
	{
	/*hw:x,20*/
		.name = "VoiceMMode2 HOST RX PLAYBACK",
		.stream_name = "VOiceMMode2 HOST RX PLAYBACK",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(voicemmode2_host_rx_playback),
	},
	{
	/*hw:x,21*/
		.name = "VoiceMMode2 HOST TX CAPTURE",
		.stream_name = "VoiceMMode2 HOST TX CAPTURE",
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		SND_SOC_DAILINK_REG(voicemmode2_host_tx_capture),
	},
	{
	/*hw:x,22*/
		.name = "VoiceMMode2 HOST TX PLAYBACK",
		.stream_name = "VOiceMMode2 HOST TX PLAYBACK",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(voicemmode2_host_tx_playback),
	},
	{
	/*hw:x,23*/
		.name = "Secondary MI2S RX Hostless",
		.stream_name = "Secondary MI2S_RX Hostless Playback",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_mi2s_rx_hostless),
	},
	{
	/*hw:x,24*/
		.name = "Secondary MI2S TX Hostless",
		.stream_name = "Secondary MI2S_TX Hostless Playback",
		.dynamic = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_mi2s_tx_hostless),
	},
	{
	/*hw:x,25*/
		.name = "Primary AUXPCM RX Hostless",
		.stream_name = "AUXPCM_HOSTLESS Playback",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(auxpcm_hostless),
	},
	{
	/*hw:x,26*/
		.name = "Primary AUXPCM TX Hostless",
		.stream_name = "AUXPCM_HOSTLESS Capture",
		.dynamic = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(auxpcm_hostless),
	},
	{
	/*hw:x,27*/
		.name = "Secondary AUXPCM RX Hostless",
		.stream_name = "SEC_AUXPCM_HOSTLESS Playback",
		.dynamic = 1,
		.dpcm_playback = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_auxpcm_rx_hostless),
	},
	{
	/*hw:x,28*/
		.name = "Secondary AUXPCM TX Hostless",
		.stream_name = "SEC_AUXPCM_HOSTLESS Capture",
		.dynamic = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_auxpcm_tx_hostless),
	},
	{
	/*hw:x,29*/
		.name = "Primary TDM0 RX Hostless",
		.stream_name = "Primary TDM0 Hostless Playback",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0_hostless),
	},
	{
	/*hw:x,30*/
		.name = "Primary TDM0 TX Hostless",
		.stream_name = "Primary TDM0 Hostless Capture",
		.dynamic = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0_hostless),
	},
	{
	/*hw:x,31*/
		.name = "Secondary TDM RX 0 Hostless",
		.stream_name = "Secondary TDM RX 0 Hostless",
		.dynamic = 1,
		.dpcm_playback = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_tdm_rx_0_hostless),
	},
	{
	/*hw:x,32*/
		.name = "Secondary TDM TX 0 Hostless",
		.stream_name = "Secondary TDM TX 0 Hostless",
		.dynamic = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_tdm_tx_0_hostless),
	},
};

static struct snd_soc_dai_link sdx_common_misc_fe_dai_links[] = {
	{
	/*hw:x,34*/
		.name = SDX_DAILINK_NAME(ASM Loopback),
		.stream_name = "MultiMedia6",
		.dynamic = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		/* this dainlink has playback support */
		.id = MSM_FRONTEND_DAI_MULTIMEDIA6,
		SND_SOC_DAILINK_REG(multimedia6),
	},
};

static int msm_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_interval *period_size = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_PERIOD_SIZE);

	switch (dai_link->id) {
	case MSM_BACKEND_DAI_AFE_PCM_TX:
		channels->min = channels->max = proxy_tx_cfg.channels;
		rate->min = rate->max = proxy_tx_cfg.sample_rate;
		period_size->min = period_size->max = proxy_tx_cfg.period_size;
		break;
	default:
		channels->min = channels->max = 2;
		rate->min = rate->max = SAMPLE_RATE_48KHZ;
		break;
	}

	return 0;
}


static struct snd_soc_dai_link sdx_common_be_dai_links[] = {
	/* Backend AFE DAI Links */
	{
		.name = LPASS_BE_AFE_PCM_RX,
		.stream_name = "AFE Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.id = MSM_BACKEND_DAI_AFE_PCM_RX,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(afe_pcm_rx),
	},
	{
		.name = LPASS_BE_AFE_PCM_TX,
		.stream_name = "AFE Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.id = MSM_BACKEND_DAI_AFE_PCM_TX,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(afe_pcm_tx),
	},
	/* Incall Record Uplink BACK END DAI Link */
	{
		.name = LPASS_BE_INCALL_RECORD_TX,
		.stream_name = "Voice Uplink Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_INCALL_RECORD_TX,
		.be_hw_params_fixup = sdx_be_hw_params_fixup,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(incall_record_tx),
	},
	/* Incall Record Downlink BACK END DAI Link */
	{
		.name = LPASS_BE_INCALL_RECORD_RX,
		.stream_name = "Voice Downlink Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_INCALL_RECORD_RX,
		.be_hw_params_fixup = sdx_be_hw_params_fixup,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(incall_record_rx),
	},
	/* Incall Music BACK END DAI Link */
	{
		.name = LPASS_BE_VOICE_PLAYBACK_TX,
		.stream_name = "Voice Farend Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
		.be_hw_params_fixup = sdx_be_hw_params_fixup,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(voice_playback_tx),
	},
	/* Incall Music BACK END DAI Link */
	{
		.name = LPASS_BE_VOICE_PLAYBACK_DL_TX,
		.stream_name = "Voice Downlink Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_VOICE_PLAYBACK_DL_TX,
		.be_hw_params_fixup = sdx_be_hw_params_fixup,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(voice_downlink_playback_tx),
	},
};

static struct snd_soc_dai_link sdx_mi2s_be_dai_links[] = {
	{
		.name = LPASS_BE_PRI_MI2S_RX,
		.stream_name = "Primary MI2S Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_PRI_MI2S_RX,
		.init = &sdx_mi2s_audrx_init,
		.be_hw_params_fixup = &sdx_mi2s_rx_be_hw_params_fixup,
		.ops = &sdx_mi2s_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tavil_i2s_rx1),
	},
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = "Primary MI2S Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_PRI_MI2S_TX,
		.be_hw_params_fixup = &sdx_mi2s_tx_be_hw_params_fixup,
		.ops = &sdx_mi2s_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tavil_i2s_tx1),
	},
	{
		.name = LPASS_BE_SEC_MI2S_RX,
		.stream_name = "Secondary MI2S Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_SECONDARY_MI2S_RX,
		.be_hw_params_fixup = &sdx_sec_mi2s_rx_be_hw_params_fixup,
		.ops = &sdx_sec_mi2s_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_mi2s_rx),
	},
	{
		.name = LPASS_BE_SEC_MI2S_TX,
		.stream_name = "Secondary MI2S Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_SECONDARY_MI2S_TX,
		.be_hw_params_fixup = &sdx_sec_mi2s_tx_be_hw_params_fixup,
		.ops = &sdx_sec_mi2s_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_mi2s_tx),
	},
};

static struct snd_soc_dai_link sdx_auxpcm_be_dai_links[] = {
	/* Primary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_AUXPCM_RX,
		.stream_name = "AUX PCM Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_AUXPCM_RX,
		.be_hw_params_fixup = sdx_auxpcm_be_params_fixup,
		.ops = &sdx_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
		/* this dainlink has playback support */
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_AUXPCM_TX,
		.stream_name = "AUX PCM Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_AUXPCM_TX,
		.be_hw_params_fixup = sdx_auxpcm_be_params_fixup,
		.ops = &sdx_auxpcm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
	/* Secondary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_SEC_AUXPCM_RX,
		.stream_name = "Sec AUX PCM Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_SEC_AUXPCM_RX,
		.be_hw_params_fixup = sdx_auxpcm_be_params_fixup,
		.ops = &sdx_sec_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_auxpcm_rx),
	},
	{
		.name = LPASS_BE_SEC_AUXPCM_TX,
		.stream_name = "Sec AUX PCM Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_SEC_AUXPCM_TX,
		.be_hw_params_fixup = sdx_auxpcm_be_params_fixup,
		.ops = &sdx_sec_auxpcm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_auxpcm_tx),
	},
};

static struct snd_soc_dai_link sdx_tdm_be_dai_links[] = {
	/* Primary RX TDM Backend DAI Links */
	{
		.name = LPASS_BE_PRI_TDM_RX_0,
		.stream_name = "Primary TDM0 Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_PRI_TDM_RX_0,
		.be_hw_params_fixup = sdx_tdm_be_hw_params_fixup,
		.ops = &sdx_pri_tdm_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0),
	},
	/* Primary TX TDM Backend DAI Links */

	{
		.name = LPASS_BE_PRI_TDM_TX_0,
		.stream_name = "Primary TDM0 Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_PRI_TDM_TX_0,
		.be_hw_params_fixup = sdx_tdm_be_hw_params_fixup,
		.ops = &sdx_pri_tdm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0),
	},

	/* Secondary RX TDM Backend DAI Links */

	{
		.name = LPASS_BE_SEC_TDM_RX_0,
		.stream_name = "Secondary TDM0 Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_SEC_TDM_RX_0,
		.be_hw_params_fixup = sdx_tdm_be_hw_params_fixup,
		.ops = &sdx_sec_tdm_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_tdm_rx_0),
	},
	/* Secondary TX TDM Backend DAI Links */

	{
		.name = LPASS_BE_SEC_TDM_TX_0,
		.stream_name = "Secondary TDM0 Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_SEC_TDM_TX_0,
		.be_hw_params_fixup = sdx_tdm_be_hw_params_fixup,
		.ops = &sdx_sec_tdm_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_tdm_tx_0),
	},

};

static struct snd_soc_dai_link sdx_auto_dai[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_PRI_MI2S_RX,
		.stream_name = "Primary MI2S Playback",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_PRI_MI2S_RX,
		.init  = &sdx_mi2s_audrx_init_auto,
		.be_hw_params_fixup = &sdx_mi2s_rx_be_hw_params_fixup,
		.ops = &sdx_mi2s_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_mi2s_auto_rx),
	},
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = "Primary MI2S Capture",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_PRI_MI2S_TX,
		.be_hw_params_fixup = &sdx_mi2s_tx_be_hw_params_fixup,
		.ops = &sdx_mi2s_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_mi2s_auto_tx),
	},

};

static struct snd_soc_dai_link sdx_tavil_snd_card_dai_links[
			 ARRAY_SIZE(sdx_common_dai_links) +
			 ARRAY_SIZE(sdx_common_misc_fe_dai_links) +
			 ARRAY_SIZE(sdx_common_be_dai_links) +
			 ARRAY_SIZE(sdx_mi2s_be_dai_links) +
			 ARRAY_SIZE(sdx_auxpcm_be_dai_links)+
			 ARRAY_SIZE(sdx_tdm_be_dai_links)];

static struct snd_soc_dai_link sdx_auto_snd_card_dai_links[
			 ARRAY_SIZE(sdx_common_dai_links) +
			 ARRAY_SIZE(sdx_common_misc_fe_dai_links) +
			 ARRAY_SIZE(sdx_common_be_dai_links) +
			 ARRAY_SIZE(sdx_auto_dai) +
			 ARRAY_SIZE(sdx_auxpcm_be_dai_links)+
			 ARRAY_SIZE(sdx_tdm_be_dai_links)];

static int sdx_populate_dai_link_component_of_node(struct snd_soc_card *card)
{
	int i, j, index, ret = 0;
	struct device_node *np = NULL;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;

	if (!cdev) {
		pr_err("%s: Sound card device memory NULL\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].platforms->of_node && dai_link[i].cpus->of_node)
			continue;

		/* populate platform_of_node for snd card dai links */
		if (dai_link[i].platforms->name &&
		    !dai_link[i].platforms->of_node) {
			index = of_property_match_string(cdev->of_node,
						"asoc-platform-names",
						dai_link[i].platforms->name);
			if (index < 0) {
				pr_debug("%s: No match found for platform name: %s\n",
					 __func__, dai_link[i].platforms->name);
				ret = index;
				goto err;
			}

			np = of_parse_phandle(cdev->of_node, "asoc-platform",
					      index);
			if (!np) {
				pr_err("%s: retrieving phandle for platform %s, index %d failed\n",
				       __func__, dai_link[i].platforms->name,
				       index);
				ret = -ENODEV;
				goto err;
			}
			dai_link[i].platforms->of_node = np;
			dai_link[i].platforms->name = NULL;
		}

		/* populate cpu_of_node for snd card dai links */
		if (dai_link[i].cpus->dai_name && !dai_link[i].cpus->of_node) {
			index = of_property_match_string(cdev->of_node,
						"asoc-cpu-names",
						dai_link[i].cpus->dai_name);
			if (index >= 0) {
				np = of_parse_phandle(cdev->of_node,
						      "asoc-cpu",
						      index);
				if (!np) {
					pr_err("%s: retrieving phandle for cpu dai %s failed\n",
					       __func__,
					       dai_link[i].cpus->dai_name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].cpus->of_node = np;
				dai_link[i].cpus->dai_name = NULL;
			}
		}

		/* populate codec_of_node for snd card dai links */
		if (dai_link[i].num_codecs > 0) {
			for (j = 0; j < dai_link[i].num_codecs; j++) {
				if (dai_link[i].codecs[j].of_node ||
						!dai_link[i].codecs[j].name)
					continue;

			index = of_property_match_string(cdev->of_node,
						"asoc-codec-names",
						dai_link[i].codecs[j].name);
			if (index < 0)
				continue;
			np = of_parse_phandle(cdev->of_node, "asoc-codec",
					      index);
			if (!np) {
				pr_err("%s: retrieving phandle for codec %s failed\n",
				       __func__, dai_link[i].codecs[j].name);
				ret = -ENODEV;
				goto err;
			}
			dai_link[i].codecs[j].of_node = np;
			dai_link[i].codecs[j].name = NULL;
			}
		}
	}

err:
	return ret;
}

static const struct of_device_id sdx_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,sdx-asoc-snd-tavil",
	  .data = "tavil_codec"	},
	{ .compatible = "qcom,sdx-asoc-snd-auto",
	  .data = "auto_codec"},
	{},
};

static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_dai_link *dailink;
	const struct of_device_id *match;
	int len_1, len_2, len_3, len_4, len_5;
	int total_links = 0;
	struct snd_soc_card *card = NULL;

	match = of_match_node(sdx_asoc_machine_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "%s: No DT match found for sound card\n",
				__func__);
		return NULL;
	}

	if (!strcmp(match->data, "tavil_codec")) {
		len_1 = ARRAY_SIZE(sdx_common_dai_links);
		len_2 = len_1 + ARRAY_SIZE(sdx_common_misc_fe_dai_links);
		len_3 = len_2 + ARRAY_SIZE(sdx_common_be_dai_links);
		len_4 = len_3 + ARRAY_SIZE(sdx_mi2s_be_dai_links);
		len_5 = len_4 + ARRAY_SIZE(sdx_auxpcm_be_dai_links);
		total_links = len_5 + ARRAY_SIZE(sdx_tdm_be_dai_links);
		memcpy(sdx_tavil_snd_card_dai_links,
			   sdx_common_dai_links,
			   sizeof(sdx_common_dai_links));
		memcpy(sdx_tavil_snd_card_dai_links + len_1,
			   sdx_common_misc_fe_dai_links,
			   sizeof(sdx_common_misc_fe_dai_links));
		memcpy(sdx_tavil_snd_card_dai_links + len_2,
			   sdx_common_be_dai_links,
			   sizeof(sdx_common_be_dai_links));
		memcpy(sdx_tavil_snd_card_dai_links + len_3,
			   sdx_mi2s_be_dai_links,
			   sizeof(sdx_mi2s_be_dai_links));
		memcpy(sdx_tavil_snd_card_dai_links + len_4,
			   sdx_auxpcm_be_dai_links,
			   sizeof(sdx_auxpcm_be_dai_links));
		memcpy(sdx_tavil_snd_card_dai_links + len_5,
			   sdx_tdm_be_dai_links,
			   sizeof(sdx_tdm_be_dai_links));
		card = &snd_soc_card_tavil_sdx;
		dailink = sdx_tavil_snd_card_dai_links;
	} else if (!strcmp(match->data, "auto_codec")) {
		len_1 = ARRAY_SIZE(sdx_common_dai_links);
		len_2 = len_1 + ARRAY_SIZE(sdx_common_misc_fe_dai_links);
		len_3 = len_2 + ARRAY_SIZE(sdx_common_be_dai_links);
		len_4 = len_3 + ARRAY_SIZE(sdx_auto_dai);
		len_5 = len_4 + ARRAY_SIZE(sdx_auxpcm_be_dai_links);
		total_links = len_5 + ARRAY_SIZE(sdx_tdm_be_dai_links);
		memcpy(sdx_auto_snd_card_dai_links,
			   sdx_common_dai_links,
			   sizeof(sdx_common_dai_links));
		memcpy(sdx_auto_snd_card_dai_links + len_1,
			   sdx_common_misc_fe_dai_links,
			   sizeof(sdx_common_misc_fe_dai_links));
		memcpy(sdx_auto_snd_card_dai_links + len_2,
			   sdx_common_be_dai_links,
			   sizeof(sdx_common_be_dai_links));
		memcpy(sdx_auto_snd_card_dai_links + len_3,
			   sdx_auto_dai,
			   sizeof(sdx_auto_dai));
		memcpy(sdx_auto_snd_card_dai_links + len_4,
			   sdx_auxpcm_be_dai_links,
			   sizeof(sdx_auxpcm_be_dai_links));
		memcpy(sdx_auto_snd_card_dai_links + len_5,
			   sdx_tdm_be_dai_links,
			   sizeof(sdx_tdm_be_dai_links));
		card = &snd_soc_card_auto_sdx;
		dailink = sdx_auto_snd_card_dai_links;
	}
	if (card) {
		card->dai_link = dailink;
		card->num_links = total_links;
	}

	return card;
}

static int sdx_init_wsa_dev(struct platform_device *pdev,
			    struct snd_soc_card *card)
{
	u32 wsa_max_devs;
	u32 wsa_dev_cnt;
	struct sdx_machine_data *pdata;
	int ret;

	/* Get maximum WSA device count for this platform */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "qcom,wsa-max-devs", &wsa_max_devs);
	if (ret) {
		dev_dbg(&pdev->dev,
			"%s: wsa-max-devs property missing in DT %s, ret = %d\n",
			__func__, pdev->dev.of_node->full_name, ret);
		return 0;
	}
	if (wsa_max_devs == 0) {
		dev_warn(&pdev->dev,
			 "%s: Max WSA devices is 0 for this target?\n",
			 __func__);
		return 0;
	}

	/* Get count of WSA device phandles for this platform */
	wsa_dev_cnt = of_count_phandle_with_args(pdev->dev.of_node,
						 "qcom,wsa-devs", NULL);
	if (wsa_dev_cnt == -ENOENT) {
		dev_dbg(&pdev->dev, "%s: No wsa device defined in DT.\n",
			__func__);
		return 0;
	} else if (wsa_dev_cnt <= 0) {
		dev_err(&pdev->dev,
			"%s: Error reading wsa device from DT. wsa_dev_cnt = %d\n",
			__func__, wsa_dev_cnt);
		return -EINVAL;
	}

	/*
	 * Expect total phandles count to be NOT less than maximum possible
	 * WSA count. However, if it is less, then assign same value to
	 * max count as well.
	 */
	if (wsa_dev_cnt < wsa_max_devs) {
		dev_dbg(&pdev->dev,
			"%s: wsa_max_devs = %d cannot exceed wsa_dev_cnt = %d\n",
			__func__, wsa_max_devs, wsa_dev_cnt);
		wsa_max_devs = wsa_dev_cnt;
	}

	pdata = snd_soc_card_get_drvdata(card);
	card->num_aux_devs = wsa_max_devs;
	card->num_configs = wsa_max_devs;

	return 0;
}

static int sdx_ssr_enable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	dev_info(dev, "%s: setting snd_card to ONLINE\n", __func__);
	mutex_lock(&snd_card_mutex);
#ifdef CONFIG_AUDIO_QGKI
	snd_soc_card_change_online_state(card, 1);
#endif
	mutex_unlock(&snd_card_mutex);

err:
	return ret;
}

static void sdx_ssr_disable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		return;
	}

	dev_info(dev, "%s: setting snd_card to OFFLINE\n", __func__);
	mutex_lock(&snd_card_mutex);
#ifdef CONFIG_AUDIO_QGKI
	snd_soc_card_change_online_state(card, 0);
#endif
	mutex_unlock(&snd_card_mutex);

	afe_clear_config(AFE_CDC_REGISTERS_CONFIG);
}

static const struct snd_event_ops sdx_ssr_ops = {
	.enable = sdx_ssr_enable,
	.disable = sdx_ssr_disable,
};

static int msm_audio_ssr_compare(struct device *dev, void *data)
{
	struct device_node *node = data;

	dev_dbg(dev, "%s: dev->of_node = 0x%p, node = 0x%p\n",
		__func__, dev->of_node, node);
	return (dev->of_node && dev->of_node == node);
}

static int msm_audio_ssr_register(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct snd_event_clients *ssr_clients = NULL;
	struct device_node *node;
	int ret;
	int i;

	for (i = 0; ; i++) {
		node = of_parse_phandle(np, "qcom,msm_audio_ssr_devs", i);
		if (!node)
			break;
		snd_event_mstr_add_client(&ssr_clients,
					msm_audio_ssr_compare, node);
	}

	ret = snd_event_master_register(dev, &sdx_ssr_ops,
					ssr_clients, NULL);
	if (!ret)
		snd_event_notify(dev, SND_EVENT_UP);

	return ret;
}


static int sdx_asoc_machine_probe(struct platform_device *pdev)
{
	int ret;
	struct sdx_machine_data *pdata;
	struct snd_soc_card *card;
	const struct of_device_id *match;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev,
			"%s No platform supplied from device tree\n", __func__);

		return -EINVAL;
	}

	match = of_match_node(sdx_asoc_machine_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "%s: No DT match found for sound card\n",
				__func__);
		return -EINVAL;
	}
	pdata = devm_kzalloc(&pdev->dev, sizeof(struct sdx_machine_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	card = populate_snd_card_dailinks(&pdev->dev);
	if (!card) {
		dev_err(&pdev->dev, "%s: Card uninitialized\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	if (!strcmp(match->data, "tavil_codec")) {
		ret = of_property_read_u32(pdev->dev.of_node,
					   "qcom,tavil-mclk-clk-freq",
					   &pdata->mclk_freq);
		if (ret) {
			dev_err(&pdev->dev,
				"%s Looking up %s property in node %s failed",
				__func__, "qcom,tavil-mclk-clk-freq",
				pdev->dev.of_node->full_name);

			goto err;
		}
	} else {
		pdata->mclk_freq = SDX_MCLK_CLK_12P288MHZ;
	}
	/* At present only 12.288MHz is supported on SDX. */
	if (q6afe_check_osr_clk_freq(pdata->mclk_freq)) {
		dev_err(&pdev->dev, "%s Unsupported tavil mclk freq %u\n",
			__func__, pdata->mclk_freq);

		ret = -EINVAL;
		goto err;
	}

	pdata->prim_master_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,prim_mi2s_aux_master",
						0);
	pdata->prim_slave_p = of_parse_phandle(pdev->dev.of_node,
					       "qcom,prim_mi2s_aux_slave", 0);
	pdata->sec_master_p = of_parse_phandle(pdev->dev.of_node,
					       "qcom,sec_mi2s_aux_master", 0);
	pdata->sec_slave_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,sec_mi2s_aux_slave", 0);
	mutex_init(&cdc_mclk_mutex);
	atomic_set(&mi2s_ref_count, 0);
	atomic_set(&sec_mi2s_ref_count, 0);
	pdata->prim_clk_usrs = 0;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret)
		goto err;
	if (of_property_read_bool(pdev->dev.of_node, "qcom,audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card,
						"qcom,audio-routing");
		if (ret)
			goto err;
	}
	ret = sdx_populate_dai_link_component_of_node(card);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	/* As Two Codec Probed, set wsa init for tavil codec */
	if (!strcmp(match->data, "tavil_codec")) {
		ret = sdx_init_wsa_dev(pdev, card);
		if (ret)
			goto err;
	}

	ret = snd_soc_register_card(card);
	if (ret == -EPROBE_DEFER) {
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto err;
	}

	pdata->lpaif_pri_muxsel_virt_addr = ioremap(LPAIF_PRI_MODE_MUXSEL, 4);
	if (pdata->lpaif_pri_muxsel_virt_addr == NULL) {
		pr_err("%s Pri muxsel virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err;
	}
	pdata->lpass_mux_spkr_ctl_virt_addr =
				ioremap(LPASS_CSR_GP_IO_MUX_SPKR_CTL, 4);
	if (pdata->lpass_mux_spkr_ctl_virt_addr == NULL) {
		pr_err("%s lpass spkr ctl virt addr is null\n", __func__);

		ret = -EINVAL;
		goto err1;
	}

	pdata->lpaif_sec_muxsel_virt_addr = ioremap(LPAIF_SEC_MODE_MUXSEL, 4);
	if (pdata->lpaif_sec_muxsel_virt_addr == NULL) {
		pr_err("%s Sec muxsel virt addr is null\n", __func__);
		ret = -EINVAL;
		goto err2;
	}

	pdata->lpass_mux_mic_ctl_virt_addr =
				ioremap(LPASS_CSR_GP_IO_MUX_MIC_CTL, 4);
	if (pdata->lpass_mux_mic_ctl_virt_addr == NULL) {
		pr_err("%s lpass_mux_mic_ctl_virt_addr is null\n",
		       __func__);
		ret = -EINVAL;
		goto err3;
	}

	ret = msm_audio_ssr_register(&pdev->dev);
	if (ret)
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",
			__func__, ret);


	return 0;
err3:
	iounmap(pdata->lpaif_sec_muxsel_virt_addr);
err2:
	iounmap(pdata->lpass_mux_spkr_ctl_virt_addr);
err1:
	iounmap(pdata->lpaif_pri_muxsel_virt_addr);
err:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int sdx_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct sdx_machine_data *pdata = snd_soc_card_get_drvdata(card);

	snd_event_master_deregister(&pdev->dev);
	pdata->mclk_freq = 0;
	gpio_free(pdata->hph_en1_gpio);
	gpio_free(pdata->hph_en0_gpio);
	iounmap(pdata->lpaif_pri_muxsel_virt_addr);
	iounmap(pdata->lpass_mux_spkr_ctl_virt_addr);
	iounmap(pdata->lpaif_sec_muxsel_virt_addr);
	iounmap(pdata->lpass_mux_mic_ctl_virt_addr);
	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver sdx_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = sdx_asoc_machine_of_match,
	},
	.probe = sdx_asoc_machine_probe,
	.remove = sdx_asoc_machine_remove,
};

static int __init sdx_soc_platform_init(void)
{
	platform_driver_register(&sdx_asoc_machine_driver);
	return 0;
}

module_init(sdx_soc_platform_init);

static void sdx_soc_platform_exit(void)
{
	platform_driver_unregister(&sdx_asoc_machine_driver);
}

module_exit(sdx_soc_platform_exit);

MODULE_DESCRIPTION("ALSA SoC sdx");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, sdx_asoc_machine_of_match);
