// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
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
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <soc/snd_event.h>
#include <dsp/audio_prm.h>
#include <soc/swr-common.h>
#include <soc/soundwire.h>
#include "device_event.h"
#include "asoc/msm-cdc-pinctrl.h"
#include "asoc/wcd-mbhc-v2.h"
#include "codecs/wcd934x/wcd934x-mbhc.h"
#include "codecs/wsa881x.h"
#include "codecs/wcd934x/wcd934x.h"
#include <dt-bindings/sound/audio-codec-port-types.h>
#include "sdx-port-config.h"
#include "msm-audio-defs.h"
#include "msm_common.h"
#include "msm_dailink.h"

#define DRV_NAME "sdx-asoc-snd"
#define __CHIPSET__ "LAHAINA "
#define MSM_DAILINK_NAME(name) (__CHIPSET__#name)

#define WCD9XXX_MBHC_DEF_RLOADS     5
#define WCD9XXX_MBHC_DEF_BUTTONS    8
#define CODEC_EXT_CLK_RATE          9600000
#define DEV_NAME_STR_LEN            32
#define WCD_MBHC_HS_V_MAX           1600

#define MSM_LL_QOS_VALUE	300 /* time in us to ensure LPM doesn't go in C3/C4 */


#define WCN_CDC_SLIM_RX_CH_MAX 2
#define WCN_CDC_SLIM_TX_CH_MAX 2
#define WCN_CDC_SLIM_TX_CH_MAX_LITO 3

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

struct msm_asoc_mach_data {
	struct snd_info_entry *codec_root;
	struct msm_common_pdata *common_pdata;
	int usbc_en2_gpio; /* used by gpio driver API */
	struct device_node *dmic01_gpio_p; /* used by pinctrl API */
	struct device_node *dmic23_gpio_p; /* used by pinctrl API */
	struct device_node *dmic45_gpio_p; /* used by pinctrl API */
	struct device_node *us_euro_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en1_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en0_gpio_p; /* used by pinctrl API */
	u32 wsa_max_devs;
	u16 prim_mi2s_mode;
	struct device_node *prim_master_p;
	void __iomem *lpaif_pri_muxsel_virt_addr;
	void __iomem *lpaif_sec_muxsel_virt_addr;
	void __iomem *lpass_mux_spkr_ctl_virt_addr;
	void __iomem *lpass_mux_mic_ctl_virt_addr;
};

static atomic_t mi2s_ref_count;
static int sdx_mi2s_mode = I2S_PCM_MASTER_MODE;
static bool codec_reg_done;
static struct snd_soc_card snd_soc_card_sdx_msm;

static void *def_wcd_mbhc_cal(void);

static int msm_aux_codec_init(struct snd_soc_pcm_runtime*);
static int msm_int_audrx_init(struct snd_soc_pcm_runtime*);

/*
 * Need to report LINEIN
 * if R/L channel impedance is larger than 5K ohm
 */
static struct wcd_mbhc_config wcd_mbhc_cfg = {
	.read_fw_bin = false,
	.calibration = NULL,
	.detect_extn_cable = true,
	.mono_stero_detection = false,
	.swap_gnd_mic = NULL,
	.hs_ext_micbias = true,
	.key_code[0] = KEY_MEDIA,
	.key_code[1] = KEY_VOICECOMMAND,
	.key_code[2] = KEY_VOLUMEUP,
	.key_code[3] = KEY_VOLUMEDOWN,
	.key_code[4] = 0,
	.key_code[5] = 0,
	.key_code[6] = 0,
	.key_code[7] = 0,
	.linein_th = 5000,
	.moisture_en = false,
	.mbhc_micbias = MIC_BIAS_2,
	.anc_micbias = MIC_BIAS_2,
	.enable_anc_mic_detect = false,
	.moisture_duty_cycle_en = true,
};

static struct snd_soc_ops msm_common_be_ops = {
	.startup = msm_common_snd_startup,
	.shutdown = msm_common_snd_shutdown,
};

static void sdx_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	struct snd_soc_card *card = rtd->card;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (atomic_dec_return(&mi2s_ref_count) == 0) {
		if (pdata->prim_mi2s_mode == 1) {
			ret = msm_cdc_pinctrl_select_sleep_state
						(pdata->prim_master_p);
			if (ret)
				pr_err("%s: failed to set pri gpios to sleep: %d\n",
			       __func__, ret);
		}
	}
}

static int sdx_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pdata->prim_mi2s_mode = sdx_mi2s_mode;
	if (atomic_inc_return(&mi2s_ref_count) == 1) {
		if (pdata->lpaif_pri_muxsel_virt_addr != NULL) {

			ret =audio_prm_set_lpass_core_clk_req( NULL, 1, 1);
			if (ret < 0) {
				dev_err(card->dev,
				"%s:set lpass core clk failed ret: %d\n",
				__func__, ret);
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
				goto done;
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
				goto done;
			}
			ret = snd_soc_dai_set_fmt(codec_dai,
						SND_SOC_DAIFMT_CBS_CFS |
						SND_SOC_DAIFMT_I2S);
			if (ret < 0) {
				dev_err(card->dev,
					"%s Set fmt for codec dai failed\n",
					__func__);
				//Disable mlck here
			}
		} else {
			/*
			 * Disable bit clk in slave mode for QC codec.
			 * Enable only mclk.
			 */
		}
		audio_prm_set_lpass_core_clk_req( NULL, 1, 0);
	}

done:
	if (ret)
		atomic_dec_return(&mi2s_ref_count);
	return ret;
}

static struct snd_soc_ops sdx_mi2s_be_ops = {
	.startup = sdx_mi2s_startup,
	.shutdown = sdx_mi2s_shutdown,
};

static int sdx_mclk_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	pr_info("%s event %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tavil_cdc_mclk_enable(component, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tavil_cdc_mclk_enable(component, 0);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget msm_int_dapm_widgets[] = {

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

static void *def_wcd_mbhc_cal(void)
{
	void *wcd_mbhc_cal;
	struct wcd_mbhc_btn_detect_cfg *btn_cfg;
	u16 *btn_high;

	wcd_mbhc_cal = kzalloc(WCD_MBHC_CAL_SIZE(WCD_MBHC_DEF_BUTTONS,
				WCD9XXX_MBHC_DEF_RLOADS), GFP_KERNEL);
	if (!wcd_mbhc_cal)
		return NULL;

	WCD_MBHC_CAL_PLUG_TYPE_PTR(wcd_mbhc_cal)->v_hs_max = WCD_MBHC_HS_V_MAX;
	WCD_MBHC_CAL_BTN_DET_PTR(wcd_mbhc_cal)->num_btn = WCD_MBHC_DEF_BUTTONS;
	btn_cfg = WCD_MBHC_CAL_BTN_DET_PTR(wcd_mbhc_cal);
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

	return wcd_mbhc_cal;
}

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm_common_be_dai_links[] = {
	/* Proxy Tx BACK END DAI Link */
	{
		.name = LPASS_BE_RT_PROXY_PCM_TX,
		.stream_name = LPASS_BE_RT_PROXY_PCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(proxy_tx),
	},
	/* Proxy Rx BACK END DAI Link */
	{
		.name = LPASS_BE_RT_PROXY_PCM_RX,
		.stream_name = LPASS_BE_RT_PROXY_PCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(proxy_rx),
	},
	{
		.name = LPASS_BE_USB_AUDIO_RX,
		.stream_name = LPASS_BE_USB_AUDIO_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(usb_audio_rx),
	},
	{
		.name = LPASS_BE_USB_AUDIO_TX,
		.stream_name = LPASS_BE_USB_AUDIO_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(usb_audio_tx),
	},
};

static struct snd_soc_dai_link msm_rx_tx_cdc_dma_be_dai_links[] = {
	/* WSA CDC DMA Backend DAI Links */
	{
		.name = LPASS_BE_PRI_MI2S_RX,
		.stream_name = LPASS_BE_PRI_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_mi2s_be_ops,
		SND_SOC_DAILINK_REG(tavil_i2s_rx1),
		.init = &msm_aux_codec_init,
	},
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = LPASS_BE_PRI_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &sdx_mi2s_be_ops,
		SND_SOC_DAILINK_REG(tavil_i2s_tx1),
	},
};

static struct snd_soc_dai_link msm_tdm_dai_links[] = {
	{
		.name = LPASS_BE_PRI_TDM_RX_0,
		.stream_name = LPASS_BE_PRI_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_mi2s_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0),
	},
	{
		.name = LPASS_BE_PRI_TDM_TX_0,
		.stream_name = LPASS_BE_PRI_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &sdx_mi2s_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0),
	},
};

static struct snd_soc_dai_link msm_sdx_dai_links[
			ARRAY_SIZE(msm_rx_tx_cdc_dma_be_dai_links) +
			ARRAY_SIZE(msm_common_be_dai_links) +
			ARRAY_SIZE(msm_tdm_dai_links)];


static int msm_populate_dai_link_component_of_node(
					struct snd_soc_card *card)
{
	int i, j, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np = NULL;
	//int codecs_enabled = 0;
	//struct snd_soc_dai_link_component *codecs_comp = NULL;

	if (!cdev) {
		dev_err(cdev, "%s: Sound card device memory NULL\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].init == NULL)
			dai_link[i].init = &msm_common_dai_link_init;

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
				np = of_parse_phandle(cdev->of_node,
						      "asoc-codec",
						      index);
				if (!np) {
					dev_err(cdev, "%s: retrieving phandle for codec %s failed\n",
						__func__,
						dai_link[i].codecs[j].name);
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

static int msm_audrx_stub_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int msm_snd_stub_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops msm_stub_be_ops = {
	.hw_params = msm_snd_stub_hw_params,
};

struct snd_soc_card snd_soc_card_stub_msm = {
	.name		= "sdx-stub-snd-card",
};

static struct snd_soc_dai_link msm_stub_be_dai_links[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_PRI_AUXPCM_RX,
		.stream_name = LPASS_BE_PRI_AUXPCM_RX,
		.playback_only = 1,
		.init = &msm_audrx_stub_init,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_PRI_AUXPCM_TX,
		.stream_name = LPASS_BE_PRI_AUXPCM_TX,
		.capture_only = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
};

static struct snd_soc_dai_link msm_stub_dai_links[
			 ARRAY_SIZE(msm_stub_be_dai_links)];

static const struct of_device_id sdx_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,sdx-asoc-snd-tavil",
	  .data = "codec"},
	{},
};

static int msm_snd_card_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_component *component = NULL;
	const char *be_dl_name = LPASS_BE_PRI_MI2S_RX;
	struct snd_soc_pcm_runtime *rtd;
	int ret = 0;
	void *mbhc_calibration;

	rtd = snd_soc_get_pcm_runtime(card, be_dl_name);
	if (!rtd) {
		dev_err(card->dev,
			"%s: snd_soc_get_pcm_runtime for %s failed!\n",
			__func__, be_dl_name);
		return -EINVAL;
	}

	component = snd_soc_rtdcom_lookup(rtd, WCD934X_DRV_NAME);
	if (!component) {
		pr_err("%s component is NULL\n", __func__);
		return -EINVAL;
	}

	mbhc_calibration = def_wcd_mbhc_cal();
	if (!mbhc_calibration)
		return -ENOMEM;
	wcd_mbhc_cfg.calibration = mbhc_calibration;
	ret = tavil_mbhc_hs_detect(component, &wcd_mbhc_cfg);
	if (ret) {
		dev_err(component->dev, "%s: mbhc hs detect failed, err:%d\n",
			__func__, ret);
		goto err_hs_detect;
	}
	return 0;

err_hs_detect:
	kfree(mbhc_calibration);
	return ret;
}

static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *dailink = NULL;
	int total_links = 0;
	int rc = 0;
	u32 val = 0;
	const struct of_device_id *match;

	match = of_match_node(sdx_asoc_machine_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "%s: No DT match found for sound card\n",
			__func__);
		return NULL;
	}

	if (!strcmp(match->data, "codec")) {
		card = &snd_soc_card_sdx_msm;

		memcpy(msm_sdx_dai_links + total_links,
		       msm_rx_tx_cdc_dma_be_dai_links,
		       sizeof(msm_rx_tx_cdc_dma_be_dai_links));
		total_links +=
			ARRAY_SIZE(msm_rx_tx_cdc_dma_be_dai_links);

		memcpy(msm_sdx_dai_links + total_links,
		       msm_common_be_dai_links,
		       sizeof(msm_common_be_dai_links));
		total_links +=
			ARRAY_SIZE(msm_common_be_dai_links);

		rc = of_property_read_u32(dev->of_node,
				"qcom,tdm-audio-intf", &val);
		if (!rc && val) {
			memcpy(msm_sdx_dai_links + total_links,
					msm_tdm_dai_links,
					sizeof(msm_tdm_dai_links));
			total_links += ARRAY_SIZE(msm_tdm_dai_links);
		}

		dailink = msm_sdx_dai_links;
	} else if(!strcmp(match->data, "stub_codec")) {
		card = &snd_soc_card_stub_msm;

		memcpy(msm_stub_dai_links,
		       msm_stub_be_dai_links,
		       sizeof(msm_stub_be_dai_links));

		dailink = msm_stub_dai_links;
		total_links = ARRAY_SIZE(msm_stub_be_dai_links);
	}

	if (card) {
		card->dai_link = dailink;
		card->num_links = total_links;
		card->late_probe = msm_snd_card_late_probe;
	}

	return card;
}

static int msm_int_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	u8 spkleft_ports[WSA881X_MAX_SWR_PORTS] = {100, 101, 102, 106};
	u8 spkright_ports[WSA881X_MAX_SWR_PORTS] = {103, 104, 105, 107};
	u8 spkleft_port_types[WSA881X_MAX_SWR_PORTS] = {SPKR_L, SPKR_L_COMP,
						SPKR_L_BOOST, SPKR_L_VI};
	u8 spkright_port_types[WSA881X_MAX_SWR_PORTS] = {SPKR_R, SPKR_R_COMP,
						SPKR_R_BOOST, SPKR_R_VI};
	unsigned int ch_rate[WSA881X_MAX_SWR_PORTS] = {2400, 600, 300, 1200};
	unsigned int ch_mask[WSA881X_MAX_SWR_PORTS] = {0x1, 0xF, 0x3, 0x3};
	struct snd_soc_component *component = NULL;
	struct snd_soc_dapm_context *dapm = NULL;
	struct snd_card *card = NULL;
	struct snd_info_entry *entry = NULL;
	struct msm_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	if (codec_reg_done) {
		return 0;
	}

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

		if (dapm->component) {
			snd_soc_dapm_ignore_suspend(dapm, "SpkrLeft IN");
			snd_soc_dapm_ignore_suspend(dapm, "SpkrLeft SPKR");
		}

		wsa881x_codec_info_create_codec_entry(pdata->codec_root,
			component);
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

		wsa881x_codec_info_create_codec_entry(pdata->codec_root,
			component);
	}


	card = rtd->card->snd_card;
	if (!pdata->codec_root) {
		entry = msm_snd_info_create_subdir(card->module, "codecs",
						 card->proc_root);
		if (!entry) {
			pr_debug("%s: Cannot create codecs module entry\n",
				 __func__);
			ret = 0;
			goto err;
		}
		pdata->codec_root = entry;
	}
	msm_common_dai_link_init(rtd);

err:
	return ret;
}

static int msm_aux_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	struct snd_soc_dapm_context *dapm = NULL;
	struct snd_info_entry *entry;
	struct snd_card *card = NULL;
	struct msm_asoc_mach_data *pdata;

	component = snd_soc_rtdcom_lookup(rtd, WCD934X_DRV_NAME);
	if (!component) {
		pr_err("%s component is NULL\n", __func__);
		return -EINVAL;
	}
	dapm = snd_soc_component_get_dapm(component);
	card = component->card->snd_card;

	snd_soc_dapm_new_controls(dapm, msm_int_dapm_widgets,
				  ARRAY_SIZE(msm_int_dapm_widgets));

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

	pdata = snd_soc_card_get_drvdata(component->card);
	if (!pdata->codec_root) {
		entry = msm_snd_info_create_subdir(card->module, "codecs",
						 card->proc_root);
		if (!entry) {
			dev_dbg(component->dev, "%s: Cannot create codecs module entry\n",
				 __func__);
			 return 0;
		}
		pdata->codec_root = entry;
	}
	tavil_codec_info_create_codec_entry(pdata->codec_root, component);

#if 0
	codec_variant = wcd934x_get_codec_variant(component);
	dev_dbg(component->dev, "%s: variant %d\n", __func__, codec_variant);
	if (codec_variant == WCD9380)
		ret = snd_soc_add_component_controls(component,
					msm_int_wcd9380_snd_controls,
					ARRAY_SIZE(msm_int_wcd9380_snd_controls));
	else if (codec_variant == WCD9385)
		ret = snd_soc_add_component_controls(component,
					msm_int_wcd9385_snd_controls,
					ARRAY_SIZE(msm_int_wcd9385_snd_controls));

	if (ret < 0) {
		dev_err(component->dev, "%s: add codec specific snd controls failed: %d\n",
			__func__, ret);
		return ret;
	}
#endif

	msm_common_dai_link_init(rtd);
	msm_int_audrx_init(rtd);
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

	if (!strcmp(card->name, "sdx-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO \n", __func__);
	}

#if IS_ENABLED(CONFIG_AUDIO_QGKI)
	snd_soc_card_change_online_state(card, 1);
#endif /* CONFIG_AUDIO_QGKI */
	dev_dbg(dev, "%s: setting snd_card to ONLINE\n", __func__);

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

	dev_dbg(dev, "%s: setting snd_card to OFFLINE\n", __func__);
#if IS_ENABLED(CONFIG_AUDIO_QGKI)
	snd_soc_card_change_online_state(card, 0);
#endif /* CONFIG_AUDIO_QGKI */

	if (!strcmp(card->name, "sdx-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO \n", __func__);
	}
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
	struct device_node *node = NULL;
	int ret = 0;
	int i = 0;

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

struct msm_common_pdata *msm_common_get_pdata(struct snd_soc_card *card)
{
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return NULL;

	return pdata->common_pdata;
}

void msm_common_set_pdata(struct snd_soc_card *card,
			  struct msm_common_pdata *common_pdata)
{
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return;

	pdata->common_pdata = common_pdata;
}

static int msm_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = NULL;
	struct msm_asoc_mach_data *pdata = NULL;
	int ret = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "%s: No platform supplied from device tree\n", __func__);
		return -EINVAL;
	}
	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_asoc_mach_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	card = populate_snd_card_dailinks(&pdev->dev);
	if (!card) {
		dev_err(&pdev->dev, "%s: Card uninitialized\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "%s: parse card name failed, err:%d\n",
			__func__, ret);
		goto err;
	}

	ret = snd_soc_of_parse_audio_routing(card, "qcom,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "%s: parse audio routing failed, err:%d\n",
			__func__, ret);
		goto err;
	}

	ret = msm_populate_dai_link_component_of_node(card);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	/* Get maximum WSA device count for this platform */
	ret = of_property_read_u32(pdev->dev.of_node,
		"qcom,wsa-max-devs", &pdata->wsa_max_devs);
	if (ret) {
		dev_info(&pdev->dev,
			"%s: wsa-max-devs property missing in DT %s, ret = %d\n",
			__func__, pdev->dev.of_node->full_name, ret);
		pdata->wsa_max_devs = 0;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		if (codec_reg_done)
			ret = -EINVAL;
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "%s: snd_soc_register_card failed (%d)\n",
			__func__, ret);
		goto err;
	}
	dev_info(&pdev->dev, "%s: Sound card %s registered\n",
		 __func__, card->name);

	pdata->hph_en1_gpio_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,hph-en1-gpio", 0);
	if (!pdata->hph_en1_gpio_p) {
		dev_dbg(&pdev->dev, "%s: property %s not detected in node %s\n",
			__func__, "qcom,hph-en1-gpio",
			pdev->dev.of_node->full_name);
	}

	pdata->hph_en0_gpio_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,hph-en0-gpio", 0);
	if (!pdata->hph_en0_gpio_p) {
		dev_dbg(&pdev->dev, "%s: property %s not detected in node %s\n",
			__func__, "qcom,hph-en0-gpio",
			pdev->dev.of_node->full_name);
	}

	pdata->dmic01_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic01-gpios",
					       0);
	pdata->dmic23_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic23-gpios",
					       0);
	pdata->dmic45_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic45-gpios",
					       0);
	if (pdata->dmic01_gpio_p)
		msm_cdc_pinctrl_set_wakeup_capable(pdata->dmic01_gpio_p, false);
	if (pdata->dmic23_gpio_p)
		msm_cdc_pinctrl_set_wakeup_capable(pdata->dmic23_gpio_p, false);
	if (pdata->dmic45_gpio_p)
		msm_cdc_pinctrl_set_wakeup_capable(pdata->dmic45_gpio_p, false);

	msm_common_snd_init(pdev, card);

	pdata->prim_master_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,prim_mi2s_master",
						0);

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
	atomic_set(&mi2s_ref_count, 0);

	ret = msm_audio_ssr_register(&pdev->dev);
	if (ret)
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",
			__func__, ret);

	return 0;
err1:
	iounmap(pdata->lpass_mux_spkr_ctl_virt_addr);
err:
	iounmap(pdata->lpaif_pri_muxsel_virt_addr);
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int msm_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = NULL;
	struct msm_common_pdata *common_pdata = NULL;

	if (card)
		pdata = snd_soc_card_get_drvdata(card);

	if (pdata)
		common_pdata = pdata->common_pdata;

	msm_common_snd_deinit(common_pdata);
	snd_event_master_deregister(&pdev->dev);
	if (card)
		snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver sdx_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = sdx_asoc_machine_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_asoc_machine_probe,
	.remove = msm_asoc_machine_remove,
};
module_platform_driver(sdx_asoc_machine_driver);

MODULE_SOFTDEP("pre: bt_fm_slim");
MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, sdx_asoc_machine_of_match);
