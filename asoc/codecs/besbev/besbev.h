/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _BESBEV_H
#define _BESBEV_H

#define BESBEV_MAX_SWR_PORTS 1
#ifdef CONFIG_SND_SOC_BESBEV

enum {
	MIC_BIAS_1 = 1,
	MIC_BIAS_2
};

enum {
	MICB_PULLUP_ENABLE,
	MICB_PULLUP_DISABLE,
	MICB_ENABLE,
	MICB_DISABLE,
};

struct besbev_temp_register {
	u8 d1_msb;
	u8 d1_lsb;
	u8 d2_msb;
	u8 d2_lsb;
	u8 dmeas_msb;
	u8 dmeas_lsb;
};

enum wcd_notify_event {
	WCD_EVENT_INVALID,
	/* events for micbias ON and OFF */
	WCD_EVENT_PRE_MICBIAS_2_OFF,
	WCD_EVENT_POST_MICBIAS_2_OFF,
	WCD_EVENT_PRE_MICBIAS_2_ON,
	WCD_EVENT_POST_MICBIAS_2_ON,
	WCD_EVENT_PRE_DAPM_MICBIAS_2_OFF,
	WCD_EVENT_POST_DAPM_MICBIAS_2_OFF,
	WCD_EVENT_PRE_DAPM_MICBIAS_2_ON,
	WCD_EVENT_POST_DAPM_MICBIAS_2_ON,
	/* events for PA ON and OFF */
	WCD_EVENT_OCP_OFF,
	WCD_EVENT_OCP_ON,
	WCD_EVENT_LAST,
};

extern int besbev_info_create_codec_entry(struct snd_info_entry *codec_root,
				    struct snd_soc_component *component);
#else
extern int besbev_info_create_codec_entry(struct snd_info_entry *codec_root,
				    struct snd_soc_component *component)
{
	return 0;
}
#endif /* CONFIG_SND_SOC_BESBEV */

#endif /* _BESBEV_H */
