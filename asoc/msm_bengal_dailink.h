/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

#include <sound/soc.h>

/* FE dai-links */
SND_SOC_DAILINK_DEFS(voicemmode1,
	DAILINK_COMP_ARRAY(COMP_CPU("VoiceMMode1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-voice")));

#ifdef CONFIG_ENABLE_AUDIO_PLAY_REC
SND_SOC_DAILINK_DEFS(multimedia1,
	DAILINK_COMP_ARRAY(COMP_CPU("MultiMedia1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-dsp.0")));

SND_SOC_DAILINK_DEFS(multimedia2,
	DAILINK_COMP_ARRAY(COMP_CPU("MultiMedia2")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-dsp.0")));

SND_SOC_DAILINK_DEFS(msmvoip,
	DAILINK_COMP_ARRAY(COMP_CPU("VoIP")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-voip-dsp")));

SND_SOC_DAILINK_DEFS(multimedia3,
	DAILINK_COMP_ARRAY(COMP_CPU("MultiMedia3")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-dsp.2")));

SND_SOC_DAILINK_DEFS(afepcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.241")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-afe")));

SND_SOC_DAILINK_DEFS(afepcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.240")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-afe")));

SND_SOC_DAILINK_DEFS(auxpcm_hostless,
	DAILINK_COMP_ARRAY(COMP_CPU("AUXPCM_HOSTLESS")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-hostless")));

SND_SOC_DAILINK_DEFS(cdcdma_hostless,
	DAILINK_COMP_ARRAY(COMP_CPU("CDC_DMA_HOSTLESS")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-hostless")));

SND_SOC_DAILINK_DEFS(tx3_cdcdma_hostless,
	DAILINK_COMP_ARRAY(COMP_CPU("TX3_CDC_DMA_HOSTLESS")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-hostless")));

SND_SOC_DAILINK_DEFS(dtmf_rx_hostless,
	DAILINK_COMP_ARRAY(COMP_CPU("DTMF_RX_HOSTLESS")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-dtmf")));

#endif
/* BE dai-links */
SND_SOC_DAILINK_DEFS(afe_pcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.224")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(afe_pcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.225")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(incall_record_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.32772")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(incall_record_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.32771")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(voice_playback_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.32773")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(voice2_playback_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.32770")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(usb_audio_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.28672")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(usb_audio_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.28673")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(pri_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36864")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(pri_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36865")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(sec_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36880")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(sec_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36881")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tert_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36896")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tert_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36897")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(quat_tdm_rx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36912")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(quat_tdm_tx_0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-tdm.36913")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(slimbus_7_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.16398")),
	DAILINK_COMP_ARRAY(COMP_CODEC("btfmslim_slave",
			"btfm_bt_sco_a2dp_slim_rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(slimbus_7_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.16399")),
	DAILINK_COMP_ARRAY(COMP_CODEC("btfmslim_slave",
			"btfm_bt_sco_slim_tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(slimbus_8_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.16401")),
	DAILINK_COMP_ARRAY(COMP_CODEC("btfmslim_slave",
			"btfm_fm_slim_tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(pri_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(pri_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(sec_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(sec_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tert_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.2")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tert_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.2")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(quat_mi2s_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.3")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(quat_mi2s_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-mi2s.3")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.1")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(sec_auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.2")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(sec_auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.2")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tert_auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.3")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tert_auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.3")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(quat_auxpcm_rx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.4")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-rx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(quat_auxpcm_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-auxpcm.4")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(rx_dma_rx0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45104")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "rx_macro_rx1"),
				COMP_CODEC("wsa-codec0", "wsa_rx0"),
				COMP_CODEC("rouleur_codec", "rouleur_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(rx_dma_rx1,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45106")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "rx_macro_rx2"),
               COMP_CODEC("wsa-codec0", "wsa_rx0"),
			   COMP_CODEC("rouleur_codec", "rouleur_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(rx_dma_rx2,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45108")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "rx_macro_rx3"),
			   COMP_CODEC("rouleur_codec", "rouleur_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(rx_dma_rx3,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45110")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "rx_macro_rx4"),
			   COMP_CODEC("rouleur_codec", "rouleur_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tx_dma_tx3,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45111")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "tx_macro_tx1"),
			   COMP_CODEC("rouleur_codec", "rouleur_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(tx_dma_tx4,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45113")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "tx_macro_tx2"),
			   COMP_CODEC("rouleur_codec", "rouleur_cdc")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(va_dma_tx0,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45089")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "va_macro_tx1")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(va_dma_tx1,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45091")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "va_macro_tx2")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(va_dma_tx2,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-cdc-dma-dev.45093")),
	DAILINK_COMP_ARRAY(COMP_CODEC("bolero_codec", "va_macro_tx3")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

SND_SOC_DAILINK_DEFS(afe_loopback_tx,
	DAILINK_COMP_ARRAY(COMP_CPU("msm-dai-q6-dev.24577")),
	DAILINK_COMP_ARRAY(COMP_CODEC("msm-stub-codec.1", "msm-stub-tx")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("msm-pcm-routing")));

