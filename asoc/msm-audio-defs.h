/* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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
#ifndef _MSM_AUDIO_DEFS_H
#define _MSM_AUDIO_DEFS_H

/*
 * These names are used by HAL/AGM to specify the BE. If any changes are
 * made to the string names or the max name length corresponding
 * changes need to be made in the HA/AGM to ensure they still match.
 */

#define LPASS_BE_DISPLAY_PORT_RX "DISPLAY_PORT-RX"

#define LPASS_BE_PRI_AUXPCM_RX "AUXPCM-LPAIF-RX-PRIMARY"
#define LPASS_BE_PRI_AUXPCM_TX "AUXPCM-LPAIF-TX-PRIMARY"
#define LPASS_BE_SEC_AUXPCM_RX "AUXPCM-LPAIF-RX-SECONDARY"
#define LPASS_BE_SEC_AUXPCM_TX "AUXPCM-LPAIF-TX-SECONDARY"
#define LPASS_BE_TERT_AUXPCM_RX "AUXPCM-LPAIF-RX-TERTIARY"
#define LPASS_BE_TERT_AUXPCM_TX "AUXPCM-LPAIF-TX-TERTIARY"
#define LPASS_BE_QUAT_AUXPCM_RX "AUXPCM-LPAIF_RXTX-RX-PRIMARY"
#define LPASS_BE_QUAT_AUXPCM_TX "AUXPCM-LPAIF_RXTX-TX-PRIMARY"
#define LPASS_BE_QUIN_AUXPCM_RX "AUXPCM-LPAIF_VA-RX-PRIMARY"
#define LPASS_BE_QUIN_AUXPCM_TX "AUXPCM-LPAIF_VA-TX-PRIMARY"
#define LPASS_BE_SEN_AUXPCM_RX "AUXPCM-LPAIF_WSA-RX-PRIMARY"
#define LPASS_BE_SEN_AUXPCM_TX "AUXPCM-LPAIF_WSA-TX-PRIMARY"

#define LPASS_BE_PRI_MI2S_RX "MI2S-LPAIF-RX-PRIMARY"
#define LPASS_BE_PRI_MI2S_TX "MI2S-LPAIF-TX-PRIMARY"
#define LPASS_BE_SEC_MI2S_RX "MI2S-LPAIF-RX-SECONDARY"
#define LPASS_BE_SEC_MI2S_TX "MI2S-LPAIF-TX-SECONDARY"
#define LPASS_BE_TERT_MI2S_RX "MI2S-LPAIF-RX-TERTIARY"
#define LPASS_BE_TERT_MI2S_TX "MI2S-LPAIF-TX-TERTIARY"
#define LPASS_BE_QUAT_MI2S_RX "MI2S-LPAIF_RXTX-RX-PRIMARY"
#define LPASS_BE_QUAT_MI2S_TX "MI2S-LPAIF_RXTX-TX-PRIMARY"
#define LPASS_BE_QUIN_MI2S_RX "MI2S-LPAIF_VA-RX-PRIMARY"
#define LPASS_BE_QUIN_MI2S_TX "MI2S-LPAIF_VA-TX-PRIMARY"
#define LPASS_BE_SEN_MI2S_TX "MI2S-LPAIF_WSA-RX-PRIMARY"
#define LPASS_BE_SEN_MI2S_RX "MI2S-LPAIF_WSA-TX-PRIMARY"

#define LPASS_BE_SLIMBUS_0_RX "SLIM-DEV1-RX-0"
#define LPASS_BE_SLIMBUS_0_TX "SLIM-DEV1-TX-0"
#define LPASS_BE_SLIMBUS_1_RX "SLIM-DEV1-RX-1"
#define LPASS_BE_SLIMBUS_1_TX "SLIM-DEV1-TX-1"
#define LPASS_BE_SLIMBUS_2_RX "SLIM-DEV1-RX-2"
#define LPASS_BE_SLIMBUS_2_TX "SLIM-DEV1-TX-2"
#define LPASS_BE_SLIMBUS_3_RX "SLIM-DEV1-RX-3"
#define LPASS_BE_SLIMBUS_3_TX "SLIM-DEV1-TX-3"
#define LPASS_BE_SLIMBUS_4_RX "SLIM-DEV1-RX-4"
#define LPASS_BE_SLIMBUS_4_TX "SLIM-DEV1-TX-4"
#define LPASS_BE_SLIMBUS_5_RX "SLIM-DEV1-RX-5"
#define LPASS_BE_SLIMBUS_5_TX "SLIM-DEV1-TX-5"
#define LPASS_BE_SLIMBUS_6_RX "SLIM-DEV1-RX-6"
#define LPASS_BE_SLIMBUS_6_TX "SLIM-DEV1-TX-6"
#define LPASS_BE_SLIMBUS_7_RX "SLIM-DEV1-RX-7"
#define LPASS_BE_SLIMBUS_7_TX "SLIM-DEV1-TX-7"

#define LPASS_BE_PRI_TDM_RX_0 "TDM-LPAIF-RX-PRIMARY"
#define LPASS_BE_PRI_TDM_TX_0 "TDM-LPAIF-TX-PRIMARY"
#define LPASS_BE_SEC_TDM_RX_0 "TDM-LPAIF-RX-SECONDARY"
#define LPASS_BE_SEC_TDM_TX_0 "TDM-LPAIF-TX-SECONDARY"
#define LPASS_BE_TERT_TDM_RX_0 "TDM-LPAIF-RX-TERTIARY"
#define LPASS_BE_TERT_TDM_TX_0 "TDM-LPAIF-TX-TERTIARY"
#define LPASS_BE_QUAT_TDM_RX_0 "TDM-LPAIF_RXTX-RX-PRIMARY"
#define LPASS_BE_QUAT_TDM_TX_0 "TDM-LPAIF_RXTX-TX-PRIMARY"
#define LPASS_BE_QUIN_TDM_RX_0 "TDM-LPAIF_VA-RX-PRIMARY"
#define LPASS_BE_QUIN_TDM_TX_0 "TDM-LPAIF_VA-TX-PRIMARY"
#define LPASS_BE_SEN_TDM_RX_0 "TDM-LPAIF_WSA-RX-PRIMARY"
#define LPASS_BE_SEN_TDM_TX_0 "TDM-LPAIF_WSA-TX-PRIMARY"

#define LPASS_BE_USB_AUDIO_RX "USB_AUDIO-RX"
#define LPASS_BE_USB_AUDIO_TX "USB_AUDIO-TX"

#define LPASS_BE_WSA_CDC_DMA_RX_0 "CODEC_DMA-LPAIF_WSA-RX-0"
#define LPASS_BE_WSA_CDC_DMA_TX_0 "CODEC_DMA-LPAIF_WSA-TX-0"
#define LPASS_BE_WSA_CDC_DMA_RX_1 "CODEC_DMA-LPAIF_WSA-RX-1"
#define LPASS_BE_WSA_CDC_DMA_TX_1 "CODEC_DMA-LPAIF_WSA-TX-1"
#define LPASS_BE_WSA_CDC_DMA_TX_2 "CODEC_DMA-LPAIF_WSA-TX-2"

#define LPASS_BE_VA_CDC_DMA_TX_0 "CODEC_DMA-LPAIF_VA-TX-0"
#define LPASS_BE_VA_CDC_DMA_TX_1 "CODEC_DMA-LPAIF_VA-TX-1"
#define LPASS_BE_VA_CDC_DMA_TX_2 "CODEC_DMA-LPAIF_VA-TX-2"

#define LPASS_BE_RX_CDC_DMA_RX_0 "CODEC_DMA-LPAIF_RXTX-RX-0"
#define LPASS_BE_RX_CDC_DMA_RX_1 "CODEC_DMA-LPAIF_RXTX-RX-1"
#define LPASS_BE_RX_CDC_DMA_RX_2 "CODEC_DMA-LPAIF_RXTX-RX-2"
#define LPASS_BE_RX_CDC_DMA_RX_3 "CODEC_DMA-LPAIF_RXTX-RX-3"
#define LPASS_BE_RX_CDC_DMA_RX_4 "CODEC_DMA-LPAIF_RXTX-RX-4"
#define LPASS_BE_RX_CDC_DMA_RX_5 "CODEC_DMA-LPAIF_RXTX-RX-5"
#define LPASS_BE_RX_CDC_DMA_RX_6 "CODEC_DMA-LPAIF_RXTX-RX-6"
#define LPASS_BE_RX_CDC_DMA_RX_7 "CODEC_DMA-LPAIF_RXTX-RX-7"
#define LPASS_BE_TX_CDC_DMA_TX_0 "CODEC_DMA-LPAIF_RXTX-TX-0"
#define LPASS_BE_TX_CDC_DMA_TX_1 "CODEC_DMA-LPAIF_RXTX-TX-1"
#define LPASS_BE_TX_CDC_DMA_TX_2 "CODEC_DMA-LPAIF_RXTX-TX-2"
#define LPASS_BE_TX_CDC_DMA_TX_3 "CODEC_DMA-LPAIF_RXTX-TX-3"
#define LPASS_BE_TX_CDC_DMA_TX_4 "CODEC_DMA-LPAIF_RXTX-TX-4"
#define LPASS_BE_TX_CDC_DMA_TX_5 "CODEC_DMA-LPAIF_RXTX-TX-5"

#define LPASS_BE_RT_PROXY_PCM_RX "PCM_RT_PROXY-RX-1"
#define LPASS_BE_RT_PROXY_PCM_TX "PCM_RT_PROXY-TX-1"

#endif /*_MSM_AUDIO_DEFS_H*/
