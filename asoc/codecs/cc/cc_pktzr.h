/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __CC_PKTZR_H_
#define __CC_PKTZR_H_

#include <linux/types.h>
#include <ipc/audio-cc-ipc.h>
#define CC_IPC_CH_NAME_NONGPR		"nongpr_channel"
#define GPR_SVC_CC_CDC			0x0E
#define GPR_IDS_DOMAIN_ID_CCDSP_V	6
#define GPR_SVC_CC_PCDC			0x0F
#define GPR_IDS_DOMAIN_ID_APPS_V	3

struct cc_pktzr_pkt_t {
	struct audio_cc_msg_pkt pkt_hdr;
	uint32_t payload_size;
	uint8_t payload[0];
} __packed;

struct cc_pktzr_pkt_priv {
	struct completion thread_complete;
	struct mutex cc_pktzr_lock;
	uint32_t token;
	struct list_head cc_list;
	bool pktzr_init_complete;
	void *handle;
} __packed;

struct cc_pktzr_pkt_node {
	struct list_head list;
	uint32_t token;
	uint32_t opcode;
	void *resp_payload;
	size_t resp_size;
} __packed;

int cc_pktzr_send_packet(uint32_t opcode, void *req_payload, size_t req_size,
	void **resp_payload, size_t *resp_size);
int cc_pktzr_init(void);
void cc_pktzr_deinit(void);
#endif /* __CC_PKTZR_H_  */
