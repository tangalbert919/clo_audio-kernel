// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include "cc_pktzr.h"

#define CC_THREAD_TIMEOUT (9 * (HZ / 10))

static struct cc_pktzr_pkt_priv *ppriv;

static void cc_pktzr_recv_cb(void *data, size_t size)
{
	struct cc_pktzr_pkt_t *msg_pkt = NULL;
	struct cc_pktzr_pkt_node *pnode = NULL, *tmp = NULL;

	if (!data)
		return;
	msg_pkt = (struct cc_pktzr_pkt_t *)data;

	mutex_lock(&ppriv->cc_pktzr_lock);
	list_for_each_entry_safe(pnode, tmp, &ppriv->cc_list, list) {
		if (pnode && (pnode->token == msg_pkt->pkt_hdr.token)) {
			pnode->resp_payload = kzalloc(size,
							 GFP_KERNEL);
			if (!pnode->resp_payload)
				return;
			memcpy(pnode->resp_payload, msg_pkt->payload,
						size);
			pnode->resp_size = size;
			complete(&ppriv->thread_complete);
			list_del(&pnode->list);
			mutex_unlock(&ppriv->cc_pktzr_lock);
			return;
		}
	}
	mutex_unlock(&ppriv->cc_pktzr_lock);
	return;
}

/**
 * cc_pktzr_send_packet - Function to send packet to IPC
 * @opcode:       Unique int number for every packet
 * @req_payload:  Payload to be send
 * @req_size:     size of requested payload
 * @resp_payload: Pointer to copy response payload from IPC
 *		  Calling function should do memory free
 * @resp_size:    size of response payload
 *
 * Returns 0 on success or error on failure
 */
int cc_pktzr_send_packet(uint32_t opcode, void *req_payload, size_t req_size,
	void **resp_payload, size_t *resp_size)
{
	struct cc_pktzr_pkt_t *msg_pkt = NULL;
	struct cc_pktzr_pkt_node *pnode = NULL;
	ssize_t pkt_size = 0;
	int ret = 0;

	if (!ppriv || !ppriv->pktzr_init_complete) {
		pr_err("%s: packetizer not initialized\n",
							__func__);
		return -EINVAL;
	}
	mutex_lock(&ppriv->cc_pktzr_lock);
	if (++ppriv->token == 0)
		ppriv->token = 1;

	pkt_size = sizeof(struct cc_pktzr_pkt_t) + req_size;
	msg_pkt = kzalloc(pkt_size, GFP_KERNEL);
	if (!msg_pkt) {
		mutex_unlock(&ppriv->cc_pktzr_lock);
		return -ENOMEM;
	}

	memcpy(msg_pkt->payload, req_payload, req_size);
	msg_pkt->payload_size = req_size;

	pnode = kzalloc(sizeof(struct cc_pktzr_pkt_node), GFP_KERNEL);
	if (!pnode) {
		kfree(msg_pkt);
		mutex_unlock(&ppriv->cc_pktzr_lock);
		return -ENOMEM;
	}
	pnode->token = ppriv->token;
	pnode->opcode = opcode;
	msg_pkt->pkt_hdr.opcode = opcode;
	msg_pkt->pkt_hdr.token = ppriv->token;
	msg_pkt->pkt_hdr.dst_port = GPR_SVC_CC_CDC;
	msg_pkt->pkt_hdr.dst_domain_id = GPR_IDS_DOMAIN_ID_CCDSP_V;
	msg_pkt->pkt_hdr.src_port = GPR_SVC_CC_PCDC;
	msg_pkt->pkt_hdr.src_domain_id = GPR_IDS_DOMAIN_ID_APPS_V;
	msg_pkt->payload_size = req_size;

	memcpy(msg_pkt->payload, (char *)req_payload, req_size);
	INIT_LIST_HEAD(&pnode->list);
	list_add_tail(&pnode->list, &ppriv->cc_list);
	mutex_unlock(&ppriv->cc_pktzr_lock);

	ret = audio_cc_ipc_send_pkt(ppriv->handle, msg_pkt, pkt_size);
	if (ret < 0) {
		pr_err("%s Failed to send pkt\n", __func__);
		goto err;
	}

	ret = wait_for_completion_timeout(&ppriv->thread_complete,
					CC_THREAD_TIMEOUT);
	if (!ret) {
		pr_err("%s: Wait for thread timedout\n", __func__);
		ret = -ETIMEDOUT;
		goto err;
	}

	mutex_lock(&ppriv->cc_pktzr_lock);
	*(resp_payload) = kzalloc(pnode->resp_size, GFP_KERNEL);
	if (!(*resp_payload)) {
		ret = -ENOMEM;
		goto err;
	}
	memcpy(*resp_payload, pnode->resp_payload, pnode->resp_size);
	*resp_size = pnode->resp_size;
	mutex_unlock(&ppriv->cc_pktzr_lock);
	kfree(msg_pkt);
	msg_pkt = NULL;
	kfree(pnode);
	pnode = NULL;
	return 0;
err:
	kfree(msg_pkt);
	msg_pkt = NULL;
	mutex_lock(&ppriv->cc_pktzr_lock);
	if (pnode) {
		list_del(&pnode->list);
		kfree(pnode);
		pnode = NULL;
	}
	mutex_unlock(&ppriv->cc_pktzr_lock);
	return ret;
}
EXPORT_SYMBOL(cc_pktzr_send_packet);

/**
 * cc_pktzr_init - Function to register device
 *
 * Returns 0 on success or error on failure
 */
int cc_pktzr_init(void)
{
	int ret = 0;

	if (!ppriv) {
		ppriv = kzalloc((sizeof(struct cc_pktzr_pkt_priv)),
							GFP_KERNEL);
		if (!ppriv)
			return -ENOMEM;

	} else {
		pr_debug("%s: Already initialized\n", __func__);
		goto done;
	}

	ret = audio_cc_ipc_register_device(GPR_SVC_CC_CDC,
		CC_IPC_CH_NAME_NONGPR, cc_pktzr_recv_cb, &(ppriv->handle));
	if (ret < 0) {
		pr_err("%s: Failed to register device\n", __func__);
		goto err;
	}
	init_completion(&ppriv->thread_complete);
	mutex_init(&ppriv->cc_pktzr_lock);
	INIT_LIST_HEAD(&ppriv->cc_list);
	ppriv->pktzr_init_complete = true;
done:
	return 0;
err:
	if (ppriv && ppriv->handle)
		kfree(ppriv->handle);
	if (ppriv)
		kfree(ppriv);
	ppriv = NULL;
	return -EINVAL;
}
EXPORT_SYMBOL(cc_pktzr_init);

/**
 * cc_pktzr_deinit - Function to de-register device
 */
void cc_pktzr_deinit(void)
{
	int ret = 0;
	if (!ppriv)
		return;

	ret = audio_cc_ipc_deregister_device(ppriv->handle, GPR_SVC_CC_CDC);
	reinit_completion(&ppriv->thread_complete);
	mutex_destroy(&ppriv->cc_pktzr_lock);
	if (ppriv->handle)
		kfree(ppriv->handle);
	kfree(ppriv);
	ppriv = NULL;
}
EXPORT_SYMBOL(cc_pktzr_deinit);
