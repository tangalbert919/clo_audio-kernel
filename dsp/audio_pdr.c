// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include "audio_pdr.h"


struct audio_pdr_service {
	void *pdr_handle;
	char service_name[SERVREG_NAME_LENGTH + 1];
	char service_path[SERVREG_NAME_LENGTH + 1];
}
static struct audio_pdr_service audio_pdr_services[AUDIO_PDR_DOMAIN_MAX] = {
	{	/* AUDIO_PDR_DOMAIN_ADSP */
		.service_name = "avs/audio",
		.service_path = "msm/adsp/audio_pd",
	}
};

struct srcu_notifier_head audio_pdr_cb_list;

static int audio_pdr_locator_callback(struct notifier_block *this,
				      unsigned long opcode, void *data)
{
	unsigned long pdr_state = AUDIO_PDR_FRAMEWORK_DOWN;

	if (opcode == LOCATOR_DOWN) {
		pr_debug("%s: Service %s is down!", __func__,
			audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP].
			service_name);
		goto done;
	}

	memcpy(&audio_pdr_services, data,
		sizeof(audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP]));
	if (audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP].total_domains == 1) {
		pr_debug("%s: Service %s, returned total domains %d, ",
			__func__,
			audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP].service_name,
			audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP].
			total_domains);
		pdr_state = AUDIO_PDR_FRAMEWORK_UP;
		goto done;
	} else
		pr_err("%s: Service %s returned invalid total domains %d",
			__func__,
			audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP].service_name,
			audio_pdr_services[AUDIO_PDR_DOMAIN_ADSP].
			total_domains);
done:
	srcu_notifier_call_chain(&audio_pdr_cb_list, pdr_state, NULL);
	return NOTIFY_OK;
}

static struct notifier_block audio_pdr_locator_nb = {
	.notifier_call = audio_pdr_locator_callback,
	.priority = 0,
};

/**
 * audio_pdr_register -
 *        register to PDR framework
 *
 * @nb: notifier block
 *
 * Returns 0 on success or error on failure
 */
int audio_pdr_register(struct notifier_block *nb)
{
	if (nb == NULL) {
		pr_err("%s: Notifier block is NULL\n", __func__);
		return -EINVAL;
	}
	return srcu_notifier_chain_register(&audio_pdr_cb_list, nb);
}
EXPORT_SYMBOL(audio_pdr_register);

/**
 * audio_pdr_deregister -
 *        Deregister from PDR framework
 *
 * @nb: notifier block
 *
 * Returns 0 on success or error on failure
 */
int audio_pdr_deregister(struct notifier_block *nb)
{
	if (nb == NULL) {
		pr_err("%s: Notifier block is NULL\n", __func__);
		return -EINVAL;
	}
	return srcu_notifier_chain_unregister(&audio_pdr_cb_list, nb);
}
EXPORT_SYMBOL(audio_pdr_deregister);

void *audio_pdr_service_register(int domain_id,
				 void (*cb)(int, char *, void *))
{

	if ((domain_id < 0) ||
	    (domain_id >= AUDIO_PDR_DOMAIN_MAX)) {
		pr_err("%s: Invalid service ID %d\n", __func__, domain_id);
		return ERR_PTR(-EINVAL);
	}

	audio_pdr_services[domain_id].pdr_handle = pdr_handle_alloc(cb, NULL);

	return pdr_add_lookup(audio_pdr_services[domain_id].pdr_handle,
						  audio_pdr_services[domain_id].service_name,
						  audio_pdr_services[domain_id].service_path);
}
EXPORT_SYMBOL(audio_pdr_service_register);

int audio_pdr_service_deregister(int domain_id)
{
	if ((domain_id < 0) ||
		(domain_id >= AUDIO_PDR_DOMAIN_MAX)) {
			pr_err("%s: Invalid service ID %d\n", __func__, domain_id);
			return -EINVAL;
	}
	pdr_handle_release(audio_pdr_services[domain_id].pdr_handle);

	return 0;
}
EXPORT_SYMBOL(audio_pdr_service_deregister);

static int __init audio_pdr_late_init(void)
{
	return 0;
}
module_init(audio_pdr_late_init);

static void __exit audio_pdr_late_exit(void)
{
}
module_exit(audio_pdr_late_exit);

MODULE_DESCRIPTION("PDR framework driver");
MODULE_LICENSE("GPL v2");
