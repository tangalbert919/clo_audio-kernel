/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/rpmsg.h>
#include <ipc/audio-cc-ipc.h>

#define CC_IPC_DRIVER_NAME "audio-cc-ipc"
#define CC_IPC_NAME_MAX_LEN 32
#define CC_IPC_MAX_DEV		2
#define CC_IPC_MAX_CLIENTS	2

#define MINOR_NUMBER_COUNT 1
#define TIMEOUT_MS 2000

/*
 * struct cc_ipc_cdev - holds char device related data
 * @cls			class of the char device
 * @dev			device region allocated
 * @cdev		char dev created
 * @dev_num		count for the device
 */
struct cc_ipc_cdev {
	struct class *cls;
	struct device *dev;
	struct cdev cdev;
	dev_t dev_num;
};

/*
 * struct cc_ipc_priv - for device private data
 * @rsp_queue		    sk_buff queue having response buffers available
 * @rsp_qwait		    wait queue to wait on response buffer queue
 * @slock_rsp			lock for accessing response queue
 * @clients			    clients registering kernel calbacks with the device
 * @slock_client		spin lock to synchronize client operations
 * @ch			        reference to rpmsg endpoint
 * @ch_name				channel associated with device and rpmsg
 * @cdev_name			char device name
 * @cdev				char dev info
 * @mlock			    mutex for file ops and othre general synchronization
 * @dev					info of the device created
 * @add_child_dev_work	work queue to add child device if any
 *
 * This struture holds the private data
 * for each of the devices parsed from the DT
 */
struct cc_ipc_priv {
	/* Respone buffer related */
	struct sk_buff_head rsp_queue;
	wait_queue_head_t rsp_qwait;
	spinlock_t slock_rsp;

	/* Registered Clients for the device */
	struct audio_cc_ipc_client_info clients[CC_IPC_MAX_CLIENTS];
	spinlock_t slock_client;

	/* Channel related */
	struct rpmsg_endpoint *ch;
	char ch_name[CC_IPC_NAME_MAX_LEN];
	char cdev_name[CC_IPC_NAME_MAX_LEN];
	struct cc_ipc_cdev cdev;

	struct mutex mlock;
	struct device *dev;
	struct work_struct add_child_dev_work;
};

struct mutex g_ipriv_lock;
static struct cc_ipc_priv *g_ipriv[CC_IPC_MAX_DEV] = {0};

static void cc_ipc_add_child_dev_func(struct work_struct *work)
{
	int ret = 0;
	struct cc_ipc_priv *ipriv = NULL;

	ipriv = container_of(work, typeof(*ipriv), add_child_dev_work);
	ret = of_platform_populate(ipriv->dev->of_node, NULL, NULL, ipriv->dev);
	if (ret)
		dev_err(ipriv->dev, "%s: failed to add child nodes, ch %s, ret %d\n",
				 __func__, ipriv->ch_name, ret);
}

static ssize_t cc_ipc_fread(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;
	struct sk_buff *skb = NULL;
	int ret = 0, copy = 0;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}

	/*
	 * Wait is signalled from cc_ipc_rpmsg_callback(), or from flush API.
	 * Also use interruptible wait_for_completion API to allow the system
	 * to go in suspend.
	 */
	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	if (skb_queue_empty(&ipriv->rsp_queue)) {
		spin_unlock_irqrestore(&ipriv->slock_rsp, flags);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		/* Wait until we get data or the endpoint goes away */
		if (wait_event_interruptible(ipriv->rsp_qwait,
					!skb_queue_empty(&ipriv->rsp_queue)))
			return -ERESTARTSYS;

		spin_lock_irqsave(&ipriv->slock_rsp, flags);
	}
	skb = skb_dequeue(&ipriv->rsp_queue);
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);
	if (!skb)
		return -EFAULT;

	copy = min_t(size_t, count, skb->len);
	if (copy_to_user(buf, skb->data, copy))
		ret = -EFAULT;
	else
		ret = copy;

	dev_dbg(ipriv->dev, "%s: ch %s, cp %d, ret %d\n",
		__func__, ipriv->ch_name, copy, ret);

	kfree_skb(skb);
	return ret;
}

static unsigned int cc_ipc_fpoll(struct file *file, poll_table *wait)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return POLLERR;
	}

	poll_wait(file, &ipriv->rsp_qwait, wait);
	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	if (!skb_queue_empty(&ipriv->rsp_queue))
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);

	return mask;
}

static int cc_ipc_send_pkt(struct cc_ipc_priv *ipriv, void *pkt, uint32_t pkt_size)
{
	unsigned long flags;
	int ret;

	dev_dbg(ipriv->dev, "%s: ch %s, size %d\n",
		__func__, ipriv->ch_name, pkt_size);

	spin_lock_irqsave(&ipriv->slock_client, flags);
	ret = rpmsg_send(ipriv->ch, pkt, pkt_size);
	spin_unlock_irqrestore(&ipriv->slock_client, flags);
	if (ret < 0)
		dev_err_ratelimited(ipriv->dev, "%s: failed, ch %s, ret %d\n",
				__func__, ipriv->ch_name, ret);
	return ret;
}


static ssize_t cc_ipc_fwrite(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	void *kbuf;
	int ret = 0;
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}

	dev_dbg(ipriv->dev, "%s: count %zd\n", __func__, count);

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	if (mutex_lock_interruptible(&ipriv->mlock)) {
		ret = -ERESTARTSYS;
		goto free_buf;
	}
	ret = cc_ipc_send_pkt(ipriv, kbuf, count);
	if (ret < 0) {
		dev_err_ratelimited(ipriv->dev, "%s: Send Failed, ch %s, ret %d\n",
				__func__, ipriv->ch_name, ret);
		mutex_unlock(&ipriv->mlock);
		goto free_buf;
	}
	mutex_unlock(&ipriv->mlock);

free_buf:
	kfree(kbuf);
	return ret < 0 ? ret : count;
}

static int cc_ipc_fflush(struct file *file, fl_owner_t id)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}

	wake_up_interruptible(&ipriv->rsp_qwait);
	return 0;
}

static int cc_ipc_fopen(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct cc_ipc_cdev *cdev =  container_of(inode->i_cdev, struct cc_ipc_cdev, cdev);
	struct cc_ipc_priv *ipriv =  container_of(cdev, struct cc_ipc_priv, cdev);

	file->private_data = ipriv;

	pr_debug("%s: ch %s\n", __func__,
		 ((struct cc_ipc_priv *)(file->private_data))->ch_name);

	return ret;
}

static int cc_ipc_frelease(struct inode *inode, struct file *file)
{
	struct cc_ipc_priv *ipriv =
			(struct cc_ipc_priv *)file->private_data;
	struct sk_buff *skb = NULL;
	unsigned long flags;

	if (!ipriv) {
		pr_err("%s: Invalid private data\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: ch %s\n", __func__, ipriv->ch_name);

	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	/* Discard all SKBs */
	while (!skb_queue_empty(&ipriv->rsp_queue)) {
		skb = skb_dequeue(&ipriv->rsp_queue);
		kfree_skb(skb);
	}
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);
	wake_up_interruptible(&ipriv->rsp_qwait);
	file->private_data = NULL;
	return 0;
}

static const struct file_operations cc_ipc_fops = {
	.owner =                THIS_MODULE,
	.open =                 cc_ipc_fopen,
	.read =                 cc_ipc_fread,
	.write =                cc_ipc_fwrite,
	.flush =                cc_ipc_fflush,
	.poll =                 cc_ipc_fpoll,
	.release =              cc_ipc_frelease,
};

/* Description	: clients register for the ipc service using this api
 * srvc_id		: service id of the client
 * channel_name	: channel name to be used for the communication
 * cbf	        : callback function for client
 * handle       : private handle of the device registered, this will be filled
 *        by ipc driver and client needs to use it for further communications.
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_register_device(int srvc_id, char *channel_name,
	audio_cc_ipc_cb_t cbf, void **handle)
{
	struct cc_ipc_priv *ipriv = NULL;
	unsigned long flags;
	int i = 0, ret = 0;

	mutex_lock(&g_ipriv_lock);
	for (i = 0; i < CC_IPC_MAX_DEV; i++) {
		if ((g_ipriv[i] != NULL) &&
			!strncmp(channel_name, g_ipriv[i]->ch_name, CC_IPC_NAME_MAX_LEN)) {
			ipriv = g_ipriv[i];
			break;
		}
	}
	mutex_unlock(&g_ipriv_lock);

	if (ipriv == NULL) {
		ret = -ENODEV;
		goto done;
	}

	ret = -ENOSPC;
	spin_lock_irqsave(&ipriv->slock_client, flags);
	for (i = 0; i < CC_IPC_MAX_CLIENTS; i++) {
		if (ipriv->clients[i].cbf == NULL) {
			ipriv->clients[i].cbf = cbf;
			ipriv->clients[i].srvc_id = srvc_id;
			*handle = (void *)ipriv;
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&ipriv->slock_client, flags);

done:
	pr_debug("%s: ch %s, ret %d\n", __func__, channel_name, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(audio_cc_ipc_register_device);

/* Description	: clients deregister for the ipc service using this api
 * handle       : private handle of the device returned during register
 * srvc_id		: service id of the client
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_deregister_device(void *handle, int srvc_id)
{
	struct cc_ipc_priv *ipriv = (struct cc_ipc_priv *)handle;
	unsigned long flags;
	int ret = -ENOENT;
	int i = 0;

	if (!ipriv)
		return ret;

	spin_lock_irqsave(&ipriv->slock_client, flags);
	for (i = 0; i < CC_IPC_MAX_CLIENTS; i++) {
		if (ipriv->clients[i].srvc_id == srvc_id) {
			ipriv->clients[i].cbf = NULL;
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&ipriv->slock_client, flags);
	pr_debug("%s: ch %s, ret %d\n", __func__, ipriv->ch_name, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(audio_cc_ipc_deregister_device);

/* Description 	: clients can use this function to send their packets
 * handle       : private handle of the device returned during register
 * pkt			: packet to be send
 * pkt_size		: size of the packet
 * return 		: returns 0 on success, failure otherwise
 */
int audio_cc_ipc_send_pkt(void *handle, void *pkt, uint32_t pkt_size)
{
	return cc_ipc_send_pkt(handle, pkt, pkt_size);
}
EXPORT_SYMBOL_GPL(audio_cc_ipc_send_pkt);

static int cc_ipc_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				int len, void *priv, u32 addr__unused)
{
	struct cc_ipc_priv *ipriv = dev_get_drvdata(&rpdev->dev);
	struct audio_cc_msg_pkt *msg_pkt = NULL;
	unsigned long flags;
	int ret = 0, i = 0;
	struct sk_buff *skb = NULL;

	if (!ipriv || !data) {
		pr_err("%s: Invalid ipriv or data\n", __func__);
		return -EINVAL;
	}

	dev_dbg_ratelimited(ipriv->dev, "%s: received buff: ch %s\n",
			__func__, ipriv->ch_name);
	msg_pkt = (struct audio_cc_msg_pkt *) data;

	/* call client callback if matches with clients srv_id registered with
	 * device, else call cache read buff and call complete().
	 */
	spin_lock_irqsave(&ipriv->slock_client, flags);
	for (i = 0; i < CC_IPC_MAX_CLIENTS; i++) {
		if (ipriv->clients[i].cbf != NULL &&
			ipriv->clients[i].srvc_id == msg_pkt->dst_port) {
			spin_unlock_irqrestore(&ipriv->slock_client, flags);
			ipriv->clients[i].cbf(data, len);
			return ret;
		}
	}
	spin_unlock_irqrestore(&ipriv->slock_client, flags);

	if (len <= 0 || len > AUDIO_CC_IPC_READ_SIZE_MAX) {
		dev_info_ratelimited(ipriv->dev,
				"%s: ignoring read with len(%d) ch %s\n",
				__func__, len, ipriv->ch_name);
		return -EINVAL;
	}

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb) {
		dev_err_ratelimited(ipriv->dev,
				"%s: failed to alloc skb ch %s\n", __func__, ipriv->ch_name);
		return -ENOMEM;
	}

	skb_put_data(skb, data, len);
	spin_lock_irqsave(&ipriv->slock_rsp, flags);
	skb_queue_tail(&ipriv->rsp_queue, skb);
	spin_unlock_irqrestore(&ipriv->slock_rsp, flags);

	/* wake up any blocking processes, waiting for new data */
	wake_up_interruptible(&ipriv->rsp_qwait);
	return ret;
}

static int cc_ipc_create_char_device(struct cc_ipc_priv *ipriv)
{
	int ret = 0;
	struct device *dev = ipriv->dev;
	struct cc_ipc_cdev *cdev = NULL;
	const char *cdev_name = NULL;

	ret = of_property_read_string(dev->of_node, "cdev_name", &cdev_name);
	if (ret) {
		dev_err(dev, "cdev name not not specified ch %s\n", ipriv->ch_name);
		goto done;
	}
	strlcpy(ipriv->cdev_name, cdev_name, CC_IPC_NAME_MAX_LEN);

	cdev = &ipriv->cdev;
	ret = alloc_chrdev_region(&cdev->dev_num, 0, MINOR_NUMBER_COUNT,
							ipriv->cdev_name);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to alloc char dev, ch %s, err %d\n",
			__func__, ipriv->ch_name, ret);
		goto done;
	}

	cdev->cls = class_create(THIS_MODULE, ipriv->cdev_name);
	if (IS_ERR(cdev->cls)) {
		ret = PTR_ERR(cdev->cls);
		dev_err(dev, "%s: Failed to create class, ch %s, err %d\n",
			__func__, ipriv->ch_name, ret);
		goto err_class;
	}

	cdev->dev = device_create(cdev->cls, NULL, cdev->dev_num,
				  NULL, ipriv->cdev_name);
	if (IS_ERR(cdev->dev)) {
		ret = PTR_ERR(cdev->dev);
		dev_err(dev, "%s: Failed to create device, ch %s, err %d\n",
			__func__, ipriv->ch_name, ret);
		goto err_dev_create;
	}

	cdev_init(&cdev->cdev, &cc_ipc_fops);
	ret = cdev_add(&cdev->cdev, cdev->dev_num, MINOR_NUMBER_COUNT);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to register char dev, ch %s, err %d\n",
			__func__, ipriv->ch_name, ret);
		goto err_cdev_add;
	}
	dev_dbg(dev, "%s: registered char dev, ch %s, cdev_name %s\n",
		__func__, ipriv->ch_name, ipriv->cdev_name);

	goto done;

err_cdev_add:
	device_destroy(cdev->cls, cdev->dev_num);

err_dev_create:
	class_destroy(cdev->cls);

err_class:
	unregister_chrdev_region(0, MINOR_NUMBER_COUNT);

done:
	return ret;
}

static int cc_ipc_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;
	struct device *dev = &rpdev->dev;
	const char *ch_name = NULL;
	struct cc_ipc_priv *ipriv = NULL;
	int itr = 0;

	mutex_lock(&g_ipriv_lock);
	for (itr = 0; itr < CC_IPC_MAX_DEV; itr ++) {
		if (g_ipriv[itr] == NULL)
			break;
	}

	if (itr >= CC_IPC_MAX_DEV) {
		dev_err(dev, "Maximum device count reached \n");
		mutex_unlock(&g_ipriv_lock);
		return -ENODEV;
	}

	ipriv = devm_kzalloc(dev, sizeof(*ipriv), GFP_KERNEL);
	if (!ipriv) {
		mutex_unlock(&g_ipriv_lock);
		return -ENOMEM;
	}

	g_ipriv[itr] = ipriv;

	mutex_unlock(&g_ipriv_lock);

	mutex_init(&ipriv->mlock);
	spin_lock_init(&ipriv->slock_rsp);
	skb_queue_head_init(&ipriv->rsp_queue);
	init_waitqueue_head(&ipriv->rsp_qwait);
	spin_lock_init(&ipriv->slock_client);
	ipriv->ch = rpdev->ept;
	ipriv->dev = dev;

	ret = of_property_read_string(dev->of_node,
				"qcom,glink-channels", &ch_name);
	if (ret) {
		dev_err(dev, "cannot obtain channel name from dt node\n");
		goto cleanup;
	}
	strlcpy(ipriv->ch_name, ch_name, CC_IPC_NAME_MAX_LEN);

	ret = cc_ipc_create_char_device(ipriv);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to create char device\n", __func__);
		goto cleanup;
	}

	dev_set_drvdata(dev, ipriv);

	INIT_WORK(&ipriv->add_child_dev_work, cc_ipc_add_child_dev_func);
	schedule_work(&ipriv->add_child_dev_work);

	return 0;

cleanup:
	mutex_lock(&g_ipriv_lock);
	g_ipriv[itr] = NULL;
	mutex_unlock(&g_ipriv_lock);
	mutex_destroy(&ipriv->mlock);

	return ret;
}

static void cc_ipc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct cc_ipc_priv *ipriv = dev_get_drvdata(dev);
	int i = 0;

	of_platform_depopulate(dev);

	if (ipriv) 	{
		cdev_del(&ipriv->cdev.cdev);
		device_destroy(ipriv->cdev.cls, ipriv->cdev.dev_num);
		class_destroy(ipriv->cdev.cls);
		unregister_chrdev_region(MAJOR(ipriv->cdev.dev_num),
				MINOR_NUMBER_COUNT);

		mutex_destroy(&ipriv->mlock);
		dev_dbg(dev, "%s: unregistered char dev, ch %s, cdev_name %s\n",
			__func__, ipriv->ch_name, ipriv->cdev_name);

		mutex_lock(&g_ipriv_lock);
		for (i = 0; i < CC_IPC_MAX_DEV; i++) {
			if (g_ipriv[i] == ipriv)	{
				g_ipriv[i] = NULL;
				break;
			}
		}
		mutex_unlock(&g_ipriv_lock);
	}
}

static const struct of_device_id cc_ipc_of_match[] = {
	{.compatible = "qcom,audio_cc_ipc"},
	{ }
};
MODULE_DEVICE_TABLE(of, cc_ipc_of_match);

static struct rpmsg_driver cc_ipc_rpmsg_driver = {
	.probe = cc_ipc_rpmsg_probe,
	.remove = cc_ipc_rpmsg_remove,
	.callback = cc_ipc_rpmsg_callback,
	.drv = {
		.name = "audio_cc_ipc",
		.of_match_table = cc_ipc_of_match,
	},
};

static int __init audio_cc_ipc_init(void)
{
	int ret = 0;

	ret = register_rpmsg_driver(&cc_ipc_rpmsg_driver);
	if (ret < 0)
		pr_err("audio_cc_ipc: failed to register rpmsg driver\n");
	mutex_init(&g_ipriv_lock);

	return ret;
}

static void __exit audio_cc_ipc_exit(void)
{
	mutex_destroy(&g_ipriv_lock);
	unregister_rpmsg_driver(&cc_ipc_rpmsg_driver);
}

module_init(audio_cc_ipc_init);
module_exit(audio_cc_ipc_exit);
MODULE_DESCRIPTION("SoC Audio CC_IPC Driver");
MODULE_LICENSE("GPL v2");
