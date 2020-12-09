// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include "cc_defs.h"

#define CC_MAX_SYSFS_STRLEN 64
enum {
	CC_REG_REGION_INFO_ATTR,
	CC_REG_MAX_ADDR_ATTR,
	CC_REG_ADDR_ATTR,
	CC_REG_COUNT_ATTR,
	CC_REG_VALUE_ATTR,
	CC_REG_ATTR_MAX,
};

struct cc_id_name_map {
	uint32_t id;
	char name[CC_MAX_STRLEN];
};

struct register_region {
	struct cc_id_name_map *region_id;
	uint32_t max_addr;
	uint32_t addr;
	uint32_t count;
	struct device_attribute reg_region_attr[CC_REG_ATTR_MAX];
	struct attribute *attr[CC_REG_ATTR_MAX];
	struct attribute_group attr_grp;
};

struct cc_start_uc_header {
	uint32_t uc_id;
	uint32_t sample_rate;
	uint32_t bit_width;
	uint32_t channels;
} __packed;

struct cc_interface {
	struct cc_start_uc_header uc_header;
	size_t payload_size;
	size_t action_size;
	void *payload;
	struct list_head path_list;
	struct list_head action_value_list;
};

struct cc_path_list {
	struct list_head list;
	uint32_t src_id;
	uint32_t sink_id;
};

struct cc_action_value_list {
	struct list_head list;
	uint32_t action_id;
	uint32_t action_type;
	uint32_t num_params;
	union {
		uint32_t enumerated;
		uint32_t *uint_array;
		int32_t *int_array;
		uint8_t *char_array;
	} param;
};

struct cc_action {
	uint32_t action_type;
	struct cc_id_name_map *act_id;
	struct snd_kcontrol_new kcontrol;
	char (*enum_str)[CC_MAX_STRLEN];
	char **texts;
	struct soc_enum action_enum;
	struct soc_multi_mixer_control mixer;
	uint32_t num_params;
	uint32_t def;
	uint32_t param_min;
	uint32_t param_max;
	struct list_head *list;
};

struct cc_element {
	uint32_t elem_type;
	struct snd_kcontrol_new *kcontrol;
	char **texts;
	char s_name[CC_MAX_STREAM_STRLEN];
	struct soc_enum elem_enum;
	struct soc_mixer_control mixer;
	struct snd_soc_dapm_widget widget;
	struct cc_id_name_map *elem_id;
	uint32_t num_src;
	struct cc_id_name_map *src_id;
	struct list_head *list;
};

struct cc_codec_priv {
	uint32_t num_id;
	struct cc_id_name_map *id_name;
	uint32_t num_reg_region;
	struct register_region *reg_region;
	uint32_t num_elem;
	struct cc_element *elems;
	uint32_t num_action;
	struct cc_action *actions;
	uint32_t num_routes;
	struct snd_soc_dapm_route *audio_map;
	uint32_t num_iface;
	struct cc_interface *iface;
	struct snd_soc_dai_driver *dai;
	struct snd_soc_component_driver cc_comp;
};

struct cc_sysfs_attr_mapping {
	umode_t mode;
	char *name;
	ssize_t (*show)(struct device *dev, struct device_attribute *attr,
			char *buf);
	ssize_t (*store)(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);
};

typedef int (cc_add_elem_func)(struct cc_element *, int);

static int cc_parse_name(char *dst, uint8_t *ptr, uint32_t *parsed_len)
{
	uint32_t i = 0;

	for (i = 0; i < CC_MAX_STRLEN; i++) {
		dst[i] = ptr[i];
		if (ptr[i] == 0)
			break;
	}

	if (ptr[i] != 0)
		return -EINVAL;

	*parsed_len = ((i + 1) * sizeof(char));

	return 0;
}

static int cc_parse_id_name(struct cc_codec_priv *cc_priv,
				uint8_t *ptr, uint32_t *parsed_len)
{
	uint32_t tmp_len = 0;
	int ret = 0;

	cc_priv->id_name[cc_priv->num_id].id = *(uint32_t *)ptr;
	ptr += sizeof(uint32_t);

	ret = cc_parse_name(cc_priv->id_name[cc_priv->num_id].name,
				ptr, &tmp_len);
	if (ret) {
		pr_err("%s: Invalid name of id: 0x%x\n",
			__func__, cc_priv->id_name[cc_priv->num_id].id);
		return ret;
	}

	cc_priv->num_id++;
	*parsed_len = (sizeof(uint32_t) + tmp_len);

	return 0;
}

static int cc_find_region_id_from_attr(struct cc_codec_priv *cc_priv,
					struct device_attribute *attr)
{
	int i = 0;
	int j = 0;

	for (i = 0; i < cc_priv->num_reg_region; i++) {
		for (j = 0; j < CC_REG_ATTR_MAX; j++) {
			if (&attr->attr == cc_priv->reg_region[i].attr[j])
				return i;
		}
	}

	return -EINVAL;
}

static ssize_t cc_reg_region_info_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int id = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(dev);

	id = cc_find_region_id_from_attr(cc_priv, attr);
	if (id < 0)
		return id;

	return snprintf(buf, CC_MAX_SYSFS_STRLEN, "%d:%s\n",
			cc_priv->reg_region[id].region_id->id,
			cc_priv->reg_region[id].region_id->name);
}

static ssize_t cc_reg_max_addr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int id = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(dev);

	id = cc_find_region_id_from_attr(cc_priv, attr);
	if (id < 0)
		return id;

	return snprintf(buf, CC_MAX_SYSFS_STRLEN, "0x%04x\n",
			cc_priv->reg_region[id].max_addr);
}

static ssize_t cc_reg_addr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int id = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(dev);

	id = cc_find_region_id_from_attr(cc_priv, attr);
	if (id < 0)
		return id;

	return snprintf(buf, CC_MAX_SYSFS_STRLEN, "0x%04x\n",
			cc_priv->reg_region[id].addr);
}

static ssize_t cc_reg_addr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int id = 0;
	uint32_t addr = 0;
	int ret = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(dev);

	id = cc_find_region_id_from_attr(cc_priv, attr);
	if (id < 0)
		return id;

	ret = kstrtouint(buf, 16, &addr);
	if (ret < 0)
		return ret;

	if (addr > cc_priv->reg_region[id].max_addr) {
		dev_err(dev, "Invalid addr %x, max addr: %x\n",
			addr, cc_priv->reg_region[id].max_addr);
		return -EINVAL;
	}

	cc_priv->reg_region[id].addr = addr;

	return count;
}

static ssize_t cc_reg_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int id = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(dev);

	id = cc_find_region_id_from_attr(cc_priv, attr);
	if (id < 0)
		return id;

	return snprintf(buf, CC_MAX_SYSFS_STRLEN, "0x%04x\n",
			cc_priv->reg_region[id].count);
}

static ssize_t cc_reg_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int id = 0;
	uint32_t cnt = 0;
	int ret = 0;
	uint32_t addr = 0;
	uint32_t max_addr = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(dev);

	id = cc_find_region_id_from_attr(cc_priv, attr);
	if (id < 0)
		return id;

	ret = kstrtouint(buf, 10, &cnt);
	if (ret < 0)
		return ret;

	addr = cc_priv->reg_region[id].addr;
	max_addr = cc_priv->reg_region[id].max_addr;
	if (cnt > (max_addr - addr + 1)) {
		dev_err(dev, "Count should be <= %d\n", (max_addr - addr + 1));
		return -EINVAL;
	}

	cc_priv->reg_region[id].count = cnt;

	return count;
}

static int cc_send_packet(uint32_t opcode, void *req_payload, size_t req_size,
	void **resp_payload, size_t *resp_size)
{
	/* TODO: integrate with ipc */
	return 0;
}

static int cc_send_pkt_generic_resp(uint32_t opcode, void *payload, size_t size)
{
	int rc = 0;
	struct cc_resp_generic *resp = NULL;
	size_t resp_size = 0;

	rc = cc_send_packet(opcode, payload, size, (void **)&resp, &resp_size);
	if (rc)
		return rc;

	if (resp && resp_size && (resp->opcode == opcode)) {
		if (resp->response == RESPONSE_SUCESS)
			return 0;
		rc = -(resp->response);
		pr_err("%s: opcode: %d resp: %d\n",
			__func__, opcode, resp->response);
	}

	return rc;
}

static ssize_t cc_reg_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	/* TODO: Create payload */
	return CC_MAX_SYSFS_STRLEN;
}

static ssize_t cc_reg_value_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	/* TODO: Create payload */
	return count;
}

static struct cc_sysfs_attr_mapping cc_sysfs_attr_map[] = {
	[CC_REG_REGION_INFO_ATTR] = {
		0444, "info", cc_reg_region_info_show, NULL,
	},
	[CC_REG_MAX_ADDR_ATTR] = {
		0444, "max_addr", cc_reg_max_addr_show, NULL,
	},
	[CC_REG_ADDR_ATTR] = {
		0644, "addr", cc_reg_addr_show, cc_reg_addr_store,
	},
	[CC_REG_COUNT_ATTR] = {
		0644, "count", cc_reg_count_show, cc_reg_count_store,
	},
	[CC_REG_VALUE_ATTR] = {
		0644, "value", cc_reg_value_show, cc_reg_value_store,
	},
};

static void cc_reg_region_init(struct register_region *reg_region,
				char *name)
{
	int i = 0;

	for (i = 0; i < CC_REG_ATTR_MAX; i++) {
		reg_region->reg_region_attr[i].attr.name =
			cc_sysfs_attr_map[i].name;
		reg_region->reg_region_attr[i].attr.mode =
			cc_sysfs_attr_map[i].mode;
		reg_region->reg_region_attr[i].show =
			cc_sysfs_attr_map[i].show;
		reg_region->reg_region_attr[i].store =
			cc_sysfs_attr_map[i].store;
	}
}

static int cc_parse_reg_region(struct cc_codec_priv *cc_priv,
		struct device *dev, uint8_t *payload,
		uint32_t num_dev, uint32_t *parsed_len)
{
	int ret = 0;
	int i = 0;
	uint32_t tmp_len = 0;

	if (!cc_priv || !dev || !payload || !parsed_len)
		return -EINVAL;

	if (num_dev > CC_MAX_REG_REGION) {
		dev_err(dev, "num register regions > %d\n",
			CC_MAX_REG_REGION);
		return -EINVAL;
	}

	if (num_dev == 0)
		return 0;

	cc_priv->reg_region = kzalloc(
			num_dev * sizeof(struct register_region), GFP_KERNEL);
	if (!cc_priv->reg_region)
		return -ENOMEM;
	cc_priv->num_reg_region = num_dev;

	*parsed_len = 0;
	for (i = 0; i < num_dev; i++) {
		ret = cc_parse_id_name(cc_priv, payload, &tmp_len);
		if (ret)
			return ret;
		payload += tmp_len;
		*parsed_len += tmp_len;

		cc_priv->reg_region[i].region_id =
			&cc_priv->id_name[cc_priv->num_id - 1];

		cc_priv->reg_region[i].max_addr = *((uint32_t *)payload);
		payload += tmp_len;
		*parsed_len += tmp_len;

		cc_reg_region_init(&cc_priv->reg_region[i],
				cc_priv->reg_region[i].region_id->name);
	}

	return 0;
}

static int cc_parse_elem_src(struct cc_codec_priv *cc_priv,
	struct cc_element *elem, uint8_t *payload, uint32_t *parsed_len)
{
	int ret = 0;
	int i = 0;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;

	if (!cc_priv || !elem || !payload || !parsed_len)
		return -EINVAL;

	if (elem->num_src == 0 || elem->num_src > CC_MAX_NUM_SRC_PER_ELEM) {
		pr_err("%s: invalid number of source for element: %d\n",
			__func__, elem->num_src);
		return -EINVAL;
	}

	*parsed_len = 0;
	elem->src_id = &cc_priv->id_name[cc_priv->num_id];
	for (i = 0; i < elem->num_src; i++) {
		ret = cc_parse_id_name(cc_priv, tmp, &tmp_len);
		if (ret)
			break;
		*parsed_len += tmp_len;
		tmp += tmp_len;
	}

	return ret;
}

static int cc_elem_ctl_mux_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_enum *elem_enum = (struct soc_enum *)kcontrol->private_value;
	struct cc_element *elem = &cc_priv->elems[elem_enum->shift_l];
	struct cc_interface *iface = container_of(elem->list,
					struct cc_interface, path_list);
	uint32_t val = ucontrol->value.enumerated.item[0];
	struct cc_path_list *path_list = NULL;
	struct list_head *node = NULL, *next = NULL;
	int found = 0;

	if (val > elem->num_src)
		return -EINVAL;

	list_for_each_safe(node, next, elem->list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (path_list->sink_id == elem->elem_id->id) {
			found = 1;
			break;
		}
	}

	if (!found) {
		path_list = kzalloc(sizeof(struct cc_path_list),
					GFP_KERNEL);
		if (!path_list)
			return -ENOMEM;

		path_list->src_id = elem->src_id[val].id;
		path_list->sink_id = elem->elem_id->id;
		list_add_tail(&path_list->list, elem->list);

		iface->payload += (sizeof(path_list->src_id) +
					sizeof(path_list->sink_id));
	}

	path_list->src_id = elem->src_id[val].id;
	return 0;
}

static int cc_elem_ctl_mux_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_enum *elem_enum = (struct soc_enum *)kcontrol->private_value;
	struct cc_element *elem = &cc_priv->elems[elem_enum->shift_l];
	struct cc_path_list *path_list = NULL;
	struct list_head *node = NULL, *next = NULL;
	int found = 0;
	int i = 0;

	elem = &cc_priv->elems[elem_enum->shift_l];

	list_for_each_safe(node, next, elem->list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (path_list->sink_id == elem->elem_id->id) {
			found = 1;
			break;
		}
	}

	if (!found) {
		ucontrol->value.enumerated.item[0] = 0;
		return 0;
	}

	for (i = 0; i < elem->num_src; i++) {
		if (path_list->src_id == elem->src_id[i].id)
			break;
	}

	if (i == elem->num_src)
		return -EINVAL;

	ucontrol->value.enumerated.item[0] = i;

	return 0;
}


static int cc_elem_ctl_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct cc_path_list *path_list = NULL;
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_element *elem = &cc_priv->elems[mixer->shift];
	struct cc_interface *iface = container_of(elem->list,
					struct cc_interface, path_list);
	uint32_t sink_id = elem->elem_id->id;
	uint32_t src_id = 0;
	struct list_head *node = NULL, *next = NULL;
	int found = 0;
	uint32_t val = 0;

	if (elem->elem_type == ELEM_TYPE_MIX) {
		val = ucontrol->value.integer.value[0];
		if (val)
			src_id = elem->src_id[mixer->rshift].id;
		else
			src_id = 0;
	} else if (elem->elem_type == ELEM_TYPE_SWITCH) {
		src_id = val;
	} else {
		dev_err(comp->dev, "%s: invalid element: %d\n",
			__func__, elem->elem_id->id);
		return -EINVAL;
	}

	list_for_each_safe(node, next, elem->list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (path_list->sink_id == elem->elem_id->id) {
			found = 1;
			break;
		}
	}

	if (src_id && found)
		return 0;

	if (src_id) {
		path_list = kzalloc(sizeof(struct cc_path_list),
					GFP_KERNEL);
		if (!path_list)
			return -ENOMEM;

		path_list->src_id = src_id;
		path_list->sink_id = sink_id;
		list_add_tail(&path_list->list, elem->list);

		iface->payload += (sizeof(path_list->src_id) +
					sizeof(path_list->sink_id));
	} else if (found) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		list_del(node);
		kfree(path_list);
		path_list = NULL;
		iface->payload -= (sizeof(path_list->src_id) +
					sizeof(path_list->sink_id));
	}

	return 0;
}

static int cc_elem_ctl_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *w = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_element *elem = &cc_priv->elems[mixer->shift];
	struct cc_path_list *path_list = NULL;
	struct list_head *node = NULL, *next = NULL;
	int found = 0;

	list_for_each_safe(node, next, elem->list) {
		path_list = list_entry(node,
				struct cc_path_list, list);
		if (elem->elem_id->id == path_list->sink_id) {
			found = 1;
			break;
		}
	}

	if (elem->elem_type == ELEM_TYPE_MIX) {
		if (found && (path_list->src_id ==
				elem->src_id[mixer->rshift].id))
			ucontrol->value.integer.value[0] = 1;
		else
			ucontrol->value.integer.value[0] = 0;
	} else if (elem->elem_type == ELEM_TYPE_SWITCH) {
		ucontrol->value.integer.value[0] = found;
	} else {
		dev_err(comp->dev, "%s: invalid element\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void cc_widget_populate(struct cc_element *elem,
	enum snd_soc_dapm_type id, uint32_t num_kcontrol)
{
	elem->widget.id = id;
	elem->widget.name = elem->elem_id->name;
	elem->widget.reg = SND_SOC_NOPM;
	elem->widget.mask = 1;
	elem->widget.shift = 0;
	elem->widget.on_val = 1;
	elem->widget.off_val = 0;
	elem->widget.kcontrol_news = elem->kcontrol;
	elem->widget.num_kcontrols = num_kcontrol;
}

static int cc_add_mux_element(struct cc_element *elem, int index)
{
	int i = 0;

	elem->kcontrol = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!elem->kcontrol)
		return -ENOMEM;

	elem->texts = kzalloc(elem->num_src * sizeof(char *), GFP_KERNEL);
	if (!elem->texts)
		return -ENOMEM;

	for (i = 0; i < elem->num_src; i++)
		elem->texts[i] = elem->src_id[i].name;

	elem->elem_enum.reg = SND_SOC_NOPM;
	elem->elem_enum.shift_l = index;
	elem->elem_enum.shift_r = 0;
	elem->elem_enum.items = elem->num_src;
	elem->elem_enum.texts = (const char *const *)elem->texts;
	elem->elem_enum.mask = elem->num_src ?
			roundup_pow_of_two(elem->num_src) - 1 : 0;

	elem->kcontrol->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	elem->kcontrol->name = elem->elem_id->name;
	elem->kcontrol->info = snd_soc_info_enum_double;
	elem->kcontrol->get = cc_elem_ctl_mux_get;
	elem->kcontrol->put = cc_elem_ctl_mux_put;
	elem->kcontrol->private_value = (unsigned long)&elem->elem_enum;

	cc_widget_populate(elem, snd_soc_dapm_mux, 1);

	return 0;
}

static int cc_add_mixer_element(struct cc_element *elem, int index)
{
	int i = 0;
	uint32_t num_kcontrol = 0;

	num_kcontrol = elem->num_src;
	elem->kcontrol = kzalloc(num_kcontrol * sizeof(struct snd_kcontrol_new),
					GFP_KERNEL);
	if (!elem->kcontrol)
		return -ENOMEM;

	for (i = 0; i < num_kcontrol; i++) {
		elem->mixer.reg = SND_SOC_NOPM;
		elem->mixer.rreg = SND_SOC_NOPM;
		elem->mixer.shift = index;
		elem->mixer.rshift = i;
		elem->mixer.max = 1;
		elem->mixer.platform_max = 1;
		elem->mixer.invert = 0;
		elem->mixer.autodisable = 0;

		elem->kcontrol[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		elem->kcontrol[i].name = elem->src_id[i].name;
		elem->kcontrol[i].info = snd_soc_info_volsw;
		elem->kcontrol[i].get = cc_elem_ctl_get;
		elem->kcontrol[i].put = cc_elem_ctl_put;
		elem->kcontrol[i].private_value = (unsigned long)&elem->mixer;
	}

	cc_widget_populate(elem, snd_soc_dapm_mixer, num_kcontrol);

	return 0;
}

static int cc_add_switch_element(struct cc_element *elem, int index)
{
	if (!elem)
		return -EINVAL;

	elem->kcontrol = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!elem->kcontrol)
		return -ENOMEM;

	elem->mixer.reg = SND_SOC_NOPM;
	elem->mixer.rreg = SND_SOC_NOPM;
	elem->mixer.shift = index;
	elem->mixer.rshift = 0;
	elem->mixer.max = 1;
	elem->mixer.platform_max = 1;
	elem->mixer.invert = 0;
	elem->mixer.autodisable = 0;

	elem->kcontrol->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	elem->kcontrol->name = "Switch";
	elem->kcontrol->info = snd_soc_info_enum_double;
	elem->kcontrol->get = cc_elem_ctl_get;
	elem->kcontrol->put = cc_elem_ctl_put;
	elem->kcontrol->private_value = (unsigned long)&elem->mixer;

	cc_widget_populate(elem, snd_soc_dapm_switch, 1);

	return 0;
}

static int cc_add_input_element(struct cc_element *elem, int index)
{
	if (!elem)
		return -EINVAL;

	cc_widget_populate(elem, snd_soc_dapm_input, 0);

	return 0;
}

static int cc_add_output_element(struct cc_element *elem, int index)
{
	if (!elem)
		return -EINVAL;

	cc_widget_populate(elem, snd_soc_dapm_output, 0);

	return 0;
}

static int cc_add_stream_element(struct cc_element *elem, int index)
{
	if (!elem)
		return -EINVAL;

	strlcpy(elem->s_name, elem->elem_id->name,
		CC_MAX_STREAM_STRLEN);
	if (elem->elem_type == ELEM_TYPE_STREAM_IN) {
		cc_widget_populate(elem, snd_soc_dapm_aif_out, 0);
		strlcat(elem->s_name, " Capture", sizeof(elem->s_name));
	} else {
		cc_widget_populate(elem, snd_soc_dapm_aif_in, 0);
		strlcat(elem->s_name, " Playback", sizeof(elem->s_name));
	}

	elem->widget.sname = elem->s_name;

	return 0;
}

static int cc_parse_element(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_elem, uint32_t *parsed_len)
{
	int i = 0;
	uint32_t elem_type = 0;
	uint32_t num_src = 0;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;
	struct cc_element *elem = NULL;
	cc_add_elem_func *elem_func = NULL;
	int ret = 0;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	if (num_elem == 0 || num_elem > CC_NUM_ELEM_MAX) {
		pr_err("%s: requested elements %d not in range of 0 and %d\n",
				__func__, num_elem, CC_NUM_ELEM_MAX);
		return -EINVAL;
	}

	cc_priv->elems = kzalloc(num_elem * sizeof(struct cc_element),
					GFP_KERNEL);
	if (!cc_priv->elems)
		return -ENOMEM;

	cc_priv->num_elem = num_elem;
	*parsed_len = 0;

	for (i = 0; i < num_elem; i++) {
		elem = &cc_priv->elems[i];
		elem_type = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_parse_id_name(cc_priv, tmp, &tmp_len);
		if (ret)
			return ret;
		tmp += tmp_len;
		*parsed_len += tmp_len;

		switch (elem_type) {
		case ELEM_TYPE_MUX:
			num_src =  *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);
			elem_func = cc_add_mux_element;
			break;
		case ELEM_TYPE_MIX:
			num_src =  *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);
			elem_func = cc_add_mixer_element;
			break;
		case ELEM_TYPE_SWITCH:
			num_src = 0;
			elem_func = cc_add_switch_element;
			break;
		case ELEM_TYPE_INPUT:
			num_src = 0;
			elem_func = cc_add_input_element;
			break;
		case ELEM_TYPE_OUTPUT:
			num_src = 0;
			elem_func = cc_add_output_element;
			break;
		case ELEM_TYPE_STREAM_IN:
		case ELEM_TYPE_STREAM_OUT:
			num_src = 0;
			elem_func = cc_add_stream_element;
			break;
		case ELEM_TYPE_MAX:
		default:
			pr_err("%s: unsupported element type: %d\n",
					__func__, elem_type);
			return -EINVAL;
			break;
		}

		elem->elem_type = elem_type;
		elem->elem_id = &cc_priv->id_name[cc_priv->num_id - 1];
		elem->num_src = num_src;

		if (num_src) {
			ret = cc_parse_elem_src(cc_priv, elem,
							tmp, &tmp_len);
			if (ret)
				return ret;

			tmp += tmp_len;
			*parsed_len += tmp_len;
		}

		if (!elem_func)
			return -EINVAL;

		ret = elem_func(elem, i);
		if (ret) {
			pr_err("%s: could not add element: %d\n");
			return ret;
		}
	}

	return 0;
}

static inline int cc_act_is_volatile(uint32_t action_type)
{
	return (action_type % 2);
}

static int cc_action_set(uint32_t id, void *ptr, size_t size)
{
	struct cc_set_get_param_t *set_param = NULL;
	size_t param_sz = 0;
	int rc = 0;

	param_sz = sizeof(*set_param) - sizeof(set_param->payload) + size;

	set_param = (struct cc_set_get_param_t *)kzalloc(param_sz, GFP_KERNEL);
	set_param->action_id = id;
	memcpy(set_param->payload, ptr, size);

	rc = cc_send_pkt_generic_resp(CC_CODEC_OPCODE_SET_PARAM,
					set_param, param_sz);

	kfree(set_param);
	set_param = NULL;
	return rc;
}

static int cc_action_get(uint32_t id, void *ptr, size_t size)
{
	struct cc_set_get_param_t get_param;
	struct cc_set_get_param_t *resp = 0;
	size_t resp_size = 0;
	int rc = 0;

	memset(&get_param, 0, sizeof(get_param));
	get_param.action_id = id;
	rc = cc_send_packet(CC_CODEC_OPCODE_GET_PARAM,
		&get_param, sizeof(get_param), (void **)&resp, &resp_size);

	if (rc || (resp_size != size))
		return -EINVAL;

	memcpy(ptr, resp->payload, size);

	return 0;
}

static int cc_action_ctl_enum_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_enum *tmp_enum = (struct soc_enum *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[tmp_enum->shift_l];
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_action_value_list *act_val = NULL;
	uint32_t value = 0;
	int rc = 0;

	if (cc_act_is_volatile(act->action_type)) {
		rc = cc_action_get(act->act_id->id, &value, sizeof(uint32_t));
		if (!rc)
			ucontrol->value.enumerated.item[0] = value;
		return rc;
	}

	list_for_each_safe(node, next, act->list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			break;
		}
	}

	if (found)
		ucontrol->value.enumerated.item[0] = act_val->param.enumerated;
	else
		ucontrol->value.enumerated.item[0] = 0;

	return 0;
}

static int cc_action_ctl_enum_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_enum *tmp_enum = (struct soc_enum *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[tmp_enum->shift_l];
	struct cc_action_value_list *act_val = NULL;
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_interface *iface = NULL;
	uint32_t value = 0;

	iface = container_of(act->list, struct cc_interface,
				action_value_list);

	if (cc_act_is_volatile(act->action_type)) {
		value = ucontrol->value.enumerated.item[0];
		return cc_action_set(act->act_id->id, &value, sizeof(uint32_t));
	}

	list_for_each_safe(node, next, act->list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			return 0;
		}
	}

	act_val = kzalloc(sizeof(struct cc_action_value_list), GFP_KERNEL);
	act_val->action_id = act->act_id->id;
	act_val->num_params = 1;
	act_val->action_type = act->action_type;
	act_val->param.enumerated = ucontrol->value.enumerated.item[0];
	list_add_tail(&act_val->list, act->list);
	iface->action_size += (sizeof(act_val->action_id) +
				sizeof(act_val->param.enumerated));

	return 0;
}

static int cc_action_ctl_array_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[mixer->shift];
	struct cc_action_value_list *act_val = NULL;
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	size_t size = 0;
	void *ptr = NULL;
	int i = 0;

	if (cc_act_is_volatile(act->action_type)) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY_VOLATILE:
			size = act_val->num_params * sizeof(uint8_t);
			ptr = (void *)&ucontrol->value.bytes.data[0];
			break;
		case ACTION_TYPE_INT_ARRAY_VOLATILE:
		case ACTION_TYPE_UINT_ARRAY_VOLATILE:
			size = act_val->num_params * sizeof(int32_t);
			ptr = (void *)&ucontrol->value.integer.value[0];
			break;
		default:
			return -EINVAL;
			break;
		}
		return cc_action_get(act->act_id->id, ptr, size);
	}

	list_for_each_safe(node, next, act->list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			break;
		}
	}

	if (!found)
		return 0;

	for (i = 0; i < act->num_params; i++) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY:
			ucontrol->value.bytes.data[i] =
				act_val->param.char_array[i];
			break;
		case ACTION_TYPE_INT_ARRAY:
			ucontrol->value.integer.value[i] =
				act_val->param.int_array[i];
			break;
		case ACTION_TYPE_UINT_ARRAY:
			ucontrol->value.integer.value[i] =
				act_val->param.uint_array[i];
			break;
		default:
			break;
		}
	}

	return 0;
}

static int cc_action_ctl_array_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cc_codec_priv *cc_priv = dev_get_drvdata(comp->dev);
	struct soc_multi_mixer_control *mixer =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	struct cc_action *act = &cc_priv->actions[mixer->shift];
	struct cc_action_value_list *act_val = NULL;
	int found = 0;
	struct list_head *node = NULL, *next = NULL;
	struct cc_interface *iface = NULL;
	size_t size = 0;
	void *ptr = NULL;
	int i = 0;

	iface = container_of(act->list, struct cc_interface,
				action_value_list);

	if (cc_act_is_volatile(act->action_type)) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY_VOLATILE:
			size = act_val->num_params * sizeof(uint8_t);
			ptr = (void *)&ucontrol->value.bytes.data[0];
			break;
		case ACTION_TYPE_INT_ARRAY_VOLATILE:
		case ACTION_TYPE_UINT_ARRAY_VOLATILE:
			size = act_val->num_params * sizeof(int32_t);
			ptr = (void *)&ucontrol->value.integer.value[0];
			break;
		default:
			return -EINVAL;
			break;
		}
		return cc_action_set(act->act_id->id, ptr, size);
	}

	list_for_each_safe(node, next, act->list) {
		act_val = list_entry(node,
				struct cc_action_value_list, list);
		if (act_val->action_id == act->act_id->id) {
			found = 1;
			break;
		}
	}

	if (!found) {
		act_val = kzalloc(sizeof(struct cc_action_value_list),
					GFP_KERNEL);
		if (!act_val)
			return -ENOMEM;

		act_val->action_id = act->act_id->id;
		act_val->action_type = act->action_type;
		act_val->num_params = act->num_params;
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY:
			size = act_val->num_params * sizeof(uint8_t);
			act_val->param.char_array = kzalloc(size, GFP_KERNEL);
			break;
		case ACTION_TYPE_INT_ARRAY:
			size = act_val->num_params * sizeof(int32_t);
			act_val->param.int_array = kzalloc(size, GFP_KERNEL);
			break;
		case ACTION_TYPE_UINT_ARRAY:
			size = act_val->num_params * sizeof(uint32_t);
			act_val->param.uint_array = kzalloc(size, GFP_KERNEL);
			break;
		default:
			return -EINVAL;
			break;
		}
		iface->action_size += (size + sizeof(act_val->action_id));
	}

	for (i = 0; i < act->num_params; i++) {
		switch (act->action_type) {
		case ACTION_TYPE_CHAR_ARRAY:
			act_val->param.char_array[i] =
					ucontrol->value.bytes.data[i];
			break;
		case ACTION_TYPE_INT_ARRAY:
			act_val->param.int_array[i] =
				ucontrol->value.integer.value[i];
			break;
		case ACTION_TYPE_UINT_ARRAY:
			act_val->param.uint_array[i] =
				ucontrol->value.integer.value[i];
			break;
		default:
			return -EINVAL;
			break;
		}
	}

	return 0;
}

static int cc_create_enum_action(struct cc_codec_priv *cc_priv,
	struct cc_action *action, uint8_t *payload,
	uint32_t *parsed_len, int index)
{
	int i = 0;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;
	int ret = 0;

	*parsed_len = 0;
	action->enum_str = kzalloc(
		action->num_params * sizeof(*(action->enum_str)), GFP_KERNEL);
	if (!action->enum_str)
		return -ENOMEM;

	action->texts = kzalloc(
		action->num_params * sizeof(char *), GFP_KERNEL);
	if (!action->texts)
		return -ENOMEM;;

	for (i = 0; i < action->num_params; i++) {
		ret = cc_parse_name(action->enum_str[i], tmp, &tmp_len);
		if (ret)
			return ret;
		tmp += tmp_len;
		*parsed_len += tmp_len;
		action->texts[i] = action->enum_str[i];
	}

	action->action_enum.reg = SND_SOC_NOPM;
	action->action_enum.shift_l = index;
	action->action_enum.shift_r = 0;
	action->action_enum.items = action->num_params;
	action->action_enum.texts = (const char *const *)action->texts;
	action->action_enum.mask = action->num_params ?
			roundup_pow_of_two(action->num_params) - 1 : 0;

	action->kcontrol.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	action->kcontrol.name = action->act_id->name;
	action->kcontrol.info = snd_soc_info_enum_double;
	action->kcontrol.get = cc_action_ctl_enum_get;
	action->kcontrol.put = cc_action_ctl_enum_put;
	action->kcontrol.private_value = (unsigned long)&action->action_enum;

	return 0;
}

static int cc_create_array_action(struct cc_codec_priv *cc_priv,
					struct cc_action *action, int index)
{
	action->mixer.reg = SND_SOC_NOPM;
	action->mixer.shift = index;
	action->mixer.rshift = 0;
	action->mixer.max = action->param_max;
	action->mixer.count = action->num_params;
	action->mixer.platform_max = action->param_max;
	action->mixer.invert = 0;

	action->kcontrol.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	action->kcontrol.name = action->act_id->name;
	action->kcontrol.info = snd_soc_info_multi_ext;
	action->kcontrol.get = cc_action_ctl_array_get;
	action->kcontrol.put = cc_action_ctl_array_put;
	action->kcontrol.private_value = (unsigned long)&action->mixer;

	return 0;
}

static int cc_parse_action(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_action, uint32_t *parsed_len)
{
	int ret = 0;
	struct cc_action *action = NULL;
	uint8_t *tmp = payload;
	uint32_t tmp_len = 0;
	int i = 0;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	*parsed_len = 0;
	if (num_action == 0 || num_action > CC_NUM_ACTIONS_MAX)
		return -EINVAL;

	cc_priv->num_action = num_action;
	cc_priv->actions = kzalloc(num_action * sizeof(struct cc_action),
					GFP_KERNEL);
	if (!cc_priv->actions)
		return -ENOMEM;

	for (i = 0; i < num_action; i++) {
		action = &cc_priv->actions[i];
		action->action_type = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_parse_id_name(cc_priv, tmp, &tmp_len);
		if (ret)
			return ret;
		tmp += tmp_len;
		*parsed_len += tmp_len;

		action->num_params = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		action->act_id = &cc_priv->id_name[cc_priv->num_id - 1];

		switch (action->action_type) {
		case ACTION_TYPE_ENUM:
		case ACTION_TYPE_ENUM_VOLATILE:
			ret = cc_create_enum_action(cc_priv, action,
							tmp, &tmp_len, i);
			if (ret)
				return ret;
			tmp += tmp_len;
			*parsed_len += tmp_len;
			break;
		case ACTION_TYPE_CHAR_ARRAY:
		case ACTION_TYPE_CHAR_ARRAY_VOLATILE:
		case ACTION_TYPE_INT_ARRAY:
		case ACTION_TYPE_INT_ARRAY_VOLATILE:
		case ACTION_TYPE_UINT_ARRAY:
		case ACTION_TYPE_UINT_ARRAY_VOLATILE:
			action->param_min = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			action->param_max = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			action->def = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_create_array_action(cc_priv, action, i);
			if (ret)
				return ret;
			break;
		case ACTION_TYPE_MAX:
		default:
			pr_err("%s: unsupported action type: %d\n",
					__func__, action->action_type);
			return -EINVAL;
			break;
		}
	}

	return 0;
}

static int cc_find_name_from_id(struct cc_codec_priv *cc_priv,
		uint32_t id, char **str)
{
	int i = 0;

	if (!cc_priv || !str)
		return -EINVAL;

	for (i = 0; i < cc_priv->num_id; i++) {
		if (cc_priv->id_name[i].id == id)
			break;
	}

	if (i == cc_priv->num_id) {
		pr_err("%s: invalid id %d\n", __func__, id);
		return -EINVAL;
	}

	*str = cc_priv->id_name[i].name;

	return 0;
}

static int cc_find_elem_from_id(struct cc_codec_priv *cc_priv,
		uint32_t id, struct cc_element **elem)
{
	int i = 0;

	if (!cc_priv || !elem)
		return -EINVAL;

	for (i = 0; i < cc_priv->num_elem; i++) {
		if (cc_priv->elems[i].elem_id->id == id)
			break;
	}

	if (i == cc_priv->num_elem) {
		pr_err("%s: invalid element id %d\n", __func__, id);
		return -EINVAL;
	}

	*elem = &cc_priv->elems[i];

	return 0;
}

static int cc_find_action_from_id(struct cc_codec_priv *cc_priv,
		uint32_t id, struct cc_action **act)
{
	int i = 0;

	if (!cc_priv || !act)
		return -EINVAL;

	for (i = 0; i < cc_priv->num_action; i++) {
		if (cc_priv->actions[i].act_id->id == id)
			break;
	}

	if (i == cc_priv->num_action) {
		pr_err("%s: invalid element id %d\n", __func__, id);
		return -EINVAL;
	}

	*act = &cc_priv->actions[i];

	return 0;
}

static int cc_parse_route(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_route, uint32_t *parsed_len)
{
	int ret = 0;
	uint8_t *tmp = payload;
	uint32_t route_type = 0;
	uint32_t id = 0;
	char *str = NULL;
	int i = 0;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	if (num_route == 0 || num_route > CC_NUM_ROUTES_MAX)
		return -EINVAL;

	cc_priv->audio_map = kzalloc(
		num_route * sizeof(struct snd_soc_dapm_route), GFP_KERNEL);
	if (!cc_priv->audio_map)
		return -ENOMEM;

	*parsed_len = 0;
	cc_priv->num_routes = num_route;

	for (i = 0; i < num_route; i++) {
		route_type = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		id = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_find_name_from_id(cc_priv, id, &str);
		if (ret)
			return ret;
		cc_priv->audio_map[i].sink = str;

		switch (route_type) {
		case ROUTE_TYPE_CONTROL:
			id = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_find_name_from_id(cc_priv, id, &str);
			if (ret)
				return ret;
			break;
		case ROUTE_TYPE_SWITCH:
			str = "Switch";
			break;
		case ROUTE_TYPE_DUMMY:
			str = NULL;
			break;
		case ROUTE_TYPE_MAX:
		default:
			pr_err("%s: unsupported route type: %d\n",
					__func__, route_type);
			return -EINVAL;
			break;
		}

		cc_priv->audio_map[i].control = str;

		id = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_find_name_from_id(cc_priv, id, &str);
		if (ret)
			return ret;
		cc_priv->audio_map[i].source = str;

		pr_info("++++ parsed routes: %s %s %s\n",
			cc_priv->audio_map[i].sink,
			cc_priv->audio_map[i].control ? cc_priv->audio_map[i].control : "NULL",
			cc_priv->audio_map[i].source);
	}

	return 0;
}

static int cc_dai_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	pr_info("++++ %s:%d\n", __func__, __LINE__);
#if 0
	struct cc_codec_priv cc_priv = (struct cc_codec_priv *)
					dai_get_drvdata(dai->component->dev);
	int id = dai->driver->id;
	struct cc_interface *iface = cc_priv->iface[id];
#endif
	/* TODO: create payload */
	return 0;
}

static void cc_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	pr_info("++++ %s:%d\n", __func__, __LINE__);
#if 0
	struct cc_codec_priv cc_priv = (struct cc_codec_priv *)
					dai_get_drvdata(dai->component->dev);
	int id = dai->driver->id;
	struct cc_interface *iface = cc_priv->iface[id];
#endif
	/* TODO: create payload */
}

static int cc_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params, struct snd_soc_dai *dai)
{
	struct cc_codec_priv *cc_priv = (struct cc_codec_priv *)
					dev_get_drvdata(dai->component->dev);
	int id = dai->driver->id;
	struct cc_interface *iface = &cc_priv->iface[id];

	pr_info("++++ %s:%d\n", __func__, __LINE__);
	iface->uc_header.sample_rate = params_rate(hw_params);
	iface->uc_header.channels = params_channels(hw_params);
	iface->uc_header.bit_width = params_width(hw_params);

	return 0;
}

static int cc_dai_trigger(struct snd_pcm_substream *substream, int on,
		struct snd_soc_dai *dai)
{
	pr_info("++++ %s:%d ON: %d\n", __func__, __LINE__, on);
	return 0;
}

static struct snd_soc_dai_ops cc_dai_ops = {
	.hw_params = cc_dai_hw_params,
	.startup = cc_dai_startup,
	.shutdown = cc_dai_shutdown,
	.trigger = cc_dai_trigger,
};

static int cc_parse_interface(struct cc_codec_priv *cc_priv,
		uint8_t *payload, uint32_t num_intf, uint32_t *parsed_len)
{
	int ret = 0;
	uint8_t *tmp = payload;
	uint32_t id = 0;
	int i = 0;
	int j = 0;
	struct cc_interface *iface = NULL;
	uint32_t num_elements = 0;
	uint32_t num_actions = 0;
	struct cc_element *elem = NULL;
	struct cc_action *act = NULL;
	struct snd_soc_pcm_stream *stream = NULL;

	if (!cc_priv || !payload || !parsed_len)
		return -EINVAL;

	if (num_intf == 0 || num_intf > CC_NUM_IFACE_MAX)
		return -EINVAL;

	cc_priv->iface = kzalloc(
		num_intf * sizeof(struct cc_interface), GFP_KERNEL);
	if (!cc_priv->iface)
		return -ENOMEM;

	cc_priv->dai = kzalloc(
		num_intf * sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!cc_priv->iface)
		return -ENOMEM;

	*parsed_len = 0;
	cc_priv->num_iface = num_intf;

	for (i = 0; i < num_intf; i++) {
		iface = cc_priv->iface;
		id = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		ret = cc_find_elem_from_id(cc_priv, id, &elem);
		if (ret) {
			pr_err("%s: Invalid stream id %d in interface\n",
				__func__, id);
			return ret;
		}

		if (elem->elem_type == ELEM_TYPE_STREAM_IN) {
			stream = &cc_priv->dai[i].playback;
		} else if (elem->elem_type == ELEM_TYPE_STREAM_OUT) {
			stream = &cc_priv->dai[i].capture;
		} else {
			pr_err("%s: %d Element is not stream\n", __func__, id);
			return -EILSEQ;
		}
		stream->stream_name = elem->s_name;

		stream->formats = *(uint64_t *)tmp;
		tmp += sizeof(uint64_t);
		*parsed_len += sizeof(uint64_t);

		stream->rates = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->rate_min = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->rate_max = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->channels_min = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		stream->channels_max = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		num_elements = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		num_actions = *(uint32_t *)tmp;
		tmp += sizeof(uint32_t);
		*parsed_len += sizeof(uint32_t);

		cc_priv->dai[i].name = elem->elem_id->name;
		cc_priv->dai[i].id = i + 1;
		cc_priv->dai[i].ops = &cc_dai_ops;

		INIT_LIST_HEAD(&cc_priv->iface[i].path_list);
		INIT_LIST_HEAD(&cc_priv->iface[i].action_value_list);

		pr_info("++++ dai name: %s\n", cc_priv->dai[i].name);
		pr_info("++++ dai id: %u\n", cc_priv->dai[i].id);
		pr_info("++++ dai stream: %s\n", stream->stream_name);
		pr_info("++++ dai format: %llx\n", stream->formats);
		pr_info("++++ dai rates: %x\n", stream->rates);
		pr_info("++++ dai rate_min: %u\n", stream->rate_min);
		pr_info("++++ dai rate_max: %u\n", stream->rate_max);
		pr_info("++++ dai ch_min: %u\n", stream->channels_min);
		pr_info("++++ dai ch_max: %u\n", stream->channels_max);
		for (j = 0; j < num_elements; j++) {
			id = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_find_elem_from_id(cc_priv, id, &elem);
			if (ret) {
				pr_err("%s: iface:%d, invalid elem:%d\n",
					__func__, i, id);
				return ret;
			}

			elem->list = &cc_priv->iface[i].path_list;
		}

		for (j = 0; j < num_actions; j++) {
			id = *(uint32_t *)tmp;
			tmp += sizeof(uint32_t);
			*parsed_len += sizeof(uint32_t);

			ret = cc_find_action_from_id(cc_priv, id, &act);
			if (ret) {
				pr_err("%s: iface:%d, invalid action:%d\n",
					__func__, i, id);
				return ret;
			}

			act->list = &cc_priv->iface[i].action_value_list;
		}
	}

	return 0;
}

static int cc_parse_plat_info(struct cc_codec_priv *cc_priv,
		struct device *dev, uint8_t *plat_info, uint32_t size)
{
	struct cc_plat_info_t *pinfo = NULL;
	uint8_t *tmp = NULL;
	uint32_t tmp_len = 0;
	int ret = 0;

	if (size < CC_MIN_PLATINFO_SIZE) {
		pr_err("%s: Invalid payload\n", __func__);
		return -EINVAL;
	}

	pinfo = (struct cc_plat_info_t *)plat_info;
	pr_info("++++ id:%d,dev:%d,elem:%d,act:%d, route:%d, iface: %d\n",
		pinfo->num_id, pinfo->num_dev, pinfo->num_elem,
		pinfo->num_action, pinfo->num_route, pinfo->num_intf);

	cc_priv->id_name = kzalloc(pinfo->num_id *
				sizeof(struct cc_id_name_map), GFP_KERNEL);
	if (!cc_priv->id_name)
		return -ENOMEM;

	tmp = pinfo->payload;

	ret = cc_parse_reg_region(cc_priv, dev, tmp,
				pinfo->num_dev, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;

	ret = cc_parse_element(cc_priv, tmp, pinfo->num_elem, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;

	ret = cc_parse_action(cc_priv, tmp, pinfo->num_action, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;

	ret = cc_parse_route(cc_priv, tmp, pinfo->num_route, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;

	ret = cc_parse_interface(cc_priv, tmp, pinfo->num_intf, &tmp_len);
	if (ret)
		return ret;
	tmp += tmp_len;

	if (tmp != (plat_info + size)) {
		pr_err("%s: Invalid platform information size\n", __func__);
		return -EILSEQ;
	}

	return 0;
}

static int cc_soc_codec_probe(struct snd_soc_component *component)
{
	int ret = 0;
	struct cc_codec_priv *cc_priv = dev_get_drvdata(component->dev);
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);
	struct device *dev = component->dev;
	int i = 0;

	pr_info("+++ %s:%d\n", __func__, __LINE__);
	for (i = 0; i < cc_priv->num_elem; i++) {
		ret = snd_soc_dapm_new_controls(dapm,
				&cc_priv->elems[i].widget, 1);
		if (ret < 0) {
			dev_err(dev, "%s: failed to add dapm control, ret:%d\n",
				__func__, ret);
			return ret;
		}
	}

	ret = snd_soc_dapm_add_routes(dapm,
			cc_priv->audio_map, cc_priv->num_routes);
	if (ret < 0) {
		dev_err(dev, "%s: failed to add routes, ret:%d\n",
				__func__, ret);
		return ret;
	}

	ret = snd_soc_dapm_new_widgets(dapm->card);
	if (ret < 0) {
		dev_err(dev, "%s: failed to add widgets, ret:%d\n",
				__func__, ret);
		return ret;
	}

	for (i = 0; i < cc_priv->num_action; i++) {
		ret = snd_soc_add_component_controls(component,
			&cc_priv->actions[i].kcontrol, 1);
		if (ret < 0) {
			dev_err(dev, "%s: failed to add snd ctrls, ret:%d\n",
					__func__, ret);
			return ret;
		}
	}

	return 0;
}

static void cc_soc_codec_remove(struct snd_soc_component *component)
{
	return;
}

static int cc_get_plat_info(uint8_t **ptr, uint32_t *size)
{
	if (!ptr || !size)
		return -EINVAL;

	/* TODO: integrate it with IPC */
	return 0;
}

static void cc_cleanup_iface(struct cc_interface *iface)
{
	struct cc_path_list *path = NULL;
	struct cc_action_value_list *act_val = NULL;
	struct list_head *next = NULL, *node = NULL;
	int type = 0;

	if (iface->path_list.prev && iface->path_list.next) {
		list_for_each_safe(node, next, &iface->path_list) {
			path = list_entry(node,
					struct cc_path_list, list);
			list_del(node);
			if (path)
				kfree(path);
		}
	}

	if (iface->action_value_list.prev && iface->action_value_list.next) {
		list_for_each_safe(node, next, &iface->action_value_list) {
			act_val = list_entry(node,
					struct cc_action_value_list, list);
			type = act_val ? act_val->action_type : -1;

			if (type == ACTION_TYPE_CHAR_ARRAY) {
				kfree(act_val->param.char_array);
				act_val->param.char_array = NULL;
			} else if (type == ACTION_TYPE_INT_ARRAY) {
				kfree(act_val->param.int_array);
				act_val->param.int_array = NULL;
			} else if (type == ACTION_TYPE_UINT_ARRAY) {
				kfree(act_val->param.uint_array);
				act_val->param.char_array = NULL;
			}
			list_del(node);
			if (act_val)
				kfree(act_val);
		}
	}

	memset(&iface->path_list, 0, sizeof(struct list_head));
	memset(&iface->action_value_list, 0, sizeof(struct list_head));
}

static void cc_cleanup_action(struct cc_action *action)
{
	if (action->act_id)
		action->act_id = NULL;

	if (action->enum_str) {
		kfree(action->enum_str);
		action->enum_str = NULL;
	}

	if (action->texts) {
		kfree(action->texts);
		action->texts = NULL;
	}

	if (action->list)
		action->list = NULL;
}

static void cc_cleanup_element(struct cc_element *element)
{
	if (element->kcontrol) {
		kfree(element->kcontrol);
		element->kcontrol = NULL;
	}

	if (element->texts) {
		kfree(element->texts);
		element->texts = NULL;
	}

	element->elem_id = NULL;

	element->src_id = NULL;
}

static void cc_cleanup(struct cc_codec_priv *cc_priv)
{
	int i = 0;

	if (!cc_priv)
		return;

	if (cc_priv->iface) {
		for (i = 0; i < cc_priv->num_iface; i++)
			cc_cleanup_iface(&cc_priv->iface[i]);
		kfree(cc_priv->iface);
		cc_priv->iface = NULL;
		kfree(cc_priv->dai);
		cc_priv->dai = NULL;
		cc_priv->num_iface = 0;
	}

	if (cc_priv->audio_map) {
		kfree(cc_priv->audio_map);
		cc_priv->audio_map = NULL;
		cc_priv->num_routes = 0;
	}

	if (cc_priv->actions) {
		for (i = 0; i < cc_priv->num_action; i++)
			cc_cleanup_action(&cc_priv->actions[i]);
		kfree(cc_priv->actions);
		cc_priv->actions = NULL;
		cc_priv->num_action = 0;
	}

	if (cc_priv->elems) {
		for (i = 0; i < cc_priv->num_elem; i++)
			cc_cleanup_element(&cc_priv->elems[i]);
		kfree(cc_priv->elems);
		cc_priv->num_elem = 0;
	}

	if (cc_priv->id_name) {
		kfree(cc_priv->id_name);
		cc_priv->id_name = NULL;
		cc_priv->num_id = 0;
	}
}

static int cc_cdc_probe(struct platform_device *pdev)
{
	struct cc_codec_priv *cc_priv = NULL;
	uint32_t plat_info_size = 0;
	uint8_t *plat_info = NULL;
	int ret = 0;

	cc_priv = devm_kzalloc(&pdev->dev, sizeof(struct cc_codec_priv),
					GFP_KERNEL);
	if (!cc_priv)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, cc_priv);

	ret = cc_get_plat_info(&plat_info, &plat_info_size);
	if (!plat_info || ret) {
		dev_err(&pdev->dev, "%s: cc payload not found\n", __func__);
		goto free_priv;
	}

	ret = cc_parse_plat_info(cc_priv, &pdev->dev,
				plat_info, plat_info_size);
	if (ret)
		goto free_priv;

	cc_priv->cc_comp.probe = cc_soc_codec_probe;
	cc_priv->cc_comp.remove = cc_soc_codec_remove;
	/* TODO: read it from device tree */
	cc_priv->cc_comp.name = "cc_codec";

	ret = snd_soc_register_component(&pdev->dev, &cc_priv->cc_comp,
				cc_priv->dai, cc_priv->num_iface);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: register codec failed, ret:%d\n",
				__func__, ret);
		goto free_priv;
	}

	return 0;

free_priv:
	cc_cleanup(cc_priv);
	devm_kfree(&pdev->dev, cc_priv);
	cc_priv = NULL;
	return ret;
}

static int cc_cdc_remove(struct platform_device *pdev)
{
	struct cc_codec_priv *cc_priv = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);
	cc_cleanup(cc_priv);
	devm_kfree(&pdev->dev, cc_priv);
	cc_priv = NULL;

	return 0;
}

static const struct of_device_id cc_cdc_dt_match[] = {
	{.compatible = "qcom,cc-codec"},
	{}
};

static struct platform_driver cc_cdc_driver = {
	.driver = {
		.name = "cc-codec",
		.owner = THIS_MODULE,
		.of_match_table = cc_cdc_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = cc_cdc_probe,
	.remove = cc_cdc_remove,
};

static int __init cc_cdc_init(void)
{
	return platform_driver_register(&cc_cdc_driver);
}
module_init(cc_cdc_init);

static void __exit cc_cdc_exit(void)
{
	platform_driver_unregister(&cc_cdc_driver);
}
module_exit(cc_cdc_exit);

MODULE_DESCRIPTION("cc codec driver");
MODULE_LICENSE("GPL v2");
