/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef __CC_RTOS_H__
#define __CC_RTOS_H__

/* Minimum platform information size */
#define CC_MIN_PLATINFO_SIZE 24

/* Maximum allowed debug register regions */
#define CC_MAX_REG_REGION 16

/* Maximum allowed string length including terminating NULL character */
#define CC_MAX_STRLEN 20

/* Maximum allowed stream name length including terminall NULL character */
#define CC_MAX_STREAM_STRLEN 28

/* Maximum number of elements & srouces per element allowed */
#define CC_NUM_ELEM_MAX 512
#define CC_MAX_NUM_SRC_PER_ELEM 32

/* Maximum number of action and number of parameters per action allowed */
#define CC_NUM_ACTIONS_MAX 512
#define CC_MAX_NUM_PARAM_PER_ACT 128

/* Maximum number of routes */
#define CC_NUM_ROUTES_MAX 512

/* Maximum number of interfaces */
#define CC_NUM_IFACE_MAX 32

enum cc_path_types {
	PATH_TYPE_DUMMY,
	PATH_TYPE_CTRL,
	PATH_TYPE_MAX
};

enum cc_element_types {
	ELEM_TYPE_MUX,
	ELEM_TYPE_MIX,
	ELEM_TYPE_SWITCH,
	ELEM_TYPE_INPUT,
	ELEM_TYPE_OUTPUT,
	ELEM_TYPE_STREAM_IN,
	ELEM_TYPE_STREAM_OUT,
	ELEM_TYPE_MAX
};

enum cc_action_types {
	ACTION_TYPE_ENUM,
	ACTION_TYPE_ENUM_VOLATILE,
	ACTION_TYPE_CHAR_ARRAY,
	ACTION_TYPE_CHAR_ARRAY_VOLATILE,
	ACTION_TYPE_INT_ARRAY,
	ACTION_TYPE_INT_ARRAY_VOLATILE,
	ACTION_TYPE_UINT_ARRAY,
	ACTION_TYPE_UINT_ARRAY_VOLATILE,
	ACTION_TYPE_MAX,
};

enum cc_route_type {
	ROUTE_TYPE_CONTROL,
	ROUTE_TYPE_SWITCH,
	ROUTE_TYPE_DUMMY,
	ROUTE_TYPE_MAX,
};

struct cc_plat_info_t {
	uint32_t num_id;
	uint32_t num_dev;
	uint32_t num_elem;
	uint32_t num_action;
	uint32_t num_route;
	uint32_t num_intf;
	uint8_t payload[0];
} __packed;

struct cc_uc_start_stop_t {
	uint32_t version;
	uint32_t sample_rate;
	uint32_t bit_width;
	uint32_t direction;
	uint32_t stream;
	uint32_t num_paths;
	uint32_t num_action;
	uint8_t payload[0];
} __packed;

struct cc_set_get_param_t {
	uint32_t version;
	uint32_t action_id;
	uint8_t payload[0];
} __packed;

struct cc_read_write_reg_req {
	uint32_t version;
	uint32_t start_addr;
	uint32_t num_reg;
	uint8_t payload[0];
} __packed;

#define RESPONSE_SUCESS 1
#define RESPONSE_FAILURE 0

struct cc_resp_generic {
	uint32_t opcode;
	uint32_t response;
} __packed;

/* OPCODE */
#define CC_CODEC_OPCODE_GET_PLAT_INFO 0
#define CC_CODEC_OPCODE_GET_PARAM 0x00000001
#define CC_CODEC_OPCODE_SET_PARAM 2
#define CC_CODEC_OPCODE_START_USECASE 3
#define CC_CODEC_OPCODE_STOP_USECASE 4
#define CC_CODEC_OPCODE_READ_REGISTER 5
#define CC_CODEC_OPCODE_WRITE_REGISTER 6

#define CC_CODEC_RESPONSE_CMD 0x100
#define CC_CODEC_CH_INFO 0

#endif //__CC_RTOS_H__
