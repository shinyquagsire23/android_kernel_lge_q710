/* touch_ili9881h_mp.c
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: BSP-TOUCH@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef LGE_TOUCH_ILI9881H_MP_H
#define LGE_TOUCH_ILI9881H_MP_H

#define PARSER_MAX_CFG_BUF			(512 * 3)
#define PARSER_MAX_KEY_NUM			400 // 1800
#define PARSER_MAX_KEY_NAME_LEN			50 // 100
#define PARSER_MAX_KEY_VALUE_LEN		300 // 2000
#define BENCHMARK_KEY_NAME			"Benchmark_Data"
#define NODE_TYPE_KEY_NAME			"node_type_Data"

#define INI_ERR_OUT_OF_LINE			-1

#define MP_PASS					1
#define MP_FAIL					-1
#define BENCHMARK				1
#define NODETYPE				1
#define RAWDATA_NO_BK_DATA_SHIFT_9881H		8192
#define RAWDATA_NO_BK_DATA_SHIFT_9881F		4096
#define TYPE_BENCHMARK				0
#define TYPE_NO_JUGE				1
#define TYPE_JUGE				2
#define TYPE_OPTION_VALUE			0

#define INT_CHECK				0
#define POLL_CHECK				1
#define DELAY_CHECK				2
#define RETRY_COUNT				3

#define CSV_PATH				"/sdcard"
#define CSV_PASS_NAME				"touch_mp_pass"
#define CSV_FAIL_NAME				"touch_mp_fail"

#define CSV_FILE_SIZE				(500 * 1024)//500KByte
#define LOG_BUF_SIZE				(256)
#define TIME_STR_LEN				(64)
#define MAX_LOG_FILE_SIZE			(10 * 1024 * 1024) /* 10 M byte */
#define MAX_LOG_FILE_COUNT			(4)

#define CMD_MP_CDC_INIT				0xF1

enum mp_test_cmd {
	CMD_DAC = 0x1,
	CMD_RAWDATA_BK = 0x8,
	CMD_RADDATA_NO_BK = 0x5,
	CMD_SHORT = 0x4,
	CMD_GET_TIMING = 0x30,
	CMD_LCM_HEADER = 0xF,
	CMD_CDC_LEN = 16,
};

enum mp_test_catalog {
	MUTUAL_TEST = 0,
	SELF_TEST = 1,
	KEY_TEST = 2,
	ST_TEST = 3,
	TX_RX_DELTA = 4,
	UNTOUCH_P2P = 5,
	PIXEL = 6,
	OPEN_TEST = 7,
	PEAK_TO_PEAK_TEST = 8,
	SHORT_TEST = 9,
};

enum open_test_node_type {
	NO_COMPARE = 0x00,	/* Not A Area, No Compare  */
	AA_Area = 0x01,		/* AA Area, Compare using Charge_AA  */
	Border_Area = 0x02,	/* Border Area, Compare using Charge_Border  */
	Notch = 0x04,		/* Notch Area, Compare using Charge_Notch  */
	Round_Corner = 0x08,	/* Round Corner, No Compare */
	Skip_Micro = 0x10,	/* Skip_Micro, No Compare */
};

struct ini_file_data {
	char pSectionName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyValue[PARSER_MAX_KEY_VALUE_LEN];
	int iSectionNameLen;
	int iKeyNameLen;
	int iKeyValueLen;
};

struct mp_test_P540_open {
	int32_t *cbk_700;
	int32_t *cbk_250;
	int32_t *cbk_200;
	int32_t *charg_rate;
	int32_t *full_Open;
	int32_t *dac;
};

struct mp_test_items {
	char *name;
	/* The description must be the same as ini's section name */
	char *desp;
	char *result;
	int catalog;
	uint8_t spec_option;
	uint8_t type_option;
	bool run;
	int max;
	int max_res;
	int item_result;
	int min;
	int min_res;
	int frame_count;
	int trimmed_mean;
	int lowest_percentage;
	int highest_percentage;
	int32_t *result_buf;
	int32_t *buf;
	int32_t *max_buf;
	int32_t *min_buf;
	int32_t *bench_mark_max;
	int32_t *bench_mark_min;
	int32_t *node_type;
	int (*do_test)(int index);
};

struct mp_nodp_calc {
	bool is60HZ;
	bool isLongV;

	/* Input */
	uint16_t tshd;
	uint8_t multi_term_num_120;
	uint8_t multi_term_num_60;
	uint16_t tsvd_to_tshd;
	uint16_t qsh_tdf;

	/* Settings */
	uint8_t auto_trim;
	uint16_t tp_tshd_wait_120;
	uint16_t ddi_width_120;
	uint16_t tp_tshd_wait_60;
	uint16_t ddi_width_60;
	uint16_t dp_to_tp;
	uint16_t tx_wait_const;
	uint16_t tx_wait_const_multi;
	uint16_t tp_to_dp;
	uint8_t phase_adc;
	uint8_t r2d_pw;
	uint8_t rst_pw;
	uint8_t rst_pw_back;
	uint8_t dac_td;
	uint8_t qsh_pw;
	uint8_t qsh_td;
	uint8_t drop_nodp;

	/* Output */
	uint32_t first_tp_width;
	uint32_t tp_width;
	uint32_t txpw;
	uint32_t long_tsdh_wait;
	uint32_t nodp;
};

struct core_mp_test_data {
	struct device *dev;
	struct mp_nodp_calc nodp;
	/* A flag shows a test run in particular */
	bool retry;

	int xch_len;
	int ych_len;
	int frame_len;
	int mp_items;

	int tdf;
	bool busy_cdc;
	bool ctrl_lcm;

	u8 version[4];
};

int ili9881h_mp_move_code(void);
int ili9881h_mp_register_sysfs(struct device *dev);

extern struct core_mp_test_data *core_mp;
extern struct mp_test_items tItems[];

#endif
