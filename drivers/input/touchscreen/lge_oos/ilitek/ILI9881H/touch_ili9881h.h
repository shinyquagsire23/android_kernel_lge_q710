/* touch_ili9881h.h
 *
 * Copyright (C) 2015 LGE.
 *
 * Author: PH1-BSP-Touch@lge.com
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

#ifndef LGE_TOUCH_ILI9881H_H
#define LGE_TOUCH_ILI9881H_H

#define CHIP_ID				0x9881
#define CHIP_TYPE			0x11
#define CORE_TYPE			0x03
#define CHIP_PID			0x98811103
#define TPD_WIDTH			2048

#define MAX_TOUCH_NUM			10

#define ILI9881_SLAVE_I2C_ADDR		0x41
#define ILI9881_ICE_MODE_ADDR		0x181062
#define ILI9881_PID_ADDR		0x4009C
#define ILI9881_WDT_ADDR		0x5100C
#define ILI9881_IC_RESET_ADDR		0x040050
#define ILI9881_IC_RESET_KEY		0x00019881
#define ILI9881_IC_RESET_GESTURE_ADDR	0x040054
#define ILI9881_IC_RESET_GESTURE_KEY	0xF38A94EF
#define ILI9881_IC_RESET_GESTURE_RUN	0xA67C9DFE

#define W_CMD_HEADER_SIZE		1
#define W_ICE_HEADER_SIZE		4
#define R_ICE_DATA_SIZE			4

#define CMD_NONE			0x00
#define CMD_GET_TP_INFORMATION		0x20
#define CMD_GET_FIRMWARE_VERSION	0x21
#define CMD_GET_PROTOCOL_VERSION	0x22
#define CMD_GET_CORE_VERSION		0x23
#define CMD_GET_MCU_INTERNAL_DATA	0x25
#define CMD_GET_KEY_INFORMATION		0x27
#define CMD_GET_IC_STATUS		0x2A
#define CMD_GET_TC_STATUS		0x2B
#define CMD_MODE_CONTROL		0xF0
#define CMD_SET_CDC_INIT		0xF1
#define CMD_GET_CDC_DATA		0xF2
#define CMD_CDC_BUSY_STATE		0xF3
#define CMD_READ_DATA_CTRL		0xF6
#define CMD_I2C_UART			0x40
#define CMD_ICE_MODE_EXIT		0x1B
#define CMD_READ_MP_TEST_CODE_INFO	0xFE

#define CMD_CONTROL_OPTION		0x01
#define CONTROL_SENSE			0x01
#define CONTROL_SLEEP			0x02
#define CONTROL_SCAN_RATE		0x03
#define CONTROL_CALIBRATION		0x04
#define CONTROL_INTERRUPT_OUTPUT	0x05
#define CONTROL_GLOVE			0x06
#define CONTROL_STYLUS			0x07
#define CONTROL_TP_SCAN_MODE		0x08
#define CONTROL_SW_RESET		0x09
#define CONTROL_LPWG			0x0A
#define CONTROL_GESTURE			0x0B
#define CONTROL_PHONE_COVER		0x0C
#define CONTROL_FINGER_SENSE		0x0F
#define CONTROL_PROXIMITY		0x10
#define CONTROL_PLUG			0x11
#define CONTROL_EDGE_PALM		0x12
#define CONTROL_LOCK_POINT		0x13

#define FIRMWARE_UNKNOWN_MODE		0xFF
#define FIRMWARE_DEMO_MODE		0x00
#define FIRMWARE_TEST_MODE		0x01
#define FIRMWARE_DEBUG_MODE		0x02
#define FIRMWARE_I2CUART_MODE		0x03
#define FIRMWARE_GESTURE_MODE		0x04

#define DEMO_PACKET_ID			0x5A
#define DEBUG_PACKET_ID			0xA7
#define TEST_PACKET_ID			0xF2
#define GESTURE_PACKET_ID		0xAA
#define TCI_FAILREASON_PACKET_ID	0xAB
#define SWIPE_FAILREASON_PACKET_ID	0xAD
#define I2CUART_PACKET_ID		0x7A

#define GESTURE_DOUBLECLICK		0x58
#define GESTURE_UP			0x60
#define GESTURE_DOWN			0x61
#define GESTURE_LEFT			0x62
#define GESTURE_RIGHT			0x63
#define GESTURE_M			0x64
#define GESTURE_W			0x65
#define GESTURE_C			0x66
#define GESTURE_E			0x67
#define GESTURE_V			0x68
#define GESTURE_O			0x69
#define GESTURE_S			0x6A
#define GESTURE_Z			0x6B

#define DEBUG_MODE_PACKET_LENGTH	2048 //1308
#define DEBUG_MODE_PACKET_HEADER_LENGTH	5
#define DEBUG_MODE_PACKET_FRAME_LIMIT	1024
#define TEST_MODE_PACKET_LENGTH		1180
#define GESTURE_MODE_PACKET_INFO_LENGTH          170
#define GESTURE_MODE_PACKET_NORMAL_LENGTH        8

#define CONNECT_NONE			(0x00)
#define CONNECT_USB			(0x01)
#define CONNECT_TA			(0x02)
#define CONNECT_OTG			(0x03)
#define CONNECT_WIRELESS		(0x10)

#define CHECK_EQUAL(X, Y) 		((X == Y) ? 0 : -1)
#define ERR_ALLOC_MEM(X)		((IS_ERR(X) || X == NULL) ? 1 : 0)
#define MIN(a,b)			(((a)<(b))?(a):(b))
#define MAX(a,b)			(((a)>(b))?(a):(b))

struct project_param {
	u8 touch_power_control_en;
	u8 touch_maker_id_control_en;
	u8 dsv_toggle;
	u8 deep_sleep_power_control;
	u8 dump_packet;

#if 0
	u8 chip_id;
	u8 protocol;
	u8 fw_cfg_use_en;
	u32 flash_fw_size;
	u8 used_mode;
	u8 osc_clock_ctrl;
	u8 tci_setting;
	u8 flex_report;
	u8 dynamic_tune_table;
	u8 palm_log;
#endif
};

enum {
	LCD_MODE_U0 = 0,
	LCD_MODE_U2_UNBLANK,
	LCD_MODE_U2,
	LCD_MODE_U3,
	LCD_MODE_U3_PARTIAL,
	LCD_MODE_U3_QUICKCOVER,
	LCD_MODE_STOP,
};

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
enum {
	MH4_BOE_ILI9881H = 0,
	MH4_BOE_HX83102D,
	MH4_CSOT_FT8736,
};
#endif

enum {
	FUNC_OFF = 0,
	FUNC_ON,
};

enum {
	IC_INIT_NEED = 0,
	IC_INIT_DONE,
};

enum {
	CHANGING_DISPLAY_MODE_READY = 0,
	CHANGING_DISPLAY_MODE_NOT_READY,
};

enum {
	DEBUG_IDLE = 0,
	DEBUG_GET_DATA_DONE,
	DEBUG_GET_DATA,
};

enum {
	SW_RESET = 0,
	HW_RESET_ASYNC,
	HW_RESET_SYNC,
	HW_RESET_ONLY,
};

enum {
	SWIPE_NONE = 0,
	SWIPE_UP = 1,
};

enum {
	IC_STATE_INIT = 0,
	IC_STATE_RESET,
	IC_STATE_WAKE,
	IC_STATE_SLEEP,
	IC_STATE_STANDBY,
};

enum {
	LPWG_FAILREASON_DISABLE = 0,
	LPWG_FAILREASON_ENABLE,
};

#if 0
enum {
	NEW_SAMPLE_TUNE_TABLE = 0,	// Board rev above 1.1
	OLD_SAMPLE_TUNE_TABLE,		// Board rev A, B, C, 1.0
};

enum {
	S_MODE_6LHB = 0,
	S_MODE_V_BLANK,
};

enum {
	TOUCHSTS_IDLE = 0,
	TOUCHSTS_DOWN,
	TOUCHSTS_MOVE,
	TOUCHSTS_UP,
};

enum {
	MODE_NONE = 0,
	MODE_NORMAL,
	MODE_NOISE,
	MODE_QUICKCOVER,
	MODE_IN_WATER,
	MODE_LPWG,
};

enum {
	KNOCK_ON,
	SWIPE_UP,
	SWIPE_DOWN,
	SWIPE_RIGHT,
	SWIPE_LEFT,
}

enum {
	KNOCK_1,
	KNOCK_2,
	SWIPE_UP,
	SWIPE_DOWN,
	SWIPE_RIGHT,
	SWIPE_LEFT,
};

enum {
	SWIPE_R = 0,
	SWIPE_D,
	SWIPE_L,
	SWIPE_U
};

// Vendor tmp -> Core
enum {
	SWIPE_ENABLE_CMD = 0,
	SWIPE_DISABLE_CMD,
	SWIPE_DIST_CMD,
	SWIPE_RATIO_THR_CMD,
	SWIPE_RATIO_PERIOD_CMD,
	SWIPE_RATIO_DIST_CMD,
	SWIPE_TIME_MIN_CMD,
	SWIPE_TIME_MAX_CMD,
	SWIPE_ACTIVE_AREA_CMD,
	SWIPE_WRONG_DIRECTION_THD_CMD,
	SWIPE_INIT_RATIO_CHK_DIST_CMD,
	SWIPE_INIT_RATIO_THD_CMD,
};

enum {
	E_FW_CODE_SIZE_ERR = 1,
	E_FW_CODE_ONLY_VALID = 2,
	E_FW_CODE_AND_CFG_VALID = 3,
	E_FW_CODE_CFG_ERR = 4
};

struct flash_table {
	u16 mid;
	u16 dev_id;
	int mem_size;
	int program_page;
	int sector;
	int block;
};

struct ili9881h_touch_debug_info {
	u32 info[3]; // ic_debug_info_addr : 0x23E ~ 0x240
	u32 type:24; // ic_debug_info_header_addr : 0x241
	u32 length:8;
} __packed;

/* report packet */
struct ili9881h_touch_data {
	u8 tool_type:4;
	u8 event:4;
	s8 track_id;
	u16 x;
	u16 y;
	u8 pressure;
	u8 angle;
	u16 width_major;
	u16 width_minor;
} __packed;

struct ili9881h_touch_info {
	u32 ic_status;
	u32 tc_status;
	u32 wakeup_type:8;
	u32 touch_cnt:5;
	u32 button_cnt:3;
	u32 current_mode:3;
	u32 palm_bit:13;
	struct ili9881h_touch_data data[10];	//120byte
} __packed;

struct ili9881h_tc_version {
	u8 minor_ver;
	u8 major_ver:4;
	u8 build_ver:4;
	u8 chip_id;
	u8 protocol_ver:4;
	u8 reverved:4;
} __packed;

struct ili9881h_pt_info {
	u32 lcm_type;
	u32 lot_num;
	u32 fpc_type;
	u16 pt_date_year;
	u8 pt_date_month;
	u8 pt_date_day;
	u8 pt_time_hour;
	u8 pt_time_min;
	u8 pt_time_sec;
	u8 pt_site;
	u32 chip_rev;
} __packed;

struct ili9881h_ic_info {
	struct ili9881h_tc_version version;
	u8 product_id[9];
	struct ili9881h_pt_info pt_info;
};
#endif

struct ili9881h_touch_abs_data {
	u8 y_high:4;
	u8 x_high:4;
	u8 x_low;
	u8 y_low;
	u8 pressure;
} __packed;
/*
struct ili9881h_touch_shape_data {
	s8 degree;
	u8 width_major_high;
	u8 width_major_low;
	u8 width_minor_high;
	u8 width_minor_low;
} __packed;
*/
/* report packet */
struct ili9881h_touch_info {
	u8 packet_id;
	struct ili9881h_touch_abs_data abs_data[10];
	u8 p_sensor:4;
	u8 key:4;
	//struct ili9881h_touch_shape_data shape_data[10];
	u8 checksum;
} __packed;

struct ili9881h_debug_info {
	u8 data[DEBUG_MODE_PACKET_LENGTH];
	u8 buf[DEBUG_MODE_PACKET_FRAME_LIMIT][2048];
	u16 frame_cnt;
	bool enable;
} __packed;

struct ili9881h_gesture_info {
	u8 data[GESTURE_MODE_PACKET_INFO_LENGTH];
} __packed;

struct ili9881h_chip_info {
	u32 id;
	u32 type;
	u32 core_type;
};

struct ili9881h_fw_info {
	u8 command_id;
	u8 core;
	u8 customer_code;
	u8 major;
	u8 minor;
	// not used below
	u8 mp_major_core;
	u8 mp_core;
	u8 release_code;
	u8 test_code;
} __packed;

struct ili9881h_protocol_info {
	u8 command_id;
	u8 major;
	u8 mid;
	u8 minor;
} __packed;

struct ili9881h_core_info {
	u8 command_id;
	u8 code_base; // for algorithm modify
	u8 minor;
	u8 revision_major; // for tunning parameter change
	u8 revision_minor;
} __packed;

struct ili9881h_ic_info {
	struct ili9881h_chip_info chip_info;
	struct ili9881h_fw_info fw_info;
	struct ili9881h_protocol_info protocol_info;
	struct ili9881h_core_info core_info;
};

struct ili9881h_tp_info {
	u8 command_id;
	u8 nMinX;
	u8 nMinY;
	u8 nMaxX_Low;
	u8 nMaxX_High;
	u8 nMaxY_Low;
	u8 nMaxY_High;
	u8 nXChannelNum;
	u8 nYChannelNum;
	u8 nMaxTouchPoint;
	u8 nTouchKeyNum;
	/* added for protocol v5 */
	u8 self_tx_channel_num;
	u8 self_rx_channel_num;
	u8 side_touch_type;
} __packed;

struct ili9881h_data {
	struct device *dev;
	struct kobject kobj;
	struct mutex io_lock;
	struct mutex apk_lock;
	struct mutex debug_lock;
	struct delayed_work fb_notify_work;

	struct project_param p_param;

	struct ili9881h_ic_info ic_info;
	struct ili9881h_tp_info tp_info;
	struct ili9881h_touch_info touch_info;
	struct ili9881h_debug_info debug_info;
	struct ili9881h_gesture_info gesture_info;

	atomic_t init;
	atomic_t reset;
	atomic_t changing_display_mode;
	wait_queue_head_t inq;

	u8 lcd_mode;
	u32 charger;
	u16 actual_fw_mode;
	u8 lpwg_failreason_ctrl;
	u8 err_cnt;

#if 0
	struct ili9881h_touch_debug_info debug_info;
	struct ili9881h_ic_info ic_info;

	u8 driving_mode;
	u8 intr_type;
	u8 lpwg_failreason_ctrl;
	u32 charger;
	u32 q_sensitivity;
	u8 err_cnt;
	u8 boot_err_cnt;
	u8 select_tune_table;

	u16 palm_new_mask;
	u16 palm_old_mask;
	int pcount;
#endif
};

#if 0
typedef union {
	struct {
		unsigned common_cfg_size : 16;
		unsigned specific_cfg_size : 16;
	} b;
	u32 w;
} t_cfg_size;

typedef union
{
	struct {
		unsigned chip_rev : 8;
		unsigned model_id : 8;
		unsigned lcm_id : 8;
		unsigned fpcb_id : 8;
	} b;
	u32 w;
} t_cfg_specific_info1;

typedef union
{
	struct {
		unsigned lot_id : 8;
		unsigned reserved : 24;
	} b;
	u32 w;
} t_cfg_specific_info2;

typedef struct
{
	t_cfg_specific_info1 cfg_specific_info1;/* 0x0000 */
	t_cfg_specific_info2 cfg_specific_info2;/* 0x0001 */
	u32 cfg_specific_version; 		/* 0x0002 */
	u32 cfg_model_name; 	   	/* 0x0003 */
	u32 cfg_header_reserved1; 		/* 0x0004 */
	u32 cfg_header_reserved2; 		/* 0x0005 */
	u32 cfg_header_reserved3; 		/* 0x0006 */
	u32 cfg_header_reserved4; 		/* 0x0007 */
}t_cfg_s_header_def;

typedef struct {
	u32 cfg_common_ver;
} t_cfg_c_header_def;

typedef struct
{
	u32 cfg_magic_code;	/* 0x0000 */
	u32 cfg_info_reserved0;	/* 0x0001 */
	u32 cfg_chip_id;		/* 0x0002 */
	u32 cfg_struct_version;	/* 0x0003 */
	u32 cfg_specific_cnt;	/* 0x0004 */
	t_cfg_size cfg_size;		/* 0x0005 */
	u32 cfg_global_date;	/* 0x0006 */
	u32 cfg_global_time;	/* 0x0007 */
	u32 cfg_info_reserved1;	/* 0x0008 */
	u32 cfg_info_reserved2;	/* 0x0009 */
	u32 cfg_info_reserved3;	/* 0x000A */
	u32 cfg_info_reserved4;	/* 0x000B */
}t_cfg_info_def;
#endif

extern int ili9811h_switch_fw_mode(struct device *dev, u8 mode);
extern int ili9881h_ice_mode_disable(struct device *dev);
extern int ili9881h_ice_mode_enable(struct device *dev);
extern int ili9881h_ice_reg_write(struct device *dev, u32 addr, u32 data, int size);
extern int ili9881h_ice_reg_read(struct device *dev, u32 addr);
extern int ili9881h_reg_read(struct device *dev, u8 cmd, void *data, int size);
extern int ili9881h_reg_write(struct device *dev, u8 cmd, void *data, int size);
extern int ili9881h_tp_info(struct device *dev);
extern int ili9881h_reset_ctrl(struct device *dev, int ctrl);
extern int ili9881h_check_cdc_busy(struct device *dev, int conut, int delay);
extern void ili9881h_dump_packet(void *data, int type, int len, int row_len, const char *name);
extern u8 ili9881h_calc_data_checksum(void *pMsg, int type, u32 nLength);
// int ili9881h_ic_info(struct device *dev);
// int ili9881h_tc_driving(struct device *dev, int mode);
// int ili9881h_irq_abs(struct device *dev);
// int ili9881h_irq_lpwg(struct device *dev);
// int ili9881h_irq_handler(struct device *dev);
// int ili9881h_check_status(struct device *dev);
// int ili9881h_debug_info(struct device *dev);

static inline struct ili9881h_data *to_ili9881h_data(struct device *dev)
{
	return (struct ili9881h_data *)touch_get_device(to_touch_core(dev));
}

static inline struct ili9881h_data *to_ili9881h_data_from_kobj(struct kobject *kobj)
{
	return (struct ili9881h_data *)container_of(kobj,
			struct ili9881h_data, kobj);
}
static inline int ili9881h_read_value(struct device *dev,
		u16 addr, u32 *value)
{
	return ili9881h_reg_read(dev, addr, value, sizeof(*value));
}

static inline int ili9881h_write_value(struct device *dev,
		u16 addr, u32 value)
{
	return ili9881h_reg_write(dev, addr, &value, sizeof(value));
}

static inline void ipio_kfree(void **mem)
{
	if(*mem != NULL) {
		kfree(*mem);
		*mem = NULL;
	}
}

/* extern */
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
extern int check_recovery_boot;
#endif

#if defined(CONFIG_LGE_MODULE_DETECT)
extern int panel_id;
#endif

/* Extern our apk api for LG */
extern int ili9881h_apk_init(struct device *dev);

#endif /* LGE_TOUCH_ILI9881H_H */
