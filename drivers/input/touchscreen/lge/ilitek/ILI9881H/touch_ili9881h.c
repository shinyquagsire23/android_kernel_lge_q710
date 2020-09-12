/* touch_ili9881h.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <soc/qcom/lge/board_lge.h>
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <soc/mediatek/lge/board_lge.h>
#endif
#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_ili9881h.h"
#include "touch_ili9881h_fw.h"
#include "touch_ili9881h_mp.h"

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <linux/msm_lcd_power_mode.h>
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <linux/lcd_power_mode.h>
#endif
#if defined(CONFIG_LGE_MODULE_DETECT)
int panel_id = 0;
#endif

static void project_param_set(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);

	/* incell touch driver have no authority to control vdd&vio power pin */
	d->p_param.touch_power_control_en = FUNC_OFF;

	/* TODO */
	d->p_param.touch_maker_id_control_en = FUNC_OFF;

	/* Use DSV Toggle mode */
	d->p_param.dsv_toggle = FUNC_OFF;

	/* Use Power control when deep sleep */
	d->p_param.deep_sleep_power_control = FUNC_ON;

	/* TODO */
	d->p_param.dump_packet = FUNC_ON;
}

#define LPWG_FAILREASON_TCI_NUM 8
static const char const *lpwg_failreason_tci_str[LPWG_FAILREASON_TCI_NUM] = {
	[0] = "SUCCESS",
	[1] = "DISTANCE_INTER_TAP",
	[2] = "DISTANCE_TOUCHSLOP",
	[3] = "TIMEOUT_INTER_TAP",
	[4] = "MULTI_FINGER",
	[5] = "DELAY_TIME", //Over Tap
	[6] = "PALM_STATE",
	[7] = "OUTOF_AREA",
};

#define LPWG_FAILREASON_SWIPE_NUM 8
static const char const *lpwg_failreason_swipe_str[LPWG_FAILREASON_SWIPE_NUM] = {
	[0] = "SUCCESS",
	[1] = "FINGER_FAST_RELEASE",
	[2] = "MULTI_FINGER",
	[3] = "FAST_SWIPE",
	[4] = "SLOW_SWIPE",
	[5] = "WRONG_DIRECTION",
	[6] = "RATIO_FAIL",
	[7] = "OUT_OF_AREA",
};

#define LCD_POWER_MODE_NUM 4
static const char const *lcd_power_mode_str[LCD_POWER_MODE_NUM] = {
	[0] = "DEEP_SLEEP_ENTER",
	[1] = "DEEP_SLEEP_EXIT",
	[2] = "DSV_TOGGLE",
	[3] = "DSV_ALWAYS_ON",
};

int ili9881h_reg_read(struct device *dev, u8 cmd, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->io_lock);
	switch (cmd) {
		case CMD_NONE:
			ts->tx_buf[0] = CMD_READ_DATA_CTRL;//this command is not used(dummy command)
			//ts->tx_buf[0] = 0xFF;

			msg.tx_buf = ts->tx_buf;
			msg.tx_size = 1;
			msg.rx_buf = ts->rx_buf;
			msg.rx_size = size;
			break;
		default:
			ts->tx_buf[0] = cmd;

			msg.tx_buf = ts->tx_buf;
			msg.tx_size = W_CMD_HEADER_SIZE;
			msg.rx_buf = ts->rx_buf;
			msg.rx_size = size;
			break;
	}

	ret = touch_bus_read(dev, &msg);
	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	memcpy(data, &ts->rx_buf[0], size);
	mutex_unlock(&d->io_lock);
	return 0;
}

int ili9881h_ice_reg_read(struct device *dev, u32 addr)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct touch_bus_msg msg;
	u32 data = 0;
	int ret = 0;

	mutex_lock(&d->io_lock);
	ts->tx_buf[0] = CMD_GET_MCU_INTERNAL_DATA;
	ts->tx_buf[1] = (char)((addr & 0x000000FF) >> 0);
	ts->tx_buf[2] = (char)((addr & 0x0000FF00) >> 8);
	ts->tx_buf[3] = (char)((addr & 0x00FF0000) >> 16);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_ICE_HEADER_SIZE;
	msg.rx_buf = ts->rx_buf;
	msg.rx_size = R_ICE_DATA_SIZE;

	ret = touch_bus_read(dev, &msg);
	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	data = (ts->rx_buf[0] | (ts->rx_buf[1] << 8) | (ts->rx_buf[2] << 16) | (ts->rx_buf[3] << 24));
	mutex_unlock(&d->io_lock);

	return data;
}

int ili9881h_reg_write(struct device *dev, u8 cmd, void *data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;

	mutex_lock(&d->io_lock);
	ts->tx_buf[0] = cmd;
	memcpy(&ts->tx_buf[W_CMD_HEADER_SIZE], data, size);

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_CMD_HEADER_SIZE + size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	mutex_unlock(&d->io_lock);

	return 0;
}

int ili9881h_ice_reg_write(struct device *dev, u32 addr, u32 data, int size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct touch_bus_msg msg;
	int ret = 0;
	int i;

	mutex_lock(&d->io_lock);
	ts->tx_buf[0] = CMD_GET_MCU_INTERNAL_DATA;
	ts->tx_buf[1] = (char)((addr & 0x000000FF) >> 0);
	ts->tx_buf[2] = (char)((addr & 0x0000FF00) >> 8);
	ts->tx_buf[3] = (char)((addr & 0x00FF0000) >> 16);
	//memcpy(&ts->tx_buf[W_ICE_HEADER_SIZE], data, size);
	for (i = 0; i < size; i++) {
		ts->tx_buf[W_ICE_HEADER_SIZE + i] = (char)(data >> (8 * i));
	}

	msg.tx_buf = ts->tx_buf;
	msg.tx_size = W_ICE_HEADER_SIZE + size;
	msg.rx_buf = NULL;
	msg.rx_size = 0;

	ret = touch_bus_write(dev, &msg);

	if (ret < 0) {
		TOUCH_E("touch bus error : %d\n", ret);
		mutex_unlock(&d->io_lock);
		return ret;
	}

	mutex_unlock(&d->io_lock);

	return 0;
}

int ili9881h_ice_mode_disable(struct device *dev)
{
	int ret = 0;
	u8 buf[3] = {0x62, 0x10, 0x18};

	TOUCH_I("ICE Mode disabled\n");

	ret = ili9881h_reg_write(dev, CMD_ICE_MODE_EXIT, &buf, 3);
	if (ret < 0) {
		TOUCH_E("ice_mode disable error : %d\n", ret);
	}

	return ret;
}

int ili9881h_ice_mode_enable(struct device *dev)
{
	int ret = 0;

	TOUCH_I("ICE Mode enabled\n");

	ret = ili9881h_ice_reg_write(dev, 0x181062, 0x0, 0);
	if (ret < 0) {
		TOUCH_E("ice_mode enable error : %d\n", ret);
	}

	return ret;
}

/* Print data for debug */
void ili9881h_dump_packet(void *data, int type, int len, int row_len, const char *name)
{
	int i, row = 32;
	u8 *p8 = NULL;
	int32_t *p32 = NULL;
	u8 buf[256] = {0, };
	int offset = 0;

	if (row_len > 0)
		row = row_len;

	if (data == NULL) {
		TOUCH_E("The data going to dump is NULL\n");
		return;
	}

	TOUCH_D(GET_DATA, "== Dump %s data ==\n", name);

	if (type == 8)
		p8 = (u8 *) data;
	if (type == 32 || type == 10)
		p32 = (int32_t *) data;

	for (i = 0; i < len; i++) {
		if (type == 8)
			offset += snprintf(buf + offset, sizeof(buf) - offset, "%2x ", p8[i]);
		else if (type == 32)
			offset += snprintf(buf + offset, sizeof(buf) - offset, "%4x ", p32[i]);
		else if (type == 10)
			offset += snprintf(buf + offset, sizeof(buf) - offset, "%4d ", p32[i]);

		if ((i % row) == row-1 || i == len - 1) {
			TOUCH_D(GET_DATA, "%s\n", buf);
			memset(buf, 0x0, sizeof(buf));
			offset = 0;
		}
	}
}

int ili9881h_sense_ctrl(struct device *dev, bool start)
{
	int ret = 0;
	u8 data[2] = {0};

	data[0] = CONTROL_SENSE;
	data[1] = start ? 1 : 0; // 0x0 : sensing stop, 0x1 : sensing start

	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));
	if (ret < 0)
		TOUCH_E("Write sense cmd error\n");

	TOUCH_I("%s - %s\n", __func__, data[1] ? "Sensing Start" : "Sensing Stop");

	return ret;
}

int ili9881h_sleep_ctrl(struct device *dev, bool start)
{
	int ret = 0;
	u8 data[2] = {0};

	data[0] = CONTROL_SLEEP;
	data[1] = start ? 3 : 1; // 0x0 : sleep in, 0x1 : sleep out, 0x3 : deep sleep in

	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));
	if (ret < 0)
		TOUCH_E("Write sleep cmd error\n");

	TOUCH_I("%s - %s\n", __func__, (data[1] == 1) ? "Sleep out" : "Deep Sleep in");

	return ret;
}

int ili9881h_plug_ctrl(struct device *dev, bool plugin)
{
	int ret = 0;
	u8 data[2] = {0};

	data[0] = CONTROL_PLUG;
	data[1] = plugin ? 0 : 1; // 0x0 : plug in, 0x1 : plug out

	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));
	if (ret < 0)
		TOUCH_E("Write plug cmd error\n");

	TOUCH_I("%s - %s\n", __func__, data[1] ? "Plug out" : "Plug in");

	return ret;
}

int ili9881h_gesture_ctrl(struct device *dev, int tci_mode, int swipe_mode)
{
	int ret = 0;
	u8 data[3] = {0, };

	data[0] = CONTROL_LPWG;
	data[1] = 0x08; // TCI / SWIPE enable
	data[2] = (tci_mode ? 0x1 : 0x0) | ((swipe_mode ? 0x1 : 0x0) << 1);

	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));

	TOUCH_I("%s - TCI : %s, SWIPE : %s\n", __func__,
			tci_mode ? "Enable" : "Disable", swipe_mode ? "Enable" : "Disable");

	return ret;
}

int ili9881h_gesture_failreason_ctrl(struct device *dev, int onoff)
{
	int ret = 0;
	u8 data[4] = {0, };

	data[0] = CONTROL_LPWG;
	data[1] = 0x10; // Failreason enable
	data[2] = 0x01; // Knock on mode
	data[3] = onoff ? 0xFF : 0x00;
	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));

	data[2] = 0x03; // Swipe mode
	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));

	TOUCH_I("%s - %s\n", __func__, onoff ? "Enable" : "Disable");

	return ret;
}

int ili9881h_gesture_mode(struct device *dev, bool enable)
{
	int ret = 0;
	u8 data[2] = {0};

	data[0] = CONTROL_LPWG;
	data[1] = enable ? 2 : 0; // 0x0: off, 0x2: info mode (108byte), 0x1: normal mode (8byte)

	ret = ili9881h_reg_write(dev, CMD_READ_DATA_CTRL, &data[0], 1);
	ret = ili9881h_reg_write(dev, CMD_CONTROL_OPTION, &data[0], sizeof(data));
	if (ret < 0)
		TOUCH_E("Write gesture cmd error\n");

	TOUCH_I("%s - %s\n", __func__, data[1] ? "Enable" : "Disable");

	return ret;
}

/* CDC = Capacitive to digital conversion */
int ili9881h_check_cdc_busy(struct device *dev, int count, int delay)
{
	int timer = count, ret = -1;
	u8 busy = 0, busy_byte = 0;
	struct ili9881h_data *d = to_ili9881h_data(dev);
	u8 buf = 0;

	if (d->actual_fw_mode == FIRMWARE_DEMO_MODE) {
		busy_byte = 0x41;
	} else if (d->actual_fw_mode == FIRMWARE_TEST_MODE) {
		busy_byte = 0x51;
	} else {
		TOUCH_E("Unknown fw mode (0x%x)\n", d->actual_fw_mode);
		return -EINVAL;
	}

	TOUCH_I("busy byte = %x\n", busy_byte);

	while (timer > 0) {
		buf = CMD_CDC_BUSY_STATE;
		ili9881h_reg_write(dev, CMD_READ_DATA_CTRL, &buf, 1);
		ili9881h_reg_read(dev, CMD_CDC_BUSY_STATE, &busy, 1);

		TOUCH_I("busy status = 0x%x\n", busy);

		if (busy == busy_byte) {
			TOUCH_I("Check busy is free\n");
			ret = 0;
			break;
		}
		timer--;
		touch_msleep(delay);
	}

	if (ret < -1)
		TOUCH_E("Check busy (0x%x) timeout\n", busy);

	return ret;
}

static int ili9881h_chip_init(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_chip_info *chip_info = &d->ic_info.chip_info;
	int ret = 0;
	uint32_t PIDData = 0;

	TOUCH_TRACE();

	ret = ili9881h_ice_mode_enable(dev);
	if (ret) {
		TOUCH_E("Failed to enter ICE mode, ret = %d\n", ret);
		goto out;
	}

	touch_msleep(20);

	/* Get Chip ID */
	PIDData = ili9881h_ice_reg_read(dev, ILI9881_PID_ADDR);

	chip_info->id = PIDData >> 16;
	chip_info->type = (PIDData & 0x0000FF00) >> 8;
	chip_info->core_type = PIDData & 0xFF;

	TOUCH_I("chipinfo.id = 0x%x, type = 0x%x, core = 0x%x\n",
			chip_info->id, chip_info->type, chip_info->core_type);

	if (chip_info->id != CHIP_ID || chip_info->type != CHIP_TYPE) {
		TOUCH_E("CHIP ID ERROR: 0x%x, Type = 0x%x\n", chip_info->id, chip_info->type);
		ret = -ENODEV;
		goto out;
	}

out:
	ili9881h_ice_mode_disable(dev);
	return ret;
}

static int ili9881h_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *ev = (struct fb_event *)data;

	if (ev && ev->data && event == FB_EVENT_BLANK) {
		int *blank = (int *)ev->data;
		if (*blank == FB_BLANK_UNBLANK)
			TOUCH_I("FB_UNBLANK\n");
		else if (*blank == FB_BLANK_POWERDOWN)
			TOUCH_I("FB_BLANK\n");
	}

	return 0;
}

static int ili9881h_sw_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;

	TOUCH_I("%s : SW Reset(mode%d)\n", __func__, mode);

	if(mode == SW_RESET) {
		// TODO do_ic_reset variable setting and i2c skip
		ili9881h_ice_reg_write(dev, ILI9881_IC_RESET_ADDR, ILI9881_IC_RESET_KEY, 4);
	} else {
		TOUCH_E("%s Invalid SW reset mode!!\n", __func__);
	}

	atomic_set(&d->init, IC_INIT_NEED);

	queue_delayed_work(ts->wq, &ts->init_work, msecs_to_jiffies(ts->caps.sw_reset_delay));

	return ret;
}

static int ili9881h_hw_reset(struct device *dev, int mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);

	TOUCH_I("%s : HW Reset(mode:%d)\n", __func__, mode);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	touch_gpio_direction_output(ts->reset_pin, 0);
	touch_msleep(5);
	touch_gpio_direction_output(ts->reset_pin, 1);

	atomic_set(&d->init, IC_INIT_NEED);

	if (mode == HW_RESET_ASYNC){
		queue_delayed_work(ts->wq, &ts->init_work, msecs_to_jiffies(ts->caps.hw_reset_delay));
	} else if (mode == HW_RESET_SYNC) {
		touch_msleep(ts->caps.hw_reset_delay);
		ts->driver->init(dev);
		touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	} else if (mode == HW_RESET_ONLY) {
		touch_msleep(ts->caps.hw_reset_delay);
		d->actual_fw_mode = FIRMWARE_DEMO_MODE;
		TOUCH_I("%s HW reset pin toggle only, need to call init func\n", __func__);
	} else {
		TOUCH_E("%s Invalid HW reset mode!!\n", __func__);
	}

	return 0;
}

int ili9881h_reset_ctrl(struct device *dev, int ctrl)
{
	TOUCH_TRACE();

	switch (ctrl) {
		default :
		case SW_RESET:
			ili9881h_sw_reset(dev, ctrl);
			break;

		case HW_RESET_ASYNC:
		case HW_RESET_SYNC:
		case HW_RESET_ONLY:
			ili9881h_hw_reset(dev, ctrl);
			break;
	}

	return 0;
}

int ili9881h_display_power_ctrl(struct device *dev, int ctrl)
{
	TOUCH_TRACE();

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
	lge_panel_set_power_mode(ctrl);
#elif defined(CONFIG_LGE_TOUCH_CORE_MTK)
	primary_display_set_power_mode(ctrl);
#endif

	if (ctrl >= DEEP_SLEEP_ENTER && ctrl <= DSV_ALWAYS_ON) {
		TOUCH_I("%s, mode : %s\n", __func__, lcd_power_mode_str[ctrl]);
	}

	return 0;
}

static int ili9881h_power(struct device *dev, int ctrl)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	TOUCH_TRACE();

	switch (ctrl) {
		case POWER_OFF:
			if(d->p_param.touch_power_control_en == FUNC_ON) {
				atomic_set(&d->init, IC_INIT_NEED);
				touch_gpio_direction_output(ts->reset_pin, 0);
				touch_power_vio(dev, 0);
				touch_power_vdd(dev, 0);
				touch_msleep(1);
			} else {
				TOUCH_I("%s, off Not Supported\n", __func__);
			}
			break;

		case POWER_ON:
			if(d->p_param.touch_power_control_en == FUNC_ON) {
				touch_power_vdd(dev, 1);
				touch_power_vio(dev, 1);
				touch_gpio_direction_output(ts->reset_pin, 1);
			} else {
				TOUCH_I("%s, on Not Supported\n", __func__);
			}
			break;
		case POWER_HW_RESET:
			TOUCH_I("%s, reset\n", __func__);
			ili9881h_reset_ctrl(dev, HW_RESET_ASYNC);
			break;
		case POWER_SW_RESET:
			ili9881h_reset_ctrl(dev, SW_RESET);
			break;
		case POWER_SLEEP:
			if (d->p_param.deep_sleep_power_control == FUNC_ON) {
				ili9881h_display_power_ctrl(dev, DEEP_SLEEP_ENTER);
			}
			break;
		case POWER_WAKE:
			if (d->p_param.deep_sleep_power_control == FUNC_ON) {
				ili9881h_display_power_ctrl(dev, DEEP_SLEEP_EXIT);
			}
			break;
		case POWER_DSV_TOGGLE:
			if (d->p_param.dsv_toggle == FUNC_ON) {
				ili9881h_display_power_ctrl(dev, DSV_TOGGLE);
			}
			break;
		case POWER_DSV_ALWAYS_ON:
			if (d->p_param.dsv_toggle == FUNC_ON) {
				ili9881h_display_power_ctrl(dev, DSV_ALWAYS_ON);
			}
			break;
		default:
			TOUCH_I("%s, Not Supported. case: %d\n", __func__, ctrl);
			break;
	}

	return 0;
}

static void ili9881h_get_tci_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	ts->tci.info[TCI_1].tap_count = 2;
	ts->tci.info[TCI_1].min_intertap = 6;
	ts->tci.info[TCI_1].max_intertap = 70;
	ts->tci.info[TCI_1].touch_slop = 100;
	ts->tci.info[TCI_1].tap_distance = 10;
	ts->tci.info[TCI_1].intr_delay = 0;

	ts->tci.info[TCI_2].min_intertap = 6;
	ts->tci.info[TCI_2].max_intertap = 70;
	ts->tci.info[TCI_2].touch_slop = 100;
	ts->tci.info[TCI_2].tap_distance = 255;
	ts->tci.info[TCI_2].intr_delay = 20;
}

#if 0
static void ili9881h_get_swipe_info(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int mm_to_point = 15; // 1 mm -> about X point

	ts->swipe.info.distance = 20;
	ts->swipe.info.ratio_thres = 150;
	ts->swipe.info.min_time = 4;
	ts->swipe.info.max_time = 150;
	ts->swipe.info.area.x1 = 0 + (mm_to_point * 4);					// spec 4mm
	ts->swipe.info.area.y1 = 0;							// spec 0mm
	ts->swipe.info.area.x2 = ts->caps.max_x - (mm_to_point * 4);			// spec 4mm
	ts->swipe.info.area.y2 = ts->caps.max_y;					// spec 0mm
	ts->swipe.info.start.x1 = (ts->caps.max_x / 2) - (int)(mm_to_point * 12.5);	// spec start_area_width 25mm
	ts->swipe.info.start.y1 = ts->swipe.info.area.y2 - (int)(mm_to_point * 14.5);	// spec start_area_height 14.5mm
	ts->swipe.info.start.x2 = (ts->caps.max_x / 2) + (int)(mm_to_point * 12.5);	// spec start_area_width 25mm
	ts->swipe.info.start.y2 = ts->swipe.info.area.y2;
	ts->swipe.info.wrong_dir_thes = 5;
	ts->swipe.info.init_rat_chk_dist = 4;
	ts->swipe.info.init_rat_thres = 100;

	ts->swipe.mode = 0;
}
#endif

int ili9881h_tp_info(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_tp_info *tp_info = &d->tp_info;
	int ret = 0;
	u8 buf = CMD_GET_TP_INFORMATION;

	memset(tp_info, 0x0, sizeof(*tp_info));

	ret = ili9881h_reg_write(dev, CMD_READ_DATA_CTRL, &buf, 1);
	if (ret) {
		TOUCH_E("Failed to write preparation command, %d\n", ret);
		goto out;
	}

	/* GEt TP info */
	ret = ili9881h_reg_read(dev, CMD_GET_TP_INFORMATION, tp_info, sizeof(*tp_info));
	if (ret) {
		TOUCH_E("Failed to read touch panel information, %d\n", ret);
		goto out;
	}

	TOUCH_I("==================== TouchPanel Info ====================\n");

	TOUCH_I("minX = %d, minY = %d, maxX = %d, maxY = %d\n",
		 tp_info->nMinX, tp_info->nMinY,
		 ((u16)tp_info->nMaxX_High << 8) | (tp_info->nMaxX_Low),
		 ((u16)tp_info->nMaxY_High << 8) | (tp_info->nMaxY_Low));
	TOUCH_I("xChannel = %d, yChannel = %d, self_tx = %d, self_rx = %d\n",
		 tp_info->nXChannelNum, tp_info->nYChannelNum,
		 tp_info->self_tx_channel_num, tp_info->self_rx_channel_num);
	TOUCH_I("side_touch_type = %d, max_touch_point= %d, max_key_num = %d\n",
		 tp_info->side_touch_type, tp_info->nMaxTouchPoint,
		 tp_info->nTouchKeyNum);
	TOUCH_I("======================================================\n");

out:
	return ret;
}
int ili9881h_ic_info(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_fw_info *fw_info = &d->ic_info.fw_info;
	struct ili9881h_protocol_info *protocol_info = &d->ic_info.protocol_info;
	struct ili9881h_core_info *core_info = &d->ic_info.core_info;
	int ret = 0;

	memset(&d->ic_info, 0, sizeof(d->ic_info));

	/* Get Chip info */
	ret = ili9881h_chip_init(dev);
	if (ret) {
		TOUCH_E("Failed to get chip info, %d\n", ret);
		goto error;
	}

	/* Get Fw info */
	ret = ili9881h_reg_read(dev, CMD_GET_FIRMWARE_VERSION, fw_info, sizeof(*fw_info));
	if (ret) {
		TOUCH_E("Failed to read firmware version, %d\n", ret);
		goto error;
	}

	/* Get Protocol ver */
	ret = ili9881h_reg_read(dev, CMD_GET_PROTOCOL_VERSION, protocol_info, sizeof(*protocol_info));
	if (ret) {
		TOUCH_E("Failed to read protocol version, %d\n", ret);
		goto error;
	}

	/* Get FW core ver */
	ret = ili9881h_reg_read(dev, CMD_GET_CORE_VERSION, core_info, sizeof(core_info));
	if (ret) {
		TOUCH_E("Failed to read core version, %d\n", ret);
		goto error;
	}

	TOUCH_I("==================== Version Info ====================\n");
	TOUCH_I("Version = v%d.%02d, fw_core.customer_id = %d.%d\n",
			fw_info->major, fw_info->minor, fw_info->core, fw_info->customer_code);
	TOUCH_I("Procotol Version = %d.%d.%d\n",
			protocol_info->major, protocol_info->mid, protocol_info->minor);
	TOUCH_I("Core Version = %d.%d.%d.%d\n",
			core_info->code_base, core_info->minor, core_info->revision_major, core_info->revision_minor); 
	TOUCH_I("======================================================\n");

	// TODO fw version check (0.00 or F.FF) and goto error

	return ret;

error:
	if(d->err_cnt > 5) {
		TOUCH_I("CHIP_ID or CHIP_TYPE wrong. Retry Over (cnt:%d)\n", d->err_cnt);
	} else {
		TOUCH_I("CHIP_ID or CHIP_TYPE wrong. (cnt:%d)\n", d->err_cnt);
		ili9881h_reset_ctrl(dev, HW_RESET_ASYNC);
		d->err_cnt++;
	}

	return ret;
}

#ifdef LG_KNOCK_CODE
static int ili9881h_tci_password(struct device *dev)
{
	return ili9881h_tci_knock(dev);
}
#endif

static int ili9881h_lpwg_control(struct device *dev, int tci_mode, int swipe_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;
	char* tci_mode_str[4] = {"Lpwg_None", "Knock-On", "Knock-On/Code", "Knock-Code"};
	char* swipe_mode_str[2] = {"Swipe_None", "Swipe-Up"};

	if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
		TOUCH_I("Skip tci control in deep sleep\n");
		return 0;
	}

	if ((tci_mode >= LPWG_NONE && tci_mode <= LPWG_PASSWORD_ONLY) &&
			(swipe_mode >= SWIPE_NONE && swipe_mode <= SWIPE_UP)) {
		TOUCH_I("ili9881h_lpwg_control - tci_mode = %s, swipe_mode = %s\n",
				tci_mode_str[tci_mode], swipe_mode_str[swipe_mode]);
	}

	if (tci_mode || swipe_mode) {
		ret = ili9881h_gesture_mode(dev, true);
		ret = ili9881h_gesture_ctrl(dev, tci_mode, swipe_mode);
		ret = ili9881h_gesture_failreason_ctrl(dev, d->lpwg_failreason_ctrl);
	} else {
		ret = ili9881h_gesture_mode(dev, false);
	}

	return ret;
}

#if 0
static int ili9881h_swipe_active_area(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 active_area[4] = {0, };
	u32 start_area[4] = {0, };
	int ret = 0;

	// set by ili9881h_get_swipe_info func
	active_area[0] = ts->swipe.info.area.x1;
	active_area[1] = ts->swipe.info.area.y1;
	active_area[2] = ts->swipe.info.area.x2;
	active_area[3] = ts->swipe.info.area.y2;
	start_area[0] = ts->swipe.info.start.x1;
	start_area[1] = ts->swipe.info.start.y1;
	start_area[2] = ts->swipe.info.start.x2;
	start_area[3] = ts->swipe.info.start.y2;

	TOUCH_I("%s - active_area: x1[%d], y1[%d], x2[%d], y2[%d]\n", __func__,
			active_area[0], active_area[1], active_area[2], active_area[3]);
	TOUCH_I("%s - start_area:  x1[%d], y1[%d], x2[%d], y2[%d]\n", __func__,
			start_area[0], start_area[1], start_area[2], start_area[3]);

	ret = ili9881h_reg_write(dev, SWIPE_ACTIVE_AREA_X1_CTRL, active_area, sizeof(active_area));
	ret = ili9881h_reg_write(dev, SWIPE_START_AREA_X1_CTRL, start_area, sizeof(start_area));

	return ret;
}

static int ili9881h_swipe_control(struct device *dev, u8 lcd_mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	u32 swipe_data[5] = {0x0, };
	u32 swipe_data_2[3] = {0x0, };
	int ret = 0;

	if (!ts->swipe.mode)
		return ret;


	if (lcd_mode == LCD_MODE_U3) {
		TOUCH_I("%s swipe mode = %s\n", __func__,  "Disable");
		ret = ili9881h_swipe_command(dev, SWIPE_DISABLE_CMD);
	} else {
		TOUCH_I("%s swipe mode = %s\n", __func__,  "Enable");

		swipe_data[0] = ts->swipe.mode;
		swipe_data[1] = ts->swipe.info.distance;
		swipe_data[2] = ts->swipe.info.ratio_thres;
		swipe_data[3] = ts->swipe.info.min_time;
		swipe_data[4] = ts->swipe.info.max_time;
		ret = ili9881h_reg_write(dev, SWIPE_ON_CTRL, &swipe_data[0], sizeof(swipe_data));

		swipe_data_2[0] = ts->swipe.info.wrong_dir_thes;
		swipe_data_2[1] = ts->swipe.info.init_rat_chk_dist;
		swipe_data_2[2] = ts->swipe.info.init_rat_thres;
		ret = ili9881h_reg_write(dev, SWIPE_WRONG_DIRECTION_THD_CTRL, &swipe_data_2[0], sizeof(swipe_data_2));

		ret = ili9881h_swipe_command(dev, SWIPE_ACTIVE_AREA_CMD);
	}

	return ret;
}
#endif

static int ili9881h_deep_sleep_ctrl(struct device *dev, int state)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;
	int timeout = 20;
	int value = 0;

	if (!atomic_read(&ts->state.incoming_call)) { /* IDLE status */
		if (state == IC_DEEP_SLEEP) { // IC_DEEP_SLEEP
			if (d->actual_fw_mode != FIRMWARE_GESTURE_MODE) {
				ili9881h_sense_ctrl(dev, false);
				ili9881h_check_cdc_busy(dev, 10, 50);
			}
			ili9881h_sleep_ctrl(dev, true);


			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);

			ili9881h_power(dev, POWER_SLEEP); // Reset Low
			d->actual_fw_mode = FIRMWARE_UNKNOWN_MODE;
		} else { // IC_NORMAL
			ili9881h_power(dev, POWER_WAKE); // Reset High
			/* Gesture can be reentered successfully when the status of proximity turns FAR from NEAR,
			 * 1. Write a speical key to a particular register.
			 * 2. FW saw the register modified, gesture should be enable by itself after did TP Reset.
			 * 3. Polling another specific register to see if gesutre is enabled properly. */
			ili9881h_ice_mode_enable(dev);
			ili9881h_ice_reg_write(dev, ILI9881_IC_RESET_GESTURE_ADDR,
					ILI9881_IC_RESET_GESTURE_KEY, sizeof(u32));
			ili9881h_reset_ctrl(dev, HW_RESET_ONLY);

			touch_msleep(30);

			while (timeout > 0) {
				ili9881h_ice_mode_enable(dev);
				value = ili9881h_ice_reg_read(dev, ILI9881_IC_RESET_GESTURE_ADDR);
				ili9881h_ice_mode_disable(dev);
				if (value == ILI9881_IC_RESET_GESTURE_RUN) {
					TOUCH_I("Gesture mode set successfully\n");
					break;
				}
				TOUCH_I("Waiting gesture mode - value = 0x%x\n", value);
				touch_msleep(50);
				timeout--;
			}
			if (timeout <= 0) {
				TOUCH_I("Gesture mode set failed\n");
			}

			touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
			atomic_set(&d->init, IC_INIT_DONE);
			atomic_set(&ts->state.sleep, IC_NORMAL);
			d->actual_fw_mode = FIRMWARE_GESTURE_MODE;
		}

		TOUCH_I("%s - %s\n", __func__, (state == IC_NORMAL) ? "Lpwg Mode" : "Deep Sleep");
	} else { /* RINGING or OFFHOOK status */
		if (state == IC_DEEP_SLEEP) { // IC_DEEP_SLEEP
			if (d->actual_fw_mode != FIRMWARE_GESTURE_MODE) {
				ili9881h_sense_ctrl(dev, false);
				ili9881h_check_cdc_busy(dev, 10, 50);
			}
			ili9881h_sleep_ctrl(dev, true);

			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			atomic_set(&ts->state.sleep, IC_DEEP_SLEEP);
			d->actual_fw_mode = FIRMWARE_UNKNOWN_MODE;
		} else { // IC_NORMAL
			touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
			atomic_set(&ts->state.sleep, IC_NORMAL);
			ili9881h_lpwg_control(dev, ts->lpwg.mode, ts->swipe.mode);
			d->actual_fw_mode = FIRMWARE_GESTURE_MODE;
		}
		TOUCH_I("%s - %s But avoid deep sleep Power Sequence during Call\n",
				__func__,  (state == IC_NORMAL) ? "Lpwg Mode" : "Deep Sleep");
	}

	return ret;
}

static int ili9881h_lpwg_mode(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&d->init) == IC_INIT_NEED) {
		TOUCH_I("Skip lpwg mode in IC_INIT_NEED status\n");
		return 0;
	}

	if (atomic_read(&d->reset) == LCD_EVENT_TOUCH_RESET_START) {
		TOUCH_I("Skip lpwg mode during ic reset\n");
		return 0;
	}

	if (atomic_read(&d->changing_display_mode) == CHANGING_DISPLAY_MODE_NOT_READY) {
		TOUCH_I("Skip lpwg mode during changing display mode\n");
		return 0;
	}

	if (atomic_read(&ts->state.fb) == FB_SUSPEND) {
		if (ts->role.mfts_lpwg) {
			ili9881h_sense_ctrl(dev, false);
			ili9881h_check_cdc_busy(dev, 10, 50);
			ili9881h_lpwg_control(dev, LPWG_DOUBLE_TAP, SWIPE_NONE);
			d->actual_fw_mode = FIRMWARE_GESTURE_MODE;
			return 0;
		}

		if (ts->lpwg.screen) {
			TOUCH_I("Skip lpwg setting\n");
			ili9881h_power(dev, POWER_DSV_ALWAYS_ON);
		} else if (ts->lpwg.sensor == PROX_NEAR) {
			/* deep sleep */
			TOUCH_I("suspend sensor == PROX_NEAR\n");
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP)
				ret = ili9881h_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		} else if (ts->lpwg.qcover == HALL_NEAR) {
			/* Deep Sleep same as Prox near  */
			TOUCH_I("Qcover == HALL_NEAR\n");
			if (atomic_read(&ts->state.sleep) != IC_DEEP_SLEEP)
				ret = ili9881h_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		} else if (ts->swipe.mode == SWIPE_NONE && ts->lpwg.mode == LPWG_NONE
				&& d->lcd_mode == LCD_MODE_U0) {
			/* knock on/code disable */
			TOUCH_I("Knock-on mode == LPWG_NONE & Swipe mode == SWIPE_NONE & lcd mode == LCD_MODE_U0\n");
			ret = ili9881h_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		} else {
			/* knock on/code */
			if (atomic_read(&ts->state.sleep) == IC_DEEP_SLEEP) {
				ret = ili9881h_deep_sleep_ctrl(dev, IC_NORMAL);
			} else {
				TOUCH_I("knock on setting\n");
				ili9881h_sense_ctrl(dev, false);
				ili9881h_check_cdc_busy(dev, 10, 50);
				ili9881h_lpwg_control(dev, ts->lpwg.mode, ts->swipe.mode);
				d->actual_fw_mode = FIRMWARE_GESTURE_MODE;
				if(d->p_param.dsv_toggle == FUNC_ON) {
					ili9881h_power(dev, POWER_DSV_TOGGLE);
				}
			}

		}
		return ret;
	}

	touch_report_all_event(ts);

	/* resume */
	if (ts->lpwg.screen) {
		/* normal */
		TOUCH_I("resume ts->lpwg.screen on\n");
	} else if (ts->lpwg.sensor == PROX_NEAR) {
		TOUCH_I("resume ts->lpwg.sensor == PROX_NEAR\n");
	} else {
		/* partial */
		TOUCH_I("resume Partial\n");
	}

	return ret;
}

static int ili9881h_lpwg(struct device *dev, u32 code, void *param)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int *value = (int *)param;
	int ret = 0;

	switch (code) {
		case LPWG_TAP_COUNT:
			ts->tci.info[TCI_2].tap_count = value[0];
			break;

		case LPWG_DOUBLE_TAP_CHECK:
			ts->tci.double_tap_check = value[0];
			break;

		case LPWG_UPDATE_ALL:
			TOUCH_I("LPWG_UPDATE_ALL: mode[%d], screen[%s], sensor[%s], qcover[%s]\n",
					value[0],
					value[1] ? "ON" : "OFF",
					value[2] ? "FAR" : "NEAR",
					value[3] ? "CLOSE" : "OPEN");

			if (ts->lpwg.screen != value[1] &&
					atomic_read(&d->changing_display_mode) == CHANGING_DISPLAY_MODE_READY) {
				atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_NOT_READY);
				TOUCH_I("Skip lpwg mode - START\n");
			}

			ts->lpwg.mode = value[0];
			ts->lpwg.screen = value[1];
			ts->lpwg.sensor = value[2];
			ts->lpwg.qcover = value[3];

			ret = ili9881h_lpwg_mode(dev);
			if (ret)
				TOUCH_E("failed to lpwg_mode, ret:%d", ret);
			break;

		case LPWG_REPLY:
			break;

		default:
			TOUCH_I("%s - Unknown Lpwg Code : %d\n", __func__, code);
			break;
	}

	return ret;
}

static void ili9881h_connect(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int charger_state = atomic_read(&ts->state.connect);
	int wireless_state = atomic_read(&ts->state.wireless);
	int val = 0;

	TOUCH_TRACE();

	d->charger = 0;
	/* wire */
	if (charger_state == CONNECT_INVALID)
		d->charger = CONNECT_NONE;
	else if ((charger_state == CONNECT_DCP)	|| (charger_state == CONNECT_PROPRIETARY))
		d->charger = CONNECT_TA;
	else if (charger_state == CONNECT_HUB)
		d->charger = CONNECT_OTG;
	else
		d->charger = CONNECT_USB;

	/* wireless */
	if (wireless_state)
		d->charger = d->charger | CONNECT_WIRELESS;

	TOUCH_I("%s: write charger_state = 0x%02X\n", __func__, d->charger);
	if (atomic_read(&ts->state.pm) > DEV_PM_RESUME) {
		TOUCH_I("DEV_PM_SUSPEND - Don't try I2C\n");
		return;
	}

	val = d->charger ? true : false;
	ili9881h_plug_ctrl(dev, val);
}

static void ili9881h_lcd_mode(struct device *dev, u32 mode)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);

	TOUCH_I("lcd_mode: %d (prev: %d)\n", mode, d->lcd_mode);

	d->lcd_mode = mode;
}

static int ili9881h_usb_status(struct device *dev, u32 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("TA Type: %d\n", atomic_read(&ts->state.connect));
	ili9881h_connect(dev);
	return 0;
}

static int ili9881h_wireless_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Wireless charger: 0x%02X\n", atomic_read(&ts->state.wireless));
	ili9881h_connect(dev);
	return 0;
}

static int ili9881h_earjack_status(struct device *dev, u32 onoff)
{
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();
	TOUCH_I("Earjack Type: 0x%02X\n", atomic_read(&ts->state.earjack));
	return 0;
}

static void ili9881h_fb_notify_work_func(struct work_struct *fb_notify_work)
{
	struct ili9881h_data *d = container_of(to_delayed_work(fb_notify_work),
			struct ili9881h_data, fb_notify_work);
	int ret = 0;

	if (d->lcd_mode == LCD_MODE_U0 || d->lcd_mode == LCD_MODE_U2)
		ret = FB_SUSPEND;
	else
		ret = FB_RESUME;

	touch_notifier_call_chain(NOTIFY_FB, &ret);
}

static int ili9881h_notify_charger(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
		case LCD_EVENT_TOUCH_RESET_START:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");

			atomic_set(&d->reset, event);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
			break;

		case LCD_EVENT_TOUCH_RESET_END:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");

			atomic_set(&d->reset, event);
			touch_gpio_direction_output(ts->reset_pin, 1);
			break;

		default:
			TOUCH_E("%lu is not supported in charger mode\n", event);
			break;
	}

	return ret;
}

static int ili9881h_notify_normal(struct device *dev, ulong event, void *data)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;

	TOUCH_TRACE();

	switch (event) {
		case NOTIFY_TOUCH_RESET:
			if(atomic_read(&ts->state.debug_option_mask) & DEBUG_OPTION_1)
				ret = 1;
			else
				ret = 0;
			TOUCH_I("NOTIFY_TOUCH_RESET! return = %d\n", ret);
			break;

		case LCD_EVENT_TOUCH_RESET_START:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_START!\n");

			atomic_set(&d->reset, event);
			touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);
			touch_gpio_direction_output(ts->reset_pin, 0);
			break;

		case LCD_EVENT_TOUCH_RESET_END:
			TOUCH_I("LCD_EVENT_TOUCH_RESET_END!\n");

			atomic_set(&d->reset, event);
			touch_gpio_direction_output(ts->reset_pin, 1);
			break;

		case LCD_EVENT_LCD_MODE:
			TOUCH_I("LCD_EVENT_LCD_MODE!\n");
			ili9881h_lcd_mode(dev, *(u32 *)data);
			queue_delayed_work(ts->wq, &d->fb_notify_work, 0);
			break;

		case NOTIFY_CONNECTION:
			TOUCH_I("NOTIFY_CONNECTION!\n");
			ret = ili9881h_usb_status(dev, *(u32 *)data);
			break;

		case NOTIFY_WIRELEES:
			TOUCH_I("NOTIFY_WIRELEES!\n");
			ret = ili9881h_wireless_status(dev, *(u32 *)data);
			break;

		case NOTIFY_EARJACK:
			TOUCH_I("NOTIFY_EARJACK!\n");
			ret = ili9881h_earjack_status(dev, *(u32 *)data);
			break;

		case NOTIFY_IME_STATE:
			TOUCH_I("NOTIFY_IME_STATE!\n");
			//ret = ili9881h_reg_write(dev, SPECIAL_IME_STATUS_CTRL, (u32*)data, sizeof(u32));
			break;

		case NOTIFY_CALL_STATE:
			/* Notify Touch IC only for GSM call and idle state */
			if (*(u32*)data >= INCOMING_CALL_IDLE && *(u32*)data <= INCOMING_CALL_LTE_OFFHOOK) {
				TOUCH_I("NOTIFY_CALL_STATE!\n");
				//ret = ili9881h_reg_write(dev, SPECIAL_CALL_INFO_CTRL, (u32*)data, sizeof(u32));
			}
			break;

		case NOTIFY_QMEMO_STATE:
			TOUCH_I("NOTIFY_QMEMO_STATE!\n");
			break;

		default:
			TOUCH_E("%lu is not supported\n", event);
			break;
	}

	return ret;
}

static int ili9881h_notify(struct device *dev, ulong event, void *data)
{
	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		TOUCH_I("CHARGER MODE notify\n");
		return ili9881h_notify_charger(dev,event,data);
	}

	return ili9881h_notify_normal(dev,event,data);
}

static void ili9881h_init_works(struct ili9881h_data *d)
{
	INIT_DELAYED_WORK(&d->fb_notify_work, ili9881h_fb_notify_work_func);
	init_waitqueue_head(&(d->inq));
}

static void ili9881h_init_locks(struct ili9881h_data *d)
{
	mutex_init(&d->io_lock);
	mutex_init(&d->apk_lock);
	mutex_init(&d->debug_lock);
}

static int ili9881h_probe(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = NULL;
	int ret = 0;

	TOUCH_TRACE();

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);

	if (!d) {
		TOUCH_E("failed to allocate ic data\n");
		return -ENOMEM;
	}

	d->dev = dev;
	touch_set_device(ts, d);

	project_param_set(dev);

	ret = touch_gpio_init(ts->reset_pin, "touch_reset");
	if (ret) {
		TOUCH_E("failed to touch gpio init\n");
		return ret;
	}

	ret = touch_gpio_direction_output(ts->reset_pin, 1);
	if (ret) {
		TOUCH_E("failed to touch gpio direction output\n");
		return ret;
	}

	ret = touch_gpio_init(ts->int_pin, "touch_int");
	if (ret) {
		TOUCH_E("failed to touch gpio init\n");
		return ret;
	}
	ret = touch_gpio_direction_input(ts->int_pin);
	if (ret) {
		TOUCH_E("failed to touch gpio direction input\n");
		return ret;
	}

	if(d->p_param.touch_maker_id_control_en == FUNC_ON) {
		ret = touch_gpio_init(ts->maker_id_pin, "touch_make_id");
		if (ret) {
			TOUCH_E("failed to touch gpio init\n");
			return ret;
		}
		ret = touch_gpio_direction_input(ts->maker_id_pin);
		if (ret) {
			TOUCH_E("failed to touch gpio direction input\n");
			return ret;
		}
	}
	/*******************************************************
	 * Display driver does control the power in ili9881h IC *
	 * due to its design from INCELL 1-chip. Here we skip  *
	 * the control power.                                  *
	 *******************************************************/
	if(d->p_param.touch_power_control_en == FUNC_ON) {
		ret = touch_power_init(dev);
		if (ret) {
			TOUCH_E("failed to touch power init\n");
			return ret;
		}
	}

	ret = touch_bus_init(dev, MAX_XFER_BUF_SIZE);
	if (ret) {
		TOUCH_E("failed to touch bus init\n");
		return ret;
	}

	ili9881h_init_works(d);
	ili9881h_init_locks(d);

	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		ret = touch_gpio_init(ts->reset_pin, "touch_reset");
		if (ret) {
			TOUCH_E("failed to touch gpio init\n");
			return ret;
		}
		ret = touch_gpio_direction_output(ts->reset_pin, 1);
		if (ret) {
			TOUCH_E("failed to touch gpio direction output\n");
			return ret;
		}
		/* Deep Sleep */
		ret = ili9881h_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		if (ret) {
			TOUCH_E("failed to deep sleep ctrl\n");
			return ret;
		}
		return ret;
	}

	ili9881h_reset_ctrl(dev, HW_RESET_ONLY);

	/* Get flash size info */
	ret = ili9881h_read_flash_info(dev, 0x9F, 4);
	if (ret) {
		TOUCH_E("failed to read flash info\n");
		return ret;
	}

	/* Read touch panel information */
	ret = ili9881h_tp_info(dev);
	if (ret) {
		TOUCH_E("failed to read touch panel info\n");
		return ret;
	}

	/* Initialize knock on tuning parameter info */
	ili9881h_get_tci_info(dev);

	/* Initialize ILITEK's APK for fw debug */
	ili9881h_apk_init(dev);

	d->actual_fw_mode = FIRMWARE_DEMO_MODE;
	d->err_cnt = 0;
	d->lcd_mode = LCD_MODE_U3;
	d->lpwg_failreason_ctrl = LPWG_FAILREASON_ENABLE;

	return ret;
}

static int ili9881h_remove(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int ili9881h_shutdown(struct device *dev)
{
	TOUCH_TRACE();

	return 0;
}

static int ili9881h_upgrade(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	const struct firmware *fw = NULL;
	char fwpath[256] = {0};
	int ret = 0;

	if ((lge_get_laf_mode() == LGE_LAF_MODE_LAF) || (touch_boot_mode() == TOUCH_CHARGER_MODE)) {
		TOUCH_I("laf & charger mode booting fw upgrade skip!!\n");
		return -EPERM;
	}

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
	if (check_recovery_boot == LGE_RECOVERY_BOOT) {
		TOUCH_I("recovery mode booting fw upgrade skip!!\n");
		return -EPERM;
	}
#endif

	if (atomic_read(&ts->state.fb) >= FB_SUSPEND) {
		TOUCH_I("state.fb is not FB_RESUME\n");
		return -EPERM;
	}

	if (ts->test_fwpath[0]) {
		memcpy(fwpath, &ts->test_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from test_fwpath:%s\n", &ts->test_fwpath[0]);
	} else if (ts->def_fwcnt) {
		memcpy(fwpath, ts->def_fwpath[0], sizeof(fwpath));
		TOUCH_I("get fwpath from def_fwpath : %s", fwpath);
	} else {
		TOUCH_E("no firmware file\n");
		return -EPERM;
	}

	fwpath[sizeof(fwpath) - 1] = '\0';

	if (strlen(fwpath) <= 0) {
		TOUCH_E("error get fw path\n");
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, dev);
	if (ret) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		return ret;
	}

	TOUCH_I("fw size:%zu, data: %p\n", fw->size, fw->data);

	ret = ili9881h_fw_upgrade(dev, fw);
	if (ret) {
		TOUCH_E("fail to firmware upgrade");
		ili9881h_reset_ctrl(dev, HW_RESET_ASYNC);
	}

	release_firmware(fw);

	return ret;
}

static int ili9881h_suspend(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int mfts_mode = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (touch_boot_mode() == TOUCH_CHARGER_MODE)
		return -EPERM;

	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		TOUCH_I("%s : touch_suspend - MFTS\n", __func__);
		ili9881h_power(dev, POWER_OFF);
		return -EPERM;
	} else {
		TOUCH_I("%s : touch_suspend start\n", __func__);
	}

	atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_READY);
	TOUCH_I("Skip lpwg mode - END (suspend)\n");

	if (atomic_read(&d->init) == IC_INIT_DONE) {
		ret = ili9881h_lpwg_mode(dev);
		if (ret)
			TOUCH_E("failed to lpwg_mode, ret:%d", ret);
	} else { /* need init */
		return 1;
	}

	return ret;
}

static int ili9881h_resume(struct device *dev)
{
#if 0 /* FW-upgrade not working at MFTS mode */
	struct touch_core_data *ts = to_touch_core(dev);
	int mfts_mode = 0;
#endif

	TOUCH_TRACE();

#if 0 /* FW-upgrade not working at MFTS mode */
	mfts_mode = touch_boot_mode_check(dev);
	if ((mfts_mode >= MINIOS_MFTS_FOLDER) && !ts->role.mfts_lpwg) {
		ili9881h_power(dev, POWER_ON);
		touch_msleep(ts->caps.hw_reset_delay);
		ili9881h_ic_info(dev);
		if (ili9881h_upgrade(dev) == 0) {
			ili9881h_power(dev, POWER_OFF);
			ili9881h_power(dev, POWER_ON);
			touch_msleep(ts->caps.hw_reset_delay);
		}
	}
#endif
	if (touch_boot_mode() == TOUCH_CHARGER_MODE) {
		ili9881h_deep_sleep_ctrl(dev, IC_DEEP_SLEEP);
		return -EPERM;
	}

	return 0;
}

static int ili9881h_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	u8 val = 0;
	int ret = 0;

	TOUCH_TRACE();

	if (atomic_read(&ts->state.core) == CORE_PROBE) {
		TOUCH_I("fb_notif change\n");
		fb_unregister_client(&ts->fb_notif);
		ts->fb_notif.notifier_call = ili9881h_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
	}

	ret = ili9881h_ic_info(dev);
	if (ret) {
		TOUCH_E("ili9881h_ic_info failed, ret:%d\n", ret);
		return ret;
	}

	/* Control option usb plug */
	TOUCH_I("%s: charger_state = 0x%02X\n", __func__, d->charger);
	val = d->charger ? true : false;
	ili9881h_plug_ctrl(dev, val);

	/* TODO Control option ime status */

	/* TODO Control option phone cover */

	atomic_set(&d->init, IC_INIT_DONE);
	atomic_set(&ts->state.sleep, IC_NORMAL);
	atomic_set(&d->changing_display_mode, CHANGING_DISPLAY_MODE_READY);
	TOUCH_I("Skip lpwg mode - END (init)\n");
	d->actual_fw_mode = FIRMWARE_DEMO_MODE;
	d->err_cnt = 0;

	ret = ili9881h_lpwg_mode(dev);
	if (ret)
		TOUCH_E("failed to lpwg_mode, ret:%d", ret);

	return 0;
}

int ili9811h_switch_fw_mode(struct device *dev, u8 mode)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0, i, prev_mode;
	int checksum = 0, codeLength = 8;
	u8 mp_code[8] = { 0 };
	struct ili9881h_data *d = to_ili9881h_data(dev);

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	if (mode < FIRMWARE_DEMO_MODE || mode > FIRMWARE_GESTURE_MODE) {
		TOUCH_E("Arguments from user space are invaild\n");
		goto out;
	}

	prev_mode = d->actual_fw_mode;
	d->actual_fw_mode = mode;

	TOUCH_I("Switch FW mode= %x, Prev FW mode = %x\n", mode, prev_mode);

	if (mode == FIRMWARE_DEMO_MODE || mode == FIRMWARE_DEBUG_MODE) {
		TOUCH_I("Switch to Demo/Debug mode, cmd = 0x%x, mode = 0x%x\n", CMD_MODE_CONTROL, mode);

		if (ili9881h_reg_write(dev, CMD_MODE_CONTROL, &mode, sizeof(mode)) < 0) {
			TOUCH_E("Failed to switch Demo/Debug mode\n");
			ret = -1;
			goto out;
		}
	} else if (mode == FIRMWARE_TEST_MODE) {
		TOUCH_I("Switch to Test mode, cmd = 0x%x, mode = 0x%x\n", CMD_MODE_CONTROL, mode);

		if (ili9881h_reg_write(dev, CMD_MODE_CONTROL, &mode, sizeof(mode)) < 0) {
			TOUCH_E("Failed to switch Test mode\n");
			ret = -1;
			goto out;
		}

		/* Read MP Test information to ensure if fw supports test mode. */
		ili9881h_reg_read(dev, CMD_READ_MP_TEST_CODE_INFO, mp_code, codeLength);

		for (i = 0; i < codeLength - 1; i++)
			checksum += mp_code[i];

		if ((-checksum & 0xFF) != mp_code[codeLength - 1]) {
			TOUCH_E("checksume error (0x%x), FW doesn't support test mode.\n",
					(-checksum & 0XFF));
			ret = -1;
			goto out;
		}

		/* After command to test mode, fw stays at demo mode until busy free. */
		d->actual_fw_mode = FIRMWARE_DEMO_MODE;

		/* Check ready to switch test mode from demo mode */
		if (ili9881h_check_cdc_busy(dev, 10, 50) < 0) {
			TOUCH_E("Check busy Timout\n");
			ret = -1;
			goto out;
		}

		/* Now set up fw as test mode */
		d->actual_fw_mode = FIRMWARE_TEST_MODE;

		if (ili9881h_mp_move_code() != 0) {
			TOUCH_E("Switch to test mode failed\n");
			ret = -1;
			goto out;
		}
	} else {
		TOUCH_E("Unknown firmware mode: %x\n", mode);
		ret = -1;
	}

out:
	if (ret)
		d->actual_fw_mode = prev_mode;

	TOUCH_I("Actual FW mode = %d\n", d->actual_fw_mode);
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	return ret;
}

u8 ili9881h_calc_data_checksum(void *pMsg, int type, u32 nLength)
{
	int i;
	u8 *p8 = NULL;
	u32 nCheckSum = 0;

	TOUCH_TRACE();

	if (pMsg == NULL) {
		TOUCH_E("The data going to dump is NULL\n");
		return -1;
	}

	if (type == 8)
		p8 = (u8 *) pMsg;

	for (i = 0; i < nLength; i++) {
		nCheckSum += p8[i];
	}

	return (u8) ((-nCheckSum) & 0xFF);
}


static int ili9881h_get_gesture_data(struct device *dev, int count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	u8 *data = d->gesture_info.data;
	u16 nX = 0, nY = 0;
	u8 i = 0;

	if (!count)
		return 0;

	ts->lpwg.code_num = count;

	for (i = 0; i < count; i++) {

		nX = (((data[(3 * i) + 4] & 0xF0) << 4) | data[(3 * i) + 6]);
		nY = (((data[(3 * i) + 4] & 0x0F) << 8) | data[(3 * i) + 5]);

		ts->lpwg.code[i].x = nX * (ts->caps.max_x) / TPD_WIDTH;
		ts->lpwg.code[i].y = nY * (ts->caps.max_y) / TPD_WIDTH;

		if (ts->lpwg.mode >= LPWG_PASSWORD)
			TOUCH_I("LPWG data xxxx, xxxx\n");
		else
			TOUCH_I("LPWG data %d, %d\n",
				ts->lpwg.code[i].x, ts->lpwg.code[i].y);
	}
	ts->lpwg.code[count].x = -1;
	ts->lpwg.code[count].y = -1;

	return 0;
}

static int ili9881h_get_swipe_data(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	u8 *data = d->gesture_info.data;
	u16 X1 = 0, Y1 = 0, X2 = 0, Y2 = 0;

	/* start (X, Y), end (X, Y), time = 3/3/2 bytes  */
	X1 = (((data[4] & 0xF0) << 4) | data[5]) * (ts->caps.max_x) / TPD_WIDTH;
	Y1 = (((data[4] & 0x0F) << 8) | data[6]) * (ts->caps.max_y) / TPD_WIDTH;
	X2 = (((data[7] & 0xF0) << 4) | data[8]) * (ts->caps.max_x) / TPD_WIDTH;
	Y2 = (((data[7] & 0x0F) << 8) | data[9]) * (ts->caps.max_y) / TPD_WIDTH;

	TOUCH_I("Swipe Gesture: start(%4d,%4d) end(%4d,%4d) swipe_time(%dms)\n",
		X1, Y1, X2, Y2, (((data[160] & 0xFF) << 8) | data[161]));


	ts->lpwg.code_num = 1;
	ts->lpwg.code[0].x = X2;
	ts->lpwg.code[0].y = Y2;

	ts->lpwg.code[1].x = -1;
	ts->lpwg.code[1].y = -1;

	return 0;

}
static int ili9881h_irq_abs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_touch_abs_data *abs_data = d->touch_info.abs_data;
	//struct ili9881h_touch_shape_data *shape_data = d->touch_info.shape_data;
	struct touch_data *tdata;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;
	u16 nX = 0, nY = 0;
	u8 check_sum = 0;

	TOUCH_TRACE();

	ts->new_mask = 0;
	memset(&d->touch_info, 0x0, sizeof(struct ili9881h_touch_info));

	ret = ili9881h_reg_read(dev, CMD_NONE, &d->touch_info, sizeof(d->touch_info));
	if (ret) {
		TOUCH_E("touch_info read fail\n");
		ret = -1;
		goto error;
	}

	// for touch data debug
	ili9881h_dump_packet(&d->touch_info, 8, sizeof(d->touch_info), 8, "DEMO");

	if (d->touch_info.packet_id != DEMO_PACKET_ID) {
		TOUCH_E("Packet ID Error, pid = 0x%x\n", d->touch_info.packet_id);
		ret = -1;
		goto error;
	}

	//Compare checksum
	check_sum = ili9881h_calc_data_checksum(&d->touch_info, 8, (sizeof(d->touch_info) - 1));
	if (check_sum != d->touch_info.checksum) {
		TOUCH_E("Packet check sum Error, check_sum = %d, abs_data = %d\n",
				check_sum, d->touch_info.checksum);
		ret = -1;
		goto error;
	}

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((abs_data[i].x_high == 0xF) && (abs_data[i].y_high && 0xF) &&
				(abs_data[i].x_low == 0xFF) && (abs_data[i].y_low == 0xFF)) {
			continue;
		}

		nX = abs_data[i].x_high << 8 | abs_data[i].x_low;
		nY = abs_data[i].y_high << 8 | abs_data[i].y_low;

		tdata = ts->tdata + i;
		tdata->id = i;
		tdata->x = nX * (ts->caps.max_x) / TPD_WIDTH;
		tdata->y = nY * (ts->caps.max_y) / TPD_WIDTH;
		tdata->pressure = abs_data[i].pressure;
		tdata->width_major = 0;
		tdata->width_minor = 0;
		tdata->orientation = 0;
		/*
		tdata->width_major = shape_data[i].width_major_high << 8 | shape_data[i].width_major_low;
		tdata->width_minor = shape_data[i].width_minor_high << 8 | shape_data[i].width_minor_low;
		tdata->orientation = shape_data[i].degree;
		*/

		ts->new_mask |= (1 << i);
		finger_index++;

		TOUCH_D(ABS,
			"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
				tdata->id,
				tdata->type,
				tdata->x,
				tdata->y,
				tdata->pressure,
				tdata->width_major,
				tdata->width_minor,
				tdata->orientation);
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

error:
	return ret;
}

static void ili9881h_irq_lpwg_failreason(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	u8 *data = d->gesture_info.data;
	u8 failreason, pid;

	if (!d->lpwg_failreason_ctrl)
		return;

	pid = data[0];
	failreason = data[1];
	failreason = failreason & 0xFF;

	if (pid == TCI_FAILREASON_PACKET_ID) {
		TOUCH_I("[TCI-0 Knock-on] TCI_FAILREASON_BUF = [0X%x]%s\n", failreason, lpwg_failreason_tci_str[failreason]);
	} else if (pid == SWIPE_FAILREASON_PACKET_ID) {	
		TOUCH_I("[WIPE] SWIPE_FAILREASON_BUF = [0X%x]%s\n", failreason, lpwg_failreason_swipe_str[failreason]);
	} else {
		TOUCH_I("not supported lpwg_failreason, pid = 0x%x\n", pid);
	}
}

static void ili9881h_irq_lpwg_gesture(struct device *dev, int gcode)
{
	struct touch_core_data *ts = to_touch_core(dev);

	switch (gcode) {
		case GESTURE_DOUBLECLICK:
			if (ts->lpwg.mode != LPWG_NONE) {
				ili9881h_get_gesture_data(dev, ts->tci.info[TCI_1].tap_count);
				ts->intr_status = TOUCH_IRQ_KNOCK;
			}
			break;
		case GESTURE_UP:
			if (ts->swipe.mode == SWIPE_UP) {
				ts->intr_status = TOUCH_IRQ_SWIPE_UP;
				ili9881h_get_swipe_data(dev);
			}
			break;
		default:
			TOUCH_I("not supported LPWG wakeup_type, gesture_code = 0x%x\n", gcode);
			break;
	}
}

int ili9881h_irq_lpwg(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	u8 *data = d->gesture_info.data;
	int ret = 0, pid = 0, gcode = 0;

	TOUCH_TRACE();

	ts->new_mask = 0;
	memset(&d->gesture_info, 0x0, sizeof(struct ili9881h_gesture_info));

	ret = ili9881h_reg_read(dev, CMD_NONE, &d->gesture_info, sizeof(d->gesture_info));
	if (ret < 0) {
		TOUCH_E("touch_info read fail\n");
		goto error;
	}

	ili9881h_dump_packet(data, 8, sizeof(d->touch_info), 8, "Gesture");

	pid = data[0];
	gcode = data[1];

	TOUCH_I("pid = 0x%x, code = 0x%x\n", pid, gcode);

	switch (pid) {
		case GESTURE_PACKET_ID:
			ili9881h_irq_lpwg_gesture(dev, gcode);
			break;
		case TCI_FAILREASON_PACKET_ID:
		case SWIPE_FAILREASON_PACKET_ID:
			ili9881h_irq_lpwg_failreason(dev);
			break;
		default:
			TOUCH_E("Packet Gesture ID Error, pid = 0x%x\n", pid);
			break;
	}

error:
	return ret;
}

int ili9881h_irq_debug(struct device *dev) {
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct touch_data *tdata;
	int debug_len = 0;
	u8 *data = d->debug_info.data;
	u8 finger_index = 0;
	int ret = 0;
	int i = 0;
	u16 nX = 0, nY = 0;

	TOUCH_TRACE();

	ts->new_mask = 0;
	memset(data, 0x0, sizeof(*data));

	/* read debug packet header */
	ret = ili9881h_reg_read(dev, CMD_NONE, &d->debug_info.data,
			DEBUG_MODE_PACKET_HEADER_LENGTH);
	if (ret) {
		TOUCH_E("d->debug_info.data header read fail\n");
		goto error;
	}
	if (d->debug_info.data[0] != DEBUG_PACKET_ID) {
		TOUCH_E("Packet ID Error, pid = 0x%x\n", d->debug_info.data[0]);
		goto error;
	}

	debug_len = d->debug_info.data[1] << 8 | d->debug_info.data[2];
	TOUCH_D(ABS, "debug_length = %d\n", debug_len);
	ret = ili9881h_reg_read(dev, CMD_NONE, &(d->debug_info.data[5]),
			debug_len - DEBUG_MODE_PACKET_HEADER_LENGTH);
	if (ret) {
		TOUCH_E("d->debug_info.data read fail\n");
		goto error;
	}

	// for touch data debug
	ili9881h_dump_packet(data, 8, sizeof(d->debug_info.data), 8, "DEBUG");

	/* Decode debug report point*/
	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((data[(3 * i) + 5] == 0xFF) &&
				(data[(3 * i) + 6] && 0xFF) &&
				(data[(3 * i) + 7] == 0xFF)) {
			continue;
		}

		nX = (((data[(3 * i) + 5] & 0xF0) << 4) | (data[(3 * i) + 6]));
		nY = (((data[(3 * i) + 5] & 0x0F) << 8) | (data[(3 * i) + 7]));

		tdata = ts->tdata + i;
		tdata->id = i;
		tdata->x = nX * (ts->caps.max_x) / TPD_WIDTH;
		tdata->y = nY * (ts->caps.max_y) / TPD_WIDTH;
		/* TODO TMP */
		tdata->pressure = 60;
		tdata->width_major = 1;
		tdata->width_minor = 1;
		tdata->orientation = 1;

		ts->new_mask |= (1 << i);
		finger_index++;

		TOUCH_D(ABS,
			"tdata [id:%d t:%d x:%d y:%d z:%d-%d,%d,%d]\n",
				tdata->id,
				tdata->type,
				tdata->x,
				tdata->y,
				tdata->pressure,
				tdata->width_major,
				tdata->width_minor,
				tdata->orientation);
	}

	ts->tcount = finger_index;
	ts->intr_status = TOUCH_IRQ_FINGER;

	/* Save debug raw data to apk buffer */
	if (d->debug_info.enable) {
		mutex_lock(&d->debug_lock);
		memset(d->debug_info.buf[d->debug_info.frame_cnt], 0x00, (u8) sizeof(u8) * 2048);
		memcpy(d->debug_info.buf[d->debug_info.frame_cnt], d->debug_info.data, sizeof(d->debug_info.data));
		d->debug_info.frame_cnt++;
		if (d->debug_info.frame_cnt > 1) {
			TOUCH_I("frame_cnt = %d\n", d->debug_info.frame_cnt);
		}
		if (d->debug_info.frame_cnt > 1023) {
			TOUCH_E("frame_cnt = %d > 1024\n",
				d->debug_info.frame_cnt);
			d->debug_info.frame_cnt = 1023;
		}
		mutex_unlock(&d->debug_lock);
		wake_up(&d->inq);
	}

error:
	return ret;

}

int ili9881h_irq_handler(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;

	switch (d->actual_fw_mode) {
		case FIRMWARE_DEMO_MODE:
			ret = ili9881h_irq_abs(dev);
			break;
		case FIRMWARE_DEBUG_MODE:
			ret = ili9881h_irq_debug(dev);
			break;
		case FIRMWARE_GESTURE_MODE:
			ret = ili9881h_irq_lpwg(dev);
			break;
		default:
			TOUCH_E("Unknow fw mode (0x%x)\n", d->actual_fw_mode);
			break;
	}

	return ret;
}

static ssize_t show_swipe_enable(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	ret = snprintf(buf + ret, PAGE_SIZE, "SWIPE mode = 0x%X\n", ts->swipe.mode);
	TOUCH_I("Swipe mode = %s\n", ts->swipe.mode == SWIPE_UP ? "SWIPE_UP" : "Disable");

	return ret;
}

static ssize_t store_swipe_enable(struct device *dev, const char *buf, size_t count)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if(value > 1 || value < 0) {
		TOUCH_I("Set Swipe mode wrong, 0(Disable), 1(SWIPE_UP) only\n");
		return count;
	}

	ts->swipe.mode = value ? SWIPE_UP : 0;
	TOUCH_I("%s - Swipe Enable Value = %s\n", __func__, ts->swipe.mode ? "Enable" : "Disable");

	return count;
}

static ssize_t store_reg_ctrl(struct device *dev, const char *buf, size_t count)
{
	char command[6] = {0};
	u32 cmd32 = 0;
	u8 cmd = 0;
	int data = 0;

	if (sscanf(buf, "%5s %x %d", command, &cmd32, &data) <= 0)
		return count;

	cmd = cmd32;
	if (!strcmp(command, "write")) {
		if (ili9881h_reg_write(dev, cmd, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x write fail\n", cmd);
		else
			TOUCH_I("reg[%x] = 0x%x\n", cmd, data);
	} else if (!strcmp(command, "read")) {
		if (ili9881h_reg_read(dev, cmd, &data, sizeof(u32)) < 0)
			TOUCH_E("reg addr 0x%x read fail\n", cmd);
		else
			TOUCH_I("reg[%x] = 0x%x\n", cmd, data);
	} else {
		TOUCH_D(BASE_INFO, "Usage\n");
		TOUCH_D(BASE_INFO, "Write reg value\n");
		TOUCH_D(BASE_INFO, "Read reg\n");
	}
	return count;
}

static ssize_t show_lpwg_failreason(struct device *dev, char *buf)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;

	TOUCH_I("Failreason Ctrl[Driver] = %s\n", d->lpwg_failreason_ctrl ? "Enable" : "Disable");

	return ret;
}

static ssize_t store_lpwg_failreason(struct device *dev, const char *buf, size_t count)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	if (value > 1 || value < 0) {
		TOUCH_I("Set Lpwg Failreason Ctrl - 0(disable), 1(enable) only\n");
		return count;
	}

	d->lpwg_failreason_ctrl = (u8)value;
	TOUCH_I("Set Lpwg Failreason Ctrl = %s\n", value ? "Enable" : "Disable");

	return count;
}

static ssize_t store_reset_ctrl(struct device *dev, const char *buf, size_t count)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0)
		return count;

	ili9881h_reset_ctrl(dev, value);

	return count;
}

static ssize_t show_pinstate(struct device *dev, char *buf)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	ret = snprintf(buf, PAGE_SIZE, "RST:%d, INT:%d\n",
			gpio_get_value(ts->reset_pin), gpio_get_value(ts->int_pin));
	TOUCH_I("%s() buf:%s",__func__, buf);
	return ret;
}

static TOUCH_ATTR(swipe_enable, show_swipe_enable, store_swipe_enable);
static TOUCH_ATTR(reg_ctrl, NULL, store_reg_ctrl);
static TOUCH_ATTR(lpwg_failreason, show_lpwg_failreason, store_lpwg_failreason);
static TOUCH_ATTR(reset_ctrl, NULL, store_reset_ctrl);
static TOUCH_ATTR(pinstate, show_pinstate, NULL);


static struct attribute *ili9881h_attribute_list[] = {
	&touch_attr_swipe_enable.attr,
	&touch_attr_reg_ctrl.attr,
	&touch_attr_lpwg_failreason.attr,
	&touch_attr_reset_ctrl.attr,
	&touch_attr_pinstate.attr,
	NULL,
};

static const struct attribute_group ili9881h_attribute_group = {
	.attrs = ili9881h_attribute_list,
};

static int ili9881h_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &ili9881h_attribute_group);
	if (ret) {
		TOUCH_E("ili9881h sysfs register failed\n");

		goto error;
	}

	ili9881h_mp_register_sysfs(dev);
	if (ret) {
		TOUCH_E("ili9881h register failed\n");

		goto error;
	}

	return 0;

error:
	kobject_del(&ts->kobj);

	return ret;
}

static int ili9881h_get_cmd_version(struct device *dev, char *buf)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_fw_info *fw_info = &d->ic_info.fw_info;
	struct ili9881h_protocol_info *protocol_info = &d->ic_info.protocol_info;
	struct ili9881h_core_info *core_info = &d->ic_info.core_info;
	int offset = 0;
	int ret = 0;

	ret = ili9881h_ic_info(dev);
	if (ret) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Read Fail Touch IC Info\n");
		return offset;
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"==================== Version Info ====================\n");
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Version = v%d.%02d, fw_core.customer_id = %d.%d\n",
			fw_info->major, fw_info->minor, fw_info->core, fw_info->customer_code);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Procotol Version = %d.%d.%d\n",
			protocol_info->major, protocol_info->mid, protocol_info->minor);
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Core Version = %d.%d.%d.%d\n",
			core_info->code_base, core_info->minor, core_info->revision_major, core_info->revision_minor); 
	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"======================================================\n");

	return offset;
}

static int ili9881h_get_cmd_atcmd_version(struct device *dev, char *buf)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_fw_info *fw_info = &d->ic_info.fw_info;
	int offset = 0;
	int ret = 0;

	ret = ili9881h_ic_info(dev);
	if (ret) {
		offset += snprintf(buf + offset, PAGE_SIZE, "-1\n");
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
				"Read Fail Touch IC Info\n");
		return offset;
	}

	offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"v%d.%02d\n", fw_info->major, fw_info->minor);

	return offset;
}

static int ili9881h_set(struct device *dev, u32 cmd, void *input, void *output)
{
	TOUCH_TRACE();

	return 0;
}

static int ili9881h_get(struct device *dev, u32 cmd, void *input, void *output)
{
	int ret = 0;

	TOUCH_D(BASE_INFO, "%s : cmd %d\n", __func__, cmd);

	switch (cmd) {
		case CMD_VERSION:
			ret = ili9881h_get_cmd_version(dev, (char *)output);
			break;

		case CMD_ATCMD_VERSION:
			ret = ili9881h_get_cmd_atcmd_version(dev, (char *)output);
			break;

		default:
			break;
	}

	return ret;
}

static struct touch_driver touch_driver = {
	.probe = ili9881h_probe,
	.remove = ili9881h_remove,
	.shutdown = ili9881h_shutdown,
	.suspend = ili9881h_suspend,
	.resume = ili9881h_resume,
	.init = ili9881h_init,
	.irq_handler = ili9881h_irq_handler,
	.power = ili9881h_power,
	.upgrade = ili9881h_upgrade,
	.lpwg = ili9881h_lpwg,
	.notify = ili9881h_notify,
	.register_sysfs = ili9881h_register_sysfs,
	.set = ili9881h_set,
	.get = ili9881h_get,
};

#define MATCH_NAME	"lge,ili9881h"

static struct of_device_id touch_match_ids[] = {
	{ .compatible = MATCH_NAME, },
	{ },
};

static struct touch_hwif hwif = {
	.bus_type = HWIF_I2C,
	.name = LGE_TOUCH_NAME,
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(touch_match_ids),
};

#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
extern int lge_get_maker_id(void);
#endif

static int __init touch_device_init(void)
{
	TOUCH_TRACE();

	TOUCH_I("touch_device_init func\n");

#if defined(CONFIG_LGE_MODULE_DETECT)
#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
	panel_id = lge_get_panel_type();

	switch (panel_id) {
		case BOE_INCELL_ILI9881H:
			TOUCH_I("%s, ili9881h found!\n", __func__);
			break;
		default:
			TOUCH_I("%s, ili9881h not found. panel_id : %d\n", __func__, panel_id);
			return 0;
	}
#elif defined(CONFIG_LGE_TOUCH_CORE_MTK)
	panel_id = lge_get_maker_id(); //0: LGD panel, 1: TOVIS panel

	switch (panel_id) {
		case BOE_INCELL_ILI9881H:
			TOUCH_I("%s, ili9881h found!\n", __func__);
			break;
		default:
			TOUCH_I("%s, ili9881h not found. panel_id : %d\n", __func__, panel_id);
			return 0;
	}
#endif
#endif

	return touch_bus_device_init(&hwif, &touch_driver);
}

static void __exit touch_device_exit(void)
{
	TOUCH_TRACE();
	touch_bus_device_exit(&hwif);
}

module_init(touch_device_init);
module_exit(touch_device_exit);

MODULE_AUTHOR("BSP-TOUCH@lge.com");
MODULE_DESCRIPTION("LGE touch driver v3");
MODULE_LICENSE("GPL");

