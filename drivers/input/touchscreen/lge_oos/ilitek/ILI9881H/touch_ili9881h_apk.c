/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <linux/file.h>

#include <touch_core.h>

#include "touch_ili9881h.h"
#include "touch_ili9881h_mp.h"
#include "touch_ili9881h_fw.h"

#define USER_STR_BUFF	128
#define ILITEK_IOCTL_MAGIC	100
#define ILITEK_IOCTL_MAXNR	19

#define ILITEK_IOCTL_I2C_WRITE_DATA		_IOWR(ILITEK_IOCTL_MAGIC, 0, uint8_t*)
#define ILITEK_IOCTL_I2C_SET_WRITE_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA		_IOWR(ILITEK_IOCTL_MAGIC, 2, uint8_t*)
#define ILITEK_IOCTL_I2C_SET_READ_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 3, int)

#define ILITEK_IOCTL_TP_HW_RESET		_IOWR(ILITEK_IOCTL_MAGIC, 4, int)
#define ILITEK_IOCTL_TP_POWER_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 5, int)
#define ILITEK_IOCTL_TP_REPORT_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 6, int)
#define ILITEK_IOCTL_TP_IRQ_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 7, int)

#define ILITEK_IOCTL_TP_FUNC_MODE		_IOWR(ILITEK_IOCTL_MAGIC, 9, int)

#define ILITEK_IOCTL_TP_FW_VER			_IOWR(ILITEK_IOCTL_MAGIC, 10, uint8_t*)
#define ILITEK_IOCTL_TP_PL_VER			_IOWR(ILITEK_IOCTL_MAGIC, 11, uint8_t*)
#define ILITEK_IOCTL_TP_CORE_VER		_IOWR(ILITEK_IOCTL_MAGIC, 12, uint8_t*)
#define ILITEK_IOCTL_TP_DRV_VER			_IOWR(ILITEK_IOCTL_MAGIC, 13, uint8_t*)
#define ILITEK_IOCTL_TP_CHIP_ID			_IOWR(ILITEK_IOCTL_MAGIC, 14, uint32_t*)

#define ILITEK_IOCTL_TP_MODE_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 17, uint8_t*)
#define ILITEK_IOCTL_TP_MODE_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 18, int*)
#define ILITEK_IOCTL_ICE_MODE_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 19, int)

#define ILITEK_IOCTL_DEBUG_BUF_LENGTH 2048

unsigned char g_user_buf[USER_STR_BUFF] = { 0 };
static struct ili9881h_data *touch_dev = NULL;
#define REGISTER_READ	0
#define REGISTER_WRITE	1
uint32_t temp[5] = {0};

int str2hex(char *str)
{
	int strlen, result, intermed, intermedtop;
	char *s = str;

	while (*s != 0x0) {
		s++;
	}

	strlen = (int)(s - str);
	s = str;
	if (*s != 0x30) {
		return -1;
	}

	s++;

	if (*s != 0x78 && *s != 0x58) {
		return -1;
	}
	s++;

	strlen = strlen - 3;
	result = 0;
	while (*s != 0x0) {
		intermed = *s & 0x0f;
		intermedtop = *s & 0xf0;
		if (intermedtop == 0x60 || intermedtop == 0x40) {
			intermed += 0x09;
		}
		intermed = intermed << (strlen << 2);
		result = result | intermed;
		strlen -= 1;
		s++;
	}
	return result;
}


static ssize_t ilitek_proc_read_write_register_read(struct file *pFile, char __user *buf, size_t nCount, loff_t *pos)
{
	int ret = 0;
	uint32_t type, addr, read_data, write_data, write_len;

	if (*pos != 0)
		return 0;

	type = temp[0];
	addr = temp[1];
	write_data = temp[2];
	write_len = temp[3];

	touch_interrupt_control(touch_dev->dev, INTERRUPT_DISABLE);

	ret = ili9881h_ice_mode_enable(touch_dev->dev);
	if (ret < 0) {
		TOUCH_E("Failed to enter ICE mode, ret = %d\n", ret);
		return -1;
	}

	if (type == REGISTER_READ) {
		read_data = ili9881h_ice_reg_read(touch_dev->dev, addr);
		TOUCH_I("READ:addr = 0x%06x, read = 0x%08x\n", addr, read_data);
		nCount = snprintf(g_user_buf, PAGE_SIZE, "READ:addr = 0x%06x, read = 0x%08x\n", addr, read_data);

	} else {
		ili9881h_ice_reg_write(touch_dev->dev, addr, write_data, write_len);
		TOUCH_I("WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
		nCount = snprintf(g_user_buf, PAGE_SIZE, "WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
	}

	ili9881h_ice_mode_disable(touch_dev->dev);

	touch_interrupt_control(touch_dev->dev, INTERRUPT_ENABLE);

	ret = copy_to_user(buf, g_user_buf, nCount);
	if (ret < 0) {
		TOUCH_E("Failed to copy data to user space");
	}

	*pos += nCount;

	return nCount;

}

static ssize_t ilitek_proc_read_write_register_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	char *token = NULL, *cur = NULL;
	char cmd[256] = { 0 };
	uint32_t count = 0;

	if (buff != NULL) {
		ret = copy_from_user(cmd, buff, size - 1);
		if (ret < 0) {
			TOUCH_I("copy data from user space, failed\n");
			return -1;
		}
	}

	token = cur = cmd;

	while ((token = strsep(&cur, ",")) != NULL) {
		temp[count] = str2hex(token);
		TOUCH_I("data[%d] = 0x%x\n", count, temp[count]);
		count++;
	}

	return size;
}

static ssize_t ilitek_proc_debug_switch_read(struct file *pFile, char __user *buff, size_t nCount, loff_t *pPos)
{
	int res = 0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	touch_dev->debug_info.enable = !touch_dev->debug_info.enable;

	TOUCH_I(" %s debug_flag message = %x\n", touch_dev->debug_info.enable ? "Enabled" : "Disabled", touch_dev->debug_info.enable);

	nCount = sprintf(g_user_buf, "touch_dev->debug_info.enable : %s\n", touch_dev->debug_info.enable ? "Enabled" : "Disabled");

	*pPos += nCount;

	res = copy_to_user(buff, g_user_buf, nCount);
	if (res < 0) {
		TOUCH_E("Failed to copy data to user space");
	}

	return nCount;
}

static ssize_t ilitek_proc_debug_message_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	unsigned long p = *pPos;
	unsigned int count = size;
	int i = 0;
	int send_data_len = 0;
	size_t ret = 0;
	unsigned char *tmpbuf = NULL;
	unsigned char tmpbufback[128] = { 0 };

	TOUCH_TRACE();

	mutex_lock(&touch_dev->apk_lock);

	while (touch_dev->debug_info.frame_cnt <= 0) {
		if (filp->f_flags & O_NONBLOCK) {
			mutex_unlock(&touch_dev->apk_lock);
			return -EAGAIN;
		}
		wait_event_interruptible(touch_dev->inq, touch_dev->debug_info.frame_cnt > 0);
	}

	//TOUCH_I("Start to copy raw data to apk buffer\n");

	mutex_lock(&touch_dev->debug_lock);

	tmpbuf = kmalloc((2*ILITEK_IOCTL_DEBUG_BUF_LENGTH), GFP_KERNEL);	/* buf size if even */
	if (ERR_ALLOC_MEM(tmpbuf)) {
		TOUCH_E("buffer vmalloc error\n");
		send_data_len += sprintf(tmpbufback + send_data_len, "buffer vmalloc error\n");
		ret = copy_to_user(buff, tmpbufback, send_data_len);
	} else {
		TOUCH_D(ABS, "frame count = %d\n",touch_dev->debug_info.frame_cnt);
		if (touch_dev->debug_info.frame_cnt > 0) {
			for (i = 0; i < ILITEK_IOCTL_DEBUG_BUF_LENGTH - 8; i++) {
				send_data_len += sprintf(tmpbuf + send_data_len, "%02X", touch_dev->debug_info.buf[0][i]);

				if (send_data_len >= (2*ILITEK_IOCTL_DEBUG_BUF_LENGTH)) {
					TOUCH_E("send_data_len = %d set 4096 i = %d\n", send_data_len, i);
					send_data_len = (2*ILITEK_IOCTL_DEBUG_BUF_LENGTH);
					break;
				}
			}

			send_data_len += sprintf(tmpbuf + send_data_len, "\n\n");

			if (p == 5 || size == (2*ILITEK_IOCTL_DEBUG_BUF_LENGTH) || size == ILITEK_IOCTL_DEBUG_BUF_LENGTH) {
				touch_dev->debug_info.frame_cnt--;
				if (touch_dev->debug_info.frame_cnt < 0) {
					touch_dev->debug_info.frame_cnt = 0;
				}

				for (i = 1; i <= touch_dev->debug_info.frame_cnt; i++) {
					memcpy(touch_dev->debug_info.buf[i - 1], touch_dev->debug_info.buf[i], ILITEK_IOCTL_DEBUG_BUF_LENGTH);
				}
			}
		} else {
			TOUCH_E("no data send\n");
			send_data_len += sprintf(tmpbuf + send_data_len, "no data send\n");
		}

		/* Preparing to send data to user */
		if (size == (2*ILITEK_IOCTL_DEBUG_BUF_LENGTH))
			ret = copy_to_user(buff, tmpbuf, send_data_len);
		else
			ret = copy_to_user(buff, tmpbuf + p, send_data_len - p);

		if (ret) {
			TOUCH_E("copy_to_user err\n");
			ret = -EFAULT;
		} else {
			*pPos += count;
			ret = count;
			TOUCH_D(ABS, "Read %d bytes(s) from %ld\n", count, p);
		}
	}

	/* TOUCH_E("send_data_len = %d\n", send_data_len); */
	if (send_data_len <= 0 || send_data_len > (2*ILITEK_IOCTL_DEBUG_BUF_LENGTH)) {
		TOUCH_E("send_data_len = %d set %d\n", send_data_len, ILITEK_IOCTL_DEBUG_BUF_LENGTH);
		send_data_len = (2*ILITEK_IOCTL_DEBUG_BUF_LENGTH);
	}
	if (tmpbuf != NULL) {
		kfree(tmpbuf);
		tmpbuf = NULL;
	}

	mutex_unlock(&touch_dev->debug_lock);
	mutex_unlock(&touch_dev->apk_lock);
	return send_data_len;

}

static ssize_t ilitek_proc_fw_process_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int res = 0;
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;

	memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	len = sprintf(g_user_buf, "%02d", g_update_percentage);

	TOUCH_I("update status = %d\n", g_update_percentage);

	res = copy_to_user((uint32_t *) buff, &g_update_percentage, len);
	if (res < 0) {
		TOUCH_E("Failed to copy data to user space");
	}

	*pPos = len;

	return len;
}

#define APK_FW_FILE "ILITEK_FW"

static ssize_t ilitek_proc_fw_upgrade_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	uint32_t len = 0;
	const struct firmware *fw = NULL;
	char fwpath[256] = APK_FW_FILE;
	struct touch_core_data *ts = to_touch_core(touch_dev->dev);


	TOUCH_I("Upgarde FW called by APK\n");

	if (*pPos != 0)
		return 0;

	TOUCH_I("update hex path:%s\n", fwpath);

	fwpath[sizeof(fwpath) - 1] = '\0';

	if (strlen(fwpath) <= 0) {
		TOUCH_E("error get fw path\n");
		g_update_percentage = -1;
		return -EPERM;
	}

	TOUCH_I("fwpath[%s]\n", fwpath);

	ret = request_firmware(&fw, fwpath, touch_dev->dev);
	if (ret) {
		TOUCH_E("fail to request_firmware fwpath: %s (ret:%d)\n", fwpath, ret);
		g_update_percentage = -1;
		goto out;
	}

	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret = ili9881h_fw_upgrade(touch_dev->dev, fw);
	if (ret) {
		g_update_percentage = ret;
		TOUCH_E("Failed to upgrade firwmare\n");
	} else {
		g_update_percentage = 100;
		TOUCH_I("Succeed to upgrade firmware\n");
	}

	ili9881h_reset_ctrl(touch_dev->dev, HW_RESET_SYNC);

out:
	*pPos = len;
	return len;
}


static long ilitek_proc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int res = 0, length = 0;
	uint8_t szBuf[512] = { 0 };
	static uint16_t i2c_rw_length = 0;
	uint32_t id_to_user[3] = {0};

	TOUCH_I("cmd = %d\n", _IOC_NR(cmd));

	if (_IOC_TYPE(cmd) != ILITEK_IOCTL_MAGIC) {
		TOUCH_E("The Magic number doesn't match\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > ILITEK_IOCTL_MAXNR) {
		TOUCH_E("The number of ioctl doesn't match\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case ILITEK_IOCTL_I2C_WRITE_DATA:
		res = copy_from_user(szBuf, (uint8_t *) arg, i2c_rw_length);
		if (res < 0) {
			TOUCH_E("Failed to copy data from user space\n");
		} else {
			res = ili9881h_reg_write(touch_dev->dev, szBuf[0], &szBuf[1], (i2c_rw_length-1));
			if (res < 0)
				TOUCH_E("I2C Write Error\n");
		}
		break;

	case ILITEK_IOCTL_I2C_READ_DATA:
		res = ili9881h_reg_read(touch_dev->dev, CMD_NONE, szBuf, i2c_rw_length);
		if (res < 0) {
			TOUCH_E("Failed to read data via i2c\n");
		} else {
			res = copy_to_user((uint8_t *) arg, szBuf, i2c_rw_length);
			if (res < 0)
				TOUCH_E("Failed to copy data to user space\n");
		}
		break;

	case ILITEK_IOCTL_I2C_SET_WRITE_LENGTH:
	case ILITEK_IOCTL_I2C_SET_READ_LENGTH:
		i2c_rw_length = arg;
		TOUCH_I("i2c length  = %d\n", i2c_rw_length);
		break;

	case ILITEK_IOCTL_TP_HW_RESET:
		ili9881h_reset_ctrl(touch_dev->dev, HW_RESET_ONLY);
		break;

	case ILITEK_IOCTL_TP_REPORT_SWITCH:
		TOUCH_I("CMD (%d) does nothing in LG\n", _IOC_NR(cmd));
		break;

	case ILITEK_IOCTL_TP_IRQ_SWITCH:
		res = copy_from_user(szBuf, (uint8_t *) arg, 1);
		if (res < 0) {
			TOUCH_E("Failed to copy data from user space\n");
		} else {
			if (szBuf[0]) {
				touch_interrupt_control(touch_dev->dev, INTERRUPT_ENABLE);
			} else {
				touch_interrupt_control(touch_dev->dev, INTERRUPT_DISABLE);
			}
		}
		break;

	case ILITEK_IOCTL_TP_FUNC_MODE:
		res = copy_from_user(szBuf, (uint8_t *) arg, 3);
		if (res < 0) {
			TOUCH_E("Failed to copy data from user space\n");
		} else {
			res = ili9881h_reg_write(touch_dev->dev, szBuf[0], &szBuf[1], 2);
		}
		break;

	case ILITEK_IOCTL_TP_FW_VER:
		res = copy_to_user((uint8_t *) arg, &touch_dev->ic_info.fw_info, sizeof(touch_dev->ic_info.fw_info));
		if (res < 0) {
			TOUCH_E("Failed to copy protocol version to user space\n");
		}
		TOUCH_I("Firmware Version = %d.%d.%d.%d\n", touch_dev->ic_info.fw_info.core, touch_dev->ic_info.fw_info.customer_code,
								 touch_dev->ic_info.fw_info.major, touch_dev->ic_info.fw_info.minor);
		break;

	case ILITEK_IOCTL_TP_PL_VER:

		res = copy_to_user((uint8_t *) arg, &touch_dev->ic_info.protocol_info.major, sizeof(touch_dev->ic_info.protocol_info.major));
		if (res < 0)
			TOUCH_E("Failed to copy protocol version to user space\n");

		TOUCH_I("Procotol Version = %d.%d.%d\n",
		touch_dev->ic_info.protocol_info.major, touch_dev->ic_info.protocol_info.mid, touch_dev->ic_info.protocol_info.minor);
		break;

	case ILITEK_IOCTL_TP_CORE_VER:
		res = copy_to_user((uint8_t *) arg, &touch_dev->ic_info.core_info, sizeof(touch_dev->ic_info.core_info));
		if (res < 0)
			TOUCH_E("Failed to copy protocol version to user space\n");

		TOUCH_I("Core Version = %d.%d.%d.%d\n",
		 touch_dev->ic_info.core_info.code_base, touch_dev->ic_info.core_info.minor,
		 touch_dev->ic_info.core_info.revision_major, touch_dev->ic_info.core_info.revision_minor);
		break;

	case ILITEK_IOCTL_TP_DRV_VER:
		length = sprintf(szBuf, "1.0.3.8");
		if (!length) {
			TOUCH_E("Failed to convert driver version from definiation\n");
		} else {
			res = copy_to_user((uint8_t *) arg, szBuf, length);
			if (res < 0)
				TOUCH_E("Failed to copy driver ver to user space\n");
		}
		TOUCH_I("Driver version = %s",szBuf);
		ili9881h_tp_info(touch_dev->dev);
		break;

	case ILITEK_IOCTL_TP_CHIP_ID:

		id_to_user[0] = touch_dev->ic_info.chip_info.id << 16 | touch_dev->ic_info.chip_info.type;

		res = copy_to_user((uint32_t *) arg, id_to_user, sizeof(uint32_t));
		if (res < 0)
			TOUCH_E("Failed to copy chip id to user space\n");

		TOUCH_I("PID = 0x%x, Core type = 0x%x\n",
		touch_dev->ic_info.chip_info.id, touch_dev->ic_info.chip_info.type);
		break;


	case ILITEK_IOCTL_TP_MODE_CTRL:
		res = copy_from_user(szBuf, (uint8_t *) arg, 4);
		if (res < 0) {
			TOUCH_E("Failed to copy data from user space\n");
		} else {
			ili9811h_switch_fw_mode(touch_dev->dev, szBuf[0]);
		}
		break;

	case ILITEK_IOCTL_TP_MODE_STATUS:
		TOUCH_I( "Current firmware mode : %d", touch_dev->actual_fw_mode);
		res = copy_to_user((int *)arg, &touch_dev->actual_fw_mode, sizeof(touch_dev->actual_fw_mode));
		if (res < 0)
			TOUCH_E("Failed to copy chip id to user space\n");
		break;
	case ILITEK_IOCTL_ICE_MODE_SWITCH:
		TOUCH_I("CMD (%d) does nothing in LG\n", ILITEK_IOCTL_ICE_MODE_SWITCH);
		break;

	default:
		res = -ENOTTY;
		break;
	}

	return res;
}

struct proc_dir_entry *proc_dir_ilitek;
struct proc_dir_entry *proc_ioctl;
struct proc_dir_entry *proc_fw_process;
struct proc_dir_entry *proc_fw_upgrade;
struct proc_dir_entry *proc_debug_message;
struct proc_dir_entry *proc_debug_message_switch;

struct file_operations proc_ioctl_fops = {
	.unlocked_ioctl = ilitek_proc_ioctl,
};

struct file_operations proc_fw_process_fops = {
	.read = ilitek_proc_fw_process_read,
};

struct file_operations proc_fw_upgrade_fops = {
	.read = ilitek_proc_fw_upgrade_read,
};

struct file_operations proc_debug_message_fops = {
	.read = ilitek_proc_debug_message_read,
};

struct file_operations proc_debug_message_switch_fops = {
	.read = ilitek_proc_debug_switch_read,
};

struct file_operations proc_read_write_register_fops = {
	.read = ilitek_proc_read_write_register_read,
	.write = ilitek_proc_read_write_register_write,
};

typedef struct {
	char *name;
	struct proc_dir_entry *node;
	struct file_operations *fops;
	bool isCreated;
} proc_node_t;

proc_node_t proc_table[] = {
	{"ioctl", NULL, &proc_ioctl_fops, false},
	{"fw_process", NULL, &proc_fw_process_fops, false},
	{"fw_upgrade", NULL, &proc_fw_upgrade_fops, false},
	{"debug_message", NULL, &proc_debug_message_fops, false},
	{"debug_message_switch", NULL, &proc_debug_message_switch_fops, false},
	{"read_write_register", NULL, &proc_read_write_register_fops, false},
};


/* Extern our apk api for LG */
int ili9881h_apk_init(struct device *dev)
{
	int i = 0, res = 0;
	touch_dev = to_ili9881h_data(dev);
	proc_dir_ilitek = proc_mkdir("ilitek", NULL);

	for (; i < ARRAY_SIZE(proc_table); i++) {
		proc_table[i].node = proc_create(proc_table[i].name, 0666, proc_dir_ilitek, proc_table[i].fops);

		if (proc_table[i].node == NULL) {
			proc_table[i].isCreated = false;
			TOUCH_E("Failed to create %s under /proc\n", proc_table[i].name);
			res = -ENODEV;
		} else {
			proc_table[i].isCreated = true;
			TOUCH_I("Succeed to create %s under /proc\n", proc_table[i].name);
		}
	}

	return res;
}

