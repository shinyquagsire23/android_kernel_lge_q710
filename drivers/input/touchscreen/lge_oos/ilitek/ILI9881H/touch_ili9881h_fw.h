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

#ifndef LGE_TOUCH_ILI9881H_FW_H
#define LGE_TOUCH_ILI9881H_FW_H

#include <linux/firmware.h>
/* The size of firmware upgrade */
#define MAX_HEX_FILE_SIZE		(160*1024)
#define MAX_FLASH_FIRMWARE_SIZE		(256*1024)
#define MAX_IRAM_FIRMWARE_SIZE		(60*1024)
#define ILI_FILE_HEADER			64
#define MAX_AP_FIRMWARE_SIZE		(64*1024)
#define MAX_DLM_FIRMWARE_SIZE		(8*1024)
#define MAX_MP_FIRMWARE_SIZE		(64*1024)
#define MAX_GESTURE_FIRMWARE_SIZE	(8*1024)
#define DLM_START_ADDRESS		0x20610
#define DLM_HEX_ADDRESS             0x10000
#define MP_HEX_ADDRESS              0x13000
#define SPI_UPGRADE_LEN		        2048
#define UPDATE_RETRY_COUNT          3

struct flash_table {
	uint16_t mid;
	uint16_t dev_id;
	int mem_size;
	int program_page;
	int sector;
	int block;
};

extern int ili9881h_read_flash_info(struct device *dev, u8 cmd, int len);
extern int ili9881h_fw_upgrade(struct device *dev, const struct firmware *fw);

extern int g_update_percentage;
#endif
