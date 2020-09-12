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

#include <touch_core.h>
#include <touch_hwif.h>
#include "touch_ili9881h.h"
#include "touch_ili9881h_fw.h"

#define K			(1024)
#define M			(K * K)
#define CHECK_FW_FAIL		(-1)
#define NEED_UPDATE		1
#define NO_NEED_UPDATE		0
#define FW_VER_ADDR		0xFFE0
#define CRC_ONESET(X, Y)	({Y = (*(X+0) << 24) | (*(X+1) << 16) | (*(X+2) << 8) | (*(X+3));})

/*
 * The table contains fundamental data used to program our flash, which
 * would be different according to the vendors.
 */
struct flash_table ft[] = {
	{0xEF, 0x6011, (128 * K), 256, (4 * K), (64 * K)},	/*  W25Q10EW  */
	{0xEF, 0x6012, (256 * K), 256, (4 * K), (64 * K)},	/*  W25Q20EW  */
	{0xC8, 0x6012, (256 * K), 256, (4 * K), (64 * K)},	/*  GD25LQ20B */
	{0xC8, 0x6013, (512 * K), 256, (4 * K), (64 * K)},	/*  GD25LQ40 */
	{0x85, 0x6013, (4 * M), 256, (4 * K), (64 * K)},
	{0xC2, 0x2812, (256 * K), 256, (4 * K), (64 * K)},
	{0x1C, 0x3812, (256 * K), 256, (4 * K), (64 * K)},
};

/* Store flash information */
struct flash_table *flashtab = NULL;

/*
 * the size of two arrays is different depending on
 * which of methods to upgrade firmware you choose for.
 */
u8 *flash_fw = NULL;
u8 iram_fw[MAX_IRAM_FIRMWARE_SIZE] = { 0 };

int g_update_percentage = 0;

/* the length of array in each sector */
int g_section_len = 0;
int g_total_sector_len = 0;

/* The addr of block reserved for customers */
int g_start_resrv = 0x1D000;
int g_end_resrv = 0x1DFFF;

u32 old_fw_cb;
u32 new_fw_cb;
u32 fw_end_addr;
u32 fw_start_addr;

struct flash_sector {
	u32 ss_addr;
	u32 se_addr;
	u32 checksum;
	u32 crc32;
	u32 dlength;
	bool data_flag;
	bool block_flag;
};

struct flash_block_info {
	u32 start_addr;
	u32 end_addr;
	u32 hex_crc;
	u32 block_crc;
	u32 number;
};

struct flash_sector *g_flash_sector = NULL;
struct flash_block_info g_flash_block_info[5];

static u32 ili9881h_HexToDec(const char *pHex, int32_t nLength)
{
	u32 nRetVal = 0, nTemp = 0, i;
	int32_t nShift = (nLength - 1) * 4;

	for (i = 0; i < nLength; nShift -= 4, i++) {
		if ((pHex[i] >= '0') && (pHex[i] <= '9')) {
			nTemp = pHex[i] - '0';
		} else if ((pHex[i] >= 'a') && (pHex[i] <= 'f')) {
			nTemp = (pHex[i] - 'a') + 10;
		} else if ((pHex[i] >= 'A') && (pHex[i] <= 'F')) {
			nTemp = (pHex[i] - 'A') + 10;
		} else {
			return -1;
		}

		nRetVal |= (nTemp << nShift);
	}

	return nRetVal;
}

static u32 ili9881h_calc_crc32(u32 start_addr, u32 end_addr, u8 *data)
{
	int i, j;
	u32 CRC_POLY = 0x04C11DB7;
	u32 ReturnCRC = 0xFFFFFFFF;
	u32 len = start_addr + end_addr;

	for (i = start_addr; i < len; i++) {
		ReturnCRC ^= (data[i] << 24);

		for (j = 0; j < 8; j++) {
			if ((ReturnCRC & 0x80000000) != 0) {
				ReturnCRC = ReturnCRC << 1 ^ CRC_POLY;
			} else {
				ReturnCRC = ReturnCRC << 1;
			}
		}
	}

	return ReturnCRC;
}

static u32 ili9881h_read_hw_crc(struct device *dev, u32 start_addr, u32 len)
{
	int timer = 500;
	u32 busy = 0;
	u32 iram_check = 0;

	TOUCH_I("start = 0x%x len = %x\n", start_addr, len);

	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	ili9881h_ice_reg_write(dev, 0x041008, 0x3b, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0xFF0000) >> 16, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x00FF00) >> 8, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x0000FF), 1);

	ili9881h_ice_reg_write(dev, 0x041003, 0x01, 1);	/* Enable Dio_Rx_dual */
	ili9881h_ice_reg_write(dev, 0x041008, 0xFF, 1);	/* Dummy */

	/* Set Receive count */
	ili9881h_ice_reg_write(dev, 0x04100C, len, 3);

	/* Clear Int Flag */
	ili9881h_ice_reg_write(dev, 0x048007, 0x02, 1);

	/* Checksum_En */
	ili9881h_ice_reg_write(dev, 0x041016, 0x00, 1);
	ili9881h_ice_reg_write(dev, 0x041016, 0x01, 1);

	/* Start to receive */
	ili9881h_ice_reg_write(dev, 0x041010, 0xFF, 1);

	while (timer > 0) {
		busy = ili9881h_ice_reg_read(dev, 0x048007);
		busy &= 0xFF;
		busy = busy >> 1;  // read 1 byte

		if ((busy & 0x01) == 0x01)
			break;

		timer--;
		touch_msleep(10);
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */

	if (timer >= 0) {
		/* Disable dio_Rx_dual */
		ili9881h_ice_reg_write(dev, 0x041003, 0x0, 1);
		iram_check = ili9881h_ice_reg_read(dev, 0x04101C);
	} else {
		TOUCH_E("TIME OUT\n");
		goto out;
	}

	return iram_check;

out:
	TOUCH_E("Failed to read Checksum/CRC from IC\n");
	return -1;

}

static int ili9881h_flash_read(struct device *dev, u32 start, u32 end, u8 *data, int dlen)
{
	u32 i, cont = 0;
	u32 buf;

	if (data == NULL) {
		TOUCH_E("data is null, read failed\n");
		return -1;
	}

	if (end - start > dlen) {
		TOUCH_E("the length (%d) reading crc is over than dlen(%d)\n", end - start, dlen);
		return -1;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */
	ili9881h_ice_reg_write(dev, 0x041008, 0x03, 1);

	ili9881h_ice_reg_write(dev, 0x041008, (start & 0xFF0000) >> 16, 1);
	ili9881h_ice_reg_write(dev ,0x041008, (start & 0x00FF00) >> 8, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start & 0x0000FF), 1);

	for (i = start; i <= end; i++) {
		ili9881h_ice_reg_write(dev, 0x041008, 0xFF, 1);	/* Dummy */
		buf = ili9881h_ice_reg_read(dev, 0x41010);
		data[cont] = (u8)(buf & 0xFF);
		TOUCH_I("data[%d] = %x\n", cont, data[cont]);
		cont++;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */
	return 0;
}

static int ili9881h_fw_compare(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = NO_NEED_UPDATE;
	int i, crc_byte_len = 4;
	u8 flash_crc[4] = {0};
	u32 start_addr = 0, end_addr = 0, flash_crc_cb;

	TOUCH_TRACE();

	if (flash_fw == NULL) {
		TOUCH_E("Flash data is null, ignore upgrade\n");
		return CHECK_FW_FAIL;
	}

	/* Check FW version */
	TOUCH_I("New FW ver = 0x%x, Old FW ver = 0x%x\n", new_fw_cb, old_fw_cb);

	if (ts->force_fwup) {
		TOUCH_I("Force upgrade, Don't do check crc\n");
		return NEED_UPDATE;
	}

	if (new_fw_cb != old_fw_cb) {
		TOUCH_I("New Firmware version is different from the old, need to check flash crc if it's correct\n");
		goto check_hw_crc;
	} else {
		TOUCH_I("New firmware version is same as before, need to check hw crc if it's correct\n");
		goto check_flash_crc;
	}

check_hw_crc:
	/* Check HW and Hex CRC */
	for(i = 0; i < ARRAY_SIZE(g_flash_block_info); i++) {
		start_addr = g_flash_block_info[i].start_addr;
		end_addr = g_flash_block_info[i].end_addr;

		/* Invaild end address */
		if (end_addr == 0)
			continue;

		/* Get hex(bin) CRC for each block */
		g_flash_block_info[i].hex_crc = (flash_fw[end_addr - 3] << 24) + (flash_fw[end_addr - 2] << 16) + (flash_fw[end_addr - 1] << 8) + flash_fw[end_addr];

		/* Get HW CRC for each block */
		g_flash_block_info[i].block_crc = ili9881h_read_hw_crc(dev, start_addr, end_addr - start_addr - crc_byte_len + 1);

		TOUCH_I("HW CRC = 0x%06x, HEX CRC = 0x%06x\n", g_flash_block_info[i].block_crc, g_flash_block_info[i].hex_crc);

		/* Compare HW CRC with HEX CRC */
		if(g_flash_block_info[i].hex_crc != g_flash_block_info[i].block_crc)
			ret = NEED_UPDATE;
	}

	return ret;

check_flash_crc:
	/* Check Flash CRC and HW CRC */
	for(i = 0; i < ARRAY_SIZE(g_flash_block_info); i++) {
		start_addr = g_flash_block_info[i].start_addr;
		end_addr = g_flash_block_info[i].end_addr;

		/* Invaild end address */
		if (end_addr == 0)
			continue;

		/* Get flash CRC for each block */
		ret = ili9881h_flash_read(dev, end_addr - crc_byte_len + 1, end_addr, flash_crc, sizeof(flash_crc));
		if (ret < 0) {
			TOUCH_E("Read Flash failed\n");
			return CHECK_FW_FAIL;
		}

		/* Get HW CRC for each block */
		g_flash_block_info[i].block_crc = ili9881h_read_hw_crc(dev, start_addr, end_addr - start_addr - crc_byte_len + 1);

		CRC_ONESET(flash_crc, flash_crc_cb);

		TOUCH_I("HW CRC = 0x%06x, Flash CRC = 0x%06x\n", g_flash_block_info[i].block_crc, flash_crc_cb);

		/* Compare Flash CRC with HW CRC */
		if(flash_crc_cb != g_flash_block_info[i].block_crc)
			ret = NEED_UPDATE;

		memset(flash_crc, 0, sizeof(flash_crc));
	}

	return ret;
}

static int ili9881h_do_check(struct device *dev, u32 start, u32 len)
{
	int ret = 0;
	u32 hw_crc = 0, local = 0;

	TOUCH_I("start = 0x%x, len = %d\n",start,len);

	local = ili9881h_calc_crc32(start, len, flash_fw);
	hw_crc = ili9881h_read_hw_crc(dev, start, len);
	ret = CHECK_EQUAL(hw_crc, local);

	TOUCH_I("%s (%x) : (%x)\n", (ret < 0 ? "Invalid !" : "Correct !"), hw_crc, local);

	return ret;
}

static int ili9881h_fw_verify_data(struct device *dev)
{
	int i = 0, ret = 0, len = 0;
	int fps = flashtab->sector;
	u32 ss = 0x0;

	TOUCH_TRACE();

	for (i = 0; i < g_section_len + 1; i++) {
		if (g_flash_sector[i].data_flag) {
			if (ss > g_flash_sector[i].ss_addr || len == 0)
				ss = g_flash_sector[i].ss_addr;

			len = len + g_flash_sector[i].dlength;

			/* if larger than max count, then committing data to check */
			if (len >= 0x1FFFF - fps) {
				ret = ili9881h_do_check(dev, ss, len);
				if (ret < 0)
					goto out;

				ss = g_flash_sector[i].ss_addr;
				len = 0;
			}
		} else {
			/* split flash sector and commit the last data to fw */
			if (len != 0) {
				ret = ili9881h_do_check(dev, ss, len);
				if (ret < 0)
					goto out;

				ss = g_flash_sector[i].ss_addr;
				len = 0;
			}
		}
	}

	/* it might be lower than the size of sector if calc the last array. */
	if (len != 0 && ret != -1)
		ret = ili9881h_do_check(dev, ss, fw_end_addr - ss);

out:
	return ret;
}

static int ili9881h_flash_write_enable(struct device *dev)
{
	TOUCH_TRACE();

	if (ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1) < 0)
		goto out;
	if (ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3) < 0)	/* Key */
		goto out;
	if (ili9881h_ice_reg_write(dev, 0x041008, 0x6, 1) < 0)
		goto out;
	if (ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1) < 0)
		goto out;

	return 0;

out:
	TOUCH_E("Write enable failed !\n");
	return -EIO;
}
static int ili9881h_flash_poll_busy(struct device *dev)
{
	int timer = 500, ret = 0;
	u32 buf;

	TOUCH_TRACE();

	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	ili9881h_ice_reg_write(dev, 0x041008, 0x5, 1);
	while (timer > 0) {
		ili9881h_ice_reg_write(dev, 0x041008, 0xFF, 1);

		touch_msleep(1);

		buf = ili9881h_ice_reg_read(dev, 0x041010);
		buf &= 0xFF;
		if ((buf & 0x03) == 0x00)
			goto out;

		timer--;
	}

	TOUCH_E("Polling busy Time out !\n");
	ret = -1;
out:
	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */
	return ret;
}

static int ili9881h_do_program_flash(struct device *dev, u32 start_addr)
{
	int ret = 0;
	u32 k;
	u8 buf[512] = { 0 };
	u32 addr = 0;
	int current_percentage = 0;

	TOUCH_TRACE();

	ret = ili9881h_flash_write_enable(dev);
	if (ret < 0) {
		TOUCH_E("Failed to config write enable\n");
		goto out;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	ili9881h_ice_reg_write(dev, 0x041008, 0x02, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0xFF0000) >> 16, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x00FF00) >> 8, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x0000FF), 1);

	addr = 0x041008;
	buf[0] = (char)((addr & 0x000000FF) >> 0);
	buf[1] = (char)((addr & 0x0000FF00) >> 8);
	buf[2] = (char)((addr & 0x00FF0000) >> 16);

	for (k = 0; k < flashtab->program_page; k++) {
		if (start_addr + k <= fw_end_addr)
			buf[3 + k] = flash_fw[start_addr + k];
		else
			buf[3 + k] = 0xFF;
	}

	if (ili9881h_reg_write(dev, CMD_GET_MCU_INTERNAL_DATA, buf, flashtab->program_page + 3) < 0) {
	//if (ili9881h_ice_reg_write(dev, 0x041008, buf, flashtab->program_page) < 0) {
		TOUCH_E("Failed to write data at start_addr = 0x%X, k = 0x%X, addr = 0x%x\n",
			start_addr, k, start_addr + k);
		ret = -EIO;
		goto out;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */

	ret = ili9881h_flash_poll_busy(dev);
	if (ret < 0)
		goto out;

	current_percentage = (start_addr * 101) / fw_end_addr;
	/* Don't use TOUCH_I to print log because it needs to be kpet in the same line */
	if (g_update_percentage != current_percentage)
		TOUCH_I("Upgrading firmware ... start_addr = 0x%x, %02d%c\n", start_addr, current_percentage,'%');
	g_update_percentage = current_percentage;

out:
	return ret;
}

static int ili9881h_flash_program_sector(struct device *dev)
{
	int i, j, ret = 0;

	TOUCH_TRACE();

	for (i = 0; i < g_section_len + 1; i++) {
		/*
		 * If running the boot stage, fw will only be upgrade data with the flag of block,
		 * otherwise data with the flag itself will be programed.
		 */
		// if (core_firmware->isboot) {
		// 	if (!g_flash_sector[i].block)
		// 		continue;
		// } else {
		if (!g_flash_sector[i].data_flag)
			continue;
		// }

		/* programming flash by its page size */
		for (j = g_flash_sector[i].ss_addr; j < g_flash_sector[i].se_addr; j += flashtab->program_page) {
			 if (j > fw_end_addr)
			 	goto out;

			ret = ili9881h_do_program_flash(dev, j);
			if (ret < 0)
				goto out;
		}
	}

out:
	return ret;
}

static int ili9881h_do_erase_flash(struct device *dev, u32 start_addr)
{
	int ret = 0;
	u32 temp_buf = 0;

	TOUCH_TRACE();

	ret = ili9881h_flash_write_enable(dev);
	if (ret < 0) {
		TOUCH_E("Failed to config write enable\n");
		goto out;
	}

	/* Erase the flash sector */
	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	ili9881h_ice_reg_write(dev, 0x041008, 0x20, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0xFF0000) >> 16, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x00FF00) >> 8, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x0000FF), 1);

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */

	touch_msleep(1);

	ret = ili9881h_flash_poll_busy(dev);
	if (ret < 0)
		goto out;

	/* check the flash sector is 0xFF (erased) */
	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	ili9881h_ice_reg_write(dev, 0x041008, 0x3, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0xFF0000) >> 16, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x00FF00) >> 8, 1);
	ili9881h_ice_reg_write(dev, 0x041008, (start_addr & 0x0000FF), 1);
	ili9881h_ice_reg_write(dev, 0x041008, 0xFF, 1);

	temp_buf = ili9881h_ice_reg_read(dev, 0x041010);
	temp_buf &= 0xFF; // Only read 1 byte
	if (temp_buf != 0xFF) {
		TOUCH_E("Failed to erase data(0x%x) at 0x%x\n", temp_buf, start_addr);
		ret = -EINVAL;
		goto out;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */

	TOUCH_I("Earsing data at start addr: %x\n", start_addr);

out:
	return ret;
}

static int ili9881h_flash_erase_sector(struct device *dev)
{
	int i, ret = 0;

	TOUCH_TRACE();

	for (i = 0; i < g_total_sector_len; i++) {
		// if (core_firmware->isboot) {
		// 	if (!g_flash_sector[i].block)
		// 		continue;
		// } else {
		if (!g_flash_sector[i].data_flag && !g_flash_sector[i].block_flag)
			continue;
		//}

		ret = ili9881h_do_erase_flash(dev, g_flash_sector[i].ss_addr);
		if (ret < 0)
			goto out;
	}

out:
	return ret;
}


static void ili9881h_flash_enable_protect(struct device *dev, bool enable)
{
	TOUCH_I("Set flash protect as (%d)\n", enable);

	if (ili9881h_flash_write_enable(dev) < 0) {
		TOUCH_E("Failed to config flash's write enable\n");
		return;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);	/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	switch (flashtab->mid) {
	case 0xEF:
		if (flashtab->dev_id == 0x6012 || flashtab->dev_id == 0x6011) {
			ili9881h_ice_reg_write(dev, 0x041008, 0x1, 1);
			ili9881h_ice_reg_write(dev, 0x041008, 0x00, 1);

			if (enable)
				ili9881h_ice_reg_write(dev, 0x041008, 0x7E, 1);
			else
				ili9881h_ice_reg_write(dev, 0x041008, 0x00, 1);
		}
		break;
	case 0xC8:
		if (flashtab->dev_id == 0x6012 || flashtab->dev_id == 0x6013) {
			ili9881h_ice_reg_write(dev, 0x041008, 0x1, 1);
			ili9881h_ice_reg_write(dev, 0x041008, 0x00, 1);

			if (enable)
				ili9881h_ice_reg_write(dev, 0x041008, 0x7A, 1);
			else
				ili9881h_ice_reg_write(dev, 0x041008, 0x00, 1);
		}
		break;
	default:
		TOUCH_E("Can't find flash id, ignore protection\n");
		break;
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);	/* CS high */
	touch_msleep(5);
}

static int ili9881h_do_fw_upgrade(struct device *dev)
{
	int ret = 0;
	struct touch_core_data *ts = to_touch_core(dev);

	TOUCH_TRACE();

	g_update_percentage = -1;

	//ili9881h_reset_ctrl(dev, HW_RESET_ONLY);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	ret = ili9881h_ice_mode_enable(dev);
	if (ret < 0) {
		TOUCH_E("Failed to enable ICE mode\n");
		return ret;
	}

	touch_msleep(25);

	/* Check FW Version & CRC correction */
	ret = ili9881h_fw_compare(dev);
	if (ret == NEED_UPDATE) {
		TOUCH_I("FW CRC is different doing upgrade or Force fw update \n");
	} else if (ret == NO_NEED_UPDATE) {
		TOUCH_I("FW CRC is the same, doing nothing\n");
		goto out;
	} else {
		TOUCH_E("FW check is incorrect, unexpected errors\n");
		goto out;
	}

	/* Disable flash protection from being written */
	ili9881h_flash_enable_protect(dev, false);

	ret = ili9881h_flash_erase_sector(dev);
	if (ret < 0) {
		TOUCH_E("Failed to erase flash\n");
		goto out;
	}

	ret = ili9881h_flash_program_sector(dev);
	if (ret < 0) {
		TOUCH_E("Failed to program flash\n");
		goto out;
	}

	/* We do have to reset chip in order to move new code from flash to Touch IC ram. */
	ili9881h_reset_ctrl(dev, HW_RESET_ONLY);

	ret = ili9881h_ice_mode_enable(dev);
	if (ret < 0) {
		TOUCH_E("Failed to enable ICE mode\n");
		goto out;
	}

	touch_msleep(25);

	/* check the data that we've just written into the iram. */
	ret = ili9881h_fw_verify_data(dev);
	if (ret == 0)
		TOUCH_I("Data Correct !\n");

	ret = ili9881h_ice_mode_disable(dev);
	if (ret < 0) {
		TOUCH_E("Failed to disable ICE mode\n");
		goto out;
	}

out:
	return ret;
}

static int ili9881h_convert_hex_file(struct device *dev, const u8 *pBuf, u32 nSize)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int index = 0, block = 0;
	u32 i = 0, j = 0, k = 0;
	u32 nLength = 0, nAddr = 0, nType = 0;
	u32 nStartAddr = 0x0, nEndAddr = 0x0, nChecksum = 0x0, nExAddr = 0;
	u32 tmp_addr = 0x0;
	bool has_block_info = false;

	TOUCH_TRACE();

	memset(g_flash_block_info, 0x0, sizeof(g_flash_block_info));

	/* Parsing HEX file */
	for (; i < nSize;) {
		int32_t nOffset;

		nLength = ili9881h_HexToDec(&pBuf[i + 1], 2);
		nAddr = ili9881h_HexToDec(&pBuf[i + 3], 4);
		nType = ili9881h_HexToDec(&pBuf[i + 7], 2);

		/* calculate checksum */
		for (j = 8; j < (2 + 4 + 2 + (nLength * 2)); j += 2) {
			if (nType == 0x00) {
				/* for ice mode write method */
				nChecksum = nChecksum + ili9881h_HexToDec(&pBuf[i + 1 + j], 2);
			}
		}

		if (nType == 0x04) {
			nExAddr = ili9881h_HexToDec(&pBuf[i + 9], 4);
		}

		if (nType == 0x02) {
			nExAddr = ili9881h_HexToDec(&pBuf[i + 9], 4);
			nExAddr = nExAddr >> 12;
		}

		if (nType == 0xAE || nType == 0xAF) {

			has_block_info = true;
			/* insert block info extracted from hex */
			if (block < 5) {
				g_flash_block_info[block].start_addr = ili9881h_HexToDec(&pBuf[i + 9], 6);
				g_flash_block_info[block].end_addr = ili9881h_HexToDec(&pBuf[i + 9 + 6], 6);
				TOUCH_I("Block[%d]: start_addr = %x, end = %x\n",
				    block, g_flash_block_info[block].start_addr, g_flash_block_info[block].end_addr);
				if (nType == 0xAF) {
					g_flash_block_info[block].number = ili9881h_HexToDec(&pBuf[i + 9 + 6 + 6], 2);
					TOUCH_I("Block[%d]: number = %x\n", block, g_flash_block_info[block].number);

				}
			}
			block++;
		}

		nAddr = nAddr + (nExAddr << 16);
		if (pBuf[i + 1 + j + 2] == 0x0D) {
			nOffset = 2;
		} else {
			nOffset = 1;
		}

		if (nType == 0x00) {
			if (nAddr > MAX_HEX_FILE_SIZE) {
				TOUCH_E("Invalid hex format\n");
				goto out;
			}

			if (nAddr < nStartAddr) {
				nStartAddr = nAddr;
			}
			if ((nAddr + nLength) > nEndAddr) {
				nEndAddr = nAddr + nLength;
			}

			/* fill data */
			for (j = 0, k = 0; j < (nLength * 2); j += 2, k++) {
				flash_fw[nAddr + k] = ili9881h_HexToDec(&pBuf[i + 9 + j], 2);
				if ((nAddr + k) != 0) {
					index = ((nAddr + k) / flashtab->sector);
					if (!g_flash_sector[index].data_flag) {
						g_flash_sector[index].ss_addr = index * flashtab->sector;
						g_flash_sector[index].se_addr = (index + 1) * flashtab->sector - 1;
						g_flash_sector[index].dlength = (g_flash_sector[index].se_addr - g_flash_sector[index].ss_addr) + 1;
						g_flash_sector[index].data_flag = true;
					}
				}
			}
		}
		i += 1 + 2 + 4 + 2 + (nLength * 2) + 2 + nOffset;
	}

	/* store old fw ver temparaly before upgrade */
	old_fw_cb = (d->ic_info.fw_info.core << 24) | (d->ic_info.fw_info.customer_code << 16) |
		(d->ic_info.fw_info.major << 8) | d->ic_info.fw_info.minor;
	TOUCH_I("FW old ver = 0x%x\n", old_fw_cb);

	/* Get hex fw ver */
	new_fw_cb = (flash_fw[FW_VER_ADDR] << 24) | (flash_fw[FW_VER_ADDR + 1] << 16) |
		(flash_fw[FW_VER_ADDR + 2] << 8) | (flash_fw[FW_VER_ADDR + 3]);
	TOUCH_I("New FW ver = 0x%x\n", new_fw_cb);

	/* Update the length of section */
	g_section_len = index;

	if (g_flash_sector[g_section_len - 1].se_addr > flashtab->mem_size) {
		TOUCH_E("The size written to flash is larger than it required (%x) (%x)\n",
			g_flash_sector[g_section_len - 1].se_addr, flashtab->mem_size);
		goto out;
	}

	for (i = 0; i < g_total_sector_len; i++) {
		/* fill meaing address in an array where is empty */
		if (g_flash_sector[i].ss_addr == 0x0 && g_flash_sector[i].se_addr == 0x0) {
			g_flash_sector[i].ss_addr = tmp_addr;
			g_flash_sector[i].se_addr = (i + 1) * flashtab->sector - 1;
		}

		tmp_addr += flashtab->sector;

		/* set erase flag in the block if the addr of sectors is between them. */
		if (has_block_info) {
			for (j = 0; j < ARRAY_SIZE(g_flash_block_info); j++) {
				if (g_flash_sector[i].ss_addr >= g_flash_block_info[j].start_addr
				    && g_flash_sector[i].se_addr <= g_flash_block_info[j].end_addr) {
					g_flash_sector[i].block_flag = true;
					break;
				}
			}
		}
	}

	/* DEBUG: for showing data with address that will write into fw or be erased
	for (i = 0; i < g_total_sector_len; i++) {
		TOUCH_I("sector[%d]: start = 0x%x, end = 0x%x, length = %x, data = %d, block = %d\n", i,
		    g_flash_sector[i].ss_addr, g_flash_sector[i].se_addr, g_flash_sector[index].dlength,
		    g_flash_sector[i].data_flag, g_flash_sector[i].block_flag);
	}*/

	TOUCH_I("nStartAddr = 0x%06X, nEndAddr = 0x%06X\n", nStartAddr, nEndAddr);
	fw_end_addr = nEndAddr;
	fw_start_addr = nStartAddr;
	return 0;

out:
	TOUCH_E("Failed to convert HEX data\n");
	return -1;
}

int ili9881h_fw_upgrade(struct device *dev, const struct firmware *fw)
{
	int ret = 0;

	TOUCH_TRACE();

	if (flashtab == NULL) {
		TOUCH_E("Flash table isn't created, upgrade fail\n");
		ret = -ENOMEM;
		goto out;
	}

	flash_fw = kcalloc(flashtab->mem_size, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(flash_fw)) {
		TOUCH_E("Failed to allocate flash_fw memory, %ld\n", PTR_ERR(flash_fw));
		ret = -ENOMEM;
		goto out;
	}
	memset(flash_fw, 0xff, sizeof(u8) * flashtab->mem_size);

	g_total_sector_len = flashtab->mem_size / flashtab->sector;
	if (g_total_sector_len <= 0) {
		TOUCH_E("Flash configure is wrong\n");
		ret = -1;
		goto out;
	}

	g_flash_sector = kcalloc(g_total_sector_len, sizeof(*g_flash_sector), GFP_KERNEL);
	if (ERR_ALLOC_MEM(g_flash_sector)) {
		TOUCH_E("Failed to allocate g_flash_sector memory, %ld\n", PTR_ERR(g_flash_sector));
		ret = -ENOMEM;
		goto out;
	}

	ret = ili9881h_convert_hex_file(dev, fw->data, fw->size);
	if (ret < 0) {
		TOUCH_E("Failed to covert firmware data, ret = %d\n", ret);
		goto out;
	}

	ret = ili9881h_do_fw_upgrade(dev);
	if (ret < 0) {
		TOUCH_E("Failed to upgrade firmware, ret = %d\n", ret);
		goto out;
	}

	TOUCH_I("Update TP/Firmware information...\n");
out:
	//core_firmware->isUpgrading = false;
	ipio_kfree((void **)&g_flash_sector);
	ipio_kfree((void **)&flash_fw);
	ipio_kfree((void **)&g_flash_sector);
	return ret;
}

static void ili9881h_store_flash_info(struct device *dev, u16 mid, u16 dev_id)
{
	int i = 0;

	TOUCH_I("M_ID = %x, DEV_ID = %x", mid, dev_id);

	flashtab = devm_kzalloc(dev, sizeof(ft), GFP_KERNEL);
	if (ERR_ALLOC_MEM(flashtab)) {
		TOUCH_E("Failed to allocate flashtab memory, %ld\n", PTR_ERR(flashtab));
		return;
	}

	for (; i < ARRAY_SIZE(ft); i++) {
		if (mid == ft[i].mid && dev_id == ft[i].dev_id) {
			TOUCH_I("Find them in flash table\n");

			flashtab->mid = mid;
			flashtab->dev_id = dev_id;
			flashtab->mem_size = ft[i].mem_size;
			flashtab->program_page = ft[i].program_page;
			flashtab->sector = ft[i].sector;
			flashtab->block = ft[i].block;
			break;
		}
	}

	if (i >= ARRAY_SIZE(ft)) {
		TOUCH_E("Can't find them in flash table, apply default flash config\n");
		flashtab->mid = mid;
		flashtab->dev_id = dev_id;
		flashtab->mem_size = (256 * K);
		flashtab->program_page = 256;
		flashtab->sector = (4 * K);
		flashtab->block = (64 * K);
	}

	TOUCH_I("Max Memory size = %d\n", flashtab->mem_size);
	TOUCH_I("Per program page = %d\n", flashtab->program_page);
	TOUCH_I("Sector size = %d\n", flashtab->sector);
	TOUCH_I("Block size = %d\n", flashtab->block);
}

int ili9881h_read_flash_info(struct device *dev, u8 cmd, int len)
{
	int i, ret = 0;
	u16 flash_id = 0, flash_mid = 0;
	u8 buf[4] = { 0 };

	ret = ili9881h_ice_mode_enable(dev);
	if (ret < 0) {
		TOUCH_E("Failed to enter ICE mode, ret = %d\n", ret);
		goto out;
	}

	touch_msleep(20);

	ili9881h_ice_reg_write(dev, 0x041000, 0x0, 1);		/* CS low */
	ili9881h_ice_reg_write(dev, 0x041004, 0x66AA55, 3);	/* Key */

	ili9881h_ice_reg_write(dev, 0x041008, cmd, 1);
	for (i = 0; i < len; i++) {
		ili9881h_ice_reg_write(dev, 0x041008, 0xFF, 1);
		buf[i] = ili9881h_ice_reg_read(dev, 0x041010);
	}

	ili9881h_ice_reg_write(dev, 0x041000, 0x1, 1);		/* CS high */

	/* look up flash info and init its struct after obtained flash id. */
	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];

	ili9881h_store_flash_info(dev, flash_mid, flash_id);

out:
	ret = ili9881h_ice_mode_disable(dev);

	return ret;
}
