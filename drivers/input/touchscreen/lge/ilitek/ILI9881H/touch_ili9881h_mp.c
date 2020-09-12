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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>

#if defined(CONFIG_LGE_TOUCH_CORE_QCT)
#include <soc/qcom/lge/board_lge.h>
#endif
#if defined(CONFIG_LGE_TOUCH_CORE_MTK)
#include <soc/mediatek/lge/board_lge.h>
#endif
#include <touch_core.h>
#include <touch_hwif.h>

#include "touch_ili9881h.h"
#include "touch_ili9881h_mp.h"

int g_ini_items = 0;

#define Mathabs(x) ({						\
		long ret;							\
		if (sizeof(x) == sizeof(long)) {	\
		long __x = (x);						\
		ret = (__x < 0) ? -__x : __x;		\
		} else {							\
		int __x = (x);						\
		ret = (__x < 0) ? -__x : __x;		\
		}									\
		ret;								\
		})

struct mp_test_items tItems[] = {
	{.name = "mutual_dac", .desp = "Calibration Data(DAC)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_no_bk", .desp = "Raw Data(No BK)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_no_bk_lcm_off", .desp = "Raw Data(No BK) (LCM OFF)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_has_bk", .desp = "Raw Data(Have BK)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_has_bk_lcm_off", .desp = "Raw Data(Have BK) (LCM OFF)", .result = "FAIL", .catalog = MUTUAL_TEST},

	{.name = "rx_short", .desp = "Short Test -ILI9881", .result = "FAIL", .catalog = SHORT_TEST},
	{.name = "open_integration_sp", .desp = "Open Test(integration)_SP", .result = "FAIL", .catalog = OPEN_TEST},

	{.name = "noise_peak_to_peak_ic", .desp = "Noise Peak to Peak(IC Only)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_panel", .desp = "Noise Peak To Peak(With Panel)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_ic_lcm_off", .desp = "Noise Peak to Peak(IC Only) (LCM OFF)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_panel_lcm_off", .desp = "Noise Peak To Peak(With Panel) (LCM OFF)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "doze_raw_td_lcm_off", .desp = "Raw Data_TD (LCM OFF)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "doze_p2p_td_lcm_off", .desp = "Peak To Peak_TD (LCM OFF)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
};

int32_t *frame_buf = NULL;
int32_t *frame1_cbk700 = NULL, *frame1_cbk250 = NULL, *frame1_cbk200 = NULL;

struct core_mp_test_data *core_mp = NULL;
struct ini_file_data ini_data[PARSER_MAX_KEY_NUM];

static void write_file(struct device *dev, char *data, bool write_time)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[TIME_STR_LEN] = {0};
	struct timespec my_time;
	struct tm my_date;
	int boot_mode = 0;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
		case TOUCH_NORMAL_BOOT:
			fname = "/sdcard/touch_self_test.txt";
			break;
		case MINIOS_AAT:
			fname = "/data/touch/touch_self_test.txt";
			break;
		case MINIOS_MFTS_FOLDER:
		case MINIOS_MFTS_FLAT:
		case MINIOS_MFTS_CURVED:
			fname = "/data/touch/touch_self_mfts.txt";
			break;
		default:
			TOUCH_I("%s : not support mode\n", __func__);
			break;
	}

	if (fname) {
		fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n", __func__);
		set_fs(old_fs);
		return;
	}

	if (fd >= 0) {
		if (write_time) {
			my_time = current_kernel_time();
			time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
			snprintf(time_string, TIME_STR_LEN,
				"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
				my_date.tm_mon + 1,
				my_date.tm_mday, my_date.tm_hour,
				my_date.tm_min, my_date.tm_sec,
				(unsigned long) my_time.tv_nsec / 1000000);
			sys_write(fd, time_string, strlen(time_string));
		}
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	} else {
		TOUCH_I("File open failed\n");
	}
	set_fs(old_fs);
}

static void log_file_size_check(struct device *dev)
{
	char *fname = NULL;
	struct file *file;
	loff_t file_size = 0;
	int i = 0;
	char buf1[128] = {0};
	char buf2[128] = {0};
	mm_segment_t old_fs = get_fs();
	int ret = 0;
	int boot_mode = 0;

	set_fs(KERNEL_DS);

	boot_mode = touch_boot_mode_check(dev);

	switch (boot_mode) {
	case TOUCH_NORMAL_BOOT:
		fname = "/sdcard/touch_self_test.txt";
		break;
	case MINIOS_AAT:
		fname = "/data/touch/touch_self_test.txt";
		break;
	case MINIOS_MFTS_FOLDER:
	case MINIOS_MFTS_FLAT:
	case MINIOS_MFTS_CURVED:
		fname = "/data/touch/touch_self_mfts.txt";
		break;
	default:
		TOUCH_I("%s : not support mode\n", __func__);
		break;
	}

	if (fname) {
		file = filp_open(fname, O_RDONLY, 0666);
		sys_chmod(fname, 0666);
	} else {
		TOUCH_E("%s : fname is NULL, can not open FILE\n",
				__func__);
		goto error;
	}

	if (IS_ERR(file)) {
		TOUCH_I("%s : ERR(%ld) Open file error [%s]\n",
				__func__, PTR_ERR(file), fname);
		goto error;
	}

	file_size = vfs_llseek(file, 0, SEEK_END);
	TOUCH_I("%s : [%s] file_size = %lld\n",
			__func__, fname, file_size);

	filp_close(file, 0);

	if (file_size > MAX_LOG_FILE_SIZE) {
		TOUCH_I("%s : [%s] file_size(%lld) > MAX_LOG_FILE_SIZE(%d)\n",
				__func__, fname, file_size, MAX_LOG_FILE_SIZE);

		for (i = MAX_LOG_FILE_COUNT - 1; i >= 0; i--) {
			if (i == 0)
				sprintf(buf1, "%s", fname);
			else
				sprintf(buf1, "%s.%d", fname, i);

			ret = sys_access(buf1, 0);

			if (ret == 0) {
				TOUCH_I("%s : file [%s] exist\n",
						__func__, buf1);

				if (i == (MAX_LOG_FILE_COUNT - 1)) {
					if (sys_unlink(buf1) < 0) {
						TOUCH_E("%s : failed to remove file [%s]\n",
								__func__, buf1);
						goto error;
					}

					TOUCH_I("%s : remove file [%s]\n",
							__func__, buf1);
				} else {
					sprintf(buf2, "%s.%d",
							fname,
							(i + 1));

					if (sys_rename(buf1, buf2) < 0) {
						TOUCH_E("%s : failed to rename file [%s] -> [%s]\n",
								__func__, buf1, buf2);
						goto error;
					}

					TOUCH_I("%s : rename file [%s] -> [%s]\n",
							__func__, buf1, buf2);
				}
			} else {
				TOUCH_I("%s : file [%s] does not exist (ret = %d)\n",
						__func__, buf1, ret);
			}
		}
	}

error:
	set_fs(old_fs);
	return;
}

static int katoi(char *string)
{
	int result = 0;
	unsigned int digit;
	int sign;

	if (*string == '-') {
		sign = 1;
		string += 1;
	} else {
		sign = 0;
		if (*string == '+') {
			string += 1;
		}
	}

	for (;; string += 1) {
		digit = *string - '0';
		if (digit > 9)
			break;
		result = (10 * result) + digit;
	}

	if (sign) {
		return -result;
	}
	return result;
}

static int isspace_t(int x)
{
	if(x==' '||x=='\t'||x=='\n'||x=='\f'||x=='\b'||x=='\r')
		return 1;
	else
		return 0;
}

static char *ili9881h_ini_str_trim_r(char *buf)
{
	int len, i;
	char tmp[512] = { 0 };

	len = strlen(buf);

	for (i = 0; i < len; i++) {
		if (buf[i] != ' ')
			break;
	}

	if (i < len) {
			strncpy(tmp, (buf + i), sizeof(tmp) - 1);
	}

	strncpy(buf, tmp, len);
	return buf;
}

/* Count the number of each line and assign the content to tmp buffer */
static int ili9881h_get_ini_phy_line(const char *data, char *buffer, int maxlen)
{
	int i = 0;
	int j = 0;
	int iRetNum = -1;
	char ch1 = '\0';

	TOUCH_TRACE();

	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = data[j];
		iRetNum = j + 1;
		if (ch1 == '\n' || ch1 == '\r') {	/* line end */
			ch1 = data[j + 1];
			if (ch1 == '\n' || ch1 == '\r') {
				iRetNum++;
			}

			break;
		} else if (ch1 == 0x0) {
			//iRetNum = -1;
			break;	/* file end */
		}

		buffer[i++] = ch1;
	}

	buffer[i] = '\0';
	return iRetNum;
}

/* get_ini_key_value - get ini's key and value based on its section from its array
 *
 * A function is digging into the key and value by its section from the ini array.
 * The comparsion is not only a string's name, but its length.
 */
static int ili9881h_get_ini_key_value(char *section, char *key, char *value)
{
	int i = 0;
	int ret = -1;

	TOUCH_TRACE();

	for (i = 0; i < g_ini_items; i++) {
		if (strncmp(section, ini_data[i].pSectionName, strlen(section)) != 0)
			continue;

		if (strncmp(key, ini_data[i].pKeyName, strlen(key)) == 0) {
			memcpy(value, ini_data[i].pKeyValue, ini_data[i].iKeyValueLen);
			TOUCH_I(" key:%s, value:%s pKeyValue: %s\n", key, value, ini_data[i].pKeyValue);
			ret = 0;
			break;
		}
	}

	return ret;
}

int ili9881h_get_ini_u8_data(char *key, u8 *cmd, int len)
{
	char *s = key;
	char *pToken;
	int ret, count = 0;
	long s_to_long = 0;

	TOUCH_TRACE();

	if(isspace_t((int)(unsigned char)*s) == 0) {
		while((pToken = strsep(&s, ",")) != NULL) {
			if(count > len)
				break;

			ret = kstrtol(pToken, 0, &s_to_long);
			if(ret == 0)
				cmd[count] = s_to_long;
			else
				TOUCH_I("convert string too long, ret = %d\n", ret);
			count++;
		}
	}

	return count;
}

int ili9881h_get_ini_int_data(char *section, char *keyname, char *rv)
{
	int len = 0;
	char value[PARSER_MAX_KEY_VALUE_LEN] = {0, };

	TOUCH_TRACE();

	if (rv == NULL || section == NULL || keyname == NULL) {
		TOUCH_E("Parameters are invalid\n");
		return -EINVAL;
	}

	/* return a white-space string if get nothing */
	if (ili9881h_get_ini_key_value(section, keyname, value) < 0) {
		sprintf(rv, "%s", value);
		return 0;
	}

	len = sprintf(rv, "%s", value);
	return len;
}

void ili9881h_parse_nodetype(int32_t* type_ptr, char *desp)
{

	int i = 0, j = 0, index1 =0, temp, count = 0;
	char str[512] = { 0 }, record = ',';

	for (i = 0; i < g_ini_items; i++) {

		if ((strstr(ini_data[i].pSectionName, desp) <= 0) ||
				strcmp(ini_data[i].pKeyName, NODE_TYPE_KEY_NAME) != 0) {
			continue;
		}

		record = ',';
		for(j=0, index1 = 0; j <= ini_data[i].iKeyValueLen; j++) {
			if(ini_data[i].pKeyValue[j] == ';' || j == ini_data[i].iKeyValueLen){

				if(record != '.') {
					memset(str, 0 ,sizeof(str));
					memcpy(str, &ini_data[i].pKeyValue[index1], (j -index1));
					temp=katoi(str);
					type_ptr[count] = temp;
					//printk("%04d,",temp);
					count++;
				}
				record = ini_data[i].pKeyValue[j];
				index1 = j+1;
			}
		}
		//printk("\n");
	}
}

void ili9881h_parse_benchmark(int32_t* max_ptr, int32_t* min_ptr, int8_t type, char *desp)
{
	int i = 0, j = 0, index1 = 0, temp, count = 0;
	char str[512] = { 0 }, record = ',';
	int32_t data[4];
	char benchmark_str[256] ={0};

	TOUCH_TRACE();

	/* format complete string from the name of section "_Benchmark_Data". */
	sprintf(benchmark_str, "%s%s", desp, "_Benchmark_Data");

	for (i = 0; i < g_ini_items; i++) {

		if ((strcmp(ini_data[i].pSectionName, benchmark_str) != 0) ||
				strcmp(ini_data[i].pKeyName, BENCHMARK_KEY_NAME) != 0)
			continue;

		record = ',';

		for(j = 0, index1 = 0; j <= ini_data[i].iKeyValueLen; j++) {
			if(ini_data[i].pKeyValue[j] == ',' || ini_data[i].pKeyValue[j] == ';' ||
					ini_data[i].pKeyValue[j] == '.'|| j == ini_data[i].iKeyValueLen) {

				if(record != '.') {
					memset(str, 0, sizeof(str));
					memcpy(str, &ini_data[i].pKeyValue[index1], (j - index1));
					temp = katoi(str);
					data[(count % 4)] = temp;

					/* Over boundary, end the calc. */
					if ((count / 4 ) >= core_mp->frame_len) {
						TOUCH_I("count (%d) is larger than len (%d), break\n", (count/4), core_mp->frame_len);
						break;
					}

					if ((count % 4) == 3) {
						if (data[0] == 1) {
							if (type == TYPE_OPTION_VALUE) {
								max_ptr[count/4] = data[1] + data[2];
								min_ptr[count/4] = data[1] + data[3];
							} else {
								max_ptr[count/4] = data[1] + (data[1] * data[2]) / 100;
								min_ptr[count/4] = data[1] - (data[1] * data[3]) / 100;
							}
						} else {
							max_ptr[count/4] = INT_MAX;
							min_ptr[count/4] = INT_MIN;
						}
					}
					count++;
				}
				record = ini_data[i].pKeyValue[j];
				index1 = j + 1;
			}
		}
	}
}

static int ili9881h_get_ini_phy_data(const u8 *data, int size)
{
	int i, n = 0, ret = 0 , banchmark_flag = 0, empty_section, nodetype_flag = 0;
	int offset = 0, isEqualSign = 0;
	char *ini_buf = NULL, *tmpSectionName = NULL;
	char M_CFG_SSL = '[';
	char M_CFG_SSR = ']';
	/* char M_CFG_NIS = ':'; */
	char M_CFG_NTS = '#';
	char M_CFG_EQS = '=';

	TOUCH_TRACE();

	if (data == NULL) {
		TOUCH_E("INI data is NULL\n");
		ret = -EINVAL;
		goto out;
	}

	ini_buf = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ini_buf)) {
		TOUCH_E("Failed to allocate ini_buf memory, %ld\n", PTR_ERR(ini_buf));
		ret = -ENOMEM;
		goto out;
	}

	tmpSectionName = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
	if (ERR_ALLOC_MEM(tmpSectionName)) {
		TOUCH_E("Failed to allocate tmpSectionName memory, %ld\n", PTR_ERR(tmpSectionName));
		ret = -ENOMEM;
		goto out;
	}

	while (true) {
		banchmark_flag = 0;
		empty_section = 0;
		nodetype_flag = 0;

		if (g_ini_items > PARSER_MAX_KEY_NUM) {
			TOUCH_I("MAX_KEY_NUM: Out of length\n");
			goto out;
		}

		if(offset >= size)
			goto out;/*over size*/

		n = ili9881h_get_ini_phy_line(data + offset, ini_buf, PARSER_MAX_CFG_BUF);

		if (n < 0) {
			TOUCH_I("End of Line\n");
			goto out;
		}

		offset += n;

		n = strlen(ili9881h_ini_str_trim_r(ini_buf));

		if (n == 0 || ini_buf[0] == M_CFG_NTS)
			continue;

		/* Get section names */
		if (n > 2 && ((ini_buf[0] == M_CFG_SSL && ini_buf[n - 1] != M_CFG_SSR))) {
			TOUCH_E("Bad Section: %s\n", ini_buf);
			ret = -EINVAL;
			goto out;
		} else {
			if (ini_buf[0] == M_CFG_SSL) {
				ini_data[g_ini_items].iSectionNameLen = n - 2;
				if (ini_data[g_ini_items].iSectionNameLen > PARSER_MAX_KEY_NAME_LEN) {
					TOUCH_E("MAX_KEY_NAME_LEN: Out Of Length\n");
					ret = INI_ERR_OUT_OF_LINE;
					goto out;
				}

				ini_buf[n - 1] = 0x00;
				strcpy((char *)tmpSectionName, ini_buf + 1);
				banchmark_flag = 0;
				nodetype_flag = 0;
				TOUCH_D(PRODUCTION, "Section Name: %s, Len: %d, offset = %d\n", tmpSectionName, n - 2, offset);
				continue;
			}
		}

		/* copy section's name without square brackets to its real buffer */
		//strncpy(ini_data[g_ini_items].pSectionName, tmpSectionName, strlen(tmpSectionName));
		//CHECK
		strlcpy(ini_data[g_ini_items].pSectionName, tmpSectionName, sizeof(ini_data[g_ini_items].pSectionName));
		ini_data[g_ini_items].iSectionNameLen = strlen(tmpSectionName);

		isEqualSign = 0;
		for (i = 0; i < n; i++) {
			if (ini_buf[i] == M_CFG_EQS) {
				isEqualSign = i;
				break;
			}
			if(ini_buf[i] == M_CFG_SSL || ini_buf[i] == M_CFG_SSR){
				empty_section = 1;
				break;
			}
		}

		if (isEqualSign == 0) {
			if(empty_section)
				continue;

			if (strstr(ini_data[g_ini_items].pSectionName, "Benchmark_Data") > 0){
				banchmark_flag = 1;
				isEqualSign =-1;
			} else if (strstr(ini_data[g_ini_items].pSectionName, "Node Type") > 0){
				nodetype_flag = 1;
				isEqualSign =-1;
			} else {
				continue;
			}
		}

		if(banchmark_flag) {
			/* Get Key names */
			ini_data[g_ini_items].iKeyNameLen = strlen(BENCHMARK_KEY_NAME);
			strcpy(ini_data[g_ini_items].pKeyName, BENCHMARK_KEY_NAME);
			ini_data[g_ini_items].iKeyValueLen = n;
		} else if(nodetype_flag) {
			/* Get Key names */
			ini_data[g_ini_items].iKeyNameLen = strlen(NODE_TYPE_KEY_NAME);
			strcpy(ini_data[g_ini_items].pKeyName, NODE_TYPE_KEY_NAME);
			ini_data[g_ini_items].iKeyValueLen = n;
		} else {
			/* Get Key names */
			ini_data[g_ini_items].iKeyNameLen = isEqualSign;
			if (ini_data[g_ini_items].iKeyNameLen > PARSER_MAX_KEY_NAME_LEN) {
				/* ret = CFG_ERR_OUT_OF_LEN; */
				TOUCH_E("MAX_KEY_NAME_LEN: Out Of Length\n");
				ret = INI_ERR_OUT_OF_LINE;
				goto out;
			}

			memcpy(ini_data[g_ini_items].pKeyName,
					ini_buf, ini_data[g_ini_items].iKeyNameLen);
			ini_data[g_ini_items].iKeyValueLen = n - isEqualSign - 1;
		}

		/* Get a value assigned to a key */
		if (ini_data[g_ini_items].iKeyValueLen > PARSER_MAX_KEY_VALUE_LEN) {
			TOUCH_E("MAX_KEY_VALUE_LEN: Out Of Length\n");
			ret = INI_ERR_OUT_OF_LINE;
			goto out;
		}

		memcpy(ini_data[g_ini_items].pKeyValue, ini_buf + isEqualSign + 1, ini_data[g_ini_items].iKeyValueLen);

		TOUCH_D(PRODUCTION, "%s = %s\n", ini_data[g_ini_items].pKeyName, ini_data[g_ini_items].pKeyValue);

		g_ini_items++;
	}

out:
	ipio_kfree((void **)&ini_buf);
	ipio_kfree((void **)&tmpSectionName);
	return ret;
}

int ili9881h_parse_ini_file(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;
	const struct firmware *ini = NULL;
	int path_idx = 0;
	int i = 0;
	const char *path[2] = {ts->dual_panel_spec[0], ts->dual_panel_spec_mfts[0]};

	TOUCH_TRACE();

	if(touch_boot_mode_check(dev) >= MINIOS_MFTS_FOLDER)
		path_idx = 1;
	else
		path_idx = 0;

	if (ts->dual_panel_spec[0] == NULL || ts->dual_panel_spec_mfts[0] == NULL) {
		TOUCH_E("dual_panel_spec file name is null\n");
		ret = -1;
		goto error;
	}

	TOUCH_I("touch_panel_spec file path = %s\n", path[path_idx]);
	//TOUCH_I("ini path = %s\n", inipath);

	/* Get ini file location */
	ret = request_firmware(&ini, path[path_idx], dev);
	if (ret < 0) {
		TOUCH_E("fail to request_firmware inipath: %s (ret:%d)\n",
				path[path_idx], ret);
		ret = -1;
		goto error;
	}

	if (ini->data == NULL) {
		ret = -1;
		TOUCH_I("ini->data is NULL\n");
		goto error;
	}
	TOUCH_I("firmware ini size:%zu, data: %p\n", ini->size, ini->data);

	g_ini_items = 0;
	/*
	ini_data = kcalloc(PARSER_MAX_KEY_NUM, sizeof(*ini_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ini_data)) {
		TOUCH_E("Failed to allocate ini_data\n");
		ret = -ENOMEM;
		goto error;
	}
	*/
	// Initialise ini strcture
	for (i = 0; i < PARSER_MAX_KEY_NUM; i++) {
		memset(ini_data[i].pSectionName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ini_data[i].pKeyName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ini_data[i].pKeyValue, 0, PARSER_MAX_KEY_VALUE_LEN);
		ini_data[i].iSectionNameLen = 0;
		ini_data[i].iKeyNameLen = 0;
		ini_data[i].iKeyValueLen = 0;
	}


	ret = ili9881h_get_ini_phy_data(ini->data, ini->size);
	if (ret < 0) {
		TOUCH_E("Failed to get ini's physical data, ret = %d\n", ret);
	}

	TOUCH_I("Parsing INI file done\n");

error:
	if (ini)
		release_firmware(ini);
	return ret;
}

int ili9881h_check_int_status(bool high)
{
	struct touch_core_data *ts = to_touch_core(core_mp->dev);
	int timer = 1000, ret = -1;

	/* From FW request, timeout should at least be 5 sec */
	while (timer) {
		if(high) {
			if (gpio_get_value(ts->int_pin)) {
				TOUCH_I("Check busy is free\n");
				ret = 0;
				break;
			}
		} else {
			if (!gpio_get_value(ts->int_pin)) {
				TOUCH_I("Check busy is free\n");
				ret = 0;
				break;
			}
		}

		touch_msleep(5);
		timer--;
	}

	if (ret < -1)
		TOUCH_I("Check busy timeout !!\n");

	return ret;
}

static void ili9881h_mp_print_csv_header(struct device *dev)
{
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_fw_info *fw_info = &d->ic_info.fw_info;
	struct ili9881h_protocol_info *protocol_info = &d->ic_info.protocol_info;
	struct ili9881h_core_info *core_info = &d->ic_info.core_info;
	unsigned char buffer[LOG_BUF_SIZE] = {0,};
	int ret = 0;
	/* file create , time log */
	if (d->lcd_mode == LCD_MODE_U3) {
		write_file(dev, "\nShow_sd Test Start", false);
	} else if (d->lcd_mode == LCD_MODE_U0) {
		write_file(dev, "\nShow_lpwg_sd Test Start", false);
	}
	write_file(dev, "\n", true);

	ret = snprintf(buffer, LOG_BUF_SIZE,
			"======== Firmware Info ========\n");
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"version : v%d.%02d, fw_core.customer_id = %d.%d\n",
			fw_info->major, fw_info->minor, fw_info->core, fw_info->customer_code);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"protocol version : %d.%d.%d\n",
			protocol_info->major, protocol_info->mid, protocol_info->minor);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"core version : %d.%d.%d.%d\n",
			core_info->code_base, core_info->minor, core_info->revision_major, core_info->revision_minor);
	ret += snprintf(buffer + ret, LOG_BUF_SIZE - ret,
			"==============================\n");

	write_file(dev, buffer, false);
}

static void ili9881h_mp_print_csv_tail(char *csv, int *csv_len)
{
	int i, tmp_len = *csv_len;;

	TOUCH_TRACE();

	tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len,
			"==============================================================================\n");
	tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "Result_Summary           \n");

	for (i = 0; i < core_mp->mp_items; i++) {
		if (tItems[i].run) {
			if (tItems[i].item_result == MP_PASS)
				tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "   {%s}     =>OK\n",tItems[i].desp);
			else
				tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "   {%s}     =>NG\n",tItems[i].desp);
		}
	}

	*csv_len = tmp_len;
}

int ili9881h_mp_test_data_sort_average(int32_t *oringin_data, int index, int32_t *avg_result)
{
	int i, j, k, x, y; //, len = 5;
	int32_t u32temp;
	int u32up_frame, u32down_frame;
	int32_t *u32sum_raw_data;
	int32_t *u32data_buff;
	int ret = 0;

	TOUCH_TRACE();

	if (tItems[index].frame_count <= 1)
		return 0;


	if (ERR_ALLOC_MEM(oringin_data)) {
		TOUCH_E("Input wrong adress\n");
		ret= -ENOMEM;
		goto error;
	}

	u32data_buff = kcalloc(core_mp->frame_len * tItems[index].frame_count, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(u32data_buff)) {
		TOUCH_E("Failed to allocate u32data_buff FRAME buffer\n");
		ret= -ENOMEM;
		goto error;
	}

	u32sum_raw_data = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(u32sum_raw_data)) {
		TOUCH_E("Failed to allocate u32sum_raw_data FRAME buffer\n");
		ret= -ENOMEM;
		goto error;
	}

	for (i = 0; i < core_mp->frame_len * tItems[index].frame_count; i++) {
		u32data_buff[i] = oringin_data[i];
	}

	u32up_frame = tItems[index].frame_count * tItems[index].highest_percentage / 100;
	u32down_frame = tItems[index].frame_count * tItems[index].lowest_percentage / 100;

	TOUCH_I("Up=%d, Down=%d -%s\n", u32up_frame, u32down_frame, tItems[index].desp);

	// if (ipio_debug_level & DEBUG_MP_TEST) {
	// 	printk("\n[Show Original frist%d and last%d node data]\n", len, len);
	// 	for (i = 0; i < core_mp->frame_len; i++) {
	// 		for (j = 0 ; j < tItems[index].frame_count ; j++) {
	// 			if ((i < len) || (i >= (core_mp->frame_len-len)))
	// 				printk("%d,", u32data_buff[j * core_mp->frame_len + i]);
	// 		}
	// 		if ((i < len) || (i >= (core_mp->frame_len-len)))
	// 			printk("\n");
	// 	}
	// }

	for (i = 0; i < core_mp->frame_len; i++) {
		for (j = 0; j < tItems[index].frame_count-1; j++) {
			for (k = 0; k < (tItems[index].frame_count-1-j); k++) {
				x = i+k*core_mp->frame_len;
				y = i+(k+1)*core_mp->frame_len;
				if (*(u32data_buff+x) > *(u32data_buff+y)) {
					u32temp = *(u32data_buff+x);
					*(u32data_buff+x) = *(u32data_buff+y);
					*(u32data_buff+y) = u32temp;
				}
			}
		}
	}

	// if (ipio_debug_level & DEBUG_MP_TEST) {
	// 	printk("\n[After sorting frist%d and last%d node data]\n", len, len);
	// 	for (i = 0; i < core_mp->frame_len; i++) {
	// 		for (j = u32down_frame; j < tItems[index].frame_count - u32up_frame; j++) {
	// 			if ((i < len) || (i >= (core_mp->frame_len - len)))
	// 				printk("%d,", u32data_buff[i + j * core_mp->frame_len]);
	// 		}
	// 		if ((i < len) || (i >= (core_mp->frame_len-len)))
	// 			printk("\n");
	// 	}
	// }

	for (i = 0 ; i < core_mp->frame_len ; i++) {
		u32sum_raw_data[i] = 0;
		for (j = u32down_frame; j < tItems[index].frame_count - u32up_frame; j++)
			u32sum_raw_data[i] += u32data_buff[i + j * core_mp->frame_len];

		avg_result[i] = u32sum_raw_data[i] / (tItems[index].frame_count - u32down_frame - u32up_frame);
	}

	// if (ipio_debug_level & DEBUG_MP_TEST) {
	// 	printk("\n[Average result frist%d and last%d node data]\n", len, len);
	// 	for (i = 0; i < core_mp->frame_len; i++) {
	// 		if ((i < len) || (i >= (core_mp->frame_len-len)))
	// 			printk("%d,", avg_result[i]);
	// 	}
	// 	if ((i < len) || (i >= (core_mp->frame_len-len)))
	// 		printk("\n");
	// }
error:
	ipio_kfree((void **)&u32data_buff);
	ipio_kfree((void **)&u32sum_raw_data);
	return ret;
}

static void ili9881h_mp_compare_raw_data(int index, int32_t *tmp, int32_t *max_ts, int32_t *min_ts, int* result)
{
	int x, y;

	TOUCH_TRACE();

	if (ERR_ALLOC_MEM(tmp)) {
		TOUCH_E("The data of test item is null (%p)\n", tmp);
		*result = MP_FAIL;
		return;
	}

	/* In Short teset, we only identify if its value is low than min threshold. */
	if (tItems[index].catalog == SHORT_TEST) {
		for (y = 0; y < core_mp->ych_len; y++) {
			for (x = 0; x < core_mp->xch_len; x++) {
				int shift = y * core_mp->xch_len + x;

				if (tmp[shift] < min_ts[shift]) {
					*result = MP_FAIL;
					return;
				}
			}
		}
	} else {
		for (y = 0; y < core_mp->ych_len; y++) {
			for (x = 0; x < core_mp->xch_len; x++) {
				int shift = y * core_mp->xch_len + x;

				if (tmp[shift] > max_ts[shift] || tmp[shift] < min_ts[shift])
					*result = MP_FAIL;
			}
		}
	}
}

static void ili9881h_mp_compare_raw_data_ret(int index, int32_t *tmp, char *csv, int *csv_len,
		int type, int32_t *max_ts, int32_t *min_ts, const char *desp)
{
	int x, y, tmp_len = *csv_len;
	int mp_result = MP_PASS;

	TOUCH_TRACE();

	if (ERR_ALLOC_MEM(tmp)) {
		TOUCH_E("The data of test item is null (%p)\n", tmp);
		mp_result = MP_FAIL;
		goto out;
	}

	// print X raw only
	for (x = 0; x < core_mp->xch_len; x++) {
		if (x == 0) {
			if (desp != NULL)
				tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "============= %s ============\n   : ", desp);
			else
				tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "   : ");
		}
		tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, " [%2d] ", (x+1));
	}
	tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "\n");

	for (y = 0; y < core_mp->ych_len; y++) {
		tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "[%2d] ", (y+1));

		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len + x;

			/* In Short teset, we only identify if its value is low than min threshold. */
			if (tItems[index].catalog == SHORT_TEST) {
				if (tmp[shift] < min_ts[shift]) {
					TOUCH_D(PRODUCTION, " #%7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, "#%7d,", tmp[shift]);
					mp_result = MP_FAIL;
				} else {
					TOUCH_D(PRODUCTION, " %7d ", tmp[shift]);
					tmp_len += sprintf(csv + tmp_len, " %7d, ", tmp[shift]);
				}
				continue;
			}

			if ((tmp[shift] <= max_ts[shift] && tmp[shift] >= min_ts[shift]) || (type != TYPE_JUGE)) {

				if((tmp[shift] == INT_MAX || tmp[shift] == INT_MIN) && (type == TYPE_BENCHMARK))
					tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "   X  ");//BYPASS
				else
					tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, " %5d", tmp[shift]);
			} else {
				if (tmp[shift] > max_ts[shift])
					tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "*%5d", tmp[shift]);
				else
					tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "#%5d", tmp[shift]);

				mp_result = MP_FAIL;
			}
		}
		tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "\n");
	}

out:
	if (type == TYPE_JUGE) {
		if (mp_result == MP_PASS)
			tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "Result : PASS\n");
		else
			tmp_len += snprintf(csv + tmp_len, CSV_FILE_SIZE - tmp_len, "Result : FAIL\n");
	}

	*csv_len = tmp_len;
}

static int ili9881h_create_mp_test_frame_buffer(int index, int frame_count)
{
	TOUCH_TRACE();

	TOUCH_D(PRODUCTION, "Create MP frame buffers (index = %d), count = %d\n"
			,index, frame_count);

	if (tItems[index].buf == NULL) {
		tItems[index].buf = vmalloc(frame_count * core_mp->frame_len * sizeof(int32_t));
		if (ERR_ALLOC_MEM(tItems[index].buf)) {
			TOUCH_E("Failed to allocate buf mem\n");
			vfree(tItems[index].buf);
			tItems[index].buf = NULL;
			return -ENOMEM;
		}
	}

	if (tItems[index].result_buf == NULL) {
		tItems[index].result_buf = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(tItems[index].result_buf)) {
			TOUCH_E("Failed to allocate result_buf mem\n");
			ipio_kfree((void **)&tItems[index].result_buf);
			return -ENOMEM;
		}
	}

	if (tItems[index].max_buf == NULL) {
		tItems[index].max_buf = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(tItems[index].max_buf)) {
			TOUCH_E("Failed to allocate max_buf mem\n");
			ipio_kfree((void **)&tItems[index].max_buf);
			return -ENOMEM;
		}
	}

	if (tItems[index].min_buf == NULL) {
		tItems[index].min_buf = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(tItems[index].min_buf)) {
			TOUCH_E("Failed to allocate min_buf mem\n");
			ipio_kfree((void **)&tItems[index].min_buf);
			return -ENOMEM;
		}
	}

	if (tItems[index].spec_option == BENCHMARK) {
		if (tItems[index].bench_mark_max == NULL) {
			tItems[index].bench_mark_max = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
			if (ERR_ALLOC_MEM(tItems[index].bench_mark_max)) {
				TOUCH_E("Failed to allocate bench_mark_max mem\n");
				ipio_kfree((void **)&tItems[index].bench_mark_max);
				return -ENOMEM;
			}
		}
		if (tItems[index].bench_mark_min == NULL) {
			tItems[index].bench_mark_min = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
			if (ERR_ALLOC_MEM(tItems[index].bench_mark_min)) {
				TOUCH_E("Failed to allocate bench_mark_min mem\n");
				ipio_kfree((void **)&tItems[index].bench_mark_min);
				return -ENOMEM;
			}
		}
	}

	return 0;
}

int ili9881h_core_mp_ctrl_lcm_status(bool on)
{
	int ret = 0, ctrl = 0, delay = 0;
	u8 lcd[15] = {0};

	memset(lcd, 0xFF, ARRAY_SIZE(lcd));

	ctrl = ((on) ? 1 : 2);
	delay = ((on) ? 100 : 10);

	lcd[0] = 0x2; //mutual->bg
	lcd[1] = 0;
	lcd[2] = ctrl;

	ili9881h_dump_packet(lcd, 8, ARRAY_SIZE(lcd), 0, "LCM Command");
	touch_msleep(delay);

	ret = ili9881h_reg_write(core_mp->dev, CMD_LCM_HEADER, lcd, ARRAY_SIZE(lcd) - 1);
	if (ret < 0) {
		TOUCH_E("Failed to write LCM command\n");
		goto out;
	}

	touch_msleep(delay);

out:
	return ret;
}

static void ili9881h_mp_calc_nodp(bool long_v)
{
	u8 at, phase;
	u8 r2d, rst, rst_back, dac_td, qsh_pw, qsh_td, dn;
	u16 tshd, tsvd_to_tshd, qsh_tdf, dp2tp, twc, twcm, tp2dp;
	u16 multi_term_num, tp_tsdh_wait, ddi_width;
	u32 real_tx_dura, tmp, tmp1;

	TOUCH_TRACE();

	TOUCH_I("DDI Mode = %d\n", long_v);

	tshd = core_mp->nodp.tshd;
	tsvd_to_tshd = core_mp->nodp.tsvd_to_tshd;
	multi_term_num = core_mp->nodp.multi_term_num_120;
	qsh_tdf = core_mp->nodp.qsh_tdf;
	at = core_mp->nodp.auto_trim;
	tp_tsdh_wait = core_mp->nodp.tp_tshd_wait_120;
	ddi_width = core_mp->nodp.ddi_width_120;
	dp2tp = core_mp->nodp.dp_to_tp;
	twc = core_mp->nodp.tx_wait_const;
	twcm = core_mp->nodp.tx_wait_const_multi;
	tp2dp = core_mp->nodp.tp_to_dp;
	phase = core_mp->nodp.phase_adc;
	r2d = core_mp->nodp.r2d_pw;
	rst = core_mp->nodp.rst_pw;
	rst_back = core_mp->nodp.rst_pw_back;
	dac_td = core_mp->nodp.dac_td;
	qsh_pw = core_mp->nodp.qsh_pw;
	qsh_td = core_mp->nodp.qsh_td;
	dn = core_mp->nodp.drop_nodp;

	/* NODP formulation */
	if (!long_v) {
		if (core_mp->nodp.is60HZ) {
			multi_term_num = core_mp->nodp.multi_term_num_60;
			tp_tsdh_wait = core_mp->nodp.tp_tshd_wait_60;
			ddi_width = core_mp->nodp.ddi_width_60;
		}

		if (multi_term_num == 0)
			multi_term_num = 1;

		tmp = ((tshd << 2) - (at << 6) - tp_tsdh_wait - ddi_width * (multi_term_num - 1) - 64 - dp2tp - twc) * 5;
		tmp1 = (phase << 5) - ((twcm * 5 + (phase << 5) + (tp2dp * 5 << 6)) * (multi_term_num - 1));
		real_tx_dura = (tmp - tmp1) / (multi_term_num * 5);

		core_mp->nodp.first_tp_width = (dp2tp * 5  + twc * 5  + (phase << 5) + real_tx_dura * 5) / 5;
		core_mp->nodp.tp_width = (((tp2dp * 10 + phase) << 6)  + real_tx_dura * 10) / 10;
		core_mp->nodp.txpw = (qsh_tdf + rst + qsh_pw + qsh_td + 2);

		if ( core_mp->nodp.txpw % 2 == 1 )
			core_mp->nodp.txpw = core_mp->nodp.txpw + 1;

		core_mp->nodp.nodp = real_tx_dura / core_mp->nodp.txpw / 2;
	} else {
		if (multi_term_num == 0)
			multi_term_num = 1;

		real_tx_dura = (((tshd << 2) - (at << 6) - ddi_width * (11) - 64 - dp2tp - twc) * 5 - (phase << 5) - ((twcm * 5 + (phase << 5) + (tp2dp * 5 << 6)) * (11))) / (12 * 5);

		core_mp->nodp.long_tsdh_wait = (tsvd_to_tshd + 10) << 6;

		core_mp->nodp.first_tp_width = (dp2tp * 5  + twc * 5  + (phase << 5) + real_tx_dura * 5) / 5;
		core_mp->nodp.tp_width = (((tp2dp * 10 + phase) << 6)  + real_tx_dura * 10) / 10;
		core_mp->nodp.txpw = (qsh_tdf + rst + qsh_pw + qsh_td + 2);

		if (core_mp->nodp.txpw % 2 == 1)
			core_mp->nodp.txpw = core_mp->nodp.txpw + 1;

		core_mp->nodp.nodp = real_tx_dura / core_mp->nodp.txpw / 2;
	}

	TOUCH_I("Read Tx Duration = %d\n", real_tx_dura);
	TOUCH_I("First TP Width = %d\n", core_mp->nodp.first_tp_width);
	TOUCH_I("TP Width = %d\n", core_mp->nodp.tp_width);
	TOUCH_I("TXPW = %d\n", core_mp->nodp.txpw);
	TOUCH_I("NODP = %d\n", core_mp->nodp.nodp);
}

int ili9881h_core_mp_calc_timing_nodp(void)
{
	int ret = 0;
	u8 test_type = 0x0;
	u8 timing_cmd[15] = {0};
	u8 get_timing[64] = {0};

	TOUCH_TRACE();

	if (ERR_ALLOC_MEM(core_mp)) {
		TOUCH_E("core_mp is NULL\n");
		return -ENOMEM;
	}

	memset(timing_cmd, 0xFF, sizeof(timing_cmd));

	timing_cmd[0] = CMD_GET_TIMING;
	timing_cmd[1] = test_type;

	/*
	 * To calculate NODP, we need to get timing parameters first from fw,
	 * which returnes 40 bytes data.
	 */
	ili9881h_dump_packet(timing_cmd, 8, sizeof(timing_cmd), 0, "Timing command");

	ret = ili9881h_reg_write(core_mp->dev, CMD_MP_CDC_INIT, &timing_cmd[0], sizeof(timing_cmd));
	if (ret < 0) {
		TOUCH_E("Failed to write timing command\n");
		goto out;
	}

	ret = ili9881h_reg_read(core_mp->dev, CMD_NONE, get_timing, sizeof(get_timing));
	if (ret < 0) {
		TOUCH_E("Failed to read timing parameters\n");
		goto out;
	}

	ili9881h_dump_packet(get_timing, 8, 41, 0, "Timing parameters (41bytes)");

	/* Combine timing data */
	core_mp->nodp.is60HZ = false; /* This will get from ini file by default.  */
	core_mp->nodp.isLongV = get_timing[2];
	core_mp->nodp.tshd = (get_timing[3] << 8) + get_timing[4];
	core_mp->nodp.multi_term_num_120 = get_timing[5];
	core_mp->nodp.multi_term_num_60 = get_timing[6];
	core_mp->nodp.tsvd_to_tshd = (get_timing[7] << 8) + get_timing[8];
	core_mp->nodp.qsh_tdf = (get_timing[9] << 8) + get_timing[10];
	core_mp->nodp.auto_trim = get_timing[11];
	core_mp->nodp.tp_tshd_wait_120 = (get_timing[12] << 8) + get_timing[13];
	core_mp->nodp.ddi_width_120 = (get_timing[14] << 8) + get_timing[15];
	core_mp->nodp.tp_tshd_wait_60 = (get_timing[16] << 8) + get_timing[17];
	core_mp->nodp.ddi_width_60 = (get_timing[18] << 8) + get_timing[19];
	core_mp->nodp.dp_to_tp = (get_timing[20] << 8) + get_timing[21];
	core_mp->nodp.tx_wait_const = (get_timing[22] << 8) + get_timing[23];
	core_mp->nodp.tx_wait_const_multi = (get_timing[24] << 8) + get_timing[25];
	core_mp->nodp.tp_to_dp = (get_timing[26] << 8) + get_timing[27];
	core_mp->nodp.phase_adc = get_timing[28];
	core_mp->nodp.r2d_pw = get_timing[29];
	core_mp->nodp.rst_pw = get_timing[30];
	core_mp->nodp.rst_pw_back = get_timing[31];
	core_mp->nodp.dac_td = get_timing[32];
	core_mp->nodp.qsh_pw = get_timing[33];
	core_mp->nodp.qsh_td = get_timing[34];
	core_mp->nodp.drop_nodp = get_timing[35];

	TOUCH_I("60HZ = %d\n", core_mp->nodp.is60HZ);
	TOUCH_I("DDI Mode = %d\n", core_mp->nodp.isLongV);
	TOUCH_I("TSHD = %d\n", core_mp->nodp.tshd);

	ili9881h_mp_calc_nodp(core_mp->nodp.isLongV);

out:
	return ret;
}

static int ili9881h_mp_cdc_get_pv5_4_command(u8 *cmd, int len, int index)
{
	int ret = 0;
	char str[PARSER_MAX_KEY_VALUE_LEN] = {0};
	char tmp[PARSER_MAX_KEY_VALUE_LEN] = {0};
	char *key = tItems[index].desp;

	TOUCH_TRACE();

#if 0
	if (strncmp(key, "Raw Data(No BK) (LCM OFF)", strlen(key)) == 0)
		key = "Raw Data(No BK)";
	else if (strncmp(key, "Raw Data(Have BK) (LCM OFF)", strlen(key)) == 0)
		key = "Raw Data(Have BK)";
	else if (strncmp(key, "Noise Peak To Peak(With Panel) (LCM OFF)", strlen(key)) == 0)
		key = "Noise Peak To Peak(With Panel)";
	else if (strncmp(key, "Raw Data_TD (LCM OFF)", strlen(key)) == 0)
    		key = "Doze Raw Data";
	else if (strncmp(key, "Peak To Peak_TD (LCM OFF)", strlen(key)) == 0)
		key = "Doze Peak To Peak";
#endif

	TOUCH_I("%s gets %s command from INI.\n", tItems[index].desp, key);

	ret = ili9881h_get_ini_int_data("PV5_4 Command", key, str);
	if (ret < 0) {
		TOUCH_E("Failed to parse PV54 command, ret = %d\n", ret);
		goto out;
	}

	strncpy(tmp, str, ret);
	ili9881h_get_ini_u8_data(tmp, cmd, CMD_CDC_LEN - 1);

out:
	return ret;
}

static int ili9881h_mp_cdc_init_cmd_common(u8 *cmd, int len, int index)
{
	TOUCH_I("Get CDC command with protocol v5.4\n");
	return ili9881h_mp_cdc_get_pv5_4_command(cmd, len, index);
}

static int ili9881h_allnode_mutual_cdc_data(int index)
{
	int i = 0, ret = 0, len = 0;
	u8 cmd[CMD_CDC_LEN] = {0, };
	u8 *ori = NULL;
	u8 check_sum = 0;

	TOUCH_TRACE();

	/* Multipling by 2 is due to the 16 bit in each node */
	len = (core_mp->xch_len * core_mp->ych_len * 2) + 2;

	TOUCH_I("Read X/Y Channel length = %d\n", len);
	TOUCH_I("core_mp->frame_len = %d\n", core_mp->frame_len);

	if (len <= 2) {
		TOUCH_E("Length is invalid\n");
		ret = -1;
		goto out;
	}

	memset(cmd, 0xFF, CMD_CDC_LEN);

	/* CDC init */
	ili9881h_mp_cdc_init_cmd_common(cmd, CMD_CDC_LEN, index);

	ili9881h_dump_packet(cmd, 8, CMD_CDC_LEN, 0, "Mutual CDC command");

	/* NOTE: If TP driver is doing MP test and commanding 0xF1 to FW, we add a checksum
	 * to the last index and plus 1 with size. */
	check_sum = ili9881h_calc_data_checksum(&cmd[0], 8, CMD_CDC_LEN - 1);
	cmd[CMD_CDC_LEN - 1] = check_sum;

	ret = ili9881h_reg_write(core_mp->dev, CMD_MP_CDC_INIT, &cmd[1], CMD_CDC_LEN - 1);
	if (ret < 0) {
		TOUCH_E("I2C Write Error while initialising cdc\n");
		goto out;
	}

	/* Check busy */
	TOUCH_I("Check busy method = %d\n", core_mp->busy_cdc);
	if (core_mp->busy_cdc == POLL_CHECK) {
		ret = ili9881h_check_cdc_busy(core_mp->dev, 10, 50);
	} else if (core_mp->busy_cdc == INT_CHECK) {
		ret = ili9881h_check_int_status(true);//dependent from F/W(we use POLL_CHECK)
	} else if (core_mp->busy_cdc == DELAY_CHECK) {
		touch_msleep(600);
	}

	if (ret < 0) {
		TOUCH_E("Check busy timeout !\n");
		ret = -1;
		goto out;
	}

	/* Prepare to get cdc data */
	cmd[0] = CMD_READ_DATA_CTRL;
	cmd[1] = CMD_GET_CDC_DATA;

	ret = ili9881h_reg_write(core_mp->dev, CMD_READ_DATA_CTRL, &cmd[1], 1);
	if (ret < 0) {
		TOUCH_E("I2C Write Error\n");
		goto out;
	}
	touch_msleep(1);

	ret = ili9881h_reg_write(core_mp->dev, CMD_GET_CDC_DATA, cmd, 0);
	if (ret < 0) {
		TOUCH_E("I2C Write Error\n");
		goto out;
	}
	touch_msleep(1);

	/* Allocate a buffer for the original */
	ori = kcalloc(len, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ori)) {
		TOUCH_E("Failed to allocate ori mem (%ld)\n", PTR_ERR(ori));
		goto out;
	}

	/* Get original frame(cdc) data */
	ret = ili9881h_reg_read(core_mp->dev, CMD_NONE, ori, len);
	if (ret < 0) {
		TOUCH_E("I2C Read Error while getting original cdc data\n");
		goto out;
	}

	ili9881h_dump_packet(ori, 8, len, 0, "Mutual CDC original");

	if (frame_buf == NULL) {
		frame_buf = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(frame_buf)) {
			TOUCH_E("Failed to allocate FrameBuffer mem (%ld)\n", PTR_ERR(frame_buf));
			goto out;
		}
	} else {
		memset(frame_buf, 0x0, core_mp->frame_len);
	}

	/* Convert original data to the physical one in each node */
	for (i = 0; i < core_mp->frame_len; i++) {
			/* H byte + L byte */
			int32_t tmp = (ori[(2 * i) + 1] << 8) + ori[(1 + (2 * i)) + 1];

			if ((tmp & 0x8000) == 0x8000)
				frame_buf[i] = tmp - 65536;
			else
				frame_buf[i] = tmp;

			if (strncmp(tItems[index].name, "mutual_no_bk", strlen("mutual_no_bk")) == 0 ||
					strncmp(tItems[index].name, "mutual_no_bk_lcm_off", strlen("mutual_no_bk_lcm_off")) == 0) {
				frame_buf[i] -= RAWDATA_NO_BK_DATA_SHIFT_9881H;
			}
	}

	ili9881h_dump_packet(frame_buf, 32, core_mp->frame_len,  core_mp->xch_len, "Mutual CDC combined");

out:
	ipio_kfree((void **)&ori);
	return ret;
}


static void ili9881h_compare_MaxMin_result(int index, int32_t *data)
{
	int x, y;

	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			int shift = y * core_mp->xch_len;

			if (tItems[index].max_buf[shift + x] < data[shift + x])
				tItems[index].max_buf[shift + x] = data[shift + x];

			if (tItems[index].min_buf[shift + x] > data[shift + x])
				tItems[index].min_buf[shift + x] = data[shift + x];
		}
	}
}

#define ABS(a, b) ((a > b) ? (a - b) : (b - a))
#define ADDR(x, y) ((y * core_mp->xch_len) + (x))

int ili9881h_full_open_rate_compare(int32_t* full_open, int32_t* cbk, int x, int y, int32_t inNodeType, int full_open_rate)
{
	int ret = true;

	if ((inNodeType == NO_COMPARE) || ((inNodeType & Round_Corner) == Round_Corner)) {
		return true;
	}

	if (full_open[ADDR(x, y)] < (cbk[ADDR(x, y)] * full_open_rate / 100))
		ret = false;

	return ret;
}


int ili9881h_compare_charge(int32_t* charge_rate, int x, int y, int32_t* inNodeType, int Charge_AA, int Charge_Border, int Charge_Notch)
{
	int OpenThreadhold,tempY, tempX, ret, k;
	int sx[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int sy[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };

	TOUCH_TRACE();

	ret = charge_rate[ADDR(x, y)];

	/*Setting Threadhold from node type  */

	if(charge_rate[ADDR(x, y)] == 0)
		return ret;
	else if ((inNodeType[ADDR(x, y)] & AA_Area) == AA_Area)
		OpenThreadhold = Charge_AA;
	else if ((inNodeType[ADDR(x, y)] & Border_Area) == Border_Area)
		OpenThreadhold = Charge_Border;
	else if ((inNodeType[ADDR(x, y)] & Notch) == Notch)
		OpenThreadhold = Charge_Notch;
	else
		return ret;

	/* compare carge rate with 3*3 node */
	/* by pass => 1.no compare 2.corner 3.Skip_Micro 4.full open fail node */
	for (k = 0; k < 8; k++) {
		tempX = x + sx[k];
		tempY = y + sy[k];

		if ((tempX < 0) || (tempX >= core_mp->xch_len) || (tempY < 0) || (tempY >= core_mp->ych_len)) /*out of range */
			continue;

		if ((inNodeType[ADDR(tempX, tempY)] == NO_COMPARE) || ((inNodeType[ADDR(tempX, tempY)] & Round_Corner) == Round_Corner) ||
				((inNodeType[ADDR(tempX, tempY)] & Skip_Micro) == Skip_Micro) || charge_rate[ADDR(tempX, tempY)] == 0)
			continue;

		if ((charge_rate[ADDR(tempX, tempY)]-charge_rate[ADDR(x, y)])> OpenThreadhold)
			return OpenThreadhold;
	}
	return ret;
}

/* This will be merged to allnode_mutual_cdc_data in next version */
int ili9881h_allnode_open_cdc_data(int mode, int *buf, int *dac)
{
	int i = 0, ret = 0, len = 0;
	int inDACp = 0, inDACn = 0;
	u8 cmd[CMD_CDC_LEN] = {0};
	u8 *ori = NULL;
	char str[128] = {0};
	char tmp[128] = {0};
	char *key[] = {"OPEN DAC", "OPEN Raw1", "OPEN Raw2", "OPEN Raw3"};
	u8 check_sum = 0;

	TOUCH_TRACE();

	/* Multipling by 2 is due to the 16 bit in each node */
	len = (core_mp->xch_len * core_mp->ych_len * 2) + 2;

	TOUCH_I("Read X/Y Channel length = %d\n", len);
	TOUCH_I("core_mp->frame_len = %d\n", core_mp->frame_len);

	if (len <= 2) {
		TOUCH_E("Length is invalid\n");
		ret = -1;
		goto out;
	}

	/* CDC init. Read command from ini file */
	ret = ili9881h_get_ini_int_data("PV5_4 Command", key[mode], str);
	if (ret < 0) {
		TOUCH_E("Failed to parse PV54 command, ret = %d\n", ret);
		goto out;
	}

	strncpy(tmp, str, ret);
	ili9881h_get_ini_u8_data(tmp, cmd, CMD_CDC_LEN - 1);

	ili9881h_dump_packet(cmd, 8, sizeof(cmd), 0, "Open SP command");

	/* NOTE: If TP driver is doing MP test and commanding 0xF1 to FW, we add a checksum
	 * to the last index and plus 1 with size. */
	check_sum = ili9881h_calc_data_checksum(&cmd[0], 8, CMD_CDC_LEN - 1);
	cmd[CMD_CDC_LEN - 1] = check_sum;

	ret = ili9881h_reg_write(core_mp->dev, CMD_MP_CDC_INIT, &cmd[1], CMD_CDC_LEN - 1);
	if (ret < 0) {
		TOUCH_E("I2C Write Error while initialising cdc\n");
		goto out;
	}

	/* Check busy */
	TOUCH_I("Check busy method = %d\n", core_mp->busy_cdc);
	if (core_mp->busy_cdc == POLL_CHECK) {
		ret = ili9881h_check_cdc_busy(core_mp->dev, 10, 50);
	} else if (core_mp->busy_cdc == INT_CHECK) {
		ret = ili9881h_check_int_status(true);//dependent from F/W(we use POLL_CHECK)
	} else if (core_mp->busy_cdc == DELAY_CHECK) {
		touch_msleep(600);
	}

	if (ret < 0) {
		TOUCH_E("Check busy timeout !\n");
		ret = -1;
		goto out;
	}

	/* Prepare to get cdc data */
	cmd[0] = CMD_READ_DATA_CTRL;
	cmd[1] = CMD_GET_CDC_DATA;

	ret = ili9881h_reg_write(core_mp->dev, CMD_READ_DATA_CTRL, &cmd[1], 1);
	if (ret < 0) {
		TOUCH_E("I2C Write Error\n");
		goto out;
	}

	touch_msleep(1);

	ret = ili9881h_reg_write(core_mp->dev, CMD_GET_CDC_DATA, cmd, 0);
	if (ret < 0) {
		TOUCH_E("I2C Write Error\n");
		goto out;
	}

	touch_msleep(1);

	/* Allocate a buffer for the original */
	ori = kcalloc(len, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ori)) {
		TOUCH_E("Failed to allocate ori mem (%ld)\n", PTR_ERR(ori));
		goto out;
	}

	/* Get original frame(cdc) data */
	ret = ili9881h_reg_read(core_mp->dev, CMD_NONE, ori, len);
	if (ret < 0) {
		TOUCH_E("I2C Read Error while getting original cdc data\n");
		goto out;
	}

	ili9881h_dump_packet(ori, 8, len, 0, "Open SP CDC original");

	/* Convert original data to the physical one in each node */
	for (i = 0; i < core_mp->frame_len; i++) {
		if (mode == 0) {
			/* DAC - P */
			if (((ori[(2 * i) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACp = 0 - (int)(ori[(2 * i) + 1] & 0x7F);
			} else {
				inDACp = ori[(2 * i) + 1] & 0x7F;
			}

			/* DAC - N */
			if (((ori[(1 + (2 * i)) + 1] & 0x80) >> 7) == 1) {
				/* Negative */
				inDACn = 0 - (int)(ori[(1 + (2 * i)) + 1] & 0x7F);
			} else {
				inDACn = ori[(1 + (2 * i)) + 1] & 0x7F;
			}

			buf[i] = (inDACp + inDACn) / 2;
		} else {
			/* H byte + L byte */
			int32_t tmp = (ori[(2 * i) + 1] << 8) + ori[(1 + (2 * i)) + 1];
			if ((tmp & 0x8000) == 0x8000)
				buf[i] = tmp - 65536;
			else
				buf[i] = tmp;
			buf[i] = (int)((int)(dac[i] * 2 * 10000 * 161 / 100) - (int)(16384 / 2 - (int)buf[i]) * 20000 * 7 / 16384 * 36 / 10) / 31 / 2;
		}
	}
	ili9881h_dump_packet(buf, 10, core_mp->frame_len,  core_mp->xch_len, "Open SP CDC combined");
out:
	ipio_kfree((void **)&ori);

	return ret;
}

static int ili9881h_mp_open_sp_test(int index)
{
	struct mp_test_P540_open open[tItems[index].frame_count];
	int i = 0, x = 0, y = 0, ret = 0, addr = 0; /*get_frame_count = tItems[index].frame_count*/
	int Charge_AA = 0, Charge_Border = 0, Charge_Notch = 0, full_open_rate = 0;
	char str[512] = { 0 };

	TOUCH_TRACE();
	TOUCH_D(PRODUCTION,"index = %d, name = %s, Frame Count = %d\n",
			index, tItems[index].name, tItems[index].frame_count);

	/*
	 * We assume that users who are calling the test forget to config frame count
	 * as 1, so we just help them to set it up.
	 */
	if (tItems[index].frame_count <= 0) {
		TOUCH_E("Frame count is zero, which should be at least set by 1\n");
		tItems[index].frame_count = 1;
	}

	ret = ili9881h_create_mp_test_frame_buffer(index, tItems[index].frame_count);
	if (ret < 0)
		goto out;

	if (frame1_cbk700 == NULL) {
		frame1_cbk700 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(frame1_cbk700)) {
			TOUCH_E("Failed to allocate frame1_cbk700 buffer\n");
			ipio_kfree((void **)&frame1_cbk700);
			return -ENOMEM;
		}
	} else {
		memset(frame1_cbk700, 0x0, core_mp->frame_len);
	}

	if (frame1_cbk250 == NULL) {
		frame1_cbk250 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(frame1_cbk250)) {
			TOUCH_E("Failed to allocate frame1_cbk250 buffer\n");
			ipio_kfree((void **)&frame1_cbk700);
			ipio_kfree((void **)&frame1_cbk250);
			return -ENOMEM;
		}
	} else {
		memset(frame1_cbk250, 0x0, core_mp->frame_len);
	}

	if (frame1_cbk200 == NULL) {
		frame1_cbk200 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(frame1_cbk200)) {
			TOUCH_E("Failed to allocate cbk_200 buffer\n");
			ipio_kfree((void **)&frame1_cbk700);
			ipio_kfree((void **)&frame1_cbk250);
			ipio_kfree((void **)&frame1_cbk200);
			return -ENOMEM;
		}
	} else {
		memset(frame1_cbk200, 0x0, core_mp->frame_len);
	}

	tItems[index].node_type = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(tItems[index].node_type)) {
		TOUCH_E("Failed to allocate node_type FRAME buffer\n");
		ret= -ENOMEM;
		goto out;
	}

	/* Init Max/Min buffer */
	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			tItems[index].max_buf[y * core_mp->xch_len + x] = INT_MIN;
			tItems[index].min_buf[y * core_mp->xch_len + x] = INT_MAX;
		}
	}

	if (tItems[index].spec_option == BENCHMARK)
		ili9881h_parse_benchmark(tItems[index].bench_mark_max, tItems[index].bench_mark_min, tItems[index].type_option, tItems[index].desp);

	ili9881h_parse_nodetype(tItems[index].node_type, "Node Type");

	ret = ili9881h_get_ini_int_data(tItems[index].desp, "Charge_AA", str);
	if (ret || ret == 0)
		Charge_AA = katoi(str);

	ret = ili9881h_get_ini_int_data(tItems[index].desp, "Charge_Border", str);
	if (ret || ret == 0)
		Charge_Border = katoi(str);

	ret = ili9881h_get_ini_int_data(tItems[index].desp, "Charge_Notch", str);
	if (ret || ret == 0)
		Charge_Notch = katoi(str);

	ret = ili9881h_get_ini_int_data(tItems[index].desp, "Full Open", str);
	if (ret || ret == 0)
		full_open_rate = katoi(str);

	if (ret < 0) {
		TOUCH_E("Failed to get parameters from ini file\n");
		goto out;
	}

	TOUCH_D(PRODUCTION, "pen test frame_cont %d, AA %d,Border %d, Notch %d, full_open_rate %d \n",
			tItems[index].frame_count,Charge_AA,Charge_Border,Charge_Notch,full_open_rate);

	/* Allocate internal buffers for the use of open sp test */
	for(i = 0; i < tItems[index].frame_count; i++) {
		open[i].cbk_700 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(open[i].cbk_700)) {
			TOUCH_E("Failed to open[%d].cbk_700 buf mem\n", i);
			goto out;
		}

		open[i].cbk_250 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(open[i].cbk_250)) {
			TOUCH_E("Failed to open[%d].cbk_250 buf mem\n", i);
			goto out;
		}

		open[i].cbk_200 = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(open[i].cbk_200)) {
			TOUCH_E("Failed to open[%d].cbk_200 buf mem\n", i);
			goto out;
		}

		open[i].charg_rate = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(open[i].charg_rate)) {
			TOUCH_E("Failed to open[%d].charg_rate buf mem\n", i);
			goto out;
		}

		open[i].full_Open = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(open[i].full_Open)) {
			TOUCH_E("Failed to open[%d].full_Open buf mem\n", i);
			goto out;
		}

		open[i].dac = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
		if (ERR_ALLOC_MEM(open[i].dac)) {
			TOUCH_E("Failed to open[%d].dac buf mem\n", i);
			goto out;
		}
	}

	/* The formulor of calculation */
	for (i = 0; i < tItems[index].frame_count; i++) {
		ret = ili9881h_allnode_open_cdc_data(0, open[i].dac, open[i].dac);
		if (ret < 0) {
			TOUCH_E("Failed to get Open SP DAC data, %d\n", ret);
			goto out;
		}
		ret = ili9881h_allnode_open_cdc_data(1, open[i].cbk_700, open[i].dac);
		if (ret < 0) {
			TOUCH_E("Failed to get Open SP Raw1 data, %d\n", ret);
			goto out;
		}
		ret = ili9881h_allnode_open_cdc_data(2, open[i].cbk_250, open[i].dac);
		if (ret < 0) {
			TOUCH_E("Failed to get Open SP Raw2 data, %d\n", ret);
			goto out;
		}
		ret = ili9881h_allnode_open_cdc_data(3, open[i].cbk_200, open[i].dac);
		if (ret < 0) {
			TOUCH_E("Failed to get Open SP Raw3 data, %d\n", ret);
			goto out;
		}

		addr = 0;

		/* record fist frame for debug */
		if(i == 0) {
			memcpy(frame1_cbk700, open[i].cbk_700, core_mp->frame_len * sizeof(int32_t));
			memcpy(frame1_cbk250, open[i].cbk_250, core_mp->frame_len * sizeof(int32_t));
			memcpy(frame1_cbk200, open[i].cbk_200, core_mp->frame_len * sizeof(int32_t));
		}

		// ili9881h_dump_packet(open[i].cbk_700, 10, core_mp->frame_len, core_mp->xch_len, "cbk 700");
		// ili9881h_dump_packet(open[i].cbk_250, 10, core_mp->frame_len, core_mp->xch_len, "cbk 250");
		// ili9881h_dump_packet(open[i].cbk_200, 10, core_mp->frame_len, core_mp->xch_len, "cbk 200");

		for (y = 0; y < core_mp->ych_len; y++) {
			for (x = 0; x < core_mp->xch_len; x++) {
				open[i].charg_rate[addr] = open[i].cbk_250[addr] * 100 / open[i].cbk_700[addr];
				open[i].full_Open[addr] = open[i].cbk_700[addr] - open[i].cbk_200[addr];
				addr++;
			}
		}

		// ili9881h_dump_packet(open[i].charg_rate, 10, core_mp->frame_len, core_mp->xch_len, "origin charge rate");
		// ili9881h_dump_packet(open[i].full_Open, 10, core_mp->frame_len, core_mp->xch_len, "origin full open");

		addr = 0;
		for(y = 0; y < core_mp->ych_len; y++){
			for(x = 0; x < core_mp->xch_len; x++){
				if(ili9881h_full_open_rate_compare(open[i].full_Open, open[i].cbk_700, x, y, tItems[index].node_type[addr], full_open_rate) == false) {
					tItems[index].buf[(i * core_mp->frame_len) + addr] = 0;
					open[i].charg_rate[addr] = 0;
				}
				addr++;
			}
		}

		// ili9881h_dump_packet(&tItems[index].buf[(i * core_mp->frame_len)], 10, core_mp->frame_len, core_mp->xch_len, "after ili9881h_full_open_rate_compare");

		addr = 0;
		for(y = 0; y < core_mp->ych_len; y++) {
			for(x = 0; x < core_mp->xch_len; x++) {
				tItems[index].buf[(i * core_mp->frame_len) + addr] = ili9881h_compare_charge(open[i].charg_rate, x, y, tItems[index].node_type, Charge_AA, Charge_Border, Charge_Notch);
				addr++;
			}
		}

		// ili9881h_dump_packet(&tItems[index].buf[(i * core_mp->frame_len)], 10, core_mp->frame_len, core_mp->xch_len, "after compare charge rate");

		ili9881h_compare_MaxMin_result(index, &tItems[index].buf[(i * core_mp->frame_len)]);
	}

out:
	ipio_kfree((void **)&tItems[index].node_type);

	for (i = 0; i < tItems[index].frame_count; i++) {
		ipio_kfree((void **)&open[i].cbk_700);
		ipio_kfree((void **)&open[i].cbk_250);
		ipio_kfree((void **)&open[i].cbk_200);
		ipio_kfree((void **)&open[i].charg_rate);
		ipio_kfree((void **)&open[i].full_Open);
		ipio_kfree((void **)&open[i].dac);
	}

	return ret;
}

int ili9881h_codeToOhm(int32_t Code)
{
	int douTDF1 = 0;
	int douTDF2 = 0;
	int douTVCH = 24;
	int douTVCL = 8;
	int douCint = 7;
	int douVariation = 64;
	int douRinternal = 930;
	int32_t temp = 0;

	if (core_mp->nodp.isLongV) {
		douTDF1 = 300;
		douTDF2 = 100;
	} else {
		douTDF1 = 219;
		douTDF2 = 100;
	}

	/* Unit = M Ohm */
	if (Code == 0) {
		TOUCH_E("code is invalid\n");
	} else {
		temp = ((douTVCH - douTVCL) * douVariation * (douTDF1 - douTDF2) * (1<<12) / (9 * Code * douCint)) * 100;
		temp = (temp - douRinternal) / 1000;
	}

	return temp;
}

static int ili9881h_short_test(int index, int frame_index)
{
	int j = 0, ret = 0;

	TOUCH_TRACE();
	/* Calculate code to ohm and save to tItems[index].buf */
	for (j = 0; j < core_mp->frame_len; j++)
		tItems[index].buf[frame_index * core_mp->frame_len + j] = ili9881h_codeToOhm(frame_buf[j]);

	return ret;
}

static int ili9881h_mp_retry_comp_cdc_result(int index)
{
	int i, test_result = MP_PASS;
	int32_t *max_threshold = NULL, *min_threshold = NULL;

	TOUCH_TRACE();

	max_threshold = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(max_threshold)) {
		TOUCH_E("Failed to allocate max threshold FRAME buffer\n");
		test_result = MP_FAIL;
		goto out;
	}

	min_threshold = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(min_threshold)) {
		TOUCH_E("Failed to allocate min threshold FRAME buffer\n");
		test_result = MP_FAIL;
		goto out;
	}

	/* Show test result as below */
	if (ERR_ALLOC_MEM(tItems[index].buf) || ERR_ALLOC_MEM(tItems[index].max_buf) ||
			ERR_ALLOC_MEM(tItems[index].min_buf) || ERR_ALLOC_MEM(tItems[index].result_buf)) {
		TOUCH_E("This test item (%s) has no data inside its buffer\n", tItems[index].desp);
		test_result = MP_FAIL;
		goto out;
	}

	if (tItems[index].spec_option == BENCHMARK) {
		for(i = 0; i < core_mp->frame_len; i++) {
			max_threshold[i] = tItems[index].bench_mark_max[i];
			min_threshold[i] = tItems[index].bench_mark_min[i];
		}
	} else {
		for(i = 0; i < core_mp->frame_len; i++) {
			max_threshold[i] = tItems[index].max;
			min_threshold[i] = tItems[index].min;
		}
	}

	/* general result */
	if(tItems[index].trimmed_mean && tItems[index].catalog != PEAK_TO_PEAK_TEST) {
		ili9881h_mp_test_data_sort_average(tItems[index].buf, index, tItems[index].result_buf);
		ili9881h_mp_compare_raw_data(index, tItems[index].result_buf, max_threshold, min_threshold, &test_result);
	} else {
		ili9881h_mp_compare_raw_data(index, tItems[index].max_buf, max_threshold, min_threshold, &test_result);
		ili9881h_mp_compare_raw_data(index, tItems[index].min_buf, max_threshold, min_threshold, &test_result);
	}

out:
	ipio_kfree((void **)&max_threshold);
	ipio_kfree((void **)&min_threshold);
	tItems[index].item_result = test_result;
	return test_result;
}

static int ili9881h_mp_do_retry(int index, int count)
{
	int result = MP_FAIL;

	TOUCH_TRACE();

	if (count == 0) {
		TOUCH_I("Retry was finished\n");
		return MP_FAIL;
	}

	ili9881h_ice_mode_enable(core_mp->dev);

	ili9881h_reset_ctrl(core_mp->dev, HW_RESET_ONLY);

	ili9881h_ice_mode_disable(core_mp->dev);

	/* Switch to test mode */
	ili9811h_switch_fw_mode(core_mp->dev, FIRMWARE_TEST_MODE);

	TOUCH_I("retry = %d, item = %s\n", count, tItems[index].desp);

	touch_interrupt_control(core_mp->dev, INTERRUPT_DISABLE);

	tItems[index].do_test(index);
#if 0
	if (ili9881h_mp_retry_comp_cdc_result(index) == MP_FAIL)
		return ili9881h_mp_do_retry(index, count - 1);
#endif

	result = ili9881h_mp_retry_comp_cdc_result(index);

	if (result == MP_FAIL)
		return ili9881h_mp_do_retry(index, count - 1);
	else
		return MP_PASS;
}

void ili9881h_core_mp_show_result(struct device *dev)
{
	int i, j, csv_len = 0, get_frame_count = 1;
	//int pass_item_count = 0, line_count = 0;
	int32_t *max_threshold = NULL, *min_threshold = NULL;
	char *csv = NULL;
	//char csv_name[128] = { 0 };
	//char *ret_pass_name = NULL, *ret_fail_name = NULL;
	//struct file *f = NULL;
	//struct timespec my_time;
	//struct tm my_date;
	//mm_segment_t fs;
	//loff_t pos;

	TOUCH_TRACE();

	//csv = kcalloc(CSV_FILE_SIZE, sizeof(u8), GFP_KERNEL);
	csv = vmalloc(CSV_FILE_SIZE);
	if (ERR_ALLOC_MEM(csv)) {
		TOUCH_E("Failed to allocate CSV mem\n");
		goto fail_open;
	}

	max_threshold = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(max_threshold)) {
		TOUCH_E("Failed to allocate max_threshold FRAME buffer\n");
		goto fail_open;
	}

	min_threshold = kcalloc(core_mp->frame_len, sizeof(int32_t), GFP_KERNEL);
	if (ERR_ALLOC_MEM(min_threshold)) {
		TOUCH_E("Failed to allocate min_threshold FRAME buffer\n");
		goto fail_open;
	}

	ili9881h_mp_print_csv_header(dev);

	for (i = 0; i < core_mp->mp_items; i++) {
		if (tItems[i].run != 1)
			continue;

		csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "\n\n==========%s==========\n", tItems[i].desp);

		if (tItems[i].item_result == MP_PASS) {
			TOUCH_I("[%s] Result is OK!\n", tItems[i].desp);
			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "%s is OK\n", tItems[i].desp);
		} else {
			TOUCH_I("[%s] Result is NG!\n", tItems[i].desp);
			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "%s is NG\n", tItems[i].desp);
		}

		TOUCH_I(" Frame count is %d\n", tItems[i].frame_count);
		csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "Frame count = %d\n", tItems[i].frame_count);

		if(tItems[i].trimmed_mean && tItems[i].catalog != PEAK_TO_PEAK_TEST) {
			TOUCH_I(" Lowest Percentage =  %d\n", tItems[i].lowest_percentage);
			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "Lowest Percentage = %d\n", tItems[i].lowest_percentage);
			TOUCH_I(" Highest Percentage =  %d\n", tItems[i].highest_percentage);
			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "Highest Percentage = %d\n", tItems[i].highest_percentage);
		}

		/* Show result of benchmark max and min */
		if ( tItems[i].spec_option == BENCHMARK) {
			for(j = 0 ;j < core_mp->frame_len ; j++){
				max_threshold[j] = tItems[i].bench_mark_max[j];
				min_threshold[j] = tItems[i].bench_mark_min[j];
			}

			ili9881h_mp_compare_raw_data_ret(i, tItems[i].bench_mark_max, csv, &csv_len, TYPE_BENCHMARK, max_threshold, min_threshold, "Max_Bench");
			ili9881h_mp_compare_raw_data_ret(i, tItems[i].bench_mark_min, csv, &csv_len, TYPE_BENCHMARK, max_threshold, min_threshold, "Min_Bench");
		} else {
			for(j = 0 ;j < core_mp->frame_len ; j++) {
				max_threshold[j] = tItems[i].max;
				min_threshold[j] = tItems[i].min;
			}

			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "Max = %d\n", tItems[i].max);
			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "Min = %d\n", tItems[i].min);
		}

		if (strncmp(tItems[i].name, "open_integration_sp", strlen(tItems[i].name)) == 0) {
			ili9881h_mp_compare_raw_data_ret(i, frame1_cbk700, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, "frame1 cbk700");
			ili9881h_mp_compare_raw_data_ret(i, frame1_cbk250, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, "frame1 cbk250");
			ili9881h_mp_compare_raw_data_ret(i, frame1_cbk200, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, "frame1 cbk200");
		}

		if (ERR_ALLOC_MEM(tItems[i].buf) || ERR_ALLOC_MEM(tItems[i].max_buf) || ERR_ALLOC_MEM(tItems[i].min_buf)) {
			TOUCH_E("This test item (%s) has no data inside its buffer\n", tItems[i].desp);
			continue;
		}

		/* Show test result as below */
		if(tItems[i].catalog != PEAK_TO_PEAK_TEST)
			get_frame_count = tItems[i].frame_count;

		if (get_frame_count > 1) {
			if(tItems[i].trimmed_mean && tItems[i].catalog != PEAK_TO_PEAK_TEST) {
				ili9881h_mp_compare_raw_data_ret(i, tItems[i].result_buf, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, "Mean result");
			} else {
				ili9881h_mp_compare_raw_data_ret(i, tItems[i].max_buf, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, "Max Hold");
				ili9881h_mp_compare_raw_data_ret(i, tItems[i].min_buf, csv, &csv_len, TYPE_NO_JUGE, max_threshold, min_threshold, "Min Hold");
			}

			/* result of each frame */
			for(j = 0; j < get_frame_count; j++) {
				csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "========Frame %d========\n", (j+1));
				ili9881h_mp_compare_raw_data_ret(i, &tItems[i].buf[(j*core_mp->frame_len)], csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold, NULL);
			}
		} else {
			csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len, "========%s Data========\n", tItems[i].desp);
			ili9881h_mp_compare_raw_data_ret(i, &tItems[i].buf[0], csv, &csv_len, TYPE_JUGE, max_threshold, min_threshold, NULL);
		}
	}

//	memset(csv_name, 0, 128 * sizeof(char));

	ili9881h_mp_print_csv_tail(csv, &csv_len);

	write_file(dev, csv, false);
#if 0
	/* print test time */
	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec,
			sys_tz.tz_minuteswest * 60 * (-1),
			&my_date);
	csv_len += snprintf(csv + csv_len, CSV_FILE_SIZE - csv_len,
			"\n[%02d-%02d %02d:%02d:%02d.%03lu]\n",
			my_date.tm_mon + 1,
			my_date.tm_mday, my_date.tm_hour,
			my_date.tm_min, my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);


	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		if (tItems[i].run) {
			if (tItems[i].item_result == MP_FAIL) {
				pass_item_count = 0;
				break;
			}
			pass_item_count++;
		}
	}

	ret_pass_name = CSV_PASS_NAME;
	ret_fail_name = CSV_FAIL_NAME;

	if (pass_item_count == 0) {
		core_mp->final_result = MP_FAIL;
		snprintf(csv_name, sizeof(csv_name), "%s/%s.txt", CSV_PATH, ret_fail_name);
	} else {
		core_mp->final_result = MP_PASS;
		snprintf(csv_name, sizeof(csv_name), "%s/%s.txt", CSV_PATH, ret_pass_name);
	}

	TOUCH_I("Open CSV : %s\n", csv_name);

	if (f == NULL)
		f = filp_open(csv_name, O_WRONLY | O_CREAT | O_TRUNC, 644);

	if (ERR_ALLOC_MEM(f)) {
		TOUCH_E("Failed to open CSV file");
		goto fail_open;
	}

	TOUCH_I("Open CSV succeed, its length = %d\n ", csv_len);

	if (csv_len >= CSV_FILE_SIZE) {
		TOUCH_E("The length saved to CSV is too long !\n");
		goto fail_open;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(f, csv, csv_len, &pos);
	set_fs(fs);
	filp_close(f, NULL);

	TOUCH_I("Writing Data into CSV succeed\n");
#endif

fail_open:
	// if(csv != NULL)
	// 	ipio_kfree((void **)&csv);
	vfree(csv);
	csv = NULL;
	ipio_kfree((void **)&max_threshold);
	ipio_kfree((void **)&min_threshold);
}

int ili9881h_mp_raw_data_test(int index)
{
	int i = 0, j = 0, x = 0, y = 0, ret = 0, get_frame_count = 1;

	TOUCH_TRACE();
	TOUCH_D(PRODUCTION,"index = %d, name = %s, Frame Count = %d\n",
			index, tItems[index].name, tItems[index].frame_count);

	/*
	 * We assume that users who are calling the test forget to config frame count
	 * as 1, so we just help them to set it up.
	 */
	if (tItems[index].frame_count <= 0) {
		TOUCH_E("Frame count is zero, which should be at least set by 1\n");
		tItems[index].frame_count = 1;
	}

	ret = ili9881h_create_mp_test_frame_buffer(index, tItems[index].frame_count);
	if (ret < 0)
		goto out;

	/* Init Max/Min buffer */
	for (y = 0; y < core_mp->ych_len; y++) {
		for (x = 0; x < core_mp->xch_len; x++) {
			tItems[index].max_buf[y * core_mp->xch_len + x] = INT_MIN;
			tItems[index].min_buf[y * core_mp->xch_len + x] = INT_MAX;
		}
	}

	if (tItems[index].catalog != PEAK_TO_PEAK_TEST)
		get_frame_count = tItems[index].frame_count;

	if (tItems[index].spec_option == BENCHMARK)
		ili9881h_parse_benchmark(tItems[index].bench_mark_max, tItems[index].bench_mark_min, tItems[index].type_option, tItems[index].desp);

	for (i = 0; i < get_frame_count; i++) {
		ret = ili9881h_allnode_mutual_cdc_data(index);
		if (ret < 0) {
			TOUCH_E("Failed to initialise CDC data, %d\n", ret);
			goto out;
		}
		switch (tItems[index].catalog) {
			case SHORT_TEST:
				ili9881h_short_test(index , i);
				break;
			default:
				for (j = 0; j < core_mp->frame_len; j++)
					tItems[index].buf[i * core_mp->frame_len + j] = frame_buf[j];
				break;
		}
		ili9881h_compare_MaxMin_result(index, &tItems[index].buf[i * core_mp->frame_len]);
	}

out:
	return ret;
}

int ili9881h_mp_move_code(void)
{
	TOUCH_TRACE();

	TOUCH_I("Prepaing to enter Test Mode\n");

	if (ili9881h_ice_mode_enable(core_mp->dev) < 0) {
		TOUCH_E("Failed to enter ICE mode\n");
		return -1;
	}

	/* DMA Trigger */
	ili9881h_ice_reg_write(core_mp->dev, 0x041010, 0xFF, 1);
	touch_msleep(30);

	/* CS High */
	ili9881h_ice_reg_write(core_mp->dev, 0x041000, 0x1, 1);
	touch_msleep(60);

	/* Code reset */
	ili9881h_ice_reg_write(core_mp->dev, 0x040040, 0xAE, 1);

	if (ili9881h_ice_mode_disable(core_mp->dev) < 0) {
		TOUCH_E("Failed to exit ICE mode\n");
		return -1;
	}

	if (ili9881h_check_cdc_busy(core_mp->dev, 100, 50) < 0) {
		TOUCH_E("Check busy is timout ! Enter Test Mode failed\n");
		return -1;
	}

	TOUCH_I("FW Test Mode ready\n");
	return 0;
}

void ili9881h_core_mp_test_free(void)
{
	int i;

	TOUCH_TRACE();

	TOUCH_I("Free all allocated mem\n");

	for (i = 0; i < ARRAY_SIZE(tItems); i++) {
		tItems[i].run = false;
		tItems[i].max_res = MP_FAIL;
		tItems[i].min_res = MP_FAIL;
		tItems[i].item_result = MP_PASS;
		if (tItems[i].result != NULL)
			sprintf(tItems[i].result, "%s", "FAIL");

		ipio_kfree((void **)&tItems[i].result_buf);
		vfree(tItems[i].buf);
		tItems[i].buf = NULL;
		ipio_kfree((void **)&tItems[i].max_buf);
		ipio_kfree((void **)&tItems[i].min_buf);

		if (tItems[i].spec_option == BENCHMARK) {
			ipio_kfree((void **)&tItems[i].bench_mark_max);
			ipio_kfree((void **)&tItems[i].bench_mark_min);
		}
	}

	ipio_kfree((void **)&frame1_cbk700);
	ipio_kfree((void **)&frame1_cbk250);
	ipio_kfree((void **)&frame1_cbk200);
	ipio_kfree((void **)&frame_buf);
	ipio_kfree((void **)&core_mp);
//	ipio_kfree((void **)&ini_data);
}

static void ili9881h_mp_test_init_item(void)
{
	int i;

	TOUCH_TRACE();

	core_mp->mp_items = ARRAY_SIZE(tItems);

	/* assign test functions run on MP flow according to their catalog */
	for (i = 0; i < core_mp->mp_items; i++) {

		tItems[i].spec_option = 0;
		tItems[i].type_option = 0;
		tItems[i].run = false;
		tItems[i].max = 0;
		tItems[i].max_res = MP_FAIL;
		tItems[i].item_result = MP_PASS;
		tItems[i].min = 0;
		tItems[i].min_res = MP_FAIL;
		tItems[i].frame_count = 0;
		tItems[i].trimmed_mean = 0;
		tItems[i].lowest_percentage = 0;
		tItems[i].highest_percentage = 0;
		tItems[i].result_buf = NULL;
		tItems[i].buf = NULL;
		tItems[i].max_buf = NULL;
		tItems[i].min_buf = NULL;
		tItems[i].bench_mark_max = NULL;
		tItems[i].bench_mark_min = NULL;
		tItems[i].node_type = NULL;

		if (tItems[i].catalog == MUTUAL_TEST) {
			tItems[i].do_test = ili9881h_mp_raw_data_test;
		} else if (tItems[i].catalog == OPEN_TEST){
			tItems[i].do_test = ili9881h_mp_open_sp_test;
		} else if (tItems[i].catalog == PEAK_TO_PEAK_TEST) {
			tItems[i].do_test = ili9881h_mp_raw_data_test;
		} else if (tItems[i].catalog == SHORT_TEST) {
			tItems[i].do_test = ili9881h_mp_raw_data_test;
		}

		tItems[i].result = kmalloc(16, GFP_KERNEL);
		snprintf(tItems[i].result, 16, "%s", "FAIL");
	}

	/* In protocol 5.4.0, these commands are read from the INI file */
}

int ili9881h_core_mp_init(struct device *dev)
{
	int ret = 0;
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_fw_info *fw_info = &d->ic_info.fw_info;
	struct ili9881h_tp_info *tp_info = &d->tp_info;

	TOUCH_TRACE();

	if (core_mp == NULL) {
		core_mp = kzalloc(sizeof(*core_mp), GFP_KERNEL);
		if (ERR_ALLOC_MEM(core_mp)) {
			TOUCH_E("Failed to init core_mp, %ld\n", PTR_ERR(core_mp));
			ret = -ENOMEM;
			goto out;
		}

		core_mp->dev = dev;

		core_mp->xch_len = tp_info->nXChannelNum;
		core_mp->ych_len = tp_info->nYChannelNum;

		core_mp->version[0] = fw_info->core;
		core_mp->version[1] = fw_info->customer_code;
		core_mp->version[2] = fw_info->major;
		core_mp->version[3] = fw_info->minor;
		//compute the total length in one frame
		core_mp->frame_len = core_mp->xch_len * core_mp->ych_len;

		core_mp->tdf = 240;
		core_mp->busy_cdc = INT_CHECK;

		core_mp->retry = false;

		ili9881h_mp_test_init_item();
	}

out:
	return ret;
}

int ili9881h_core_mp_run_test(struct device *dev, char *item)
{
	int i = 0;
	char str[512] = {0 };
	int result = 0;// 1 is PASS, -1 is FAIL

	TOUCH_TRACE();

	if (ERR_ALLOC_MEM(core_mp)) {
		TOUCH_E("core_mp is null, do nothing\n");
		return MP_FAIL;
	}

	if (item == NULL || strncmp(item, " ", strlen(item)) == 0 || core_mp->frame_len == 0 ) {
		tItems[i].result = "FAIL";
		TOUCH_E("Invaild string or length\n");
		return MP_FAIL;
	}

	TOUCH_I("item = %s\n", item);

	for (i = 0; i < core_mp->mp_items; i++) {
		if (strncmp(item, tItems[i].desp, strlen(item)) == 0) {
			ili9881h_get_ini_int_data(item, "Enable", str);
			tItems[i].run = katoi(str);
			ili9881h_get_ini_int_data(item, "SPEC Option", str);
			tItems[i].spec_option= katoi(str);
			ili9881h_get_ini_int_data(item, "Type Option", str);
			tItems[i].type_option= katoi(str);
			ili9881h_get_ini_int_data(item, "Frame Count", str);
			tItems[i].frame_count= katoi(str);
			ili9881h_get_ini_int_data(item, "Trimmed Mean", str);
			tItems[i].trimmed_mean= katoi(str);
			ili9881h_get_ini_int_data(item, "Lowest Percentage", str);
			tItems[i].lowest_percentage= katoi(str);
			ili9881h_get_ini_int_data(item, "Highest Percentage", str);
			tItems[i].highest_percentage= katoi(str);

			/* Get threshold from ini structure in parser */
			ili9881h_get_ini_int_data(item, "Max", str);
			tItems[i].max = katoi(str);
			ili9881h_get_ini_int_data(item, "Min", str);
			tItems[i].min = katoi(str);

			ili9881h_get_ini_int_data(item, "Frame Count", str);
			tItems[i].frame_count = katoi(str);

			TOUCH_D(PRODUCTION, "%s: run = %d, max = %d, min = %d, frame_count = %d\n", tItems[i].desp,
					tItems[i].run, tItems[i].max, tItems[i].min, tItems[i].frame_count);

			if (tItems[i].run) {
				/* LCM off */
				//if (strnstr(tItems[i].desp, "LCM", strlen(tItems[i].desp)) != NULL)
				//ili9881h_core_mp_ctrl_lcm_status(false);

				TOUCH_I("Running Test Item : %s\n", tItems[i].desp);
				tItems[i].do_test(i);

				if (core_mp->retry) {
					/* To see if this item needs to do retry  */
					result = ili9881h_mp_retry_comp_cdc_result(i);
					if (result == MP_FAIL) {
						TOUCH_I("MP failed, doing retry\n");
						result = ili9881h_mp_do_retry(i, RETRY_COUNT);
					}
				} else {
					result = ili9881h_mp_retry_comp_cdc_result(i);
				}

				/* LCM on */
				//if (strnstr(tItems[i].desp, "LCM", strlen(tItems[i].desp)) != NULL)
				//	ili9881h_core_mp_ctrl_lcm_status(true);
				TOUCH_I("%s result is  = %s\n", tItems[i].desp, result == MP_PASS? "PASS": "FAIL");
			}
			break;
		}
	}
	return result;
}

static ssize_t show_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	int ret = 0;
	int rawdata_ret = 0;
	int jitter_ret = 0;
	int open_ret = 0;
	int short_ret = 0;

	TOUCH_TRACE();

	TOUCH_I("Show sd Test Start!\n");

	if (atomic_read(&ts->state.core) == CORE_UPGRADE) {
		TOUCH_E("FW is upgrading, stop running mp test\n");
		return -1;
	}

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U3) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U3. Test Result : Fail\n");
		return ret;
	}

	/* Parse ini file */
	if (ili9881h_parse_ini_file(dev)) {
		TOUCH_E("Failed to parse ini file\n");
		goto out;
	}

	if (ili9881h_core_mp_init(dev)) {
		TOUCH_E("core_mp failed to initialize\n");
		goto out;
	}

	/* Switch to demo mode */
	if (ili9811h_switch_fw_mode(dev, FIRMWARE_TEST_MODE)) {
		TOUCH_E("Failed to switch test mode\n");
		goto out;
	}

	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	/* Run Items of MP test */
	open_ret = ili9881h_core_mp_run_test(dev, "Open Test(integration)_SP");
	short_ret = ili9881h_core_mp_run_test(dev, "Short Test -ILI9881");
	rawdata_ret = ili9881h_core_mp_run_test(dev, "Raw Data(No BK)");
	jitter_ret = ili9881h_core_mp_run_test(dev, "Noise Peak to Peak(IC Only)");

	/* Write final test result into a file */
	ili9881h_core_mp_show_result(dev);

	/* Write final test result into cmd terminal */
	ret = snprintf(buf, PAGE_SIZE, "\n=========== RESULT ===========\n");
	TOUCH_I("=========== RESULT ===========\n");

	if (rawdata_ret == 1 && jitter_ret == 1) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Pass\n");
		TOUCH_I("Raw Data : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Raw Data : Fail (raw:%d/jitter:%d)\n",
				(rawdata_ret == MP_FAIL)? 0 : 1, (jitter_ret == MP_FAIL)? 0 : 1);
		TOUCH_I("Raw Data : Fail (raw:%d/jitter:%d)\n",
				(rawdata_ret == MP_FAIL)? 0 : 1, (jitter_ret == MP_FAIL)? 0 : 1);
	}
	if (open_ret == 1 && short_ret == 1) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"Channel Status : Pass\n");
		TOUCH_I("Channel Status : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"Channel Status : Fail (open:%d/short:%d)\n",
			(open_ret == MP_FAIL) ? 0 : 1, (short_ret == MP_FAIL) ? 0 : 1);
		TOUCH_I("Channel Status : Fail (open:%d/short:%d)\n",
			(open_ret == MP_FAIL) ? 0 : 1, (short_ret == MP_FAIL) ? 0 : 1);
	}
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "==============================\n");
	TOUCH_I("==============================\n");

	ili9881h_reset_ctrl(dev, HW_RESET_SYNC);

	/* Switch to demo mode */
	if (ili9811h_switch_fw_mode(dev, FIRMWARE_DEMO_MODE)) {
		TOUCH_E("Failed to switch demo mode\n");
		goto out;
	}
out:
	/* Free memory must be executed if runs mp test */
	ili9881h_core_mp_test_free();
	/* Enable IRQ */
	touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);

	write_file(dev, "Show_sd Test End\n", true);
	log_file_size_check(dev);
	TOUCH_I("Show sd Test End!\n");
	return ret;
}

static ssize_t show_lpwg_sd(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
#if 0
	int lpwg_rawdata_ret = 0;
	int lpwg_jitter_ret = 0;
#endif
	int lpwg_doze_raw_ret = 0;
	int lpwg_doze_p2p_ret = 0;
	int ret = 0;

	TOUCH_TRACE();

	TOUCH_I("Show lpwg_sd Test Start\n");

	if (atomic_read(&ts->state.core) == CORE_UPGRADE) {
		TOUCH_E("FW is upgrading, stop running mp test\n");
		return -1;
	}

	/* LCD mode check */
	if (d->lcd_mode != LCD_MODE_U0) {
		ret = snprintf(buf + ret, PAGE_SIZE - ret,
			"LCD mode is not U3. Test Result : Fail\n");
		return ret;
	}

	/* Parse ini file */
	if (ili9881h_parse_ini_file(dev)) {
		TOUCH_E("Failed to parse ini file\n");
		goto out;
	}

	if (ili9881h_core_mp_init(dev) < 0) {
		TOUCH_E("core_mp failed to initialize\n");
		goto out;
	}

	/* Switch to demo mode */
	if (ili9811h_switch_fw_mode(dev, FIRMWARE_TEST_MODE)) {
		TOUCH_E("Failed to switch test mode\n");
		goto out;
	}
	mutex_lock(&ts->lock);
	touch_interrupt_control(ts->dev, INTERRUPT_DISABLE);

	/* Run Items of MP test */
#if 0
	lpwg_rawdata_ret = ili9881h_core_mp_run_test(dev, "Raw Data(No BK) (LCM OFF)");
	lpwg_jitter_ret = ili9881h_core_mp_run_test(dev, "Noise Peak To Peak(With Panel) (LCM OFF)");
#endif
	lpwg_doze_raw_ret = ili9881h_core_mp_run_test(dev, "Raw Data_TD (LCM OFF)");
	lpwg_doze_p2p_ret = ili9881h_core_mp_run_test(dev, "Peak To Peak_TD (LCM OFF)");

	/* Write final test result into a file */
	ili9881h_core_mp_show_result(dev);

	/* Write final test result into cmd terminal */
	ret = snprintf(buf, PAGE_SIZE, "\n=========== RESULT ===========\n");
	TOUCH_I("=========== RESULT ===========\n");
	if (lpwg_doze_raw_ret ==1 && lpwg_doze_p2p_ret == 1) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LPWG RawData : Pass\n");
		TOUCH_I("LPWG RawData : Pass\n");
	} else {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"LPWG RawData : Fail (doze_raw:%d/doze_p2p:%d)\n",
				(lpwg_doze_raw_ret == MP_FAIL)? 0 : 1, (lpwg_doze_p2p_ret == MP_FAIL)? 0 : 1);
		TOUCH_I("LPWG RawData : Fail (doze_raw:%d/doze_p2p:%d)\n",
				(lpwg_doze_raw_ret == MP_FAIL)? 0 : 1, (lpwg_doze_p2p_ret == MP_FAIL)? 0 : 1);
	}
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "==============================\n");
	TOUCH_I("==============================\n");

	/*ili9881h_reset_ctrl(dev, HW_RESET_ONLY);

	//Switch to demo mode
	if (ili9811h_switch_fw_mode(dev, FIRMWARE_DEMO_MODE)) {
		TOUCH_E("Failed to switch demo mode\n");
		goto out;
	}*/

out:
	/* Free memory must be executed if runs mp test */
	ili9881h_core_mp_test_free();
	//touch_interrupt_control(ts->dev, INTERRUPT_ENABLE);
	mutex_unlock(&ts->lock);
	
	write_file(dev, "Show lpwg_sd Test End\n", true);
	log_file_size_check(dev);
	TOUCH_I("Show lpwg_sd Test End\n");

	TOUCH_I("Need to turn on the lcd after lpwg_sd test (ILITEK Limitation)\n");
	return ret;
}

static ssize_t show_delta(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_debug_info *debug_info = &d->debug_info;
	struct ili9881h_tp_info *tp_info = &d->tp_info;
	int ret = 0, try = 10;
	int log_ret = 0;
	int mutual_data_size = 0;
	int16_t *delta = NULL;
	int row = 0, col = 0,  index = 0;
	int i, x, y;
	int read_length = 0;
	u8 tmp = 0x00;
	u8 check_sum = 0;

	TOUCH_TRACE();

	TOUCH_I("Show Delta Data\n");

	mutex_lock(&ts->lock);

	row = tp_info->nYChannelNum;
	col = tp_info->nXChannelNum;
	read_length = 4 + 2 * row * col + 1 ;

	delta = kcalloc(DEBUG_MODE_PACKET_LENGTH, sizeof(int16_t), GFP_KERNEL);
	if(ERR_ALLOC_MEM(delta)) {
		TOUCH_E("Failed to allocate delta mem\n");
		goto out;
	}

retry:
	memset(delta, 0, DEBUG_MODE_PACKET_LENGTH);

	tmp = 0x01; //Set signal once data mode
	ret = ili9881h_reg_write(dev, 0xB7, &tmp, 1);
	if (ret) {
		TOUCH_E("Failed to write preparation command, %d\n", ret);
		goto out;
	}

	mdelay(20);

	/* read debug packet header */
	ret = ili9881h_reg_read(dev, CMD_NONE, debug_info->data, read_length);

	tmp = 0x03; //switch to normal mode
	ret = ili9881h_reg_write(dev, 0xB7, &tmp, 1);
	if (ret) {
		TOUCH_E("Failed to write preparation command, %d\n", ret);
		goto out;
	}

	/* Do retry if it couldn't get the correct header of delta data */
	if (debug_info->data[0] != 0xB7 && try != 0) {
		TOUCH_E("It's incorrect header (0x%x) of delta, do retry (%d)\n", debug_info->data[0], try);
		mdelay(20);
		try--;
		goto retry;
	}

	//Compare checksum
	check_sum = ili9881h_calc_data_checksum(debug_info->data, 8, (read_length - 1));
	if (check_sum != debug_info->data[read_length -1]) {
		TOUCH_E("Packet check sum Error, check_sum = %d, abs_data = %d\n",
				check_sum, debug_info->data[read_length -1]);
		try--;
		goto retry;
	}

	/* Decode mutual/self delta */
	mutual_data_size = (row * col * 2);

	for (i = 4, index = 0; index < row * col; i += 2, index++) {
		delta[index] = (debug_info->data[i] << 8) + debug_info->data[i + 1];
	}

	ret = snprintf(buf, PAGE_SIZE, "======== Deltadata ========\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Header 0x%x ,Type %d, Length %d\n",debug_info->data[0], debug_info->data[3],
									((debug_info->data[2] << 8) + debug_info->data[1]));

	// print X raw only
	for (y = 0; y < row; y++) {
		char log_buf[LOG_BUF_SIZE]= {0,};
		log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", (y+1));
		log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret, "[%2d] ", (y+1));

		for (x = 0; x < col; x++) {
			int shift = y * col + x;
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%5d", delta[shift]);
			log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret, "%5d", delta[shift]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}


out:
	if (delta != NULL)
		kfree(delta);

	mutex_unlock(&ts->lock);

	return ret;
}

static ssize_t show_rawdata(struct device *dev, char *buf)
{
	struct touch_core_data *ts = to_touch_core(dev);
	struct ili9881h_data *d = to_ili9881h_data(dev);
	struct ili9881h_debug_info *debug_info = &d->debug_info;
	struct ili9881h_tp_info *tp_info = &d->tp_info;
	int ret = 0, try = 10;
	int log_ret = 0;
	int mutual_data_size = 0;
	int16_t *rawdata = NULL;
	int row = 0, col = 0,  index = 0;
	int i, x, y;
	int read_length = 0;
	u8 tmp = 0x0;
	u8 check_sum = 0;

	TOUCH_TRACE();
	TOUCH_I("Show RawData\n");

	mutex_lock(&ts->lock);

	row = tp_info->nYChannelNum;
	col = tp_info->nXChannelNum;
	read_length = 4 + 2 * row * col + 1 ;

	rawdata = kcalloc(DEBUG_MODE_PACKET_LENGTH, sizeof(int16_t), GFP_KERNEL);
	if(ERR_ALLOC_MEM(rawdata)) {
		TOUCH_E("Failed to allocate rawdata mem\n");
		goto out;
	}

retry:
	memset(rawdata, 0, DEBUG_MODE_PACKET_LENGTH);

	tmp = 0x02; //set raw once data mode
	ret = ili9881h_reg_write(dev, 0xB7, &tmp, 1);
	if (ret) {
		TOUCH_E("Failed to write preparation command, %d\n", ret);
		goto out;
	}

	mdelay(20);

	/* read debug packet header */
	ret = ili9881h_reg_read(dev, CMD_NONE, debug_info->data, read_length);

	tmp = 0x03;//switch to normal mode
	ret = ili9881h_reg_write(dev, 0xB7, &tmp, 1);
	if (ret) {
		TOUCH_E("Failed to write preparation command, %d\n", ret);
		goto out;
	}

	/* Do retry if it couldn't get the correct header of raw data */
	if (debug_info->data[0] != 0xB7 && try != 0) {
		TOUCH_E("It's incorrect header (0x%x) of rawdata, do retry (%d)\n", debug_info->data[0], try);
		mdelay(20);
		try--;
		goto retry;
	}

	//Compare checksum
	check_sum = ili9881h_calc_data_checksum(debug_info->data, 8, (read_length - 1));
	if (check_sum != debug_info->data[read_length -1]) {
		TOUCH_E("Packet check sum Error, check_sum = %d, abs_data = %d\n",
				check_sum, debug_info->data[read_length -1]);
		try--;
		goto retry;
	}

	/* Decode mutual/self rawdata */
	mutual_data_size = (row * col * 2);

	for (i = 4, index = 0; index < row * col; i += 2, index++) {
		rawdata[index] = (debug_info->data[i] << 8) + debug_info->data[i + 1];
	}

	ret = snprintf(buf, PAGE_SIZE, "======== Rawdata ========\n");
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "Header 0x%x ,Type %d, Length %d\n",debug_info->data[0], debug_info->data[3],
									((debug_info->data[2] << 8) + debug_info->data[1]));

	// print X raw only
	for (y = 0; y < row; y++) {
		char log_buf[LOG_BUF_SIZE]= {0,};
		log_ret = 0;

		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%2d] ", (y+1));
		log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret, "[%2d] ", (y+1));

		for (x = 0; x < col; x++) {
			int shift = y * col + x;
			ret += snprintf(buf + ret, PAGE_SIZE - ret, "%5d", rawdata[shift]);
			log_ret += snprintf(log_buf + log_ret, sizeof(log_buf) - log_ret, "%5d", rawdata[shift]);
		}
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		TOUCH_I("%s\n", log_buf);
	}


out:
	if (rawdata != NULL)
		kfree(rawdata);

	mutex_unlock(&ts->lock);

	return ret;
}

static TOUCH_ATTR(sd, show_sd, NULL);
static TOUCH_ATTR(delta, show_delta, NULL);
// static TOUCH_ATTR(fdata, show_fdata, NULL);
static TOUCH_ATTR(rawdata, show_rawdata, NULL);
static TOUCH_ATTR(lpwg_sd, show_lpwg_sd, NULL);
// static TOUCH_ATTR(label, show_labeldata, NULL);
// static TOUCH_ATTR(debug, show_debug, NULL);
// static TOUCH_ATTR(base, show_base, NULL);


static struct attribute *mp_attribute_list[] = {
	&touch_attr_sd.attr,
	&touch_attr_delta.attr,
	// &touch_attr_fdata.attr,
	&touch_attr_rawdata.attr,
	&touch_attr_lpwg_sd.attr,
	// &touch_attr_label.attr,
	// &touch_attr_debug.attr,
	// &touch_attr_base.attr,
	NULL,
};

static const struct attribute_group mp_attribute_group = {
	.attrs = mp_attribute_list,
};

int ili9881h_mp_register_sysfs(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	ret = sysfs_create_group(&ts->kobj, &mp_attribute_group);

	if (ret < 0)
		TOUCH_E("failed to create sysfs\n");

	return ret;
}
