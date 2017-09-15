/*
 * Copyright (C) 2010 - 2016 Novatek, Inc.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "NVTtouch_207.h"
#include "NVTtouch_207_mp_ctrlram.h"


#if NVT_TOUCH_MP

#define TEST_MODE_OPEN 0x40
#define TEST_MODE_SHORT_RXRX 0x41
#define TEST_MODE_SHORT_RXRX1 0x42
#define TEST_MODE_BUTTON 0x43

static uint8_t AIN_RX_Order[IC_RX_CFG_SIZE] = {0};
static uint8_t AIN_TX_Order[IC_TX_CFG_SIZE] = {0};

#define MaxStatisticsBuf 100
static int64_t StatisticsNum[MaxStatisticsBuf];
static int64_t StatisticsSum[MaxStatisticsBuf];
static int64_t golden_Ratio[40 * 40] = {0};
static uint8_t RecordResultShort_RXRX[40] = {0};
static uint8_t RecordResultOpen[40 * 40] = {0};
static uint8_t RecordResultPixelRaw[40 * 40] = {0};
#if TOUCH_KEY_NUM > 0
static uint8_t RecordResultBtn[Btn_Number] = {0};
#endif /* #if TOUCH_KEY_NUM > 0 */

static int32_t TestResult_Short_RXRX = 0;
static int32_t TestResult_Open = 0;
static int32_t TestResult_PixelRaw = 0;
#if TOUCH_KEY_NUM > 0
static int32_t TestResult_Btn = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

static int32_t rawdata_short_rxrx[40] = {0};
static int32_t rawdata_short_rxrx0[40] = {0};
static int32_t rawdata_short_rxrx1[40] = {0};
static int32_t rawdata_open[40 * 40] = {0};
static int64_t PixelRawCmRatio[40 * 40] = {0};
static int32_t PixelRawCmRatioMax[40 * 40] = {0};
#if TOUCH_KEY_NUM > 0
static int32_t rawdata_btn[Btn_Number] = {0};
#endif /* #if TOUCH_KEY_NUM > 0 */

static struct proc_dir_entry *NVT_proc_selftest_entry = NULL;
static int8_t nvt_mp_test_result_printed = 0;

extern struct nvt_ts_data *ts;
extern void nvt_hw_reset(void);
extern int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address);
extern int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len);
extern void nvt_sw_reset_idle(void);
extern void nvt_bootloader_reset(void);
extern int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
extern int32_t nvt_clear_fw_status(void);
extern void nvt_change_mode(uint8_t mode);
extern int32_t nvt_check_fw_status(void);

static void nvt_cal_ain_order(void)
{
	uint32_t i = 0;

	for (i = 0; i < IC_RX_CFG_SIZE; i++) {
		if (AIN_RX[i] == 0xFF)
			continue;
		AIN_RX_Order[AIN_RX[i]] = (uint8_t)i;
	}

	for (i = 0; i < IC_TX_CFG_SIZE; i++) {
		if (AIN_TX[i] == 0xFF)
			continue;
		AIN_TX_Order[AIN_TX[i]] = (uint8_t)i;
	}

	NVT_LOG("AIN_RX_Order:\n");
	for (i = 0; i < IC_RX_CFG_SIZE; i++)
		NVT_LOG("%d, ", AIN_RX_Order[i]);
	NVT_LOG("\n");

	NVT_LOG("AIN_TX_Order:\n");
	for (i = 0; i < IC_TX_CFG_SIZE; i++)
		NVT_LOG("%d, ", AIN_TX_Order[i]);
	NVT_LOG("\n");
}

#define ABS(x)	(((x) < 0) ? -(x) : (x))
/*******************************************************
Description:
	Novatek touchscreen read short test RX-RX raw data
	function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_read_short_rxrx(void)
{
	int32_t i = 0;
	uint8_t buf[128] = {0};
	struct file *fp = NULL;
	char *fbufp = NULL;
	mm_segment_t org_fs;
	char file_path[64] = "/data/local/tmp/ShortTestRX-RX.csv";
	loff_t pos = 0;
	int32_t write_ret = 0;
	uint32_t output_len = 0;
	int16_t sh = 0;
	int16_t sh1 = 0;

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_SHORT_RXRX);

	if (nvt_check_fw_status()) {
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, IC_RX_CFG_SIZE * 2 + 1);
	for (i = 0; i < AIN_RX_NUM; i++) {
		rawdata_short_rxrx0[i] = (int16_t)(buf[AIN_RX_Order[i] * 2 + 1] + 256 * buf[AIN_RX_Order[i] * 2 + 2]);
	}

	nvt_bootloader_reset();
	if (nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}

	if (nvt_clear_fw_status()) {
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_SHORT_RXRX1);

	if (nvt_check_fw_status()) {
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	buf[0] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, IC_RX_CFG_SIZE * 2 + 1);
	for (i = 0; i < AIN_RX_NUM; i++) {
		rawdata_short_rxrx1[i] = (int16_t)(buf[AIN_RX_Order[i] * 2 + 1] + 256 * buf[AIN_RX_Order[i] * 2 + 2]);
	}

	for (i = 0; i < AIN_RX_NUM; i++) {
		sh = rawdata_short_rxrx0[i];
		sh1 = rawdata_short_rxrx1[i];
		if (ABS(sh) < ABS(sh1))
			sh = sh1;
		rawdata_short_rxrx[i] = sh;
	}

	fbufp = kzalloc(8192, GFP_KERNEL);
	if (!fbufp) {
		NVT_ERR("kzalloc for fbufp failed.\n");
		return -ENOMEM;
	}

	NVT_LOG("%s:\n", __func__);
	for (i = 0; i < AIN_RX_NUM; i++) {
		NVT_LOG("%5d, ", rawdata_short_rxrx[i]);
		snprintf(fbufp + 7 * i, (8192 - 7 * i), "%5d, ", rawdata_short_rxrx[i]);
	}
	NVT_LOG("\n");

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file_path, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open %s failed\n", file_path);
		set_fs(org_fs);
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	output_len = AIN_RX_NUM * 7;
	write_ret = vfs_write(fp, (char __user *)fbufp, output_len, &pos);
	if (write_ret <= 0) {
		NVT_ERR("write %s failed\n", file_path);
		set_fs(org_fs);
		if (fp) {
			filp_close(fp, NULL);
			fp = NULL;
		}
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	set_fs(org_fs);
	if (fp) {
		filp_close(fp, NULL);
		fp = NULL;
	}
	if (fbufp != NULL) {
		kfree(fbufp);
		fbufp = NULL;
	}

	NVT_LOG("--\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen read open test raw data function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_read_open(void)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
	uint8_t buf[128] = {0};
	struct file *fp = NULL;
	char *fbufp = NULL;
	mm_segment_t org_fs;
	char file_path[64] = "/data/local/tmp/OpenTest.csv";
	loff_t pos = 0;
	int32_t write_ret = 0;
	uint32_t output_len = 0;
	uint8_t *rawdata_buf = NULL;

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_OPEN);

	if (nvt_check_fw_status()) {
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	rawdata_buf = kzalloc(AIN_TX_NUM * AIN_RX_NUM * 2, GFP_KERNEL);
	if (!rawdata_buf) {
		NVT_ERR("kzalloc for rawdata_buf failed.\n");
		return -ENOMEM;
	}

	for (i = 0; i < AIN_TX_NUM; i++) {
		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0x00 + (uint8_t)(((i * AIN_RX_NUM * 2) & 0xFF00) >> 8);
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);
		buf[0] = (uint8_t)((i * AIN_RX_NUM * 2) & 0xFF);
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, AIN_RX_NUM * 2 + 1);
		memcpy(rawdata_buf + i * AIN_RX_NUM * 2, buf + 1, AIN_RX_NUM * 2);
	}
	for (i = 0; i < AIN_TX_NUM; i++) {
		for (j = 0; j < AIN_RX_NUM; j++) {
			iArrayIndex = i * AIN_RX_NUM + j;
			rawdata_open[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2]
				+ 256 * rawdata_buf[iArrayIndex * 2 + 1]);
		}
	}
	if (rawdata_buf  != NULL) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	fbufp = kzalloc(8192, GFP_KERNEL);
	if (!fbufp) {
		NVT_ERR("kzalloc for fbufp failed.\n");
		return -ENOMEM;
	}

	NVT_LOG("%s:\n", __func__);
	for (i = 0; i < AIN_TX_NUM; i++) {
		for (j = 0; j < AIN_RX_NUM; j++) {
			iArrayIndex = i * AIN_RX_NUM + j;
			NVT_LOG("%5d, ", rawdata_open[iArrayIndex]);
			snprintf(fbufp + 7 * iArrayIndex + i * 2, (8192 - 7 * iArrayIndex + i * 2),
				"%5d, ", rawdata_open[iArrayIndex]);
		}
		NVT_LOG("\n");
		snprintf(fbufp + 7 * (iArrayIndex + 1) + i * 2, (8192 - 7 * (iArrayIndex + 1) + i * 2), "\r\n");
	}
	NVT_LOG("\n");

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file_path, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open %s failed\n", file_path);
		set_fs(org_fs);
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	output_len = AIN_TX_NUM * AIN_RX_NUM * 7 + AIN_TX_NUM * 2;
	write_ret = vfs_write(fp, (char __user *)fbufp, output_len, &pos);
	if (write_ret <= 0) {
		NVT_ERR("write %s failed\n", file_path);
		set_fs(org_fs);
		if (fp) {
			filp_close(fp, NULL);
			fp = NULL;
		}
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	set_fs(org_fs);
	if (fp) {
		filp_close(fp, NULL);
		fp = NULL;
	}
	if (fbufp != NULL) {
		kfree(fbufp);
		fbufp = NULL;
	}

	NVT_LOG("--\n");
	return 0;
}

#if TOUCH_KEY_NUM > 0
/*******************************************************
Description:
	Novatek touchscreen read btn test raw data function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_read_btn(void)
{
	int32_t i = 0;
	uint8_t buf[128] = {0};
	struct file *fp = NULL;
	char *fbufp = NULL;
	mm_segment_t org_fs;
	char file_path[64] = "/data/local/tmp/BtnTest.csv";
	loff_t pos = 0;
	int32_t write_ret = 0;
	uint32_t output_len = 0;
	uint8_t *rawdata_buf = NULL;

	NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_BUTTON);

	if (nvt_check_fw_status()) {
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	rawdata_buf = kzalloc(Btn_Number * 2, GFP_KERNEL);
	if (!rawdata_buf) {
		NVT_ERR("kzalloc for rawdata_buf failed\n");
		return -ENOMEM;
	}

	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);
	buf[0] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, Btn_Number * 2 + 1);
	memcpy(rawdata_buf, buf + 1, Btn_Number * 2);

	for (i = 0; i < Btn_Number; i++) {
		rawdata_btn[i] = (int16_t)(rawdata_buf[i * 2] + 256 * rawdata_buf[i * 2 + 1]);
	}
	if (rawdata_buf != NULL) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	fbufp = kzalloc(8192, GFP_KERNEL);
	if (!fbufp) {
		NVT_ERR("kzalloc for fbufp failed.\n");
		return -ENOMEM;
	}

	NVT_LOG("%s:\n", __func__);
	for (i = 0; i < Btn_Number; i++) {
		NVT_LOG("%5d, ", rawdata_btn[i]);
		snprintf(fbufp + 7 * i, (8192 - 7 * i), "%5d, ", rawdata_btn[i]);
	}
	NVT_LOG("\n");
	snprintf(fbufp + 7 * Btn_Number, (8192 - 7 * Btn_Number), "\r\n");

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file_path, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open %s failed\n", file_path);
		set_fs(org_fs);
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	output_len = Btn_Number * 7 + 2;
	write_ret = vfs_write(fp, (char __user *)fbufp, output_len, &pos);
	if (write_ret <= 0) {
		NVT_ERR("write %s failed\n", file_path);
		set_fs(org_fs);
		if (fp) {
			filp_close(fp, NULL);
			fp = NULL;
		}
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	set_fs(org_fs);
	if (fp) {
		filp_close(fp, NULL);
		fp = NULL;
	}
	if (fbufp != NULL) {
		kfree(fbufp);
		fbufp = NULL;
	}

	NVT_LOG("--\n");
	return 0;
}
#endif /* #if TOUCH_KEY_NUM > 0 */

/*******************************************************
Description:
	Novatek touchscreen calculate G Ratio and Normal
	function.

return:
	Executive outcomes. 0---succeed. 1---failed.
*******************************************************/
static int32_t Test_CaluateGRatioAndNormal(int32_t boundary[], int32_t rawdata[], uint8_t x_len, uint8_t y_len)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	int64_t tmpValue = 0;
	int64_t MaxSum = 0;
	int32_t SumCnt = 0;
	int32_t MaxNum = 0;
	int32_t MaxIndex = 0;
	int32_t Max = -99999999;
	int32_t Min =  99999999;
	int32_t offset = 0;
	int32_t Data = 0;
	int32_t StatisticsStep = 0;

	for (j = 0; j < y_len; j++) {
		for (i = 0; i < x_len; i++) {
			Data = rawdata[j * x_len + i];
			if (Data == 0)
				Data = 1;

			golden_Ratio[j * x_len + i] = Data - boundary[j * x_len + i];
			golden_Ratio[j * x_len + i] = ((golden_Ratio[j * x_len + i] * 1000) / Data);
		}
	}


	for (j = 0; j < y_len; j++) {
		for (i = 0; i < x_len; i++) {
			golden_Ratio[j * x_len + i] *= 1000;
		}
	}

	for (j = 0; j < y_len; j++) {
		for (i = 0; i < x_len; i++) {
			if (Max < golden_Ratio[j * x_len + i])
				Max = (int32_t)golden_Ratio[j * x_len + i];
			if (Min > golden_Ratio[j * x_len + i])
				Min = (int32_t)golden_Ratio[j * x_len + i];
		}
	}

	offset = 0;
	if (Min < 0) {
		offset = 0 - Min;
		for (j = 0; j < y_len; j++) {
			for (i = 0; i < x_len; i++) {
				golden_Ratio[j * x_len + i] += offset;
			}
		}
		Max += offset;
	}
	StatisticsStep = Max / MaxStatisticsBuf;
	StatisticsStep += 1;
	if (StatisticsStep < 0) {
		NVT_ERR("FAIL! (StatisticsStep < 0)\n");
		return 1;
	}

	memset(StatisticsSum, 0, sizeof(int64_t) * MaxStatisticsBuf);
	memset(StatisticsNum, 0, sizeof(int64_t) * MaxStatisticsBuf);
	for (i = 0; i < MaxStatisticsBuf; i++) {
		StatisticsSum[i] = 0;
		StatisticsNum[i] = 0;
	}
	for (j = 0; j < y_len; j++) {
		for (i = 0; i < x_len; i++) {
			tmpValue = golden_Ratio[j * x_len + i];
			tmpValue /= StatisticsStep;
			StatisticsNum[tmpValue] += 2;
			StatisticsSum[tmpValue] += (2 * golden_Ratio[j * x_len + i]);

			if ((tmpValue + 1) < MaxStatisticsBuf) {
				StatisticsNum[tmpValue + 1] += 1;
				StatisticsSum[tmpValue + 1] += golden_Ratio[j * x_len + i];
			}

			if ((tmpValue - 1) >= 0) {
				StatisticsNum[tmpValue - 1] += 1;
				StatisticsSum[tmpValue - 1] += golden_Ratio[j * x_len + i];
			}
		}
	}

	MaxNum = 0;
	for (k = 0; k < MaxStatisticsBuf; k++) {
		if (MaxNum < StatisticsNum[k]) {
			MaxSum = StatisticsSum[k];
			MaxNum = StatisticsNum[k];
			MaxIndex = k;
		}
	}

	if (MaxSum > 0) {
		if (StatisticsNum[MaxIndex] != 0) {
			tmpValue = (int64_t)(StatisticsSum[MaxIndex] / StatisticsNum[MaxIndex]) * 2;
			SumCnt += 2;
		}

		if ((MaxIndex + 1) < (MaxStatisticsBuf)) {
			if (StatisticsNum[MaxIndex + 1] != 0) {
				tmpValue += (int64_t)(StatisticsSum[MaxIndex + 1] / StatisticsNum[MaxIndex + 1]);
				SumCnt++;
			}
		}

		if ((MaxIndex - 1) >= 0) {
			if (StatisticsNum[MaxIndex - 1] != 0) {
				tmpValue += (int64_t)(StatisticsSum[MaxIndex - 1] / StatisticsNum[MaxIndex - 1]);
				SumCnt++;
			}
		}

		if (SumCnt > 0)
			tmpValue /= SumCnt;
	} else {
		StatisticsSum[0] = 0;
		StatisticsNum[0] = 0;
		for (j = 0; j < y_len; j++) {
			for (i = 0; i < x_len; i++) {
				StatisticsSum[0] += (int64_t)golden_Ratio[j * x_len + i];
				StatisticsNum[0]++;
			}
		}
		tmpValue = StatisticsSum[0] / StatisticsNum[0];
	}

	tmpValue -= offset;
	for (j = 0; j < y_len; j++) {
		for (i = 0; i < x_len; i++) {
			golden_Ratio[j * x_len + i] -= offset;

			golden_Ratio[j * x_len + i] = golden_Ratio[j * x_len + i] - tmpValue;
			golden_Ratio[j * x_len + i] = golden_Ratio[j * x_len + i] / 1000;
		}
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen raw data test function.

return:
	Executive outcomes. 0---passed. negative---failed.
*******************************************************/
static int32_t RawDataTest_Sub(int32_t boundary[], int32_t rawdata[], uint8_t RecordResult[],
							uint8_t x_ch, uint8_t y_ch,
							int32_t Tol_P, int32_t Tol_N,
							int32_t Dif_P, int32_t Dif_N,
							int32_t Rawdata_Limit_Postive, int32_t Rawdata_Limit_Negative)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
	int32_t iBoundary = 0;
	int32_t iTolLowBound = 0;
	int32_t iTolHighBound = 0;
	bool isAbsCriteria = false;
	bool isPass = true;

	if ((Rawdata_Limit_Postive != 0) || (Rawdata_Limit_Negative != 0))
		isAbsCriteria = true;

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			iBoundary = boundary[iArrayIndex];

			RecordResult[iArrayIndex] = 0x00;

			if (isAbsCriteria) {
				iTolLowBound = Rawdata_Limit_Negative;
				iTolHighBound = Rawdata_Limit_Postive;
			} else {
				if (iBoundary > 0) {
					iTolLowBound = (iBoundary * (1000 + Tol_N));
					iTolHighBound = (iBoundary * (1000 + Tol_P));
				} else {
					iTolLowBound = (iBoundary * (1000 - Tol_N));
					iTolHighBound = (iBoundary * (1000 - Tol_P));
				}
			}

			if ((rawdata[iArrayIndex] * 1000) > iTolHighBound)
				RecordResult[iArrayIndex] |= 0x01;

			if ((rawdata[iArrayIndex] * 1000) < iTolLowBound)
				RecordResult[iArrayIndex] |= 0x02;
		}
	}

	if (!isAbsCriteria) {
		Test_CaluateGRatioAndNormal(boundary, rawdata, x_ch, y_ch);

		for (j = 0; j < y_ch; j++) {
			for (i = 0; i < x_ch; i++) {
				iArrayIndex = j * x_ch + i;

				if (golden_Ratio[iArrayIndex] > Dif_P)
					RecordResult[iArrayIndex] |= 0x04;

				if (golden_Ratio[iArrayIndex] < Dif_N)
					RecordResult[iArrayIndex] |= 0x08;
			}
		}
	}

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			if (RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}

	if (isPass == false) {
		return -EPERM;
	} else {
		return 0;
	}
}

#define MAX(a, b) ((a) > (b) ? (a) : (b))
/*******************************************************
Description:
	Novatek touchscreen raw data test function.

return:
	Executive outcomes. 0---passed. negative---failed.
*******************************************************/
static int32_t PixelRawTest_Sub(int32_t rawdata[], int64_t PixelRawCmRatio[],
							int32_t PixelRawCmRatioMax[], uint8_t RecordResult[],
								uint8_t x_ch, uint8_t y_ch, int32_t PixelRaw_Diff)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
	int32_t up = 0;
	int32_t down = 0;
	int32_t right = 0;
	int32_t left = 0;
	int64_t tmpRatio[4] = {0};
	bool isPass = true;
	struct file *fp = NULL;
	char *fbufp = NULL;
	mm_segment_t org_fs;
	char file_path[64] = "/data/local/tmp/PixelRaw.csv";
	loff_t pos = 0;
	int32_t write_ret = 0;
	uint32_t output_len = 0;

	NVT_LOG("++\n");

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			RecordResult[iArrayIndex] = 0x00;
			PixelRawCmRatio[iArrayIndex] = (int64_t)rawdata[iArrayIndex];
		}
	}

	for (j = 0; j < y_ch; j++) {
		if (j == 0) {
			up = 0;
			down = j + 1;
		} else if (j == y_ch - 1) {
			up = j - 1;
			down = y_ch - 1;
		} else {
			up = j - 1;
			down = j + 1;
		}
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;

			if (i == 0) {
				left = 0;
				right = i + 1;
			} else if (i == x_ch - 1) {
				left = i - 1;
				right = x_ch - 1;
			} else {
				left = i - 1;
				right = i + 1;
			}

			tmpRatio[0] = ABS(PixelRawCmRatio[up * x_ch + i] - PixelRawCmRatio[iArrayIndex]);
			tmpRatio[1] = ABS(PixelRawCmRatio[down * x_ch + i] - PixelRawCmRatio[iArrayIndex]);
			tmpRatio[2] = ABS(PixelRawCmRatio[j * x_ch + left] - PixelRawCmRatio[iArrayIndex]);
			tmpRatio[3] = ABS(PixelRawCmRatio[j * x_ch + right] - PixelRawCmRatio[iArrayIndex]);
			PixelRawCmRatioMax[iArrayIndex] = (int32_t) MAX(MAX(tmpRatio[0], tmpRatio[1]),
						MAX(tmpRatio[2], tmpRatio[3]));
		}
	}

	fbufp = kzalloc(8192, GFP_KERNEL);
	if (!fbufp) {
		NVT_ERR("kzalloc for fbufp failed.\n");
		return -ENOMEM;
	}

	NVT_LOG("%s:\n", __func__);
	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			NVT_LOG("%5d, ", PixelRawCmRatioMax[iArrayIndex]);
			snprintf(fbufp + 7 * iArrayIndex + j * 2, (8192 - 7 * iArrayIndex + j * 2),
				"%5d, ", PixelRawCmRatioMax[iArrayIndex]);
		}
		NVT_LOG("\n");
		snprintf(fbufp + 7 * (iArrayIndex + 1) + j * 2, (8192 - 7 * (iArrayIndex + 1) + j * 2), "\r\n");
	}
	NVT_LOG("\n");

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file_path, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open %s failed\n", file_path);
		set_fs(org_fs);
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}
	output_len = y_ch * x_ch * 7 + y_ch * 2;
	write_ret = vfs_write(fp, (char __user *)fbufp, output_len, &pos);
	if (write_ret <= 0) {
		NVT_ERR("write %s failed\n", file_path);
		set_fs(org_fs);
		if (fp) {
			filp_close(fp, NULL);
			fp = NULL;
		}
		if (fbufp != NULL) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -EPERM;
	}

	set_fs(org_fs);
	if (fp) {
		filp_close(fp, NULL);
		fp = NULL;
	}
	if (fbufp != NULL) {
		kfree(fbufp);
		fbufp = NULL;
	}

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			if (PixelRawCmRatioMax[iArrayIndex] > PixelRaw_Diff) {
				RecordResult[iArrayIndex] |= 0x01;
				isPass = false;
				break;
			}
		}
	}

	NVT_LOG("--\n");

	if (isPass == false) {
		return -EPERM;
	} else {
		return 0;
	}

}

#if TOUCH_KEY_NUM > 0
static int32_t BtnTest_Sub(int32_t boundary[], int32_t rawdata[], uint8_t RecordResult[],
							uint8_t x_ch, uint8_t y_ch,
							int32_t Tol_P, int32_t Tol_N)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
	int32_t iBoundary = 0;
	int32_t iTolLowBound = 0;
	int32_t iTolHighBound = 0;
	bool isPass = true;

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;
			iBoundary = boundary[iArrayIndex];

			RecordResult[iArrayIndex] = 0x00;
			if (iBoundary > 0) {
				iTolLowBound = (iBoundary * (1000 + Tol_N));
				iTolHighBound = (iBoundary * (1000 + Tol_P));
			} else {
				iTolLowBound = (iBoundary * (1000 - Tol_N));
				iTolHighBound = (iBoundary * (1000 - Tol_P));
			}

			if ((rawdata[iArrayIndex] * 1000) > iTolHighBound)
				RecordResult[iArrayIndex] |= 0x01;

			if ((rawdata[iArrayIndex] * 1000) < iTolLowBound)
				RecordResult[iArrayIndex] |= 0x02;
		}
	}

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			if (RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}

	if (isPass == false) {
		return -EPERM;
	} else {
		return 0;
	}
}
#endif /* #if TOUCH_KEY_NUM > 0 */

/*******************************************************
Description:
	Novatek touchscreen print self-test result function.

return:
	n.a.
*******************************************************/
static void print_selftest_result(struct seq_file *m, int32_t TestResult,
			uint8_t RecordResult[], int32_t rawdata[], uint8_t x_len, uint8_t y_len)
{
	int32_t i = 0;
	int32_t j = 0;

		switch (TestResult) {
		case 0:
			seq_printf(m, " PASS!");
			seq_puts(m, "\n");
			if (!nvt_mp_test_result_printed)
				NVT_LOG(" PASS!\n");
			break;

		case 1:
			seq_printf(m, " ERROR! Read Data FAIL!");
			seq_puts(m, "\n");
			if (!nvt_mp_test_result_printed)
				NVT_LOG(" ERROR! Read Data FAIL!\n");
			break;

		case -1:
			seq_printf(m, " FAIL!");
			seq_puts(m, "\n");
			if (!nvt_mp_test_result_printed)
				NVT_LOG(" FAIL!\n");
			seq_printf(m, "RecordResult:");
			seq_puts(m, "\n");
			if (!nvt_mp_test_result_printed)
				NVT_LOG("RecordResult:\n");
			for (i = 0; i < y_len; i++) {
				for (j = 0; j < x_len; j++) {
					seq_printf(m, "0x%02X, ", RecordResult[i * x_len + j]);
					if (!nvt_mp_test_result_printed)
						NVT_LOG("0x%02X, ", RecordResult[i * x_len + j]);
				}
				seq_puts(m, "\n");
				if (!nvt_mp_test_result_printed)
					NVT_LOG("\n");
			}
			seq_printf(m, "ReadData:");
			seq_puts(m, "\n");
			if (!nvt_mp_test_result_printed)
				NVT_LOG("ReadData:\n");
			for (i = 0; i < y_len; i++) {
				for (j = 0; j < x_len; j++) {
					seq_printf(m, "%5d, ", rawdata[i * x_len + j]);
					if (!nvt_mp_test_result_printed)
						NVT_LOG("%5d, ", rawdata[i * x_len + j]);
				}
				seq_puts(m, "\n");
				if (!nvt_mp_test_result_printed)
					NVT_LOG("\n");
			}
			break;
	}
	seq_printf(m, "\n");
	if (!nvt_mp_test_result_printed)
		NVT_LOG("\n");
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show_selftest(struct seq_file *m, void *v)
{
	seq_printf(m, "Short Test RXRX");
	if (!nvt_mp_test_result_printed)
		NVT_LOG("Short Test RXRX");
	print_selftest_result(m, TestResult_Short_RXRX, RecordResultShort_RXRX, rawdata_short_rxrx, AIN_RX_NUM, 1);

	seq_printf(m, "Open Test");
	if (!nvt_mp_test_result_printed)
		NVT_LOG("Open Test");
	print_selftest_result(m, TestResult_Open, RecordResultOpen, rawdata_open, AIN_RX_NUM, AIN_TX_NUM);

	seq_printf(m, "PixelRaw Test");
	if (!nvt_mp_test_result_printed)
		NVT_LOG("PixelRaw Test");
	print_selftest_result(m, TestResult_PixelRaw, RecordResultPixelRaw, PixelRawCmRatioMax, AIN_RX_NUM, AIN_TX_NUM);

#if TOUCH_KEY_NUM > 0
	seq_printf(m, "Button Test");
	if (!nvt_mp_test_result_printed)
		NVT_LOG("Button Test");
	print_selftest_result(m, TestResult_Btn, RecordResultBtn, rawdata_btn, Btn_Number, 1);
#endif /* #if TOUCH_KEY_NUM > 0 */

	nvt_mp_test_result_printed = 1;
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
}

static const struct seq_operations nvt_selftest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show_selftest
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_selftest open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_selftest_open(struct inode *inode, struct file *file)
{
	TestResult_Short_RXRX = 0;
	TestResult_Open = 0;
	TestResult_PixelRaw = 0;
#if TOUCH_KEY_NUM > 0
	TestResult_Btn = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	nvt_cal_ain_order();

	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	if (nvt_read_short_rxrx() != 0) {
		TestResult_Short_RXRX = 1;
	} else {
		TestResult_Short_RXRX = RawDataTest_Sub(BoundaryShort_RXRX,
								rawdata_short_rxrx,
								RecordResultShort_RXRX,
								AIN_RX_NUM, 1, 0, 0, 0, 0,
						PSConfig_Rawdata_Limit_Postive_Short_RXRX,
						PSConfig_Rawdata_Limit_Negative_Short_RXRX);
	}

	nvt_bootloader_reset();
	if (nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}

	if (nvt_read_open() != 0) {
		TestResult_Open = 1;
	} else {
		TestResult_Open = RawDataTest_Sub(BoundaryOpen,
											rawdata_open,
											RecordResultOpen,
									AIN_RX_NUM, AIN_TX_NUM,
						PSConfig_Tolerance_Postive_Mutual,
						PSConfig_Tolerance_Negative_Mutual,
						PSConfig_DiffLimitG_Postive_Mutual,
						PSConfig_DiffLimitG_Negative_Mutual,
						0, 0);
	}

	if (TestResult_Open == 1) {
		TestResult_PixelRaw = 1;
	} else {
		TestResult_PixelRaw = PixelRawTest_Sub(rawdata_open,
									PixelRawCmRatio,
									PixelRawCmRatioMax,
									RecordResultPixelRaw,
									AIN_RX_NUM, AIN_TX_NUM,
									PSConfig_PixelRaw_Diff);
	}

#if TOUCH_KEY_NUM > 0
	nvt_bootloader_reset();
	if (nvt_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}

	if (nvt_read_btn() != 0) {
		TestResult_Btn = 1;
	} else {
		TestResult_Btn = BtnTest_Sub(BoundaryBtn, rawdata_btn, RecordResultBtn,
										Btn_Number, 1,
										PSConfig_Tolerance_Postive_Btn,
										PSConfig_Tolerance_Negative_Btn);
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	nvt_hw_reset();
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);

	mutex_unlock(&ts->lock);
	nvt_mp_test_result_printed = 0;
	return seq_open(file, &nvt_selftest_seq_ops);
}

static const struct file_operations nvt_selftest_fops = {
	.owner = THIS_MODULE,
	.open = nvt_selftest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen MP function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_mp_proc_init(void)
{
	NVT_proc_selftest_entry = proc_create("nvt_selftest", 0444, NULL, &nvt_selftest_fops);
	if (NVT_proc_selftest_entry == NULL) {
		NVT_ERR("create /proc/nvt_selftest Failed!\n");
		return -EPERM;
	}

	NVT_LOG("create /proc/nvt_selftest Succeeded!\n");
	return 0;
}
#endif
