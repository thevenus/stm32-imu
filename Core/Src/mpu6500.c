/*
 * mpu9250.c
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#include <mpu6500.h>
#include "stdio.h"

// ====== INTERNAL FUNCTIONS ===============================
static inline void i2c_start(struct MPU_Handle *mpu)
{
	if (mpu == NULL)
		return;

	LL_I2C_GenerateStartCondition(mpu->i2cx);
	while (!LL_I2C_IsActiveFlag_SB(mpu->i2cx));
}

static inline void i2c_stop(struct MPU_Handle *mpu)
{
	LL_I2C_GenerateStopCondition(mpu->i2cx);
}

static inline void i2c_ack(struct MPU_Handle *mpu)
{
	LL_I2C_AcknowledgeNextData(mpu->i2cx, LL_I2C_ACK);
}

static inline void i2c_nack(struct MPU_Handle *mpu)
{
	LL_I2C_AcknowledgeNextData(mpu->i2cx, LL_I2C_NACK);
}

// rw = 0 -> write operation
// rw = 1 -> read operation
static inline void i2c_send_addr(struct MPU_Handle *mpu, uint8_t address, uint8_t rw)
{
	if (mpu == NULL)
		return;

	LL_I2C_TransmitData8(mpu->i2cx, (address << 1)+rw);
	while (!LL_I2C_IsActiveFlag_ADDR(mpu->i2cx));
	LL_I2C_ClearFlag_ADDR(mpu->i2cx);
}

static inline void i2c_send_data(struct MPU_Handle *mpu, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return;
	if (pdata == NULL)
		return;

	for (uint8_t i = 0; i < count; i++) {
		LL_I2C_TransmitData8(mpu->i2cx, pdata[i]);
		while (!LL_I2C_IsActiveFlag_TXE(mpu->i2cx));
	}
}

static inline void i2c_recv_data(struct MPU_Handle *mpu, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return;
	if (pdata == NULL)
		return;

	for (uint8_t i=0; i < count; i++) {
		if (i < count - 1) {
			i2c_ack(mpu);
		} else {
			i2c_nack(mpu);
		}

		while (!LL_I2C_IsActiveFlag_RXNE(mpu->i2cx));
		pdata[i] = LL_I2C_ReceiveData8(mpu->i2cx);
	}
}

// ====== END INTERNAL FUNCTIONS ===========================
// ---------------------------------------------------------
// ---------------------------------------------------------
// ====== LIBRARY FUNCTIONS ================================
enum MPU_Status MPU_Init(struct MPU_Handle *mpu, I2C_TypeDef *i2cx)
{
	if (mpu == NULL || i2cx == NULL)
		return MPU_NULLPTR;

	mpu->i2cx = i2cx;

	// Reset the sensor ==========================================================
	uint8_t data = 0b10000000;
	MPU_WriteReg(mpu, MPU6500_PWR_MGMT_1, &data, 1);
	LL_mDelay(10);

	data = 0x00;
	MPU_WriteReg(mpu, MPU6500_PWR_MGMT_1, &data, 1);
	LL_mDelay(10);

	data = 0x01;
	MPU_WriteReg(mpu, MPU6500_PWR_MGMT_1, &data, 1);
	LL_mDelay(10);
	// ===========================================================================


	// Enable bypass mode and set up the external magnetometer ===================
//	MPU_WriteRegBit(mpu, MPU6500_INT_PIN_CFG, 1, MPU6500_INT_PIN_CFG_BYPASS_Msk);
//	LL_mDelay(10);
//
//
//	HMC_SetConfig(
//			mpu,
//			HMC5883_AVG_4,
//			HMC5883_DO_30,
//			HMC5883_NO_BIAS,
//			HMC5883_GAIN_3,
//			HMC5883_MODE_CONT
//	);

	//Disable bypass mode
//	MPU_WriteRegBit(mpu, MPU6500_INT_PIN_CFG, 0, MPU6500_INT_PIN_CFG_BYPASS_Msk);
//	LL_mDelay(10);
	// ===========================================================================

//	data = 0x20;
//	MPU_WriteReg(mpu, MPU6500_USER_CTRL, &data, 1);
//
//	data = HMC5883_ADDR | 0x80;
//	MPU_WriteReg(mpu, MPU6500_I2C_SLV0_ADDR, &data, 1);
//
//	data = HMC5883_DATA;
//	MPU_WriteReg(mpu, MPU6500_I2C_SLV0_REG, &data, 1);
//
//	data = 0x80 | HMC5883_DATA_Size;
//	MPU_WriteReg(mpu, MPU6500_I2C_SLV0_CTRL, &data, 1);

	return MPU_OK;
}

enum MPU_Status MPU_ReadReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return MPU_NULLPTR;
	if (pdata == NULL)
		return MPU_NULLPTR;

	i2c_start(mpu);
	i2c_send_addr(mpu, MPU6500_ADDR, MPU6500_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_start(mpu);
	i2c_send_addr(mpu, MPU6500_ADDR, MPU6500_READ);
	i2c_recv_data(mpu, pdata, count);
	i2c_stop(mpu);

	return MPU_OK;
}

enum MPU_Status MPU_WriteReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return MPU_NULLPTR;
	if (pdata == NULL)
		return MPU_NULLPTR;

	i2c_start(mpu);
	i2c_send_addr(mpu, MPU6500_ADDR, MPU6500_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_send_data(mpu, pdata, count);
	i2c_stop(mpu);

	return MPU_OK;
}

//
// The bits with the value of 0 remains untouched
enum MPU_Status MPU_WriteRegBit(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t data, uint8_t mask)
{
	if (mpu == NULL)
		return MPU_NULLPTR;

	// shift the given data to the correct position on the register
	uint8_t i = 0;
	while ((mask & (1 << i)) == 0) {
		i++;
	}
	data <<= i;

	// read the register and only change the masked bits
	uint8_t reg_data;
	if (MPU_ReadReg(mpu, reg_addr, &reg_data, 1) == MPU_OK) {
		data = (reg_data & (~mask)) | data;
		if (MPU_WriteReg(mpu, reg_addr, &data, 1) == MPU_OK) {
			return MPU_OK;
		}
	}

	return MPU_ERR;
}

enum MPU_Status MPU_SetAccelFS(struct MPU_Handle *mpu, uint8_t accel_fs)
{
	if (mpu == NULL)
		return MPU_NULLPTR;

	if (MPU_WriteRegBit(mpu, MPU6500_ACCEL_CONFIG, accel_fs, MPU6500_ACCEL_CONFIG_FS_SEL_Msk) == MPU_OK) {
		uint16_t fs = 2;
		for (uint8_t i = 0; i < accel_fs; i++) {
			fs <<= 1; // multiply with 2
		}
		mpu->a.fs = (float)fs;
		return MPU_OK;
	}

	return MPU_ERR;
}

enum MPU_Status MPU_SetGyroFS(struct MPU_Handle *mpu, uint8_t gyro_fs)
{
	if (mpu == NULL)
		return MPU_NULLPTR;

	if (MPU_WriteRegBit(mpu, MPU6500_GYRO_CONFIG, gyro_fs, MPU6500_GYRO_CONFIG_FS_SEL_Msk) == MPU_OK) {
		uint16_t fs = 250;
		for (uint8_t i = 0; i < gyro_fs; i++) {
			fs <<= 1; // multiply with 2
		}
		mpu->g.fs = (float)fs;
		return MPU_OK;
	}

	return MPU_ERR;
}

float MPU_Temp(struct MPU_Handle *mpu)
{
	uint8_t data[2];
	int16_t temperature;

	MPU_ReadReg(mpu, MPU6500_TEMP_OUT_H, data, 2);

	temperature = ((int16_t)data[0] << 8) | (int16_t)data[1];

	return mpu->t = (float)temperature/333.87f + 21.0f;
}

enum MPU_Status MPU_GetSensorData(struct MPU_Handle *mpu)
{
	uint8_t data[14];
	uint8_t start_reg = MPU6500_ACCEL_XOUT_H;

	// == Read accel, gyro, and temperature data ==
	MPU_ReadReg(mpu, start_reg, data, 14);
	mpu->a.xraw = ((int16_t)data[0] << 8) | (int16_t)data[1];
	mpu->a.yraw = ((int16_t)data[2] << 8) | (int16_t)data[3];
	mpu->a.zraw = ((int16_t)data[4] << 8) | (int16_t)data[5];

	mpu->t = (float)(((int16_t)data[6] << 8) | (int16_t)data[7]) / 333.87f + 21.0f;

	mpu->g.xraw = ((int16_t)data[8] << 8)  | (int16_t)data[9];
	mpu->g.yraw = ((int16_t)data[10] << 8) | (int16_t)data[11];
	mpu->g.zraw = ((int16_t)data[12] << 8) | (int16_t)data[13];

	// converting raw data into physical units
	mpu->a.x = ((float)mpu->a.xraw / 32768.0f) * mpu->a.fs;
	mpu->a.y = ((float)mpu->a.yraw / 32768.0f) * mpu->a.fs;
	mpu->a.z = ((float)mpu->a.zraw / 32768.0f) * mpu->a.fs;

	mpu->g.x = ((float)mpu->g.xraw / 32768.0f) * mpu->g.fs;
	mpu->g.y = ((float)mpu->g.yraw / 32768.0f) * mpu->g.fs;
	mpu->g.z = ((float)mpu->g.zraw / 32768.0f) * mpu->g.fs;

	// == Read magnetometer data ==
//	HMC_ReadReg(mpu, reg_addr, pdata, count)
//
//	printf("Accel X: %f\r\n", mpu->a.x);
//	printf("Accel Y: %f\r\n", mpu->a.y);
//	printf("Accel Z: %f\r\n", mpu->a.z);
//
//	printf("Gyro X: %f\r\n", mpu->g.x);
//	printf("Gyro Y: %f\r\n", mpu->g.y);
//	printf("Gyro Z: %f\r\n", mpu->g.z);
//
//	printf("Temperature: %f\r\n", mpu->t);

	return MPU_OK;
}

// ============= HMC5883L Magnetometer functions ====================================================
enum MPU_Status HMC_SetConfig(struct MPU_Handle *mpu, uint8_t avg, uint8_t drate, uint8_t bias, uint8_t gain, uint8_t mode)
{
	if (mpu == NULL)
		return MPU_NULLPTR;

	// Set CRA - averaging number, data rate, bias
	uint8_t data = (avg << 5) | (drate << 2) | bias;
	HMC_WriteReg(mpu, HMC5883_CONF_A, &data, 1);

	// Set CRB Register - gain value
	data = (gain << 5) & 0xE0;
	HMC_WriteReg(mpu, HMC5883_CONF_B, &data, 1);

	// Set Mode Register
	data = 0x02 & mode;
	HMC_WriteReg(mpu, HMC5883_MODE, &data, 1);

	// Save the gain value
	switch (gain) {
		case 0:
			mpu->m.fs = 0.88;
			break;

		case 1:
			mpu->m.fs = 1.3;
			break;

		case 2:
			mpu->m.fs = 1.9;
			break;

		case 3:
			mpu->m.fs = 2.5;
			break;

		case 4:
			mpu->m.fs = 4.0;
			break;

		case 5:
			mpu->m.fs = 4.7;
			break;

		case 6:
			mpu->m.fs = 5.6;
			break;

		case 7:
			mpu->m.fs = 8.1;
			break;

		default:
			mpu->m.fs = 9999;
			break;
	}

	return MPU_OK;
}

enum MPU_Status HMC_ReadReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return MPU_NULLPTR;
	if (pdata == NULL)
		return MPU_NULLPTR;

	i2c_start(mpu);
	i2c_send_addr(mpu, HMC5883_ADDR, HMC5883_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_start(mpu);
	i2c_send_addr(mpu, HMC5883_ADDR, HMC5883_READ);
	i2c_recv_data(mpu, pdata, count);
	i2c_stop(mpu);

	return MPU_OK;
}

enum MPU_Status HMC_WriteReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return MPU_NULLPTR;
	if (pdata == NULL)
		return MPU_NULLPTR;

	i2c_start(mpu);
	i2c_send_addr(mpu, HMC5883_ADDR, HMC5883_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_send_data(mpu, pdata, count);
	i2c_stop(mpu);

	return MPU_OK;

}
