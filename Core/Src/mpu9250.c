/*
 * mpu9250.c
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#include "mpu9250.h"
#include "stdio.h"

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

enum MPU_Status MPU_Init(struct MPU_Handle *mpu, I2C_TypeDef *i2cx)
{
	if (mpu == NULL || i2cx == NULL)
		return MPU_NULLPTR;

	mpu->i2cx = i2cx;

	return MPU_OK;
}

enum MPU_Status MPU_ReadReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return MPU_NULLPTR;
	if (pdata == NULL)
		return MPU_NULLPTR;

	i2c_start(mpu);
	i2c_send_addr(mpu, MPU9250_ADDR, MPU9250_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_start(mpu);
	i2c_send_addr(mpu, MPU9250_ADDR, MPU9250_READ);
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
	i2c_send_addr(mpu, MPU9250_ADDR, MPU9250_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_send_data(mpu, pdata, count);
	i2c_stop(mpu);

	return MPU_OK;
}

enum MPU_Status MPU_SetAccelFS(struct MPU_Handle *mpu, uint8_t accel_fs)
{
	if (mpu == NULL)
		return MPU_NULLPTR;

	uint8_t data;

	// shift the given value to match with the actual register bit position
	uint8_t afs_shifted = accel_fs << MPU9250_ACCEL_CONFIG_FS_SEL_Bit;
	if (MPU_ReadReg(mpu, MPU9250_ACCEL_CONFIG, &data, 1) == MPU_OK) {
		data = (data & 0xe7) | afs_shifted;
		if (MPU_WriteReg(mpu, MPU9250_ACCEL_CONFIG, &data, 1) == MPU_OK) {
			// calculate full-scale value and write it into the MPU struct
			uint16_t fs = 2;
			for (uint8_t i = 0; i < accel_fs; i++) {
				fs <<= 1; // multiply with 2
			}
			mpu->a.fs = fs;
			return MPU_OK;
		}
	}

	return MPU_ERR;
}

enum MPU_Status MPU_SetGyroFS(struct MPU_Handle *mpu, uint8_t gyro_fs)
{
	if (mpu == NULL)
		return MPU_NULLPTR;

	uint8_t data;

	// shift the given value to match with the actual register bit position
	uint8_t gfs_shifted = gyro_fs << MPU9250_GYRO_CONFIG_FS_SEL_Bit;
	if (MPU_ReadReg(mpu, MPU9250_GYRO_CONFIG, &data, 1) == MPU_OK) {
		data = (data & 0xe7) | gfs_shifted;
		if (MPU_WriteReg(mpu, MPU9250_GYRO_CONFIG, &data, 1) == MPU_OK) {
			// calculate full-scale value and write it into the MPU struct
			uint16_t fs = 250;
			for (uint8_t i = 0; i < gyro_fs; i++) {
				fs <<= 1; // multiply with 2
			}
			mpu->g.fs = fs;
			return MPU_OK;
		}
	}

	return MPU_ERR;
}

enum MPU_Status MPU_GetSensorData(struct MPU_Handle *mpu)
{
	uint8_t data[14];
	uint8_t start_reg = MPU9250_ACCEL_XOUT_H;

	MPU_ReadReg(mpu, start_reg, data, 14);

	mpu->a.xraw = ((int16_t)data[0] << 8) | (int16_t)data[1];
	mpu->a.yraw = ((int16_t)data[2] << 8) | (int16_t)data[3];
	mpu->a.zraw = ((int16_t)data[4] << 8) | (int16_t)data[5];

	mpu->t = (float)(((int16_t)data[6] << 8) | (int16_t)data[7]) / 333.87f + 21.0f;

	mpu->g.xraw = ((int16_t)data[8] << 8)  | (int16_t)data[9];
	mpu->g.yraw = ((int16_t)data[10] << 8) | (int16_t)data[11];
	mpu->g.zraw = ((int16_t)data[12] << 8) | (int16_t)data[13];

	// converting raw data into physical units
	mpu->a.x = ((float)mpu->a.xraw / 32768.0f) * (float)mpu->a.fs;
	mpu->a.y = ((float)mpu->a.yraw / 32768.0f) * (float)mpu->a.fs;
	mpu->a.z = ((float)mpu->a.zraw / 32768.0f) * (float)mpu->a.fs;

	mpu->g.x = ((float)mpu->g.xraw / 32768.0f) * (float)mpu->g.fs;
	mpu->g.y = ((float)mpu->g.yraw / 32768.0f) * (float)mpu->g.fs;
	mpu->g.z = ((float)mpu->g.zraw / 32768.0f) * (float)mpu->g.fs;

	printf("Accel X: %f\r\n", mpu->a.x);
	printf("Accel Y: %f\r\n", mpu->a.y);
	printf("Accel Z: %f\r\n", mpu->a.z);

	printf("Gyro X raw: %f\r\n", mpu->g.x);
	printf("Gyro Y raw: %f\r\n", mpu->g.y);
	printf("Gyro Z raw: %f\r\n", mpu->g.z);

	printf("Temperature: %f\r\n", mpu->t);

	return MPU_OK;
}

float MPU_Temp(struct MPU_Handle *mpu)
{
	uint8_t data[2];
	int16_t temperature;

	MPU_ReadReg(mpu, MPU9250_TEMP_OUT_H, data, 2);

	temperature = ((int16_t)data[0] << 8) | (int16_t)data[1];

	return mpu->t = (float)temperature/333.87f + 21.0f;
}

