/*
 * mpu9250.h
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include <mpu6500_reg.h>
#include "main.h"

struct MPU_Data {
	int16_t xraw, yraw, zraw;
	float fs;
	float x, y, z;
};

struct MPU_Handle {
	// Parameters
	I2C_TypeDef *i2cx;

	// Data
	struct MPU_Data a;
	struct MPU_Data g;
	struct MPU_Data m;
	float t;
};

enum MPU_Status {
	MPU_OK = 0,
	MPU_ERR,
	MPU_NULLPTR

};

enum MPU_Status MPU_Init(struct MPU_Handle*, I2C_TypeDef*);
enum MPU_Status MPU_ReadReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU_Status MPU_WriteReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU_Status MPU_WriteRegBit(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t data, uint8_t mask);
enum MPU_Status MPU_SetAccelFS(struct MPU_Handle *mpu, uint8_t accel_fs);
enum MPU_Status MPU_SetGyroFS(struct MPU_Handle *mpu, uint8_t gyro_fs);
float MPU_Temp(struct MPU_Handle *mpu);
enum MPU_Status MPU_GetSensorData(struct MPU_Handle *mpu);


// HMC5883L Magnetometer functions
enum MPU_Status HMC_SetConfig(struct MPU_Handle *mpu, uint8_t avg, uint8_t drate, uint8_t bias, uint8_t gain, uint8_t mode);
enum MPU_Status HMC_ReadReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU_Status HMC_WriteReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);

// set CRA +
// set sample averaging
// set data rate
// set measurement bias

// set CRB +
// set gain

// set Mode




#endif /* INC_MPU6500_H_ */
