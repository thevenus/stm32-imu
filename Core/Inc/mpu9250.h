/*
 * mpu9250.h
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"
#include "mpu9250_reg.h"

struct MPU_Data {
	int16_t xraw, yraw, zraw;
	uint16_t fs;
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
enum MPU_Status MPU_SetAccelFS(struct MPU_Handle *mpu, uint8_t accel_fs);
enum MPU_Status MPU_SetGyroFS(struct MPU_Handle *mpu, uint8_t gyro_fs);
enum MPU_Status MPU_GetSensorData(struct MPU_Handle *mpu);

enum MPU_Status MPU_AK8963_ReadReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU_Status MPU_AK8963_WriteReg(struct MPU_Handle *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);


float MPU_Temp(struct MPU_Handle *mpu);


#endif /* INC_MPU9250_H_ */
