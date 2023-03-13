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

struct MPU9250 {
	// Parameters
	I2C_TypeDef *i2cx;

	// Data
	struct MPU_Data a;
	struct MPU_Data g;
	struct MPU_Data m;
	float t;
};

enum MPU9250_Status {
	MPU9250_OK = 0,
	MPU9250_ERR,
	MPU9250_NULLPTR

};

enum MPU9250_Status MPU9250_Init(struct MPU9250*, I2C_TypeDef*);
enum MPU9250_Status MPU9250_ReadReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU9250_Status MPU9250_WriteReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU9250_Status MPU9250_SetAccelFS(struct MPU9250 *mpu, uint8_t accel_fs);
enum MPU9250_Status MPU9250_SetGyroFS(struct MPU9250 *mpu, uint8_t gyro_fs);
enum MPU9250_Status MPU9250_GetSensorData(struct MPU9250 *mpu);

enum MPU9250_Status MPU_AK8963_ReadReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU9250_Status MPU_AK8963_WriteReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);


float MPU9250_Temp(struct MPU9250 *mpu);


#endif /* INC_MPU9250_H_ */
