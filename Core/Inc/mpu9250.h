/*
 * mpu9250.h
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"

// Parameters
#define MPU9250_ADDR		(0x68)
#define MPU9250_READ		(0x01)
#define MPU9250_WRITE		(0x00)

#define MPU9250_ACCEL_FS_2G 	(0x00)
#define MPU9250_ACCEL_FS_4G 	(0x01)
#define MPU9250_ACCEL_FS_8G 	(0x10)
#define MPU9250_ACCEL_FS_16G 	(0x11)

#define MPU9250_GYRO_FS_250DPS	(0x00)
#define MPU9250_GYRO_FS_500DPS	(0x01)
#define MPU9250_GYRO_FS_1000DPS	(0x10)
#define MPU9250_GYRO_FS_2000DPS	(0x11)

// Registers
#define CONFIG			(0x1A)
#define GYRO_CONFIG		(0x1B)
#define GYRO_CONFIG_FS_SEL_Bit	(0x03)
#define ACCEL_CONFIG		(0x1C)
#define ACCEL_CONFIG_FS_SEL_Bit	(0x03)
#define ACCEL_CONFIG_2		(0x1D)
#define ACCEL_XOUT_H 		(0x3B)
#define ACCEL_XOUT_L 		(0x3C)
#define ACCEL_YOUT_H 		(0x3D)
#define ACCEL_YOUT_L 		(0x3E)
#define ACCEL_ZOUT_H 		(0x3F)
#define ACCEL_ZOUT_L 		(0x40)
#define TEMP_OUT_H		(0x41)
#define TEMP_OUT_L		(0x42)
#define GYRO_XOUT_H 		(0x43)
#define GYRO_XOUT_L 		(0x44)
#define GYRO_YOUT_H 		(0x45)
#define GYRO_YOUT_L 		(0x46)
#define GYRO_ZOUT_H 		(0x47)
#define GYRO_ZOUT_L 		(0x48)
#define PWR_MGMT_1		(0x6B)
#define PWR_MGMT_2		(0x6C)
#define WHO_AM_I 		(0x75)

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

float MPU9250_Temp(struct MPU9250 *mpu);


#endif /* INC_MPU9250_H_ */
