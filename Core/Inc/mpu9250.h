/*
 * mpu9250.h
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"

#define MPU9250_ADDR	(0x68)
#define MPU9250_READ	(0x01)
#define MPU9250_WRITE	(0x00)

#define ACCEL_XOUT_H 	(0x3B)
#define ACCEL_XOUT_L 	(0x3C)
#define ACCEL_YOUT_H 	(0x3D)
#define ACCEL_YOUT_L 	(0x3E)
#define ACCEL_ZOUT_H 	(0x3F)
#define ACCEL_ZOUT_L 	(0x40)
#define TEMP_OUT_H	(0x41)
#define TEMP_OUT_L	(0x42)
#define GYRO_XOUT_H 	(0x43)
#define GYRO_XOUT_L 	(0x44)
#define GYRO_YOUT_H 	(0x45)
#define GYRO_YOUT_L 	(0x46)
#define GYRO_ZOUT_H 	(0x47)
#define GYRO_ZOUT_L 	(0x48)
#define WHO_AM_I 	(0x75)

typedef struct MPU9250 {
	I2C_TypeDef *i2cx;
	GPIO_TypeDef *ad0_gpio_port;
	uint16_t ad0_gpio_pin;
} MPU9250_t;

enum MPU9250_Status {
	MPU9250_OK = 0,
	MPU9250_ERR,
	MPU9250_NULLPTR

};

enum MPU9250_Status MPU9250_Init(struct MPU9250*, I2C_TypeDef*, GPIO_TypeDef*, uint16_t);
enum MPU9250_Status MPU9250_ReadReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
enum MPU9250_Status MPU9250_WriteReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count);
float MPU9250_Temp(struct MPU9250 *mpu);

#endif /* INC_MPU9250_H_ */
