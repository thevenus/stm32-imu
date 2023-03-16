/*
 * mpu6500_reg.h
 *
 *  Created on: Mar 13, 2023
 *      Author: fuadzade
 */

#ifndef INC_MPU6500_REG_H_
#define INC_MPU6500_REG_H_

// Parameters
#define MPU6500_ADDR			(0x68)
#define MPU6500_READ			(0x01)
#define MPU6500_WRITE			(0x00)

#define MPU6500_ACCEL_FS_2G 		(0x00)
#define MPU6500_ACCEL_FS_4G 		(0x01)
#define MPU6500_ACCEL_FS_8G 		(0x10)
#define MPU6500_ACCEL_FS_16G 		(0x11)

#define MPU6500_GYRO_FS_250DPS		(0x00)
#define MPU6500_GYRO_FS_500DPS		(0x01)
#define MPU6500_GYRO_FS_1000DPS		(0x10)
#define MPU6500_GYRO_FS_2000DPS		(0x11)

// MPU6500 Registers
#define MPU6500_CONFIG			(0x1A)
#define MPU6500_GYRO_CONFIG		(0x1B)
#define MPU6500_GYRO_CONFIG_FS_SEL_Msk	(0x18)
#define MPU6500_ACCEL_CONFIG		(0x1C)
#define MPU6500_ACCEL_CONFIG_FS_SEL_Msk	(0x18)
#define MPU6500_ACCEL_CONFIG_2		(0x1D)

#define MPU6500_I2C_MST_CTRL		(0x24)
#define MPU6500_I2C_SLV0_ADDR		(0x25)
#define MPU6500_I2C_SLV0_REG		(0x26)
#define MPU6500_I2C_SLV0_CTRL		(0x27)

#define MPU6500_INT_PIN_CFG		(0x37)
#define MPU6500_INT_PIN_CFG_BYPASS_Msk	(0x02)

#define MPU6500_ACCEL_XOUT_H 		(0x3B)
#define MPU6500_ACCEL_XOUT_L 		(0x3C)
#define MPU6500_ACCEL_YOUT_H 		(0x3D)
#define MPU6500_ACCEL_YOUT_L 		(0x3E)
#define MPU6500_ACCEL_ZOUT_H 		(0x3F)
#define MPU6500_ACCEL_ZOUT_L 		(0x40)
#define MPU6500_TEMP_OUT_H		(0x41)
#define MPU6500_TEMP_OUT_L		(0x42)
#define MPU6500_GYRO_XOUT_H 		(0x43)
#define MPU6500_GYRO_XOUT_L 		(0x44)
#define MPU6500_GYRO_YOUT_H 		(0x45)
#define MPU6500_GYRO_YOUT_L 		(0x46)
#define MPU6500_GYRO_ZOUT_H 		(0x47)
#define MPU6500_GYRO_ZOUT_L 		(0x48)

#define MPU6500_USER_CTRL		(0x6A)
#define MPU6500_PWR_MGMT_1		(0x6B)
#define MPU6500_PWR_MGMT_2		(0x6C)

#define MPU6500_WHO_AM_I 		(0x75)

// HMC5883L Parameters and Registers
#define HMC5883_ADDR			(0x1E)
#define HMC5883_READ			(0x01)
#define HMC5883_WRITE			(0x00)

#define HMC5883_AVG_1			(0x00)
#define HMC5883_AVG_2			(0x01)
#define HMC5883_AVG_4			(0x02)
#define HMC5883_AVG_8			(0x03)

#define HMC5883_DO_0_75			(0x00)
#define HMC5883_DO_1_5			(0x01)
#define HMC5883_DO_3			(0x02)
#define HMC5883_DO_7_5			(0x03)
#define HMC5883_DO_15			(0x04)
#define HMC5883_DO_30			(0x05)
#define HMC5883_DO_75			(0x06)

#define HMC5883_NO_BIAS			(0x00)
#define HMC5883_POS_BIAS		(0x01)
#define HMC5883_NEG_BIAS		(0x02)

#define HMC5883_GAIN_0			(0x00)
#define HMC5883_GAIN_1			(0x01)
#define HMC5883_GAIN_2			(0x02)
#define HMC5883_GAIN_3			(0x03)
#define HMC5883_GAIN_4			(0x04)
#define HMC5883_GAIN_5			(0x05)
#define HMC5883_GAIN_6			(0x06)
#define HMC5883_GAIN_7			(0x07)

#define HMC5883_MODE_CONT		(0x00)
#define HMC5883_MODE_SINGLE		(0x01)
#define HMC5883_MODE_IDLE		(0x02)

#define HMC5883_CONF_A			(0x00) // R/W
#define HMC5883_CONF_B			(0x01) // R/W
#define HMC5883_MODE			(0x02) // R/W

#define HMC5883_DATA			(0x03) // R
#define HMC5883_DATA_Size		(0x06) // R

#define HMC5883_STATUS			(0x09) // R
#define HMC5883_ID_A			(0x0A) // R
#define HMC5883_ID_B			(0x0B) // R
#define HMC5883_ID_C			(0x0C) // R

#endif /* INC_MPU6500_REG_H_ */
