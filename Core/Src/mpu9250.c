/*
 * mpu9250.c
 *
 *  Created on: Mar 3, 2023
 *      Author: fuadzade
 */

#include "mpu9250.h"

static void i2c_start(struct MPU9250 *mpu)
{
	if (mpu == NULL)
		return;

	LL_I2C_GenerateStartCondition(mpu->i2cx);
	while (!LL_I2C_IsActiveFlag_SB(mpu->i2cx));
}

static void i2c_stop(struct MPU9250 *mpu)
{
	LL_I2C_GenerateStopCondition(mpu->i2cx);
}

// rw = 0 -> write operation
// rw = 1 -> read operation
static void i2c_send_addr(struct MPU9250 *mpu, uint8_t address, uint8_t rw)
{
	if (mpu == NULL)
		return;

	LL_I2C_TransmitData8(mpu->i2cx, (address << 1)+rw);
	while (!LL_I2C_IsActiveFlag_ADDR(mpu->i2cx));
	LL_I2C_ClearFlag_ADDR(mpu->i2cx);
}

static void i2c_send_data(struct MPU9250 *mpu, uint8_t *pdata, uint8_t count)
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

static void i2c_recv_data(struct MPU9250 *mpu, uint8_t *pdata, uint8_t count)
{
	if (mpu == NULL)
		return;
	if (pdata == NULL)
		return;

	for (uint8_t i=0; i< count; i++) {
		while (!LL_I2C_IsActiveFlag_RXNE(mpu->i2cx));
		pdata[i] = LL_I2C_ReceiveData8(mpu->i2cx);
	}
}


static void i2c_ack(struct MPU9250 *mpu)
{
	LL_I2C_AcknowledgeNextData(mpu->i2cx, LL_I2C_ACK);

}

static void i2c_nack(struct MPU9250 *mpu)
{
	LL_I2C_AcknowledgeNextData(mpu->i2cx, LL_I2C_NACK);

}

enum MPU9250_Status MPU9250_Init(struct MPU9250 *mpu, I2C_TypeDef *i2cx, GPIO_TypeDef *ad0_gpio_port, uint16_t ad0_gpio_pin)
{
	if (mpu == NULL || i2cx == NULL || ad0_gpio_port == NULL)
		return MPU9250_NULLPTR;

	mpu->i2cx = i2cx;
	mpu->ad0_gpio_port = ad0_gpio_port;
	mpu->ad0_gpio_pin = ad0_gpio_pin;

	ad0_gpio_port->BSRR = (uint32_t) (1 << (ad0_gpio_pin + 16));

	return MPU9250_OK;
}

enum MPU9250_Status MPU9250_ReadReg(struct MPU9250 *mpu, uint8_t reg_addr, uint8_t *pdata, uint8_t count)
{
	i2c_start(mpu);
	i2c_send_addr(mpu, MPU9250_ADDR, MPU9250_WRITE);
	i2c_send_data(mpu, &reg_addr, 1);
	i2c_start(mpu);
	i2c_send_addr(mpu, MPU9250_ADDR, MPU9250_READ);
	i2c_recv_data(mpu, pdata, count);
	i2c_nack(mpu);
	i2c_stop(mpu);

	return MPU9250_OK;
}

float MPU9250_Temp(struct MPU9250 *mpu)
{
	uint8_t data[2];
	int16_t temperature;

	MPU9250_ReadReg(mpu, TEMP_OUT_H, &data[1], 1);
	MPU9250_ReadReg(mpu, TEMP_OUT_L, &data[0], 1);

	temperature = ((int16_t)data[1] << 8) | (int16_t)data[0];

	return (float)temperature/333.87f + 21.0f;
}

