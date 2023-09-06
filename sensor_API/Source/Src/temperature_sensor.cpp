/*
 * temperature_sensor.cpp
 *
 *  Created on: Apr 15, 2023
 *      Author: frank
 *        Date: Apr 15, 2023
 */

#include "temperature_sensor.h"

//extern I2C_HandleTypeDef hi2c1;

/*
 * function name: temperature constructor
 * input:
 * 		* address: uint8_t, the i2c sensor of temperature sensor
 * output:
 * 		* none
 * description:
 * 		set the address of the temperature sensor
 */
temperature_sensor::temperature_sensor(I2C_HandleTypeDef* hi2c_pointer, uint8_t address)
{
	this->hi2c_p = hi2c_pointer;
	this->addr = address;
}

bool temperature_sensor::init()
{
	uint8_t txbuff = 0x02;
	rv = HAL_I2C_Mem_Write(hi2c_p, addr, 0x0C, I2C_MEMADD_SIZE_8BIT, &txbuff, 1, 1);
	txbuff = 0x00;
	rv = HAL_I2C_Mem_Write(hi2c_p, addr, 0x0C, I2C_MEMADD_SIZE_8BIT, &txbuff, 1, 1);
	return rv == HAL_OK;
}

bool temperature_sensor::is_ready()
{
	rv = HAL_I2C_IsDeviceReady(hi2c_p, addr, 1, 10);
	return rv == HAL_OK;
}

void temperature_sensor::set_continuous()
{
	uint8_t txbuff = 0x3C;
	rv = HAL_I2C_Mem_Write(hi2c_p, addr, 0x04, I2C_MEMADD_SIZE_8BIT, &txbuff, 1, 10);
}

float temperature_sensor::get_temperature()
{
	uint8_t rxbuff[2];
	int temp;
	rv = HAL_I2C_Mem_Read(hi2c_p, addr, 0x06, I2C_MEMADD_SIZE_8BIT, rxbuff, 2, 10);
	temp = rxbuff[1]<<8 | rxbuff[0];
	return 0.01 * temp;
}
