/*
 * temperature_sensor.h
 *
 *  Created on: Apr 15, 2023
 *      Author: frank
 */

#ifndef INC_TEMPERATURE_SENSOR_H_
#define INC_TEMPERATURE_SENSOR_H_
#include "stm32f4xx_hal.h"

typedef unsigned char uint8_t;

#define TEMP_ADDR1 (0x3F << 1) // I2C address should shift one

class temperature_sensor
{
	/*
	 * 	product name: 2525020210001
	 *     	protocol: iic
	 *     datasheet: https://www.we-online.com/components/products/manual/2525020210001_WSEN-HIDS%202525020210001%20Manual_rev1.6.pdf
	 */
private:
	uint8_t addr;
	I2C_HandleTypeDef* hi2c_p;
	HAL_StatusTypeDef rv = HAL_OK;

public:
	temperature_sensor(I2C_HandleTypeDef* hi2c_pointer, uint8_t address);
	bool init();
	bool is_ready();
	void set_continuous();
	float get_temperature();
};

#endif /* INC_TEMPERATURE_SENSOR_H_ */
