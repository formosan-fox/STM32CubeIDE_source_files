#include "Pressure_WSEN_PADS.h"
#include "util.h"

// TODO: map

static const uint32_t I2C_TIMEOUT = 100;

// ***** public funcitions *****


Pressure_WSEN_PADS::Pressure_WSEN_PADS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO): i2c_handler(_i2c_handler)
{
    i2c_device_addr = (SAO == 0) ? PADS_ADDRESS_I2C_0 : PADS_ADDRESS_I2C_1;
}

bool Pressure_WSEN_PADS::init(
    PADS_outputDataRate_t outputDataRate,
    PADS_state_t blockDataUpdate)
{
	PADS_state_t swReset;
   	PADS_outputDataRate_t odr;


    // TODO: Initialize sensor interface.


    // Wait for boot
	HAL_Delay(50);

    // TODO: Check Sensor Interface Ready

    // Communication test.
	if (!isCommunicationReady()) {
		return false;
	}

    debug_print("WSEN PADS: Initializes.\r\n");

    // Soft reset for setting the internal registers to the default values. (example)
    softReset();
    do
    {
    	getSoftResetState(&swReset);
    } while (swReset);
    debug_print("WSEN PADS: Soft reset completes.\r\n");


    // Automatic increment register address (default: enabled)
	enableAutoIncrement();
	if (isAutoIncrementEnabled()) {
		debug_print("WSEN PADS: Auto increment is enabled.\r\n");
	} else {
        debug_print("<Error> WSEN PADS: Failed to enable auto increment.\r\n");
    }

    // Enable block data update
    if (blockDataUpdate) {
        enableBlockDataUpdate();
        if (isBlockDataUpdateEnabled()) {
            debug_print("WSEN PADS: Block data update is enabled.\r\n");
        } else {
            debug_print("<Error> WSEN PADS: Failed to enable block data update.\r\n");
        }
    }

    setOutputDataRate(outputDataRate);
	getOutputDataRate(&odr);
	if (odr == outputDataRate) {          
		debug_print("WSEN PADS: Set output data\r\n");  // TODO: prine message
    } else {
        debug_print("<Error> WSEN PADS: Failed to set the output data rate.\r\n");
    }
	// TODO: Enable Low-noise configuration
	// TODO: Enable additional low pass filter

    return true;
}

bool Pressure_WSEN_PADS::isCommunicationReady()
{
    uint8_t device_ID = 0;

    getDeviceID(&device_ID);
	if (device_ID == PADS_DEVICE_ID_VALUE) {
		debug_print<uint8_t>("WSEN PADS: device ID = 0x%x. Communication is successful.\r\n", device_ID);
		return true;
	} else {
		debug_print<uint8_t>("WSEN PADS: device ID = 0x%x. Communication is failed.\r\n", device_ID);
		return false;
	}
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getDeviceID(uint8_t *device_ID)
{
    return PADS_ReadReg(PADS_DEVICE_ID_REG, 1, device_ID);
}

void Pressure_WSEN_PADS::singleConversionModeExample()
{
	// TODO: Fix the bug of status register not accessible.
	/*If the sensor is configured in the single conversion mode, the STATUS register
	will not be updated after the first measurement because the sensor goes to
	power down mode after the first acquisition.
	 * */

    float pressure = 0;
    float temperature = 0;
    int n;

    debug_print("WSEN PADS: Run the single conversion mode example.\r\n");
    while (1) {
        // Start a measurement (one shot). */
        n = 0;
        enableOneShot();
        HAL_Delay(10);

        if (isOneShotEnabled()) {
        	debug_print("WSEN PADS: One shot is enabled.\r\n");
        } else {
        	debug_print("<Error> WSEN PADS: One shot is not enabled.\r\n");
        }

		while (!isPressureDataReady()) {
			HAL_Delay(100);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN PADS: Pressure data not available.\r\n");
				break;
			}
		};

        if (HAL_OK != getPressure_float(&pressure)) {
            debug_print("<Error> WSEN PADS: Read pressure data error.\r\n");
        }

		n = 0;
		while (!isTemperatureDataReady()) {
			HAL_Delay(100);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN PADS: temperature data not available.\r\n");
				break;
			}
		};
        if (HAL_OK != getTemperature_float(&temperature)) {
            debug_print("<Error> WSEN PADS: Read temperature data error.\r\n");
        }

		debug_print<float>("Pressure = %.3f kPa   Temperature = %.2f degC\r\n", pressure, temperature);
    }
}

void Pressure_WSEN_PADS::Pressure_WSEN_PADS::continuousModeExampleLoop()
{
	float pressure = 0;
	float temperature = 0;
	int n;

	while (1) {
		n = 0;
		while (isPressureDataReady() == false) {
			HAL_Delay(10);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN PADS: Pressure data not available.\r\n");
				break;
			}
		};
        getPressure_float(&pressure);

		n = 0;
		while (!isTemperatureDataReady()) {
			HAL_Delay(10);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN PADS: temperature data not available.\r\n");
				break;
			}
		};
        getTemperature_float(&temperature);

		debug_print<float>("Pressure = %.3f kPa   Temperature = %.2f degC\r\n", pressure, temperature);

		HAL_Delay(1000);
	}
}

HAL_StatusTypeDef Pressure_WSEN_PADS::softReset()
{
	PADS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	ctrl2.softwareReset = PADS_enable;
	return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getSoftResetState(PADS_state_t *swReset)
{
	PADS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	*swReset = (PADS_state_t) ctrl2.softwareReset;
	return status;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::enableAutoIncrement()
{
	PADS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	ctrl2.autoAddIncr = PADS_enable;
	return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::disableAutoIncrement()
{
	PADS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	ctrl2.autoAddIncr = PADS_disable;
	return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::isAutoIncrementEnabled(PADS_state_t *ai)
{
	PADS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	*ai = (PADS_state_t)ctrl2.autoAddIncr;
	return status;
}

bool Pressure_WSEN_PADS::isAutoIncrementEnabled()
{
	PADS_ctrl2_t ctrl2;

	PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	return ((PADS_state_t)ctrl2.autoAddIncr) == PADS_enable;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::enableBlockDataUpdate()
{
    PADS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK) {
		return status;
	}
    ctrl1.blockDataUpdate = PADS_enable;
    return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::disableBlockDataUpdate()
{
    PADS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK) {
		return status;
	}
    ctrl1.blockDataUpdate = PADS_disable;
    return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::isBlockDataUpdateEnabled(PADS_state_t *bdu)
{
    PADS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK) {
		return status;
	}
    *bdu = (PADS_state_t) ctrl1.blockDataUpdate;
    return status;
}

bool Pressure_WSEN_PADS::isBlockDataUpdateEnabled()
{
    PADS_ctrl1_t ctrl1;

    PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    return ((PADS_state_t) ctrl1.blockDataUpdate == PADS_enable);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::setOutputDataRate(PADS_outputDataRate_t odr)
{
	PADS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK) {
		return status;
	}
	ctrl1.outputDataRate = odr;
	return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getOutputDataRate(PADS_outputDataRate_t* odr)
{
	PADS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status;

	status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK) {
		return status;
	}
	*odr = (PADS_outputDataRate_t) ctrl1.outputDataRate;
	return status;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::enableOneShot()
{
    PADS_ctrl2_t ctrl2;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
    ctrl2.oneShotBit = PADS_enable;
    return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::disableOneShot()
{
    PADS_ctrl2_t ctrl2;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
    ctrl2.oneShotBit = PADS_disable;
    return PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::isOneShotEnabled(PADS_state_t *oneShot)
{
    PADS_ctrl2_t ctrl2;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
    *oneShot = (PADS_state_t)ctrl2.oneShotBit;
    return status;
}

bool Pressure_WSEN_PADS::isOneShotEnabled()
{
    PADS_ctrl2_t ctrl2;

    PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl2);
    return ((PADS_state_t)ctrl2.oneShotBit == PADS_enable);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::isPressureDataReady(PADS_state_t *state)
{
    PADS_status_t statusReg;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg);
    if (status != HAL_OK) {
		return status;
	}
    *state = (PADS_state_t) statusReg.presDataAvailable;
    return status;
}

bool Pressure_WSEN_PADS::isPressureDataReady()
{
    PADS_status_t statusReg;

    PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg);
    return (((PADS_state_t) statusReg.presDataAvailable) == PADS_enable);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::isTemperatureDataReady(PADS_state_t *state)
{
    PADS_status_t statusReg;
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg);
    if (status != HAL_OK) {
		return status;
	}
    *state = (PADS_state_t) statusReg.tempDataAvailable;
    return status;
}

bool Pressure_WSEN_PADS::isTemperatureDataReady()
{
    PADS_status_t statusReg;

    PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg);
    return (((PADS_state_t) statusReg.tempDataAvailable) == PADS_enable);
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getRawPressure(int32_t *rawPres)
{
    uint8_t tmp[3] = {0};
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_DATA_P_XL_REG, 3, tmp);
    if (status != HAL_OK) {
		return status;
	}
    *rawPres = (int32_t) (tmp[2] << 24);
    *rawPres |= (int32_t) (tmp[1] << 16);
    *rawPres |= (int32_t) (tmp[0] << 8);
    *rawPres /= 256;
    return status;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getRawTemperature(int16_t *rawTemp)
{
    uint8_t tmp[2] = {0};
    HAL_StatusTypeDef status;

    status = PADS_ReadReg(PADS_DATA_T_L_REG, 2, tmp);
    if (status != HAL_OK) {
		return status;
	}
    *rawTemp = (int16_t) (tmp[1] << 8);
    *rawTemp |= (int16_t) tmp[0];
    return status;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getPressure_float(float *presskPa)
{
    int32_t rawPressure = 0;
    HAL_StatusTypeDef status;

    status = getRawPressure(&rawPressure);
    if (status != HAL_OK) {
		return status;
	}
    *presskPa = convertPressure_float(rawPressure);
    return status;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getDifferentialPressure_float(float *presskPa)
{
    int32_t rawPressure = 0;
    HAL_StatusTypeDef status;

    status = getRawPressure(&rawPressure);
    if (status != HAL_OK) {
		return status;
	}
    *presskPa = convertDifferentialPressure_float(rawPressure);
    return status;
}

HAL_StatusTypeDef Pressure_WSEN_PADS::getTemperature_float(float *tempDegC)
{
    int16_t rawTemp = 0;
    HAL_StatusTypeDef status;

    status = getRawTemperature(&rawTemp);
    if (status != HAL_OK) {
		return status;
	}
    *tempDegC = (float) rawTemp;
    *tempDegC = *tempDegC / 100;
    return status;
}


// ***** private funcitions *****

float Pressure_WSEN_PADS::convertPressure_float(const int32_t& rawPres)
{
    return ((float) rawPres) / 40960;
}


float Pressure_WSEN_PADS::convertDifferentialPressure_float(const int32_t &rawPres)
{
    return ((float) rawPres) * 0.00625f;
}

int32_t Pressure_WSEN_PADS::convertPressure_int(const int32_t &rawPres)
{
    return (rawPres * 100) / 4096;
}

int32_t Pressure_WSEN_PADS::convertDifferentialPressure_int(const int32_t &rawPres)
{
    return (rawPres * 25600) / 4096;
}

inline HAL_StatusTypeDef Pressure_WSEN_PADS::PADS_ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf)
{
    // HAL_I2C_Mem_Read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
    return HAL_I2C_Mem_Read(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, buf, numBytesToRead, I2C_TIMEOUT);
}

inline HAL_StatusTypeDef Pressure_WSEN_PADS::PADS_WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data)
{
    // HAL_I2C_Mem_Write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
    return HAL_I2C_Mem_Write(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, data, numBytesToWrite, I2C_TIMEOUT);
}
