#include "IMU_3DIM_WSEN_ITDS.h"
#include "util.h"

// TODO: map

static const uint32_t I2C_TIMEOUT = 100;

// ***** public funcitions *****

IMU_3DIM_WSEN_ITDS::IMU_3DIM_WSEN_ITDS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO): i2c_handler(_i2c_handler)
{
    i2c_device_addr = (SAO == 0) ? ITDS_ADDRESS_I2C_0 : ITDS_ADDRESS_I2C_1;
}

bool IMU_3DIM_WSEN_ITDS::init(
    ITDS_operatingMode_t operatingMode,
    ITDS_powerMode_t powerMode,
    ITDS_outputDataRate_t outputDataRate,
    ITDS_fullScale_t fullScale,
    ITDS_state_t blockDataUpdate)
{
    ITDS_operatingMode_t opMode;
    ITDS_powerMode_t powMode;
    ITDS_outputDataRate_t odr;
    ITDS_fullScale_t fs;

	// ITDS_state_t swReset;

    // TODO: Initialize sensor interface.


    // Wait for boot
	HAL_Delay(50);

    // TODO: Check Sensor Interface Ready

    // Communication test.
	if (!isCommunicationReady()) {
		return false;
	}

    debug_print("WSEN_ITDS: Initializes.\r\n");

    // Automatic increment register address (default: enabled)
	enableAutoIncrement();
	if (isAutoIncrementEnabled()) {
		debug_print("WSEN ITDS: Auto increment is enabled.\r\n");
	}

    // Enable block data update
    if (blockDataUpdate == ITDS_enable) {
        enableBlockDataUpdate();
        if (isBlockDataUpdateEnabled()) {
            debug_print("WSEN ITDS: Block data update is enabled.\r\n");
        } else {
            debug_print("<Error> WSEN ITDS: Failed to enable block data update.\r\n");
        }
    }

    // Set operating mode
    setOperatingMode(operatingMode);
    getOperatingMode(&opMode);
    if (opMode == operatingMode) {
    	debug_print("WSEN ITDS: Set the operating mode to high performance mode.\r\n");     
    } else {
        debug_print("<Error> WSEN ITDS: The operating mode is not set.\r\n");
    }

    // Set power power
    setPowerMode(powerMode);
    getPowerMode(&powMode);
    if (powMode == powerMode) {
    	debug_print("WSEN ITDS: Set the power mode to normal mode.\r\n");
    } else {
        debug_print("<Error> WSEN ITDS: The power mode is not set.\r\n");
    }

    // Set the sampling rate
    setOutputDataRate(outputDataRate);
    getOutputDataRate(&odr);
    if (odr == outputDataRate) {
        debug_print("WSEN ITDS: Output data rate is set.\r\n");     // TODO: print message
    } else {
        debug_print("<Error> WSEN ITDS: Output data rate is not set.\r\n");
    }

    // Set the full scale
    setFullScale(fullScale);
    getFullScale(&fs);
    if (fs == fullScale) {
        debug_print("WSEN ITDS: Full scale is set.\r\n");           // TODO: print message
    } else {
        debug_print("<Error> WSEN ITDS: Full scale is not set.\r\n");
    }

    return true;
}

bool IMU_3DIM_WSEN_ITDS::isCommunicationReady()
{
    uint8_t device_ID = 0;

    getDeviceID(&device_ID);
	if (device_ID == ITDS_DEVICE_ID_VALUE) {
		debug_print<uint8_t>("WSEN ITDS: device ID = 0x%x. Communication is successful.\r\n", device_ID);
		return true;
	} else {
		debug_print<uint8_t>("WSEN ITDS: device ID = 0x%x. Communication is failed.\r\n", device_ID);
		return false;
	}
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getDeviceID(uint8_t *device_ID)
{
    return ITDS_ReadReg(ITDS_DEVICE_ID_REG, 1, device_ID);
}
void IMU_3DIM_WSEN_ITDS::exampleLoop()
{
    ITDS_state_t dataReady = ITDS_disable;
    float acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;
    int n;

    while (1) {
        n = 0;
        do {
            isAccelerationDataReady(&dataReady);
            if (dataReady == ITDS_disable)
                n++;
            if (n > 10) {
				debug_print("<Error> WSEN ITDS: Acceleration data not available.\r\n");
				break;
			}
            HAL_Delay(10);
        } while (dataReady == ITDS_disable);

        getAccelerationX(&acc_x);
        getAccelerationY(&acc_y);
        getAccelerationZ(&acc_z);

        debug_print<float>("acc_x = %6.3f, acc_y = %6.3f, acc_z = %6.3f (m/s)\r\n", acc_x * 9.8f / 1000.0f, acc_y * 9.8f / 1000.0f, acc_z * 9.8f / 1000.0f);

        HAL_Delay(100);
    }
}


HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::enableAutoIncrement()
{
	ITDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	ctrl2.autoAddIncr = ITDS_enable;
	return ITDS_WriteReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::disableAutoIncrement()
{
	ITDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	ctrl2.autoAddIncr = ITDS_disable;
	return ITDS_WriteReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::isAutoIncrementEnabled(ITDS_state_t *ai)
{
	ITDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
	*ai = (ITDS_state_t)ctrl2.autoAddIncr;
	return status;
}

bool IMU_3DIM_WSEN_ITDS::isAutoIncrementEnabled()
{
	ITDS_ctrl2_t ctrl2;

	ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	return ((ITDS_state_t)ctrl2.autoAddIncr) == ITDS_enable;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::enableBlockDataUpdate()
{
    ITDS_ctrl2_t ctrl2;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
    ctrl2.blockDataUpdate = ITDS_enable;
    return ITDS_WriteReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::disableBlockDataUpdate()
{
    ITDS_ctrl2_t ctrl2;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
    ctrl2.blockDataUpdate = ITDS_disable;
    return ITDS_WriteReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::isBlockDataUpdateEnabled(ITDS_state_t *bdu)
{
    ITDS_ctrl2_t ctrl2;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK) {
		return status;
	}
    *bdu = (ITDS_state_t) ctrl2.blockDataUpdate;
    return status;
}

bool IMU_3DIM_WSEN_ITDS::isBlockDataUpdateEnabled()
{
    ITDS_ctrl2_t ctrl2;

    ITDS_ReadReg(ITDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
    return ((ITDS_state_t) ctrl2.blockDataUpdate == ITDS_enable);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::setOperatingMode(ITDS_operatingMode_t opMode)
{
    ITDS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    if (status != HAL_OK) {
		return status;
	}
    ctrl1.operatingMode = opMode;
    return ITDS_WriteReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getOperatingMode(ITDS_operatingMode_t *opMode)
{
    ITDS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    if (status != HAL_OK) {
		return status;
	}
    *opMode = (ITDS_operatingMode_t) ctrl1.operatingMode;
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::setPowerMode(ITDS_powerMode_t powerMode)
{
    ITDS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    if (status != HAL_OK) {
		return status;
	}
    ctrl1.powerMode = powerMode;
    return ITDS_WriteReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getPowerMode(ITDS_powerMode_t *powerMode)
{
    ITDS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    if (status != HAL_OK) {
		return status;
	}
    *powerMode = (ITDS_powerMode_t)ctrl1.powerMode;
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::setOutputDataRate(ITDS_outputDataRate_t odr)
{
    ITDS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    if (status != HAL_OK) {
		return status;
	}
    ctrl1.outputDataRate = odr;
    return ITDS_WriteReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getOutputDataRate(ITDS_outputDataRate_t* odr)
{
    ITDS_ctrl1_t ctrl1;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
    if (status != HAL_OK) {
		return status;
	}
    *odr = (ITDS_outputDataRate_t) ctrl1.outputDataRate;
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::setFullScale(ITDS_fullScale_t fullScale)
{
    ITDS_ctrl6_t ctrl6;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_6_REG, 1, (uint8_t *) &ctrl6);
    if (status != HAL_OK) {
		return status;
	}
    ctrl6.fullScale = fullScale;
    status = ITDS_WriteReg(ITDS_CTRL_6_REG, 1, (uint8_t *) &ctrl6);
    if (status != HAL_OK) {
		return status;
	}
    currentFullScale = fullScale;
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getFullScale(ITDS_fullScale_t *fullScale)
{
    ITDS_ctrl6_t ctrl6;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_CTRL_6_REG, 1, (uint8_t *) &ctrl6);
    if (status != HAL_OK) {
		return status;
	}
    *fullScale = (ITDS_fullScale_t) ctrl6.fullScale;
    currentFullScale = (ITDS_fullScale_t) ctrl6.fullScale;
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::isAccelerationDataReady(ITDS_state_t *dataReady)
{
    ITDS_status_t statusRegister;
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_STATUS_REG, 1, (uint8_t *) &statusRegister);
    if (status != HAL_OK) {
		return status;
	}
    *dataReady = (ITDS_state_t) statusRegister.dataReady;
    return status;
}

bool IMU_3DIM_WSEN_ITDS::isAccelerationDataReady()
{
    ITDS_status_t statusRegister;

    ITDS_ReadReg(ITDS_STATUS_REG, 1, (uint8_t *) &statusRegister);
    return (((ITDS_state_t) statusRegister.dataReady) == ITDS_enable);
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getAccelerationX(float *xAcc)
{
    int16_t rawAcc = 0;
    uint8_t tmp[2] = {0};
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_X_OUT_L_REG, 2, tmp);
    if (status != HAL_OK) {
		return status;
	}
    rawAcc = (int16_t) (tmp[1] << 8);
    rawAcc |= (int16_t) tmp[0];
    *xAcc = convertAcceleration_float(rawAcc);
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getAccelerationY(float *yAcc)
{
    int16_t rawAcc = 0;
    uint8_t tmp[2] = {0};
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_Y_OUT_L_REG, 2, tmp);
    if (status != HAL_OK) {
		return status;
	}
    rawAcc = (int16_t) (tmp[1] << 8);
    rawAcc |= (int16_t) tmp[0];
    *yAcc = convertAcceleration_float(rawAcc);
    return status;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getAccelerationZ(float *zAcc)
{
    int16_t rawAcc = 0;
    uint8_t tmp[2] = {0};
    HAL_StatusTypeDef status;

    status = ITDS_ReadReg(ITDS_Z_OUT_L_REG, 2, tmp);
    if (status != HAL_OK) {
		return status;
	}
    rawAcc = (int16_t) (tmp[1] << 8);
    rawAcc |= (int16_t) tmp[0];
    *zAcc = convertAcceleration_float(rawAcc);
    return status;
}


// ***** private funcitions *****

inline float IMU_3DIM_WSEN_ITDS::convertAcceleration_float(const int16_t &acc)
{
    switch (currentFullScale)
    {
        case ITDS_twoG:     return ((float) acc) * 0.061f;
        case ITDS_fourG:    return ((float) acc) * 0.122f;
        case ITDS_eightG:   return ((float) acc) * 0.122f;
        case ITDS_sixteenG: return ((float) acc) * 0.488f;
        default:            return 0;
    }
}

inline HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::ITDS_ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf)
{
    // HAL_I2C_Mem_Read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
    return HAL_I2C_Mem_Read(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, buf, numBytesToRead, I2C_TIMEOUT);
}

inline HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::ITDS_WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data)
{
    // HAL_I2C_Mem_Write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
    return HAL_I2C_Mem_Write(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, data, numBytesToWrite, I2C_TIMEOUT);
}


