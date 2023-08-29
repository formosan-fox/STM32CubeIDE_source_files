#include <IMU_6DIM_WSEN_ISDS.h>
#include "util.h"
#include <map>
#include <string>

static const uint32_t SPI_TIMEOUT = 100;
static const uint32_t I2C_TIMEOUT = 100;

static std::map<ISDS_accOutputDataRate_t, std::string> accOdr_str_map = {
    {ISDS_accOdrOff, "Off"},
    {ISDS_accOdr12Hz5, "12.5 Hz"},
    {ISDS_accOdr26Hz, "26 Hz"},
    {ISDS_accOdr52Hz, "52 Hz"},
    {ISDS_accOdr104Hz, "104 Hz"},
    {ISDS_accOdr208Hz, "208 Hz"},
    {ISDS_accOdr416Hz, "416 Hz"},
    {ISDS_accOdr833Hz, "833 Hz"},
    {ISDS_accOdr1k66Hz, "1.66 kHz"},
    {ISDS_accOdr3k33Hz, "3.33 kHz"},
    {ISDS_accOdr6k66Hz, "6.66 kHz"},
    {ISDS_accOdr1Hz6, "1.6 Hz"}
};

static std::map<ISDS_gyroOutputDataRate_t, std::string> gyroOdr_str_map = {
    {ISDS_gyroOdrOff, "Off"},
    {ISDS_gyroOdr12Hz5, "12.5 Hz"},
    {ISDS_gyroOdr26Hz, "26 Hz"},
    {ISDS_gyroOdr52Hz, "52 Hz"},
    {ISDS_gyroOdr104Hz, "104 Hz"},
    {ISDS_gyroOdr208Hz, "208 Hz"},
    {ISDS_gyroOdr416Hz, "416 Hz"},
    {ISDS_gyroOdr833Hz, "833 Hz"},
    {ISDS_gyroOdr1k66Hz, "1.66 kHz"},
    {ISDS_gyroOdr3k33Hz, "3.33 kHz"},
    {ISDS_gyroOdr6k66Hz, "6.66 kHz"}
};

static std::map<ISDS_accFullScale_t, std::string> accFullScale_str_map = {
    {ISDS_accFullScaleTwoG, "2G"},
    {ISDS_accFullScaleSixteenG, "16G"},
    {ISDS_accFullScaleFourG, "4G"},
    {ISDS_accFullScaleEightG, "8G"},
    {ISDS_accFullScaleInvalid, "Invalid"}
};

static std::map<ISDS_gyroFullScale_t, std::string> gyroFullScale_str_map = {
    {ISDS_gyroFullScale250dps, "250 dps"},
    {ISDS_gyroFullScale125dps, "125 dps"},
    {ISDS_gyroFullScale500dps, "500 dps"},
    {ISDS_gyroFullScale1000dps, "1000 dps"},
    {ISDS_gyroFullScale2000dps, "2000 dps"}
};


IMU_6DIM_WSEN_ISDS::IMU_6DIM_WSEN_ISDS(SPI_HandleTypeDef* _spi_handler, GPIO_TypeDef *_CS_PORT, uint16_t _CS_PIN):
    spi_handler(_spi_handler), CS_PORT(_CS_PORT), CS_PIN(_CS_PIN)
{
	interface = ISDS_SPI;
    currentAccFullScale = ISDS_accFullScaleTwoG;
    currentGyroFullScale = ISDS_gyroFullScale250dps;
}

IMU_6DIM_WSEN_ISDS::IMU_6DIM_WSEN_ISDS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO): i2c_handler(_i2c_handler)
{
	interface = ISDS_I2C;
    i2c_device_addr = (SAO == 0) ? ISDS_ADDRESS_I2C_0 : ISDS_ADDRESS_I2C_1;
    currentAccFullScale = ISDS_accFullScaleTwoG;
    currentGyroFullScale = ISDS_gyroFullScale250dps;
}

// ***** public funcitions *****

bool IMU_6DIM_WSEN_ISDS::init(
    ISDS_state_t blockDataUpdate,
    ISDS_accOutputDataRate_t accOutputDataRate,
    ISDS_gyroOutputDataRate_t gyroOutputDataRate,
    ISDS_accFullScale_t accFullScale,
    ISDS_gyroFullScale_t gyroFullScale)
{
    HAL_StatusTypeDef status;

	// TODO: Initialize sensor interface.

	// Pull CS high if it uses SPI interface.
    if (interface == ISDS_SPI) {
    	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
    }

	// Wait for boot
	HAL_Delay(50);


	// TODO: Check Sensor Interface Ready

	// Communication test.
	if (!isCommunicationReady()) {
		return false;
	}

    debug_print("WSEN_ISDS: Initializes.\r\n");

	// Enable block data update
    if (blockDataUpdate == ISDS_enable){
	    status = enableBlockDataUpdate();
        if (isBlockDataUpdateEnabled()) {
            debug_print("WSEN ISDS: Block data update is enabled.\r\n");
        } else {
            debug_print("<Error> WSEN ISDS: Failed to enable block data update.\r\n");
        }
        if (status != HAL_OK) {
            return false;
        }
    }
    

	// Accelerometer sampling rate (default: 104 Hz)
	status = setAccOutputDataRate(accOutputDataRate);
	if (accOutputDataRate == getAccOutputDataRate()) {
		debug_print("WSEN ISDS: Accelerator output data rate is %s.\r\n", accOdr_str_map[accOutputDataRate].c_str());
	} else {
		debug_print("<Error> WSEN ISDS: Failed to set accelerator output data rate.\r\n");
        if (status != HAL_OK) {
            return false;
        }
	}

    // Gyroscope sampling rate (default: 104 Hz)
	status = setGyroOutputDataRate(gyroOutputDataRate);
	if (gyroOutputDataRate == getGyroOutputDataRate()) {
		debug_print("WSEN ISDS: Gyroscope output data rate is %s.\r\n", gyroOdr_str_map[gyroOutputDataRate].c_str());
	} else {
		debug_print("<Error> WSEN ISDS: Failed to set gyroscope output data rate.\r\n");
        if (status != HAL_OK) {
            return false;
        }
	}

	// Accelerometer full scale
	status = setAccFullScale(accFullScale);
    if (accFullScale == getAccFullScale()) {
        debug_print("WSEN ISDS: Accelerator full scale is %s.\r\n", accFullScale_str_map[accFullScale].c_str());
	} else {
		debug_print("<Error> WSEN ISDS: Failed to set accelerator full scale.\r\n");
        if (status != HAL_OK) {
            return false;
        }
	}

	// Gyroscope full scale
	status = setGyroFullScale(gyroFullScale);
    if (gyroFullScale == getGyroFullScale()) {
        debug_print("WSEN ISDS: Gyroscope full scale is %s.\r\n", gyroFullScale_str_map[gyroFullScale].c_str());
	} else {
		debug_print("<Error> WSEN ISDS: Failed to set gyroscope full scale.\r\n");
        if (status != HAL_OK) {
            return false;
        }
	}

	return true;
}

bool IMU_6DIM_WSEN_ISDS::isCommunicationReady()
{
    uint8_t device_ID = 0;

    getDeviceID(&device_ID);
	if (device_ID == ISDS_DEVICE_ID_VALUE) {
		debug_print<uint8_t>("WSEN ISDS: device ID = 0x%x. Communication is successful.\r\n", device_ID);
		return true;
	} else {
		debug_print<uint8_t>("WSEN ISDS: device ID = 0x%x. Communication is failed.\r\n", device_ID);
		return false;
	}
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getDeviceID(uint8_t *deviceID)
{
    HAL_StatusTypeDef status = HAL_OK;

	// %% Bug %%: The first read returns 0.
    ISDS_ReadReg(ISDS_DEVICE_ID_REG, 1, deviceID);

	// The second read returns a meaningful value.
	status = ISDS_ReadReg(ISDS_DEVICE_ID_REG, 1, deviceID);

    return status;
}

void IMU_6DIM_WSEN_ISDS::example_loop()
{
	float xAcc = 0;
	float yAcc = 0;
	float zAcc = 0;
	float xRate = 0;
	float yRate = 0;
	float zRate = 0;
	float temperature = 0;
    int n;

	char uart_buf[150];
	int uart_buf_len;

	while (1) {
        n = 0;
		while (isAccelerationDataReady() == false) {
            HAL_Delay(10);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN ISDS: Acceleration data not available.\r\n");
				break;
			}
        };
		getAccelerationX_float(&xAcc);
		getAccelerationY_float(&yAcc);
		getAccelerationZ_float(&zAcc);

        n = 0;
		while (isGyroscopeDataReady() == false) {
            HAL_Delay(10);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN ISDS: Gyroscope data not available.\r\n");
				break;
			}
        };
		getAngularRateX_float(&xRate);
		getAngularRateY_float(&yRate);
		getAngularRateZ_float(&zRate);

        n = 0;
        while (isTemperatureDataReady() == false) {
            HAL_Delay(10);
			n++;
			if (n > 10) {
				debug_print("<Error> WSEN ISDS: Temperature data not available.\r\n");
				break;
			}
        };
		getTemperature_float(&temperature);


		uart_buf_len = sprintf(uart_buf, "acc = (%5.2f, %5.2f, %5.2f)(m/s),   angular_acc = (%9.2f, %9.2f, %9.2f)(mdps),   temp = %.2f (C)\r\n",
							   xAcc * 9.8 / 1000.0, yAcc * 9.8 / 1000.0, zAcc * 9.8 / 1000.0, xRate, yRate, zRate, temperature);
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, SPI_TIMEOUT);

		HAL_Delay(100);
	}
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::setAccFullScale(ISDS_accFullScale_t fullScale)
{
	ISDS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;

	ctrl1.accFullScale = fullScale;
	status = ISDS_WriteReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);

	// Store current full scale value to allow convenient conversion of sensor readings.
	if (status == HAL_OK)
		currentAccFullScale = fullScale;
	return status;
}

uint8_t IMU_6DIM_WSEN_ISDS::getAccFullScale()
{
	ISDS_ctrl1_t ctrl1;
    ISDS_accFullScale_t fullScale;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	fullScale = (ISDS_accFullScale_t) ctrl1.accFullScale;
	currentAccFullScale = fullScale;
	return fullScale;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::setAccOutputDataRate(ISDS_accOutputDataRate_t odr)
{
	ISDS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	ctrl1.accOutputDataRate = odr;
	return ISDS_WriteReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAccFullScale(ISDS_accFullScale_t *fullScale)
{
	ISDS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	*fullScale = (ISDS_accFullScale_t) ctrl1.accFullScale;
	currentAccFullScale = *fullScale;
	return status;
}

uint8_t IMU_6DIM_WSEN_ISDS::getAccOutputDataRate()
{
	ISDS_ctrl1_t ctrl1;
    ISDS_accOutputDataRate_t odr;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	odr = (ISDS_accOutputDataRate_t) ctrl1.accOutputDataRate;
	return odr;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAccOutputDataRate(ISDS_accOutputDataRate_t *odr)
{
	ISDS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	*odr = (ISDS_accOutputDataRate_t) ctrl1.accOutputDataRate;
	return status;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::setGyroFullScale(ISDS_gyroFullScale_t fullScale)
{
	ISDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;

	ctrl2.gyroFullScale = fullScale;
	status = ISDS_WriteReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);

	// Store current full scale value to allow convenient conversion of sensor readings.
	if (status == HAL_OK)
		currentGyroFullScale = fullScale;
	return status;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getGyroFullScale(ISDS_gyroFullScale_t *fullScale)
{
	ISDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;
	*fullScale = (ISDS_gyroFullScale_t) ctrl2.gyroFullScale;
	currentGyroFullScale = *fullScale;
	return status;
}

uint8_t IMU_6DIM_WSEN_ISDS::getGyroFullScale()
{
	ISDS_ctrl2_t ctrl2;
    ISDS_gyroFullScale_t fullScale;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;
	fullScale = (ISDS_gyroFullScale_t) ctrl2.gyroFullScale;
	currentGyroFullScale = fullScale;
	return fullScale;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::setGyroOutputDataRate(ISDS_gyroOutputDataRate_t odr)
{
	ISDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;
	ctrl2.gyroOutputDataRate = odr;
	return ISDS_WriteReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getGyroOutputDataRate(ISDS_gyroOutputDataRate_t *odr)
{
	ISDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;
	*odr = (ISDS_gyroOutputDataRate_t) ctrl2.gyroOutputDataRate;
	return status;
}

uint8_t IMU_6DIM_WSEN_ISDS::getGyroOutputDataRate()
{
	ISDS_ctrl2_t ctrl2;
    ISDS_gyroOutputDataRate_t odr;
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;
	odr = (ISDS_gyroOutputDataRate_t) ctrl2.gyroOutputDataRate;
	return odr;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::enableBlockDataUpdate()
{
	ISDS_ctrl3_t ctrl3;
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
	if (status != HAL_OK)
		return status;
	ctrl3.blockDataUpdate = ISDS_enable;
	return ISDS_WriteReg(ISDS_CTRL_3_REG, 1, (uint8_t *)&ctrl3);
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::disableBlockDataUpdate()
{
	ISDS_ctrl3_t ctrl3;
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
	if (status != HAL_OK)
		return status;
	ctrl3.blockDataUpdate = ISDS_disable;
	return ISDS_WriteReg(ISDS_CTRL_3_REG, 1, (uint8_t *)&ctrl3);
}

bool IMU_6DIM_WSEN_ISDS::isBlockDataUpdateEnabled()
{
	ISDS_ctrl3_t ctrl3;
    ISDS_state_t bdu;  
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
	if (status != HAL_OK)
		return status;
	bdu = (ISDS_state_t) ctrl3.blockDataUpdate;
	return (bdu == ISDS_enable);
}

bool IMU_6DIM_WSEN_ISDS::isAccelerationDataReady()
{
	ISDS_status_t statusRegister;
    ISDS_state_t dataReady = ISDS_disable;

	ISDS_ReadReg(ISDS_STATUS_REG, 1, (uint8_t *) &statusRegister);
    dataReady = (ISDS_state_t) statusRegister.accDataReady;
	return (dataReady == ISDS_enable);
}

bool IMU_6DIM_WSEN_ISDS::isGyroscopeDataReady()
{
	ISDS_status_t statusRegister;
    ISDS_state_t dataReady = ISDS_disable;

    ISDS_ReadReg(ISDS_STATUS_REG, 1, (uint8_t *) &statusRegister);
	dataReady = (ISDS_state_t) statusRegister.gyroDataReady;
	return (dataReady == ISDS_enable);
}

bool IMU_6DIM_WSEN_ISDS::isTemperatureDataReady()
{
	ISDS_status_t statusRegister;
    ISDS_state_t dataReady = ISDS_disable;

	ISDS_ReadReg(ISDS_STATUS_REG, 1, (uint8_t *) &statusRegister);
	dataReady = (ISDS_state_t) statusRegister.tempDataReady;
	return (dataReady == ISDS_enable);
}


HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAccelerationX_float(float *xAcc)
{
	int16_t rawAcc;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_X_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	*xAcc = convertAcceleration_float(rawAcc, currentAccFullScale);
    return status;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAccelerationY_float(float *yAcc)
{
	int16_t rawAcc;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Y_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;


	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	*yAcc = convertAcceleration_float(rawAcc, currentAccFullScale);
    return status;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAccelerationZ_float(float *zAcc)
{
	int16_t rawAcc;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Z_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	*zAcc = convertAcceleration_float(rawAcc, currentAccFullScale);
    return status;
}

float IMU_6DIM_WSEN_ISDS::getAccelerationX_float()
{
	int16_t rawAcc;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_X_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	return convertAcceleration_float(rawAcc, currentAccFullScale);
}

float IMU_6DIM_WSEN_ISDS::getAccelerationY_float()
{
	int16_t rawAcc;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Y_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	return convertAcceleration_float(rawAcc, currentAccFullScale);
}

float IMU_6DIM_WSEN_ISDS::getAccelerationZ_float()
{
	int16_t rawAcc;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Z_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	return convertAcceleration_float(rawAcc, currentAccFullScale);
}


HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAngularRateX_float(float *xRate)
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_X_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	*xRate = convertAngularRate_float(rawRate, currentGyroFullScale);
    return status;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAngularRateY_float(float *yRate)
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Y_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	*yRate = convertAngularRate_float(rawRate, currentGyroFullScale);
    return status;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getAngularRateZ_float(float *zRate)
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Z_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	*zRate = convertAngularRate_float(rawRate, currentGyroFullScale);
    return status;
}

float IMU_6DIM_WSEN_ISDS::getAngularRateX_float()
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_X_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	return convertAngularRate_float(rawRate, currentGyroFullScale);
}

float IMU_6DIM_WSEN_ISDS::getAngularRateY_float()
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Y_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	return convertAngularRate_float(rawRate, currentGyroFullScale);
}


float IMU_6DIM_WSEN_ISDS::getAngularRateZ_float()
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_Z_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	return convertAngularRate_float(rawRate, currentGyroFullScale);
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::getTemperature_float(float *temperature)
{
	int16_t tempRaw;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_OUT_TEMP_L_REG, 2, tmp);
	if (status != HAL_OK)
		return status;
	tempRaw = (int16_t) (tmp[1] << 8);
  	tempRaw |= (int16_t) tmp[0];
	*temperature = ((((float) tempRaw) / 256.0f) + 25.0f);
    return status;
}

float IMU_6DIM_WSEN_ISDS::getTemperature_float()
{
	int16_t tempRaw;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status;

	status = ISDS_ReadReg(ISDS_OUT_TEMP_L_REG, 2, tmp);
	if (status != HAL_OK)
		return status;
	tempRaw = (int16_t) (tmp[1] << 8);
  	tempRaw |= (int16_t) tmp[0];
	return ((((float) tempRaw) / 256.0f) + 25.0f);
}


// ***** private funcitions *****

float IMU_6DIM_WSEN_ISDS::convertAcceleration_float(int16_t acc, ISDS_accFullScale_t fullScale)
{
	switch (fullScale)
	{
		case ISDS_accFullScaleTwoG:		return ((float) acc * 0.061f);
		case ISDS_accFullScaleFourG:	return ((float) acc * 0.122f);
		case ISDS_accFullScaleEightG:	return ((float) acc * 0.244f);
		case ISDS_accFullScaleSixteenG:	return ((float) acc * 0.488f);
		case ISDS_accFullScaleInvalid:
		default:						return 0;
	}
}

float IMU_6DIM_WSEN_ISDS::convertAngularRate_float(int16_t rate, ISDS_gyroFullScale_t fullScale)
{
	switch (fullScale)
	{
		case ISDS_gyroFullScale125dps: 		return ((float) rate * 4.375f);
		case ISDS_gyroFullScale250dps: 		return ((float) rate * 8.75f);
		case ISDS_gyroFullScale500dps: 		return ((float) rate * 17.5f);
		case ISDS_gyroFullScale1000dps:		return ((float) rate * 35.0f);
		case ISDS_gyroFullScale2000dps:		return ((float) rate * 70.0f);
		default:							return 0;
	}
}

inline HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::ISDS_ReadReg(uint8_t regAddr, uint16_t numBytesToRead, uint8_t *buf)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t header;

    // TODO:  Use ifdef, instead of if-else.
	if (interface == ISDS_SPI) {
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
		header = (regAddr) | (1 << 7);
		status = HAL_SPI_Transmit(spi_handler, &header, 1, SPI_TIMEOUT);
		if (status != HAL_OK)
			return status;
		status = HAL_SPI_Receive(spi_handler, (uint8_t *)buf, numBytesToRead, SPI_TIMEOUT);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
		return status;
	} else {
		return HAL_I2C_Mem_Read(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, buf, numBytesToRead, I2C_TIMEOUT);
	}
}

inline HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::ISDS_WriteReg(uint8_t regAddr, uint16_t numBytesToWrite, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_OK;

    // TODO:  Use ifdef, instead of if-else.
	if (interface == ISDS_SPI) {
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
		status = HAL_SPI_Transmit(spi_handler, &regAddr, 1, SPI_TIMEOUT);
		if (status != HAL_OK)
			return status;
		status = HAL_SPI_Transmit(spi_handler, data, numBytesToWrite, SPI_TIMEOUT);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
		return status;
	} else {
		return HAL_I2C_Mem_Write(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, data, numBytesToWrite, I2C_TIMEOUT);
	}
}

