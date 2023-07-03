#include "IMU_6DIM_WSEN_ISDS.h"
#include "util.h"

static const uint32_t SPI_TIMEOUT = 100;

IMU_6DIM_WSEN_ISDS::IMU_6DIM_WSEN_ISDS(SPI_HandleTypeDef* _spi_handler, GPIO_TypeDef *_CS_PORT, uint16_t _CS_PIN):
    spi_handler(_spi_handler), CS_PORT(_CS_PORT), CS_PIN(_CS_PIN)
{
    currentAccFullScale = ISDS_accFullScaleTwoG;
    currentGyroFullScale = ISDS_gyroFullScale250dps;
}

bool IMU_6DIM_WSEN_ISDS::init(
    ISDS_state_t blockDataUpdate,
    ISDS_accOutputDataRate_t accOutputDataRate,
    ISDS_gyroOutputDataRate_t gyroOutputDataRate,
    ISDS_accFullScale_t accFullScale,
    ISDS_gyroFullScale_t gyroFullScale)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t device_ID = 0;

    char uart_buf[100];
    int uart_buf_len;

	// TODO: Initialize sensor interface.

	// Pull CS high.
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

	// Wait for boot
	HAL_Delay(50);


	// TODO: Check Sensor Interface Ready

	// Communication test.
	device_ID = getDeviceID();
	if (device_ID == ISDS_DEVICE_ID_VALUE) {
		uart_buf_len = sprintf(uart_buf, "device ID = %x\r\n", device_ID);
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, SPI_TIMEOUT);
		debug_print("WSEN_ISDS communicates successfully.\r\n");
	} else {
		debug_print("WSEN_ISDS Communicates failed.\r\n");
		return false;
	}

	// TODO: soft reset

	// TODO: reboot

    debug_print("WSEN_ISDS initializes.\r\n");

	// Enable block data update
    if (blockDataUpdate == ISDS_enable){
	    status = enableBlockDataUpdate();
    } else {
        status = disableBlockDataUpdate();
    }
    if (status != HAL_OK)
        return false;

	// Accelerometer sampling rate (default: 104 Hz)
	status = setAccOutputDataRate(accOutputDataRate);
    if (status != HAL_OK)
        return false;

    // Gyroscope sampling rate (default: 104 Hz)
	status = setGyroOutputDataRate(gyroOutputDataRate);
    if (status != HAL_OK)
        return false;

	// Accelerometer (default: 4g range)
	status = setAccFullScale(accFullScale);
    if (status != HAL_OK)
        return false;

	// Gyroscope (default: 500 dps range)
	status = setGyroFullScale(gyroFullScale);
    if (status != HAL_OK)
        return false;

	return true;
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

	char uart_buf[150];
	int uart_buf_len;

	while (1) {
		while (isAccelerationDataReady() == false) {};  // TODO: prevent infinite loop
		xAcc = getAccelerationX_float();
		yAcc = getAccelerationY_float();
		zAcc = getAccelerationZ_float();

		while (isGyroscopeDataReady() == false) {};
		xRate = getAngularRateX_float();
		yRate = getAngularRateY_float();
		zRate = getAngularRateZ_float();

        while (isTemperatureDataReady() == false) {};
		temperature = getTemperature_float();


		uart_buf_len = sprintf(uart_buf, "acc = (%8.2f, %8.2f, %8.2f)(mg),   angular_acc = (%8.2f, %8.2f, %8.2f)(mdps),   temp = %.2f (C)\r\n",
							   xAcc, yAcc, zAcc, xRate, yRate, zRate, temperature);
		HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, SPI_TIMEOUT);

		HAL_Delay(1000);
	}
}

uint8_t IMU_6DIM_WSEN_ISDS::getDeviceID()
{
    uint8_t deviceID = 0;
    HAL_StatusTypeDef status = HAL_OK;

	// %% Bug %%: The first read returns 0.
	ISDS_ReadReg(ISDS_DEVICE_ID_REG, 1, &deviceID);

	// The second read returns a meaningful value.
	ISDS_ReadReg(ISDS_DEVICE_ID_REG, 1, &deviceID);

    return deviceID;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::setAccFullScale(ISDS_accFullScale_t fullScale)
{
	ISDS_ctrl1_t ctrl1;
	HAL_StatusTypeDef status = HAL_OK;

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
	HAL_StatusTypeDef status = HAL_OK;

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
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	ctrl1.accOutputDataRate = odr;
	return ISDS_WriteReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

uint8_t IMU_6DIM_WSEN_ISDS::getAccOutputDataRate()
{
	ISDS_ctrl1_t ctrl1;
    ISDS_accOutputDataRate_t odr;
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
	if (status != HAL_OK)
		return status;
	odr = (ISDS_accOutputDataRate_t) ctrl1.accOutputDataRate;
	return odr;
}

HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::setGyroFullScale(ISDS_gyroFullScale_t fullScale)
{
	ISDS_ctrl2_t ctrl2;
	HAL_StatusTypeDef status = HAL_OK;

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

uint8_t IMU_6DIM_WSEN_ISDS::getGyroFullScale()
{
	ISDS_ctrl2_t ctrl2;
    ISDS_gyroFullScale_t fullScale;
	HAL_StatusTypeDef status = HAL_OK;

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
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
	if (status != HAL_OK)
		return status;
	ctrl2.gyroOutputDataRate = odr;
	return ISDS_WriteReg(ISDS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
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
	HAL_StatusTypeDef status = HAL_OK;

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
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_Z_OUT_L_ACC_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawAcc = (int16_t) (tmp[1] << 8);
	rawAcc |= (int16_t) tmp[0];
	return convertAcceleration_float(rawAcc, currentAccFullScale);
}

float IMU_6DIM_WSEN_ISDS::getAngularRateX_float()
{
	int16_t rawRate;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status = HAL_OK;

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
	HAL_StatusTypeDef status = HAL_OK;

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
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_Z_OUT_L_GYRO_REG, 2, tmp);
	if (status != HAL_OK)
		return status;

	rawRate = (int16_t) (tmp[1] << 8);
	rawRate |= (int16_t) tmp[0];
	return convertAngularRate_float(rawRate, currentGyroFullScale);
}

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

float IMU_6DIM_WSEN_ISDS::getTemperature_float()
{
	int16_t tempRaw;
	uint8_t tmp[2] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = ISDS_ReadReg(ISDS_OUT_TEMP_L_REG, 2, tmp);
	if (status != HAL_OK)
		return status;
	tempRaw = (int16_t) (tmp[1] << 8);
  	tempRaw |= (int16_t) tmp[0];
	return ((((float) tempRaw) / 256.0f) + 25.0f);
}

inline HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::ISDS_ReadReg(uint8_t regAddr, uint16_t numBytesToRead, uint8_t *buf)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t header;

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

	for (uint16_t i = 0; (i < numBytesToRead) && (status == HAL_OK); i++) {
		header = (regAddr + i) | (1 << 7);
		status = HAL_SPI_Transmit(spi_handler, &header, 1, SPI_TIMEOUT);				// (handler, (pointer)data to send, # of bytes to send, timeout)
		if (status != HAL_OK)
			return status;
		status = HAL_SPI_Receive(spi_handler, (uint8_t *)(buf + i), 1, SPI_TIMEOUT);

	}
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return status;
}

inline HAL_StatusTypeDef IMU_6DIM_WSEN_ISDS::ISDS_WriteReg(uint8_t regAddr, uint16_t numBytesToWrite, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_OK;

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(spi_handler, &regAddr, 1, SPI_TIMEOUT);
	if (status != HAL_OK)
		return status;
	status = HAL_SPI_Transmit(spi_handler, data, numBytesToWrite, SPI_TIMEOUT);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
	return status;
}

