#include "TMAG5273.h"
#include "util.h"
#include "math.h"

static const uint32_t I2C_TIMEOUT = 100;

TMAG5273::TMAG5273(I2C_HandleTypeDef* i2c_handler): i2c_handler(i2c_handler)
{
    i2c_device_addr = TMAG5273A1_I2C_ADDRESS;
}

 bool TMAG5273::init(
    tmag5273_conv_avg_rate_t CONV_AVG,
    tmag5273_lp_ln_mode_t LP_LN,
	tmag5273_mag_range_t RANGE,
	tmag5273_angle_ch_en_t ANGLE)
{
    tmag5273_conv_avg_rate_t odr;
    tmag5273_lp_ln_mode_t lp_ln;
	tmag5273_mag_range_t range;
	tmag5273_angle_ch_en_t angle;
    // Wait for Time to go into continuous measure mode from stand-by mode
	HAL_Delay(70);

	changeToStandByMode();
    // Communication test.
	if (!isCommunicationReady()) {
		return false;
	}
    debug_print("TMAG: Initializes.\r\n");
	HAL_Delay(10);

	changeToContinuousMeasureMode();
    debug_print("TMAG: ContinuousMeasureMode.\r\n");

	enableMagneticChannels();
	enableTemperatureChannels();
	debug_print("TMAG: EnableMagneticChannels.\r\n");

    setOutputDataRate(CONV_AVG);
	getOutputDataRate(&odr);
	if (odr == CONV_AVG) {
		debug_print("TMAG: Set output data\r\n");  // TODO: prine message
    } else {
        debug_print("<Error> TMAG: Failed to set the output data rate.\r\n");
		return false;
    }

    setLPLNMode(LP_LN);
	getLPLNMode(&lp_ln);
	if (lp_ln == LP_LN) {
		debug_print("TMAG: Set output data\r\n");  // TODO: prine message
    } else {
        debug_print("<Error> TMAG: Failed to set the lp_ln mode.\r\n");
		return false;
    }

    setTMAGRange(RANGE);
	getTMAGRange(&range);
	if (range == RANGE) {
		debug_print("TMAG: Set output data\r\n");  // TODO: prine message
    } else {
        debug_print("<Error> TMAG: Failed to set the range.\r\n");
		return false;
    }

    setTMAGAngleEnable(ANGLE);
	getTMAGAngleEnable(&angle);
	if (angle == ANGLE) {
		debug_print("TMAG: Set angle data\r\n");  // TODO: prine message
    } else {
        debug_print("<Error> TMAG: Failed to set the angle.\r\n");
		return false;
    }

    return true;
}

HAL_StatusTypeDef TMAG5273::getDeviceID(uint8_t *device_ID)
{
    return TMAG_ReadReg(DEVICE_ID, 1, device_ID);
}

bool TMAG5273::isCommunicationReady()
{
    uint8_t device_ID = 0;
    HAL_StatusTypeDef status;

    status = getDeviceID(&device_ID);
	if (device_ID == DEVICE_ID_VALUE) {
		debug_print<uint8_t>("TMAG5273: device ID = 0x%x. Communication is successful.\r\n", device_ID);
		return true;
	} else {
		debug_print<uint8_t>("TMAG5273: device ID = 0x%x. Communication is failed.\r\n", device_ID);
		return false;
	}
}

HAL_StatusTypeDef TMAG5273::changeToStandByMode()
{
	tmag5273_device_config_2_t device_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
	if (status != HAL_OK) {
		return status;
	}

	device_config_2.OPERATING_MODE = TMAG5273_OPERATING_MODE_STANDBY;
	return TMAG_WriteReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
}

HAL_StatusTypeDef TMAG5273::changeToContinuousMeasureMode()
{
	tmag5273_device_config_2_t device_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
	if (status != HAL_OK) {
		return status;
	}

	if (device_config_2.OPERATING_MODE == TMAG5273_OPERATING_MODE_STANDBY){
		device_config_2.OPERATING_MODE = TMAG5273_OPERATING_MODE_MEASURE;
	}
	return TMAG_WriteReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
}

HAL_StatusTypeDef TMAG5273::enableMagneticChannels()
{
	tmag5273_sensor_config_1_t sensor_config_1;
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(SENSOR_CONFIG_1, 1, (uint8_t *) &sensor_config_1);
	if (status != HAL_OK)
		return status;
	sensor_config_1.MAG_CH_EN = TMAG5273_TEMP_CH_ENABLED_XYZ;
	return TMAG_WriteReg(SENSOR_CONFIG_1, 1, (uint8_t *)&sensor_config_1);
}

HAL_StatusTypeDef TMAG5273::disableMagneticChannels()
{
	tmag5273_sensor_config_1_t sensor_config_1;
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(SENSOR_CONFIG_1, 1, (uint8_t *) &sensor_config_1);
	if (status != HAL_OK)
		return status;
	sensor_config_1.MAG_CH_EN = TMAG5273_TEMP_CH_DISABLED;
	return TMAG_WriteReg(SENSOR_CONFIG_1, 1, (uint8_t *)&sensor_config_1);
}

bool TMAG5273::isMagneticChannelsEnabled()
{
	tmag5273_sensor_config_1_t sensor_config_1;
    tmag5273_temp_ch_en_t channel;
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(SENSOR_CONFIG_1, 1, (uint8_t *) &sensor_config_1);
	if (status != HAL_OK)
		return status;
	channel = (tmag5273_temp_ch_en_t) sensor_config_1.MAG_CH_EN;
	return (channel == TMAG5273_TEMP_CH_ENABLED_XYZ);
}

HAL_StatusTypeDef TMAG5273::enableTemperatureChannels()
{
	tmag5273_T_config_t tmag5273_T_config;
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(T_CONFIG, 1, (uint8_t *) &tmag5273_T_config);
	if (status != HAL_OK)
		return status;
	tmag5273_T_config.T_CH_EN = TMAG5273_Temperature_CH_ENABLED;
	return TMAG_WriteReg(T_CONFIG, 1, (uint8_t *)&tmag5273_T_config);
}

HAL_StatusTypeDef TMAG5273::disableTemperatureChannels()
{
	tmag5273_T_config_t tmag5273_T_config;
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(T_CONFIG, 1, (uint8_t *) &tmag5273_T_config);
	if (status != HAL_OK)
		return status;
	tmag5273_T_config.T_CH_EN = TMAG5273_Temperature_CH_DISABLED;
	return TMAG_WriteReg(T_CONFIG, 1, (uint8_t *)&tmag5273_T_config);
}

bool TMAG5273::isMagneticTemperatureEnabled()
{
	tmag5273_T_config_t tmag5273_T_config;
    tmag5273_temperature_ch_en_t channel;
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(T_CONFIG, 1, (uint8_t *) &tmag5273_T_config);
	if (status != HAL_OK)
		return status;
	channel = (tmag5273_temperature_ch_en_t) tmag5273_T_config.T_CH_EN;
	return (channel == TMAG5273_Temperature_CH_ENABLED);
}



HAL_StatusTypeDef TMAG5273::setOutputDataRate(tmag5273_conv_avg_rate_t odr)
{
	tmag5273_device_config_1_t device_config_1;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(DEVICE_CONFIG_1, 1, (uint8_t *) &device_config_1);
	if (status != HAL_OK) {
		return status;
	}
	device_config_1.CONV_AVG = odr;
	return TMAG_WriteReg(DEVICE_CONFIG_1, 1, (uint8_t *) &device_config_1);
}

HAL_StatusTypeDef TMAG5273::getOutputDataRate(tmag5273_conv_avg_rate_t* odr)
{
	tmag5273_device_config_1_t device_config_1;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(DEVICE_CONFIG_1, 1, (uint8_t *) &device_config_1);
	if (status != HAL_OK) {
		return status;
	}
	*odr = (tmag5273_conv_avg_rate_t) device_config_1.CONV_AVG;
	return status;
}

HAL_StatusTypeDef TMAG5273::setLPLNMode(tmag5273_lp_ln_mode_t lp_ln)
{
	tmag5273_device_config_2_t device_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
	if (status != HAL_OK) {
		return status;
	}
	device_config_2.LP_LN = lp_ln;
	return TMAG_WriteReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
}

HAL_StatusTypeDef TMAG5273::getLPLNMode(tmag5273_lp_ln_mode_t* lp_ln)
{
	tmag5273_device_config_2_t device_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(DEVICE_CONFIG_2, 1, (uint8_t *) &device_config_2);
	if (status != HAL_OK) {
		return status;
	}
	*lp_ln = (tmag5273_lp_ln_mode_t) device_config_2.LP_LN;
	return status;
}

HAL_StatusTypeDef TMAG5273::setTMAGRange(tmag5273_mag_range_t range)
{
	tmag5273_sensor_config_2_t sensor_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(SENSOR_CONFIG_2, 1, (uint8_t *) &sensor_config_2);
	if (status != HAL_OK) {
		return status;
	}
	sensor_config_2.Z_RANGE = range;
	sensor_config_2.X_Y_RANGE = range;
	return TMAG_WriteReg(SENSOR_CONFIG_2, 1, (uint8_t *) &sensor_config_2);
}

HAL_StatusTypeDef TMAG5273::getTMAGRange(tmag5273_mag_range_t* range)
{
	tmag5273_sensor_config_2_t sensor_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(SENSOR_CONFIG_2, 1, (uint8_t *) &sensor_config_2);
	if (status != HAL_OK) {
		return status;
	}
	*range = (tmag5273_mag_range_t) sensor_config_2.Z_RANGE;
	*range = (tmag5273_mag_range_t) sensor_config_2.X_Y_RANGE;
	return status;
}

HAL_StatusTypeDef TMAG5273::getMagneticField_X(float *Bx)
{
	int16_t rawB;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(X_MSB_RESULT, 1, tmp_MSB);  // Read X_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(X_LSB_RESULT, 1, tmp_LSB);  // Read X_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawB = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	// *Bx = (float) rawB;
	*Bx = convertMagneticField_float((float) rawB, range);  // Store rawB value as float
    return status;
}

HAL_StatusTypeDef TMAG5273::getMagneticField_Y(float *By)
{
	int16_t rawB;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(Y_MSB_RESULT, 1, tmp_MSB);  // Read Y_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(Y_LSB_RESULT, 1, tmp_LSB);  // Read Y_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawB = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	// *By = (float) rawB;
	*By = convertMagneticField_float((float) rawB, range);  // Store rawB value as float
    return status;
}

HAL_StatusTypeDef TMAG5273::getMagneticField_Z(float *Bz)
{
	int16_t rawB;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(Z_MSB_RESULT, 1, tmp_MSB);  // Read Z_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(Z_LSB_RESULT, 1, tmp_LSB);  // Read Z_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawB = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	// *Bz = (float) rawB;
	*Bz = convertMagneticField_float((float) rawB, range);  // Store rawB value as float
    return status;
}

float TMAG5273::getMagneticField_X()
{
	int16_t rawB;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(X_MSB_RESULT, 1, tmp_MSB);  // Read X_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(X_LSB_RESULT, 1, tmp_LSB);  // Read X_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawB = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	return rawB;
}

float TMAG5273::getMagneticField_Y()
{
	int16_t rawB;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(Y_MSB_RESULT, 1, tmp_MSB);  // Read Y_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(Y_LSB_RESULT, 1, tmp_LSB);  // Read Y_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawB = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	return rawB;
}

float TMAG5273::getMagneticField_Z()
{
	int16_t rawB;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(Z_MSB_RESULT, 1, tmp_MSB);  // Read Z_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(Z_LSB_RESULT, 1, tmp_LSB);  // Read Z_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawB = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	return rawB;
}

HAL_StatusTypeDef TMAG5273::getMagneticField_T(float *T)
{
	int16_t rawT;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(T_MSB_RESULT, 1, tmp_MSB);  // Read Y_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(T_LSB_RESULT, 1, tmp_LSB);  // Read Y_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawT = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	*T = convertTemperature_float(rawT);
    return status;
}

float TMAG5273::getMagneticField_T()
{
	int16_t rawT;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(T_MSB_RESULT, 1, tmp_MSB);  // Read Z_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(T_LSB_RESULT, 1, tmp_LSB);  // Read Z_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawT = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
	return convertTemperature_float(rawT);
}

HAL_StatusTypeDef TMAG5273::setTMAGAngleEnable(tmag5273_angle_ch_en_t angle)
{
	tmag5273_sensor_config_2_t sensor_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(SENSOR_CONFIG_2, 1, (uint8_t *) &sensor_config_2);
	if (status != HAL_OK) {
		return status;
	}
	sensor_config_2.ANGLE_EN = angle;
	return TMAG_WriteReg(SENSOR_CONFIG_2, 1, (uint8_t *) &sensor_config_2);
}

HAL_StatusTypeDef TMAG5273::getTMAGAngleEnable(tmag5273_angle_ch_en_t* angle)
{
	tmag5273_sensor_config_2_t sensor_config_2;
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(SENSOR_CONFIG_2, 1, (uint8_t *) &sensor_config_2);
	if (status != HAL_OK) {
		return status;
	}
	*angle = (tmag5273_angle_ch_en_t) sensor_config_2.ANGLE_EN;
	return status;
}

HAL_StatusTypeDef TMAG5273::getMagneticAngle_w(float *W)
{
	int16_t rawW;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(ANGLE_RESULT_MSB, 1, tmp_MSB);  // Read Y_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(ANGLE_RESULT_LSB, 1, tmp_LSB);  // Read Y_LSB_RESULT
	if (status != HAL_OK)
		return status;

	rawW = (int16_t) ((tmp_MSB[0] << 8) | tmp_LSB[0]);  // Combine MSB and LSB
    int16_t integerPart = rawW >> 4;          // Shift right by 4 bits to get the integer part
    uint16_t fractionalPart = rawW & 0x0F;    // Mask the last 4 bits to get the fractional part

    // Combine the integer and fractional parts to get the floating-point result
    *W = (float)integerPart + ((float)fractionalPart / 16.0f);
    return status;
}

float TMAG5273::getMagneticAngle_W()
{
	int16_t rawW;
	uint8_t tmp_MSB[1] = {0}; 
	uint8_t tmp_LSB[1] = {0};
	HAL_StatusTypeDef status = HAL_OK;

	status = TMAG_ReadReg(T_MSB_RESULT, 1, tmp_MSB);  // Read Z_MSB_RESULT
	if (status != HAL_OK)
		return status;

	status = TMAG_ReadReg(T_LSB_RESULT, 1, tmp_LSB);  // Read Z_LSB_RESULT
	if (status != HAL_OK)
		return status;

    // Extract the integer and fractional parts
    int16_t integerPart = rawW >> 4;          // Shift right by 4 bits to get the integer part
    uint16_t fractionalPart = rawW & 0x0F;    // Mask the last 4 bits to get the fractional part

    // Combine the integer and fractional parts to get the floating-point result
    float result = (float)integerPart + ((float)fractionalPart / 16.0f);

    return result;
}

HAL_StatusTypeDef TMAG5273::getMagneticOffest_e(float *E)
{
	int8_t rawE;
	uint8_t tmp[1] = {0};
	HAL_StatusTypeDef status;

	status = TMAG_ReadReg(MAG_OFFSET_CONFIG_2, 1, tmp);  
	if (status != HAL_OK)
		return status;

	rawE = (int8_t) tmp[0];  
	//*E = convertMagneticOffest_float((float) rawE, range);  // Store rawB value as float
	*E = (float) rawE;
    return status;
}


// ***** private funcitions *****


float TMAG5273::convertMagneticField_float(int16_t B, tmag5273_mag_range_t range)
{
	switch (range)
	{
		case TMAG5273_MAG_RANGE_40MT:	return ((float) 2 * 40 * B) / pow(2 , 16);
		case TMAG5273_MAG_RANGE_80MT:	return ((float) 2 * 80 * B) / pow(2 , 16);
	}
}

float TMAG5273::convertMagneticOffest_float(int8_t E, tmag5273_mag_range_t range)
{
	switch (range)
	{
		case TMAG5273_MAG_RANGE_40MT:	return ((float) 2 * 40 * E) / pow(2 , 12);
		case TMAG5273_MAG_RANGE_80MT:	return ((float) 2 * 80 * E) / pow(2 , 12);
	}
}

float TMAG5273::convertTemperature_float(int16_t T)
{
	return 	25 + ((float)T - 17508) / 60.1;
}

inline HAL_StatusTypeDef TMAG5273::TMAG_ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf)
{
    // HAL_I2C_Mem_Read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
    return HAL_I2C_Mem_Read(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, buf, numBytesToRead, I2C_TIMEOUT);
}

inline HAL_StatusTypeDef TMAG5273::TMAG_WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data)
{
    // HAL_I2C_Mem_Write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout)
    return HAL_I2C_Mem_Write(i2c_handler, i2c_device_addr, regAddr, i2cMemAddSize, data, numBytesToWrite, I2C_TIMEOUT);
}
