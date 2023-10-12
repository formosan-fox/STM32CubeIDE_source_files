#ifndef __Humidity_WSEN_HIDS_H__
#define __Humidity_WSEN_HIDS_H__

/* Device name: WSEN-HIDS Humidity Sensor with Integrated Temperature Sensor
 * Order code: 2525020210001 (THIS API CONNOT BE USE WITH 2525020210002)
 * Datasheet: https://www.we-online.com/components/products/datasheet/2525020210001.pdf
 * User Manual: https://www.we-online.com/components/products/manual/2525020210001_WSEN-HIDS%202525020210001%20Manual_rev1.6.pdf
 * Reference:
 *   - https://github.com/WurthElektronik/Sensors-SDK/tree/master/WSEN-HIDS_2525020210001/drivers
 * Author: Jun-yu Chen
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Device ID */
#define HIDS_DEVICE_ID_VALUE    0xBC

/* I2C slave addresses */
#define HIDS_I2C_ADDRESS      0x5F << 1  

/* Register address definitions */
#define HIDS_DEVICE_ID_REG    0x0F
#define HIDS_AVERAGE_REG      0x10
#define HIDS_CTRL_1_REG       0x20
#define HIDS_CTRL_2_REG       0x21
#define HIDS_CTRL_3_REG       0x22
#define HIDS_STATUS_REG       0x27

/* Humidity calibration registers */
#define HIDS_H0_rH_x2_REG     0x30
#define HIDS_H1_rH_x2_REG     0x31
#define HIDS_H0_T0_OUT_L_REG  0x36
#define HIDS_H0_T0_OUT_H_REG  0x37
#define HIDS_H1_T0_OUT_L_REG  0x3A
#define HIDS_H1_T0_OUT_H_REG  0x3B

/* Humidity output registers (raw values) */
#define HIDS_H_OUT_L_REG      0x28
#define HIDS_H_OUT_H_REG      0x29

/* Temperature calibration registers */
#define HIDS_T1_T0_REG        0x35
#define HIDS_T0_degC_x8_REG   0x32
#define HIDS_T1_degC_x8_REG   0x33
#define HIDS_T0_OUT_L_REG     0x3C
#define HIDS_T0_OUT_H_REG     0x3D
#define HIDS_T1_OUT_L_REG     0x3E
#define HIDS_T1_OUT_H_REG     0x3F

/* Temperature output registers (raw values) */
#define HIDS_T_OUT_L_REG       0x2A
#define HIDS_T_OUT_H_REG       0x2B

typedef struct
{
	GPIO_TypeDef* GPIO_x;
	uint16_t      GPIOPin;
} GPIO_Pin_t;

//======================================================================================
/* Register type definitions using struct */
typedef struct
{
	uint8_t avgHum     : 3;    /* AVGH	: select the numbers of averaged humidity samples (4 - 512),*/
	uint8_t avgTem     : 3;    /* AVGT : select the numbers of averaged temperature samples */
	uint8_t notUsed01  : 2;    /* This bit must be set to 0 for proper operation of the device */
} HIDS_average_config_t;

//*******************************
typedef struct
{
	uint8_t odr              : 2;        /* ODR : output data rate selection */
	uint8_t bdu              : 1;        /* BDU : block data update. 0 - continuous update; 1 - output registers not update till both MSB and LSB are read */
	uint8_t notUsed01        : 4;        /* This bit must be set to 0 for proper operation of the device */
	uint8_t powerControlMode : 1;        /* PD : (0: power-down mode; 1: continuous mode) */
} HIDS_ctrl_1_t;

//*******************************
typedef struct
{
	uint8_t oneShotBit   : 1;       /* One-shot enable : (0: waiting for start of conversion; 1: start for a new dataset) */
	uint8_t heater       : 1;       /* Heater : (0: heater disable; 1: heater enable)*/
	uint8_t notUsed01    : 5;       /* This bit must be set to 0 for proper operation of the device */
	uint8_t rebootMemory : 1;       /* BOOT: : ((0: normal mode; 1: reboot memory content)*/
} HIDS_ctrl_2_t;

//*******************************
typedef struct
{
	uint8_t notUsed01          : 2;    /* This bit must be set to 0 for proper operation of the device */
	uint8_t enDataReady        : 1;    /* (0: Data Ready disabled - default;1: Data Ready signal available on pin 3)*/
	uint8_t notUsed02          : 3;    /* This bit must be set to 0 for proper operation of the device */
	uint8_t interruptPinConfig : 1;    /* PP_OD : : (0: push-pull - default; 1: open drain)*/
	uint8_t drdyOutputLevel    : 1;    /* DRDY_H_L :(0: active high - default;1: active low)*/
} HIDS_ctrl_3_t;

//*******************************
typedef struct
{
	uint8_t humDataAvailable : 1;  /* H_DA : Pressure data available.(0: new data for humidity is not yet available; 1: new data for humidity is available */
	uint8_t tempDataAvailable : 1; /* T_DA :Temperature data available. (0: temperature not yet available; 1: a new temperature data is available) */
	uint8_t notUsed01 : 6;         /* This bit must be set to 0 for proper operation of the device */
} HIDS_status_t;

//================================================================================
/* Definitions using enum */
typedef enum 
{
	HIDS_TemAvg_2   = 0b000,
	HIDS_TemAvg_4   = 0b001,
	HIDS_TemAvg_8   = 0b010,
	HIDS_TemAvg_16  = 0b011,  //default after power up
	HIDS_TemAvg_32  = 0b100,
	HIDS_TemAvg_64  = 0b101,
	HIDS_TemAvg_128 = 0b110,
	HIDS_TemAvg_256 = 0b111,
}HIDS_TemAvgMode_t;

//*******************************
typedef enum 
{
	HIDS_HumAvg_4   = 0b000,
	HIDS_HumAvg_8   = 0b001,
	HIDS_HumAvg_16  = 0b010,
	HIDS_HumAvg_32  = 0b011,  //default after power up
	HIDS_HumAvg_64  = 0b100,
	HIDS_HumAvg_128 = 0b101,
	HIDS_HumAvg_256 = 0b110,
	HIDS_HumAvg_512 = 0b111,
}HIDS_HumAvgMode_t;

//*******************************
typedef enum
{
	HIDS_outputDataRate_OneShot  = 0b00,
	HIDS_outputDataRate_1Hz      = 0b01,
	HIDS_outputDataRate_7Hz      = 0b10,
	HIDS_outputDataRate_12_5Hz   = 0b11,
}HIDS_outputDataRate_t;

//*******************************
typedef enum
{
	HIDS_heaterDisable = 0,
	HIDS_heaterEnable  = 1,
}HIDS_heater_t;

//*******************************
typedef enum
{
	HIDS_oneShotMode    = 0,
	HIDS_continuousMode = 1,
}HIDS_operatingMode_t;

typedef enum
{
	HIDS_INTPinActiveHigh  = 0,
	HIDS_INTPinActiveLow   = 1,
}HIDS_DRDY_H_L_t;

typedef enum
{
	HIDS_INTPinPushPull  = 0,
	HIDS_INTPinOpenDrain = 1,
}HIDS_PP_OD_t;

typedef enum
{
	HIDS_INTPinDisable  = 0,
	HIDS_INTPinEnable   = 1,
}HIDS_EN_t;

//================================================================================
/* Class definitions */
class WSEN_HIDS
{
public:
	WSEN_HIDS(I2C_HandleTypeDef* _i2c_handler);
	WSEN_HIDS(I2C_HandleTypeDef* _i2c_handler, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	
	HAL_StatusTypeDef getDeviceID(uint8_t *device_ID);
	HAL_StatusTypeDef setOutputDataRate(HIDS_outputDataRate_t odr);
	HAL_StatusTypeDef setAvgMode(HIDS_TemAvgMode_t tempAvg, HIDS_HumAvgMode_t humAvg);
	HAL_StatusTypeDef reBootMem();
	HAL_StatusTypeDef heater(HIDS_heater_t mode);
	HAL_StatusTypeDef setDataReadyINTPin(HIDS_DRDY_H_L_t OutputLevel, HIDS_PP_OD_t Mode, HIDS_EN_t able);

	HAL_StatusTypeDef setOperatingMode(HIDS_operatingMode_t operatingMode);

	HAL_StatusTypeDef getRawData(int16_t& rawHumi, int16_t& rawTemp);  // Pass by reference
	HAL_StatusTypeDef getHumiRawData(int16_t& rawHumi);
	HAL_StatusTypeDef getTempRawData(int16_t& rawTemp);

	HAL_StatusTypeDef getHumiAndTemp(int16_t& humi, int16_t& temper);  // Pass by reference
	HAL_StatusTypeDef getTemp(int16_t& temper);
	HAL_StatusTypeDef getHumi(int16_t& humi);

	//inline HAL_StatusTypeDef ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf);

private:
	// I2C Interface
	I2C_HandleTypeDef* i2c_handler;
	const uint16_t i2cMemAddSize = I2C_MEMADD_SIZE_8BIT;
	GPIO_Pin_t DRDY_pin; // DRDY pin
	HIDS_operatingMode_t opMode;
	
	// Humidity Calibration Value
	float H0_rH;
	float H1_rH;
	int16_t H0_T0_OUT;
	int16_t H1_T0_OUT;

	// Temperature Calibration Value
	float T0_degC;
	float T1_degC;
	int16_t T0_OUT;
	int16_t T1_OUT;

	// ReadReg(): Read register via I2C
	// - regAddr: The address of the register to be read
	// - numBytesToRead: The number of bytes to be read
	// - buf: A pointer to the buffer that records the read data. Note: the array size should be larger than numBytesToRead.
	inline HAL_StatusTypeDef ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf);

	// WriteReg(): Write register via I2C
	// - regAddr: The address of the register to be written
	// - numBytesToRead: The number of bytes to be written
	// - buf: A pointer to a buffer that records the data to be written
	inline HAL_StatusTypeDef WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data);

	//readCalibrationValue(): Read calibration values of the sensor;
	HAL_StatusTypeDef readCalibrationValue();
};


#endif


































