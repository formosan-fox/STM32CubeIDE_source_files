#ifndef __PRESSURE_WSEN_PADS_H__
#define __PRESSURE_WSEN_PADS_H__

/* Device name: ABSOLUTE PRESSURE SENSOR WSEN-PADS
 * Order code: 2511020213301
 * Datasheet: https://www.we-online.com/components/products/datasheet/2511020213301.pdf
 * User Manual: https://www.we-online.com/components/products/manual/2511020213301_WSEN-PADS%202511020213301%20Manual_rev2.2.pdf
 * Reference:
 *   - https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_PADS_2511020213301
 *   - https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/examples/WSEN_PADS
 * Author: Zhi-Kai Xu
 */

#include "stdint.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>



/* PADS 2511020213301 DEVICE_ID */
#define PADS_DEVICE_ID_VALUE        0xB3     /**< Device ID of PADS 2511020213301 Sensor */

/* Available PADS I2C slave addresses */
#define PADS_ADDRESS_I2C_0          0x5C << 1    /**< when SAO of PADS is connected to ground */
#define PADS_ADDRESS_I2C_1          0x5D << 1    /**< when SAO of PADS is connected to positive supply voltage */

/* Register address definitions */
#define PADS_INT_CFG_REG            0x0B     /**< Interrupt configuration register */
#define PADS_THR_P_L_REG            0x0C     /**< Pressure threshold LSB register */
#define PADS_THR_P_H_REG            0x0D     /**< Pressure threshold MSB register */
#define PADS_INTERFACE_CTRL_REG     0x0E     /**< Interface control register */
#define PADS_DEVICE_ID_REG          0x0F     /**< Device ID register */
#define PADS_CTRL_1_REG             0x10     /**< Control register 1 */
#define PADS_CTRL_2_REG             0x11     /**< Control register 2 */
#define PADS_CTRL_3_REG             0x12     /**< Control register 3 */
#define PADS_FIFO_CTRL_REG          0x13     /**< FIFO control register */
#define PADS_FIFO_WTM_REG           0x14     /**< FIFO threshold register */
#define PADS_REF_P_L_REG            0x15     /**< Reference pressure LSB value register */
#define PADS_REF_P_H_REG            0x16     /**< Reference pressure MSB value register */
#define PADS_OPC_P_L_REG            0x18     /**< Pressure offset LSB value register */
#define PADS_OPC_P_H_REG            0x19     /**< Pressure offset MSB value register */
#define PADS_INT_SOURCE_REG         0x24     /**< Interrupt source register */
#define PADS_FIFO_STATUS1_REG       0x25     /**< FIFO status register 1 */
#define PADS_FIFO_STATUS2_REG       0x26     /**< FIFO status register 2 */
#define PADS_STATUS_REG             0x27     /**< Status register */
#define PADS_DATA_P_XL_REG          0x28     /**< Pressure output LSB value register */
#define PADS_DATA_P_L_REG           0x29     /**< Pressure output MID value register */
#define PADS_DATA_P_H_REG           0x2A     /**< Pressure output MSB value register */
#define PADS_DATA_T_L_REG           0x2B     /**< Temperature output LSB value register */
#define PADS_DATA_T_H_REG           0x2C     /**< Temperature output MSB value register */
#define PADS_FIFO_DATA_P_XL_REG     0x78     /**< Pressure LSB data in FIFO buffer */
#define PADS_FIFO_DATA_P_L_REG      0x79     /**< Pressure MID data in FIFO buffer */
#define PADS_FIFO_DATA_P_H_REG      0x7A     /**< Pressure MSB data in FIFO buffer */
#define PADS_FIFO_DATA_T_L_REG      0x7B     /**< Temperature LSB data in FIFO buffer */
#define PADS_FIFO_DATA_T_H_REG      0x7C     /**< Temperature MSB data in FIFO buffer */

/* Misc. defines */
#define PADS_FIFO_BUFFER_SIZE     128

/* Register type definitions */

/**
 * @brief Control register 1
 * Address 0x0F
 * Type  R/W
 * Default value: 0x00
 *
 *    ODR2  | ODR1  | ODR0   | Pressure/Temperature output data rate (Hz)
 * ---------------- ----------------------------------------------------
 *     0    |  0    |  0     |           Single conversion
 *     0    |  0    |  1     |                 1
 *     0    |  1    |  0     |                 10
 *     0    |  1    |  1     |                 25
 *     1    |  0    |  0     |                 50
 *     1    |  0    |  1     |                 75
 *     1    |  1    |  0     |                 100
 *     1    |  1    |  1     |                 200
 *
 *  -------------------------------------------------------------------
 *
 *  EN_LPFP  |   LPFP_CFG     |    LPF2 status         | Device Bandwidth | Samples to be discarded
 * --------------------------------------------------------------------------------------------------
 *      0    | x (don't care) |  Disabled/reset filter |      ODR/2       |          0
 *      1    |     0          |  Enabled               |      ODR/9       |          2
 *      1    |     1          |  Enabled               |      ODR/20      |          2
 */
typedef struct
{
	uint8_t notUsed01 : 1;            /**< This bit must be set to 0 for proper operation of the device */
	uint8_t blockDataUpdate : 1;      /**< BDU: Block data update. 0 - continuous update; 1 - output registers are not updated until both MSB and LSB have been read */
	uint8_t lowPassFilterConfig : 1;  /**< LPFP_CFG: Configure low pass filter for pressure data */
	uint8_t enLowPassFilter : 1;      /**< EN_LPFP: Enable low pass filter for pressure data */
	uint8_t outputDataRate : 3;       /**< ODR[2:0]: Output data rate. Default '000' */
	uint8_t notUsed02 : 1;            /**< This bit must be set to 0 for proper operation of the device */
} PADS_ctrl1_t;


/**
 * @brief Control register 2
 * Address 0x11
 * Type  R/W
 * Default value: 0x10
 */
typedef struct
{
	uint8_t oneShotBit : 1;         /**< ONE_SHOT: 0: Normal operation; 1: Start single conversion measurement */
	uint8_t lowNoiseMode : 1;       /**< LOW_NOISE_EN: Enables low noise mode (used only if ODR is lower than 100 Hz). Default value: 0 (0: low-power mode; 1: low-noise mode) */
	uint8_t softwareReset : 1;      /**< SWRESET: Software reset. 0: normal mode; 1: SW reset;  Self-clearing upon completion */
	uint8_t notUsed01 : 1;          /**< This bit must be set to 0 for proper operation of the device */
	uint8_t autoAddIncr : 1;        /**< IF_ADD_INC: Register address automatically incremented during a multiple byte access with I2C interface. Default value 1 (0: disable; 1: enable) */
	uint8_t openDrainOnINTPin : 1;  /**< PP_OD: Push-pull/open-drain selection on interrupt pad. Default value: 0 (0: push-pull; 1: open-drain) */
	uint8_t intActiveLevel : 1;     /**< INT_H_L: Interrupt active high, active low. Default value: 0 (0: active high; 1: active low) */
	uint8_t boot : 1;               /**< BOOT: Reboot memory content. 0: normal mode; 1: reboot memory content. Self-clearing upon completion */
} PADS_ctrl2_t;


/**
 * @brief Control register 3
 * Address 0x12
 * Type  R/W
 * Default value: 0x00
 *
 *             Interrupt configurations
 *     INT_S1     |   INT_S0    |   INT pin configuration
 *   ------------------------------------------------------
 *        0       |      0      |   Data signal (in order of priority: DRDY or INT_F_WTM or INT_F_OVR or INT_F_FULL)
 *        0       |      1      |   Pressure high event
 *        1       |      0      |   Pressure low event
 *        1       |      1      |   Pressure low or high event
 */
typedef struct
{
	uint8_t intEventCtrl : 2;       /**< INT_S: Data signal on INT pad control bits: Default value: 00 */
	uint8_t dataReadyInt : 1;       /**< DRDY: Data-ready signal on INT pin. Default value: 0 (0: disable; 1: enable) */
	uint8_t fifoOverrunInt : 1;     /**< INT_F_OVR: Enable FIFO overrun interrupt. Default value: 0 (0: disable; 1: enable) */
	uint8_t fifoThresholdInt : 1;   /**< INT_F_WTM: Enable FIFO threshold (watermark) interrupt. Default value: 0 (0: disable; 1: enable) */
	uint8_t fifoFullInt : 1;        /**< INT_F_FULL: Enable FIFO full interrupt. Default value: 0 (0: disable; 1: enable) */
	uint8_t notUsed01 : 2;          /**< These 2 bits must be set to 0 for proper operation of the device */
} PADS_ctrl3_t;

/**
 * @brief Status register
 * Address 0x27
 * Type  R
 * Default value: Output; 0x00
 */
typedef struct
{
    uint8_t presDataAvailable : 1;  /**< P_DA: Pressure data available. (0: Pressure sample not yet available; 1: A new pressure sample is available) */
    uint8_t tempDataAvailable : 1;  /**< T_DA: Temperature data available. (0: Temperature sample not yet available; 1: A new temperature sample is available) */
    uint8_t notUsed01 : 2;          /**< These 2 bits must be set to 0 for proper operation of the device */
    uint8_t presDataOverrun : 1;    /**< P_OR: Pressure data overrun. (0: No overrun; 1: Pressure data overwritten) */
    uint8_t tempDataOverrun : 1;    /**< T_OR: Temperature data overrun. (0: No overrun; 1: Temperature data overwritten) */
    uint8_t notUsed02 : 2;          /**< These 2 bits must be set to 0 for proper operation of the device */
} PADS_status_t;



/* Enum definitions */
typedef enum
{
	PADS_disable = 0,
	PADS_enable = 1
} PADS_state_t;


typedef enum
{
	PADS_outputDataRatePowerDown = 0,     /**< single conversion / power down */
	PADS_outputDataRate1Hz = 1,           /**< 1Hz */
	PADS_outputDataRate10Hz = 2,          /**< 10Hz */
	PADS_outputDataRate25Hz = 3,          /**< 25Hz */
	PADS_outputDataRate50Hz = 4,          /**< 50Hz */
	PADS_outputDataRate75Hz = 5,          /**< 75Hz */
	PADS_outputDataRate100Hz = 6,         /**< 100Hz */
	PADS_outputDataRate200Hz = 7          /**< 200Hz */
} PADS_outputDataRate_t;

typedef enum
{
    PADS_lowPower = 0,      /**< Low power mode */
    PADS_lowNoise = 1,      /**< Low noise mode */
} PADS_powerMode_t;

/* Class definitions */
class Pressure_WSEN_PADS
{
public:
    // Construtor
    Pressure_WSEN_PADS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO);

    // Communication test
    bool isCommunicationReady();

    // Setting
    bool init();

    // Usage example
    void singleConversionModeExample();		// TODO: bugs
    void continuousModeExampleLoop();

    HAL_StatusTypeDef softReset();
    HAL_StatusTypeDef getSoftResetState(PADS_state_t *swReset);
    
    HAL_StatusTypeDef enableAutoIncrement();
    HAL_StatusTypeDef disableAutoIncrement();
    HAL_StatusTypeDef isAutoIncrementEnabled(PADS_state_t *ai);
    bool isAutoIncrementEnabled();

    HAL_StatusTypeDef enableBlockDataUpdate();
    HAL_StatusTypeDef disableBlockDataUpdate();
    HAL_StatusTypeDef isBlockDataUpdateEnabled(PADS_state_t *bdu);
    bool isBlockDataUpdateEnabled();

    HAL_StatusTypeDef setOutputDataRate(PADS_outputDataRate_t odr);
    HAL_StatusTypeDef getOutputDataRate(PADS_outputDataRate_t* odr);

    HAL_StatusTypeDef PADS_setPowerMode(PADS_powerMode_t mode);
    HAL_StatusTypeDef PADS_getPowerMode(PADS_powerMode_t *mode);

    HAL_StatusTypeDef enableOneShot();
    HAL_StatusTypeDef disableOneShot();
    HAL_StatusTypeDef isOneShotEnabled(PADS_state_t *oneShot);
    bool isOneShotEnabled();

    HAL_StatusTypeDef isPressureDataReady(PADS_state_t *state);
    bool isPressureDataReady();
    HAL_StatusTypeDef isTemperatureDataReady(PADS_state_t *state);
    bool isTemperatureDataReady();

    HAL_StatusTypeDef getRawPressure(int32_t *rawPres);
    HAL_StatusTypeDef getRawTemperature(int16_t *rawTemp);

    HAL_StatusTypeDef getPressure_float(float *presskPa);
    HAL_StatusTypeDef getDifferentialPressure_float(float *presskPa);
    HAL_StatusTypeDef getTemperature_float(float *tempDegC);

private:
    // I2C Interface
    I2C_HandleTypeDef* i2c_handler;
    uint8_t i2c_device_addr;
    const uint16_t i2cMemAddSize = I2C_MEMADD_SIZE_8BIT;

    // Communication test
    HAL_StatusTypeDef getDeviceID(uint8_t *device_ID);

    float convertPressure_float(const int32_t &rawPres);
    float convertDifferentialPressure_float(const int32_t &rawPres);
    int32_t convertPressure_int(const int32_t &rawPres);
    int32_t convertDifferentialPressure_int(const int32_t &rawPres);

    inline HAL_StatusTypeDef PADS_ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf);
    inline HAL_StatusTypeDef PADS_WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data);
};

#endif
