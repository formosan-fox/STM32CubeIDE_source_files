#ifndef __IMU_6DIM_WSEN_ISDS_H__
#define __IMU_6DIM_WSEN_ISDS_H__

/* Device name: WSEN-ISDS INERTIAL MEASUREMENT UNIT
 * Order code: 2536030320001
 * Datasheet: https://www.we-online.com/components/products/datasheet/2536030320001.pdf
 * User Manual: https://www.we-online.com/components/products/manual/2536030320001_Manual_UM_WSEN-ISDS_2536030320001_rev1.1.pdf
 * Reference:
 *   - https://github.com/WurthElektronik/Sensors-SDK_STM32/blob/main/SensorsSDK/WSEN_ISDS_2536030320001
 *   - https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/examples/WSEN_ISDS_SPI
 * Author: Zhi-Kai Xu
 */

#include "platform.h"

/* ISDS 2536030320001 DEVICE_ID */
#define ISDS_DEVICE_ID_VALUE                  0x6A      /**< This is the expected answer when requesting the ISDS_DEVICE_ID_REG */

/* Available ISDS I2C slave addresses */
#define ISDS_ADDRESS_I2C_0                    0x6A << 1     /**< When SAO of ISDS is connected to ground */
#define ISDS_ADDRESS_I2C_1                    0x6B << 1     /**< When SAO of ISDS is connected to positive supply voltage */

/* Register address definitions */
#define ISDS_FIFO_CTRL_1_REG                  0x06      /**< FIFO configuration register 1 */
#define ISDS_FIFO_CTRL_2_REG                  0x07      /**< FIFO configuration register 2 */
#define ISDS_FIFO_CTRL_3_REG                  0x08      /**< FIFO configuration register 3 */
#define ISDS_FIFO_CTRL_4_REG                  0x09      /**< FIFO configuration register 4 */
#define ISDS_FIFO_CTRL_5_REG                  0x0A      /**< FIFO configuration register 5 */
#define ISDS_DRDY_PULSE_CFG_REG               0x0B      /**< Data ready configuration register */
#define ISDS_INT0_CTRL_REG                    0x0D      /**< INT0 pin control */
#define ISDS_INT1_CTRL_REG                    0x0E      /**< INT1 pin control */
#define ISDS_DEVICE_ID_REG                    0x0F      /**< Device ID register */
#define ISDS_CTRL_1_REG                       0x10      /**< Control register 1 (linear acceleration sensor) */
#define ISDS_CTRL_2_REG                       0x11      /**< Control register 2 (angular rate sensor) */
#define ISDS_CTRL_3_REG                       0x12      /**< Control register 3 */
#define ISDS_CTRL_4_REG                       0x13      /**< Control register 4 */
#define ISDS_CTRL_5_REG                       0x14      /**< Control register 5 */
#define ISDS_CTRL_6_REG                       0x15      /**< Control register 6 (angular rate sensor) */
#define ISDS_CTRL_7_REG                       0x16      /**< Control register 7 (angular rate sensor) */
#define ISDS_CTRL_8_REG                       0x17      /**< Control register 8 (linear acceleration sensor) */
#define ISDS_CTRL_9_REG                       0x18      /**< Control register 9 (linear acceleration sensor) */
#define ISDS_CTRL_10_REG                      0x19      /**< Control register 10 */
#define ISDS_WAKE_UP_EVENT_REG                0x1B      /**< Wake-up interrupt source register */
#define ISDS_TAP_EVENT_REG                    0x1C      /**< Tap source register */
#define ISDS_6D_EVENT_REG                     0x1D      /**< 6D orientation source register */
#define ISDS_STATUS_REG                       0x1E      /**< Status data register */
#define ISDS_OUT_TEMP_L_REG                   0x20      /**< Temperature output value LSB */
#define ISDS_OUT_TEMP_H_REG                   0x21      /**< Temperature output value MSB */
#define ISDS_X_OUT_L_GYRO_REG                 0x22      /**< Angular rate output value X (pitch) LSB */
#define ISDS_X_OUT_H_GYRO_REG                 0x23      /**< Angular rate output value X (pitch) MSB */
#define ISDS_Y_OUT_L_GYRO_REG                 0x24      /**< Angular rate output value Y (roll) LSB */
#define ISDS_Y_OUT_H_GYRO_REG                 0x25      /**< Angular rate output value Y (roll) MSB */
#define ISDS_Z_OUT_L_GYRO_REG                 0x26      /**< Angular rate output value Z (yaw) LSB */
#define ISDS_Z_OUT_H_GYRO_REG                 0x27      /**< Angular rate output value Z (yaw) MSB */
#define ISDS_X_OUT_L_ACC_REG                  0x28      /**< Linear acceleration output value X LSB */
#define ISDS_X_OUT_H_ACC_REG                  0x29      /**< Linear acceleration output value X MSB */
#define ISDS_Y_OUT_L_ACC_REG                  0x2A      /**< Linear acceleration output value Y LSB */
#define ISDS_Y_OUT_H_ACC_REG                  0x2B      /**< Linear acceleration output value Y MSB */
#define ISDS_Z_OUT_L_ACC_REG                  0x2C      /**< Linear acceleration output value Z LSB */
#define ISDS_Z_OUT_H_ACC_REG                  0x2D      /**< Linear acceleration output value Z MSB */
#define ISDS_FIFO_STATUS_1_REG                0x3A      /**< FIFO status register 1 */
#define ISDS_FIFO_STATUS_2_REG                0x3B      /**< FIFO status register 2 */
#define ISDS_FIFO_STATUS_3_REG                0x3C      /**< FIFO status register 3 */
#define ISDS_FIFO_STATUS_4_REG                0x3D      /**< FIFO status register 4 */
#define ISDS_FIFO_DATA_OUT_L_REG              0x3E      /**< FIFO data output register LSB */
#define ISDS_FIFO_DATA_OUT_H_REG              0x3F      /**< FIFO data output register MSB */
#define ISDS_FUNC_SRC_1_REG                   0x53      /**< Tilt interrupt source register */
#define ISDS_TAP_CFG_REG                      0x58      /**< Enables interrupt and inactivity functions, configuration of filtering and tap recognition functions */
#define ISDS_TAP_THS_6D_REG                   0x59      /**< Portrait/landscape position and tap function threshold register */
#define ISDS_INT_DUR2_REG                     0x5A      /**< Tap recognition function setting register */
#define ISDS_WAKE_UP_THS_REG                  0x5B      /**< Single and double-tap function threshold register */
#define ISDS_WAKE_UP_DUR_REG                  0x5C      /**< Free-fall, wake-up and sleep mode functions duration setting register */
#define ISDS_FREE_FALL_REG                    0x5D      /**< Free-fall function duration setting register */
#define ISDS_MD1_CFG_REG                      0x5E      /**< Functions routing on INT0 register */
#define ISDS_MD2_CFG_REG                      0x5F      /**< Functions routing on INT1 register */
#define ISDS_X_OFS_USR_REG                    0x73      /**< Accelerometer X-axis user offset correction */
#define ISDS_Y_OFS_USR_REG                    0x74      /**< Accelerometer Y-axis user offset correction */
#define ISDS_Z_OFS_USR_REG                    0x75      /**< Accelerometer Z-axis user offset correction */


/* Register type definitions */

/**
 * @brief ISDS_FIFO_CTRL_1_REG
 * Address 0x06
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t fifoThresholdLsb : 8;         /**< FIFO threshold level setting LSB. Default value: 0. Watermark flag rises when the number of bytes written to FIFO after the next write is greater than or equal to the threshold level. Minimum resolution for the FIFO is 1 LSB = 2 bytes (1 word) in FIFO. */
} ISDS_fifoCtrl1_t;

/**
 * @brief ISDS_FIFO_CTRL_2_REG
 * Address 0x07
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t fifoThresholdMsb : 3;         /**< FIFO threshold level setting MSB. Default value: 0. Watermark flag rises when the number of bytes written to FIFO after the next write is greater than or equal to the threshold level. Minimum resolution for the FIFO is 1LSB = 2 bytes (1 word) in FIFO. */
    uint8_t enFifoTemperature : 1;        /**< Enables the temperature data storage in FIFO. Default: 0. 0: temperature not included in FIFO; 1: temperature included in FIFO. */
    uint8_t notUsed01 : 4;                /**< These bits must be set to 0 for proper operation of the device. */
} ISDS_fifoCtrl2_t;

/**
 * @brief ISDS_FIFO_CTRL_3_REG
 * Address 0x08
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t fifoAccDecimation : 3;        /**< Accelerometer FIFO (second data set) decimation setting. Default: 0. See ISDS_fifoDecimation_t. */
    uint8_t fifoGyroDecimation : 3;       /**< Gyro FIFO (first data set) decimation setting. Default: 0. See ISDS_fifoDecimation_t. */
    uint8_t notUsed01 : 2;                /**< These bits must be set to 0 for proper operation of the device. */
} ISDS_fifoCtrl3_t;

/**
 * @brief ISDS_FIFO_CTRL_4_REG
 * Address 0x09
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t fifoThirdDecimation : 3;      /**< Third FIFO data set decimation setting. Default: 0. See ISDS_fifoDecimation_t. */
    uint8_t fifoFourthDecimation : 3;     /**< Fourth FIFO data set decimation setting. Default: 0. See ISDS_fifoDecimation_t. */
    uint8_t enOnlyHighData : 1;           /**< 8-bit data storage in FIFO. Default: 0. 0: disable MSB only memorization in FIFO for XL and Gyro; 1: enable MSB only memorization in FIFO for XL and Gyro in FIFO. */
    uint8_t enStopOnThreshold : 1;        /**< Enable FIFO threshold level use. Default value: 0. 0: FIFO depth is not limited; 1: FIFO depth is limited to threshold level. */
} ISDS_fifoCtrl4_t;

/**
 * @brief ISDS_FIFO_CTRL_5_REG
 * Address 0x0A
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t fifoMode : 3;                 /**< FIFO mode. See ISDS_fifoMode_t. */
    uint8_t fifoOdr : 4;                  /**< FIFO output data rate. See ISDS_fifoOutputDataRate_t. */
    uint8_t notUsed01 : 1;                /**< This bit must be set to 0 for proper operation of the device. */
} ISDS_fifoCtrl5_t;

/**
 * @brief ISDS_DRDY_PULSE_CFG_REG
 * Address 0x0B
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t notUsed01 : 7;                /**< These bits must be set to 0 for proper operation of the device. */
    uint8_t enDataReadyPulsed : 1;        /**< Enable pulsed data-ready mode. Default value: 0. 0: data-ready latched mode. Returns to 0 only after output data has been read; 1: data-ready pulsed mode. The data-ready pulses are 75 μs long. */
} ISDS_dataReadyPulseCfg_t;

/**
 * @brief ISDS_INT0_CTRL_REG
 * Address 0x0D
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t int0AccDataReady : 1;         /**< Accelerometer data-ready on INT0 pin. Default value: 0. */
    uint8_t int0GyroDataReady : 1;        /**< Gyroscope data-ready on INT0 pin. Default value: 0. */
    uint8_t int0Boot : 1;                 /**< Boot status on INT0 pin. Default value: 0. */
    uint8_t int0FifoThreshold : 1;        /**< FIFO threshold interrupt on INT0 pin. Default value: 0. */
    uint8_t int0FifoOverrun : 1;          /**< FIFO overrun interrupt on INT0 pin. Default value: 0. */
    uint8_t int0FifoFull : 1;             /**< FIFO full interrupt on INT0 pin. Default value: 0. */
    uint8_t notUsed01 : 2;                /**< These bits must be set to 0 for proper operation of the device. */
} ISDS_int0Ctrl_t;

/**
 * @brief ISDS_INT1_CTRL_REG
 * Address 0x0E
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t int1AccDataReady : 1;         /**< Accelerometer data-ready on INT1 pin. Default value: 0. */
    uint8_t int1GyroDataReady : 1;        /**< Gyroscope data-ready on INT1 pin. Default value: 0. */
    uint8_t int1TempDataReady : 1;        /**< Temperature data-ready on INT1 pin. Default value: 0. */
    uint8_t int1FifoThreshold : 1;        /**< FIFO threshold interrupt on INT1 pin. Default value: 0. */
    uint8_t int1FifoOverrun : 1;          /**< FIFO overrun interrupt on INT1 pin. Default value: 0. */
    uint8_t int1FifoFull : 1;             /**< FIFO full interrupt on INT1 pin. Default value: 0. */
    uint8_t notUsed01 : 2;                /**< These bits must be set to 0 for proper operation of the device. */
} ISDS_int1Ctrl_t;

/**
 * @brief ISDS_CTRL_1_REG
 * Address 0x10
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t accAnalogBandwidth : 1;       /**< Accelerometer analog chain bandwidth selection (only for accelerometer ODR ≥ 1.67 kHz). See ISDS_accAnalogChainBandwidth_t. */
    uint8_t accDigitalBandwidth : 1;      /**< Accelerometer digital LPF (LPF1) bandwidth selection. See ISDS_accDigitalLpfBandwidth_t. */
    uint8_t accFullScale : 2;             /**< Accelerometer full-scale selection. Default value: 0. See ISDS_accFullScale_t. */
    uint8_t accOutputDataRate : 4;        /**< Output data rate and power mode selection. Default value: 0. See ISDS_accOutputDataRate_t. */
} ISDS_ctrl1_t;

/**
 * @brief ISDS_CTRL_2_REG
 * Address 0x11
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t notUsed01 : 1;                /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t gyroFullScale : 3;            /**< Gyroscope full-scale selection. Default value: 0. See ISDS_gyroFullScale_t. */
    uint8_t gyroOutputDataRate : 4;       /**< Output data rate and power mode selection. Default value: 0. See ISDS_gyroOutputDataRate_t. */
} ISDS_ctrl2_t;

/**
 * @brief ISDS_CTRL_3_REG
 * Address 0x12
 * Type  R/W
 * Default value: 0x04
 */
typedef struct
{
    uint8_t softReset : 1;                /**< Software reset. 0: normal mode; 1: SW reset; Self-clearing upon completion. */
    uint8_t notUsed01 : 1;                /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t autoAddIncr : 1;              /**< Register address automatically incremented during a multiple byte access with I2C/SPI interface. Default: 1. 0: disable; 1: enable. */
    uint8_t spiMode : 1;                  /**< SPI serial interface mode. 0: 4-wire interface; 1: 3-wire interface. See ISDS_spiMode_t.*/
    uint8_t intPinConf : 1;               /**< Push-pull/open-drain selection on INT0 and INT1 pads. Default value: 0. 0: push-pull mode; 1: open-drain mode. See ISDS_interruptPinConfig_t. */
    uint8_t intActiveLevel : 1;           /**< Interrupt activation level. Default value: 0. 0: interrupt output pads active high; 1: interrupt output pads active low. See ISDS_interruptActiveLevel_t. */
    uint8_t blockDataUpdate : 1;          /**< Block data update. 0: continuous update; 1: output registers are not updated until MSB and LSB have been read. */
    uint8_t boot : 1;                     /**< Set this bit to 1 to initiate boot sequence. 0: normal mode; 1: Execute boot sequence. Self-clearing upon completion. */
} ISDS_ctrl3_t;

/**
 * @brief ISDS_CTRL_4_REG
 * Address 0x13
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t notUsed01 : 1;                  /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t enGyroLPF1 : 1;                 /**< Enable gyroscope digital LPF1. 0: disabled; 1: enabled. */
    uint8_t i2cDisable : 1;                 /**< Disable I2C interface. Default value: 0. 0: both I2C and SPI enabled; 1: I2C disabled, SPI only. */
    uint8_t dataReadyMask : 1;              /**< Enables masking of the accelerometer and gyroscope data-ready signals until the settling of the sensor filters is completed. Default value: 0. 0: Masking disabled; 1: Masking enabled. */
    uint8_t dataEnableDataReadyOnInt0 : 1;  /**< Data enable (DEN) data ready signal on INT0 pad. Default value: 0. 0: disabled; 1: enabled. */
    uint8_t int1OnInt0 : 1;                 /**< All interrupt signals available on INT0 pad enable. Default value: 0. 0: interrupt signals divided between INT0 and INT1 pads; 1: all interrupt signals in logic or on INT0 pad. */
    uint8_t enGyroSleepMode : 1;            /**< Gyroscope sleep mode enable. Default value: 0. 0: disabled; 1: enabled. */
    uint8_t dataEnableExtendToAcc : 1;      /**< Extend data enable (DEN) functionality to accelerometer sensor. Default value: 0. 0: disabled; 1: enabled. */
} ISDS_ctrl4_t;

/**
 * @brief ISDS_CTRL_5_REG
 * Address 0x14
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t accSelfTest : 2;              /**< Accelerometer self-test enable. Default value: 0. See ISDS_accSelfTestMode_t. */
    uint8_t gyroSelfTest : 2;             /**< Gyroscope self-test enable. Default value: 0. See ISDS_gyroSelfTestMode_t. */
    uint8_t dataEnableActiveLevel : 1;    /**< Data enable (DEN) active level configuration. Default value: 0. 0: active low; 1: active high. */
    uint8_t rounding : 3;                 /**< Circular burst-mode (rounding) read from output registers through the primary interface. Default value: 0. See ISDS_roundingPattern_t. */
} ISDS_ctrl5_t;

/**
 * @brief ISDS_CTRL_6_REG
 * Address 0x15
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t gyroLowPassFilterType : 2;            /**< Gyroscope low-pass filter (LPF1) bandwidth selection. See ISDS_gyroLPF_t. */
    uint8_t notUsed01 : 1;                        /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t userOffsetsWeight : 1;                /**< Weight of accelerometer user offset bits in registers ISDS_X_OFS_USR_REG, ISDS_Y_OFS_USR_REG, ISDS_Z_OFS_USR_REG. 0: 2^-10 g/LSB; 1: 2^-6 g/LSB. */
    uint8_t accHighPerformanceModeDisable : 1;    /**< High-performance operating mode disable for accelerometer. Default value: 0. 0: high-performance operating mode enabled; 1: high-performance operating mode disabled. */
    uint8_t dataEnableTriggerMode : 3;            /**< Data enable (DEN) trigger mode. See ISDS_dataEnableTriggerMode_t. */
} ISDS_ctrl6_t;

/**
 * @brief ISDS_CTRL_7_REG
 * Address 0x16
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t notUsed01 : 2;                        /**< These bits must be set to 0 for proper operation of the device. */
    uint8_t enRounding : 1;                       /**< Source register rounding function on ISDS_WAKE_UP_EVENT_REG, ISDS_TAP_EVENT_REG, ISDS_6D_EVENT_REG, ISDS_STATUS_REG and ISDS_FUNC_SRC_1_REG registers in the primary interface. Default value: 0. 0: rounding disabled; 1: rounding enabled. */
    uint8_t notUsed02 : 1;                        /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t gyroDigitalHighPassCutoff : 2;        /**< Gyroscope digital HP filter cutoff selection. Default: 0. See ISDS_gyroDigitalHighPassCutoff_t. */
    uint8_t gyroDigitalHighPassEnable : 1;        /**< Gyroscope digital high-pass filter enable. The filter is enabled only if the gyro is in HP mode. Default value: 0. 0: HPF disabled; 1: HPF enabled. */
    uint8_t gyroHighPerformanceModeDisable : 1;   /**< High-performance operating mode disable for gyroscope. Default: 0. 0: high-performance operating mode enabled; 1: high-performance operating mode disabled. */
} ISDS_ctrl7_t;

/**
 * @brief ISDS_CTRL_8_REG
 * Address 0x17
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t en6dLowPass: 1;                 /**< LPF2 on 6D function selection. */
    uint8_t notUsed01 : 1;                  /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t enAccHighPassSlopeFilter : 1;   /**< Accelerometer slope filter / high-pass filter selection. 0: the low-pass path of the composite filter block is selected; 1: the high-pass path of the composite filter block is selected. */
    uint8_t inputComposite : 1;             /**< Composite filter input selection. See ISDS_inputCompositeFilter_t. Default: 0. 0: ODR/2 low pass filtered sent to composite filter (default); 1: ODR/4 low pass filtered sent to composite filter. */
    uint8_t highPassFilterRefMode : 1;      /**< Enable HP filter reference mode (when enabled, the first output data has to be discarded). Default value: 0. 0: disabled; 1: enabled. */
    uint8_t accFilterConfig : 2;            /**< Accelerometer LPF2 and high-pass filter configuration and cutoff setting. See ISDS_accFilterConfig_t. */
    uint8_t enAccLowPass : 1;               /**< Accelerometer low-pass filter LPF2 selection. */
} ISDS_ctrl8_t;

/**
 * @brief ISDS_CTRL_9_REG
 * Address 0x18
 * Type  R/W
 * Default value: 0xE0
 */
typedef struct
{
    uint8_t notUsed01 : 4;                  /**< These bits must be set to 0 for proper operation of the device. */
    uint8_t dataEnableStampingSensor : 1;   /**< Data enable (DEN) stamping sensor selection. See ISDS_dataEnableStampingSensor_t. Default value: 0. 0: DEN pin info stamped in the gyroscope axis selected by bits [7:5]; 1: DEN pin info stamped in the accelerometer axis selected by bits [7:5]. */
    uint8_t dataEnableValueZ : 1;           /**< Data enable (DEN) value stored in LSB of Z-axis. Default value: 1. 0: DEN not stored in Z-axis LSB; 1: DEN stored in Z-axis LSB. */
    uint8_t dataEnableValueY : 1;           /**< Data enable (DEN) value stored in LSB of Y-axis. Default value: 1. 0: DEN not stored in Y-axis LSB; 1: DEN stored in Y-axis LSB. */
    uint8_t dataEnableValueX : 1;           /**< Data enable (DEN) value stored in LSB of X-axis. Default value: 1. 0: DEN not stored in X-axis LSB; 1: DEN stored in X-axis LSB. */
} ISDS_ctrl9_t;

/**
 * @brief ISDS_CTRL_10_REG
 * Address 0x19
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t notUsed01 : 2;                  /**< These bits must be set to 0 for proper operation of the device. */
    uint8_t enEmbeddedFunc : 1;             /**< Enable embedded functionalities (tilt). Default value: 0. 0: disable functionalities of embedded functions and accelerometer filters; 1: enable functionalities of embedded functions and accelerometer filters. */
    uint8_t enTiltCalculation : 1;          /**< Enable tilt calculation. */
    uint8_t notUsed02 : 1;                  /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t notUsed03 : 3;                  /**< These bits must be set to 0 for proper operation of the device. */
} ISDS_ctrl10_t;

/**
 * @brief ISDS_STATUS_REG
 * Address 0x1E
 * Type  R
 * Default value: 0x00
 */
typedef struct
{
    uint8_t accDataReady : 1;               /**< 1: New acceleration data available; 0: No new data available. */
    uint8_t gyroDataReady : 1;              /**< 1: New gyroscope data available; 0: No new data available. */
    uint8_t tempDataReady: 1;               /**< 1: New temperature data available; 0: No new data available. */
    uint8_t notUsed01 : 5;                  /**< These bits must be set to 0 for proper operation of the device. */
} ISDS_status_t;


/* Enum definitions */

typedef enum
{
	ISDS_I2C = 0,
	ISDS_SPI = 1
} ISDS_interface;

typedef enum
{
	ISDS_disable = 0,
	ISDS_enable = 1
} ISDS_state_t;

typedef enum
{
	ISDS_accFullScaleTwoG       = 0,
	ISDS_accFullScaleSixteenG   = 1,
	ISDS_accFullScaleFourG      = 2,
	ISDS_accFullScaleEightG     = 3,
	ISDS_accFullScaleInvalid    = 4
} ISDS_accFullScale_t;

typedef enum
{
	ISDS_accOdrOff      =  0,
	ISDS_accOdr12Hz5    =  1,
	ISDS_accOdr26Hz     =  2,
	ISDS_accOdr52Hz     =  3,
	ISDS_accOdr104Hz    =  4,
	ISDS_accOdr208Hz    =  5,
	ISDS_accOdr416Hz    =  6,
	ISDS_accOdr833Hz    =  7,
	ISDS_accOdr1k66Hz   =  8,
	ISDS_accOdr3k33Hz   =  9,
	ISDS_accOdr6k66Hz   = 10,
	ISDS_accOdr1Hz6     = 11
} ISDS_accOutputDataRate_t;

typedef enum
{
	ISDS_gyroFullScale250dps   = 0,
	ISDS_gyroFullScale125dps   = 1,
	ISDS_gyroFullScale500dps   = 2,
	ISDS_gyroFullScale1000dps  = 4,
	ISDS_gyroFullScale2000dps  = 6
} ISDS_gyroFullScale_t;

typedef enum
{
	ISDS_gyroOdrOff    =  0,
	ISDS_gyroOdr12Hz5  =  1,
	ISDS_gyroOdr26Hz   =  2,
	ISDS_gyroOdr52Hz   =  3,
	ISDS_gyroOdr104Hz  =  4,
	ISDS_gyroOdr208Hz  =  5,
	ISDS_gyroOdr416Hz  =  6,
	ISDS_gyroOdr833Hz  =  7,
	ISDS_gyroOdr1k66Hz =  8,
	ISDS_gyroOdr3k33Hz =  9,
	ISDS_gyroOdr6k66Hz = 10
} ISDS_gyroOutputDataRate_t;


/* Class definitions */

class IMU_6DIM_WSEN_ISDS
{
public:
    // Construtor
    IMU_6DIM_WSEN_ISDS(SPI_HandleTypeDef* _spi_handler, GPIO_TypeDef *_CS_PORT, uint16_t _CS_PIN);
    IMU_6DIM_WSEN_ISDS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO);

    // Initilizes the settings
    /** blockDataUpdate:    ISDS_enable / ISDS_disable
     *  accOutputDataRate: 	ISDS_accOdrOff / ISDS_accOdr12Hz5 / ISDS_accOdr26Hz / ISDS_accOdr52Hz / ISDS_accOdr104Hz / ISDS_accOdr208Hz /
	                        ISDS_accOdr416Hz / ISDS_accOdr833Hz / ISDS_accOdr1k66Hz / ISDS_accOdr3k33Hz / ISDS_accOdr6k66Hz / ISDS_accOdr1Hz6
     *  gyroOutputDataRate: ISDS_gyroOdrOff / ISDS_gyroOdr12Hz5 / ISDS_gyroOdr26Hz / ISDS_gyroOdr52Hz / ISDS_gyroOdr104Hz / ISDS_gyroOdr208Hz /
                            ISDS_gyroOdr416Hz / ISDS_gyroOdr833Hz / ISDS_gyroOdr1k66Hz / ISDS_gyroOdr3k33Hz / ISDS_gyroOdr6k66Hz
     *  accFullScale:       ISDS_accFullScaleTwoG / ISDS_accFullScaleSixteenG / ISDS_accFullScaleFourG / ISDS_accFullScaleEightG
     *  gyroFullScale:      ISDS_gyroFullScale125dps / ISDS_gyroFullScale250dps / ISDS_gyroFullScale500dps / ISDS_gyroFullScale1000dps /
     *                      ISDS_gyroFullScale2000dps
    */
    bool init(
    	    ISDS_state_t blockDataUpdate = ISDS_enable,
    	    ISDS_accOutputDataRate_t accOutputDataRate = ISDS_accOdr104Hz,
    	    ISDS_gyroOutputDataRate_t gyroOutputDataRate = ISDS_gyroOdr104Hz,
    	    ISDS_accFullScale_t accFullScale = ISDS_accFullScaleFourG,
    	    ISDS_gyroFullScale_t gyroFullScale = ISDS_gyroFullScale500dps);

    // Communication test
    bool isCommunicationReady();
    HAL_StatusTypeDef getDeviceID(uint8_t *device_ID);

    // Usage example
    void example_loop();

    // ISDS_CTRL_1_REG
    HAL_StatusTypeDef setAccFullScale(ISDS_accFullScale_t fullScale);
    HAL_StatusTypeDef getAccFullScale(ISDS_accFullScale_t *fullScale);
    uint8_t getAccFullScale();
    HAL_StatusTypeDef setAccOutputDataRate(ISDS_accOutputDataRate_t odr);
    HAL_StatusTypeDef getAccOutputDataRate(ISDS_accOutputDataRate_t *odr);
    uint8_t getAccOutputDataRate();

    // ISDS_CTRL_2_REG
    HAL_StatusTypeDef setGyroFullScale(ISDS_gyroFullScale_t fullScale);
    HAL_StatusTypeDef getGyroFullScale(ISDS_gyroFullScale_t *fullScale);
    uint8_t getGyroFullScale();
    HAL_StatusTypeDef setGyroOutputDataRate(ISDS_gyroOutputDataRate_t odr);
    HAL_StatusTypeDef getGyroOutputDataRate(ISDS_gyroOutputDataRate_t *odr);
    uint8_t getGyroOutputDataRate();

    // ISDS_CTRL_3_REG
    HAL_StatusTypeDef enableBlockDataUpdate();
    HAL_StatusTypeDef disableBlockDataUpdate();
    bool isBlockDataUpdateEnabled();    // TODO: error handler
    // TODO: auto increment

    // ISDS_STATUS_REG
    bool isAccelerationDataReady();     // TODO: error handler
    bool isGyroscopeDataReady();
    bool isTemperatureDataReady();

    // Accelerometer output
    HAL_StatusTypeDef getAccelerationX_float(float *xAcc);
    HAL_StatusTypeDef getAccelerationY_float(float *yAcc);
    HAL_StatusTypeDef getAccelerationZ_float(float *zAcc);
    float getAccelerationX_float();
    float getAccelerationY_float();
    float getAccelerationZ_float();

    // Gyroscope output
    HAL_StatusTypeDef getAngularRateX_float(float *xRate);
    HAL_StatusTypeDef getAngularRateY_float(float *yRate);
    HAL_StatusTypeDef getAngularRateZ_float(float *zRate);
    float getAngularRateX_float();
    float getAngularRateY_float();
    float getAngularRateZ_float();

    // Temperature sensor output
    HAL_StatusTypeDef getTemperature_float(float *temperature);
    float getTemperature_float();

private:
    // Interface
    ISDS_interface interface;

    // SPI Interface
    SPI_HandleTypeDef* spi_handler;
    GPIO_TypeDef* CS_PORT;
    uint16_t CS_PIN;

    // I2C Interface
    I2C_HandleTypeDef* i2c_handler;
    uint8_t i2c_device_addr;
    const uint16_t i2cMemAddSize = I2C_MEMADD_SIZE_8BIT;

    ISDS_accFullScale_t currentAccFullScale;
    ISDS_gyroFullScale_t currentGyroFullScale;

    float convertAcceleration_float(int16_t acc, ISDS_accFullScale_t fullScale);
    float convertAngularRate_float(int16_t rate, ISDS_gyroFullScale_t fullScale);

    inline HAL_StatusTypeDef ISDS_ReadReg(uint8_t regAddr, uint16_t numBytesToRead, uint8_t *buf);
    inline HAL_StatusTypeDef ISDS_WriteReg(uint8_t regAddr, uint16_t numBytesToWrite, uint8_t *data);
};


#endif
