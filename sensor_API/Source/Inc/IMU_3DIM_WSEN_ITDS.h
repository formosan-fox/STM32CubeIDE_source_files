#ifndef __IMU_3DIM_WSEN_ITDS_H__
#define __IMU_3DIM_WSEN_ITDS_H__

/* Device name: WSEN-ITDS 3 Axis Acceleration Sensor
 * Order code: 2533020201601
 * Datasheet: https://www.we-online.com/components/products/datasheet/2533020201601.pdf
 * User Manual: https://www.we-online.com/components/products/manual/2533020201601_WSEN-ITDS%202533020201601%20Manual_rev2.3.pdf
 * Reference:
 *   - https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_ITDS_2533020201601
 *   - https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/examples/WSEN_ITDS
 * Author: Zhi-Kai Xu
 */

#include "platform.h"

/* ITDS 2533020201601 DEVICE_ID */

#define ITDS_DEVICE_ID_VALUE              0x44      /**< This is the expected answer when requesting the ITDS_DEVICE_ID_REG */


/* Available ITDS I2C slave addresses */

#define ITDS_ADDRESS_I2C_0                0x18 << 1     /**< When SAO of ITDS is connected to ground */
#define ITDS_ADDRESS_I2C_1                0x19 << 1     /**< When SAO of ITDS is connected to positive supply voltage */


/* Register address definitions */
#define ITDS_T_OUT_L_REG                  0x0D      /**< Temperature output LSB value register */
#define ITDS_T_OUT_H_REG                  0x0E      /**< Temperature output MSB value register */
#define ITDS_DEVICE_ID_REG                0x0F      /**< Device ID register */

/* Registers 0x10 - 0x1F are reserved. They contain factory calibration values that shall not be changed */
#define ITDS_CTRL_1_REG                   0x20      /**< Control register 1 */
#define ITDS_CTRL_2_REG                   0x21      /**< Control register 2 */
#define ITDS_CTRL_3_REG                   0x22      /**< Control register 3 */
#define ITDS_CTRL_4_REG                   0x23      /**< Control register 4 */
#define ITDS_CTRL_5_REG                   0x24      /**< Control register 5 */
#define ITDS_CTRL_6_REG                   0x25      /**< Control register 6 */
#define ITDS_T_OUT_REG                    0x26      /**< Temperature output data in 8 bit resolution register */
#define ITDS_STATUS_REG                   0x27      /**< Status register */
#define ITDS_X_OUT_L_REG                  0x28      /**< X axis acceleration output LSB value register */
#define ITDS_X_OUT_H_REG                  0x29      /**< X axis acceleration output MSB value register */
#define ITDS_Y_OUT_L_REG                  0x2A      /**< Y axis acceleration output LSB value register */
#define ITDS_Y_OUT_H_REG                  0x2B      /**< Y axis acceleration output MSB value register */
#define ITDS_Z_OUT_L_REG                  0x2C      /**< Z axis acceleration output LSB value register */
#define ITDS_Z_OUT_H_REG                  0x2D      /**< Z axis acceleration output MSB value register */
#define ITDS_FIFO_CTRL_REG                0x2E      /**< FIFO control register */
#define ITDS_FIFO_SAMPLES_REG             0x2F      /**< FIFO samples register */
#define ITDS_TAP_X_TH_REG                 0x30      /**< Tap recognition threshold in X direction register */
#define ITDS_TAP_Y_TH_REG                 0x31      /**< Tap recognition threshold in Y direction register */
#define ITDS_TAP_Z_TH_REG                 0x32      /**< Tap recognition threshold in Z direction register */
#define ITDS_INT_DUR_REG                  0x33      /**< Interrupt duration register */
#define ITDS_WAKE_UP_TH_REG               0x34      /**< Wake-up threshold register */
#define ITDS_WAKE_UP_DUR_REG              0x35      /**< Wake-up duration register */
#define ITDS_FREE_FALL_REG                0x36      /**< Free-fall register */
#define ITDS_STATUS_DETECT_REG            0x37      /**< Status detect register */
#define ITDS_WAKE_UP_EVENT_REG            0x38      /**< Wake-up event register */
#define ITDS_TAP_EVENT_REG                0x39      /**< Tap event register  */
#define ITDS_6D_EVENT_REG                 0x3A      /**< 6D (orientation change) event register */
#define ITDS_ALL_INT_EVENT_REG            0x3B      /**< All interrupts event register */
#define ITDS_X_OFS_USR_REG                0x3C      /**< Offset value for X axis register */
#define ITDS_Y_OFS_USR_REG                0x3D      /**< Offset value for Y axis register */
#define ITDS_Z_OFS_USR_REG                0x3E      /**< Offset value for Z axis register */
#define ITDS_CTRL_7_REG                   0x3F      /**< Control register 7 */


/* Register type definitions */

/**
 * @brief CTR_1_REG

 * Address 0x20
 * Type  R/W
 * Default value: 0x00
 *
 * ODR[3:0]  |       Power down / data rate configuration
 * --------------------------------------------------------------
 *   0000    | Power down
 *           |
 *           | High performance    Normal mode   Low power mode
 *   0001    |    12.5 Hz            12.5 Hz        1.6 Hz
 *   0010    |    12.5 Hz            12.5 Hz        12.5 Hz
 *   0011    |    25 Hz              25 Hz          25 Hz
 *   0100    |    50 Hz              50 Hz          50 Hz
 *   0101    |    100 Hz             100 Hz         100 Hz
 *   0110    |    200 Hz             200 Hz         200 Hz
 *   0111    |    400 Hz             200 Hz         200 Hz
 *   1000    |    800 Hz             800 Hz         200 Hz
 *   1001    |    1600Hz             1600Hz         200 Hz
 *
 * MODE[1:0] |                      Operating mode and resolution
 * ----------------------------------------------------------------------------------------
 *   00      |    Normal mode (14-bit resolution) / Low power mode (12-bit resolution)
 *   01      |    High performance mode (14-bit resolution)
 *   10      |    Single data conversion on demand mode (12/14-bit resolution)
 *   11      |    Unused
 */
typedef struct
{
    uint8_t powerMode : 2;          /**< LP_MODE[1:0]: Select normal mode or low power mode. Default 00 [00: low power mode; 10: normal mode] */
    uint8_t operatingMode : 2;      /**< MODE[1:0]: Select the operating mode and resolution. Default 00 */
    uint8_t outputDataRate : 4;     /**< ODR[3:0]: Output data rate selection. Default 0000 */
} ITDS_ctrl1_t;

/**
 * @brief CTR_2_REG
 * Address 0x21
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t notUsed01 : 1;        /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t i2cDisable : 1;       /**< I2C_DISABLE: Disable I2C digital Interface. Default: 0 (0: enabled; 1: disabled). */
    uint8_t autoAddIncr : 1;      /**< IF_ADD_INC: Register address automatically incremented during a multiple byte access with I2C interface. Default: 1. (0: disable; 1: enable). */
    uint8_t blockDataUpdate : 1;  /**< BDU: Block data update. 0 - continuous update; 1 - output registers are not updated until MSB and LSB have been read. */
    uint8_t disCSPullUp : 1;      /**< CP_PU_DISC: Disconnect pull up to CS pin. Default: 0 (0: connected; 1: disconnected). */
    uint8_t notUsed02 : 1;        /**< This bit must be set to 0 for proper operation of the device. */
    uint8_t softReset : 1;        /**< SOFT_RESET: Software reset. 0: normal mode; 1: SW reset; Self-clearing upon completion. */
    uint8_t boot : 1;             /**< BOOT: Set this bit to 1 to initiate boot sequence. 0: normal mode; 1: Execute boot sequence. Self-clearing upon completion. */
} ITDS_ctrl2_t;

/**
 * @brief CTR_3_REG
 * Address 0x22
 * Type  R/W
 * Default value: 0x00
 *
 *
 *   ST[1:0]    |     Self-test mode
 * -------------------------------------------
 *   00         |     Normal mode
 *   01         |     Positive sign self-test
 *   10         |     Negative sign self-test
 *   11         |              -
 */
typedef struct
{
    uint8_t startSingleDataConv : 1;    /**< SLP_MODE_1: Request single data conversion */
    uint8_t singleConvTrigger : 1;      /**< SLP_MODE_SEL: Single data conversion (on-demand) trigger signal. 0: Triggered by external signal on INT_1, 1: Triggered by writing 1 to SLP_MODE_1. */
    uint8_t notUsed01 : 1;              /**< This bit must be set to 0 for proper operation of the device */
    uint8_t intActiveLevel : 1;         /**< H_LACTIVE: Interrupt active level. Default: 0 (0: active high; 1: active low) */
    uint8_t enLatchedInterrupt : 1;     /**< LIR: Enable latched interrupt. Default: 0. (0: disabled; 1: enabled) */
    uint8_t intPinConf : 1;             /**< PP_OD: Push-pull/open-drain selection on interrupt pad. Default: 0 (0: push-pull; 1: open-drain) */
    uint8_t selfTestMode : 2;           /**< ST[1:0]: Select self test mode. Default: 00. */
} ITDS_ctrl3_t;

/**
 * @brief CTR_4_REG
 * Address 0x23
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t dataReadyINT0 : 1;      /**< INT0_DRDY: Data-ready interrupt signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t fifoThresholdINT0 : 1;  /**< INT0_FTH: FIFO threshold interrupt signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled)) */
    uint8_t fifoFullINT0 : 1;       /**< INT0_DIFF5: FIFO full interrupt signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t doubleTapINT0 : 1;      /**< INT0_TAP: Double-tap recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t freeFallINT0 : 1;       /**< INT0_FF: Free-fall recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t wakeUpINT0 : 1;         /**< INT0_WU: Wake-up recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t singleTapINT0 : 1;      /**< INT0_SINGLE_TAP: Single-tap recognition signal is routed to INT_0 pin: Default: 0 (0: disabled, 1: enabled) */
    uint8_t sixDINT0 : 1;           /**< INT0_6D: 6D recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
} ITDS_ctrl4_t;

/**
 * @brief CTR_5_REG
 * Address 0x24
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t dataReadyINT1 : 1;              /**< INT1_DRDY: Data-ready interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t fifoThresholdINT1 : 1;          /**< INT1_FTH: FIFO threshold interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled)) */
    uint8_t fifoFullINT1 : 1;               /**< INT1_DIFF5: FIFO full interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t fifoOverrunINT1 : 1;            /**< INT1_OVR: FIFO overrun interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t tempDataReadyINT1 : 1;          /**< INT1_DRDY_T: Temperature data-ready interrupt signal  is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t bootStatusINT1 : 1;             /**< INT1_BOOT: Boot status interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t sleepStatusChangeINT1 : 1;      /**< INT1_SLEEP_CHG: Sleep change status interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
    uint8_t sleepStateINT1 : 1;             /**< INT1_SLEEP_STATE: Sleep state interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
} ITDS_ctrl5_t;

/**
 * @brief CTR_6_REG
 * Address 0x25
 * Type  R/W
 * Default value: 0x00
 *
 *
 *   BW_FILT[1:0]    |          Bandwidth selection
 * -------------------------------------------------------------
 *     00            |    ODR/2 (except for ODR = 1600 Hz, 400 Hz)
 *     01            |    ODR/4 (High pass / Low pass filter)
 *     10            |    ODR/10 (High pass / Low pass filter)
 *     11            |    ODR/20 (High pass / Low pass filter)
 *
 *
 *   FS[1:0]    |   Full scale selection
 * ---------------------------------------
 *       00     |          ±2g
 *       01     |          ±4g
 *       10     |          ±8g
 *       11     |          ±16g
 */
typedef struct
{
    uint8_t notUsed01 : 1;        /**< This bit must be set to 0 for proper operation of the device */
    uint8_t notUsed02 : 1;        /**< This bit must be set to 0 for proper operation of the device */
    uint8_t enLowNoise : 1;       /**< LOW_NOISE: low noise configuration (0: disabled; 1: enabled) */
    uint8_t filterPath : 1;       /**< FDS: Filtered path configuration. Default value: 0. (0: low pass filter; 1: high pass filter) */
    uint8_t fullScale : 2;        /**< FS[1:0]: Full scale selection */
    uint8_t filterBandwidth : 2;  /**< BW_FILT[1:0]: Filter bandwidth selection  */
} ITDS_ctrl6_t;

/**
 * @brief STATUS_REG
 * Address 0x27
 * Type  R
 * Default value: 0x00
 *
 * Note: The status register is partially duplicated to the STATUS_DETECT_REG register.
 */
typedef struct
{
    uint8_t dataReady : 1;        /**< DRDY: Acceleration data-ready status bit (0: not ready, 1: X-, Y- and Z-axis new data available) */
    uint8_t freeFall : 1;         /**< FF_IA: Free-fall event detection bit (0: free-fall event not detected; 1: free-fall event detected) */
    uint8_t sixDDetection : 1;    /**< 6D_IA: Source of change in position portrait/landscape/face-up/face-down. (0: no event detected, 1: change in position detected) */
    uint8_t singleTap : 1;        /**< SINGLE_TAP: Single-tap event status bit (0: Single-tap event not detected, 1: Single-tap event detected) */
    uint8_t doubleTap : 1;        /**< DOUBLE_TAP: Double-tap event status bit (0: Double-tap event not detected, 1: Double-tap event detected) */
    uint8_t sleepState : 1;       /**< SLEEP_STATE: Sleep event status bit (0: Sleep event not detected, 1: Sleep event detected) */
    uint8_t wakeUp : 1;           /**< WU_IA: Wake-up event detection status bit (0: Wake-up event not detected, 1: Wake-up event detected) */
    uint8_t fifoThreshold : 1;    /**< FIFO_THS: FIFO threshold status bit (0: FIFO filling is lower than threshold level, 1: FIFO filling is equal to or higher than the threshold level) */
} ITDS_status_t;

/**
 * @brief CTRL_7_REG
 * Address 0x3F
 * Type  R/W
 * Default value: 0x00
 */
typedef struct
{
    uint8_t lowPassOn6D : 1;         /**< 0: ODR/2 low pass filtered data sent to 6D interrupt function (default), 1: LPF_1 output data sent to 6D interrupt function */
    uint8_t highPassRefMode : 1;     /**< HP_REF_MODE: Enables high-pass filter reference mode. Default: 0 (0: high-pass filter reference mode disabled, 1: high-pass filter reference mode enabled) */
    uint8_t userOffset : 1;          /**< USR_OFF_W: Defines the selection of weight of the user offset words specified by X_OFS_USR[7:0], Y_OFS_USR[7:0] and Z_OFS_USR[7:0] bits (0:977 µg/LSB, 1: 15.6 mg/LSB) */
    uint8_t applyWakeUpOffset : 1;   /**< USR_OFF_ON_WU: Enable application of user offset value to data for wake-up function only */
    uint8_t applyOffset : 1;         /**< USR_OFF_ON_OUT: Enable application of user offset value to output data registers. FDS: bit in CTRL_6 (0x25) must be set to ’0’-logic (low-pass path selected) */
    uint8_t enInterrupts : 1;        /**< INTERRUPTS_ENABLE: Enable interrupts */
    uint8_t INT1toINT0 : 1;          /**< INT1_ON_INT0: Defines signal routing (0: normal mode, 1: all signals available only on INT_1 are routed to INT_0) */
    uint8_t drdyPulse : 1;           /**< DRDY_PULSED: Switches between latched and pulsed mode for data ready interrupt (0: latched mode is used, 1: pulsed mode enabled for data-ready) */
} ITDS_ctrl7_t;


/* Functional type definitions */
typedef enum
{
    ITDS_disable = 0,
    ITDS_enable = 1
} ITDS_state_t;

typedef enum
{
    ITDS_positive = 0,
    ITDS_negative = 1
} ITDS_tapSign_t;

typedef enum
{
    ITDS_odr0,    /**< Power down */
                /**< High performance   Normal mode   Low power mode */
    ITDS_odr1,    /**< 12.5 Hz              12.5 Hz        1.6 Hz      */
    ITDS_odr2,    /**< 12.5 Hz              12.5 Hz        12.5 Hz     */
    ITDS_odr3,    /**< 25 Hz                25 Hz          25 Hz       */
    ITDS_odr4,    /**< 50 Hz                50 Hz          50 Hz       */
    ITDS_odr5,    /**< 100 Hz               100 Hz         100 Hz      */
    ITDS_odr6,    /**< 200 Hz               200 Hz         200 Hz      */
    ITDS_odr7,    /**< 400 Hz               200 Hz         200 Hz      */
    ITDS_odr8,    /**< 800 Hz               800 Hz         200 Hz      */
    ITDS_odr9     /**< 1600Hz               1600Hz         200 Hz      */
} ITDS_outputDataRate_t;

typedef enum
{
    ITDS_normalOrLowPower = 0,
    ITDS_highPerformance = 1,
    ITDS_singleConversion = 2
} ITDS_operatingMode_t;

typedef enum
{
    ITDS_lowPower,
    ITDS_normalMode
} ITDS_powerMode_t;

typedef enum
{
  ITDS_outputDataRate_2 = 0,    /**< ODR/2 (except for ODR = 1600 Hz, 400 Hz) */
  ITDS_outputDataRate_4 = 1,    /**< ODR/4 (High pass / Low pass filter) */
  ITDS_outputDataRate_10 = 2,   /**< ODR/10 (High pass / Low pass filter) */
  ITDS_outputDataRate_20 = 3    /**< ODR/20 (High pass / Low pass filter) */
} ITDS_bandwidth_t;

typedef enum
{
  ITDS_twoG = 0,      /**< ±2g */
  ITDS_fourG = 1,     /**< ±4g */
  ITDS_eightG = 2,    /**< ±8g */
  ITDS_sixteenG = 3   /**< ±16g */
} ITDS_fullScale_t;



/* Class definitions */
class IMU_3DIM_WSEN_ITDS
{
public:
    // Construtor
    IMU_3DIM_WSEN_ITDS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO);

    // Setting
    // TODO: description of available choises
    bool init(
        ITDS_operatingMode_t operatingMode = ITDS_highPerformance,
        ITDS_powerMode_t powerMode = ITDS_normalMode,
        ITDS_outputDataRate_t outputDataRate = ITDS_odr6,
        ITDS_fullScale_t fullScale = ITDS_sixteenG,
        ITDS_state_t blockDataUpdate = ITDS_enable
    );

    // Communication test
    bool isCommunicationReady();
    HAL_StatusTypeDef getDeviceID(uint8_t *device_ID);

    // Usage example
    void exampleLoop();       

    HAL_StatusTypeDef softReset();          // TODO
    HAL_StatusTypeDef getSoftResetState(ITDS_state_t *swReset); // TODO
    
    HAL_StatusTypeDef enableAutoIncrement();
    HAL_StatusTypeDef disableAutoIncrement();
    HAL_StatusTypeDef isAutoIncrementEnabled(ITDS_state_t *ai);
    bool isAutoIncrementEnabled();

    HAL_StatusTypeDef enableBlockDataUpdate();
    HAL_StatusTypeDef disableBlockDataUpdate();
    HAL_StatusTypeDef isBlockDataUpdateEnabled(ITDS_state_t *bdu);
    bool isBlockDataUpdateEnabled();

    HAL_StatusTypeDef setOperatingMode(ITDS_operatingMode_t opMode);
    HAL_StatusTypeDef getOperatingMode(ITDS_operatingMode_t *opMode);

    HAL_StatusTypeDef setPowerMode(ITDS_powerMode_t powerMode);
    HAL_StatusTypeDef getPowerMode(ITDS_powerMode_t *powerMode);

    HAL_StatusTypeDef setOutputDataRate(ITDS_outputDataRate_t odr);
    HAL_StatusTypeDef getOutputDataRate(ITDS_outputDataRate_t* odr);

    HAL_StatusTypeDef setFullScale(ITDS_fullScale_t fullScale);
    HAL_StatusTypeDef getFullScale(ITDS_fullScale_t *fullScale);

    HAL_StatusTypeDef enableOneShot();
    HAL_StatusTypeDef disableOneShot();
    HAL_StatusTypeDef isOneShotEnabled(ITDS_state_t *oneShot);
    bool isOneShotEnabled();

    HAL_StatusTypeDef isAccelerationDataReady(ITDS_state_t *state);
    bool isAccelerationDataReady();

    HAL_StatusTypeDef getAccelerationX(float *xAcc);
    HAL_StatusTypeDef getAccelerationY(float *yAcc);
    HAL_StatusTypeDef getAccelerationZ(float *zAcc);

    // TODO: temperature
    // HAL_StatusTypeDef getTemperature_float(float *tempDegC); 

private:
    // I2C Interface
    I2C_HandleTypeDef* i2c_handler;
    uint8_t i2c_device_addr;
    const uint16_t i2cMemAddSize = I2C_MEMADD_SIZE_8BIT;

    ITDS_fullScale_t currentFullScale;

    inline float convertAcceleration_float(const int16_t &acc);

    inline HAL_StatusTypeDef ITDS_ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf);
    inline HAL_StatusTypeDef ITDS_WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data);
};

#endif
