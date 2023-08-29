#ifndef __TMAG5273_H__
#define __TMAG5273_H__

#include "platform.h"

/* Available PADS I2C slave addresses */
#define TMAG5273A1_I2C_ADDRESS          0x35 << 1
//https://github.com/TuYuxiao/TMAG5273
// Register definition
//p25
#define DEVICE_CONFIG_1 0x00        // Configure Device Operation Modes 
#define DEVICE_CONFIG_2 0x01        // Configure Device Operation Modes
#define SENSOR_CONFIG_1 0x02        // Sensor Device Operation Modes
#define SENSOR_CONFIG_2 0x03        // Sensor Device Operation Modes
#define X_THR_CONFIG 0x04           // X Threshold Configuration
#define Y_THR_CONFIG 0x05           // Y Threshold Configuration 
#define Z_THR_CONFIG 0x06           // Z Threshold Configuration
#define T_CONFIG 0x07               // Temp Sensor Configuration
#define INT_CONFIG_1 0x08           // Configure Device Operation Modes
#define MAG_GAIN_CONFIG 0x09        // Configure Device Operation Modes
#define MAG_OFFSET_CONFIG_1 0x0A    // Configure Device Operation Modes
#define MAG_OFFSET_CONFIG_2 0x0B    // Configure Device Operation Modes
#define I2C_ADDRESS 0x0C            // I2C Address Register
#define DEVICE_ID 0x0D              // ID for the device die
#define MANUFACTURER_ID_LSB 0x0E    // Manufacturer ID lower byte 
#define MANUFACTURER_ID_MSB 0x0F    // Manufacturer ID lower byte 
#define T_MSB_RESULT 0x10           // Conversion Result Register
#define T_LSB_RESULT 0x11           // Conversion Result Register
#define X_MSB_RESULT 0x12           // Conversion Result Register
#define X_LSB_RESULT 0x13           // Conversion Result Register
#define Y_MSB_RESULT 0x14           // Conversion Result Register
#define Y_LSB_RESULT 0x15           // Conversion Result Register
#define Z_MSB_RESULT 0x16           // Conversion Result Register
#define Z_LSB_RESULT 0x17           // Conversion Result Register
#define CONV_STATUS 0x18            // Conversion Result Register
#define ANGLE_RESULT_MSB 0x19       // Conversion Result Register
#define ANGLE_RESULT_LSB 0x1A       // Conversion Result Register
#define MAGNITUDE_RESULT 0x1B       // Conversion Result Register
#define DEVICE_STATUS 0x1C          // Device_Diag Status Register


#define REG_DEVICE_CONFIG_1_DEFAULT 0x00
#define REG_DEVICE_CONFIG_2_DEFAULT 0x00
#define REG_SENSOR_CONFIG_1_DEFAULT 0x74
#define REG_SENSOR_CONFIG_2_DEFAULT 0x03
#define REG_X_THR_CONFIG_DEFAULT 0x00
#define REG_Y_THR_CONFIG_DEFAULT 0x00
#define REG_Z_THR_CONFIG_DEFAULT 0x00
#define REG_T_CONFIG_DEFAULT 0x00
#define REG_INT_CONFIG_1_DEFAULT 0x01
#define REG_MAG_GAIN_CONFIG_DEFAULT 0x00
#define REG_MAG_OFFSET_CONFIG_1_DEFAULT 0x00
#define REG_MAG_OFFSET_CONFIG_2_DEFAULT 0x00

#define DEVICE_ID_VALUE 0x01

#define TMAG5273_DEFAULT_ADDR 0x35
#define TMAG5273_ARRAY_START_ADDR 0x36

#define TSENSET0 25.
#define TADCRES 60.1
#define TADCT0 17508

// conv time upper bound? from datasheet
#define T_START_MEASURE 70   // set to zero seems ok
#define T_MEASURE_CHANNEL 25   // might be a little smaller but may cause imcomplete conversion
#define T_MEASURE_DUMMY 0
#define CONV_TIME_FROM_AVG_MODE(temp_en, mode) (T_START_MEASURE + (temp_en + 3) * T_MEASURE_CHANNEL * (1 + (1 << (mode >> 2))) + T_MEASURE_DUMMY) 

#define ARRAY_SINGLE_RC_DELAY_MS 1000
#define ARRAY_POWER_UP_TIMEOUT_MS 500

typedef struct
{
    uint8_t I2C_RD : 2;                 // Defines the I2C read mode
    uint8_t CONV_AVG : 3;               // Enables additional sampling of the sensor data to reduce the noise effect
    uint8_t MAG_TEMPCO : 2;             // Temperature coefficient of the magnet
    uint8_t CRC_EN : 1;                 // Enables I2C CRC byte to be sent
}tmag5273_device_config_1_t;

typedef struct
{
    uint8_t OPERATING_MODE : 2;         // 0h = Stand-by mode; 1h = Sleep mode; 2h = Continuous measure mode; 3h = Wake-up and sleep mode
    uint8_t TRIGGER_TO_MODE : 1;           // 0h = Conversion Start at I2C Command Bits, DEFAULT; 1h = Conversion starts through trigger signal at INT pin
    uint8_t I2C_GLITCH_FILTER : 1;      // 0h = Glitch filter on; 1h = Glitch filter off
    uint8_t LP_LN : 1;                  // 0h = Low active current mode; 1h = Low noise mode
    uint8_t THR_HYST : 3;               // Select thresholds for the interrupt function
}tmag5273_device_config_2_t;

typedef struct
{
    uint8_t SLEEPTIME : 4;               // Enables data acquisition of the magnetic axis channel
    uint8_t MAG_CH_EN : 4;               // Selects the time spent in low power mode between conversions 
}tmag5273_sensor_config_1_t;

typedef struct
{
    uint8_t Z_RANGE : 1;                 // Select the Z axis magnetic range from 2 different options.
    uint8_t X_Y_RANGE : 1;               // Select the X and Y axes magnetic range from 2 different options
    uint8_t ANGLE_EN : 2;                // Enables angle calculation, magnetic gain, and offset corrections between two selected magnetic channels
    uint8_t MAG_GAIN_CH : 1;             // Selects the axis for magnitude gain correction value entered in MAG_GAIN_CONFIG register
    uint8_t MMAG_THR_DIR : 1;            // Selects the direction of threshold check
    uint8_t THRX_COUNT : 1;              // Number of threshold crossings before the interrupt is asserted    
    uint8_t RESERVED : 1;     
}tmag5273_sensor_config_2_t;

typedef struct
{
    uint8_t T_CH_EN : 1;               // Enables data acquisition of the magnetic axis channel
    uint8_t T_THR_CONFIG : 7;               // Selects the time spent in low power mode between conversions 
}tmag5273_T_config_t;

typedef enum tmag5273_operating_mode {
    TMAG5273_OPERATING_MODE_STANDBY = 0,    // Stand-by mode
    TMAG5273_OPERATING_MODE_SLEEP = 1,      // Sleep mode
    TMAG5273_OPERATING_MODE_MEASURE = 2,    // Continuous measure mode
    TMAG5273_OPERATING_MODE_WS = 3          // Wake-up and sleep mode
} tmag5273_operating_mode_t;

typedef enum tmag5273_read_mode {
    TMAG5273_READ_MODE_STANDARD = 0x0,
    TMAG5273_READ_MODE_SENSOR16,
    TMAG5273_READ_MODE_SENSOR8
} tmag5273_read_mode_t;

typedef enum tmag5273_mag_range {
    TMAG5273_MAG_RANGE_40MT = 0,     // for TMAG5273A1
    TMAG5273_MAG_RANGE_80MT = 1     // for TMAG5273A1
} tmag5273_mag_range_t;

typedef enum tmag5273_mag_range_multiple {
    TMAG5273_MAG_RANGE_MULTIPLE_1x = 0x0, 
    TMAG5273_MAG_RANGE_MULTIPLE_2x = 0x3
} tmag5273_mag_range_multiple_t;

typedef enum tmag5273_lp_ln_mode { 
    TMAG5273_LOW_ACTIVE_CURRENT = 0,   // Low active current mode
    TMAG5273_LOW_NOISE = 1             // Low noise mode
} tmag5273_lp_ln_mode_t;

typedef enum tmag5273_mag_tempco_mode { 
    TMAG5273_NO_MAG_TEMPCO = 0x0,
    TMAG5273_MAG_TEMPCO_NdBFe = 0x1 << 5,
    TMAG5273_MAG_TEMPCO_FERRITE = 0x3 << 5
} tmag5273_mag_tempco_mode_t;

typedef enum tmag5273_conv_avg_rate { 
    TMAG5273_CONV_AVG_1X = 0,           // 10.0-kSPS (3-axes) or 20-kSPS (1 axis)
    TMAG5273_CONV_AVG_2X = 1,           // 5.7-kSPS (3-axes) or 13.3-kSPS (1 axis)
    TMAG5273_CONV_AVG_4X = 2,           // 3.1-kSPS (3-axes) or 8.0-kSPS (1 axis)
    TMAG5273_CONV_AVG_8X = 3,           // 1.6-kSPS (3-axes) or 4.4-kSPS (1 axis)
    TMAG5273_CONV_AVG_16X = 4,          // 0.8-kSPS (3-axes) or 2.4-kSPS (1 axis)
    TMAG5273_CONV_AVG_32X = 5,          // 0.4-kSPS (3-axes) or 1.2-kSPS (1 axis)
} tmag5273_conv_avg_rate_t;

typedef enum tmag5273_temp_ch_en { // only decides the TMAG5273_READ_MODE_SENSOR16 data
    TMAG5273_TEMP_CH_DISABLED = 0,
    TMAG5273_TEMP_CH_ENABLED_X = 1,
    TMAG5273_TEMP_CH_ENABLED_Y = 2,
    TMAG5273_TEMP_CH_ENABLED_XY = 3,
    TMAG5273_TEMP_CH_ENABLED_Z = 4,
    TMAG5273_TEMP_CH_ENABLED_ZX = 5,
    TMAG5273_TEMP_CH_ENABLED_YZ = 6,
    TMAG5273_TEMP_CH_ENABLED_XYZ = 7,
} tmag5273_temp_ch_en_t;

typedef enum tmag5273_temperature_ch_en { 
    TMAG5273_Temperature_CH_DISABLED = 0,
    TMAG5273_Temperature_CH_ENABLED = 1,
} tmag5273_temperature_ch_en_t;

typedef enum tmag5273_angle_ch_en { 
    TMAG5273_Angle_CH_DISABLED = 0,
    TMAG5273_Angle_CH_ENABLED_XY = 1,
    TMAG5273_Angle_CH_ENABLED_YZ = 2,
    TMAG5273_Angle_CH_ENABLED_XZ = 3,
} tmag5273_angle_ch_en_t;

// TODO support trigger


class TMAG5273 
{
public:
    TMAG5273(I2C_HandleTypeDef* i2c_handler);

    bool init(
        tmag5273_conv_avg_rate_t CONV_AVG = TMAG5273_CONV_AVG_1X,
        tmag5273_lp_ln_mode_t LP_LN = TMAG5273_LOW_ACTIVE_CURRENT,
        tmag5273_mag_range_t RANGE = TMAG5273_MAG_RANGE_80MT,
        tmag5273_angle_ch_en_t ANGLE = TMAG5273_Angle_CH_DISABLED);

    HAL_StatusTypeDef getDeviceID(uint8_t *device_ID);
    bool isCommunicationReady();

    HAL_StatusTypeDef changeToStandByMode();
    HAL_StatusTypeDef changeToContinuousMeasureMode();

    HAL_StatusTypeDef enableMagneticChannels();
    HAL_StatusTypeDef disableMagneticChannels();
    bool isMagneticChannelsEnabled();

    HAL_StatusTypeDef enableTemperatureChannels();
    HAL_StatusTypeDef disableTemperatureChannels();
    bool isMagneticTemperatureEnabled();

    HAL_StatusTypeDef setOutputDataRate(tmag5273_conv_avg_rate_t odr);
    HAL_StatusTypeDef getOutputDataRate(tmag5273_conv_avg_rate_t* odr);

    HAL_StatusTypeDef setLPLNMode(tmag5273_lp_ln_mode_t lp_ln);
    HAL_StatusTypeDef getLPLNMode(tmag5273_lp_ln_mode_t* lp_ln);

    HAL_StatusTypeDef setTMAGRange(tmag5273_mag_range_t range);
    HAL_StatusTypeDef getTMAGRange(tmag5273_mag_range_t* range);

    HAL_StatusTypeDef getMagneticField_X(float *Bx);
    HAL_StatusTypeDef getMagneticField_Y(float *By);
    HAL_StatusTypeDef getMagneticField_Z(float *Bz);
    float getMagneticField_X();
    float getMagneticField_Y();
    float getMagneticField_Z();

    HAL_StatusTypeDef getMagneticField_T(float *T);
    float getMagneticField_T();

    HAL_StatusTypeDef setTMAGAngleEnable(tmag5273_angle_ch_en_t angle);
    HAL_StatusTypeDef getTMAGAngleEnable(tmag5273_angle_ch_en_t* angle);

    HAL_StatusTypeDef getMagneticAngle_w(float *W);
    float getMagneticAngle_W();

    HAL_StatusTypeDef getMagneticOffest_e(float *E);
    float getMagneticOffest_E();

private:
    I2C_HandleTypeDef* i2c_handler;
    uint8_t i2c_device_addr;
    const uint16_t i2cMemAddSize = I2C_MEMADD_SIZE_8BIT;

    tmag5273_mag_range_t range;
    float convertMagneticField_float(int16_t B, tmag5273_mag_range_t range);
    float convertMagneticOffest_float(int8_t E, tmag5273_mag_range_t range);
    float convertTemperature_float(int16_t T);

    inline HAL_StatusTypeDef TMAG_ReadReg(uint16_t regAddr, uint16_t numBytesToRead, uint8_t *buf);
    inline HAL_StatusTypeDef TMAG_WriteReg(uint16_t regAddr, uint16_t numBytesToWrite, uint8_t *data);
};
#endif /* TMAG5273_H */
