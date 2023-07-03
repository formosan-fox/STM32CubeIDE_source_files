#include "IMU_3DIM_WSEN_ITDS.h"
#include "util.h"

static const uint32_t I2C_TIMEOUT = 100;

IMU_3DIM_WSEN_ITDS::IMU_3DIM_WSEN_ITDS(I2C_HandleTypeDef* _i2c_handler, uint8_t SAO): i2c_handler(_i2c_handler)
{
    i2c_device_addr = (SAO == 0) ? ITDS_ADDRESS_I2C_0 : ITDS_ADDRESS_I2C_1;
}


bool IMU_3DIM_WSEN_ITDS::isCommunicationReady()
{
    uint8_t device_ID = 0;

    getDeviceID(&device_ID);
	if (device_ID == ITDS_DEVICE_ID_VALUE) {
		debug_print<uint8_t>("device ID = %x\r\n", device_ID);
		debug_print("WSEN_ITDS communicates successfully.\r\n");
		return true;
	} else {
		debug_print<uint8_t>("device ID = %x\r\n", device_ID);
		debug_print("WSEN_ITDS Communicates failed.\r\n");
		return false;
	}
}

bool IMU_3DIM_WSEN_ITDS::init()
{
	// ITDS_state_t swReset;

    // TODO: Initialize sensor interface.


    // Wait for boot
	HAL_Delay(50);

    // TODO: Check Sensor Interface Ready

    // Communication test.
	if (!isCommunicationReady()) {
		return false;
	}

    // // Soft reset for setting the internal registers to the default values.
	// softReset();
	// do
	// {
	// 	getSoftResetState(&swReset);
	// } while (swReset);
	// debug_print("WSEN PADS: Soft reset completes.\r\n");

    // // TODO: reboot


    // // Automatic increment register address
	// enableAutoIncrement();
	// if (isAutoIncrementEnabled()) {
	// 	debug_print("WSEN PADS: Auto increment is enabled.\r\n");
	// }

    // // Enable block data update
    // enableBlockDataUpdate();
    // if (isBlockDataUpdateEnabled()) {
    // 	debug_print("WSEN PADS: Block data update is enabled.\r\n");
    // }

    return true;
}

HAL_StatusTypeDef IMU_3DIM_WSEN_ITDS::getDeviceID(uint8_t *device_ID)
{
    return ITDS_ReadReg(ITDS_DEVICE_ID_REG, 1, device_ID);
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


