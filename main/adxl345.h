#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// ADXL345 I2C Addresses
#define ADXL345_I2C_ADDR_GND 0x53 // When ALT ADDRESS (SDO) pin is tied to GND
#define ADXL345_I2C_ADDR_3V3 0x1D // When ALT ADDRESS (SDO) pin is tied to 3.3V

// ADXL345 Registers
#define ADXL345_REG_DEVID       0x00 // Device ID
#define ADXL345_REG_POWER_CTL   0x2D // Power-saving features control
#define ADXL345_REG_DATA_FORMAT 0x31 // Data format control
#define ADXL345_REG_DATAX0      0x32 // X-Axis Data 0

// ADXL345 Configurations
#define ADXL345_MEASURE_MODE    0x08 // Measure mode for POWER_CTL
#define ADXL345_FULL_RES_16G    0x0B // Full resolution, +/- 16g range for DATA_FORMAT

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ADXL345 sensor on the given I2C bus.
 */
esp_err_t adxl345_init(i2c_master_bus_handle_t bus_handle, uint8_t device_addr, i2c_master_dev_handle_t *adxl_handle);

/**
 * @brief Read X, Y, and Z accelerations in 'g' forces.
 */
esp_err_t adxl345_read_acceleration(i2c_master_dev_handle_t adxl_handle, float *accel_x, float *accel_y, float *accel_z);

#ifdef __cplusplus
}
#endif