#include "adxl345.h"
#include "esp_log.h"

static const char *TAG = "ADXL345";
#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t adxl345_init(i2c_master_bus_handle_t bus_handle, uint8_t device_addr, i2c_master_dev_handle_t *adxl_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = 100000, // 100kHz Standard Mode (Mais seguro para protoboards)
    };
    
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, adxl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ADXL345 (0x%02X) to I2C bus", device_addr);
        return err;
    }
    
    // 1. Check Device ID (Optional but recommended, ADXL345 should return 0xE5)
    uint8_t dev_id = 0;
    uint8_t reg_devid = ADXL345_REG_DEVID;
    err = i2c_master_transmit_receive(*adxl_handle, &reg_devid, 1, &dev_id, 1, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with ADXL345 (0x%02X). Check wiring!", device_addr);
        i2c_master_bus_rm_device(*adxl_handle);
        *adxl_handle = NULL;
        return err;
    }

    if (dev_id != 0xE5) {
        ESP_LOGE(TAG, "Error: Expected Device ID 0xE5 for ADXL345 (0x%02X), got 0x%02X. Aborting.", device_addr, dev_id);
        i2c_master_bus_rm_device(*adxl_handle);
        *adxl_handle = NULL;
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 2. Set Data Format (Full resolution, +/- 16g)
    uint8_t format_cmd[2] = {ADXL345_REG_DATA_FORMAT, ADXL345_FULL_RES_16G};
    err = i2c_master_transmit(*adxl_handle, format_cmd, sizeof(format_cmd), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(*adxl_handle);
        *adxl_handle = NULL;
        return err;
    }

    // 3. Enable Measurement Mode
    uint8_t power_cmd[2] = {ADXL345_REG_POWER_CTL, ADXL345_MEASURE_MODE};
    err = i2c_master_transmit(*adxl_handle, power_cmd, sizeof(power_cmd), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(*adxl_handle);
        *adxl_handle = NULL;
        return err;
    }

    ESP_LOGI(TAG, "ADXL345 (0x%02X) Initialized Successfully!", device_addr);
    return ESP_OK;
}

esp_err_t adxl345_read_acceleration(i2c_master_dev_handle_t adxl_handle, float *accel_x, float *accel_y, float *accel_z)
{
    uint8_t reg_start = ADXL345_REG_DATAX0;
    uint8_t data[6] = {0};

    // Read 6 bytes starting from DATAX0 (X0, X1, Y0, Y1, Z0, Z1)
    esp_err_t err = i2c_master_transmit_receive(adxl_handle, &reg_start, 1, data, sizeof(data), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    // Combine LSB and MSB into 16-bit integers
    int16_t x_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t y_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t z_raw = (int16_t)((data[5] << 8) | data[4]);

    // Convert raw values to 'g' (gravity).
    // In full resolution mode, the scale factor is typically 0.0039 g/LSB (or ~3.9 mg/LSB).
    float scale_factor = 0.0039f; 

    if (accel_x) *accel_x = (float)x_raw * scale_factor;
    if (accel_y) *accel_y = (float)y_raw * scale_factor;
    if (accel_z) *accel_z = (float)z_raw * scale_factor;

    // Note: If you want values in m/s^2 instead of 'g', multiply by 9.80665f:
    // *accel_x = (*accel_x) * 9.80665f;

    return ESP_OK;
}