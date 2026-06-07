#include "mcp23017.h"
#include "esp_log.h"

static const char *TAG = "MCP23017";

#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t mcp23017_init(i2c_master_bus_handle_t bus_handle, uint8_t device_addr, i2c_master_dev_handle_t *mcp_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = 100000, // 100kHz (Mais seguro para protoboards)
    };
    
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, mcp_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar MCP23017 (0x%02X) ao barramento I2C", device_addr);
        return err;
    }
    
    ESP_LOGI(TAG, "MCP23017 (0x%02X) inicializado com sucesso!", device_addr);
    return ESP_OK;
}

esp_err_t mcp23017_write_reg(i2c_master_dev_handle_t mcp_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    // Transmite o endereço do registo seguido do byte de configuração
    return i2c_master_transmit(mcp_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

esp_err_t mcp23017_read_reg(i2c_master_dev_handle_t mcp_handle, uint8_t reg_addr, uint8_t *data)
{
    // Transmite primeiro o endereço do registo que queremos ler e depois recebe o valor dele
    return i2c_master_transmit_receive(mcp_handle, &reg_addr, 1, data, 1, I2C_MASTER_TIMEOUT_MS);
}
