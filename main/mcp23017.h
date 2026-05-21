#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// Endereço padrão I2C do MCP23017 (quando os pinos A0, A1 e A2 estão em LOW)
#define MCP23017_I2C_ADDR_DEFAULT 0x20

// Mapeamento dos Registos do MCP23017 (assumindo IOCON.BANK = 0)
#define MCP23017_IODIRA   0x00 // Direction Port A (1=Input, 0=Output)
#define MCP23017_IODIRB   0x01 // Direction Port B
#define MCP23017_GPIOA    0x12 // Value Port A
#define MCP23017_GPIOB    0x13 // Value Port B
#define MCP23017_OLATA    0x14 // Output Latch Port A
#define MCP23017_OLATB    0x15 // Output Latch Port B

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mcp23017_init(i2c_master_bus_handle_t bus_handle, uint8_t device_addr, i2c_master_dev_handle_t *mcp_handle);
esp_err_t mcp23017_write_reg(i2c_master_dev_handle_t mcp_handle, uint8_t reg_addr, uint8_t data);
esp_err_t mcp23017_read_reg(i2c_master_dev_handle_t mcp_handle, uint8_t reg_addr, uint8_t *data);

#ifdef __cplusplus
}
#endif
