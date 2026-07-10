#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include "driver/i2c_master.h"

// Pinos I2C
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 23

// Handles I2C globais
extern i2c_master_bus_handle_t i2c_bus_handle;
extern i2c_master_dev_handle_t mcp_handle;

void init_i2c_system(void);
void scan_i2c_bus(i2c_master_bus_handle_t bus_handle, const char* bus_name);
void set_all_steppers_microsteps(uint16_t micro_x, uint16_t micro_y);

#endif // I2C_BUS_H
