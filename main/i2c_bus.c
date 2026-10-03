#include "i2c_bus.h"
#include "esp_log.h"
#include "mcp23017.h"

static const char* TAG = "I2C_BUS";

i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_dev_handle_t mcp_handle = NULL;
static uint8_t mcp_port_b_state = 0;

void scan_i2c_bus(i2c_master_bus_handle_t bus_handle, const char* bus_name) {
    ESP_LOGI("I2C_SCAN", "Starting scanner on bus: %s", bus_name);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t err = i2c_master_probe(bus_handle, addr, 20); // Reduced from 100 to 20ms
        if (err == ESP_OK) {
            ESP_LOGI("I2C_SCAN", " -> Device detected at address: 0x%02X", addr);
            found++;
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE("I2C_SCAN", " -> Bus is stuck or timed out! Aborting scan on %s to prevent log spam.", bus_name);
            break; // If timeout, bus is stuck (e.g., short circuit). Aborting to avoid spamming 127 times.
        }
    }
    if (found == 0) {
        ESP_LOGW("I2C_SCAN", " -> No devices found on %s!", bus_name);
    }
}

void init_i2c_system(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0, // Forces the use of physical block 0
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Initialize the I/O Expander
    if (mcp23017_init(i2c_bus_handle, MCP23017_I2C_ADDR_DEFAULT, &mcp_handle) == ESP_OK) {
        // Example: Configure all BANK A pins as OUTPUTS (0x00)
        esp_err_t err = mcp23017_write_reg(mcp_handle, MCP23017_IODIRA, 0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Timeout/Failed to configure MCP23017! Check SDA/SCL and RST pins.");
            i2c_master_bus_rm_device(mcp_handle);
            mcp_handle = NULL;
        } else {
            mcp23017_write_reg(mcp_handle, MCP23017_IODIRB, 0x00); // Configures BANK B as OUTPUTS
            mcp23017_write_reg(mcp_handle, MCP23017_GPIOA, 0x00); // Ensures LEDs (A0 and A1) start turned off
            mcp23017_write_reg(mcp_handle, MCP23017_GPIOB, 0x00); // Base state 0 on PORT B
            ESP_LOGI(TAG, "MCP23017 setup completed successfully!");
        }
    }

    // Run I2C Scanner to debug hardware
    scan_i2c_bus(i2c_bus_handle, "BUS 1 (MCP23017)");
}

// =========================================================================
// Converts microstepping value to 3 bits (helper for set_all_steppers_microsteps)
// =========================================================================
static uint8_t get_microstep_bits(uint16_t microsteps) {
    switch(microsteps) {
        case 4:  return 0b010; // b0=low, b1=high, b2=low -> 4
        case 8:  return 0b011; // b0=high, b1=high, b2=low -> 8
        case 16: return 0b100; // b0=low, b1=low, b2=high -> 16
        case 32: return 0b111; // b0=high, b1=high, b2=high -> 32
        default: return 0b111; // default safety (32)
    }
}

// =========================================================================
// Configures pins M0, M1, M2 (Motor X) and M3, M4, M5 (Motor Y) via I2C
// Motor X: bits 0-2 (b0, b1, b2)
// Motor Y: bits 3-5 (b3, b4, b5)
// =========================================================================
void set_all_steppers_microsteps(uint16_t micro_x, uint16_t micro_y) {
    if (!mcp_handle) return;
    
    uint8_t bits_x = get_microstep_bits(micro_x);  // bits 0-2
    uint8_t bits_y = get_microstep_bits(micro_y);  // bits 3-5
    
    // Clear relevant bits (0-5) and apply new values
    mcp_port_b_state &= ~0b00111111; // Clear bits 0-5
    mcp_port_b_state |= bits_x;                    // Bits 0-2 for Motor X
    mcp_port_b_state |= (bits_y << 3);             // Bits 3-5 for Motor Y
    
    mcp23017_write_reg(mcp_handle, MCP23017_GPIOB, mcp_port_b_state);
    ESP_LOGI(TAG, "MCP23017: Motor X=%d uSteps (b0-b2=0x%X), Motor Y=%d uSteps (b3-b5=0x%X), Port B: 0x%02X", 
             micro_x, bits_x, micro_y, bits_y, mcp_port_b_state);
}
