idf_component_register(SRCS "stepper_motor_rmt_backup.c" "i2c_bus.c" "mcp23017.c" "esp_littlefs.c" "kinematics.c" "stepper_motor_rmt.c" "stepper_motor_encoder.c" "l298n_stepper.c" "stepper_basic_test.c" "nextion.c" "uart_async_rxtxtasks_main.c" "http_server.c" "wifi.c" "GPIO_config.c" "ESP32_ShakeTable.c" "config_manager.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash driver esp_driver_rmt esp_driver_i2c esp_driver_gpio esp_driver_uart esp_adc esp_http_server esp_wifi esp_timer)
