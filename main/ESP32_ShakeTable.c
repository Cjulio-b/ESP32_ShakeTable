#include <stdio.h>
#include <time.h>
#include <math.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "functions.h"
#include "driver/gpio.h"
#include "l298n_stepper.h"
#include "kinematics.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "i2c_bus.h"

static const char* TAG = "ESP32_ShakeTable";
#define UART_TASK_STACK_SIZE 4096
extern TaskHandle_t rxTaskHandle;
extern TaskHandle_t txTaskHandle;

// =========================================================================
// Telemetria UART Task (Envia as coordenadas a 100Hz para o MCU 2)
// =========================================================================
void telemetry_task(void *arg) {
    char buf[64];
    bool sync_active = false;

    while(1) {
        if (nextion_profile != 0 && motor1_ready && motor2_ready) {
            // Activa o SYNC Pin no arranque
            if (!sync_active) {
                gpio_set_level(SYNC_GPIO_PIN, 1);
                sync_active = true;
                ESP_LOGI(TAG, "SYNC Pin = HIGH (Ensaio Iniciado)");
            }

            // Lê diretamente a posição alvo atual gerada pelos perfis dos motores
            float csv_target_x = current_target_pos_x;
            float csv_target_y = current_target_pos_y;
            
            int len = snprintf(buf, sizeof(buf), "%.3f,%.3f\n", csv_target_x, csv_target_y);
            uart_write_bytes(UART_TELEMETRY_NUM, buf, len);
        } else {
            // Desactiva o SYNC Pin no fim
            if (sync_active) {
                gpio_set_level(SYNC_GPIO_PIN, 0);
                sync_active = false;
                ESP_LOGI(TAG, "SYNC Pin = LOW (Ensaio Terminado)");
            }

            // Garante que manda zero quando parado
            int len = snprintf(buf, sizeof(buf), "0.000,0.000\n");
            uart_write_bytes(UART_TELEMETRY_NUM, buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
	GPIO_init(); //initialize GPIOs
	check_current_config(); //print GPIO configuration
	init_littlefs(); // Mount LittleFS Partition
	init_uart_to_Nextion();
	start_wifi_ap(); //start Wi-Fi
	start_webserver(); //start HTTP server
	config_manager_init(); // Carrega configurações da NVS ou usa defaults
	init_i2c_system(); // Inicia I2C e deteta o MCP23017

	// Inicializa UART Telemetry (Comunicação com MCU 2)
    init_uart_to_mcu2();

	// Atualiza o hardware com os microsteps carregados da memória para ambos os motores
	set_all_steppers_microsteps(table_config_x.stepper.microsteps, table_config_y.stepper.microsteps);

	// --- Kinematics Structure Test ---
	
	/*shake_table_config_t my_table;
	kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
	kinematics_init_stepper(&my_table.stepper_x, 1.8f, 1.0f, 32);  // 1.8 degree step, gear 1.0, 32 microsteps (6400 steps/rev)

	float angle = kinematics_calc_angular_position(&my_table.stepper_x, 1600);
	float position_mm = kinematics_calc_linear_position(&my_table.axis_x, angle);
	float real_position_mm = kinematics_calc_linear_position_relative_90(&my_table.axis_x, angle);
	ESP_LOGI(TAG, "Kinematics Test: 1600 microsteps = %.2f degrees", angle);
	ESP_LOGI(TAG, "  -> Geometric Position (Relative to rod): %.2f mm", position_mm);
	ESP_LOGI(TAG, "  -> Real Position (Relative to 90 degrees): %.2f mm", real_position_mm);
	*/
	// --- End of Kinematics Structure Test ---

	vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay

	// Tarefas do Ecrã Nextion e Telemetria (FreeRTOS gere os cores automaticamente)
	xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle);
	xTaskCreate(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle);
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, configMAX_PRIORITIES - 10, NULL);
    xTaskCreate(telemetry_rx_task, "telemetry_rx_task", 4096, NULL, configMAX_PRIORITIES - 11, NULL);

	// Tasks to control the stepper motors with DRV8825 and RMT independently
	// Mantém-se o Core 1 para máxima estabilidade e imunidade a interrupções do sistema (WiFi/SPI Flash)
	xTaskCreatePinnedToCore(stepper_rmt_task_1, "stepper_rmt_task_1", 8192, NULL, configMAX_PRIORITIES - 5, NULL, 1);
	xTaskCreatePinnedToCore(stepper_rmt_task_2, "stepper_rmt_task_2", 8192, NULL, configMAX_PRIORITIES - 5, NULL, 1);

	// FreeRTOS main loop: Keep the main task alive with periodic vTaskDelay
	// The actual work happens in the 4 tasks created above
	// vTaskDelay prevents watchdog timeout and allows FreeRTOS scheduler to run
	// while (1)
	// {	
	// 	vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
	// }

}
