#include <stdio.h>
#include <time.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "functions.h"
#include "driver/gpio.h"
#include "l298n_stepper.h"
#include "kinematics.h"

static const char* TAG = "ESP32_ShakeTable";
#define UART_TASK_STACK_SIZE 4096
extern TaskHandle_t rxTaskHandle;
extern TaskHandle_t txTaskHandle;

void app_main(void)
{
	GPIO_init(); //initialize GPIOs
	check_current_config(); //print GPIO configuration
	init_littlefs(); // Mount LittleFS Partition
	init_uart();
	start_wifi_ap(); //start Wi-Fi
	start_webserver(); //start HTTP server

	// (1/2) --- Teste do stepper motor L298N half-step ------
    ESP_LOGI("STEPMOTOR", "Initializing L298N and stepper motor...");
    l298n_init();
    l298n_set_min_step_us(1000);  // segura para a maioria dos NEMA17
	// (2/2) --- Teste do stepper motor L298N half-step ------

	printf("Hello, this is ESP32 Speaking!\n");
	ESP_LOGI(TAG, "msg: Hello\n");

	printf("Starting the Shake Table...\n");
	ESP_LOGI(TAG, "msg: Starting the Shake Table...\n");
	
	// --- Kinematics Structure Test ---
	
	/*shake_table_config_t my_table;
	kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
	kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32);  // 1.8 degree step, 32 microsteps (6400 steps/rev)

	float angle = kinematics_calc_angular_position(&my_table.stepper_x, 1600);
	float position_mm = kinematics_calc_linear_position(&my_table.axis_x, angle);
	float real_position_mm = kinematics_calc_linear_position_relative_90(&my_table.axis_x, angle);
	ESP_LOGI(TAG, "Kinematics Test: 1600 microsteps = %.2f degrees", angle);
	ESP_LOGI(TAG, "  -> Geometric Position (Relative to rod): %.2f mm", position_mm);
	ESP_LOGI(TAG, "  -> Real Position (Relative to 90 degrees): %.2f mm", real_position_mm);
	*/
	// --- End of Kinematics Structure Test ---

	vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds delay

	xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle);
	xTaskCreate(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle);
	// Task to monitor stack usage
    xTaskCreate(monitor_task, "monitor_task", 4096, NULL, configMAX_PRIORITIES - 20, NULL);

	// Task para controlar o stepper motor L298N - NOT USED, USE RMT INSTEAD
	//xTaskCreate(stepper_task, "stepper_task", 4096, NULL, configMAX_PRIORITIES - 14, NULL);

	// Tasks to control the stepper motors with DRV8825 and RMT independently
	xTaskCreate(stepper_rmt_task_1, "stepper_rmt_task_1", 4096, NULL, configMAX_PRIORITIES - 10, NULL);
	xTaskCreate(stepper_rmt_task_2, "stepper_rmt_task_2", 4096, NULL, configMAX_PRIORITIES - 10, NULL);

	while (1)
	{	
		//testing_led();
		vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
	}

}
