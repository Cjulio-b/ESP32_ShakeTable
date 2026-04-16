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
	init_uart();
	//start_wifi_ap(); //start Wi-Fi
	//start_webserver(); //start HTTP server

	// (1/2) --- Teste do stepper motor L298N half-step ------
    ESP_LOGI("STEPMOTOR", "Initializing L298N and stepper motor...");
    l298n_init();
    l298n_set_min_step_us(1000);  // segura para a maioria dos NEMA17
	// (2/2) --- Teste do stepper motor L298N half-step ------

	printf("Hello, this is ESP32 Speaking!\n");
	ESP_LOGI(TAG, "msg: Hello\n");

	printf("Starting the Shake Table...\n");
	ESP_LOGI(TAG, "msg: Starting the Shake Table...\n");
	
	// --- Teste da Estrutura de Cinemática ---
	
	shake_table_config_t my_table;
	kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm de deslocamento, 66mm de biela
	kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32);  // 1.8º, 32 microsteps (6400 passos/volta)

	float angulo = kinematics_calc_angular_position(&my_table.stepper_x, 1600);
	float posicao_mm = kinematics_calc_linear_position(&my_table.axis_x, angulo);
	float posicao_real_mm = kinematics_calc_linear_position_relative_90(&my_table.axis_x, angulo);
	ESP_LOGI(TAG, "Teste Cinemática: 1600 micropassos = %.2f graus", angulo);
	ESP_LOGI(TAG, "  -> Posição Geométrica (Relativa à biela): %.2f mm", posicao_mm);
	ESP_LOGI(TAG, "  -> Posição Real (Relativa a 90 graus): %.2f mm", posicao_real_mm);
	
	// --- Fim do Teste da Estrutura de Cinemática ---

	vTaskDelay(pdMS_TO_TICKS(3000)); // Delay de 3 segundos

	xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle);
	xTaskCreate(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle);
	// Task para monitorizar o uso de stack
    xTaskCreate(monitor_task, "monitor_task", 4096, NULL, configMAX_PRIORITIES - 20, NULL);

	// Task para controlar o stepper motor L298N - NOT USED, USE RMT INSTEAD
	//xTaskCreate(stepper_task, "stepper_task", 4096, NULL, configMAX_PRIORITIES - 14, NULL);

	// Tasks para controlar os stepper motores com DRV8825 e RMT de forma independente
	xTaskCreate(stepper_rmt_task_1, "stepper_rmt_task_1", 4096, NULL, configMAX_PRIORITIES - 10, NULL);
	xTaskCreate(stepper_rmt_task_2, "stepper_rmt_task_2", 4096, NULL, configMAX_PRIORITIES - 10, NULL);

	while (1)
	{	
		//testing_led();
		vTaskDelay(pdMS_TO_TICKS(10)); // Delay de 10ms
	}

}
