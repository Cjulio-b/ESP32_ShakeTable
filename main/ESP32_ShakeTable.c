#include <stdio.h>
#include <time.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "functions.h"
#include "driver/gpio.h"
#include "l298n_stepper.h"

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
	
	vTaskDelay(pdMS_TO_TICKS(3000)); // Delay de 3 segundos

	xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle);
	xTaskCreate(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle);
	// Task para monitorizar o uso de stack
    xTaskCreate(monitor_task, "monitor_task", 4096, NULL, configMAX_PRIORITIES - 20, NULL);

	// Task para controlar o stepper motor L298N
	xTaskCreate(stepper_task, "stepper_task", 4096, NULL, configMAX_PRIORITIES - 14, NULL);

	// Task para controlar o stepper motor com DRV8825 e RMT
	xTaskCreate(stepper_rmt_task,"stepper_rmt_task",4096,NULL,configMAX_PRIORITIES - 10,NULL);

	while (1)
	{	
		testing_led();
		vTaskDelay(pdMS_TO_TICKS(10)); // Delay de 10ms
	}

}
