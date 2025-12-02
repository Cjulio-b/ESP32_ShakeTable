#include <stdio.h>
#include <time.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "functions.h"
#include "driver/gpio.h"

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

	printf("Hello, this is ESP32 Speaking!\n");
	ESP_LOGI(TAG, "msg: Hello\n");

	printf("Starting the Shake Table...\n");
	ESP_LOGI(TAG, "msg: Starting the Shake Table...\n");
	
	vTaskDelay(pdMS_TO_TICKS(3000)); // Delay de 3 segundos

	xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle);
	xTaskCreate(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle);
	// Task para monitorizar o uso de stack
    xTaskCreate(monitor_task, "monitor_task", 4096, NULL, configMAX_PRIORITIES - 20, NULL);

	while (1)
	{	
		testing_led();
		vTaskDelay(pdMS_TO_TICKS(10)); // Delay de 10ms
/* 		step_motor(1);
		vTaskDelay(pdMS_TO_TICKS(5000)); // Delay de 1s
		step_motor(0);
		vTaskDelay(pdMS_TO_TICKS(1000)); // Delay de 1s */
	}

}
