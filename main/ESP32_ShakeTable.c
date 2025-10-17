#include <stdio.h>
#include <time.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "functions.h"
#include "driver/gpio.h"

static const char* TAG = "MyModule";

void app_main(void)
{
	GPIO_init(); //initialize GPIOs
	check_current_config(); //print GPIO configuration

	printf("Hello, this is ESP32 Speaking!\n");
	ESP_LOGI(TAG, "msg: Hello\n");

	printf("Starting the Shake Table...\n");
	ESP_LOGI(TAG, "msg: Starting the Shake Table...\n");
	
	vTaskDelay(pdMS_TO_TICKS(3000)); // Delay de 3 segundos

	while (1)
	{	
		testing_led();
		//vTaskDelay(pdMS_TO_TICKS(5000)); // Delay de 5 segundos
	}

}
