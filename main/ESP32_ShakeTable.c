#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MyModule";

void app_main(void)
{
	while (1)
	{
		printf("Hello, this is ESP32 Speaking!\n");
		ESP_LOGI(TAG, "msg: Hello\n");

		printf("Starting the Shake Table...\n");
		ESP_LOGI(TAG, "msg: Starting the Shake Table...\n");

		vTaskDelay(pdMS_TO_TICKS(2000)); // Delay de 2 segundos
		printf("Shake Table is now running!\n");
		ESP_LOGI(TAG, "msg: Starting the Shake Table...\n");

		vTaskDelay(pdMS_TO_TICKS(5000)); // Delay de 5 segundos
	}

}
