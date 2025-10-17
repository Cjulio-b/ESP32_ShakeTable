
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "stdio.h"


void check_current_config(void)
{
    /* Show the current configurations of IOs, such as pull-up/pull-down, 
    input/output enable, pin mapping, etc. */

    printf("\n\t---------------- Dumping GPIO configuration ---------------------\n");
    
    //Example dump specific GPIO:
    gpio_dump_io_configuration(stdout, (1ULL << 21) | (1ULL << 26));

    //Example dump all GPIOs:
    //gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    printf("\t---------------- GPIO configuration dumped. ---------------------\n");
}

void testing_led(void)
{
    // Example function to toggle an LED connected to GPIO4

	if(gpio_get_level(GPIO_NUM_21))
	{
		//printf("Button Pressed!\n");
		gpio_set_level(GPIO_NUM_26, 0); // Turn LED on
		vTaskDelay(pdMS_TO_TICKS(10));

	}
	else
	{
		//printf("Button Released!\n");
		gpio_set_level(GPIO_NUM_26, 1); // Turn LED OFF
		vTaskDelay(pdMS_TO_TICKS(10));
	}
    
}

void GPIO_init(void)
{
    /* Configure GPIOs as needed */

    //INPUT GPIO
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT); 
	gpio_set_pull_mode(GPIO_NUM_21, GPIO_PULLUP_ONLY); // Enable pull-up resistor
		
	//OUTPUT GPIOs
	gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
}