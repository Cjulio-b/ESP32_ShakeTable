
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "stdio.h"
#include "rom/ets_sys.h" // ets_delay_us

extern void step_motor(bool direction);

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
		gpio_set_level(GPIO_NUM_26, 0); // Turn LED off
		//step_motor(true); // Step motor in one direction
	}
	else
	{
		//printf("Button Released!\n");
		gpio_set_level(GPIO_NUM_26, 1); // Turn LED on
		//step_motor(true); // Step motor in one direction
		printf("Stepper Motor - Wave Drive mode...\n");
		// One fase is activated at a time - Step angle is large, rotation not smooth and torque low
		for(int i=0; i<1000; i++){
			gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
		}
		vTaskDelay(pdMS_TO_TICKS(1000)); // 1s delay
		printf("Stepper Motor - FULL Step mode...\n");
		gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
	    gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
		gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
		gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
		// Two fases are activated at a time - Average of the coils, torque is higher.
		for(int i=0; i<1000; i++){
			gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
		}
		vTaskDelay(pdMS_TO_TICKS(1000)); // 1s delay
		printf("Stepper Motor - HALF Step mode...\n");
		// Alternates between one and two fases activated - Step angle is smaller, rotation is smoother, torque very good
		for(int i=0; i<200; i++){
			gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay
			gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay	
			gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
			gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
			gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
			gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
			//vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			ets_delay_us(1000); // 1ms delay		
		}
		vTaskDelay(pdMS_TO_TICKS(1000)); // 1s delay
		printf("Stepper Motor - Micro step mode... (Not working)\n");
		gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
	    gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
		gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
		gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
		// Microstepping provides very smooth and precise motion by controlling the current in each coil, almost sinusoidally.
		/* for(int i=0; i<100; i++){
			for(int j=0; j<3; j++){
				gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
				gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
				gpio_set_level(GPIO_NUM_13, 0);	// A_IN1
				gpio_set_level(GPIO_NUM_12, 1);	// A_IN2
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			}
			for(int j=0; j<3; j++){	
				gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
				gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
				gpio_set_level(GPIO_NUM_12, 0);	// A_IN2
				gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			}
			for(int j=0; j<3; j++){	
				gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
				gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
				gpio_set_level(GPIO_NUM_27, 1);	// B_IN1
				gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			}
			for(int j=0; j<3; j++){
				gpio_set_level(GPIO_NUM_13, 1);	// A_IN1
				gpio_set_level(GPIO_NUM_33, 0);	// B_IN2
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
				gpio_set_level(GPIO_NUM_27, 0);	// B_IN1
				gpio_set_level(GPIO_NUM_33, 1);	// B_IN2
				vTaskDelay(pdMS_TO_TICKS(50)); // 0,5s delay
			}
		}
		vTaskDelay(pdMS_TO_TICKS(1000)); // 1s delay	 */
	}
    
}

void GPIO_init(void)
{
    /* Configure GPIOs as needed */

    //INPUT GPIO
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT);  // Button LED
	gpio_set_pull_mode(GPIO_NUM_21, GPIO_PULLUP_ONLY); // Enable pull-up resistor

	gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT); // Button Stepper Motor
	gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY); // Enable pull-up resistor
		
	//OUTPUT GPIOs
	gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);

	//Stepper Motor GPIOs
	gpio_config_t io_conf = {
    	.mode = GPIO_MODE_OUTPUT,
    	.pin_bit_mask = (1ULL<<GPIO_NUM_13) | (1ULL<<GPIO_NUM_12) | (1ULL<<GPIO_NUM_27) | (1ULL<<GPIO_NUM_33),
	};
	gpio_config(&io_conf);

}