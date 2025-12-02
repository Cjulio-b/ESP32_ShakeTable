#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#define A_IN1 GPIO_NUM_13
#define A_IN2 GPIO_NUM_12
#define B_IN1 GPIO_NUM_27
#define B_IN2 GPIO_NUM_33
#define STEP_DELAY_MS 10

static const int seq[4][4] = {

    {1,0,0,0}, // 
    {0,0,1,0}, // 
    {1,0,0,0}, // 
    {0,0,1,0},  // 
};

void step_motor(bool direction)
{
    static int step = 0;

    if (direction)
        step = (step + 1) % 4;
    else
        step = (step + 3) % 4;

    gpio_set_level(A_IN1, seq[step][0]);
    gpio_set_level(A_IN2, seq[step][1]);
    gpio_set_level(B_IN1, seq[step][2]);
    gpio_set_level(B_IN2, seq[step][3]);

    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));  
}
