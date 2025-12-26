/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"

///////////////////////////////Change the following configurations according to your board//////////////////////////////
#define STEP_MOTOR_GPIO_EN       5//0
#define STEP_MOTOR_GPIO_DIR      32//2
#define STEP_MOTOR_GPIO_STEP     4
#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "DRV8825_RMT";

static rmt_channel_handle_t motor_chan;
static rmt_encoder_handle_t accel_motor_encoder;
static rmt_encoder_handle_t uniform_motor_encoder;
static rmt_encoder_handle_t decel_motor_encoder;

void stepper_rmt_init(void)
{
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    // ESP_LOGI(TAG, "Set spin direction");
    // gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
}

esp_err_t stepper_rmt_run_steps(uint32_t uniform_speed_hz, uint32_t uniform_samples, uint32_t accel_samples, uint32_t decel_samples, bool direction)
{
    if (direction == true) {
        ESP_LOGI(TAG, "Set spin direction: CLOCKWISE");
        gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    } else {
        ESP_LOGI(TAG, "Set spin direction: COUNTERCLOCKWISE");
        gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
    }

    uint32_t steps = accel_samples + uniform_samples + decel_samples;
    
    ESP_LOGI(TAG, "Spin motor for %d steps: %d accel + %d uniform + %d decel", steps, accel_samples, uniform_samples, decel_samples);
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

    for (int i = 0; i < uniform_samples; i++) {
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
    }

    ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));

    return rmt_tx_wait_all_done(motor_chan, -1);
}

void stepper_rmt_task(void *arg) //void app_main(void)
{
    stepper_rmt_init();
    while (1) {
        
        stepper_rmt_run_steps(1500, 5000, 500, 500, true);

        vTaskDelay(pdMS_TO_TICKS(1000));

        stepper_rmt_run_steps(1500, 5000, 500, 500, false);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
