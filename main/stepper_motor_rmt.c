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
#include <stdlib.h>
#include "stepper_motor_encoder.h"
#include "functions.h"

#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "DRV8825_RMT";

struct stepper_rmt_context_t {
    uint8_t gpio_en;
    uint8_t gpio_dir;
    uint8_t gpio_step;
    rmt_channel_handle_t motor_chan;
    rmt_encoder_handle_t accel_motor_encoder;
    rmt_encoder_handle_t uniform_motor_encoder;
    rmt_encoder_handle_t decel_motor_encoder;
};

stepper_rmt_context_t* stepper_rmt_init(uint8_t gpio_en, uint8_t gpio_dir, uint8_t gpio_step)
{
    stepper_rmt_context_t *ctx = calloc(1, sizeof(stepper_rmt_context_t));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate memory for stepper context");
        return NULL;
    }

    ctx->gpio_en = gpio_en;
    ctx->gpio_dir = gpio_dir;
    ctx->gpio_step = gpio_step;

    ESP_LOGI(TAG, "Initialize EN + DIR GPIO (EN: %d, DIR: %d, STEP: %d)", gpio_en, gpio_dir, gpio_step);
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << gpio_dir) | (1ULL << gpio_en),
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = gpio_step,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &ctx->motor_chan));

    ESP_LOGI(TAG, "Disable step motor initially");
    gpio_set_level(gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // !0 = 1 (HIGH) -> Desativa o driver

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &ctx->accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &ctx->uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &ctx->decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(ctx->motor_chan));

    return ctx;
}

esp_err_t stepper_rmt_run_steps(stepper_rmt_context_t *ctx, uint32_t uniform_speed_hz, uint32_t uniform_samples, uint32_t accel_samples, uint32_t decel_samples, bool direction)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (direction == true) {
        ESP_LOGI(TAG, "Set spin direction: CLOCKWISE");
        gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    } else {
        ESP_LOGI(TAG, "Set spin direction: COUNTERCLOCKWISE");
        gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
    }

    uint32_t steps = accel_samples + uniform_samples + decel_samples;
    
    ESP_LOGI(TAG, "Spin motor for %d steps: %d accel + %d uniform + %d decel", steps, accel_samples, uniform_samples, decel_samples);
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(ctx->motor_chan, ctx->accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

    for (int i = 0; i < uniform_samples; i++) {
        ESP_ERROR_CHECK(rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
    }

    ESP_ERROR_CHECK(rmt_transmit(ctx->motor_chan, ctx->decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));

    return rmt_tx_wait_all_done(ctx->motor_chan, -1);
}

void stepper_rmt_task_1(void *arg)
{
    // Inicializa o Motor 1 com os pinos originais do código
    stepper_rmt_context_t *motor1 = stepper_rmt_init(5, 32, 4);
    
    while (1) {
        
        // O botão (GPIO 14) tem PULL-UP ativo, ou seja, lê 0 quando premido
        if (motor1 && gpio_get_level(GPIO_NUM_14) == 0) {
            gpio_set_level(GPIO_NUM_25, 1); // Turn LED on
            gpio_set_level(motor1->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Ativa o driver do motor 1
            stepper_rmt_run_steps(motor1, 1500, 5000, 500, 500, true);
            vTaskDelay(pdMS_TO_TICKS(1000));
            stepper_rmt_run_steps(motor1, 1500, 5000, 500, 500, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(motor1->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Desativa o driver após terminar
        }
        
        gpio_set_level(GPIO_NUM_25, 0); // Turn LED off
        // Atraso essencial para libertar o CPU quando o botão não está premido
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void stepper_rmt_task_2(void *arg)
{
    // Inicializa o Motor 2 com outros pinos que queiras utilizar (EN: 15, DIR: 19, STEP: 18)
    stepper_rmt_context_t *motor2 = stepper_rmt_init(19, 15, 18);

    while (1) {
        
        // O botão (GPIO 21) tem PULL-UP ativo, ou seja, lê 0 quando premido
        if (motor2 && gpio_get_level(GPIO_NUM_21) == 0) {
            gpio_set_level(GPIO_NUM_26, 1); // Turn LED on
            gpio_set_level(motor2->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Ativa o driver do motor 2
            stepper_rmt_run_steps(motor2, 1500, 5000, 500, 500, true);
            vTaskDelay(pdMS_TO_TICKS(1000));
            stepper_rmt_run_steps(motor2, 1500, 5000, 500, 500, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(motor2->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Desativa o driver após terminar
        }
        
        gpio_set_level(GPIO_NUM_26, 0); // Turn LED off
        // Atraso essencial para libertar o CPU quando o botão não está premido
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
