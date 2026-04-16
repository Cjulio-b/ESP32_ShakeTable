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
#include "kinematics.h"

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

esp_err_t stepper_rmt_homing(stepper_rmt_context_t *ctx, uint8_t gpio_limit_right, uint8_t gpio_limit_left)
{
    if (!ctx) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Starting HOMING on EN:%d, DIR:%d", ctx->gpio_en, ctx->gpio_dir);

    // Inicializa estrutura de cinemática localmente para efeitos de verificação e debug
    shake_table_config_t my_table;
    kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm de deslocamento, 66mm de biela
    kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32); // 1.8º, 32 microsteps (6400 passos/volta)

    // Configura os Pinos dos Sensores de Fim de Curso
    gpio_config_t limit_conf = {
        .pin_bit_mask = (1ULL << gpio_limit_right) | (1ULL << gpio_limit_left),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Funciona no 22/23. Nos 36/39 precisas de PULL-UP FÍSICO!
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&limit_conf);

    // Ativa o Driver do Motor para o Homing
    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(100)); // Tempo para estabilizar energia nas bobinas

    uint32_t homing_speed_hz = 500; // Velocidade do Homing
    uint32_t chunk_size = 20;        // Verifica o sensor a cada 20 micropassos (~pequena fração de mm)
    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    // --- PASSO 1: Mover à Direita (Horário) até Fim de Curso Direito ---
    ESP_LOGI(TAG, "HOMING: Step 1 - Moving Right (CW) to limit switch (GPIO %d)...", gpio_limit_right);
    gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    
    // Assume que switch envia GND (0) quando pressionado.
    while (gpio_get_level(gpio_limit_right) != 0) {
        for (int i = 0; i < chunk_size; i++) {
            rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &homing_speed_hz, sizeof(homing_speed_hz), &tx_config);
        }
        rmt_tx_wait_all_done(ctx->motor_chan, -1);
    }
    ESP_LOGI(TAG, "HOMING: Right limit hit (+180 deg)!");
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- PASSO 2: Mover à Esquerda (Anti-Horário) até Fim de Curso Esquerdo e Contar Passos ---
    ESP_LOGI(TAG, "HOMING: Step 2 - Moving Left (CCW) to limit switch (GPIO %d)...", gpio_limit_left);
    gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
    uint32_t total_steps = 0;
    
    while (gpio_get_level(gpio_limit_left) != 0) {
        for (int i = 0; i < chunk_size; i++) {
            rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &homing_speed_hz, sizeof(homing_speed_hz), &tx_config);
        }
        rmt_tx_wait_all_done(ctx->motor_chan, -1);
        total_steps += chunk_size;
    }
    ESP_LOGI(TAG, "HOMING: Left limit hit (-180 deg)! Total measured steps = %lu", total_steps);
    
    // --- VERIFICAÇÃO CINEMÁTICA (Fim de Curso a Fim de Curso) ---
    float angulo_total = kinematics_calc_angular_position(&my_table.stepper_x, total_steps);
    ESP_LOGI(TAG, "[Verificação Homing] Movimento de ponta a ponta:");
    ESP_LOGI(TAG, "  -> Passos Medidos: %lu (Teórico p/ 180 graus = %lu)", total_steps, my_table.stepper_x.microsteps_per_rev / 2);
    ESP_LOGI(TAG, "  -> Ângulo Percorrido Calculado: %.2f graus", angulo_total);
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- PASSO 3: Mover à Direita (Horário) pelo Centro Calculado ---
    uint32_t center_steps = total_steps / 2;
    ESP_LOGI(TAG, "HOMING: Step 3 - Centering (Moving CW by %lu steps)...", center_steps);
    gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    
    // Reutilizar a função já existente que lida com a injeção em bloco para ser mais rápido
    stepper_rmt_run_steps(ctx, homing_speed_hz, center_steps, 0, 0, true);
    
    ESP_LOGI(TAG, "HOMING: Calibration Finished. Motor is at Center (0 deg).");

    // --- VERIFICAÇÃO CINEMÁTICA (Centro) ---
    float angulo_centro = kinematics_calc_angular_position(&my_table.stepper_x, center_steps);
    float pos_real_centro = kinematics_calc_linear_position_relative_90(&my_table.axis_x, angulo_centro);
    ESP_LOGI(TAG, "[Verificação Homing] Posição de Centro (ponto morto intermédio):");
    ESP_LOGI(TAG, "  -> Ângulo do Motor: %.2f graus (Esperado ~90.00 graus)", angulo_centro);
    ESP_LOGI(TAG, "  -> Posição Real (Relativa a 90 graus): %.2f mm (Esperado ~0.00 mm)", pos_real_centro);

    // Desliga driver no final para não aquecer, conforme o sistema atual
    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);

    return ESP_OK;
}

void stepper_rmt_task_1(void *arg)
{
    // Inicializa o Motor 1 com os pinos originais do código
    stepper_rmt_context_t *motor1 = stepper_rmt_init(5, 32, 4);
    
    // Executa a calibração de Homing assim que o ESP32 arranca!
    if (motor1) {
        stepper_rmt_homing(motor1, 22, 23);
    }

    while (1) {
        // LARGER STEPPER
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

    // Executa a calibração de Homing assim que o ESP32 arranca!
    if (motor2) {
        stepper_rmt_homing(motor2, 27, 33);
    }

    while (1) {
        // Smaller STEPPER
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
