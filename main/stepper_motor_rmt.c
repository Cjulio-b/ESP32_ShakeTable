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
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <math.h>
#include "stepper_motor_encoder.h"
#include "functions.h"
#include "kinematics.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"

#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
#define PI_MATH 3.14159265358979323846f

static const char *TAG = "DRV8825_RMT";

// Usar 'volatile' avisa o compilador que esta variável pode ser alterada
// a qualquer momento por outra Task (neste caso, a rx_task do UART/Nextion)
volatile int8_t nextion_profile = 0; // 0 significa repouso absoluto (A aguardar comando do HMI)
volatile bool motor1_busy = false;
volatile bool motor2_busy = false;
volatile bool motor1_ready = false;
volatile bool motor2_ready = false;

// Handle global partilhado do ADC1 para ambas as tasks
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

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
        .trans_queue_depth = 100, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &ctx->motor_chan));

    ESP_LOGI(TAG, "Disable stepper motor driver initially");
    gpio_set_level(gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // !0 = 1 (HIGH) -> Disables the driver

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

    // Initialize local kinematics structure for debugging and verification purposes
    shake_table_config_t my_table;
    kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
    kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32); // 1.8 degree step, 32 microsteps (6400 steps/rev)

    // Configure limit switch pins as inputs with pull-ups
    gpio_config_t limit_conf = {
        .pin_bit_mask = (1ULL << gpio_limit_right) | (1ULL << gpio_limit_left),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Note: GPIOs 34-39 do not have internal pull-ups, external resistors required!
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&limit_conf);

    // Enable the motor driver for homing
    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for coil current to stabilize

    uint32_t homing_speed_hz = 500; // Homing speed
    uint32_t chunk_size = 20;       // Check limit switch every 20 microsteps (small fraction of a mm)
    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    // --- STEP 1: Move Right (Clockwise) until the right limit switch is hit ---
    ESP_LOGI(TAG, "HOMING: Step 1 - Moving Right (CW) to limit switch (GPIO %d)...", gpio_limit_right);
    gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    
    // Assume the switch pulls to GND (0V) when pressed
    while (gpio_get_level(gpio_limit_right) != 0) {
        for (int i = 0; i < chunk_size; i++) {
            rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &homing_speed_hz, sizeof(homing_speed_hz), &tx_config);
        }
        rmt_tx_wait_all_done(ctx->motor_chan, -1);
    }
    ESP_LOGI(TAG, "HOMING: Right limit hit (+180 deg)!");
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- STEP 2: Move Left (Counter-Clockwise) to the left limit switch and count steps ---
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
    
    // --- KINEMATICS VERIFICATION (End-to-End) ---
    float total_angle = kinematics_calc_angular_position(&my_table.stepper_x, total_steps);
    ESP_LOGI(TAG, "[Homing Verification] End-to-end movement:");
    ESP_LOGI(TAG, "  -> Measured Steps: %lu (Theoretical for 180 deg = %lu)", total_steps, my_table.stepper_x.microsteps_per_rev / 2);
    ESP_LOGI(TAG, "  -> Calculated Travelled Angle: %.2f degrees", total_angle);
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // --- STEP 3: Move Right (Clockwise) to the calculated center position ---
    uint32_t center_steps = total_steps / 2;
    ESP_LOGI(TAG, "HOMING: Step 3 - Centering (Moving CW by %lu steps)...", center_steps);
    gpio_set_level(ctx->gpio_dir, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    
    // Reuse the block execution function to move to the center quickly
    stepper_rmt_run_steps(ctx, homing_speed_hz, center_steps, 0, 0, true);
    
    ESP_LOGI(TAG, "HOMING: Calibration Finished. Motor is at Center (0 deg).");

    // --- KINEMATICS VERIFICATION (Center) ---
    float center_angle = kinematics_calc_angular_position(&my_table.stepper_x, center_steps);
    float real_center_pos = kinematics_calc_linear_position_relative_90(&my_table.axis_x, center_angle);
    ESP_LOGI(TAG, "[Homing Verification] Center Position (intermediate dead center):");
    ESP_LOGI(TAG, "  -> Motor Angle: %.2f degrees (Expected ~90.00 degrees)", center_angle);
    ESP_LOGI(TAG, "  -> Real Position (Relative to 90 deg): %.2f mm (Expected ~0.00 mm)", real_center_pos);

    // Disable the motor driver to prevent overheating while idle
    //gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);

    return ESP_OK;
}

esp_err_t stepper_rmt_run_sine_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float freq_hz, float duration_s, const shake_table_config_t *table_config)
{
    if (!ctx || !table_config) return ESP_ERR_INVALID_ARG;

    float max_p2p = table_config->axis_x.peak_to_peak_disp_mm; // Typical max displacement (e.g., 33.0mm)
    
    // Safety check: clamp requested displacement to the physical table maximum
    if (target_p2p_mm > max_p2p) {
        ESP_LOGW(TAG, "Warning: Requested displacement (%.2fmm) exceeds physical limit (%.2fmm). Clamping to maximum.", target_p2p_mm, max_p2p);
        target_p2p_mm = max_p2p;
    }

    ESP_LOGI(TAG, "Sinusoidal Seismic Profile: P2P=%.2fmm, Freq=%.2fHz, Dur=%.2fs", target_p2p_mm, freq_hz, duration_s);

    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Enable driver
    vTaskDelay(pdMS_TO_TICKS(50));
    
    int64_t start_time_us = esp_timer_get_time();
    int64_t duration_us = (int64_t)(duration_s * 1000000.0f);

    // CASE 1: Continuous Rotation (Maximum Amplitude)
    // For max P2P (~33mm), the crank-slider mechanism translates continuous rotation into full linear strokes.
    if (target_p2p_mm >= max_p2p - 0.1f) {
        ESP_LOGI(TAG, "Continuous Rotation Mode (Crank-slider naturally actuates the full stroke)");
        uint32_t speed_hz = (uint32_t)(table_config->stepper_x.microsteps_per_rev * freq_hz);
        uint32_t chunk_steps = speed_hz / 10; // Fragmentos de movimento (aprox 100ms) para podermos parar a qualquer instante
        if (chunk_steps == 0) chunk_steps = 1;
        
        while (esp_timer_get_time() - start_time_us < duration_us) {
            if (nextion_profile == 0) break;
            stepper_rmt_run_steps(ctx, speed_hz, chunk_steps, 0, 0, true);
        }
    } 
    // CASE 2: Partial Oscillation (e.g., 10mm)
    // The motor oscillates +/- X degrees from the center point.
    else {
        ESP_LOGI(TAG, "Partial Oscillation Mode (Motor reverses direction to achieve %.2fmm p2p)", target_p2p_mm);
        
        float r = table_config->axis_x.max_amplitude_mm; // Crank radius (half of maximum table displacement)
        float A = target_p2p_mm / 2.0f;                  // Desired peak amplitude from the center (e.g., 5mm)
        
        // Calculate required mechanical angle from the center (in radians) using simple inverse kinematics
        float theta_rad = asinf(A / r);
        
        // Exact motor steps required to reach the peak of this oscillation amplitude
        float s_amp = (theta_rad / (2.0f * PI_MATH)) * table_config->stepper_x.microsteps_per_rev;

        // Velocity profile generation for 1/4 of the wave cycle (0 to peak)
        #define Q_SEGMENTS 20
        uint32_t q_steps[Q_SEGMENTS];
        uint32_t q_speeds[Q_SEGMENTS];

        float T = 1.0f / freq_hz;
        float dt = (T / 4.0f) / Q_SEGMENTS;
        float accum = 0.0f;

        for (int i = 0; i < Q_SEGMENTS; i++) {
            float tau1 = (float)i / Q_SEGMENTS;
            float tau2 = (float)(i + 1) / Q_SEGMENTS;

            // The position progresses sinusoidally over time
            float s1 = s_amp * sinf((PI_MATH / 2.0f) * tau1);
            float s2 = s_amp * sinf((PI_MATH / 2.0f) * tau2);
            float ds = s2 - s1; // Step delta for this specific segment

            accum += ds;
            uint32_t steps = (uint32_t)floorf(accum);
            accum -= steps; // Accumulate rounding fractional errors to prevent step loss

            q_steps[i] = steps;
            if (steps > 0) {
                q_speeds[i] = (uint32_t)((float)steps / dt);
                if (q_speeds[i] < 10) q_speeds[i] = 10; // RMT safety limit: minimum frequency
            } else {
                q_speeds[i] = 0;
            }
        }

        rmt_transmit_config_t tx_config = { .loop_count = 0 };
        int q = 0;

        // Execute the profile quarter-by-quarter
        while (1) {
            // Verificação de tempo absoluta sugerida por ti (corte exato no milissegundo)
            if (esp_timer_get_time() - start_time_us >= duration_us) {
                ESP_LOGI(TAG, "Tempo limite exato atingido (%.2fs)! Ensaio finalizado.", duration_s);
                break;
            }

            // Verifica se o ensaio foi abortado (Botão STOP no Nextion envia nextion_profile = 0)
            if (nextion_profile == 0) {
                ESP_LOGW(TAG, "Sine Profile abortado a meio do ensaio!");
                break;
            }
            // q=0: Forward (Moving away from center to positive peak)
            // q=1: Reverse (Returning to center)
            // q=2: Reverse (Moving away from center to negative peak)
            // q=3: Forward (Returning to center)
            bool is_cw = (q % 4 == 0) || (q % 4 == 3);
            bool is_decel = (q % 4 == 0) || (q % 4 == 2); // Decelerate whenever moving away from the center

            gpio_set_level(ctx->gpio_dir, is_cw ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);

            for (int i = 0; i < Q_SEGMENTS; i++) {
                int idx = is_decel ? i : (Q_SEGMENTS - 1 - i); // Reverse speed array reading order if accelerating
                uint32_t steps = q_steps[idx];
                if (steps > 0 && q_speeds[idx] > 0) {
                    // In RMT Uniform mode, 'rmt_transmit' must be called N times for N steps
                    for (uint32_t s = 0; s < steps; s++) {
                        rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &q_speeds[idx], sizeof(q_speeds[idx]), &tx_config);
                    }
                }
            }
            // Wait for this movement to finish before reversing direction
            rmt_tx_wait_all_done(ctx->motor_chan, -1);
            q++;
        }
    }

    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Disable the driver
    ESP_LOGI(TAG, "Sinusoidal Seismic Profile Completed!");
    return ESP_OK;
}

esp_err_t stepper_rmt_run_realtime_sine_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float duration_s, adc_oneshot_unit_handle_t adc_handle, adc_channel_t adc_chan, float min_hz, float max_hz, const shake_table_config_t *table_config)
{
    if (!ctx || !table_config || !adc_handle) return ESP_ERR_INVALID_ARG;

    float max_p2p = table_config->axis_x.peak_to_peak_disp_mm;
    if (target_p2p_mm > max_p2p) target_p2p_mm = max_p2p;

    ESP_LOGI(TAG, "Real-Time Seismic Profile: P2P=%.2fmm, Dur=%.2fs, Freq=%.2f to %.2f Hz", target_p2p_mm, duration_s, min_hz, max_hz);

    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));

    int64_t start_time_us = esp_timer_get_time();
    int64_t duration_us = (int64_t)(duration_s * 1000000.0f);
    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    // CASE 1: Rotação Contínua (Curso Máximo)
    if (target_p2p_mm >= max_p2p - 0.1f) {
        while (esp_timer_get_time() - start_time_us < duration_us) {
            int adc_val = 0;
            adc_oneshot_read(adc_handle, adc_chan, &adc_val);
            
            // Mapeia ADC (0-4095) para a gama de Frequências configurada
            float freq_hz = min_hz + ((float)adc_val / 4095.0f) * (max_hz - min_hz);
            uint32_t speed_hz = (uint32_t)(table_config->stepper_x.microsteps_per_rev * freq_hz);
            
            // Transmite um pequeno bloco (aprox 100ms) à velocidade atual antes de ler novamente
            uint32_t chunk_steps = (uint32_t)(speed_hz * 0.1f);
            if(chunk_steps == 0) chunk_steps = 1;

            stepper_rmt_run_steps(ctx, speed_hz, chunk_steps, 0, 0, true);
        }
    } 
    // CASE 2: Oscilação Parcial
    else {
        float r = table_config->axis_x.max_amplitude_mm;
        float A = target_p2p_mm / 2.0f;
        float theta_rad = asinf(A / r);
        float s_amp = (theta_rad / (2.0f * PI_MATH)) * table_config->stepper_x.microsteps_per_rev;

        #define Q_SEGMENTS 20
        uint32_t q_steps[Q_SEGMENTS];
        uint32_t q_speeds[Q_SEGMENTS];

        float accum = 0.0f; // Mantido fora do loop para não perder precisão inter-quartos
        int q = 0;

        while (1) {
            // Verifica se o tempo acabou APENAS quando regressa ao centro (evita parar com a mesa de lado)
            if (q > 0 && (q % 4 == 0)) {
                if (esp_timer_get_time() - start_time_us >= duration_us) {
                    break;
                }
            }

            int adc_val = 0;
            adc_oneshot_read(adc_handle, adc_chan, &adc_val);
            float freq_hz = min_hz + ((float)adc_val / 4095.0f) * (max_hz - min_hz);

            float T = 1.0f / freq_hz;
            float dt = (T / 4.0f) / Q_SEGMENTS;

            for (int i = 0; i < Q_SEGMENTS; i++) {
                float tau1 = (float)i / Q_SEGMENTS;
                float tau2 = (float)(i + 1) / Q_SEGMENTS;
                float s1 = s_amp * sinf((PI_MATH / 2.0f) * tau1);
                float s2 = s_amp * sinf((PI_MATH / 2.0f) * tau2);
                
                accum += (s2 - s1);
                uint32_t steps = (uint32_t)floorf(accum);
                accum -= steps;

                q_steps[i] = steps;
                if (steps > 0) {
                    q_speeds[i] = (uint32_t)((float)steps / dt);
                    if (q_speeds[i] < 10) q_speeds[i] = 10;
                } else {
                    q_speeds[i] = 0;
                }
            }

            bool is_cw = (q % 4 == 0) || (q % 4 == 3);
            bool is_decel = (q % 4 == 0) || (q % 4 == 2);

            gpio_set_level(ctx->gpio_dir, is_cw ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);

            for (int i = 0; i < Q_SEGMENTS; i++) {
                int idx = is_decel ? i : (Q_SEGMENTS - 1 - i);
                if (q_steps[idx] > 0 && q_speeds[idx] > 0) {
                    for (uint32_t s = 0; s < q_steps[idx]; s++) {
                        rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &q_speeds[idx], sizeof(q_speeds[idx]), &tx_config);
                    }
                }
            }
            rmt_tx_wait_all_done(ctx->motor_chan, -1);
            q++;
        }
    }

    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
    ESP_LOGI(TAG, "Real-Time Seismic Profile Completed!");
    return ESP_OK;
}

esp_err_t stepper_rmt_run_trapezoidal_freq_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float start_freq_hz, float cruise_freq_hz, float accel_time_s, float cruise_time_s, float decel_time_s, const shake_table_config_t *table_config)
{
    if (!ctx || !table_config) return ESP_ERR_INVALID_ARG;

    float max_p2p = table_config->axis_x.peak_to_peak_disp_mm;
    if (target_p2p_mm > max_p2p) target_p2p_mm = max_p2p;

    float total_time_s = accel_time_s + cruise_time_s + decel_time_s;
    ESP_LOGI(TAG, "Trapezoidal Freq Profile: P2P=%.2fmm, Freq=%.2f->%.2fHz, Time=%.2fs (A=%.1f, C=%.1f, D=%.1f)",
             target_p2p_mm, start_freq_hz, cruise_freq_hz, total_time_s, accel_time_s, cruise_time_s, decel_time_s);

    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));

    int64_t start_time_us = esp_timer_get_time();
    int64_t total_duration_us = (int64_t)(total_time_s * 1000000.0f);
    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    // Rotação Contínua (Amplitude Máxima)
    if (target_p2p_mm >= max_p2p - 0.1f) {
        while (1) {
            int64_t elapsed_us = esp_timer_get_time() - start_time_us;
            if (elapsed_us >= total_duration_us) break;

            float t = (float)elapsed_us / 1000000.0f;
            float freq_hz = start_freq_hz;
            
            if (t < accel_time_s) {
                freq_hz = start_freq_hz + (cruise_freq_hz - start_freq_hz) * (t / accel_time_s);
            } else if (t < accel_time_s + cruise_time_s) {
                freq_hz = cruise_freq_hz;
            } else {
                float decel_t = t - accel_time_s - cruise_time_s;
                freq_hz = cruise_freq_hz - (cruise_freq_hz - start_freq_hz) * (decel_t / decel_time_s);
            }
            
            if (freq_hz < 0.1f) freq_hz = 0.1f; // Prevenir divisão por 0 e limites RMT
            uint32_t speed_hz = (uint32_t)(table_config->stepper_x.microsteps_per_rev * freq_hz);
            uint32_t chunk_steps = (uint32_t)(speed_hz * 0.1f);
            if(chunk_steps == 0) chunk_steps = 1;
            stepper_rmt_run_steps(ctx, speed_hz, chunk_steps, 0, 0, true);
        }
    } 
    // Oscilação Parcial
    else {
        float r = table_config->axis_x.max_amplitude_mm;
        float A = target_p2p_mm / 2.0f;
        float s_amp = (asinf(A / r) / (2.0f * PI_MATH)) * table_config->stepper_x.microsteps_per_rev;

        #define Q_SEGMENTS 20
        uint32_t q_steps[Q_SEGMENTS];
        uint32_t q_speeds[Q_SEGMENTS];
        float accum = 0.0f;
        int q = 0;

        while (1) {
            int64_t elapsed_us = esp_timer_get_time() - start_time_us;
            if (q > 0 && (q % 4 == 0) && elapsed_us >= total_duration_us) break;

            float t = (float)elapsed_us / 1000000.0f;
            float freq_hz = start_freq_hz;
            
            if (t < accel_time_s) {
                freq_hz = start_freq_hz + (cruise_freq_hz - start_freq_hz) * (t / accel_time_s);
            } else if (t < accel_time_s + cruise_time_s) {
                freq_hz = cruise_freq_hz;
            } else if (t < total_time_s) {
                float decel_t = t - accel_time_s - cruise_time_s;
                freq_hz = cruise_freq_hz - (cruise_freq_hz - start_freq_hz) * (decel_t / decel_time_s);
            }
            if (freq_hz < 0.1f) freq_hz = 0.1f;

            float dt = (1.0f / freq_hz / 4.0f) / Q_SEGMENTS;
            for (int i = 0; i < Q_SEGMENTS; i++) {
                float s1 = s_amp * sinf((PI_MATH / 2.0f) * ((float)i / Q_SEGMENTS));
                float s2 = s_amp * sinf((PI_MATH / 2.0f) * ((float)(i + 1) / Q_SEGMENTS));
                accum += (s2 - s1);
                uint32_t steps = (uint32_t)floorf(accum);
                accum -= steps;
                q_steps[i] = steps;
                q_speeds[i] = steps > 0 ? (uint32_t)((float)steps / dt) : 0;
                if (q_speeds[i] > 0 && q_speeds[i] < 10) q_speeds[i] = 10;
            }

            gpio_set_level(ctx->gpio_dir, (q % 4 == 0 || q % 4 == 3) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
            bool is_decel = (q % 4 == 0 || q % 4 == 2);
            for (int i = 0; i < Q_SEGMENTS; i++) {
                int idx = is_decel ? i : (Q_SEGMENTS - 1 - i);
                for (uint32_t s = 0; s < q_steps[idx]; s++)
                    rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &q_speeds[idx], sizeof(q_speeds[idx]), &tx_config);
            }
            rmt_tx_wait_all_done(ctx->motor_chan, -1);
            q++;
        }
    }
    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
    ESP_LOGI(TAG, "Trapezoidal Freq Profile Completed!");
    return ESP_OK;
}

esp_err_t stepper_rmt_run_multistep_freq_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, const float *freqs_hz, const float *times_s, uint8_t num_stages, float total_duration_s, float blend_time_s, const shake_table_config_t *table_config)
{
    if (!ctx || !table_config || !freqs_hz || !times_s || num_stages == 0) return ESP_ERR_INVALID_ARG;

    float max_p2p = table_config->axis_x.peak_to_peak_disp_mm;
    if (target_p2p_mm > max_p2p) target_p2p_mm = max_p2p;

    ESP_LOGI(TAG, "Multi-Step Freq Profile: P2P=%.2fmm, Stages=%d, TotalDur=%.2fs, Blend=%.2fs",
             target_p2p_mm, num_stages, total_duration_s, blend_time_s);

    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));

    int64_t start_time_us = esp_timer_get_time();
    int64_t total_duration_us = (int64_t)(total_duration_s * 1000000.0f);
    int64_t blend_duration_us = (int64_t)(blend_time_s * 1000000.0f);

    uint8_t stage_idx = 0;
    int64_t stage_start_us = start_time_us;
    int64_t current_stage_duration_us = (int64_t)(times_s[0] * 1000000.0f);
    
    float prev_freq_hz = 0.1f; // Ramp up from near-zero initially for safety
    float current_freq = 0.1f;
    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    // CASE 1: Continuous Rotation (Maximum Amplitude)
    if (target_p2p_mm >= max_p2p - 0.1f) {
        while (1) {
            int64_t current_time = esp_timer_get_time();
            int64_t elapsed_total_us = current_time - start_time_us;
            if (elapsed_total_us >= total_duration_us) break;

            int64_t elapsed_in_stage_us = current_time - stage_start_us;
            if (elapsed_in_stage_us >= current_stage_duration_us) {
                stage_idx = (stage_idx + 1) % num_stages;
                stage_start_us = current_time;
                current_stage_duration_us = (int64_t)(times_s[stage_idx] * 1000000.0f);
                prev_freq_hz = current_freq; // Store actual frequency before stage jump
                elapsed_in_stage_us = 0;
            }

            float freq_hz = freqs_hz[stage_idx];
            if (elapsed_in_stage_us < blend_duration_us) {
                float t_blend = (float)elapsed_in_stage_us / (float)blend_duration_us;
                freq_hz = prev_freq_hz + (freqs_hz[stage_idx] - prev_freq_hz) * t_blend;
            }
            if (freq_hz < 0.1f) freq_hz = 0.1f; // Safety limit
            current_freq = freq_hz;

            uint32_t speed_hz = (uint32_t)(table_config->stepper_x.microsteps_per_rev * freq_hz);
            uint32_t chunk_steps = (uint32_t)(speed_hz * 0.1f);
            if (chunk_steps == 0) chunk_steps = 1;
            stepper_rmt_run_steps(ctx, speed_hz, chunk_steps, 0, 0, true);
        }
    } 
    // CASE 2: Partial Oscillation
    else {
        float r = table_config->axis_x.max_amplitude_mm;
        float A = target_p2p_mm / 2.0f;
        float s_amp = (asinf(A / r) / (2.0f * PI_MATH)) * table_config->stepper_x.microsteps_per_rev;

        #define Q_SEGMENTS 20
        uint32_t q_steps[Q_SEGMENTS];
        uint32_t q_speeds[Q_SEGMENTS];
        float accum = 0.0f;
        int q = 0;

        while (1) {
            int64_t current_time = esp_timer_get_time();
            int64_t elapsed_total_us = current_time - start_time_us;

            // Sync stage jumps and termination ONLY when passing through table center (q % 4 == 0)
            if (q > 0 && (q % 4 == 0)) {
                if (elapsed_total_us >= total_duration_us) break;

                int64_t elapsed_in_stage_us = current_time - stage_start_us;
                if (elapsed_in_stage_us >= current_stage_duration_us) {
                    stage_idx = (stage_idx + 1) % num_stages;
                    stage_start_us = current_time;
                    current_stage_duration_us = (int64_t)(times_s[stage_idx] * 1000000.0f);
                    prev_freq_hz = current_freq; // Store actual frequency before stage jump
                }
            }

            int64_t elapsed_in_stage_us = current_time - stage_start_us;
            float freq_hz = freqs_hz[stage_idx];
            if (elapsed_in_stage_us < blend_duration_us) {
                float t_blend = (float)elapsed_in_stage_us / (float)blend_duration_us;
                freq_hz = prev_freq_hz + (freqs_hz[stage_idx] - prev_freq_hz) * t_blend;
            }
            if (freq_hz < 0.1f) freq_hz = 0.1f; // Safety limit
            current_freq = freq_hz;

            float dt = (1.0f / freq_hz / 4.0f) / Q_SEGMENTS;
            for (int i = 0; i < Q_SEGMENTS; i++) {
                float s1 = s_amp * sinf((PI_MATH / 2.0f) * ((float)i / Q_SEGMENTS));
                float s2 = s_amp * sinf((PI_MATH / 2.0f) * ((float)(i + 1) / Q_SEGMENTS));
                accum += (s2 - s1);
                uint32_t steps = (uint32_t)floorf(accum);
                accum -= steps;
                q_steps[i] = steps;
                q_speeds[i] = steps > 0 ? (uint32_t)((float)steps / dt) : 0;
                if (q_speeds[i] > 0 && q_speeds[i] < 10) q_speeds[i] = 10;
            }

            gpio_set_level(ctx->gpio_dir, (q % 4 == 0 || q % 4 == 3) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
            bool is_decel = (q % 4 == 0 || q % 4 == 2);
            for (int i = 0; i < Q_SEGMENTS; i++) {
                int idx = is_decel ? i : (Q_SEGMENTS - 1 - i);
                for (uint32_t s = 0; s < q_steps[idx]; s++) {
                    rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &q_speeds[idx], sizeof(q_speeds[idx]), &tx_config);
                }
            }
            rmt_tx_wait_all_done(ctx->motor_chan, -1);
            q++;
        }
    }
    
    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
    ESP_LOGI(TAG, "Multi-Step Freq Profile Completed!");
    return ESP_OK;
}

esp_err_t stepper_rmt_run_sweep_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float start_freq_hz, float end_freq_hz, float duration_s, bool is_bidirectional, const shake_table_config_t *table_config)
{
    if (!ctx || !table_config || start_freq_hz <= 0 || end_freq_hz <= 0 || duration_s <= 0) return ESP_ERR_INVALID_ARG;

    float max_p2p = table_config->axis_x.peak_to_peak_disp_mm;
    if (target_p2p_mm > max_p2p) target_p2p_mm = max_p2p;

    ESP_LOGI(TAG, "Logarithmic Sweep Profile: P2P=%.2fmm, Freq=%.2f->%.2fHz, Dur=%.2fs, Bidir=%d",
             target_p2p_mm, start_freq_hz, end_freq_hz, duration_s, is_bidirectional);

    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));

    int64_t start_time_us = esp_timer_get_time();
    int64_t total_duration_us = is_bidirectional ? (int64_t)(duration_s * 2.0f * 1000000.0f) : (int64_t)(duration_s * 1000000.0f);
    rmt_transmit_config_t tx_config = { .loop_count = 0 };

    // CASE 1: Continuous Rotation (Maximum Amplitude)
    if (target_p2p_mm >= max_p2p - 0.1f) {
        while (1) {
            int64_t elapsed_us = esp_timer_get_time() - start_time_us;
            if (elapsed_us >= total_duration_us) break;

            float t = (float)elapsed_us / 1000000.0f;
            if (is_bidirectional) {
                if (t > duration_s) t = (2.0f * duration_s) - t; // Descending phase
            }

            float norm_t = t / duration_s;
            if (norm_t > 1.0f) norm_t = 1.0f;

            // Always calculate frequency strictly Logarithmic
            float freq_hz = start_freq_hz * powf(end_freq_hz / start_freq_hz, norm_t);
            
            if (freq_hz < 0.1f) freq_hz = 0.1f;
            uint32_t speed_hz = (uint32_t)(table_config->stepper_x.microsteps_per_rev * freq_hz);
            uint32_t chunk_steps = (uint32_t)(speed_hz * 0.1f);
            if(chunk_steps == 0) chunk_steps = 1;
            stepper_rmt_run_steps(ctx, speed_hz, chunk_steps, 0, 0, true);
        }
    } 
    // CASE 2: Partial Oscillation
    else {
        float r = table_config->axis_x.max_amplitude_mm;
        float A = target_p2p_mm / 2.0f;
        float s_amp = (asinf(A / r) / (2.0f * PI_MATH)) * table_config->stepper_x.microsteps_per_rev;

        #define Q_SEGMENTS 20
        uint32_t q_steps[Q_SEGMENTS];
        uint32_t q_speeds[Q_SEGMENTS];
        float accum = 0.0f;
        int q = 0;

        while (1) {
            int64_t elapsed_us = esp_timer_get_time() - start_time_us;
            if (q > 0 && (q % 4 == 0) && elapsed_us >= total_duration_us) break;

            float t = (float)elapsed_us / 1000000.0f;
            if (is_bidirectional) {
                if (t > duration_s) t = (2.0f * duration_s) - t; // Descending phase
            }

            float norm_t = t / duration_s;
            if (norm_t > 1.0f) norm_t = 1.0f;

            // Always calculate frequency strictly Logarithmic
            float freq_hz = start_freq_hz * powf(end_freq_hz / start_freq_hz, norm_t);
            if (freq_hz < 0.1f) freq_hz = 0.1f;

            float dt = (1.0f / freq_hz / 4.0f) / Q_SEGMENTS;
            for (int i = 0; i < Q_SEGMENTS; i++) {
                float s1 = s_amp * sinf((PI_MATH / 2.0f) * ((float)i / Q_SEGMENTS));
                float s2 = s_amp * sinf((PI_MATH / 2.0f) * ((float)(i + 1) / Q_SEGMENTS));
                accum += (s2 - s1);
                uint32_t steps = (uint32_t)floorf(accum);
                accum -= steps;
                q_steps[i] = steps;
                q_speeds[i] = steps > 0 ? (uint32_t)((float)steps / dt) : 0;
                if (q_speeds[i] > 0 && q_speeds[i] < 10) q_speeds[i] = 10;
            }

            gpio_set_level(ctx->gpio_dir, (q % 4 == 0 || q % 4 == 3) ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
            bool is_decel = (q % 4 == 0 || q % 4 == 2);
            for (int i = 0; i < Q_SEGMENTS; i++) {
                int idx = is_decel ? i : (Q_SEGMENTS - 1 - i);
                for (uint32_t s = 0; s < q_steps[idx]; s++) rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &q_speeds[idx], sizeof(q_speeds[idx]), &tx_config);
            }
            rmt_tx_wait_all_done(ctx->motor_chan, -1);
            q++;
        }
    }
    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
    ESP_LOGI(TAG, "Logarithmic Sweep Profile Completed!");
    return ESP_OK;
}

// =========================================================================
// FUNÇÕES AUXILIARES PARA O CASE 6 (LEITURA DE FICHEIROS SÍSMICOS)
// =========================================================================

// Procura e devolve o caminho do primeiro ficheiro .bin no disco
static bool get_stored_sismo_file(char* filepath_out, size_t max_len) {
    DIR *dir = opendir("/storage");
    if (!dir) return false;
    
    struct dirent *ent;
    bool found = false;
    while ((ent = readdir(dir)) != NULL) {
        if (strstr(ent->d_name, ".bin") != NULL) {
            snprintf(filepath_out, max_len, "/storage/%s", ent->d_name);
            found = true;
            break;
        }
    }
    closedir(dir);
    return found;
}

// Executa o perfil sísmico a partir do ficheiro
esp_err_t stepper_rmt_run_file_profile(stepper_rmt_context_t *ctx, const char* filepath, const shake_table_config_t *table_config)
{
    if (!ctx || !table_config || !filepath) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(filepath, "r");
    if (!f) {
        ESP_LOGE(TAG, "Falha ao abrir %s", filepath);
        return ESP_FAIL;
    }

    uint32_t num_points = 0;
    float dt = 0;

    fread(&num_points, sizeof(uint32_t), 1, f);
    fread(&dt, sizeof(float), 1, f);

    if (num_points == 0 || dt <= 0.0f) {
        ESP_LOGE(TAG, "Formato invalido ou dados vazios. Pontos: %lu, dt: %.4f", num_points, dt);
        fclose(f);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "A Iniciar Perfil Sismico: %lu pontos, dt=%.4fs, tempo total=%.2fs", num_points, dt, num_points * dt);

    gpio_set_level(ctx->gpio_en, STEP_MOTOR_ENABLE_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(50));

    // O motor mecanicamente inicia no centro após o Homing (correspondente a 90 graus nas nossas contas)
    float current_angle = 90.0f;
    int32_t current_step = (int32_t)roundf((current_angle / 360.0f) * table_config->stepper_x.microsteps_per_rev);
    
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    int64_t start_time_us = esp_timer_get_time();
    uint32_t point_index = 0;

    #define CHUNK_SIZE 100
    float target_pos_chunk[CHUNK_SIZE];
    uint32_t points_left = num_points;

    while (points_left > 0) {
        uint32_t to_read = (points_left > CHUNK_SIZE) ? CHUNK_SIZE : points_left;
        size_t read_count = fread(target_pos_chunk, sizeof(float), to_read, f);
        
        if (read_count == 0) { ESP_LOGW(TAG, "Fim do ficheiro inesperado!"); break; }
        
        for (uint32_t i = 0; i < read_count; i++) {
            float target_pos_mm = target_pos_chunk[i];
            
            // 1. Cinemática Inversa
            float target_angle = kinematics_calc_inverse_position_relative_90(&table_config->axis_x, target_pos_mm);
            int32_t target_step = (int32_t)roundf((target_angle / 360.0f) * table_config->stepper_x.microsteps_per_rev);
            int32_t delta_steps = target_step - current_step;

            // Debug: Imprimir a cada 10 pontos (0.1s) para não saturar a UART e afetar o timing do RMT
            if (point_index % 10 == 0) {
                float current_time_s = point_index * dt;
                ESP_LOGI(TAG, "Tempo: %5.3fs | Pos: %7.4f mm | Angulo: %6.2f deg", current_time_s, target_pos_mm, target_angle);
            }

            // 2. Executar Movimento
            if (delta_steps != 0) {
                bool is_cw = (delta_steps > 0);
                gpio_set_level(ctx->gpio_dir, is_cw ? STEP_MOTOR_SPIN_DIR_CLOCKWISE : STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                
                uint32_t steps_to_move = abs(delta_steps);
                uint32_t freq_hz = (uint32_t)roundf((float)steps_to_move / dt);
                if (freq_hz < 10) freq_hz = 10; // Frequência mínima de segurança do RMT

                for (uint32_t s = 0; s < steps_to_move; s++) {
                    rmt_transmit(ctx->motor_chan, ctx->uniform_motor_encoder, &freq_hz, sizeof(freq_hz), &tx_config);
                }
                current_step = target_step;
            } else {
                // Se o sismo tem uma pausa (0 deslocamento), sincronizamos o relógio no microsegundo exato
                rmt_tx_wait_all_done(ctx->motor_chan, -1);
                int64_t target_time_us = start_time_us + (int64_t)((point_index + 1) * dt * 1000000.0f);
                int64_t delay_us = target_time_us - esp_timer_get_time();
                if (delay_us > 0) {
                    if (delay_us > 10000) vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
                    else esp_rom_delay_us(delay_us);
                }
            }
            
            point_index++;
            
            // Quebra de segurança (Paragem de emergência por botão ou Nextion)
            if (nextion_profile != 6) { points_left = 0; break; }
        }
        points_left -= read_count;
    }

    rmt_tx_wait_all_done(ctx->motor_chan, -1);
    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
    fclose(f);
    ESP_LOGI(TAG, "Perfil Sismico do ficheiro concluido com sucesso!");
    
    return ESP_OK;
}

void stepper_rmt_task_1(void *arg)
{
    // Initialize Motor 1 with the configured pins
    stepper_rmt_context_t *motor1 = stepper_rmt_init(5, 32, 4);
    
    shake_table_config_t my_table;
    kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
    kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32); // 1.8 degree step, 32 microsteps

    // Configuração do ADC1 (Vamos usar o GPIO 34 = ADC1_CHANNEL_6)
    if (s_adc1_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &s_adc1_handle));
    }

    adc_oneshot_chan_cfg_t adc_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    // Recomendado: Usar GPIO34 em vez do GPIO12
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_6, &adc_cfg));

    // Execute Homing calibration on startup
/*     if (motor1) {
        stepper_rmt_homing(motor1, 22, 23);
    }
 */
    while (1) {
        // LARGER STEPPER
        uint8_t current_profile = nextion_profile;
        // Fallback: Se premir o botão físico sem ensaio ativo, executa o Case 6 por defeito
        if (gpio_get_level(GPIO_NUM_14) == 0) {
            current_profile = (nextion_profile == 0) ? 6 : nextion_profile; 
        }

        if (motor1 && current_profile != 0) {
            motor1_busy = true; // Levanta a bandeira de ocupado
            gpio_set_level(GPIO_NUM_25, 1); // Turn LED on

            switch (current_profile){
                case 1:
                    // Sine Profile
                    // Example: Amplitude 32.0mm, Frequency 2.0 Hz, Duration 5 Seconds
                    // stepper_rmt_run_sine_profile(motor1, 32.0f, 2.0f, 5.0f, &my_table); // Original rígido
                    
                    motor1_ready = false;
                    ESP_LOGI(TAG, "Motor 1: A executar Auto-Homing preparatorio...");
                    stepper_rmt_homing(motor1, 22, 23);
                    motor1_ready = true;

                    if (nextion_target_disp_x > 0.0f && nextion_target_freq_x > 0.0f && nextion_target_time_s > 0.0f) {
                        
                        // Barreira de Sincronização: aguarda que o Motor 2 também acabe o seu Homing
                        while (!motor2_ready && nextion_profile == 1) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        
                        // Após ambos acabarem, arrancam em simultâneo
                        if (nextion_profile == 1) {
                            sendAckToNextion(163); // Envia ACK START MOTION apenas após o Homing acabar
                            stepper_rmt_run_sine_profile(motor1, nextion_target_disp_x, nextion_target_freq_x, nextion_target_time_s, &my_table);
                        }
                    } else {
                        ESP_LOGI(TAG, "Motor 1 parado (Deslocamento ou Freq nulos). A manter sincronia...");
                        
                        // Barreira de Sincronização: se Motor 2 não mexe, tem de aguardar o Homing do Motor 1!
                        while (!motor2_ready && nextion_profile == 1) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        
                        if (nextion_profile == 1) {
                            sendAckToNextion(163); // Envia ACK START MOTION na mesma para avisar o HMI
                        }
                        
                        int64_t start_idle_us = esp_timer_get_time();
                        int64_t duration_idle_us = (int64_t)(nextion_target_time_s * 1000000.0f);
                        while ((esp_timer_get_time() - start_idle_us) < duration_idle_us) {
                            if (nextion_profile == 0) break; // Aborta se o utilizador premir STOP
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                        // Desliga o driver no final da espera para não aquecer infinitamente
                        gpio_set_level(motor1->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);                        
                    }
                    break;
                case 2: {
                    // Multi-Step Frequency Profile (Non-Periodic Looping)
                    // Example: 4 stages, looping for 60 seconds total. 0.5s transition blend time.
                    float freqs[] = {1.0f, 2.5f, 4.0f, 1.5f};
                    float times[] = {5.0f, 10.0f, 5.0f, 8.0f};
                    // Note: This block uses { } to define local scope for the arrays inside the switch case.
                    stepper_rmt_run_multistep_freq_profile(motor1, 16.0f, freqs, times, 4, 60.0f, 0.5f, &my_table);
                    break;
                }
                case 3:
                    // Trapezoidal Freq Profile
                    // Exemplo: 16mm P2P, de 0.5Hz até 3.0Hz. (3s para acelerar, 5s constante, 3s para travar)
                    stepper_rmt_run_trapezoidal_freq_profile(motor1, 16.0f, 0.5f, 3.0f, 3.0f, 5.0f, 3.0f, &my_table);
                    break;
                case 4:
                    // Real-Time Sine Profile (Analog control with potenciometer B10k)
                    //Amplitude 16.0mm, Duration: 10 sec, Dynamic frequency manage by ADC between 0.5Hz to 5.0Hz
                    stepper_rmt_run_realtime_sine_profile(motor1, 16.0f, 10.0f, s_adc1_handle, ADC_CHANNEL_6, 0.5f, 5.0f, &my_table);
                    break;
                case 5:
                    // Sweep / Chirp Profile (IEC/ISO Standard Logarithmic Sweep)
                    // Example: 33mm P2P, from 0.5Hz to 10.0Hz, 30 seconds duration, Bidirectional (Ping-Pong) = true
                    // stepper_rmt_run_sweep_profile(motor1, 16.0f, 0.5f, 5.0f, 30.0f, true, &my_table);
                    if (nextion_target_disp_x > 0.0f && nextion_target_freq_x > 0.0f && nextion_target_time_s > 0.0f) {
                        // Assumimos frequência inicial de 0.5Hz e final como sendo a escolhida no ecrã
                        stepper_rmt_run_sweep_profile(motor1, nextion_target_disp_x, 0.5f, nextion_target_freq_x, nextion_target_time_s, true, &my_table);
                    } else {
                        ESP_LOGI(TAG, "Motor 1 parado no Sweep. A manter sincronia...");
                        int64_t start_idle_us = esp_timer_get_time();
                        while ((esp_timer_get_time() - start_idle_us) < (int64_t)(nextion_target_time_s * 1000000.0f)) {
                            if (nextion_profile == 0) break;
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                    }
                    break;
                case 6:
                {
                    char filepath[256];
                    if (get_stored_sismo_file(filepath, sizeof(filepath))) {
                        stepper_rmt_run_file_profile(motor1, filepath, &my_table);
                    } else {
                        ESP_LOGW(TAG, "Nenhum ficheiro .bin encontrado na memoria para reproduzir!");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    break;
                }
                default:
                    // 
                    // Simple test for turn motor clockwise and counterclockwise
                    /*gpio_set_level(motor1->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Enable Motor 1 driver
                    stepper_rmt_run_steps(motor1, 1500, 5000, 500, 500, true);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    stepper_rmt_run_steps(motor1, 1500, 5000, 500, 500, false);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    gpio_set_level(motor1->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Disable driver after finished
                    */
                    break;
            }
            
            motor1_busy = false; // Baixa a bandeira
            
            // Lógica de sincronização (apenas se ativado pelo Nextion)
            if (nextion_profile == current_profile && current_profile != 0) {
                // Task 1 atua como Master: Aguarda até a Task 2 terminar
                while (motor2_busy && nextion_profile == current_profile) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                // Se o perfil não foi abortado a meio da espera, finaliza!
                if (nextion_profile == current_profile) {
                    nextion_profile = 0; // Limpa o estado
                    parameters_recv = false; // Rearme obrigatório: bloqueia novos arranques até os dados serem revalidados
                    ESP_LOGI(TAG, "Ensaio concluido com sucesso! A enviar ACK 164 (MOTION END) para o HMI.");
                    sendAckToNextion(164);
                }
            }
        }
        gpio_set_level(GPIO_NUM_25, 0); // Turn LED off
        // Essential delay to yield CPU to other tasks when idle
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void stepper_rmt_task_2(void *arg)
{
    // Initialize Motor 2 with the specified pins (EN: 15, DIR: 19, STEP: 18)
    stepper_rmt_context_t *motor2 = stepper_rmt_init(19, 15, 18);

    shake_table_config_t my_table;
    kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
    kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32); // 1.8 degree step, 32 microsteps

    // Atraso curto para dar prioridade de inicialização à Task 1, mas garante segurança se a Task 2 for mais rápida
    vTaskDelay(pdMS_TO_TICKS(50));
    if (s_adc1_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &s_adc1_handle));
    }

    // Configuração do ADC1 (Vamos usar o GPIO 39 = ADC1_CHANNEL_3)
    
    adc_oneshot_chan_cfg_t adc_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    // Utilizando o GPIO 39 (Pino Input-Only, seguro para ADC e Wi-Fi)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_3, &adc_cfg));

    // Execute Homing calibration on startup
/*     if (motor2) {
        stepper_rmt_homing(motor2, 27, 33);
    } */
    while (1) {
        // Smaller STEPPER
        uint8_t current_profile = nextion_profile;
        // Fallback: Se premir o botão físico sem ensaio ativo, executa o Case 6 por defeito
        if (gpio_get_level(GPIO_NUM_21) == 0) {
            current_profile = (nextion_profile == 0) ? 6 : nextion_profile;
        }

        if (motor2 && current_profile != 0) {
            motor2_busy = true; // Levanta a bandeira de ocupado
            gpio_set_level(GPIO_NUM_26, 1); // Turn LED on
            
            switch (current_profile){
                case 1:
                    // Example: Amplitude 32.0mm, Frequency 2.0 Hz, Duration 5 Seconds
                    // stepper_rmt_run_sine_profile(motor2, 32.0f, 2.0f, 5.0f, &my_table); // Original rígido                
                    motor2_ready = false;
                    ESP_LOGI(TAG, "Motor 2: A executar Auto-Homing preparatorio...");
                    stepper_rmt_homing(motor2, 27, 33);
                    motor2_ready = true;

                    // Sine Profile
                    if (nextion_target_disp_y > 0.0f && nextion_target_freq_y > 0.0f && nextion_target_time_s > 0.0f) {
                        // Barreira de Sincronização: aguarda que o Motor 1 também acabe o seu Homing
                        while (!motor1_ready && nextion_profile == 1) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }

                        // Após ambos acabarem, arrancam em simultâneo
                        if (nextion_profile == 1) {
                            stepper_rmt_run_sine_profile(motor2, nextion_target_disp_y, nextion_target_freq_y, nextion_target_time_s, &my_table);
                        }
                    } else {
                        ESP_LOGI(TAG, "Motor 2 parado (Deslocamento ou Freq nulos). A manter sincronia...");
                        
                        
                        // Barreira de Sincronização: se Motor 2 não mexe, tem de aguardar o Homing do Motor 1!
                        while (!motor1_ready && nextion_profile == 1) {
                            vTaskDelay(pdMS_TO_TICKS(10));
                        }
                        
                        int64_t start_idle_us = esp_timer_get_time();
                        int64_t duration_idle_us = (int64_t)(nextion_target_time_s * 1000000.0f);
                        while ((esp_timer_get_time() - start_idle_us) < duration_idle_us) {
                            if (nextion_profile == 0) break; // Aborta se o utilizador premir STOP
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }

                        // Desliga o driver no final da espera para não aquecer infinitamente
                        gpio_set_level(motor2->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);
                    }
                    break;
                case 2: {
                    // Multi-Step Frequency Profile (Non-Periodic Looping)
                    // Example: 4 stages, looping for 60 seconds total. 0.5s transition blend time.
                    float multiStep_freqs[] = {1.0f, 2.5f, 4.0f, 1.5f};
                    float multiStep_times[] = {5.0f, 10.0f, 5.0f, 8.0f};
                    stepper_rmt_run_multistep_freq_profile(motor2, 33.0f, multiStep_freqs, multiStep_times, 4, 60.0f, 0.5f, &my_table);
                    break;
                }
                case 3:
                    // Trapezoidal Freq Profile
                    // Exemplo: 33mm P2P, de 0.5Hz até 3.0Hz. (3s para acelerar, 5s constante, 3s para travar)
                    stepper_rmt_run_trapezoidal_freq_profile(motor2, 33.0f, 0.5f, 3.0f, 3.0f, 5.0f, 3.0f, &my_table);
                    break;
                case 4:
                    // Real-Time Sine Profile (Analog control with potenciometer B10k)
                    //  Example: Amplitude 32.0mm, Duration: 10 Segundos, Dynamic frequency manage by ADC between 0.5Hz to 5.0Hz        
                    stepper_rmt_run_realtime_sine_profile(motor2, 33.0f, 10.0f, s_adc1_handle, ADC_CHANNEL_3, 0.5f, 5.0f, &my_table);        
                    break;
                case 5:
                    // Sweep / Chirp Profile (IEC/ISO Standard Logarithmic Sweep)
                    // Example: 33mm P2P, from 0.5Hz to 10.0Hz, 30 seconds duration, Bidirectional (Ping-Pong) = true
                    //stepper_rmt_run_sweep_profile(motor2, 33.0f, 0.5f, 5.0f, 30.0f, true, &my_table);
                    if (nextion_target_disp_y > 0.0f && nextion_target_freq_y > 0.0f && nextion_target_time_s > 0.0f) {
                        // Assumimos frequência inicial de 0.5Hz e final como sendo a escolhida no ecrã
                        stepper_rmt_run_sweep_profile(motor2, nextion_target_disp_y, 0.5f, nextion_target_freq_y, nextion_target_time_s, true, &my_table);
                    } else {
                        ESP_LOGI(TAG, "Motor 2 parado no Sweep. A manter sincronia...");
                        int64_t start_idle_us = esp_timer_get_time();
                        while ((esp_timer_get_time() - start_idle_us) < (int64_t)(nextion_target_time_s * 1000000.0f)) {
                            if (nextion_profile == 0) break;
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                    }
                    break;
                case 6:
                {
                    char filepath[256];
                    if (get_stored_sismo_file(filepath, sizeof(filepath))) {
                        stepper_rmt_run_file_profile(motor2, filepath, &my_table);
                    } else {
                        ESP_LOGW(TAG, "Nenhum ficheiro .bin encontrado na memoria para reproduzir!");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    break;
                }
                default:
                    // 
                    // Simple test for turn motor clockwise and counterclockwise
                    /*  
                    gpio_set_level(motor2->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Enable Motor 2 driver
                    stepper_rmt_run_steps(motor2, 1500, 5000, 500, 500, true);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    stepper_rmt_run_steps(motor2, 1500, 5000, 500, 500, false);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    gpio_set_level(motor2->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Disable driver after finished
                    */
                    break;
            }

            motor2_busy = false; // Baixa a bandeira

            // Lógica de sincronização (apenas se ativado pelo Nextion)
            if (nextion_profile == current_profile && current_profile != 0) {
                // A Task 2 é Escrava: apenas aguarda que a Task 1 limpe a variável global
                while (nextion_profile == current_profile) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        gpio_set_level(GPIO_NUM_26, 0); // Turn LED off
        // Essential delay to yield CPU to other tasks when idle
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
