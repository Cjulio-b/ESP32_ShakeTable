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
#include <math.h>
#include "stepper_motor_encoder.h"
#include "functions.h"
#include "kinematics.h"

#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
#define PI_MATH 3.14159265358979323846f

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
    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL);

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

    // CASE 1: Continuous Rotation (Maximum Amplitude)
    // For max P2P (~33mm), the crank-slider mechanism translates continuous rotation into full linear strokes.
    if (target_p2p_mm >= max_p2p - 0.1f) {
        ESP_LOGI(TAG, "Continuous Rotation Mode (Crank-slider naturally actuates the full stroke)");
        uint32_t speed_hz = (uint32_t)(table_config->stepper_x.microsteps_per_rev * freq_hz);
        uint32_t total_steps = (uint32_t)(speed_hz * duration_s);
        
        stepper_rmt_run_steps(ctx, speed_hz, total_steps, 0, 0, true);
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

        int num_cycles = (int)roundf(duration_s * freq_hz);
        int total_quarters = num_cycles * 4;
        rmt_transmit_config_t tx_config = { .loop_count = 0 };

        // Execute the profile quarter-by-quarter
        for (int q = 0; q < total_quarters; q++) {
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
        }
    }

    gpio_set_level(ctx->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Disable the driver
    ESP_LOGI(TAG, "Sinusoidal Seismic Profile Completed!");
    return ESP_OK;
}

void stepper_rmt_task_1(void *arg)
{
    // Initialize Motor 1 with the configured pins
    stepper_rmt_context_t *motor1 = stepper_rmt_init(5, 32, 4);
    
    shake_table_config_t my_table;
    kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
    kinematics_init_stepper(&my_table.stepper_x, 1.8f, 32); // 1.8 degree step, 32 microsteps

    // Execute Homing calibration on startup
    if (motor1) {
        stepper_rmt_homing(motor1, 22, 23);
    }

    while (1) {
        // LARGER STEPPER
        // Button (GPIO 14) has an active internal PULL-UP, so it reads 0 when pressed
        if (motor1 && gpio_get_level(GPIO_NUM_14) == 0) {
            gpio_set_level(GPIO_NUM_25, 1); // Turn LED on

            // Simple test for turn motor clockwise and counterclockwise
            /*gpio_set_level(motor1->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Enable Motor 1 driver
            stepper_rmt_run_steps(motor1, 1500, 5000, 500, 500, true);
            vTaskDelay(pdMS_TO_TICKS(1000));
            stepper_rmt_run_steps(motor1, 1500, 5000, 500, 500, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(motor1->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Disable driver after finished
            */
            //--- End Test---
            
            // Test Seismic Profile: Amplitude 32.0mm, Frequency 2.0 Hz, Duration 5 Seconds
            stepper_rmt_run_sine_profile(motor1, 32.0f, 2.0f, 5.0f, &my_table);
            //--- End Test ---
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

    // Execute Homing calibration on startup
    if (motor2) {
        stepper_rmt_homing(motor2, 27, 33);
    }

    while (1) {
        // Smaller STEPPER
        // Button (GPIO 21) has an active internal PULL-UP, so it reads 0 when pressed
        if (motor2 && gpio_get_level(GPIO_NUM_21) == 0) {
            gpio_set_level(GPIO_NUM_26, 1); // Turn LED on
            // Simple test for turn motor clockwise and counterclockwise
        /*  gpio_set_level(motor2->gpio_en, STEP_MOTOR_ENABLE_LEVEL); // Enable Motor 2 driver
            stepper_rmt_run_steps(motor2, 1500, 5000, 500, 500, true);
            vTaskDelay(pdMS_TO_TICKS(1000));
            stepper_rmt_run_steps(motor2, 1500, 5000, 500, 500, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(motor2->gpio_en, !STEP_MOTOR_ENABLE_LEVEL); // Disable driver after finished
        */
            //---End Test---

            // Test Seismic Profile: Amplitude 32.0mm, Frequency 2.0 Hz, Duration 5 Seconds
            stepper_rmt_run_sine_profile(motor2, 32.0f, 2.0f, 5.0f, &my_table);        
            //--- End Test ---
        }
        
        gpio_set_level(GPIO_NUM_26, 0); // Turn LED off
        // Essential delay to yield CPU to other tasks when idle
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
