#include <stdio.h>
#include <time.h>
#include <math.h>
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "functions.h"
#include "driver/gpio.h"
#include "l298n_stepper.h"
#include "kinematics.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "i2c_bus.h"

static const char* TAG = "ESP32_ShakeTable";
#define UART_TASK_STACK_SIZE 4096
extern TaskHandle_t rxTaskHandle;
extern TaskHandle_t txTaskHandle;

// =========================================================================
// Funções Matemáticas para gerar o Alvo Teórico Suave (Smooth Target)
// =========================================================================

static float phase_x = 0.0f, phase_y = 0.0f;
static float last_t_x = 0.0f, last_t_y = 0.0f;

static float get_instantaneous_freq(char axis, int profile, float t) {
    float freq = 0.1f;
    switch (profile) {
        case 1: // Sine Wave
            freq = (axis == 'x') ? nextion_target_freq_x : nextion_target_freq_y;
            break;
            
        case 2: { // Multi-Step Frequency
            float *freqs = (axis == 'x') ? nextion_multistep_freq_x : nextion_multistep_freq_y;
            float *times = (axis == 'x') ? nextion_multistep_time_x : nextion_multistep_time_y;
            float blend_time = 0.5f; 

            float cycle_time = times[0] + times[1] + times[2] + times[3];
            if (cycle_time <= 0.0f) return 0.0f;

            float t_cycle = fmodf(t, cycle_time);
            float st_time = 0.0f;
            int stage = 0;
            float time_in_stage = t_cycle;

            for (int i = 0; i < 4; i++) {
                if (t_cycle >= st_time && t_cycle < st_time + times[i]) {
                    stage = i;
                    time_in_stage = t_cycle - st_time;
                    break;
                }
                st_time += times[i];
            }

            int prev_stage = (stage == 0) ? 3 : stage - 1;
            freq = freqs[stage];
            if (time_in_stage < blend_time) {
                float t_blend = time_in_stage / blend_time;
                freq = freqs[prev_stage] + (freqs[stage] - freqs[prev_stage]) * t_blend;
            }
            break;
        }
        case 3: { // Trapezoidal
            float start_f = (axis == 'x') ? nextion_trapz_start_freq_x : nextion_trapz_start_freq_y;
            float cruise_f = (axis == 'x') ? nextion_trapz_cruise_freq_x : nextion_trapz_cruise_freq_y;
            float end_f = (axis == 'x') ? nextion_trapz_end_freq_x : nextion_trapz_end_freq_y;
            float accel_t = (axis == 'x') ? nextion_trapz_accel_time_x : nextion_trapz_accel_time_y;
            float cruise_t = (axis == 'x') ? nextion_trapz_cruise_time_x : nextion_trapz_cruise_time_y;
            float decel_t = (axis == 'x') ? nextion_trapz_decel_time_x : nextion_trapz_decel_time_y;
            
            if (accel_t > 0.001f && t < accel_t) freq = start_f + (cruise_f - start_f) * (t / accel_t);
            else if (t < accel_t + cruise_t) freq = cruise_f;
            else if (decel_t > 0.001f && t < accel_t + cruise_t + decel_t) freq = cruise_f - (cruise_f - end_f) * ((t - accel_t - cruise_t) / decel_t);
            else freq = end_f;
            break;
        }
        case 5: { // Sweep / Chirp
            float start_f = (axis == 'x') ? nextion_sweep_min_freq_x : nextion_sweep_min_freq_y;
            float end_f = (axis == 'x') ? nextion_sweep_max_freq_x : nextion_sweep_max_freq_y;
            float dur = nextion_target_time_s;
            bool is_bid = (axis == 'x') ? nextion_sweep_isBid_x : nextion_sweep_isBid_y;
            
            float eff_t = t;
            if (is_bid && eff_t > dur) eff_t = (2.0f * dur) - eff_t; 
            float norm_t = (dur > 0.0f) ? (eff_t / dur) : 1.0f;
            if (norm_t > 1.0f) norm_t = 1.0f;
            
            if (start_f > 0.0f) freq = start_f * powf(end_f / start_f, norm_t);
            break;
        }
    }
    if (freq < 0.1f) freq = 0.1f;
    return freq;
}

static float get_theoretical_position(char axis, int profile, float t) {
    float target_disp = (axis == 'x') ? nextion_target_disp_x : nextion_target_disp_y;
    if (target_disp <= 0.0f) return 0.0f;

    float amp = target_disp / 2.0f;
    float freq = get_instantaneous_freq(axis, profile, t);

    #ifndef M_PI
    #define M_PI 3.14159265358979323846f
    #endif

    if (axis == 'x') {
        float dt = t - last_t_x;
        if (dt < 0) dt = 0;
        phase_x += 2.0f * M_PI * freq * dt;
        last_t_x = t;
        return amp * sinf(phase_x);
    } else {
        float dt = t - last_t_y;
        if (dt < 0) dt = 0;
        phase_y += 2.0f * M_PI * freq * dt;
        last_t_y = t;
        return amp * sinf(phase_y);
    }
}

// =========================================================================
// Telemetria UART Task (Envia as coordenadas a 100Hz para o MCU 2)
// =========================================================================
void telemetry_task(void *arg) {
    char buf[64];
    bool sync_active = false;
    int64_t start_time_us = 0;

    while(1) {
        if (nextion_profile != 0 && motor1_ready && motor2_ready) {
            // Activa o SYNC Pin no arranque
            if (!sync_active) {
                gpio_set_level(SYNC_GPIO_PIN, 1);
                sync_active = true;
                start_time_us = esp_timer_get_time();
                phase_x = 0.0f; last_t_x = 0.0f;
                phase_y = 0.0f; last_t_y = 0.0f;
                ESP_LOGI(TAG, "SYNC Pin = HIGH (Ensaio Iniciado)");
            }

            float t_sec = (esp_timer_get_time() - start_time_us) / 1000000.0f;
            float csv_target_x, csv_target_y;

            // Se for Potenciómetro (4) ou Ficheiro (6), não há matemática teórica! Usamos a posição crua live.
            if (nextion_profile == 4 || nextion_profile == 6) {
                csv_target_x = current_target_pos_x;
                csv_target_y = current_target_pos_y;
            } else {
                csv_target_x = get_theoretical_position('x', nextion_profile, t_sec);
                csv_target_y = get_theoretical_position('y', nextion_profile, t_sec);
            }
            
            int len = snprintf(buf, sizeof(buf), "%.3f,%.3f\n", csv_target_x, csv_target_y);
            uart_write_bytes(UART_TELEMETRY_NUM, buf, len);
        } else {
            // Desactiva o SYNC Pin no fim
            if (sync_active) {
                gpio_set_level(SYNC_GPIO_PIN, 0);
                sync_active = false;
                ESP_LOGI(TAG, "SYNC Pin = LOW (Ensaio Terminado)");
            }

            // Garante que manda zero quando parado
            int len = snprintf(buf, sizeof(buf), "0.000,0.000\n");
            uart_write_bytes(UART_TELEMETRY_NUM, buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
	GPIO_init(); //initialize GPIOs
	check_current_config(); //print GPIO configuration
	init_littlefs(); // Mount LittleFS Partition
	init_uart_to_Nextion();
	start_wifi_ap(); //start Wi-Fi
	start_webserver(); //start HTTP server
	config_manager_init(); // Carrega configurações da NVS ou usa defaults
	init_i2c_system(); // Inicia I2C e deteta o MCP23017

	// Inicializa UART Telemetry (Comunicação com MCU 2)
    init_uart_to_mcu2();

	// Atualiza o hardware com os microsteps carregados da memória para ambos os motores
	set_all_steppers_microsteps(table_config_x.stepper.microsteps, table_config_y.stepper.microsteps);

	// --- Kinematics Structure Test ---
	
	/*shake_table_config_t my_table;
	kinematics_init_axis(&my_table.axis_x, 33.0f, 66.0f); // 33mm peak-to-peak displacement, 66mm rod length
	kinematics_init_stepper(&my_table.stepper_x, 1.8f, 1.0f, 32);  // 1.8 degree step, gear 1.0, 32 microsteps (6400 steps/rev)

	float angle = kinematics_calc_angular_position(&my_table.stepper_x, 1600);
	float position_mm = kinematics_calc_linear_position(&my_table.axis_x, angle);
	float real_position_mm = kinematics_calc_linear_position_relative_90(&my_table.axis_x, angle);
	ESP_LOGI(TAG, "Kinematics Test: 1600 microsteps = %.2f degrees", angle);
	ESP_LOGI(TAG, "  -> Geometric Position (Relative to rod): %.2f mm", position_mm);
	ESP_LOGI(TAG, "  -> Real Position (Relative to 90 degrees): %.2f mm", real_position_mm);
	*/
	// --- End of Kinematics Structure Test ---

	vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay

	// Tarefas do Ecrã Nextion e Telemetria (FreeRTOS gere os cores automaticamente)
	xTaskCreate(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle);
	xTaskCreate(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle);
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, configMAX_PRIORITIES - 10, NULL);
    xTaskCreate(telemetry_rx_task, "telemetry_rx_task", 4096, NULL, configMAX_PRIORITIES - 11, NULL);

	// Tasks to control the stepper motors with DRV8825 and RMT independently
	// Mantém-se o Core 1 para máxima estabilidade e imunidade a interrupções do sistema (WiFi/SPI Flash)
	xTaskCreatePinnedToCore(stepper_rmt_task_1, "stepper_rmt_task_1", 8192, NULL, configMAX_PRIORITIES - 5, NULL, 1);
	xTaskCreatePinnedToCore(stepper_rmt_task_2, "stepper_rmt_task_2", 8192, NULL, configMAX_PRIORITIES - 5, NULL, 1);

	// FreeRTOS main loop: Keep the main task alive with periodic vTaskDelay
	// The actual work happens in the 4 tasks created above
	// vTaskDelay prevents watchdog timeout and allows FreeRTOS scheduler to run
	// while (1)
	// {	
	// 	vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
	// }

}
