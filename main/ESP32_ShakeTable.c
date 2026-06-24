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
#include "mcp23017.h"
#include "adxl345.h"
#include "esp_timer.h"

static const char* TAG = "ESP32_ShakeTable";
#define UART_TASK_STACK_SIZE 4096
extern TaskHandle_t rxTaskHandle;
extern TaskHandle_t txTaskHandle;

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 23

#define I2C_MASTER_SCL_IO_2 25
#define I2C_MASTER_SDA_IO_2 26

i2c_master_bus_handle_t i2c_bus_handle = NULL;
i2c_master_bus_handle_t i2c_bus_2_handle = NULL;
i2c_master_dev_handle_t mcp_handle = NULL;
i2c_master_dev_handle_t adxl_table_handle = NULL;
i2c_master_dev_handle_t adxl_specimen_handle = NULL;
static uint8_t mcp_port_b_state = 0;

void scan_i2c_bus(i2c_master_bus_handle_t bus_handle, const char* bus_name) {
    ESP_LOGI("I2C_SCAN", "Starting scanner on bus: %s", bus_name);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        esp_err_t err = i2c_master_probe(bus_handle, addr, 20); // Reduzido de 100 para 20ms
        if (err == ESP_OK) {
            ESP_LOGI("I2C_SCAN", " -> Device detected at address: 0x%02X", addr);
            found++;
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE("I2C_SCAN", " -> Bus is stuck or timed out! Aborting scan on %s to prevent log spam.", bus_name);
            break; // Se deu timeout, o barramento está preso (ex: curto-circuito). Abortamos para não spammar 127 vezes.
        }
    }
    if (found == 0) {
        ESP_LOGW("I2C_SCAN", " -> No devices found on %s!", bus_name);
    }
}

void init_i2c_system(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0, // Força a usar o bloco físico 0
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));
    
    i2c_master_bus_config_t bus_config_2 = {
        .i2c_port = I2C_NUM_1, // Força a usar o bloco físico 1
        .sda_io_num = I2C_MASTER_SDA_IO_2,
        .scl_io_num = I2C_MASTER_SCL_IO_2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config_2, &i2c_bus_2_handle));

    // Inicializa o Expansor de I/O
    if (mcp23017_init(i2c_bus_handle, MCP23017_I2C_ADDR_DEFAULT, &mcp_handle) == ESP_OK) {
        // Exemplo: Configurar todos os Pinos do BANK A como OUTPUTS (0x00)
        esp_err_t err = mcp23017_write_reg(mcp_handle, MCP23017_IODIRA, 0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Timeout/Failed to configure MCP23017! Check SDA/SCL and RST pins.");
            i2c_master_bus_rm_device(mcp_handle);
            mcp_handle = NULL;
        } else {
            mcp23017_write_reg(mcp_handle, MCP23017_IODIRB, 0x00); // Configura o BANK B como OUTPUTS
            mcp23017_write_reg(mcp_handle, MCP23017_GPIOA, 0x00); // Garante que os LEDs (A0 e A1) começam apagados
            mcp23017_write_reg(mcp_handle, MCP23017_GPIOB, 0x00); // Estado base 0 no PORT B
            ESP_LOGI(TAG, "MCP23017 setup completed successfully!");
        }
    }

    // Initialize ADXL345 (Shake Table Dynamics)
    if (adxl345_init(i2c_bus_handle, ADXL345_I2C_ADDR_GND, &adxl_table_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Table Accelerometer initialized!");
    }
    
    // Run I2C Scanner to debug hardware
    scan_i2c_bus(i2c_bus_handle, "BUS 1 (MCP + ADXL Table)");
    scan_i2c_bus(i2c_bus_2_handle, "BUS 2 (ADXL Specimen - Pins 25/26)");

    // Initialize ADXL345 (Test Specimen Dynamics) na "via verde" 2 com endereço default GND!
    if (adxl345_init(i2c_bus_2_handle, ADXL345_I2C_ADDR_GND, &adxl_specimen_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Specimen Accelerometer initialized on BUS 2!");
    }
}

// =========================================================================
// Converte valor de microstepping em 3 bits (helper for set_all_steppers_microsteps)
// =========================================================================
static uint8_t get_microstep_bits(uint16_t microsteps) {
    switch(microsteps) {
        case 4:  return 0b010; // b0=low, b1=high, b2=low -> 4
        case 8:  return 0b011; // b0=high, b1=high, b2=low -> 8
        case 16: return 0b100; // b0=low, b1=low, b2=high -> 16
        case 32: return 0b111; // b0=high, b1=high, b2=high -> 32
        default: return 0b111; // default safety (32)
    }
}

// =========================================================================
// Configura os pinos M0, M1, M2 (Motor X) e M3, M4, M5 (Motor Y) via I2C
// Motor X: bits 0-2 (b0, b1, b2)
// Motor Y: bits 3-5 (b3, b4, b5)
// =========================================================================
void set_all_steppers_microsteps(uint16_t micro_x, uint16_t micro_y) {
    if (!mcp_handle) return;
    
    uint8_t bits_x = get_microstep_bits(micro_x);  // bits 0-2
    uint8_t bits_y = get_microstep_bits(micro_y);  // bits 3-5
    
    // Limpa bits relevantes (0-5) e aplica os novos valores
    mcp_port_b_state &= ~0b00111111; // Limpa bits 0-5
    mcp_port_b_state |= bits_x;                    // Bits 0-2 para Motor X
    mcp_port_b_state |= (bits_y << 3);             // Bits 3-5 para Motor Y
    
    mcp23017_write_reg(mcp_handle, MCP23017_GPIOB, mcp_port_b_state);
    ESP_LOGI(TAG, "MCP23017: Motor X=%d uSteps (b0-b2=0x%X), Motor Y=%d uSteps (b3-b5=0x%X), Port B: 0x%02X", 
             micro_x, bits_x, micro_y, bits_y, mcp_port_b_state);
}

// =========================================================================
// Calcula a posição matemática teórica em 't' (para gráficos suaves no CSV)
// =========================================================================
static float get_theoretical_position(char axis, int profile, float t) {
    float target_disp = (axis == 'x') ? nextion_target_disp_x : nextion_target_disp_y;
    if (target_disp <= 0.0f) return 0.0f;

    float amp = target_disp / 2.0f;
    float freq = 0.1f; // Frequência de segurança inicial

    switch (profile) {
        case 1: // Sine Wave
            freq = (axis == 'x') ? nextion_target_freq_x : nextion_target_freq_y;
            break;
            
        case 2: { // Multi-Step Frequency
            float *freqs = (axis == 'x') ? nextion_multistep_freq_x : nextion_multistep_freq_y;
            float *times = (axis == 'x') ? nextion_multistep_time_x : nextion_multistep_time_y;
            float blend_time = 0.5f; // Blend time hardcoded na motor task

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
            if (is_bid && eff_t > dur) eff_t = (2.0f * dur) - eff_t; // Fase descendente
            float norm_t = (dur > 0.0f) ? (eff_t / dur) : 1.0f;
            if (norm_t > 1.0f) norm_t = 1.0f;
            
            if (start_f > 0.0f) freq = start_f * powf(end_f / start_f, norm_t);
            break;
        }
    }

    #ifndef M_PI
    #define M_PI 3.14159265358979323846f
    #endif

    if (freq < 0.1f) freq = 0.1f;
    return amp * sinf(2.0f * M_PI * freq * t);
}

// =========================================================================
// CSV Recording Task (Background Buffer Writer)
// =========================================================================
typedef struct {
    float t_sec;
    float target_x;
    float target_y;
    float t_x;
    float t_y;
    float t_z;
    float s_x;
    float s_y;
    float s_z;
} csv_record_t;

QueueHandle_t csv_queue;

void csv_writer_task(void *arg) {
    FILE *f_csv = NULL;
    csv_record_t record;
    int items_to_flush = 0;
    
    ESP_LOGI("CSV_WRITER", "CSV Writer task started.");
    
    while(1) {
        if (xQueueReceive(csv_queue, &record, portMAX_DELAY)) {
            // Check for special sentinel value to open/close file
            if (record.t_sec < 0.0f) {
                if (record.target_x == 1.0f) {
                    // Open file
                    if (f_csv == NULL) {
                        f_csv = fopen("/storage/output/resultados.csv", "w");
                        if (f_csv) {
                            fprintf(f_csv, "Tempo_s,Target_X_mm,Target_Y_mm,Acc_Table_X_g,Acc_Table_Y_g,Acc_Table_Z_g,Acc_Specimen_X_g,Acc_Specimen_Y_g,Acc_Specimen_Z_g\n");
                            ESP_LOGI("CSV_WRITER", "Opened CSV file for writing.");
                        }
                    }
                } else if (record.target_x == 0.0f) {
                    // Close file
                    if (f_csv != NULL) {
                        fflush(f_csv);
                        fclose(f_csv);
                        f_csv = NULL;
                        ESP_LOGI("CSV_WRITER", "Closed CSV file.");
                        nextion_notify_test_result_available();
                    }
                }
            } else {
                // Write normal record
                if (f_csv != NULL) {
                    fprintf(f_csv, "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                            record.t_sec, record.target_x, record.target_y, 
                            record.t_x, record.t_y, record.t_z, 
                            record.s_x, record.s_y, record.s_z);
                    items_to_flush++;
                    
                    // Flush every 50 records (0.5 seconds at 100Hz) to ensure data is written
                    // without locking the flash on every single reading.
                    if (items_to_flush >= 50) {
                        fflush(f_csv);
                        items_to_flush = 0;
                    }
                }
            }
        }
    }
}

void accelerometer_task(void *arg)
{
    float t_x = 0.0f, t_y = 0.0f, t_z = 0.0f; // Table accelerations
    float s_x = 0.0f, s_y = 0.0f, s_z = 0.0f; // Specimen accelerations
    
    ESP_LOGI("ACCEL_TASK", "Accelerometer logging task started.");
    
    // --- Fase de Calibracao (Tara) ---
    float off_tx = 0, off_ty = 0, off_tz = 0;
    float off_sx = 0, off_sy = 0, off_sz = 0;
    int calib_samples = 200; // 2 segundos a 100Hz
    
    ESP_LOGI("ACCEL_TASK", "A calibrar acelerometros (Nao mexa na mesa)...");
    for (int i = 0; i < calib_samples; i++) {
        if (adxl_table_handle) {
            adxl345_read_acceleration(adxl_table_handle, &t_x, &t_y, &t_z);
            off_tx += t_x; off_ty += t_y; off_tz += t_z;
        }
        if (adxl_specimen_handle) {
            adxl345_read_acceleration(adxl_specimen_handle, &s_x, &s_y, &s_z);
            off_sx += s_x; off_sy += s_y; off_sz += s_z;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    off_tx /= calib_samples; off_ty /= calib_samples; off_tz /= calib_samples;
    off_sx /= calib_samples; off_sy /= calib_samples; off_sz /= calib_samples;
    ESP_LOGI("ACCEL_TASK", "Calibracao concluida! Offsets Table: [%.2f, %.2f, %.2f]", off_tx, off_ty, off_tz);
    // ---------------------------------

    int print_counter = 0;
    int64_t start_record_time = 0;
    bool is_recording = false;

    while (1) {
        // Only read and process if an active profile is running (to save CPU)
        if (nextion_profile != 0 && (motor1_ready && motor2_ready)) {
            bool error_detected = false;

            if (adxl_table_handle) {
                if (adxl345_read_acceleration(adxl_table_handle, &t_x, &t_y, &t_z) != ESP_OK) {
                    error_detected = true;
                }
            }
            if (adxl_specimen_handle) {
                if (adxl345_read_acceleration(adxl_specimen_handle, &s_x, &s_y, &s_z) != ESP_OK) {
                    error_detected = true;
                }
            }
            
            // Aplicar a Tara (Subtrair a gravidade estatica lida no inicio)
            t_x -= off_tx;
            t_y -= off_ty;
            t_z -= off_tz;
            s_x -= off_sx;
            s_y -= off_sy;
            s_z -= off_sz;

            if (!is_recording) {
                // Enviar comando para abrir o ficheiro
                csv_record_t cmd = {.t_sec = -1.0f, .target_x = 1.0f};
                xQueueSend(csv_queue, &cmd, 0);
                start_record_time = esp_timer_get_time();
                is_recording = true;
                ESP_LOGI("ACCEL_TASK", "Signaled CSV Writer to start.");
            }
            
            float t_sec = (float)(esp_timer_get_time() - start_record_time) / 1000000.0f;
            
            float csv_target_x = current_target_pos_x;
            float csv_target_y = current_target_pos_y;
            
            // Para os perfis determinísticos (1, 2, 3 e 5), calcular a onda matemática perfeitamente suave!
            if (nextion_profile == 1 || nextion_profile == 2 || nextion_profile == 3 || nextion_profile == 5) {
                csv_target_x = get_theoretical_position('x', nextion_profile, t_sec);
                csv_target_y = get_theoretical_position('y', nextion_profile, t_sec);
            }
            
            csv_record_t rec = {
                .t_sec = t_sec,
                .target_x = csv_target_x,
                .target_y = csv_target_y,
                .t_x = t_x,
                .t_y = t_y,
                .t_z = t_z,
                .s_x = s_x,
                .s_y = s_y,
                .s_z = s_z
            };
            
            // Enviar amostra para a Queue (Tempo limite 0, se a Queue estiver cheia ignoramos amostra para n bloquear motor)
            if (xQueueSend(csv_queue, &rec, 0) != pdTRUE) {
                ESP_LOGW("ACCEL_TASK", "CSV Queue Full! Dropped sample.");
            }

            // Test Print: Imprimir a cada ~500ms (50 loops de 10ms) para não bloquear a UART
            if (++print_counter >= 50) {
                ESP_LOGI("ACCEL_TASK", "Table[g]: X=%.2f, Y=%.2f, Z=%.2f | Specimen[g]: X=%.2f, Y=%.2f, Z=%.2f", t_x, t_y, t_z, s_x, s_y, s_z);
                print_counter = 0;
            }

            if (error_detected) {
                // Abrandar ciclo em caso de fios soltos para evitar Stack Overflow / Panic
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else {
            // Se o perfil acabou, sinaliza a writer task para fechar o ficheiro
            if (is_recording) {
                csv_record_t cmd = {.t_sec = -1.0f, .target_x = 0.0f};
                xQueueSend(csv_queue, &cmd, pdMS_TO_TICKS(100));
                is_recording = false;
                ESP_LOGI("ACCEL_TASK", "Signaled CSV Writer to stop.");
            }
            current_target_pos_x = 0.0f; current_target_pos_y = 0.0f;
        }
        // Delay 10ms = ~100Hz sampling rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
	GPIO_init(); //initialize GPIOs
	check_current_config(); //print GPIO configuration
	init_littlefs(); // Mount LittleFS Partition
	init_uart();
	start_wifi_ap(); //start Wi-Fi
	start_webserver(); //start HTTP server
	config_manager_init(); // Carrega configurações da NVS ou usa defaults
	init_i2c_system(); // Inicia I2C e deteta o MCP23017

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

	csv_queue = xQueueCreate(100, sizeof(csv_record_t)); // Buffer for 1 second of data at 100Hz

	// Tarefas presas ao Core 0 (Comunicações e Background)
	xTaskCreatePinnedToCore(rx_task, "uart_rx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 15, &rxTaskHandle, 0);
	xTaskCreatePinnedToCore(tx_task, "uart_tx_task", UART_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 16, &txTaskHandle, 0);
	//xTaskCreatePinnedToCore(csv_writer_task, "csv_writer_task", 4096, NULL, configMAX_PRIORITIES - 18, NULL, 0);

	// Tasks to control the stepper motors with DRV8825 and RMT independently
	// Presas ao Core 1 para máxima estabilidade e imunidade a interrupções do sistema (WiFi/SPI Flash)
	xTaskCreatePinnedToCore(stepper_rmt_task_1, "stepper_rmt_task_1", 4096, NULL, configMAX_PRIORITIES - 5, NULL, 1);
	xTaskCreatePinnedToCore(stepper_rmt_task_2, "stepper_rmt_task_2", 4096, NULL, configMAX_PRIORITIES - 5, NULL, 1);

	// Dedicated task to monitor structural dynamics through the two ADXL345 I2C accelerometers
	// Presa ao Core 0
	//xTaskCreatePinnedToCore(accelerometer_task, "accel_task", 4096, NULL, configMAX_PRIORITIES - 12, NULL, 0);

	// FreeRTOS main loop: Keep the main task alive with periodic vTaskDelay
	// The actual work happens in the 4 tasks created above
	// vTaskDelay prevents watchdog timeout and allows FreeRTOS scheduler to run
	while (1)
	{	
		vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
	}

}
