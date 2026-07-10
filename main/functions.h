
#ifndef MAIN_FUNCTIONS_H_
#define MAIN_FUNCTIONS_H_

#include "esp_mac.h"
#include "esp_err.h"          // Para o tipo esp_err_t
#include "esp_http_server.h"  // Para httpd_req_t e httpd_handle_t
#include "esp_adc/adc_oneshot.h"
#include "kinematics.h"
#include "driver/i2c_master.h"

// Hardware Pin Definitions ---------------------------
#define UART_TELEMETRY_NUM UART_NUM_2
#define UART_TELEMETRY_TX_PIN 26
#define UART_TELEMETRY_RX_PIN 25
#define SYNC_GPIO_PIN 21

// GLOBAL Variables -----------------------------------
extern bool MonitorTask;
extern volatile bool motor1_busy;
extern volatile bool motor2_busy;
extern volatile bool motor1_ready;
extern volatile bool motor2_ready;

// Nextion Profile Parameters -------------------------
extern volatile int8_t nextion_profile;
extern float nextion_target_freq_x;
extern float nextion_target_freq_y;
extern float nextion_target_disp_x;
extern float nextion_target_disp_y;
extern float nextion_target_time_s;
extern float nextion_multistep_freq_x[4];
extern float nextion_multistep_freq_y[4];
extern float nextion_multistep_time_x[4];
extern float nextion_multistep_time_y[4];
extern float nextion_trapz_start_freq_x;
extern float nextion_trapz_cruise_freq_x;
extern float nextion_trapz_end_freq_x;
extern float nextion_trapz_accel_time_x;
extern float nextion_trapz_cruise_time_x;
extern float nextion_trapz_decel_time_x;
extern float nextion_trapz_start_freq_y;
extern float nextion_trapz_cruise_freq_y;
extern float nextion_trapz_end_freq_y;
extern float nextion_trapz_accel_time_y;
extern float nextion_trapz_cruise_time_y;
extern float nextion_trapz_decel_time_y;
extern float nextion_rt_min_freq_x;
extern float nextion_rt_max_freq_x;
extern float nextion_rt_min_freq_y;
extern float nextion_rt_max_freq_y;
extern float nextion_sweep_min_freq_x;
extern float nextion_sweep_max_freq_x;
extern float nextion_sweep_min_freq_y;
extern float nextion_sweep_max_freq_y;
extern bool nextion_sweep_isBid_x;
extern bool nextion_sweep_isBid_y;
extern bool parameters_recv;

// I2C Handles globais --------------------------------
extern i2c_master_bus_handle_t i2c_bus_handle;
extern i2c_master_bus_handle_t i2c_bus_2_handle;
extern i2c_master_dev_handle_t mcp_handle;

// Global Tracking Variables for Data Logging ---------
extern volatile float current_target_pos_x;
extern volatile float current_target_pos_y;

// Global Configuration Structures --------------------
extern shake_table_config_t table_config_x;
extern shake_table_config_t table_config_y;

// GPIO functions ---------------------------------------
void check_current_config(void);
void testing_led(void);
void GPIO_init(void);

// LittleFS functions -------------------------------------------
void init_littlefs(void);

// WIFI - HTTP Server functions ----------------------------------
void start_wifi_ap(void);
httpd_handle_t start_webserver(void);

// I2C functions ------------------------------------------------
void init_i2c_system(void);
void set_all_steppers_microsteps(uint16_t micro_x, uint16_t micro_y);

// NVS Configuration Manager functions --------------------------
#include "config_manager.h"

// UART functions -----------------------------------------------
void init_uart_to_Nextion(void);
void init_uart_to_mcu2(void);
int sendData(const char* logName, const char* data);
void rx_task(void *arg);
void tx_task(void *arg);
void telemetry_rx_task(void *arg);
void monitor_task(void *arg);

//NEXTION functions -----------------------------------------------
void nextion_send_command(const char *cmd);
void nextion_send_data_point(uint8_t channel, uint8_t value);
void rxFromNextion(const uint8_t *data, int len);
void txToNextion(void);
void nextion_cmd_syntax(const char *objname, const char *datatype, const char *value);
uint16_t nextion_crc16_modbus(const uint8_t *data, size_t len);
void sendAckToNextion(int ackmsg);
void nextion_set_current_page(uint8_t page);
//void nextion_notify_wifi_connected(void);
//void nextion_notify_wifi_disconnected(void);
//void nextion_notify_upload_success(void);
//void nextion_notify_upload_error(void);
//void nextion_notify_test_result_available(void);
void return_data_from_nextion(const uint8_t *buff, int idx);

// STEPPER MOTOR functions -----------------------------------------------
void step_motor(bool direction);

// DRV8825 + RMT stepper motor functions -----------------------------------------------
typedef struct stepper_rmt_context_t stepper_rmt_context_t;
stepper_rmt_context_t* stepper_rmt_init(uint8_t gpio_en, uint8_t gpio_dir, uint8_t gpio_step);
esp_err_t stepper_rmt_run_steps(stepper_rmt_context_t *ctx, uint32_t uniform_speed_hz, uint32_t uniform_samples, uint32_t accel_samples, uint32_t decel_samples, bool direction);
esp_err_t stepper_rmt_homing(stepper_rmt_context_t *ctx, uint8_t gpio_limit_right, uint8_t gpio_limit_left, const shake_table_config_t *table_config);
esp_err_t stepper_rmt_run_realtime_sine_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float duration_s, adc_oneshot_unit_handle_t adc_handle, adc_channel_t adc_chan, float min_hz, float max_hz, const shake_table_config_t *table_config);
esp_err_t stepper_rmt_run_trapezoidal_freq_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float start_freq_hz, float cruise_freq_hz, float end_freq_hz, float accel_time_s, float cruise_time_s, float decel_time_s, const shake_table_config_t *table_config);
esp_err_t stepper_rmt_run_multistep_freq_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, const float *freqs_hz, const float *times_s, uint8_t num_stages, float total_duration_s, float blend_time_s, const shake_table_config_t *table_config);
esp_err_t stepper_rmt_run_sweep_profile(stepper_rmt_context_t *ctx, float target_p2p_mm, float start_freq_hz, float end_freq_hz, float duration_s, bool is_bidirectional, const shake_table_config_t *table_config);
bool get_stored_sismo_file(char* filepath_out, size_t max_len);
void stepper_rmt_task_1(void *arg);
void stepper_rmt_task_2(void *arg);

#endif /* MAIN_FUNCTIONS_H_ */