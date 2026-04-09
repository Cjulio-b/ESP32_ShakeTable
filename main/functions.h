
#ifndef MAIN_FUNCTIONS_H_
#define MAIN_FUNCTIONS_H_

#include "esp_mac.h"
#include "esp_err.h"          // Para o tipo esp_err_t
#include "esp_http_server.h"  // Para httpd_req_t e httpd_handle_t

// GLOBAL Variables -----------------------------------
extern bool MonitorTask;

// GPIO functions ---------------------------------------
void check_current_config(void);
void testing_led(void);
void GPIO_init(void);

// WIFI - HTTP Server functions ----------------------------------
void start_wifi_ap(void);
esp_err_t gpio_handler(httpd_req_t *req);
httpd_handle_t start_webserver(void);

// UART functions -----------------------------------------------
void init_uart(void);
int sendData(const char* logName, const char* data);
void rx_task(void *arg);
void tx_task(void *arg);
void monitor_task(void *arg);

//NEXTION functions -----------------------------------------------
void nextion_send_command(const char *cmd);
void nextion_send_data_point(uint8_t channel, uint8_t value);
void rxFromNextion(const uint8_t *data, int len);
void txToNextion(void);
void nextion_cmd_syntax(const char *objname, const char *datatype, const char *value);
uint16_t nextion_crc16_modbus(const uint8_t *data, size_t len);
void sendAckToNextion(bool ok);
void return_data_from_nextion(const uint8_t *buff, int idx);

// STEPPER MOTOR functions -----------------------------------------------
void step_motor(bool direction);

// DRV8825 + RMT stepper motor functions -----------------------------------------------
typedef struct stepper_rmt_context_t stepper_rmt_context_t;
stepper_rmt_context_t* stepper_rmt_init(uint8_t gpio_en, uint8_t gpio_dir, uint8_t gpio_step);
esp_err_t stepper_rmt_run_steps(stepper_rmt_context_t *ctx, uint32_t uniform_speed_hz, uint32_t uniform_samples, uint32_t accel_samples, uint32_t decel_samples, bool direction);
void stepper_rmt_task_1(void *arg);
void stepper_rmt_task_2(void *arg);

#endif /* MAIN_FUNCTIONS_H_ */