
#ifndef MAIN_FUNCTIONS_H_
#define MAIN_FUNCTIONS_H_

#include "esp_mac.h"
#include "esp_err.h"          // Para o tipo esp_err_t
#include "esp_http_server.h"  // Para httpd_req_t e httpd_handle_t

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


#endif /* MAIN_FUNCTIONS_H_ */