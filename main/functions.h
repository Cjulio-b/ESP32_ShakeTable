
#ifndef MAIN_FUNCTIONS_H_
#define MAIN_FUNCTIONS_H_

#include "esp_mac.h"
#include "esp_err.h"          // Para o tipo esp_err_t
#include "esp_http_server.h"  // Para httpd_req_t e httpd_handle_t

void check_current_config(void);
void testing_led(void);
void GPIO_init(void);
void start_wifi_ap(void);
esp_err_t gpio_handler(httpd_req_t *req);
httpd_handle_t start_webserver(void);


#endif /* MAIN_FUNCTIONS_H_ */