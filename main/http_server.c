/*#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_http_server.h"

esp_err_t gpio_handler(httpd_req_t *req)
{
    int gpio21_level = gpio_get_level(GPIO_NUM_21);
    int gpio26_level = gpio_get_level(GPIO_NUM_26);

    char response[100];
    snprintf(response, sizeof(response),
             "<html><body>"
             "<h1>GPIO Viewer</h1>"
             "<p>GPIO 21: %d</p>"
             "<p>GPIO 26: %d</p>"
             "</body></html>",
             gpio21_level, gpio26_level);

    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    httpd_start(&server, &config);

    httpd_uri_t gpio_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = gpio_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &gpio_uri);
    
    return server;
}*/

// Versao 2 -----------------------
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "esp_http_server.h"

#define TAG "Funcoes"

// GPIOs que queremos monitorizar
#define GPIO_OUTPUT GPIO_NUM_26
#define GPIO_INPUT  GPIO_NUM_21


// ========================
// Handler para JSON /gpio
// ========================
esp_err_t gpio_handler(httpd_req_t *req) {
    char resp[128];
    int gpio21_val = gpio_get_level(GPIO_INPUT);
    int gpio26_val = gpio_get_level(GPIO_OUTPUT);

    snprintf(resp, sizeof(resp),
             "{\"gpio21\": %d, \"gpio26\": %d}", gpio21_val, gpio26_val);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ========================
// Página HTML principal
// ========================
esp_err_t index_handler(httpd_req_t *req) {
    const char *html =
        "<!DOCTYPE html>"
        "<html lang='pt'>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<title>Monitor GPIO ESP32</title>"
        "<style>"
        "body{font-family:Arial;text-align:center;margin-top:40px;}"
        ".box{display:inline-block;padding:20px;margin:10px;border:2px solid #333;"
        "border-radius:10px;width:150px;}"
        ".on{background-color:#4CAF50;color:white;}"
        ".off{background-color:#f44336;color:white;}"
        "</style>"
        "</head>"
        "<body>"
        "<h2>Estado dos GPIOs (ESP32)</h2>"
        "<div id='gpio21' class='box'>GPIO21: --</div>"
        "<div id='gpio26' class='box'>GPIO26: --</div>"
        "<script>"
        "async function atualizarGPIO(){"
        "try{"
        "const res=await fetch('/gpio');"
        "const data=await res.json();"
        "atualizarBox('gpio21',data.gpio21);"
        "atualizarBox('gpio26',data.gpio26);"
        "}catch(e){console.error(e);}}"
        "function atualizarBox(id,val){"
        "const el=document.getElementById(id);"
        "el.textContent=id.toUpperCase()+': '+val;"
        "el.className='box '+(val?'on':'off');}"
        "setInterval(atualizarGPIO,500);"
        "</script>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ========================
// Inicia o servidor HTTP
// ========================
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        // Rota da página principal
        httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &index_uri);

        // Rota JSON /gpio
        httpd_uri_t gpio_uri = {
            .uri = "/gpio",
            .method = HTTP_GET,
            .handler = gpio_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &gpio_uri);

        ESP_LOGI(TAG, "Servidor HTTP iniciado!");
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar servidor HTTP!");
    }
    return server;
}