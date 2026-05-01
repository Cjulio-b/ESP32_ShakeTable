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
#include <sys/param.h>
#include <sys/stat.h>
#include <unistd.h>
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
// Handler para Upload de Ficheiros
// ========================
esp_err_t upload_handler(httpd_req_t *req) {
    FILE *fd = fopen("/storage/sismo.bin", "w");
    if (!fd) {
        ESP_LOGE(TAG, "Falha ao criar o ficheiro /storage/sismo.bin");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char buf[512];
    int received;
    int remaining = req->content_len;

    while (remaining > 0) {
        if ((received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue; // Tenta novamente
            }
            fclose(fd);
            ESP_LOGE(TAG, "Erro ao receber ficheiro durante o upload!");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        // Escrever o bloco de dados recebido diretamente no disco (LittleFS)
        fwrite(buf, 1, received, fd);
        remaining -= received;
    }
    fclose(fd);
    
    ESP_LOGI(TAG, "Upload concluido com sucesso. Tamanho recebido: %d bytes", req->content_len);
    httpd_resp_sendstr(req, "Ficheiro guardado no ESP32 com sucesso!");
    return ESP_OK;
}

// ========================
// Handler para Info do Ficheiro
// ========================
esp_err_t fileinfo_handler(httpd_req_t *req) {
    struct stat st;
    char resp[128];
    if (stat("/storage/sismo.bin", &st) == 0) {
        snprintf(resp, sizeof(resp), "{\"exists\": true, \"size\": %ld}", (long)st.st_size);
    } else {
        httpd_resp_sendstr(req, "Nenhum ficheiro para apagar.");
        snprintf(resp, sizeof(resp), "{\"exists\": false}");
    }
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
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>Monitor GPIO ESP32</title>"
        "<style>"
        "body{font-family:Arial;text-align:center;margin-top:40px;}"
        ".box{display:inline-block;padding:20px;margin:10px;border:2px solid #333;"
        "border-radius:10px;width:150px;}"
        ".on{background-color:#4CAF50;color:white;}"
        ".off{background-color:#f44336;color:white;}"
        ".upload-section{margin-top:40px;padding:20px;background:#f9f9f9;border-radius:10px;display:inline-block;}"
        "</style>"
        "</head>"
        "<body>"
        "<h2>Estado dos GPIOs (ESP32)</h2>"
        "<div id='gpio21' class='box'>GPIO21: --</div>"
        "<div id='gpio26' class='box'>GPIO26: --</div>"
        "<div class='upload-section'>"
        "<h3>Upload de Sismo (.bin)</h3>"
        "<div id='fileStatus' style='margin-bottom:15px; padding:10px; background:#e0e0e0; border-radius:5px;'>"
        "Ficheiro atual: <span id='fileName'>A verificar...</span>"
        "</div>"
        "<p style='font-size:0.9em; color:#555;'>Apenas ficheiros pré-processados (.bin). Tamanho Máx: 800 KB<br>"
        "<b>Nota: O upload de um novo ficheiro faz overwrite ao antigo.</b></p>"
        "<input type='file' id='fileInput' accept='.bin'><br><br>"
        "<button onclick='uploadFile()'>Enviar para a Mesa Sísmica</button>"
        "<p id='status' style='font-weight:bold;color:#333;'></p>"
        "</div>"
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
        "async function checkFile() {"
        "  try {"
        "    const res = await fetch('/fileinfo');"
        "    const data = await res.json();"
        "    if(data.exists) document.getElementById('fileName').innerHTML = '<b>sismo.bin</b> (' + (data.size/1024).toFixed(2) + ' KB)';"
        "    else document.getElementById('fileName').innerHTML = '<i>nenhum ficheiro armazenado</i>';"
        "  } catch(e) { document.getElementById('fileName').innerText = 'Erro ao verificar'; }"
        "}"
        "checkFile();"
        "async function uploadFile() {"
        "  const el = document.getElementById('fileInput');"
        "  if(el.files.length === 0) return alert('Selecione um ficheiro!');"
        "  const file = el.files[0];"
        "  if(file.size > 800 * 1024) return alert('O ficheiro excede o limite de 800 KB!');"
        "  document.getElementById('status').innerText = 'A enviar ' + file.name + '... aguarde.';"
        "  try {"
        "    const res = await fetch('/upload', { method: 'POST', body: file });"
        "    if(res.ok) { document.getElementById('status').innerText = await res.text(); checkFile(); }"
        "    else document.getElementById('status').innerText = 'Erro do Servidor: ' + res.status;"
        "  } catch(e) {"
        "    document.getElementById('status').innerText = 'Falha de rede (Ligação perdida com o ESP32)';"
        "    console.error(e);"
        "  }"
        "}"
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

        // Rota de Upload
        httpd_uri_t upload_uri = {
            .uri = "/upload",
            .method = HTTP_POST,
            .handler = upload_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &upload_uri);

        // Rota de Info do Ficheiro
        httpd_uri_t fileinfo_uri = {
            .uri = "/fileinfo",
            .method = HTTP_GET,
            .handler = fileinfo_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &fileinfo_uri);

        ESP_LOGI(TAG, "Servidor HTTP iniciado!");
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar servidor HTTP!");
    }
    return server;
}