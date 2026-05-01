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
#include <dirent.h>
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
    char filename[128];
    char filepath[512];
    
    // 1. Obter nome do ficheiro enviado pelo Browser via Header HTTP
    if (httpd_req_get_hdr_value_str(req, "X-File-Name", filename, sizeof(filename)) != ESP_OK) {
        strcpy(filename, "upload.bin"); // Fallback caso o browser não envie
    }

    // 2. Apagar ficheiros antigos no LittleFS para não acumular lixo
    DIR *dir = opendir("/storage");
    if (dir) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                char old_file[512];
                snprintf(old_file, sizeof(old_file), "/storage/%s", ent->d_name);
                unlink(old_file);
            }
        }
        closedir(dir);
    }

    // 3. Abrir novo ficheiro para escrita com o nome original
    snprintf(filepath, sizeof(filepath), "/storage/%s", filename);
    FILE *fd = fopen(filepath, "w");
    if (!fd) {
        ESP_LOGE(TAG, "Falha ao criar o ficheiro %s", filepath);
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
    char resp[256];
    DIR *dir = opendir("/storage");
    struct dirent *ent;
    char filename[128] = "";
    long filesize = 0;
    bool exists = false;

    // Procurar o primeiro ficheiro presente na memória
    if (dir) {
        while ((ent = readdir(dir)) != NULL) {
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                strncpy(filename, ent->d_name, sizeof(filename)-1);
                char filepath[512];
                snprintf(filepath, sizeof(filepath), "/storage/%s", ent->d_name);
                struct stat st;
                if (stat(filepath, &st) == 0) { filesize = st.st_size; exists = true; }
                break;
            }
        }
        closedir(dir);
    }

    if (exists) {
        snprintf(resp, sizeof(resp), "{\"exists\": true, \"name\": \"%s\", \"size\": %ld}", filename, filesize);
    } else {
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
        "<h3>Upload de Sismo (.dat, .txt, .bin)</h3>"
        "<div id='fileStatus' style='margin-bottom:15px; padding:10px; background:#e0e0e0; border-radius:5px;'>"
        "Ficheiro atual: <span id='fileName'>A verificar...</span>"
        "</div>"
        "<p style='font-size:0.9em; color:#555;'>Ficheiros de texto (.dat/.txt) são convertidos automaticamente. <b>Tamanho Máx: 800 KB</b><br>"
        "<b>Nota: O upload de um novo ficheiro faz overwrite ao antigo.</b></p>"
        "<input type='file' id='fileInput'><br><br>"
        "<button onclick='uploadFile()'>Enviar para a Mesa Sísmica</button>"
        "<p id='status' style='font-weight:bold;color:#333;'></p>"
        "<pre id='logOutput' style='text-align:left; font-size:0.8em; background:#eee; padding:10px; border-radius:5px; display:none; overflow-x:auto;'></pre>"
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
        "    if(data.exists) document.getElementById('fileName').innerHTML = '<b>'+data.name+'</b> (' + (data.size/1024).toFixed(2) + ' KB)';"
        "    else document.getElementById('fileName').innerHTML = '<i>nenhum ficheiro armazenado</i>';"
        "  } catch(e) { document.getElementById('fileName').innerText = 'Erro ao verificar'; }"
        "}"
        "checkFile();"
        "async function uploadFile() {"
        "  const el = document.getElementById('fileInput');"
        "  if(el.files.length === 0) return alert('Selecione um ficheiro!');"
        "  const file = el.files[0];"
        "  let filename = file.name;"
        "  let payload = file;"
        "  let lowerName = filename.toLowerCase();"
        /*"  if (lowerName.endsWith('.dat') || lowerName.endsWith('.txt')) {"
        "    document.getElementById('status').innerText = 'A processar e a converter... aguarde.';"
        "    const text = await file.text();"
        "    const lines = text.split(/\\r?\\n/);"
        "    let pos = [];"
        "    let times = [];"
        "    for (let line of lines) {"
        "      line = line.trim();"
        "      if (!line || line.startsWith('%') || line.startsWith('#') || line.toLowerCase().startsWith('time')) continue;"
        "      line = line.replace(/,/g, '.');" // Troca as vírgulas por pontos decimais
        "      let parts = line.split(/\\s+/);"
        "      if (parts.length >= 2) {"
        "        let t = parseFloat(parts[0]);"
        "        let val = parseFloat(parts[1]);"
        "        if (!isNaN(t) && !isNaN(val)) { times.push(t); pos.push(val); }"
        "      } else if (parts.length === 1) {"
        "        let val = parseFloat(parts[0]);"
        "        if (!isNaN(val)) pos.push(val);"
        "      }"
        "    }"
        "    if (pos.length === 0) { document.getElementById('status').innerText=''; return alert('Nenhum dado numérico encontrado.'); }"
        "    let dt = 0.01;"
        "    if (times.length >= 2) { dt = times[1] - times[0]; if (dt <= 0) dt = 0.01; }" // Calcula dt com base no Tempo
        "    const buffer = new ArrayBuffer(8 + pos.length * 4);"
        "    const view = new DataView(buffer);"
        "    view.setUint32(0, pos.length, true);"
        "    view.setFloat32(4, dt, true);"
        "    for (let i=0; i<pos.length; i++) view.setFloat32(8 + i * 4, pos[i], true);"
        "    payload = new Blob([buffer]);"
        "    filename = filename.substring(0, filename.lastIndexOf('.')) + '.bin';"
        "  }"*/
        "  if (!lowerName.endsWith('.bin')) {"
        "    try {"
        "      document.getElementById('status').innerText = 'A processar e a converter... aguarde.';"
        "      const text = await file.text();"
        "      const lines = text.split(/\\r?\\n/);"
        "      let pos = []; let times = [];"
        "      for (let line of lines) {"
        "        line = line.trim();"
        "        if (!line || line.startsWith('%') || line.startsWith('#') || line.match(/^[a-zA-Z]/)) continue;"
        "        line = line.replace(/,/g, '.');"
        "        let parts = line.split(/\\s+/);"
        "        if (parts.length >= 2) {"
        "          let t = parseFloat(parts[0]); let val = parseFloat(parts[1]);"
        "          if (!isNaN(t) && !isNaN(val)) { times.push(t); pos.push(val); }"
        "        } else if (parts.length === 1) {"
        "          let val = parseFloat(parts[0]);"
        "          if (!isNaN(val)) pos.push(val);"
        "        }"
        "      }"
        "      if (pos.length === 0) { document.getElementById('status').innerText=''; return alert('Nenhum dado numérico encontrado.'); }"
        "      let dt = 0.01;"
        "      if (times.length >= 2) { dt = times[1] - times[0]; if (dt <= 0) dt = 0.01; }"
        "      let logText = '--- Verificação da Conversão ---\\n';"
        "      logText += 'dt: ' + dt.toFixed(5) + ' s\\n';"
        "      logText += 'Pontos: ' + pos.length + '\\n';"
        "      logText += 'Primeiras 10 linhas:\\n';"
        "      for(let i=0; i<Math.min(10, pos.length); i++) {"
        "          logText += (times.length > i ? times[i] : 'N/A') + ' \\t ' + pos[i] + '\\n';"
        "      }"
        "      console.log(logText);"
        "      const logEl = document.getElementById('logOutput');"
        "      logEl.innerText = logText; logEl.style.display = 'block';"
        "      const buffer = new ArrayBuffer(8 + pos.length * 4);"
        "      const view = new DataView(buffer);"
        "      view.setUint32(0, pos.length, true);"
        "      view.setFloat32(4, dt, true);"
        "      for (let i=0; i<pos.length; i++) view.setFloat32(8 + i * 4, pos[i], true);"
        "      payload = new Blob([buffer], {type: 'application/octet-stream'});"
        "      let dotIdx = filename.lastIndexOf('.');"
        "      if(dotIdx > 0) filename = filename.substring(0, dotIdx) + '.bin';"
        "      else filename = filename + '.bin';"
        "    } catch(e) {"
        "      document.getElementById('status').innerText=''; return alert('Erro no processamento: ' + e.message);"
        "    }"
        "  }"
        "  if(payload.size > 800 * 1024) return alert('O ficheiro excede o limite de 800 KB!');"
        "  document.getElementById('status').innerText = 'A enviar ' + file.name + '... aguarde.';"
        "  try {"
        "    const res = await fetch('/upload', { method: 'POST', headers: {'X-File-Name': filename}, body: payload });"
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
    config.stack_size = 8192; // Aumentar a stack do servidor web para evitar overflow
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