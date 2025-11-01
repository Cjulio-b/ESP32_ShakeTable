/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;
static const char* TAG = "UART_ASYNC";

#define TXD_PIN (17)
#define RXD_PIN (16)
#define UART_BAUD_RATE (9600)
#define UART_PORT UART_NUM_1

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "Nextion initialized on UART%d (TX=%d, RX=%d, %d baud)",
             UART_PORT, TXD_PIN, RXD_PIN , UART_BAUD_RATE);    
}
// testar as duas versoes "sendData" e "nextion_send_command"
int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_PORT, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

void nextion_send_command(const char *cmd)
{
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    const uint8_t end[3] = {0xFF, 0xFF, 0xFF};
    uart_write_bytes(UART_PORT, (const char *)end, 3);
    ESP_LOGI("NEXTION", "Sent: %s", cmd);
}

void nextion_send_data_point(uint8_t channel, uint8_t value)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "addt 1,%d,%d", channel, value);
    nextion_send_command(cmd);
}


// --- RX and TX tasks
void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        nextion_send_command("t0.txt=\"Huzzah32 Online\"");
        vTaskDelay(pdMS_TO_TICKS(2000));

        nextion_send_command("n0.val=123");
        vTaskDelay(pdMS_TO_TICKS(2000));

        nextion_send_command("n0.val=0x23");
        vTaskDelay(pdMS_TO_TICKS(2000));

        nextion_send_command("x0.val=1.5");
        vTaskDelay(pdMS_TO_TICKS(2000));

        nextion_send_command("addt 6,0,random(0,255)");
        vTaskDelay(pdMS_TO_TICKS(2000));

        
    }
}

void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_PORT, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            
            if (strstr((char *)data, "B0rel")) {
                ESP_LOGI("RX_TASK", "Botão b0 largado!");
                nextion_send_command("g0.txt=\"Botão b0 OFF\"");
            }
            if (strstr((char *)data, "B0press")) {
                ESP_LOGI("RX_TASK", "Botão b0 pressionado!");
                nextion_send_command("g0.txt=\"Botão b0 ON\"");
            } 
        }
    }
    free(data);
}

TaskHandle_t rxTaskHandle = NULL;
TaskHandle_t txTaskHandle = NULL;

void monitor_task(void *arg)
{
    while (1) {
        UBaseType_t rx_stack = uxTaskGetStackHighWaterMark(rxTaskHandle);
        UBaseType_t tx_stack = uxTaskGetStackHighWaterMark(txTaskHandle);

        ESP_LOGI("STACK_MON", "RX task stack min free: %u bytes", rx_stack * 4);
        ESP_LOGI("STACK_MON", "TX task stack min free: %u bytes", tx_stack * 4);

        vTaskDelay(pdMS_TO_TICKS(5000)); // atualiza a cada 5 segundos
    }
}
