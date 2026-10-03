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
#include "functions.h"

static const int RX_BUF_SIZE = 1024;
static const char* TAG1 = "UART_to_NEXTION";
static const char* TAG2 = "UART_to_MCU2";
bool MonitorTask = false;

#define TXD_PIN (17)
#define RXD_PIN (16)
#define UART_BAUD_RATE (9600)
#define UART_PORT UART_NUM_1
#define DEBUG_UART // enable only for debug, comment to disable

void init_uart_to_Nextion(void)
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

    ESP_LOGI(TAG1, "Nextion initialized on UART%d (TX=%d, RX=%d, %d baud)",
             UART_PORT, TXD_PIN, RXD_PIN , UART_BAUD_RATE);    
}

void init_uart_to_mcu2(void)
{
    const uart_config_t uart_telemetry_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_TELEMETRY_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_TELEMETRY_NUM, &uart_telemetry_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_TELEMETRY_NUM, UART_TELEMETRY_TX_PIN, UART_TELEMETRY_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_pull_mode(UART_TELEMETRY_RX_PIN, GPIO_PULLUP_ONLY); // Prevents noise when cable is disconnected
    ESP_LOGI(TAG2, "Telemetry UART initialized on UART%d (TX=%d, RX=%d, 115200 baud)", UART_TELEMETRY_NUM, UART_TELEMETRY_TX_PIN, UART_TELEMETRY_RX_PIN);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_PORT, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

// --- RX and TX tasks
void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
/*      sendData(TX_TASK_TAG, "t0.txt=\"Hello world\"");
        vTaskDelay(2000 / portTICK_PERIOD_MS); */

        txToNextion();
        vTaskDelay(500 / portTICK_PERIOD_MS); // Sends every 500ms
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
            #ifdef DEBUG_UART 
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            #endif
                rxFromNextion(data, rxBytes);
        }
    }
    free(data);
}

void telemetry_rx_task(void *arg)
{
    static const char *TEL_RX_TAG = "TELEMETRY_RX";
    esp_log_level_set(TEL_RX_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_TELEMETRY_NUM, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0; // null terminate
            
            // Checks ASCII commands received from MCU2
            if (strstr((const char*)data, "WIFI_OK")) {
                sendAckToNextion(301); // ACK WIFI CONNECTED
                ESP_LOGI(TEL_RX_TAG, "Feedback MCU2 -> WIFI_OK -> Notified Nextion!");
            }
            else if (strstr((const char*)data, "WIFI_DIS")) {
                sendAckToNextion(302); // ACK WIFI DISCONNECTED
                ESP_LOGW(TEL_RX_TAG, "Feedback MCU2 -> WIFI_DIS -> Notified Nextion!");
            }
            else if (strstr((const char*)data, "CSV_RDY")) {
                sendAckToNextion(312); // ACK FILE SAVED
                ESP_LOGI(TEL_RX_TAG, "Feedback MCU2 -> CSV_RDY -> Notified Nextion!");
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
        if (MonitorTask) {
            UBaseType_t rx_stack = uxTaskGetStackHighWaterMark(rxTaskHandle);
            UBaseType_t tx_stack = uxTaskGetStackHighWaterMark(txTaskHandle);

            ESP_LOGI("STACK_MON", "RX task stack min free: %u bytes", rx_stack * 4);
            ESP_LOGI("STACK_MON", "TX task stack min free: %u bytes", tx_stack * 4);

            MonitorTask = false;
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // updates every 0.5 seconds
    }
}
