#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT UART_NUM_1
#define NEXTION_TERMINATOR 0xFF
#define NEXTION_MAX_LEN 64

extern bool MonitorTask;

// CRC-16 Modbus (polinómio 0xA001, inicial 0xFFFF)
uint16_t nextion_crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void nextion_send_command(const char *cmd)
{
    uart_write_bytes(UART_PORT, cmd, strlen(cmd));
    const uint8_t end[3] = {0xFF, 0xFF, 0xFF};
    uart_write_bytes(UART_PORT, (const char *)end, 3);
    ESP_LOGI("ESP32toNEXTION", "Sent: %s", cmd);
}

void nextion_cmd_syntax(const char *objname, const char *datatype, const char *value)
{
    //objname: nome do objeto (ex: t0, n0, x0)
    //datatype: tipo de dado (ex: txt, val)
    //value: valor a enviar (ex: \"Hello\"", 123, 0x23)
    //exemplo de output: t0.txt=\"Hello\"", n0.val=123, x0.val=0x23
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%s.%s=%s", objname, datatype, value);
    nextion_send_command(cmd);
}

void nextion_send_data_point(uint8_t channel, uint8_t value)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "addt 1,%d,%d", channel, value);
    nextion_send_command(cmd);
}

void sendAckToNextion(bool ok)
{
    if(ok){
        nextion_send_command("va0.val=160"); // ACK OK (0xA0)
    }
    else{
        nextion_send_command("va0.val=161"); // ACK ERROR (0xA1)
        ESP_LOGW("NEXTION", "↩️ Pedido de retransmissão enviado");
    }
}

void return_data_from_nextion(const uint8_t *buff, int idx)
{
    if (idx >= 4 && buff[0] == 0x1A &&
        buff[1] == 0xFF && buff[2] == 0xFF && buff[3] == 0xFF)
    {
        ESP_LOGW("NEXTION", "Invalid Variable name or invalid attribute was used");
    }
    else
    {
        ESP_LOGI("NEXTION", "Received unknown packet (len=%d)", idx);
    }
}


void rxFromNextion(const uint8_t *data, int len)
{
    static uint8_t buffer[NEXTION_MAX_LEN];
    static int index = 0;

    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];
        buffer[index++] = byte;

        // Detects Nextion Terminator "0xFF 0xFF 0xFF"
        if (index >= 3 &&
            buffer[index - 1] == NEXTION_TERMINATOR &&
            buffer[index - 2] == NEXTION_TERMINATOR &&
            buffer[index - 3] == NEXTION_TERMINATOR) {

            ESP_LOG_BUFFER_HEXDUMP("RX_TASK", buffer, index, ESP_LOG_INFO);

            // Touch Event (0x65) -------------------------------------------------------------
            // Example: 0x65 0x00 0x01 0x01 0xFF 0xFF 0xFF
            // 0x65 = Touch Event ; 0x00 = Page ID ; 0x01 = Component ID ; 0x01 = Event (0=release,1=press); 0xFF 0xFF 0xFF = Terminator
            if (buffer[0] == 0x65 && index >= 7) {
                uint8_t page = buffer[1];
                uint8_t component_id = buffer[2];
                uint8_t event = buffer[3];

                ESP_LOGI("NEXTION", "Evento Touch: page=%d, comp=%d, event=%d",
                         page, component_id, event);

                if (component_id == 5 && event == 1) { // botão b0 press
                    ESP_LOGI("NEXTION", "Botão B0 pressionado!");
                    nextion_send_command("g0.txt=\"b0 on\"");
                } else if (component_id == 5 && event == 0) { // botão b0 release
                    ESP_LOGI("NEXTION", "Botão B0 libertado!");
                    nextion_send_command("g0.txt=\"b0 off\"");
                }
            }
            /*// End Touch Event (0x65) -------------------------------------------------------------
            
            // ---- PARTE 2: texto adicional ("B0press"/"B0rel") ----
            buffer[index - 3] = '\0'; // remove terminador
            if (strstr((char *)buffer, "B0press")) {
                ESP_LOGI("NEXTION", "Botão B0 pressionado (via texto)!");
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if (strstr((char *)buffer, "B0rel")) {
                ESP_LOGI("NEXTION", "Botão B0 libertado (via texto)!");
            }*/
            
            // Custom Data Packet (0x55) -------------------------------------------------------------
            // Example: 55 01 F4 01 FA 00 32 00 23 00 63 76 FF FF FF
            // 55 - Custom Send Event ; 01 - Type of Data (Ex: Parameters) ; F4 01 - Frequency (5.00Hz) ; FA 00 - Velocity (2.50) ; 32 00 - Acc Tranversal (0.50G) ; 23 00 - Temp (0.35s) ; 63 76 - CRC16 ; FF FF FF - Terminator
            else if (buffer[0] == 0x55 && buffer[1] == 0x01) {
                uint16_t freq = buffer[2] | (buffer[3] << 8);
                uint16_t vel  = buffer[4] | (buffer[5] << 8);
                uint16_t acc  = buffer[6] | (buffer[7] << 8);
                uint16_t tmp  = buffer[8] | (buffer[9] << 8);
                uint16_t recv_crc = buffer[10] | (buffer[11] << 8);

                uint8_t payload[] = {buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9]};
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload));

                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - F=%.2fHz V=%.2fm/s AccT=%.2fs Tmp=%.2fs",
                            freq/100.0, vel/100.0, acc/100.0, tmp/100.0);
                    sendAckToNextion(true);
                    MonitorTask = true; // ativa monitorização de stack
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(false);
                }
            }
            else
            {
                return_data_from_nextion(buffer, index);
            }
            index = 0; // reset buffer
        }

        // Protege de overflow
        if (index >= NEXTION_MAX_LEN)
            index = 0;
    }
}

void txToNextion(void)
{
/*  nextion_send_command("t0.txt=\"Huzzah32 Online\"");
    vTaskDelay(pdMS_TO_TICKS(2000));

    nextion_send_command("n0.val=123");
    vTaskDelay(pdMS_TO_TICKS(2000));

    nextion_cmd_syntax("n0", "val", "0x23"); //nextion_send_command("n0.val=0x23");
    vTaskDelay(pdMS_TO_TICKS(2000));

    nextion_send_command("x0.val=0xA0");
    vTaskDelay(pdMS_TO_TICKS(2000));

    nextion_cmd_syntax("t0", "txt", "\"Hello Nextion!\""); //nextion_send_command("t0.txt=\"Hello Nextion!\"");
    vTaskDelay(pdMS_TO_TICKS(2000));

    nextion_send_command("addt 6,0,(randset 0,100)");
    vTaskDelay(pdMS_TO_TICKS(2000)); */

    vTaskDelay(pdMS_TO_TICKS(1000));
}