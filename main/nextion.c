#include "driver/uart.h"
#include "esp_log.h"
#include "functions.h"
#include <string.h>

#define UART_PORT UART_NUM_1
#define NEXTION_TERMINATOR 0xFF
#define NEXTION_MAX_LEN 64

// Variáveis globais para armazenar os parâmetros validados do Nextion
float nextion_target_freq_x = 0.0f;
float nextion_target_freq_y = 0.0f;
float nextion_target_disp_x = 0.0f;
float nextion_target_disp_y = 0.0f;
float nextion_target_time_s = 0.0f;

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

void sendAckToNextion(int ackmsg)
{
    /* 
    // Acknowledge to NEXTION HMI
        ackmsg==160 --> "Data sucessfuly received."
        ackmsg==161 --> "Error while receiving data. Try again..."
        ackmsg==162 --> "Performing homming of the table."
        ackmsg==163 --> "Starting table motion profile."
        ackmsg==164 --> "Table motion ended with sucess."
        ackmsg==165 --> "Error during table motion."
        ackmsg==166 --> "Invalid parameters received."
     */
    switch (ackmsg) {
        case 160:
            nextion_send_command("va0.val=160"); // ACK OK (0xA0)
            break;
        case 161:
            nextion_send_command("va0.val=161"); // ACK ERROR (0xA1)
            ESP_LOGW("NEXTION", "↩️ Pedido de retransmissão enviado");
            break;
        case 162:
            nextion_send_command("va0.val=162"); // ACK HOMMING (0xA2)
            break;
        case 163:
            nextion_send_command("va0.val=163"); // ACK START MOTION (0xA3)
            break;
        case 164:
            nextion_send_command("va0.val=164"); // ACK MOTION END (0xA4)
            break;
        case 165:
            nextion_send_command("va0.val=165"); // ACK MOTION ERROR (0xA5)
            ESP_LOGW("NEXTION", "↩️ Erro durante o movimento da mesa");
            break;
        default:
            ESP_LOGW("NEXTION", "Ack desconhecido: %d", ackmsg);
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

bool displacement_parameter_validation(float disp_mm)
{
    if (disp_mm < 0.0f || disp_mm > 33.0f) { // Limite teórico da mesa
        ESP_LOGW("NEXTION", "Displacement parameter out of range");
        return false;
    } else {
        return true;
    }
}

bool frequency_parameter_validation(float freq_hz)
{
    if (freq_hz < 0.0f || freq_hz > 50.0f) { // Exemplo: limite 50Hz
        ESP_LOGW("NEXTION", "Frequency parameter out of range");
        return false;
    } else {
        return true;
    }
}

bool time_parameter_validation(float time_s)
{
    if (time_s <= 0.0f || time_s > 1800.0f) { // Limite máximo: 30 minutos (1800s)
        ESP_LOGW("NEXTION", "Time parameter out of range");
        return false;
    } else {
        return true;
    }
}

bool compare_disp_w_freq(float disp_mm, float freq_hz){
    if ((disp_mm == 0.0f && freq_hz != 0.0f) || (disp_mm != 0.0f && freq_hz == 0.0f)) {
        ESP_LOGW("NEXTION", "Displacement and Frequency parameters must both be zero (no motion) or both be greater than zero (motion)");
        return false;
    } else {
        return true;
    }
}

void rxFromNextion(const uint8_t *data, int len)
{
    static uint8_t buffer[NEXTION_MAX_LEN];
    static int index = 0;
    static bool sine_parameters_recv = false;

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
                if (page==2){
                    if (component_id == 16 && event == 0 && sine_parameters_recv) { // START button Sinewave Profile
                        ESP_LOGI("NEXTION", "Start SineWave Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING (A mesa vai preparar-se primeiro)
                        nextion_profile = 1; // Sinewave Profile 
                    } else if (component_id == 17 && event == 0) { // STOP button Sinewave Profile
                        ESP_LOGI("NEXTION", "Stop SineWave Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // Sinal de STOP (aborta loop nos motores)
                        sine_parameters_recv = false;
                    }
                }
            }
            // End Touch Event (0x65) -------------------------------------------------------------
            
            // Shaking Table Profiles Data Packet (0x55) -------------------------------------------------------------
            // Example: 55 01 F4 01 FA 00 32 00 23 00 63 76 FF FF FF
            // 55 - Custom Send Event ; 01 - Type of Data (Ex: Sine Parameters) ; F4 01 -  freq motor axis y (5.00Hz) ; FA 00 - frequency motor axis x (2.50Hz) ; 32 00 - displacement y (0.50mm) ; 23 00 - displacement x (0.35mm) ; 58 02 - time (60s) ; 63 2B - CRC16 ; FF FF FF - Terminator
            else if (buffer[0] == 0x55 && buffer[1] == 0x01) {
                uint16_t freq_y = buffer[2] | (buffer[3] << 8);
                uint16_t freq_x = buffer[4] | (buffer[5] << 8);
                uint16_t disp_y = buffer[6] | (buffer[7] << 8);
                uint16_t disp_x = buffer[8] | (buffer[9] << 8);
                uint16_t time_s = buffer[10] | (buffer[11] << 8);
                uint16_t recv_crc = buffer[12] | (buffer[13] << 8);

                uint8_t payload[] = {buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]}; // dados sem header, tipo e CRC    
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload)); // calcula CRC dos dados recebidos

                // Conversão para floats (escala x10 enviada pelo Nextion)
                float f_freq_y = freq_y / 10.0f;
                float f_freq_x = freq_x / 10.0f;
                float f_disp_y = disp_y / 10.0f;
                float f_disp_x = disp_x / 10.0f;
                float f_time_s = time_s / 10.0f;    

                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - F_y=%.1fHz, F_x=%.1fHz, D_y=%.1fmm, D_x=%.1fmm, Tmp=%.1fs",
                            f_freq_y, f_freq_x, f_disp_y, f_disp_x, f_time_s);
                    if(displacement_parameter_validation(f_disp_x) && displacement_parameter_validation(f_disp_y) &&
                       frequency_parameter_validation(f_freq_x) && frequency_parameter_validation(f_freq_y) &&
                       time_parameter_validation(f_time_s) && compare_disp_w_freq(f_disp_x, f_freq_x) && compare_disp_w_freq(f_disp_y, f_freq_y)) {
                        ESP_LOGI("NEXTION", "All parameters are valid. Waiting to start motion profile...");
                        
                        nextion_target_freq_x = f_freq_x;
                        nextion_target_freq_y = f_freq_y;
                        nextion_target_disp_x = f_disp_x;
                        nextion_target_disp_y = f_disp_y;
                        nextion_target_time_s = f_time_s;
                                                
                        sendAckToNextion(160); // ACK DATA OK
                        MonitorTask = true; // ativa monitorização de stack
                        sine_parameters_recv=true;
                    }else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR;
                }
                // End Shaking Table Profiles Data Packet (0x55) -------------------------------------------------------------
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