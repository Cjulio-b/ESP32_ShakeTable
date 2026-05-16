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
float nextion_multistep_freq_x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float nextion_multistep_freq_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float nextion_multistep_time_x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float nextion_multistep_time_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float nextion_trapz_start_freq_x = 0.0f;
float nextion_trapz_cruise_freq_x = 0.0f;
float nextion_trapz_end_freq_x = 0.0f;
float nextion_trapz_accel_time_x = 0.0f;
float nextion_trapz_cruise_time_x = 0.0f;
float nextion_trapz_decel_time_x = 0.0f;
float nextion_trapz_start_freq_y = 0.0f;
float nextion_trapz_cruise_freq_y = 0.0f;
float nextion_trapz_end_freq_y = 0.0f;
float nextion_trapz_accel_time_y = 0.0f;
float nextion_trapz_cruise_time_y = 0.0f;
float nextion_trapz_decel_time_y = 0.0f;
float nextion_rt_min_freq_x = 0.0f;
float nextion_rt_max_freq_x = 0.0f;
float nextion_rt_min_freq_y = 0.0f;
float nextion_rt_max_freq_y = 0.0f;
float nextion_sweep_min_freq_x = 0.0f;
float nextion_sweep_max_freq_x = 0.0f;
float nextion_sweep_min_freq_y = 0.0f;
float nextion_sweep_max_freq_y = 0.0f;
bool nextion_sweep_isBid_x = false;
bool nextion_sweep_isBid_y = false;
bool parameters_recv = false;

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
        case 166:
            nextion_send_command("va0.val=166"); // ACK INVALID PARAMS (0xA6)
            ESP_LOGW("NEXTION", "↩️ Parametros invalidos recebidos do HMI");
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
                    if (component_id == 16 && event == 0 && parameters_recv) { // START button Sinewave Profile
                        ESP_LOGI("NEXTION", "Start SineWave Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING (A mesa vai preparar-se primeiro)
                        nextion_profile = 1; // Sinewave Profile 
                    } else if (component_id == 17 && event == 0) { // STOP button Sinewave Profile
                        ESP_LOGI("NEXTION", "Stop SineWave Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // Sinal de STOP (aborta loop nos motores)
                        parameters_recv = false;
                    }
                } else if (page == 3) { // <-- SUBSTITUI 3 PELO ID DA PÁGINA MULTI-STEP NO NEXTION EDITOR
                    if (component_id == 27 && event == 0 && parameters_recv) { // START button Multi-Step Profile (Substitui 16 pelo ID do botão)
                        ESP_LOGI("NEXTION", "Start Multi-Step Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 2; // Multi-Step Profile 
                    } else if (component_id == 28 && event == 0) { // STOP button Multi-Step Profile (Substitui 17 pelo ID do botão)
                        ESP_LOGI("NEXTION", "Stop Multi-Step Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // Sinal de STOP
                        parameters_recv = false;
                    }
                } else if (page == 4) {
                    if (component_id == 17 && event == 0 && parameters_recv) { // <-- ID BOTÃO START
                        ESP_LOGI("NEXTION", "Start Trapezoidal Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 3; // Trapezoidal Profile 
                    } else if (component_id == 18 && event == 0) { // <-- ID BOTÃO STOP
                        ESP_LOGI("NEXTION", "Stop Trapezoidal Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // Sinal de STOP
                        parameters_recv = false;
                    }
                } else if (page == 5) {
                    if (component_id == 3 && event == 0 && parameters_recv) { // <-- ID BOTÃO START
                        ESP_LOGI("NEXTION", "Start Real-Time Sine Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 4; // Real-Time Sine Profile 
                     } else if (component_id == 4 && event == 0) { // <-- ID BOTÃO STOP
                        ESP_LOGI("NEXTION", "Stop Real-Time Sine Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // Sinal de STOP
                        parameters_recv = false;
                     }
                }
                else if (page == 6) {
                    if (component_id == 17 && event == 0 && parameters_recv) { // <-- ID BOTÃO START
                        ESP_LOGI("NEXTION", "Start Sweep / Chirp Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 5; // Sweep Profile 
                     } else if (component_id == 18 && event == 0) { // <-- ID BOTÃO STOP
                        ESP_LOGI("NEXTION", "Stop Sweep / Chirp Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // Sinal de STOP
                        parameters_recv = false;
                     }
                }

            }
            // End Touch Event (0x65) -------------------------------------------------------------
            
            // Shaking Table Profiles Data Packet (0x55) -------------------------------------------------------------
            else if (buffer[0] == 0x55 && buffer[1] == 0x01) {
                // Sine Wave Profile
                // Example: 55 01 F4 01 FA 00 32 00 23 00 63 76 FF FF FF
                // 55 - Custom Send Event ; 01 - Type of Data (Ex: Sine Parameters) ; F4 01 -  freq motor axis y (5.00Hz) ; FA 00 - frequency motor axis x (2.50Hz) ; 32 00 - displacement y (0.50mm) ; 23 00 - displacement x (0.35mm) ; 58 02 - time (60s) ; 63 2B - CRC16 ; FF FF FF - Terminator

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
                        parameters_recv=true;
                    }else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }    
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR;
                }
            } else if (buffer[0] == 0x55 && buffer[1] == 0x02) {
                // Multi-step Profile
                // Example: 55 02 1E 00 0A 00 32 00 14 00 14 00 32 00 0A 00 1E 00 64 00 A0 00 00 00 E6 8E FF FF FF 
                // 55 - Custom Send Event ; 02 - Type of Data (Multi-Step Parameters) ; 1E 00 - freq_y1 (3.0Hz) ; 0A 00 - freq_y2 (1.0Hz) ; 32 00 - freq_y3 (5.0Hz) ; 14 00 - freq_y4 (2.0Hz) ; 14 00 - freq_x1 (2.0Hz) ; 32 00 - freq_x2 (5.0Hz) ; 0A 00 - freq_x3 (1.0Hz) ; 1E 00 - freq_x4 (3.0Hz) ; 64 00 - disp_y (10.0mm) ; A0 00 - disp_x (16.0mm) ; 00 00 - time (0.0s) ; E6 8E - CRC16 ; FF FF FF - Terminator

                uint16_t m2_freq_y1 = buffer[2] | (buffer[3] << 8);
                uint16_t m2_freq_y2 = buffer[4] | (buffer[5] << 8);
                uint16_t m2_freq_y3 = buffer[6] | (buffer[7] << 8);
                uint16_t m2_freq_y4 = buffer[8] | (buffer[9] << 8);
                uint16_t m2_freq_x1 = buffer[10] | (buffer[11] << 8);
                uint16_t m2_freq_x2 = buffer[12] | (buffer[13] << 8);
                uint16_t m2_freq_x3 = buffer[14] | (buffer[15] << 8);
                uint16_t m2_freq_x4 = buffer[16] | (buffer[17] << 8);
                uint16_t m2_time_y1 = buffer[18] | (buffer[19] << 8);
                uint16_t m2_time_y2 = buffer[20] | (buffer[21] << 8);
                uint16_t m2_time_y3 = buffer[22] | (buffer[23] << 8);
                uint16_t m2_time_y4 = buffer[24] | (buffer[25] << 8);
                uint16_t m2_time_x1 = buffer[26] | (buffer[27] << 8);
                uint16_t m2_time_x2 = buffer[28] | (buffer[29] << 8);
                uint16_t m2_time_x3 = buffer[30] | (buffer[31] << 8);
                uint16_t m2_time_x4 = buffer[32] | (buffer[33] << 8);
                uint16_t m2_disp_y = buffer[34] | (buffer[35] << 8);
                uint16_t m2_disp_x = buffer[36] | (buffer[37] << 8);
                uint16_t m2_time_s = buffer[38] | (buffer[39] << 8);
                uint16_t recv_crc  = buffer[40] | (buffer[41] << 8); 

                uint8_t payload[38]; 
                for(int j = 0; j < 38; j++) {
                    payload[j] = buffer[2 + j];
                }
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload)); // calcula CRC dos dados recebidos

                // Conversão para floats (escala x10 enviada pelo Nextion)
                float f2_freq_y1 = m2_freq_y1 / 10.0f;
                float f2_freq_y2 = m2_freq_y2 / 10.0f;
                float f2_freq_y3 = m2_freq_y3 / 10.0f;
                float f2_freq_y4 = m2_freq_y4 / 10.0f;
                float f2_freq_x1 = m2_freq_x1 / 10.0f;
                float f2_freq_x2 = m2_freq_x2 / 10.0f;
                float f2_freq_x3 = m2_freq_x3 / 10.0f;
                float f2_freq_x4 = m2_freq_x4 / 10.0f;
                float f2_time_y1 = m2_time_y1 / 10.0f;
                float f2_time_y2 = m2_time_y2 / 10.0f;
                float f2_time_y3 = m2_time_y3 / 10.0f;
                float f2_time_y4 = m2_time_y4 / 10.0f;
                float f2_time_x1 = m2_time_x1 / 10.0f;
                float f2_time_x2 = m2_time_x2 / 10.0f;
                float f2_time_x3 = m2_time_x3 / 10.0f;
                float f2_time_x4 = m2_time_x4 / 10.0f;
                float f2_disp_y = m2_disp_y / 10.0f;
                float f2_disp_x = m2_disp_x / 10.0f;
                float f2_time_s = m2_time_s / 10.0f;
                    
                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - F_y[%.1f, %.1f, %.1f, %.1f]Hz F_x[%.1f, %.1f, %.1f, %.1f]Hz", f2_freq_y1, f2_freq_y2, f2_freq_y3, f2_freq_y4, f2_freq_x1, f2_freq_x2, f2_freq_x3, f2_freq_x4);
                    ESP_LOGI("NEXTION", "           T_y[%.1f, %.1f, %.1f, %.1f]s  T_x[%.1f, %.1f, %.1f, %.1f]s", f2_time_y1, f2_time_y2, f2_time_y3, f2_time_y4, f2_time_x1, f2_time_x2, f2_time_x3, f2_time_x4);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm, TempTotal: %.1fs", f2_disp_y, f2_disp_x, f2_time_s);
                        
                    // Validação acumulada para manter o código legível e estruturado
                    bool params_ok = true;

                    // 1. Valida Deslocamentos e Tempos
                    params_ok &= displacement_parameter_validation(f2_disp_x);
                    params_ok &= displacement_parameter_validation(f2_disp_y);
                    params_ok &= time_parameter_validation(f2_time_s);

                    // 2. Valida Frequências do Motor X
                    if (f2_disp_x > 0.0f) {
                        params_ok &= time_parameter_validation(f2_time_x1) && time_parameter_validation(f2_time_x2) && time_parameter_validation(f2_time_x3) && time_parameter_validation(f2_time_x4);
                        params_ok &= frequency_parameter_validation(f2_freq_x1) && frequency_parameter_validation(f2_freq_x2) &&
                                     frequency_parameter_validation(f2_freq_x3) && frequency_parameter_validation(f2_freq_x4);
                    }
                        
                    // 3. Valida Frequências do Motor Y
                    if (f2_disp_y > 0.0f) {
                        params_ok &= time_parameter_validation(f2_time_y1) && time_parameter_validation(f2_time_y2) && time_parameter_validation(f2_time_y3) && time_parameter_validation(f2_time_y4);
                        params_ok &= frequency_parameter_validation(f2_freq_y1) && frequency_parameter_validation(f2_freq_y2) &&
                                     frequency_parameter_validation(f2_freq_y3) && frequency_parameter_validation(f2_freq_y4);
                    }

                    // 4. Valida coerência Deslocamento vs Frequência (somamos as freqs de cada eixo para o teste)
                    float sum_freq_x = f2_freq_x1 + f2_freq_x2 + f2_freq_x3 + f2_freq_x4;
                    float sum_freq_y = f2_freq_y1 + f2_freq_y2 + f2_freq_y3 + f2_freq_y4;
                    params_ok &= compare_disp_w_freq(f2_disp_x, sum_freq_x);
                    params_ok &= compare_disp_w_freq(f2_disp_y, sum_freq_y);

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Todos os parametros validos. A aguardar inicio do Multi-Step...");
                        sendAckToNextion(160); // ACK DATA OK
                        MonitorTask = true;    // ativa monitorização de stack
                        
                        // 1. Guardar as frequências Multi-Step nos arrays globais
                        nextion_multistep_freq_x[0] = f2_freq_x1; nextion_multistep_freq_x[1] = f2_freq_x2;
                        nextion_multistep_freq_x[2] = f2_freq_x3; nextion_multistep_freq_x[3] = f2_freq_x4;
                        
                        nextion_multistep_freq_y[0] = f2_freq_y1; nextion_multistep_freq_y[1] = f2_freq_y2;
                        nextion_multistep_freq_y[2] = f2_freq_y3; nextion_multistep_freq_y[3] = f2_freq_y4;
                        
                        nextion_multistep_time_x[0] = f2_time_x1; nextion_multistep_time_x[1] = f2_time_x2;
                        nextion_multistep_time_x[2] = f2_time_x3; nextion_multistep_time_x[3] = f2_time_x4;
                        
                        nextion_multistep_time_y[0] = f2_time_y1; nextion_multistep_time_y[1] = f2_time_y2;
                        nextion_multistep_time_y[2] = f2_time_y3; nextion_multistep_time_y[3] = f2_time_y4;

                        // 2. Reutilizar as variáveis globais de deslocamento e tempo
                        nextion_target_disp_x = f2_disp_x;
                        nextion_target_disp_y = f2_disp_y;
                        nextion_target_time_s = f2_time_s;
                        
                        // 3. Ativar a flag para permitir que o botão "Start" funcione
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR;
                }
            } else if (buffer[0] == 0x55 && buffer[1] == 0x03) {
                //  Perfil Trapezoidal
                //  Example: 55 03 05 00 1E 00 0A 00 05 00 19 00 32 00 0A 00 FA 00 14 00 00 00 2C 01 28 00 96 00 E6 00 AD C9 FF FF FF
                //  55 - Custom Send Event ; 03 - Type of Data (Trapezoidal Parameters) ; 05 00 - startFreq_y (0.5Hz) ; 1E 00 - cruiseFreq_y (3.0Hz) ; 0A 00 - endFreq_y (1.0Hz) ; 05 00 - startFreq_x (0.5Hz) ; 19 00 - cruiseFreq_x (2.5Hz) ; 32 00 - endFreq_x (5.0Hz) ; 0A 00 - acelTime_y (1.0s) ; FA 00 - cruiseTime_y (25.0s) ; 14 00 - decelTime_y (2.0s) ; 00 00 - acelTime_x (0.0s) ; 2C 01 - cruiseTime_x (30.0s) ; 28 00 - decelTime_x (4.0s) ; 96 00 - disp_y (15.0mm) ; E6 00 - disp_x (23.0mm) ; AD C9 - CRC16 ; FF FF FF - Terminator

                uint16_t m3_startFreq_y  = buffer[2] | (buffer[3] << 8);
                uint16_t m3_cruiseFreq_y = buffer[4] | (buffer[5] << 8);
                uint16_t m3_endFreq_y    = buffer[6] | (buffer[7] << 8);
                uint16_t m3_startFreq_x  = buffer[8] | (buffer[9] << 8);
                uint16_t m3_cruiseFreq_x = buffer[10] | (buffer[11] << 8);
                uint16_t m3_endFreq_x    = buffer[12] | (buffer[13] << 8);
                uint16_t m3_acelTime_y   = buffer[14] | (buffer[15] << 8);
                uint16_t m3_cruiseTime_y = buffer[16] | (buffer[17] << 8);
                uint16_t m3_decelTime_y  = buffer[18] | (buffer[19] << 8);
                uint16_t m3_acelTime_x   = buffer[20] | (buffer[21] << 8);
                uint16_t m3_cruiseTime_x = buffer[22] | (buffer[23] << 8);
                uint16_t m3_decelTime_x  = buffer[24] | (buffer[25] << 8);
                uint16_t m3_disp_y       = buffer[26] | (buffer[27] << 8);
                uint16_t m3_disp_x       = buffer[28] | (buffer[29] << 8);
                uint16_t recv_crc        = buffer[30] | (buffer[31] << 8); 

                uint8_t payload[28]; 
                for(int j = 0; j < 28; j++) {
                    payload[j] = buffer[2 + j];
                }
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload)); 

                // Conversão para floats (escala x10)
                float f3_startFreq_y  = m3_startFreq_y / 10.0f;
                float f3_cruiseFreq_y = m3_cruiseFreq_y / 10.0f;
                float f3_endFreq_y    = m3_endFreq_y / 10.0f;
                float f3_startFreq_x  = m3_startFreq_x / 10.0f;
                float f3_cruiseFreq_x = m3_cruiseFreq_x / 10.0f;
                float f3_endFreq_x    = m3_endFreq_x / 10.0f;
                float f3_acelTime_y   = m3_acelTime_y / 10.0f;
                float f3_cruiseTime_y = m3_cruiseTime_y / 10.0f;
                float f3_decelTime_y  = m3_decelTime_y / 10.0f;
                float f3_acelTime_x   = m3_acelTime_x / 10.0f;
                float f3_cruiseTime_x = m3_cruiseTime_x / 10.0f;
                float f3_decelTime_x  = m3_decelTime_x / 10.0f;
                float f3_disp_y       = m3_disp_y / 10.0f;
                float f3_disp_x       = m3_disp_x / 10.0f;
                    
                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - Trapezoidal Profile Recebido:");
                    ESP_LOGI("NEXTION", "           F_y[%.1f, %.1f, %.1f]Hz  F_x[%.1f, %.1f, %.1f]Hz", f3_startFreq_y, f3_cruiseFreq_y, f3_endFreq_y, f3_startFreq_x, f3_cruiseFreq_x, f3_endFreq_x);
                    ESP_LOGI("NEXTION", "           T_y[%.1f, %.1f, %.1f]s   T_x[%.1f, %.1f, %.1f]s", f3_acelTime_y, f3_cruiseTime_y, f3_decelTime_y, f3_acelTime_x, f3_cruiseTime_x, f3_decelTime_x);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm", f3_disp_y, f3_disp_x);
                        
                    bool params_ok = true;
                    params_ok &= displacement_parameter_validation(f3_disp_x);
                    params_ok &= displacement_parameter_validation(f3_disp_y);

                    if (f3_disp_x > 0.0f) {
                        float total_time_x = f3_acelTime_x + f3_cruiseTime_x + f3_decelTime_x;
                        params_ok &= time_parameter_validation(total_time_x);
                        params_ok &= frequency_parameter_validation(f3_startFreq_x) && frequency_parameter_validation(f3_cruiseFreq_x) && frequency_parameter_validation(f3_endFreq_x);
                        params_ok &= compare_disp_w_freq(f3_disp_x, f3_cruiseFreq_x);
                    }
                        
                    if (f3_disp_y > 0.0f) {
                        float total_time_y = f3_acelTime_y + f3_cruiseTime_y + f3_decelTime_y;
                        params_ok &= time_parameter_validation(total_time_y);
                        params_ok &= frequency_parameter_validation(f3_startFreq_y) && frequency_parameter_validation(f3_cruiseFreq_y) && frequency_parameter_validation(f3_endFreq_y);
                        params_ok &= compare_disp_w_freq(f3_disp_y, f3_cruiseFreq_y);
                    }

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Parametros Trapezoidais validos. A aguardar start...");
                        sendAckToNextion(160); // ACK DATA OK
                        MonitorTask = true;
                        
                        nextion_trapz_start_freq_x = f3_startFreq_x; nextion_trapz_cruise_freq_x = f3_cruiseFreq_x; nextion_trapz_end_freq_x = f3_endFreq_x;
                        nextion_trapz_accel_time_x = f3_acelTime_x; nextion_trapz_cruise_time_x = f3_cruiseTime_x; nextion_trapz_decel_time_x = f3_decelTime_x;
                        
                        nextion_trapz_start_freq_y = f3_startFreq_y; nextion_trapz_cruise_freq_y = f3_cruiseFreq_y; nextion_trapz_end_freq_y = f3_endFreq_y;
                        nextion_trapz_accel_time_y = f3_acelTime_y; nextion_trapz_cruise_time_y = f3_cruiseTime_y; nextion_trapz_decel_time_y = f3_decelTime_y;

                        nextion_target_disp_x = f3_disp_x;
                        nextion_target_disp_y = f3_disp_y;
                        
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR;
                } 
            } else if (buffer[0] == 0x55 && buffer[1] == 0x04) {
                //  Real-time sine wave profile 
                //  Example: 55 01 05 00 32 00 05 00 32 00 96 00 E6 00 2C 01 72 20 FF FF FF
                //  55 - Custom Send Event ; 04 - Type of Data (Real-Time Sine Parameters) ; 05 00 - minFreq_y (0.5Hz) ; 32 00 - maxFreq_y (5.0Hz) ; 05 00 - minFreq_x (0.5Hz) ; 32 00 - maxFreq_x (5.0Hz) ; 96 00 - disp_y (15.0mm) ; E6 00 - disp_x (23.0mm) ; 2C 01 - duration (30.0s) ; E6 20 - CRC16 ; FF FF FF - Terminator

                uint16_t m4_minFreq_y = buffer[2] | (buffer[3] << 8);
                uint16_t m4_maxFreq_y = buffer[4] | (buffer[5] << 8);
                uint16_t m4_minFreq_x = buffer[6] | (buffer[7] << 8);
                uint16_t m4_maxFreq_x = buffer[8] | (buffer[9] << 8);
                uint16_t m4_disp_y    = buffer[10] | (buffer[11] << 8);
                uint16_t m4_disp_x    = buffer[12] | (buffer[13] << 8);
                uint16_t m4_duration  = buffer[14] | (buffer[15] << 8);
                uint16_t recv_crc     = buffer[16] | (buffer[17] << 8);

                uint8_t payload[14];
                for(int j = 0; j < 14; j++) {
                    payload[j] = buffer[2 + j];
                }
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload));

                // Conversão para floats (escala x10)
                float f4_minFreq_y = m4_minFreq_y / 10.0f;
                float f4_maxFreq_y = m4_maxFreq_y / 10.0f;
                float f4_minFreq_x = m4_minFreq_x / 10.0f;
                float f4_maxFreq_x = m4_maxFreq_x / 10.0f;
                float f4_disp_y    = m4_disp_y / 10.0f;
                float f4_disp_x    = m4_disp_x / 10.0f;
                float f4_duration  = m4_duration / 10.0f;
                    
                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - Real-Time Sine Profile Recebido:");
                    ESP_LOGI("NEXTION", "           F_y[%.1f - %.1f]Hz  F_x[%.1f - %.1f]Hz", f4_minFreq_y, f4_maxFreq_y, f4_minFreq_x, f4_maxFreq_x);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm, Dur: %.1fs", f4_disp_y, f4_disp_x, f4_duration);

                    bool params_ok = true;
                    params_ok &= displacement_parameter_validation(f4_disp_x);
                    params_ok &= displacement_parameter_validation(f4_disp_y);
                    params_ok &= time_parameter_validation(f4_duration);

                    if (f4_disp_x > 0.0f) {
                        params_ok &= frequency_parameter_validation(f4_minFreq_x) && frequency_parameter_validation(f4_maxFreq_x);
                        params_ok &= compare_disp_w_freq(f4_disp_x, f4_maxFreq_x);
                    }

                    if (f4_disp_y > 0.0f) {
                        params_ok &= frequency_parameter_validation(f4_minFreq_y) && frequency_parameter_validation(f4_maxFreq_y);
                        params_ok &= compare_disp_w_freq(f4_disp_y, f4_maxFreq_y);
                    }

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Parametros Real-Time Sine validos. A aguardar start...");
                        sendAckToNextion(160); // ACK DATA OK
                        MonitorTask = true;
                        
                        nextion_rt_min_freq_x = f4_minFreq_x;
                        nextion_rt_max_freq_x = f4_maxFreq_x;
                        nextion_rt_min_freq_y = f4_minFreq_y;
                        nextion_rt_max_freq_y = f4_maxFreq_y;
                        
                        nextion_target_disp_x = f4_disp_x;
                        nextion_target_disp_y = f4_disp_y;
                        nextion_target_time_s = f4_duration;
                        
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR
                }
            } else if (buffer[0] == 0x55 && buffer[1] == 0x05) {
                //  Sweep / Chirp profile 
                //  55 05 01 00 32 00 01 00 32 00 96 00 E6 00 00 00 00 00 2C 01 B9 2B FF FF FF 
                
                uint16_t m5_minFreq_y = buffer[2] | (buffer[3] << 8);
                uint16_t m5_maxFreq_y = buffer[4] | (buffer[5] << 8);
                uint16_t m5_minFreq_x = buffer[6] | (buffer[7] << 8);
                uint16_t m5_maxFreq_x = buffer[8] | (buffer[9] << 8);
                uint16_t m5_disp_y    = buffer[10] | (buffer[11] << 8);
                uint16_t m5_disp_x    = buffer[12] | (buffer[13] << 8);
                uint16_t m5_isBid_y   = buffer[14] | (buffer[15] << 8);
                uint16_t m5_isBid_x   = buffer[16] | (buffer[17] << 8);
                uint16_t m5_duration  = buffer[18] | (buffer[19] << 8);
                uint16_t recv_crc     = buffer[20] | (buffer[21] << 8);

                uint8_t payload[18];
                for(int j = 0; j < 18; j++) {
                    payload[j] = buffer[2 + j];
                }
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload));

                // Conversão para floats (escala x10)
                float f5_minFreq_y = m5_minFreq_y / 10.0f;
                float f5_maxFreq_y = m5_maxFreq_y / 10.0f;
                float f5_minFreq_x = m5_minFreq_x / 10.0f;
                float f5_maxFreq_x = m5_maxFreq_x / 10.0f;
                float f5_disp_y    = m5_disp_y / 10.0f;
                float f5_disp_x    = m5_disp_x / 10.0f;
                bool  b5_isBid_y   = (m5_isBid_y != 0);
                bool  b5_isBid_x   = (m5_isBid_x != 0);
                float f5_duration  = m5_duration / 10.0f;

                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - Sweep/Chirp Profile Recebido:");
                    ESP_LOGI("NEXTION", "           F_y[%.1f - %.1f]Hz  F_x[%.1f - %.1f]Hz", f5_minFreq_y, f5_maxFreq_y, f5_minFreq_x, f5_maxFreq_x);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm, Dur: %.1fs", f5_disp_y, f5_disp_x, f5_duration);
                    ESP_LOGI("NEXTION", "           Bidirecional_y: %s, Bidirecional_x: %s", b5_isBid_y ? "SIM" : "NAO", b5_isBid_x ? "SIM" : "NAO");

                    bool params_ok = true;
                    params_ok &= displacement_parameter_validation(f5_disp_x);
                    params_ok &= displacement_parameter_validation(f5_disp_y);
                    params_ok &= time_parameter_validation(f5_duration);

                    if (f5_disp_x > 0.0f) {
                        params_ok &= frequency_parameter_validation(f5_minFreq_x) && frequency_parameter_validation(f5_maxFreq_x);
                        params_ok &= compare_disp_w_freq(f5_disp_x, f5_maxFreq_x);
                        if (f5_minFreq_x > f5_maxFreq_x) {
                            ESP_LOGW("NEXTION", "Sweep: Freq Min X (%.1f) maior que Freq Max X (%.1f)!", f5_minFreq_x, f5_maxFreq_x);
                            params_ok = false;
                        }
                    }

                    if (f5_disp_y > 0.0f) {
                        params_ok &= frequency_parameter_validation(f5_minFreq_y) && frequency_parameter_validation(f5_maxFreq_y);
                        params_ok &= compare_disp_w_freq(f5_disp_y, f5_maxFreq_y);
                        if (f5_minFreq_y > f5_maxFreq_y) {
                            ESP_LOGW("NEXTION", "Sweep: Freq Min Y (%.1f) maior que Freq Max Y (%.1f)!", f5_minFreq_y, f5_maxFreq_y);
                            params_ok = false;
                        }
                    }

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Parametros Sweep/Chirp validos. A aguardar start...");
                        sendAckToNextion(160); // ACK DATA OK
                        MonitorTask = true;
                        
                        nextion_sweep_min_freq_x = f5_minFreq_x; nextion_sweep_max_freq_x = f5_maxFreq_x;
                        nextion_sweep_min_freq_y = f5_minFreq_y; nextion_sweep_max_freq_y = f5_maxFreq_y;
                        nextion_sweep_isBid_x = b5_isBid_x; nextion_sweep_isBid_y = b5_isBid_y;
                        nextion_target_disp_x = f5_disp_x; nextion_target_disp_y = f5_disp_y;
                        nextion_target_time_s = f5_duration;
                        
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ CRC inválido (esperado 0x%04X, recebido 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR
                }
            }
            // End Shaking Table Profiles Data Packet (0x55) -------------------------------------------------------------
            else {
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