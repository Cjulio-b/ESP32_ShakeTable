#include "driver/uart.h"
#include "esp_log.h"
#include "functions.h"
#include <string.h>

#define UART_PORT UART_NUM_1
#define NEXTION_TERMINATOR 0xFF
#define NEXTION_MAX_LEN 64

// Global variables to store validated parameters from Nextion
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

static volatile uint8_t nextion_current_page = 0;

/* void nextion_set_current_page(uint8_t page)
{
    nextion_current_page = page;
    ESP_LOGI("NEXTION", "Current page set to %u", page);
} */

// CRC-16 Modbus (polynomial 0xA001, initial 0xFFFF)
uint16_t nextion_crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001; // LSB is 1, XOR with polynomial
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
    // objname: object name (e.g., t0, n0, x0)
    // datatype: data type (e.g., txt, val)
    // value: value to send (e.g., \"Hello\"", "123")
    // example output: t0.txt=\"Hello\"", n0.val=123
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
            nextion_send_command("va0.val=161"); // ACK ERROR - CRC/DATA INVALID (0xA1)
            ESP_LOGW("NEXTION", "↩️ Retransmission request sent (Invalid Data)");
            break;
        case 162:
            nextion_send_command("va0.val=162"); // ACK HOMING (0xA2)
            break;
        case 163:
            nextion_send_command("va0.val=163"); // ACK START MOTION (0xA3)
            break;
        case 164:
            nextion_send_command("va0.val=164"); // ACK MOTION ENDED (0xA4)
            break;
        case 165:
            nextion_send_command("va0.val=165"); // ACK MOTION ERROR (0xA5)
            ESP_LOGW("NEXTION", "↩️ Error during table motion");
            break;
        case 166:
            nextion_send_command("va0.val=166"); // ACK INVALID PARAMS (0xA6)
            ESP_LOGW("NEXTION", "↩️ Invalid parameters received from HMI");
            break;
        case 167:
            nextion_send_command("va0.val=167"); // ACK FILE NOT FOUND (0xA7)
            ESP_LOGW("NEXTION", "↩️ File for playback not found in memory");
            break;
        case 168:
            nextion_send_command("va0.val=168"); // ACK MOTION STOP (0xA8)
            ESP_LOGW("NEXTION", "Stop motion requested by user.");
            break;
        // Menu config ACKs for HMI Nextion
        case 201:
            nextion_send_command("va0.val=201"); // ACK FILE OPEN ERROR (0xC9)
            ESP_LOGW("NEXTION", "↩️ Current Shake Table configuration sent.");
            break;
        case 202:
            nextion_send_command("va0.val=202"); // ACK FILE OPEN ERROR (0xCA)
            ESP_LOGW("NEXTION", "↩️ New configuration received and applied to the Shake Table.");
            break;
        case 203:
            nextion_send_command("va0.val=203"); // ACK FILE OPEN ERROR (0xCB)
            ESP_LOGW("NEXTION", "↩️ Error while saving new configuration to flash memory (NVS).");
            break;
        case 204:
            nextion_send_command("va0.val=204"); // ACK FILE OPEN ERROR (0xCC)
            ESP_LOGW("NEXTION", "↩️ Factory default configuration restored and applied to the Shake Table.");
            break;
        // Connectivity ACKs for HMI Nextion
        case 301:
            nextion_send_command("va0.val=301"); // ACK WIFI CONNECTED (0x12D)
            ESP_LOGI("NEXTION", "↩️ ESP32 connected to WiFi network.");
            break;
        case 302:
            nextion_send_command("va0.val=302"); // ACK WIFI DISCONNECTED (0x12E)
            ESP_LOGW("NEXTION", "↩️ ESP32 disconnected from WiFi network.");
            break;
        case 310:
            nextion_send_command("va0.val=310"); // ACK HTTP SERVER STARTED (0x12F)
            ESP_LOGI("NEXTION", "↩️ File sismic profile uploaded to ESP32 via HTTP Server.");
            break;
        case 311:
            nextion_send_command("va0.val=311"); // ACK HTTP SERVER ERROR (0x130)
            ESP_LOGW("NEXTION", "↩️ Error during file upload via HTTP Server.");
            break;
        case 312:
            nextion_send_command("va0.val=312"); // ACK FILE DELETED (0x131)
            ESP_LOGI("NEXTION", "↩️ CSV file ready to be downloaded in HTTP server.");
            break;
        default:
            ESP_LOGW("NEXTION", "Unknown ACK: %d", ackmsg);
    }
}

/* void nextion_notify_wifi_connected(void)
{
    bool is_valid_page = (nextion_current_page == 2 || nextion_current_page == 3 || nextion_current_page == 4 || nextion_current_page == 5 || nextion_current_page == 6 || nextion_current_page == 7);
    if (is_valid_page) {
        sendAckToNextion(301);
    } else {
        ESP_LOGI("NEXTION", "WiFi connected, but current page %u is not 7 or 9. ACK 301 skipped.", nextion_current_page);
    }
} */

/* void nextion_notify_wifi_disconnected(void)
{
    bool is_valid_page = (nextion_current_page == 2 || nextion_current_page == 3 || nextion_current_page == 4 || nextion_current_page == 5 || nextion_current_page == 6 || nextion_current_page == 7);
    if (is_valid_page) {
        sendAckToNextion(302);
    } else {
        ESP_LOGI("NEXTION", "WiFi disconnected, but current page %u is not 7 or 9. ACK 302 skipped.", nextion_current_page);
    }
}
 */

/* void nextion_notify_upload_success(void)
{
    bool is_valid_page = (nextion_current_page == 7);
    if (is_valid_page) {
        sendAckToNextion(310);
    } else {
        ESP_LOGI("NEXTION", "Upload success, but current page %u is not 7. ACK 310 skipped.", nextion_current_page);
    }
} */

/* void nextion_notify_upload_error(void)
{
    bool is_valid_page = (nextion_current_page == 7);
    if (is_valid_page) {
        sendAckToNextion(311);
    } else {
        ESP_LOGI("NEXTION", "Upload error, but current page %u is not 7. ACK 311 skipped.", nextion_current_page);
    }
} */

/* void nextion_notify_test_result_available(void)
{
    bool is_valid_page = (nextion_current_page == 7 || nextion_current_page == 9);
    if (is_valid_page) {
        sendAckToNextion(312);
    } else {
        ESP_LOGI("NEXTION", "Test result available, but current page %u is not 7 or 9. ACK 312 skipped.", nextion_current_page);
    }
} */

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

bool displacement_parameter_validation(float disp_mm, char axis)
{
    // Use the P2P safety limit defined in the corresponding axis configuration
    float max_p2p = (axis == 'x') ? table_config_x.axis.peak_to_peak_disp_mm : table_config_y.axis.peak_to_peak_disp_mm;

    if (disp_mm < 0.0f || disp_mm > max_p2p) {
        ESP_LOGW("NEXTION", "Displacement parameter out of range");
        return false;
    } else {
        return true;
    }
}

bool frequency_parameter_validation(float freq_hz, char axis)
{
    float max_freq = (axis == 'x') ? table_config_x.axis.max_freq_hz : table_config_y.axis.max_freq_hz;

    if (freq_hz < 0.0f || freq_hz > max_freq) {
        ESP_LOGW("NEXTION", "Frequency parameter out of range");
        return false;
    } else {
        return true;
    }
}

bool time_parameter_validation(float time_s)
{
    if (time_s <= 0.0f || time_s > 1800.0f) { // Maximum limit: 30 minutes (1800s)
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
                
                if (page==1){
                    if(component_id==4){ // icon to enter Sinewave profile
                        nextion_current_page=2;
                    }
                    else if(component_id==3){ // icon to enter multi-step profile
                        nextion_current_page=3;
                    }
                    else if(component_id==6){ // icon to enter trapezoidal profile
                        nextion_current_page=4;
                    }
                    else if(component_id==7){ // icon to enter real-time sine (analog potentiometer) profile
                        nextion_current_page=5;
                    }
                    else if(component_id==5){ // icon to enter sweep/chirp profile
                        nextion_current_page=6;
                    }
                    else if(component_id==2){ // icon to enter 'from file' profile
                        nextion_current_page=7;
                    }
                }
                //nextion_set_current_page(page);

                ESP_LOGI("NEXTION", "Touch Event: page=%d, comp=%d, event=%d",
                         page, component_id, event);
                if (page==2){
                    if (component_id == 12 && event == 0 && parameters_recv) { // START button Sinewave Profile
                        ESP_LOGI("NEXTION", "Start SineWave Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING (The table will prepare first)
                        nextion_profile = 1; // Sinewave Profile 
                    } else if (component_id == 13 && event == 0) { // STOP button Sinewave Profile
                        ESP_LOGI("NEXTION", "Stop SineWave Profile Selected!");
                        sendAckToNextion(168); // ACK MOTION END
                        nextion_profile = 0; // STOP signal (aborts motor loop)
                        parameters_recv = false;
                    }
                } else if (page == 3) { // MULTI-STEP PAGE ID
                    if (component_id == 27 && event == 0 && parameters_recv) { // START button Multi-Step Profile (Replace 16 with the button ID)
                        ESP_LOGI("NEXTION", "Start Multi-Step Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 2; // Multi-Step Profile 
                    } else if (component_id == 28 && event == 0) { // STOP button Multi-Step Profile (Replace 17 with the button ID)
                        ESP_LOGI("NEXTION", "Stop Multi-Step Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // STOP signal
                        parameters_recv = false;
                    }
                } else if (page == 4) {
                    if (component_id == 15 && event == 0 && parameters_recv) { // <-- ID BOTÃO START
                        ESP_LOGI("NEXTION", "Start Trapezoidal Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 3; // Trapezoidal Profile 
                    } else if (component_id == 16 && event == 0) { // <-- ID BOTÃO STOP
                        ESP_LOGI("NEXTION", "Stop Trapezoidal Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // STOP signal
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
                        nextion_profile = 0; // STOP signal
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
                        nextion_profile = 0; // STOP signal
                        parameters_recv = false;
                     }
                }
                else if (page == 7) {
                    if (component_id == 8 && event == 0 && parameters_recv) { // <-- ID BOTÃO START
                        ESP_LOGI("NEXTION", "Start File Profile Selected!");
                        sendAckToNextion(162); // ACK HOMING
                        nextion_profile = 6; // File Profile 
                     } else if (component_id == 9 && event == 0) { // <-- ID BOTÃO STOP
                        ESP_LOGI("NEXTION", "Stop File Profile Selected!");
                        sendAckToNextion(164); // ACK MOTION END
                        nextion_profile = 0; // STOP signal
                        parameters_recv = false;
                     }
                }
                else if (page == 8) {
                    if (component_id == 28 && event == 0) { // <-- DEFAULT configuration BUTTON ID (0x1C = 28)
                        ESP_LOGI("NEXTION", "Default Settings Button Pressed! Restoring defaults to NVS...");
                        
                        // Reset global structs to default values
                        kinematics_init_axis(&table_config_x.axis, 27.33f, 14.0f, 100.0f, 1.5f);
                        kinematics_init_stepper(&table_config_x.stepper, 1.8f, 5.18f, 32);
                        
                        kinematics_init_axis(&table_config_y.axis, 26.84f, 14.0f, 95.0f, 1.5f);
                        kinematics_init_stepper(&table_config_y.stepper, 1.8f, 5.18f, 32);
                        
                        // Save directly to NVS
                        config_manager_save('x', &table_config_x);
                        config_manager_save('y', &table_config_y);
                        
                        // Restore hardware pins back to the default
                        set_all_steppers_microsteps(32,32);
                        
                        ESP_LOGI("NEXTION", "Default configuration saved to flash memory (NVS)!");
                        
                       sendAckToNextion(204); // ACK FACTORY DEFAULTS RESTORED
                    }
                    if (component_id == 31 && event == 1) { // <-- Request Configuration BUTTON ID (0x1F = 31)
                        ESP_LOGI("NEXTION", "Request for current configuration received.");
                        
                        char value_str[16];

                        #define SEND_PARAM(obj, val, scale) \
                            snprintf(value_str, sizeof(value_str), "%d", (int)(val * scale)); \
                            nextion_cmd_syntax(obj, "val", value_str); \
                            vTaskDelay(pdMS_TO_TICKS(15)); // Delay to prevent Nextion buffer overflow

                        SEND_PARAM("x0", table_config_x.axis.peak_to_peak_disp_mm, 10.0f);
                        SEND_PARAM("x1", table_config_y.axis.peak_to_peak_disp_mm, 10.0f);
                        SEND_PARAM("x2", table_config_x.axis.crank_radius_mm, 10.0f);
                        SEND_PARAM("x3", table_config_y.axis.crank_radius_mm, 10.0f);
                        SEND_PARAM("x4", table_config_x.axis.rod_length_mm, 10.0f);
                        SEND_PARAM("x5", table_config_y.axis.rod_length_mm, 10.0f);
                        SEND_PARAM("x6", table_config_x.stepper.step_angle_deg, 10.0f);
                        SEND_PARAM("x7", table_config_y.stepper.step_angle_deg, 10.0f);
                        
                        // Sends the Combobox Index (0, 1, 2 or 3) instead of absolute value * 10
                        int cb0_idx = 3, cb1_idx = 3; // Default para 32 microsteps (index 3)
                        if (table_config_x.stepper.microsteps == 4) cb0_idx = 0;
                        else if (table_config_x.stepper.microsteps == 8) cb0_idx = 1;
                        else if (table_config_x.stepper.microsteps == 16) cb0_idx = 2;
                        else if (table_config_x.stepper.microsteps == 32) cb0_idx = 3;
                        
                        if (table_config_y.stepper.microsteps == 4) cb1_idx = 0;
                        else if (table_config_y.stepper.microsteps == 8) cb1_idx = 1;
                        else if (table_config_y.stepper.microsteps == 16) cb1_idx = 2;
                        else if (table_config_y.stepper.microsteps == 32) cb1_idx = 3;

                        SEND_PARAM("cb0", cb0_idx, 1.0f); // scale 1.0f because we want the exact number
                        SEND_PARAM("cb1", cb1_idx, 1.0f);
                        
                        int cb2_idx = (table_config_x.axis.max_freq_hz > 2.0f) ? 1 : 0;
                        int cb3_idx = (table_config_y.axis.max_freq_hz > 2.0f) ? 1 : 0;
                        SEND_PARAM("cb2", cb2_idx, 1.0f);
                        SEND_PARAM("cb3", cb3_idx, 1.0f);
                        
                        SEND_PARAM("x10", table_config_x.stepper.gear_ratio, 100.0f);
                        SEND_PARAM("x11", table_config_y.stepper.gear_ratio, 100.0f);
                        
                        nextion_send_command("va0.val=201");
                        ESP_LOGI("NEXTION", "Current configuration sent to Nextion HMI.");
                        //ESP_LOGI("NEXTION", "MICROSTEPS X: %d, MICROSTEPS Y: %d", table_config_x.stepper.microsteps, table_config_y.stepper.microsteps);
                       
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

                uint8_t payload[] = {buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]}; // data without header, type and CRC
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload)); // calculate CRC of received data

                // Convert to floats (scale x10 sent by Nextion)
                float f_freq_y = freq_y / 10.0f;
                float f_freq_x = freq_x / 10.0f;
                float f_disp_y = disp_y / 10.0f;
                float f_disp_x = disp_x / 10.0f;
                float f_time_s = time_s / 10.0f;    

                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - F_y=%.1fHz, F_x=%.1fHz, D_y=%.1fmm, D_x=%.1fmm, Tmp=%.1fs",
                            f_freq_y, f_freq_x, f_disp_y, f_disp_x, f_time_s);
                    if(displacement_parameter_validation(f_disp_x, 'x') && displacement_parameter_validation(f_disp_y, 'y') &&
                       frequency_parameter_validation(f_freq_x, 'x') && frequency_parameter_validation(f_freq_y, 'y') &&
                       time_parameter_validation(f_time_s) && compare_disp_w_freq(f_disp_x, f_freq_x) && compare_disp_w_freq(f_disp_y, f_freq_y)) {
                        ESP_LOGI("NEXTION", "All parameters are valid. Waiting to start motion profile...");
                        
                        nextion_target_freq_x = f_freq_x;
                        nextion_target_freq_y = f_freq_y;
                        nextion_target_disp_x = f_disp_x;
                        nextion_target_disp_y = f_disp_y;
                        nextion_target_time_s = f_time_s;
                                                
                        sendAckToNextion(160); // ACK: DATA OK
                        MonitorTask = true; // enable stack monitoring
                        parameters_recv=true;
                    }else {
                        sendAckToNextion(166); // ACK: INVALID PARAMS
                    }    
                } else {
                    ESP_LOGW("NEXTION", "❌ Invalid CRC (expected 0x%04X, received 0x%04X)", calc_crc, recv_crc);
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
                uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload)); // calculate CRC of received data

                // Convert to floats (scale x10 sent by Nextion)
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
                        
                    // Accumulated validation to keep the code readable and structured
                    bool params_ok = true;

                    // 1. Validate Displacements and Times
                    params_ok &= displacement_parameter_validation(f2_disp_x, 'x');
                    params_ok &= displacement_parameter_validation(f2_disp_y, 'y');
                    params_ok &= time_parameter_validation(f2_time_s);

                    // 2. Validate Motor X Frequencies
                    if (f2_disp_x > 0.0f) {
                        params_ok &= time_parameter_validation(f2_time_x1) && time_parameter_validation(f2_time_x2) && time_parameter_validation(f2_time_x3) && time_parameter_validation(f2_time_x4);
                        params_ok &= frequency_parameter_validation(f2_freq_x1, 'x') && frequency_parameter_validation(f2_freq_x2, 'x') &&
                                     frequency_parameter_validation(f2_freq_x3, 'x') && frequency_parameter_validation(f2_freq_x4, 'x');
                    }
                        
                    // 3. Validate Motor Y Frequencies
                    if (f2_disp_y > 0.0f) {
                        params_ok &= time_parameter_validation(f2_time_y1) && time_parameter_validation(f2_time_y2) && time_parameter_validation(f2_time_y3) && time_parameter_validation(f2_time_y4);
                        params_ok &= frequency_parameter_validation(f2_freq_y1, 'y') && frequency_parameter_validation(f2_freq_y2, 'y') &&
                                     frequency_parameter_validation(f2_freq_y3, 'y') && frequency_parameter_validation(f2_freq_y4, 'y');
                    }

                    // 4. Validate Displacement vs. Frequency coherence (sum frequencies for each axis for the test)
                    float sum_freq_x = f2_freq_x1 + f2_freq_x2 + f2_freq_x3 + f2_freq_x4;
                    float sum_freq_y = f2_freq_y1 + f2_freq_y2 + f2_freq_y3 + f2_freq_y4;
                    params_ok &= compare_disp_w_freq(f2_disp_x, sum_freq_x);
                    params_ok &= compare_disp_w_freq(f2_disp_y, sum_freq_y);

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "All parameters valid. Waiting to start Multi-Step...");
                        sendAckToNextion(160); // ACK: DATA OK
                        MonitorTask = true;    // enable stack monitoring
                        
                        // 1. Store Multi-Step frequencies in global arrays
                        nextion_multistep_freq_x[0] = f2_freq_x1; nextion_multistep_freq_x[1] = f2_freq_x2;
                        nextion_multistep_freq_x[2] = f2_freq_x3; nextion_multistep_freq_x[3] = f2_freq_x4;
                        
                        nextion_multistep_freq_y[0] = f2_freq_y1; nextion_multistep_freq_y[1] = f2_freq_y2;
                        nextion_multistep_freq_y[2] = f2_freq_y3; nextion_multistep_freq_y[3] = f2_freq_y4;
                        
                        nextion_multistep_time_x[0] = f2_time_x1; nextion_multistep_time_x[1] = f2_time_x2;
                        nextion_multistep_time_x[2] = f2_time_x3; nextion_multistep_time_x[3] = f2_time_x4;
                        
                        nextion_multistep_time_y[0] = f2_time_y1; nextion_multistep_time_y[1] = f2_time_y2;
                        nextion_multistep_time_y[2] = f2_time_y3; nextion_multistep_time_y[3] = f2_time_y4;

                        // 2. Reuse global displacement and time variables
                        nextion_target_disp_x = f2_disp_x;
                        nextion_target_disp_y = f2_disp_y;
                        nextion_target_time_s = f2_time_s;
                        
                        // 3. Set the flag to allow the "Start" button to work
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ Invalid CRC (expected 0x%04X, received 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR;
                }
            } else if (buffer[0] == 0x55 && buffer[1] == 0x03) {
                //  Trapezoidal Profile
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

                // Convert to floats (scale x10)
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
                    ESP_LOGI("NEXTION", "✅ CRC OK - Trapezoidal Profile Received:");
                    ESP_LOGI("NEXTION", "           F_y[%.1f, %.1f, %.1f]Hz  F_x[%.1f, %.1f, %.1f]Hz", f3_startFreq_y, f3_cruiseFreq_y, f3_endFreq_y, f3_startFreq_x, f3_cruiseFreq_x, f3_endFreq_x);
                    ESP_LOGI("NEXTION", "           T_y[%.1f, %.1f, %.1f]s   T_x[%.1f, %.1f, %.1f]s", f3_acelTime_y, f3_cruiseTime_y, f3_decelTime_y, f3_acelTime_x, f3_cruiseTime_x, f3_decelTime_x);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm", f3_disp_y, f3_disp_x);
                        
                    bool params_ok = true;
                    params_ok &= displacement_parameter_validation(f3_disp_x, 'x');
                    params_ok &= displacement_parameter_validation(f3_disp_y, 'y');

                    if (f3_disp_x > 0.0f) {
                        float total_time_x = f3_acelTime_x + f3_cruiseTime_x + f3_decelTime_x;
                        params_ok &= time_parameter_validation(total_time_x);
                        params_ok &= frequency_parameter_validation(f3_startFreq_x, 'x') && frequency_parameter_validation(f3_cruiseFreq_x, 'x') && frequency_parameter_validation(f3_endFreq_x, 'x');
                        params_ok &= compare_disp_w_freq(f3_disp_x, f3_cruiseFreq_x);
                    }
                        
                    if (f3_disp_y > 0.0f) {
                        float total_time_y = f3_acelTime_y + f3_cruiseTime_y + f3_decelTime_y;
                        params_ok &= time_parameter_validation(total_time_y);
                        params_ok &= frequency_parameter_validation(f3_startFreq_y, 'y') && frequency_parameter_validation(f3_cruiseFreq_y, 'y') && frequency_parameter_validation(f3_endFreq_y, 'y');
                        params_ok &= compare_disp_w_freq(f3_disp_y, f3_cruiseFreq_y);
                    }

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Trapezoidal parameters are valid. Waiting for start...");
                        sendAckToNextion(160); // ACK: DATA OK
                        MonitorTask = true;
                        
                        nextion_trapz_start_freq_x = f3_startFreq_x; nextion_trapz_cruise_freq_x = f3_cruiseFreq_x; nextion_trapz_end_freq_x = f3_endFreq_x;
                        nextion_trapz_accel_time_x = f3_acelTime_x; nextion_trapz_cruise_time_x = f3_cruiseTime_x; nextion_trapz_decel_time_x = f3_decelTime_x;
                        
                        nextion_trapz_start_freq_y = f3_startFreq_y; nextion_trapz_cruise_freq_y = f3_cruiseFreq_y; nextion_trapz_end_freq_y = f3_endFreq_y;
                        nextion_trapz_accel_time_y = f3_acelTime_y; nextion_trapz_cruise_time_y = f3_cruiseTime_y; nextion_trapz_decel_time_y = f3_decelTime_y;

                        nextion_target_disp_x = f3_disp_x;
                        nextion_target_disp_y = f3_disp_y;
                        
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK: INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ Invalid CRC (expected 0x%04X, received 0x%04X)", calc_crc, recv_crc);
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

                // Convert to floats (scale x10)
                float f4_minFreq_y = m4_minFreq_y / 10.0f;
                float f4_maxFreq_y = m4_maxFreq_y / 10.0f;
                float f4_minFreq_x = m4_minFreq_x / 10.0f;
                float f4_maxFreq_x = m4_maxFreq_x / 10.0f;
                float f4_disp_y    = m4_disp_y / 10.0f;
                float f4_disp_x    = m4_disp_x / 10.0f;
                float f4_duration  = m4_duration / 10.0f;
                    
                if (recv_crc == calc_crc) {
                    ESP_LOGI("NEXTION", "✅ CRC OK - Real-Time Sine Profile Received:");
                    ESP_LOGI("NEXTION", "           F_y[%.1f - %.1f]Hz  F_x[%.1f - %.1f]Hz", f4_minFreq_y, f4_maxFreq_y, f4_minFreq_x, f4_maxFreq_x);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm, Dur: %.1fs", f4_disp_y, f4_disp_x, f4_duration);

                    bool params_ok = true;
                    params_ok &= displacement_parameter_validation(f4_disp_x, 'x');
                    params_ok &= displacement_parameter_validation(f4_disp_y, 'y');
                    params_ok &= time_parameter_validation(f4_duration);

                    if (f4_disp_x > 0.0f) {
                        params_ok &= frequency_parameter_validation(f4_minFreq_x, 'x') && frequency_parameter_validation(f4_maxFreq_x, 'x');
                        params_ok &= compare_disp_w_freq(f4_disp_x, f4_maxFreq_x);
                    }

                    if (f4_disp_y > 0.0f) {
                        params_ok &= frequency_parameter_validation(f4_minFreq_y, 'y') && frequency_parameter_validation(f4_maxFreq_y, 'y');
                        params_ok &= compare_disp_w_freq(f4_disp_y, f4_maxFreq_y);
                    }

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Real-Time Sine parameters are valid. Waiting for start...");
                        sendAckToNextion(160); // ACK: DATA OK
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
                        sendAckToNextion(166); // ACK: INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ Invalid CRC (expected 0x%04X, received 0x%04X)", calc_crc, recv_crc);
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

                // Convert to floats (scale x10)
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
                    ESP_LOGI("NEXTION", "✅ CRC OK - Sweep/Chirp Profile Received:");
                    ESP_LOGI("NEXTION", "           F_y[%.1f - %.1f]Hz  F_x[%.1f - %.1f]Hz", f5_minFreq_y, f5_maxFreq_y, f5_minFreq_x, f5_maxFreq_x);
                    ESP_LOGI("NEXTION", "           D_y: %.1fmm, D_x: %.1fmm, Dur: %.1fs", f5_disp_y, f5_disp_x, f5_duration);
                    ESP_LOGI("NEXTION", "           Bidirecional_y: %s, Bidirecional_x: %s", b5_isBid_y ? "SIM" : "NAO", b5_isBid_x ? "SIM" : "NAO");

                    bool params_ok = true;
                    params_ok &= displacement_parameter_validation(f5_disp_x, 'x');
                    params_ok &= displacement_parameter_validation(f5_disp_y, 'y');
                    params_ok &= time_parameter_validation(f5_duration);

                    if (f5_disp_x > 0.0f) {
                        params_ok &= frequency_parameter_validation(f5_minFreq_x, 'x') && frequency_parameter_validation(f5_maxFreq_x, 'x');
                        params_ok &= compare_disp_w_freq(f5_disp_x, f5_maxFreq_x);
                        if (f5_minFreq_x > f5_maxFreq_x) {
                            ESP_LOGW("NEXTION", "Sweep: Min Freq X (%.1f) greater than Max Freq X (%.1f)!", f5_minFreq_x, f5_maxFreq_x);
                            params_ok = false;
                        }
                    }

                    if (f5_disp_y > 0.0f) {
                        params_ok &= frequency_parameter_validation(f5_minFreq_y, 'y') && frequency_parameter_validation(f5_maxFreq_y, 'y');
                        params_ok &= compare_disp_w_freq(f5_disp_y, f5_maxFreq_y);
                        if (f5_minFreq_y > f5_maxFreq_y) {
                            ESP_LOGW("NEXTION", "Sweep: Min Freq Y (%.1f) greater than Max Freq Y (%.1f)!", f5_minFreq_y, f5_maxFreq_y);
                            params_ok = false;
                        }
                    }

                    if (params_ok) {
                        ESP_LOGI("NEXTION", "Sweep/Chirp parameters are valid. Waiting for start...");
                        sendAckToNextion(160); // ACK: DATA OK
                        MonitorTask = true;
                        
                        nextion_sweep_min_freq_x = f5_minFreq_x; nextion_sweep_max_freq_x = f5_maxFreq_x;
                        nextion_sweep_min_freq_y = f5_minFreq_y; nextion_sweep_max_freq_y = f5_maxFreq_y;
                        nextion_sweep_isBid_x = b5_isBid_x; nextion_sweep_isBid_y = b5_isBid_y;
                        nextion_target_disp_x = f5_disp_x; nextion_target_disp_y = f5_disp_y;
                        nextion_target_time_s = f5_duration;
                        
                        parameters_recv = true;
                    } else {
                        sendAckToNextion(166); // ACK: INVALID PARAMS
                    }
                } else {
                    ESP_LOGW("NEXTION", "❌ Invalid CRC (expected 0x%04X, received 0x%04X)", calc_crc, recv_crc);
                    sendAckToNextion(161); // ACK ERROR
                }
            }
            else if (buffer[0] == 0x55 && buffer[1] == 0x06) {
                // File Profile
                ESP_LOGI("NEXTION", "File Profile Selected (0x55 0x06)");
                
                char filepath[256];
                if (get_stored_sismo_file(filepath, sizeof(filepath))) {
                    ESP_LOGI("NEXTION", "✅ Seismic file found: %s. Waiting for start...", filepath);
                    sendAckToNextion(160); // ACK: DATA OK
                    parameters_recv = true; // Release lock to allow START button press
                } else {
                    ESP_LOGW("NEXTION", "❌ No seismic file in memory.");
                    sendAckToNextion(167); // ACK: FILE NOT FOUND
                    parameters_recv = false; // Keep lock
                }
            }
            // Machine Configuration (0x55 0x00)
            else if (buffer[0] == 0x55 && buffer[1] == 0x00) {
                // Exemplo: 55 00 0E 01 0E 01 8C 00 8C 00 E8 03 B6 03 12 00 12 00 40 01 40 01 06 02 06 02 22 8F FF FF FF
                if (index >= 35) { 
                    uint16_t p2p_x = buffer[2] | (buffer[3] << 8);
                    uint16_t p2p_y = buffer[4] | (buffer[5] << 8);
                    uint16_t crank_x = buffer[6] | (buffer[7] << 8);
                    uint16_t crank_y = buffer[8] | (buffer[9] << 8);
                    uint16_t rod_x = buffer[10] | (buffer[11] << 8);
                    uint16_t rod_y = buffer[12] | (buffer[13] << 8);
                    uint16_t step_x = buffer[14] | (buffer[15] << 8);
                    uint16_t step_y = buffer[16] | (buffer[17] << 8);
                    uint16_t micro_x = buffer[18] | (buffer[19] << 8);
                    uint16_t micro_y = buffer[20] | (buffer[21] << 8);
                    uint16_t gear_x = buffer[22] | (buffer[23] << 8);
                    uint16_t gear_y = buffer[24] | (buffer[25] << 8);
                    uint16_t max_freq_x_val = buffer[26] | (buffer[27] << 8);
                    uint16_t max_freq_y_val = buffer[28] | (buffer[29] << 8);
                    uint16_t recv_crc = buffer[30] | (buffer[31] << 8);

                    uint8_t payload[28];
                    for(int j = 0; j < 28; j++) {
                        payload[j] = buffer[2 + j];
                    }
                    uint16_t calc_crc = nextion_crc16_modbus(payload, sizeof(payload));

                    if (recv_crc == calc_crc) {
                        ESP_LOGI("NEXTION", "✅ CRC OK - New physical configuration received!");
                        
                        // Converts the value received from HMI (can be the Combobox index 0,1,2,3 or the value multiplied by 10)
                        uint16_t real_micro_x = 32;
                        if (micro_x == 0) real_micro_x = 4;
                        else if (micro_x == 1) real_micro_x = 8;
                        else if (micro_x == 2) real_micro_x = 16;
                        else if (micro_x == 3) real_micro_x = 32;
                        else if (micro_x >= 40) real_micro_x = micro_x / 10;
                        else real_micro_x = micro_x;

                        uint16_t real_micro_y = 32;
                        if (micro_y == 0) real_micro_y = 4;
                        else if (micro_y == 1) real_micro_y = 8;
                        else if (micro_y == 2) real_micro_y = 16;
                        else if (micro_y == 3) real_micro_y = 32;
                        else if (micro_y >= 40) real_micro_y = micro_y / 10;
                        else real_micro_y = micro_y;

                        float max_freq_x_hz = (max_freq_x_val == 0) ? 1.5f : 5.0f;
                        float max_freq_y_hz = (max_freq_y_val == 0) ? 1.5f : 5.0f;

                        // Update global structs
                        kinematics_init_axis(&table_config_x.axis, p2p_x / 10.0f, crank_x / 10.0f, rod_x / 10.0f, max_freq_x_hz);
                        kinematics_init_stepper(&table_config_x.stepper, step_x / 10.0f, gear_x / 100.0f, real_micro_x);
                        
                        kinematics_init_axis(&table_config_y.axis, p2p_y / 10.0f, crank_y / 10.0f, rod_y / 10.0f, max_freq_y_hz);
                        kinematics_init_stepper(&table_config_y.stepper, step_y / 10.0f, gear_y / 100.0f, real_micro_y);
                        
                        // Save to NVS
                        config_manager_save('x', &table_config_x);
                        config_manager_save('y', &table_config_y);
                        
                        // Update MCP23017 hardware output states dynamically based on received values
                        set_all_steppers_microsteps(real_micro_x, real_micro_y);
                        
                        ESP_LOGI("NEXTION", "New configuration saved to flash memory (NVS)!");
                        sendAckToNextion(202); // ACK: OK
                        vTaskDelay(pdMS_TO_TICKS(15)); // Small pause to ensure ACK is sent before resetting the flag
                        
                    } else {
                        ESP_LOGW("NEXTION", "❌ Invalid CRC (expected 0x%04X, received 0x%04X)", calc_crc, recv_crc);
                        sendAckToNextion(203); // ACK ERROR
                    }
                }
            }
            // End Shaking Table Profiles Data Packet (0x55) -------------------------------------------------------------
            else {
                return_data_from_nextion(buffer, index);
            }
            index = 0; // reset buffer
        }

        // Protect from overflow
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