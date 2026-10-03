#include "config_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "CONFIG_MANAGER";

// Definition of global configuration structures
shake_table_config_t table_config_x;
shake_table_config_t table_config_y;

// NVS Namespace and Keys
#define NVS_NAMESPACE "shake_config"
#define KEY_P2P_X "p2p_x"
#define KEY_CRANK_X "crank_x"
#define KEY_ROD_X "rod_x"
#define KEY_STEP_X "step_x"
#define KEY_GEAR_X "gear_x"
#define KEY_MICRO_X "micro_x"
#define KEY_FREQ_X "freq_x"

#define KEY_P2P_Y "p2p_y"
#define KEY_CRANK_Y "crank_y"
#define KEY_ROD_Y "rod_y"
#define KEY_STEP_Y "step_y"
#define KEY_GEAR_Y "gear_y"
#define KEY_MICRO_Y "micro_y"
#define KEY_FREQ_Y "freq_y"

// Helper function to load a float from NVS (stored as u32)
static esp_err_t load_float(nvs_handle_t handle, const char* key, float* value, float default_val) {
    esp_err_t err = nvs_get_u32(handle, key, (uint32_t*)value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Key '%s' not found in NVS. Using default: %.2f", key, default_val);
        *value = default_val;
        return ESP_ERR_NVS_NOT_FOUND;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading key '%s': %s", key, esp_err_to_name(err));
    }
    return err;
}

// Helper function to load a u16 from NVS
static esp_err_t load_u16(nvs_handle_t handle, const char* key, uint16_t* value, uint16_t default_val) {
    esp_err_t err = nvs_get_u16(handle, key, value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Key '%s' not found in NVS. Using default: %u", key, default_val);
        *value = default_val;
        return ESP_ERR_NVS_NOT_FOUND;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading key '%s': %s", key, esp_err_to_name(err));
    }
    return err;
}

esp_err_t config_manager_init(void) {
    esp_err_t err;
    nvs_handle_t my_handle;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // --- Load Axis X Configuration ---
    float p2p_x, crank_x, rod_x, step_angle_x, gear_ratio_x, max_freq_x;
    uint16_t microsteps_x;
    bool save_x = false;

    if (load_float(my_handle, KEY_P2P_X, &p2p_x, 27.33f) != ESP_OK) save_x = true;
    if (load_float(my_handle, KEY_CRANK_X, &crank_x, 14.0f) != ESP_OK) save_x = true;
    if (load_float(my_handle, KEY_ROD_X, &rod_x, 100.0f) != ESP_OK) save_x = true;
    if (load_float(my_handle, KEY_STEP_X, &step_angle_x, 1.8f) != ESP_OK) save_x = true;
    if (load_float(my_handle, KEY_GEAR_X, &gear_ratio_x, 5.18f) != ESP_OK) save_x = true;
    if (load_u16(my_handle, KEY_MICRO_X, &microsteps_x, 32) != ESP_OK) save_x = true;
    if (load_float(my_handle, KEY_FREQ_X, &max_freq_x, 1.5f) != ESP_OK) save_x = true;

    // --- Load Axis Y Configuration ---
    float p2p_y, crank_y, rod_y, step_angle_y, gear_ratio_y, max_freq_y;
    uint16_t microsteps_y;
    bool save_y = false;

    if (load_float(my_handle, KEY_P2P_Y, &p2p_y, 26.84f) != ESP_OK) save_y = true;
    if (load_float(my_handle, KEY_CRANK_Y, &crank_y, 14.0f) != ESP_OK) save_y = true;
    if (load_float(my_handle, KEY_ROD_Y, &rod_y, 95.0f) != ESP_OK) save_y = true;
    if (load_float(my_handle, KEY_STEP_Y, &step_angle_y, 1.8f) != ESP_OK) save_y = true;
    if (load_float(my_handle, KEY_GEAR_Y, &gear_ratio_y, 5.18f) != ESP_OK) save_y = true;
    if (load_u16(my_handle, KEY_MICRO_Y, &microsteps_y, 32) != ESP_OK) save_y = true;
    if (load_float(my_handle, KEY_FREQ_Y, &max_freq_y, 1.5f) != ESP_OK) save_y = true;

    nvs_close(my_handle);

    // Initialize kinematic structures with loaded/default values
    kinematics_init_axis(&table_config_x.axis, p2p_x, crank_x, rod_x, max_freq_x);
    kinematics_init_stepper(&table_config_x.stepper, step_angle_x, gear_ratio_x, microsteps_x);

    kinematics_init_axis(&table_config_y.axis, p2p_y, crank_y, rod_y, max_freq_y);
    kinematics_init_stepper(&table_config_y.stepper, step_angle_y, gear_ratio_y, microsteps_y);

    ESP_LOGI(TAG, "Config X Loaded: P2P=%.2f, Crank=%.2f, Rod=%.2f, Step=%.2f, Gear=%.2f, Micro=%u",
             table_config_x.axis.peak_to_peak_disp_mm, table_config_x.axis.crank_radius_mm, table_config_x.axis.rod_length_mm,
             table_config_x.stepper.step_angle_deg, table_config_x.stepper.gear_ratio, table_config_x.stepper.microsteps);

    ESP_LOGI(TAG, "Config Y Loaded: P2P=%.2f, Crank=%.2f, Rod=%.2f, Step=%.2f, Gear=%.2f, Micro=%u",
             table_config_y.axis.peak_to_peak_disp_mm, table_config_y.axis.crank_radius_mm, table_config_y.axis.rod_length_mm,
             table_config_y.stepper.step_angle_deg, table_config_y.stepper.gear_ratio, table_config_y.stepper.microsteps);

    // Save defaults to NVS if it's the first boot
    if (save_x) config_manager_save('x', &table_config_x);
    if (save_y) config_manager_save('y', &table_config_y);

    return ESP_OK;
}

esp_err_t config_manager_save(char axis, const shake_table_config_t *config) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    if (axis == 'x') {
        ESP_LOGI(TAG, "Saving Axis X config to NVS...");
        nvs_set_u32(my_handle, KEY_P2P_X,   *((uint32_t*)&config->axis.peak_to_peak_disp_mm));
        nvs_set_u32(my_handle, KEY_CRANK_X, *((uint32_t*)&config->axis.crank_radius_mm));
        nvs_set_u32(my_handle, KEY_ROD_X,   *((uint32_t*)&config->axis.rod_length_mm));
        nvs_set_u32(my_handle, KEY_STEP_X,  *((uint32_t*)&config->stepper.step_angle_deg));
        nvs_set_u32(my_handle, KEY_GEAR_X,  *((uint32_t*)&config->stepper.gear_ratio));
        nvs_set_u16(my_handle, KEY_MICRO_X, config->stepper.microsteps);
        nvs_set_u32(my_handle, KEY_FREQ_X,  *((uint32_t*)&config->axis.max_freq_hz));
    } else if (axis == 'y') {
        ESP_LOGI(TAG, "Saving Axis Y config to NVS...");
        nvs_set_u32(my_handle, KEY_P2P_Y,   *((uint32_t*)&config->axis.peak_to_peak_disp_mm));
        nvs_set_u32(my_handle, KEY_CRANK_Y, *((uint32_t*)&config->axis.crank_radius_mm));
        nvs_set_u32(my_handle, KEY_ROD_Y,   *((uint32_t*)&config->axis.rod_length_mm));
        nvs_set_u32(my_handle, KEY_STEP_Y,  *((uint32_t*)&config->stepper.step_angle_deg));
        nvs_set_u32(my_handle, KEY_GEAR_Y,  *((uint32_t*)&config->stepper.gear_ratio));
        nvs_set_u16(my_handle, KEY_MICRO_Y, config->stepper.microsteps);
        nvs_set_u32(my_handle, KEY_FREQ_Y,  *((uint32_t*)&config->axis.max_freq_hz));
    }

    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    return err;
}