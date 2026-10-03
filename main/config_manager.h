#ifndef CONFIG_MANAGER_H_
#define CONFIG_MANAGER_H_

#include "kinematics.h"
#include "esp_err.h"

// Declaration of global configuration structures to be used throughout the project
extern shake_table_config_t table_config_x;
extern shake_table_config_t table_config_y;

/**
 * @brief Initializes the configuration manager, loading parameters from NVS.
 *
 * If NVS is empty or parameters are not found, loads the default values
 * and saves them to NVS for future boots.
 *
 * @return ESP_OK on success, or an error code from NVS functions.
 */
esp_err_t config_manager_init(void);

/**
 * @brief Saves the current configuration of a specific axis to NVS.
 */
esp_err_t config_manager_save(char axis, const shake_table_config_t *config);

#endif /* CONFIG_MANAGER_H_ */