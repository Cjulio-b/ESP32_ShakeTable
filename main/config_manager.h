#ifndef CONFIG_MANAGER_H_
#define CONFIG_MANAGER_H_

#include "kinematics.h"
#include "esp_err.h"

// Declaração das estruturas de configuração globais que serão usadas em todo o projeto
extern shake_table_config_t table_config_x;
extern shake_table_config_t table_config_y;

/**
 * @brief Inicializa o gestor de configuração, carregando os parâmetros da NVS.
 *
 * Se a NVS estiver vazia ou os parâmetros não forem encontrados, carrega os valores
 * por defeito (defaults) e guarda-os na NVS para os próximos arranques.
 *
 * @return ESP_OK em caso de sucesso, ou um código de erro das funções NVS.
 */
esp_err_t config_manager_init(void);

/**
 * @brief Guarda a configuração atual de um eixo específico para a NVS.
 */
esp_err_t config_manager_save(char axis, const shake_table_config_t *config);

#endif /* CONFIG_MANAGER_H_ */