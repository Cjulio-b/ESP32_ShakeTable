#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>

// Estrutura para os parâmetros físicos de um eixo da Shake Table
typedef struct {
    float peak_to_peak_disp_mm; // Deslocamento pico a pico (mm)
    float max_amplitude_mm;     // Amplitude máxima (peak_to_peak_disp_mm / 2)
    float crank_radius_mm;      // Raio da manivela (geralmente igual à max_amplitude)
    float rod_length_mm;        // Comprimento da biela
} shake_table_axis_config_t;

// Estrutura para os parâmetros do motor de passo
typedef struct {
    float step_angle_deg;          // Resolução base do motor (ex: 1.8º)
    uint16_t microsteps;           // Modo de microstepping (ex: 32)
    
    // Parâmetros calculados automaticamente
    uint16_t full_steps_per_rev;   // Passos completos por volta (ex: 200)
    uint32_t microsteps_per_rev;   // Micropassos por volta (ex: 6400)
    float angular_resolution_deg;  // Resolução angular por micropasso (ex: 0.05625º)
} stepper_config_t;

// Estrutura global que agrega os eixos e motores X e Y
typedef struct {
    shake_table_axis_config_t axis_x;
    shake_table_axis_config_t axis_y;
    stepper_config_t stepper_x;
    stepper_config_t stepper_y;
} shake_table_config_t;

/**
 * @brief Inicializa as propriedades do motor de passo calculando os passos/volta e resolução
 */
void kinematics_init_stepper(stepper_config_t *stepper, float step_angle_deg, uint16_t microsteps);

/**
 * @brief Inicializa as dimensões físicas da mesa (Biela-Manivela)
 */
void kinematics_init_axis(shake_table_axis_config_t *axis, float peak_to_peak_mm, float rod_length_mm);

/**
 * @brief Calcula a posição angular atual do motor (em graus [0 a 360]) baseado nos micropassos dados
 */
float kinematics_calc_angular_position(const stepper_config_t *stepper, int32_t current_step);

/**
 * @brief Calcula a posição linear exata do carrinho em mm, a partir do ângulo do motor
 */
float kinematics_calc_linear_position(const shake_table_axis_config_t *axis, float angle_deg);

/**
 * @brief Calcula a posição linear real do carrinho em mm, relativa ao ponto morto (motor a 90º)
 */
float kinematics_calc_linear_position_relative_90(const shake_table_axis_config_t *axis, float angle_deg);

#endif /* KINEMATICS_H_ */