#include "kinematics.h"
#include <math.h>

#define PI 3.14159265358979323846f

void kinematics_init_stepper(stepper_config_t *stepper, float step_angle_deg, uint16_t microsteps) {
    if (!stepper) return;
    
    stepper->step_angle_deg = step_angle_deg;
    stepper->microsteps = microsteps;
    
    // Cálculos derivados
    stepper->full_steps_per_rev = (uint16_t)(360.0f / step_angle_deg);
    stepper->microsteps_per_rev = stepper->full_steps_per_rev * microsteps;
    stepper->angular_resolution_deg = 360.0f / (float)stepper->microsteps_per_rev;
}

void kinematics_init_axis(shake_table_axis_config_t *axis, float peak_to_peak_mm, float rod_length_mm) {
    if (!axis) return;
    
    axis->peak_to_peak_disp_mm = peak_to_peak_mm;
    axis->max_amplitude_mm = peak_to_peak_mm / 2.0f;
    axis->crank_radius_mm = axis->max_amplitude_mm; // O r da manivela é equivalente à amplitude
    axis->rod_length_mm = rod_length_mm;
}

float kinematics_calc_angular_position(const stepper_config_t *stepper, int32_t current_step) {
    if (!stepper) return 0.0f;
    
    // Multiplica os passos pela resolução por passo. 
    // O fmodf obriga a que o ângulo permaneça no intervalo [0, 360].
    float angle = fmodf((float)current_step * stepper->angular_resolution_deg, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

float kinematics_calc_linear_position(const shake_table_axis_config_t *axis, float angle_deg) {
    if (!axis) return 0.0f;
    
    // Converte os graus calculados para radianos para aplicar na trigonometria
    float angle_rad = angle_deg * (PI / 180.0f);
    
    float r = axis->crank_radius_mm;
    float l = axis->rod_length_mm;
    
    // Equação Biela-Manivela (Slider-Crank mechanism)
    // x(θ) = r * cos(θ) + sqrt(l² - r² * sin²(θ))
    float term1 = r * cosf(angle_rad);
    float sin_theta = sinf(angle_rad);
    float term2 = sqrtf((l * l) - (r * r * sin_theta * sin_theta));
    
    // Calculamos a distância total ao eixo do motor e subtraímos 'l' (comprimento da biela).
    // Desta forma obtemos a posição centrada em 0 (mesa repousa em 0 quando as molas estão centradas)
    float x_centered = (term1 + term2) - l;
    
    return x_centered;
}

float kinematics_calc_linear_position_relative_90(const shake_table_axis_config_t *axis, float angle_deg) {
    if (!axis) return 0.0f;
    
    float angle_rad = angle_deg * (PI / 180.0f);
    
    float r = axis->crank_radius_mm;
    float l = axis->rod_length_mm;
    
    // 1. Calcular a posição absoluta no ângulo atual: x(θ) = r*cos(θ) + sqrt(l² - r²*sin²(θ))
    float term1 = r * cosf(angle_rad);
    float sin_theta = sinf(angle_rad);
    float current_abs_pos = term1 + sqrtf((l * l) - (r * r * sin_theta * sin_theta));
    
    // 2. Calcular a posição absoluta exata quando o motor está a 90º
    // A 90º: cos(90º) = 0, sin(90º) = 1. A equação simplifica para: 0 + sqrt(l² - r²(1)²)
    float pos_90_deg = sqrtf((l * l) - (r * r));
    
    // 3. Retornar a diferença
    return current_abs_pos - pos_90_deg;
}