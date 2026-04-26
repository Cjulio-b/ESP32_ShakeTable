#include "kinematics.h"
#include <math.h>

#define PI 3.14159265358979323846f

void kinematics_init_stepper(stepper_config_t *stepper, float step_angle_deg, uint16_t microsteps) {
    if (!stepper) return;
    
    stepper->step_angle_deg = step_angle_deg;
    stepper->microsteps = microsteps;
    
    // Derived calculations
    stepper->full_steps_per_rev = (uint16_t)(360.0f / step_angle_deg);
    stepper->microsteps_per_rev = stepper->full_steps_per_rev * microsteps;
    stepper->angular_resolution_deg = 360.0f / (float)stepper->microsteps_per_rev;
}

void kinematics_init_axis(shake_table_axis_config_t *axis, float peak_to_peak_mm, float rod_length_mm) {
    if (!axis) return;
    
    axis->peak_to_peak_disp_mm = peak_to_peak_mm;
    axis->max_amplitude_mm = peak_to_peak_mm / 2.0f;
    axis->crank_radius_mm = axis->max_amplitude_mm; // The crank radius 'r' is equivalent to the amplitude
    axis->rod_length_mm = rod_length_mm;
}

float kinematics_calc_angular_position(const stepper_config_t *stepper, int32_t current_step) {
    if (!stepper) return 0.0f;
    
    // Multiply steps by the resolution per step.
    // The fmodf forces the angle to remain in the [0, 360] range.
    float angle = fmodf((float)current_step * stepper->angular_resolution_deg, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

float kinematics_calc_linear_position(const shake_table_axis_config_t *axis, float angle_deg) {
    if (!axis) return 0.0f;
    
    // Convert the calculated degrees to radians to apply in trigonometry
    float angle_rad = angle_deg * (PI / 180.0f);
    
    float r = axis->crank_radius_mm;
    float l = axis->rod_length_mm;
    
    // Slider-Crank mechanism equation
    // x(θ) = r * cos(θ) + sqrt(l² - r² * sin²(θ))
    float term1 = r * cosf(angle_rad);
    float sin_theta = sinf(angle_rad);
    float term2 = sqrtf((l * l) - (r * r * sin_theta * sin_theta));
    
    // Calculate the total distance to the motor axis and subtract 'l' (rod length).
    // This way we get the position centered at 0 (table rests at 0 when springs are centered)
    float x_centered = (term1 + term2) - l;
    
    return x_centered;
}

float kinematics_calc_linear_position_relative_90(const shake_table_axis_config_t *axis, float angle_deg) {
    if (!axis) return 0.0f;
    
    float angle_rad = angle_deg * (PI / 180.0f);
    
    float r = axis->crank_radius_mm;
    float l = axis->rod_length_mm;
    
    // 1. Calculate the absolute position at the current angle: x(θ) = r*cos(θ) + sqrt(l² - r²*sin²(θ))
    float term1 = r * cosf(angle_rad);
    float sin_theta = sinf(angle_rad);
    float current_abs_pos = term1 + sqrtf((l * l) - (r * r * sin_theta * sin_theta));
    
    // 2. Calculate the exact absolute position when the motor is at 90 degrees
    // At 90º: cos(90º) = 0, sin(90º) = 1. The equation simplifies to: 0 + sqrt(l² - r²(1)²)
    float pos_90_deg = sqrtf((l * l) - (r * r));
    
    // 3. Return the difference
    return current_abs_pos - pos_90_deg;
}