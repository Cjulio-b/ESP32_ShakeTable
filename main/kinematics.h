#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>

// Structure for the physical parameters of a Shake Table axis
typedef struct {
    float peak_to_peak_disp_mm; // Peak-to-peak displacement (mm)
    float max_amplitude_mm;     // Maximum amplitude (peak_to_peak_disp_mm / 2)
    float crank_radius_mm;      // Crank radius (usually equal to max_amplitude)
    float rod_length_mm;        // Rod length
} shake_table_axis_config_t;

// Structure for the stepper motor parameters
typedef struct {
    float step_angle_deg;          // Base resolution of the motor (e.g., 1.8º)
    float gear_ratio;              // Gearbox ratio (e.g., 5.18)
    uint16_t microsteps;           // Microstepping mode (e.g., 32)
    
    // Automatically calculated parameters
    uint16_t full_steps_per_rev;   // Full steps per revolution (e.g., 200)
    uint32_t microsteps_per_rev;   // Microsteps per revolution (e.g., 6400)
    float angular_resolution_deg;  // Angular resolution per microstep (e.g., 0.05625º)
} stepper_config_t;

// Global structure that aggregates the physical and motor config for a single axis
typedef struct {
    shake_table_axis_config_t axis;
    stepper_config_t stepper;
} shake_table_config_t;

/**
 * @brief Initializes the stepper motor properties by calculating steps/rev and resolution
 */
void kinematics_init_stepper(stepper_config_t *stepper, float step_angle_deg, float gear_ratio, uint16_t microsteps);

/**
 * @brief Initializes the physical dimensions of the table (Slider-Crank mechanism)
 */
void kinematics_init_axis(shake_table_axis_config_t *axis, float peak_to_peak_safety_limit_mm, float crank_radius_mm, float rod_length_mm);

/**
 * @brief Calculates the current angular position of the motor (in degrees [0 to 360]) based on the given microsteps
 */
float kinematics_calc_angular_position(const stepper_config_t *stepper, int32_t current_step);

/**
 * @brief Calculates the exact linear position of the carriage in mm, from the motor angle
 */
float kinematics_calc_linear_position(const shake_table_axis_config_t *axis, float angle_deg);

/**
 * @brief Calculates the real linear position of the carriage in mm, relative to the dead center (motor at 90º)
 */
float kinematics_calc_linear_position_relative_90(const shake_table_axis_config_t *axis, float angle_deg);

/**
 * @brief Calculates the required motor angle (in degrees) to reach a target linear position (relative to 90 degrees)
 */
float kinematics_calc_inverse_position_relative_90(const shake_table_axis_config_t *axis, float target_relative_pos_mm);

#endif /* KINEMATICS_H_ */