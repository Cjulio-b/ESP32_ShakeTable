#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------
// PIN CONFIGURATION
// -----------------------------------------------------
#define IN1_GPIO 13
#define IN2_GPIO 12
#define IN3_GPIO 27
#define IN4_GPIO 33

// -----------------------------------------------------
// PUBLIC API
// -----------------------------------------------------

void l298n_init(void);
void l298n_set_min_step_us(uint32_t min_us);

// basic movement: N halfsteps at a fixed frequency
void l298n_move_halfsteps(uint32_t halfsteps, float freq_hz, bool direction);

// movement with trapezoidal linear acceleration
void l298n_move_trapezoidal(uint32_t total_steps,
                            float start_hz,
                            float cruise_hz,
                            float end_hz,
                            float accel_time_s,
                            float decel_time_s,
                            bool direction);

// SEISMIC PROFILES EXECUTION
typedef struct {
    const float *speed_hz;     // speed array (Hz)
    const float *duration_s;   // duration array (seconds)
    int length;                // number of samples
} seismic_profile_t;

// Play seismic profile
void l298n_play_seismic_profile(const seismic_profile_t *profile, bool direction);

// Task for L298N stepper motor control
void stepper_task(void *arg);

#ifdef __cplusplus
}
#endif
