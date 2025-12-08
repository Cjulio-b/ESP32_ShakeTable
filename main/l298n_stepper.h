#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------
// CONFIGURAÇÃO DE PINOS
// -----------------------------------------------------
#define IN1_GPIO 13
#define IN2_GPIO 12
#define IN3_GPIO 27
#define IN4_GPIO 33

// -----------------------------------------------------
// API PÚBLICA
// -----------------------------------------------------

void l298n_init(void);
void l298n_set_min_step_us(uint32_t min_us);

// movimento básico: N halfsteps a uma frequência fixa
void l298n_move_halfsteps(uint32_t halfsteps, float freq_hz, bool direction);

// movimento com aceleração linear trapezoidal
void l298n_move_trapezoidal(uint32_t total_steps,
                            float start_hz,
                            float cruise_hz,
                            float end_hz,
                            float accel_time_s,
                            float decel_time_s,
                            bool direction);

// EXECUÇÃO DE PERFIS SÍSMICOS
typedef struct {
    const float *speed_hz;     // array de velocidades (Hz)
    const float *duration_s;   // array de durações (segundos)
    int length;                // número de amostras
} seismic_profile_t;

// Reproduzir perfil sísmico
void l298n_play_seismic_profile(const seismic_profile_t *profile, bool direction);

// Task para controle do motor de passo L298N
void stepper_task(void *arg);

#ifdef __cplusplus
}
#endif
