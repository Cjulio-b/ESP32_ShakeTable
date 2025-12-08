#include "l298n_stepper.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"   // ets_delay_us()
#include <math.h>

static uint32_t g_min_step_us = 200;   // default safety

// HALF-STEP SEQUENCE (8 steps)
static const int seq[8][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

static inline void write_coils(int s)
{
    gpio_set_level(IN1_GPIO, seq[s][0]);
    gpio_set_level(IN2_GPIO, seq[s][1]);
    gpio_set_level(IN3_GPIO, seq[s][2]);
    gpio_set_level(IN4_GPIO, seq[s][3]);
}

void l298n_init(void)
{
    gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << IN1_GPIO) |
            (1ULL << IN2_GPIO) |
            (1ULL << IN3_GPIO) |
            (1ULL << IN4_GPIO)
    };
    gpio_config(&cfg);
}

void l298n_set_min_step_us(uint32_t min_us)
{
    g_min_step_us = min_us;
}

// ---------------------------------------------------
// MOVIMENTO HALF-STEP SIMPLES
// ---------------------------------------------------
void l298n_move_halfsteps(uint32_t halfsteps, float freq_hz, bool direction)
{
    if (freq_hz <= 0) return;

    float step_us_f = 1e6f / freq_hz;
    if (step_us_f < g_min_step_us) step_us_f = g_min_step_us;
    uint32_t step_us = (uint32_t)step_us_f;

    int idx = 0;

    for (uint32_t i = 0; i < halfsteps; i++)
    {
        write_coils(idx);

        idx += direction ? 1 : -1;
        if (idx > 7) idx = 0;
        if (idx < 0) idx = 7;

        esp_rom_delay_us(step_us);
    }
}


// ---------------------------------------------------
// MOVIMENTO TRAPEZOIDAL
// ---------------------------------------------------
void l298n_move_trapezoidal(uint32_t total_steps,
                            float start_hz,
                            float cruise_hz,
                            float end_hz,
                            float accel_time_s,
                            float decel_time_s,
                            bool direction)
{
    int idx = 0;

    uint32_t accel_steps = (uint32_t)(accel_time_s * cruise_hz);
    uint32_t decel_steps = (uint32_t)(decel_time_s * cruise_hz);
    if (accel_steps + decel_steps > total_steps)
        accel_steps = decel_steps = total_steps / 2;

    uint32_t cruise_steps = total_steps - accel_steps - decel_steps;

    for (uint32_t i = 0; i < accel_steps; i++)
    {
        float t = (float)i / accel_steps;
        float freq = start_hz + t * (cruise_hz - start_hz);

        float step_us_f = 1e6f / freq;
        if (step_us_f < g_min_step_us) step_us_f = g_min_step_us;

        write_coils(idx);
        idx = (direction ? (idx + 1) : (idx + 7)) & 7;
        esp_rom_delay_us((uint32_t)step_us_f);
    }

    for (uint32_t i = 0; i < cruise_steps; i++)
    {
        float step_us_f = 1e6f / cruise_hz;
        if (step_us_f < g_min_step_us) step_us_f = g_min_step_us;

        write_coils(idx);
        idx = (direction ? (idx + 1) : (idx + 7)) & 7;
        esp_rom_delay_us((uint32_t)step_us_f);
    }

    for (uint32_t i = 0; i < decel_steps; i++)
    {
        float t = (float)i / decel_steps;
        float freq = cruise_hz + t * (end_hz - cruise_hz);

        float step_us_f = 1e6f / freq;
        if (step_us_f < g_min_step_us) step_us_f = g_min_step_us;

        write_coils(idx);
        idx = (direction ? (idx + 1) : (idx + 7)) & 7;
        esp_rom_delay_us((uint32_t)step_us_f);
    }
}


// ---------------------------------------------------
// EXECUÇÃO DE PERFIL SÍSMICO
// ---------------------------------------------------
void l298n_play_seismic_profile(const seismic_profile_t *profile, bool direction)
{
    int idx = 0;

    for (int i = 0; i < profile->length; i++)
    {
        float freq = profile->speed_hz[i];
        float duration = profile->duration_s[i];

        if (freq <= 0 || duration <= 0) continue;

        float step_us_f = 1e6f / freq;
        if (step_us_f < g_min_step_us) step_us_f = g_min_step_us;
        uint32_t step_us = (uint32_t)step_us_f;

        uint32_t halfsteps = (uint32_t)(duration * freq);

        for (uint32_t s = 0; s < halfsteps; s++)
        {
            write_coils(idx);
            idx = (direction ? (idx + 1) : (idx + 7)) & 7;
            esp_rom_delay_us(step_us);
        }
    }
}
void stepper_task(void *arg)
{
	// ---------------------------------------------------
    // 1) Mover 2000 halfsteps a 800 Hz
    // ---------------------------------------------------
	ESP_LOGI("STEPMOTOR", "Move 2000 halfsteps at 800 Hz");
    l298n_move_halfsteps(2000, 800, true);
	vTaskDelay(pdMS_TO_TICKS(2000)); // Delay de 2 segundos

    // ---------------------------------------------------
    // 2) Movimento trapezoidal
    // ---------------------------------------------------
	ESP_LOGI("STEPMOTOR", "Trapezoidal move: 4000 halfsteps, 200->1200->200 Hz, 300ms accel/decel");
    l298n_move_trapezoidal(
        4000,       // total halfsteps
        200,        // start
        1200,       // cruise
        200,        // end
        0.3f,       // ramp up 300ms
        0.3f,       // ramp down 300ms
        true
    );
	vTaskDelay(pdMS_TO_TICKS(2000)); // Delay de 2 segundos

    // ---------------------------------------------------
    // 3) Perfil sísmico
    // ---------------------------------------------------
	ESP_LOGI("STEPMOTOR", "Seismic profile move");
    static const float speeds[]   = {300, 800, 1200, 400, 200};
    static const float durations[] = {0.2, 0.2, 0.2, 0.2, 0.2};

    seismic_profile_t profile = {
        .speed_hz = speeds,
        .duration_s = durations,
        .length = 5
    };

    l298n_play_seismic_profile(&profile, true);

    ESP_LOGI("STEPMOTOR", "Done");
}