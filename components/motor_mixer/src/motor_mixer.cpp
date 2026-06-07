#include "motor_mixer.h"
#include "esp_log.h"
#include "math.h"

static const char* TAG = "motor_mixer";

static float s_min_throttle = MOTOR_MIN_THROTTLE;
static float s_max_throttle = MOTOR_MAX_THROTTLE;

esp_err_t motor_mixer_init(void)
{
    ESP_LOGI(TAG, "Motor mixer initialized (Quad X)");
    ESP_LOGI(TAG, "Throttle limits: min=%.3f max=%.3f", s_min_throttle, s_max_throttle);
    return ESP_OK;
}

esp_err_t motor_mixer_mix(const float pid_output[3], float throttle, motor_mixer_output_t* output)
{
    if (!pid_output || !output) return ESP_ERR_INVALID_ARG;

    // Clamp throttle
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;

    float roll  = pid_output[PID_ROLL];
    float pitch = pid_output[PID_PITCH];
    float yaw   = pid_output[PID_YAW];

    // Quad X mixing
    float m[4];
    m[MOTOR_FR] = throttle + roll - pitch - yaw;
    m[MOTOR_FL] = throttle - roll - pitch + yaw;
    m[MOTOR_RR] = throttle + roll + pitch + yaw;
    m[MOTOR_RL] = throttle - roll + pitch - yaw;

    // Normalize: if any motor exceeds limits, scale all down
    float max_m = m[0];
    for (int i = 1; i < 4; i++) if (m[i] > max_m) max_m = m[i];
    float min_m = m[0];
    for (int i = 1; i < 4; i++) if (m[i] < min_m) min_m = m[i];

    if (max_m > s_max_throttle) {
        float scale = (s_max_throttle - throttle) / (max_m - throttle);
        for (int i = 0; i < 4; i++) {
            m[i] = throttle + (m[i] - throttle) * scale;
        }
    }
    if (min_m < s_min_throttle) {
        float scale = (throttle - s_min_throttle) / (throttle - min_m);
        for (int i = 0; i < 4; i++) {
            m[i] = throttle - (throttle - m[i]) * scale;
        }
    }

    // Final clamp
    for (int i = 0; i < 4; i++) {
        if (m[i] < s_min_throttle) m[i] = s_min_throttle;
        if (m[i] > s_max_throttle) m[i] = s_max_throttle;
        output->motor[i] = m[i];
    }

    return ESP_OK;
}

void motor_mixer_set_limits(float min, float max)
{
    if (min >= 0.0f && min < 1.0f) s_min_throttle = min;
    if (max > 0.0f && max <= 1.0f) s_max_throttle = max;
}

void motor_mixer_get_limits(float* min, float* max)
{
    if (min) *min = s_min_throttle;
    if (max) *max = s_max_throttle;
}

uint16_t motor_mixer_to_pwm_us(float normalized)
{
    // Map 0..1 to 1000..2000 us
    float us = 1000.0f + normalized * 1000.0f;
    if (us < 1000.0f) us = 1000.0f;
    if (us > 2000.0f) us = 2000.0f;
    return (uint16_t)us;
}

void motor_mixer_disarm(motor_mixer_output_t* output)
{
    if (!output) return;
    for (int i = 0; i < 4; i++) {
        output->motor[i] = MOTOR_IDLE_THROTTLE;
    }
}