#pragma once

#include "esp_err.h"
#include "motor_mixer_types.h"
#include "pid_types.h"

/**
 * @file motor_mixer.h
 * @brief Quad X motor mixer: converts PID outputs to motor commands.
 *
 * Mixing matrix for Quad X:
 *   motor[FR] = throttle + roll - pitch - yaw
 *   motor[FL] = throttle - roll - pitch + yaw
 *   motor[RR] = throttle + roll + pitch + yaw
 *   motor[RL] = throttle - roll + pitch - yaw
 *
 * Where:
 *   throttle: collective thrust (0..1)
 *   roll:     PID roll output (-1..1)
 *   pitch:    PID pitch output (-1..1)
 *   yaw:      PID yaw output (-1..1)
 *
 * Output is normalized and clamped to [0, 1] or [min, max] throttle range.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Throttle limits (normalized 0..1)
#define MOTOR_MIN_THROTTLE  0.08f   // ~1100 us equivalent
#define MOTOR_MAX_THROTTLE  0.95f   // ~1900 us equivalent
#define MOTOR_IDLE_THROTTLE 0.10f   // Disarmed/idle

/**
 * @brief Initialize motor mixer.
 * @return ESP_OK on success
 */
esp_err_t motor_mixer_init(void);

/**
 * @brief Mix PID outputs + throttle into motor commands.
 * @param pid_output Array[3] from pid_compute: [roll, pitch, yaw] in -1..1
 * @param throttle Collective throttle 0..1
 * @param output Output motor commands (normalized 0..1)
 * @return ESP_OK on success
 */
esp_err_t motor_mixer_mix(const float pid_output[3], float throttle, motor_mixer_output_t* output);

/**
 * @brief Set throttle limits.
 * @param min Minimum throttle (0..1)
 * @param max Maximum throttle (0..1)
 */
void motor_mixer_set_limits(float min, float max);

/**
 * @brief Get current throttle limits.
 * @param min Output minimum
 * @param max Output maximum
 */
void motor_mixer_get_limits(float* min, float* max);

/**
 * @brief Convert normalized motor command (0..1) to PWM microseconds.
 * @param normalized Motor command 0..1
 * @return PWM pulse width in microseconds (1000..2000)
 */
uint16_t motor_mixer_to_pwm_us(float normalized);

/**
 * @brief Disarm all motors (set to idle).
 * @param output Output motor commands
 */
void motor_mixer_disarm(motor_mixer_output_t* output);

#ifdef __cplusplus
}
#endif