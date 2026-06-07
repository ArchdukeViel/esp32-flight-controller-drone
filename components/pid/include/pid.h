#pragma once

#include "esp_err.h"
#include "pid_types.h"
#include "estimator_types.h"

/**
 * @file pid.h
 * @brief PID controller with anti-windup for roll/pitch/yaw.
 *
 * Features:
 * - 3-axis PID (roll, pitch, yaw)
 * - Anti-windup via integral clamping
 * - Output saturation with configurable limits
 * - Derivative on measurement (not error) to avoid derivative kick
 * - Gain scheduling support (future)
 */

#ifdef __cplusplus
extern "C" {
#endif

// Default gains (tune per airframe)
#define PID_DEFAULT_KP_ROLL  4.0f
#define PID_DEFAULT_KI_ROLL  0.15f
#define PID_DEFAULT_KD_ROLL  0.03f

#define PID_DEFAULT_KP_PITCH 4.0f
#define PID_DEFAULT_KI_PITCH 0.15f
#define PID_DEFAULT_KD_PITCH 0.03f

#define PID_DEFAULT_KP_YAW   6.0f
#define PID_DEFAULT_KI_YAW   0.2f
#define PID_DEFAULT_KD_YAW   0.0f

#define PID_DEFAULT_INTEGRAL_LIMIT  1.0f   // rad/s equivalent
#define PID_DEFAULT_OUTPUT_LIMIT    1.0f   // Normalized -1..1

/**
 * @brief Initialize all 3 PID controllers with default gains.
 * @return ESP_OK on success
 */
esp_err_t pid_init(void);

/**
 * @brief Reset all PID states (call on arming).
 */
void pid_reset(void);

/**
 * @brief Compute PID output for all axes.
 * Call at fixed rate (e.g., 1 kHz control loop).
 * @param target Target attitude (roll, pitch, yaw rate for yaw)
 * @param current Current estimated attitude (roll, pitch, yaw)
 * @param dt_s Time step in seconds
 * @param output Output array[3] normalized -1..1 (roll, pitch, yaw)
 * @return ESP_OK on success
 */
esp_err_t pid_compute(const estimator_attitude_t* target,
                      const estimator_attitude_t* current,
                      float dt_s,
                      float output[3]);

/**
 * @brief Set gains for a specific axis.
 * @param axis PID_ROLL, PID_PITCH, or PID_YAW
 * @param gains New gains
 */
void pid_set_gains(pid_axis_t axis, const pid_gains_t* gains);

/**
 * @brief Get current gains for a specific axis.
 * @param axis PID_ROLL, PID_PITCH, or PID_YAW
 * @param gains Output gains
 */
void pid_get_gains(pid_axis_t axis, pid_gains_t* gains);

/**
 * @brief Set integral limit (anti-windup) for axis.
 * @param axis PID axis
 * @param limit Max absolute integral value
 */
void pid_set_integral_limit(pid_axis_t axis, float limit);

/**
 * @brief Set output saturation limit for axis.
 * @param axis PID axis
 * @param limit Output limit (positive)
 */
void pid_set_output_limit(pid_axis_t axis, float limit);

/**
 * @brief Get current PID state for debugging.
 * @param axis PID axis
 * @param state Output state
 */
void pid_get_state(pid_axis_t axis, pid_state_t* state);

#ifdef __cplusplus
}
#endif