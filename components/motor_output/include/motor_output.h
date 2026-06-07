#pragma once

#include "esp_err.h"
#include "motor_output_types.h"
#include "motor_mixer_types.h"

/**
 * @file motor_output.h
 * @brief MCPWM driver for ESC output (4 channels).
 *
 * Supports:
 * - Standard PWM (1000-2000 µs, configurable frequency)
 * - OneShot125 (125-250 µs)
 * - DShot150/300/600 (future)
 *
 * Uses ESP32 MCPWM peripheral (2 units, 3 timers each, 2 operators per timer = 12 channels).
 * We use 4 channels: one per motor.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Default configuration
#define MOTOR_OUTPUT_DEFAULT_FREQ_HZ     400
#define MOTOR_OUTPUT_DEFAULT_MIN_US      1000
#define MOTOR_OUTPUT_DEFAULT_MAX_US      2000
#define MOTOR_OUTPUT_DEFAULT_IDLE_US     1000

/**
 * @brief Initialize motor output (MCPWM).
 * @param config Configuration (can be NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t motor_output_init(const motor_output_config_t* config);

/**
 * @brief Deinitialize motor output.
 */
void motor_output_deinit(void);

/**
 * @brief Set motor pulse widths (called from control loop).
 * @param output Motor mixer output (normalized 0..1)
 * @return ESP_OK on success
 */
esp_err_t motor_output_set(const motor_mixer_output_t* output);

/**
 * @brief Arm motor output (enable MCPWM outputs).
 * @return ESP_OK on success
 */
esp_err_t motor_output_arm(void);

/**
 * @brief Disarm motor output (set to idle, disable MCPWM outputs).
 * @return ESP_OK on success
 */
esp_err_t motor_output_disarm(void);

/**
 * @brief Check if motor output is initialized.
 * @return true if initialized
 */
bool motor_output_is_initialized(void);

/**
 * @brief Check if motor output is armed.
 * @return true if armed
 */
bool motor_output_is_armed(void);

/**
 * @brief Get current motor output state.
 * @param state Output state
 */
void motor_output_get_state(motor_output_state_t* state);

/**
 * @brief Update configuration (protocol, frequency, limits).
 * @param config New configuration
 * @return ESP_OK on success
 */
esp_err_t motor_output_set_config(const motor_output_config_t* config);

/**
 * @brief Get current configuration.
 * @param config Output configuration
 */
void motor_output_get_config(motor_output_config_t* config);

/**
 * @brief Emergency stop - immediately set all motors to idle and disarm.
 */
void motor_output_emergency_stop(void);

#ifdef __cplusplus
}
#endif