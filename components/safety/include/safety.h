#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file safety.h
 * @brief Safety state machine for flight controller.
 *
 * States: BOOT, DISARMED, PRE_ARM_CHECK, ARMED, FAILSAFE, ERROR
 *
 * Safety rules:
 * - Boot never enters ARMED
 * - ARMED only via DISARMED -> PRE_ARM_CHECK -> ARMED
 * - Non-ARMED states force 1000 us idle output
 * - Sensor/estimator loss while ARMED -> FAILSAFE
 * - ERROR latches safe output, no auto-recovery
 * - Emergency stop reachable from any state
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_STATE_BOOT = 0,
    SAFETY_STATE_DISARMED,
    SAFETY_STATE_PRE_ARM_CHECK,
    SAFETY_STATE_ARMED,
    SAFETY_STATE_FAILSAFE,
    SAFETY_STATE_ERROR
} safety_state_t;

typedef struct {
    safety_state_t state;
    uint32_t state_entry_time_ms;
    uint32_t last_transition_time_ms;
    bool can_arm;
    const char* last_failure_reason;
} safety_status_t;

/**
 * @brief Initialize safety state machine.
 * Enters BOOT state.
 * @return ESP_OK on success
 */
esp_err_t safety_init(void);

/**
 * @brief Update safety state machine (call each control loop cycle).
 * Monitors health, enforces state transitions, forces safe outputs.
 * @return ESP_OK on success
 */
esp_err_t safety_update(void);

/**
 * @brief Request arming transition.
 * State machine will attempt DISARMED -> PRE_ARM_CHECK -> ARMED.
 * @return ESP_OK if request accepted, error otherwise
 */
esp_err_t safety_request_arm(void);

/**
 * @brief Request disarming transition.
 * Forces transition to DISARMED from any state except ERROR.
 * @return ESP_OK on success
 */
esp_err_t safety_request_disarm(void);

/**
 * @brief Emergency stop - immediately force safe state.
 * Callable from any state. Forces FAILSAFE or ERROR.
 */
void safety_emergency_stop(void);

/**
 * @brief Get current safety state.
 * @return Current state
 */
safety_state_t safety_get_state(void);

/**
 * @brief Get detailed safety status.
 * @param status Output status struct
 */
void safety_get_status(safety_status_t* status);

/**
 * @brief Check if system is armed.
 * @return true if in ARMED state
 */
bool safety_is_armed(void);

/**
 * @brief Check if motor output is allowed above idle.
 * @return true only if ARMED
 */
bool safety_is_motor_output_allowed(void);

#ifdef __cplusplus
}
#endif
