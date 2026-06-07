#pragma once

#include <stdint.h>

/**
 * @file motor_output_types.h
 * @brief Motor output (MCPWM) data structures.
 */

typedef enum {
    MOTOR_OUTPUT_PWM = 0,     // Standard PWM (1000-2000 us)
    MOTOR_OUTPUT_ONESHOT125,  // OneShot125 (125-250 us)
    MOTOR_OUTPUT_DSHOT150,    // DShot150
    MOTOR_OUTPUT_DSHOT300,    // DShot300
    MOTOR_OUTPUT_DSHOT600,    // DShot600
} motor_output_protocol_t;

typedef struct {
    motor_output_protocol_t protocol;
    uint32_t frequency_hz;        // PWM frequency (e.g., 400 Hz)
    uint16_t min_pulse_us;        // Minimum pulse width (e.g., 1000)
    uint16_t max_pulse_us;        // Maximum pulse width (e.g., 2000)
    uint16_t idle_pulse_us;       // Disarmed/idle pulse (e.g., 1000)
    bool inverted;                // Invert output
} motor_output_config_t;

typedef struct {
    uint16_t pulse_us[4];  // Current pulse width per motor (us)
    bool armed;            // Whether output is armed
} motor_output_state_t;
