#pragma once

#include <stdint.h>

/**
 * @file pid_types.h
 * @brief PID controller data structures.
 */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;   // Anti-windup: max |integral|
    float output_limit;     // Output saturation limit
    float derivative_limit; // Optional: limit derivative term
} pid_gains_t;

typedef struct {
    float integral;
    float prev_error;
    float prev_derivative;
    bool initialized;
} pid_state_t;

typedef struct {
    pid_gains_t gains;
    pid_state_t state;
} pid_controller_t;

/** PID axis indices */
typedef enum {
    PID_ROLL = 0,
    PID_PITCH = 1,
    PID_YAW = 2,
    PID_COUNT = 3
} pid_axis_t;
