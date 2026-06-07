#pragma once

#include <stdint.h>

/**
 * @file motor_mixer_types.h
 * @brief Motor mixer data structures for Quad X configuration.
 */

typedef struct {
    float motor[4];  // Normalized 0.0..1.0 (or -1..1 for bidirectional)
} motor_mixer_output_t;

/** Motor indices for Quad X:
 *   M0 (CW)  ---  M1 (CCW)
 *      \        /
 *       \      /
 *        \    /
 *         \  /
 *         /  \
 *        /    \
 *       /      \
 *      /        \
 *   M3 (CCW) --- M2 (CW)
 *
 * M0: front-right  (CW)
 * M1: front-left   (CCW)
 * M2: rear-right   (CW)
 * M3: rear-left    (CCW)
 */
typedef enum {
    MOTOR_FR = 0,  // Front-right, CW
    MOTOR_FL = 1,  // Front-left,  CCW
    MOTOR_RR = 2,  // Rear-right,  CW
    MOTOR_RL = 3,  // Rear-left,   CCW
} motor_index_t;
