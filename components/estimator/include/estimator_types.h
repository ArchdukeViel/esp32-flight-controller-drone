#pragma once

#include <stdint.h>

/**
 * @file estimator_types.h
 * @brief Estimator data structures for attitude and altitude estimation.
 */

typedef struct {
    float roll;   // radians, positive right wing down
    float pitch;  // radians, positive nose up
    float yaw;    // radians, positive clockwise from top (not estimated by CF)
} estimator_attitude_t;

typedef struct {
    float altitude;      // meters, fused baro + accel Z
    float vertical_vel;  // m/s, derived from altitude delta
    float baro_alt;      // meters, raw barometric altitude
    float accel_z;       // m/s^2, world-frame Z acceleration (gravity removed)
} estimator_altitude_t;

typedef struct {
    estimator_attitude_t attitude;
    estimator_altitude_t altitude;
    uint32_t timestamp_us;
    bool initialized;
} estimator_state_t;
