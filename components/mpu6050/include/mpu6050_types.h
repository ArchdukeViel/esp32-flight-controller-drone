#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file mpu6050_types.h
 * @brief MPU6050 data structures for raw/scaled reads, calibration, and error counting.
 */

/** Raw 14-byte burst read: accel X/Y/Z, temp, gyro X/Y/Z (big-endian int16_t) */
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t t;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_raw_t;

/** Scaled sensor data in SI units */
typedef struct {
    float ax;  // m/s^2
    float ay;  // m/s^2
    float az;  // m/s^2
    float t;   // degrees C
    float gx;  // rad/s
    float gy;  // rad/s
    float gz;  // rad/s
} mpu6050_scaled_t;

/** Error counters for diagnostics */
typedef struct {
    uint32_t init_errors;
    uint32_t read_errors;
    uint32_t crc_errors;
    uint32_t last_error_tick;
} mpu6050_error_count_t;

/** Calibration offsets in raw LSB */
typedef struct {
    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
    bool    valid;
} mpu6050_calibration_t;
