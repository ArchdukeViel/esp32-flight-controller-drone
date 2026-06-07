#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file bmp280_types.h
 * @brief BMP280 data structures for raw/compensated reads, calibration, and error counting.
 */

/** Raw ADC values */
typedef struct {
    uint32_t press_raw;  // 20-bit raw pressure ADC
    uint32_t temp_raw;   // 20-bit raw temperature ADC
} bmp280_raw_t;

/** Compensated sensor data in SI units */
typedef struct {
    float temperature;  // degrees C
    float pressure;     // Pa
    float altitude;     // meters (barometric formula)
} bmp280_compensated_t;

/** BMP280 calibration coefficients (24 bytes from registers 0x88-0x9F) */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

/** Error counters for diagnostics */
typedef struct {
    uint32_t init_errors;
    uint32_t read_errors;
    uint32_t last_error_tick;
} bmp280_error_count_t;