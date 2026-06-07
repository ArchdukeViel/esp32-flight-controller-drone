#pragma once

#include <stdint.h>

/**
 * @file bmp280_types.h
 * @brief BMP280 data structures for raw and compensated sensor readings.
 */

 /** Raw ADC data from registers 0xF7-0xFC (6 bytes) */
typedef struct {
    uint32_t press_raw;  ///< Pressure ADC (20 bits, from 0xF7-0xF9)
    uint32_t temp_raw;   ///< Temperature ADC (20 bits, from 0xFA-0xFC)
} bmp280_raw_t;

/** Calibration coefficients from registers 0x88-0xA1 (24 bytes) */
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

/** Compensated sensor data in SI units */
typedef struct {
    float temperature;  ///< Temperature [degrees C]
    float pressure;     ///< Pressure [Pa]
    float altitude;     ///< Altitude [m] relative to sea level (101325 Pa)
} bmp280_compensated_t;

/** Error counters for diagnostics */
typedef struct {
    uint32_t init_errors;
    uint32_t read_errors;
    uint32_t crc_errors;
    uint32_t last_error_tick;
} bmp280_error_count_t;
