#pragma once

#include "esp_err.h"
#include "bmp280_types.h"

/**
 * @file bmp280.h
 * @brief BMP280 driver: chip ID read + compensated pressure/temperature/altitude (Prompt 6).
 *
 * - Chip ID detection at 0xD0 (expect 0x58)
 * - Read calibration coefficients from 0x88-0xA1
 * - Read raw ADC from 0xF7-0xFC
 * - Compensate using Bosch formulas (integer math, returns float SI units)
 * - Altitude calculation using barometric formula
 * - Error counting for diagnostics
 * Uses ESP-IDF v6 I2C master driver via i2c_bus component.
 */

/** I2C register addresses */
#define BMP280_REG_CHIP_ID       0xD0
#define BMP280_REG_RESET         0xE0
#define BMP280_REG_CTRL_MEAS     0xF4
#define BMP280_REG_CONFIG        0xF5
#define BMP280_REG_PRESS_MSB     0xF7
#define BMP280_REG_CALIB_START   0x88

/** Reset command */
#define BMP280_RESET_CMD         0xB6

/** Default oversampling settings */
#define BMP280_OSRS_TEMP_1X      0x01
#define BMP280_OSRS_TEMP_2X      0x02
#define BMP280_OSRS_TEMP_4X      0x03
#define BMP280_OSRS_TEMP_8X      0x04
#define BMP280_OSRS_TEMP_16X     0x05

#define BMP280_OSRS_PRESS_1X     0x01
#define BMP280_OSRS_PRESS_2X     0x02
#define BMP280_OSRS_PRESS_4X     0x03
#define BMP280_OSRS_PRESS_8X     0x04
#define BMP280_OSRS_PRESS_16X    0x05

#define BMP280_MODE_SLEEP        0x00
#define BMP280_MODE_FORCED       0x01
#define BMP280_MODE_NORMAL       0x03

#define BMP280_STANDBY_0_5MS     0x00
#define BMP280_STANDBY_62_5MS    0x01
#define BMP280_STANDBY_125MS     0x02
#define BMP280_STANDBY_250MS     0x03
#define BMP280_STANDBY_500MS     0x04
#define BMP280_STANDBY_1000MS    0x05
#define BMP280_STANDBY_2000MS    0x06
#define BMP280_STANDBY_4000MS    0x07

#define BMP280_FILTER_OFF        0x00
#define BMP280_FILTER_2X         0x01
#define BMP280_FILTER_4X         0x02
#define BMP280_FILTER_8X         0x03
#define BMP280_FILTER_16X        0x04

/** Default configuration for flights */
#define BMP280_DEFAULT_OSRS_T    BMP280_OSRS_TEMP_2X
#define BMP280_DEFAULT_OSRS_P    BMP280_OSRS_PRESS_16X
#define BMP280_DEFAULT_MODE      BMP280_MODE_NORMAL
#define BMP280_DEFAULT_STANDBY   BMP280_STANDBY_0_5MS
#define BMP280_DEFAULT_FILTER    BMP280_FILTER_16X

/** Standard sea level pressure for altitude calculation */
#define BMP280_SEA_LEVEL_PRESSURE 101325.0f

/**
 * @brief Detect BMP280 by reading chip ID register (0xD0).
 *
 * Tries both primary (0x76) and secondary (0x77) addresses.
 * Reads register 0xD0, compares with expected 0x58.
 *
 * @param[out] detected_addr Pointer to store detected I2C address (optional).
 *
 * @return ESP_OK if chip ID == 0x58 at either address.
 * @return ESP_ERR_NOT_FOUND if no BMP280 found at either address.
 * @return other esp_err_t on bus/communication errors.
 */
esp_err_t bmp280_detect(uint8_t* detected_addr);

/**
 * @brief Initialize BMP280 with default flight configuration.
 *
 * Performs: soft reset -> read calibration coefficients -> configure sensor.
 * Must be called after bmp280_detect() succeeds.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_STATE if I2C bus not initialized or device not added.
 * @return other esp_err_t on bus/communication errors.
 */
esp_err_t bmp280_init(void);

/**
 * @brief Read compensated temperature, pressure, and altitude.
 *
 * Triggers forced measurement (if in sleep mode), reads raw ADC,
 * applies compensation formulas, computes altitude.
 *
 * @param[out] comp Pointer to bmp280_compensated_t struct to fill.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if comp is NULL.
 * @return ESP_ERR_INVALID_STATE if not initialized.
 * @return ESP_ERR_TIMEOUT if read times out.
 * @return other esp_err_t on bus/communication errors.
 */
esp_err_t bmp280_read_compensated(bmp280_compensated_t* comp);

/**
 * @brief Read raw ADC values (for debugging/calibration).
 *
 * @param[out] raw Pointer to bmp280_raw_t struct to fill.
 *
 * @return ESP_OK on success, or error code.
 */
esp_err_t bmp280_read_raw(bmp280_raw_t* raw);

/**
 * @brief Get calibration coefficients (for debugging).
 *
 * @param[out] calib Pointer to bmp280_calib_t struct to fill.
 */
void bmp280_get_calibration(bmp280_calib_t* calib);

/**
 * @brief Get accumulated error counters.
 *
 * @param[out] counts Pointer to bmp280_error_count_t struct to fill.
 */
void bmp280_get_error_counts(bmp280_error_count_t* counts);

/**
 * @brief Reset error counters to zero.
 */
void bmp280_reset_error_counts(void);

/**
 * @brief Soft reset the BMP280.
 *
 * @return ESP_OK on success, or error code.
 */
esp_err_t bmp280_soft_reset(void);

/**
 * @brief Set sea level pressure for altitude calculation.
 *
 * Default is 101325 Pa. Call this if you know local QNH.
 *
 * @param sea_level_hpa Sea level pressure in hPa (e.g., 1013.25).
 */
void bmp280_set_sea_level_pressure(float sea_level_hpa);