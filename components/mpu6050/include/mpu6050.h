#pragma once

#include "esp_err.h"
#include "mpu6050_types.h"

/**
 * @file mpu6050.h
 * @brief MPU6050 driver: WHO_AM_I detection + raw/scaled accel/gyro reads + calibration (Prompt 5).
 *
 * - WHO_AM_I detection at 0x75 (Prompt 3)
 * - 14-byte burst read from register 0x3B (Prompt 4)
 * - Scaled conversion using configured accel/gyro ranges
 * - Error counting for diagnostics
 * - Calibration: offset calculation + NVS storage (Prompt 5)
 * Uses ESP-IDF v6 I2C master driver (i2c_master_bus_handle_t) via i2c_bus component.
 */

/** I2C register addresses */
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_ACCEL_XOUT_H    0x3B  // Start of 14-byte burst

/** Default sensor ranges (configurable via future API) */
#define MPU6050_ACCEL_FS_SEL        0     // 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define MPU6050_GYRO_FS_SEL         0     // 0=±250, 1=±500, 2=±1000, 3=±2000 dps

/** Scale factors for default ranges */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f  // ±2g
#define MPU6050_GYRO_LSB_PER_DPS    131.0f    // ±250 dps
#define MPU6050_TEMP_LSB_PER_DEGC   340.0f
#define MPU6050_TEMP_OFFSET_DEGC    36.53f

/** NVS namespace and keys for calibration storage */
#define MPU6050_NVS_NAMESPACE       "mpu6050_cal"
#define MPU6050_NVS_KEY_CAL         "offsets"

/** Calibration sample count (collect N samples, average) */
#define MPU6050_CAL_SAMPLES         1000
#define MPU6050_CAL_DELAY_MS        2       // ~500 Hz sampling during calibration

/**
 * @brief Initialize MPU6050: wake from sleep, configure accel/gyro ranges.
 *
 * Writes PWR_MGMT_1=0x00 (wake), GYRO_CONFIG=0x00 (±250dps), ACCEL_CONFIG=0x00 (±2g).
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_STATE if I2C bus not initialized.
 * @return other esp_err_t on bus/communication errors.
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Detect MPU6050 by reading WHO_AM_I register (0x75).
 *
 * Adds device at I2C address 0x68, reads register 0x75, compares with expected 0x68.
 *
 * @return ESP_OK if WHO_AM_I == 0x68.
 * @return ESP_ERR_NOT_FOUND if no device at 0x68 or WHO_AM_I mismatch.
 * @return other esp_err_t on bus/communication errors.
 */
esp_err_t mpu6050_detect(void);

/**
 * @brief Read raw 14-byte burst from registers 0x3B-0x48.
 *
 * Reads: accel X/Y/Z, temp, gyro X/Y/Z (big-endian, int16_t).
 * Uses 5 ms timeout for control-loop suitability.
 *
 * @param[out] raw Pointer to mpu6050_raw_t struct to fill.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if raw is NULL.
 * @return ESP_ERR_INVALID_STATE if I2C bus not initialized.
 * @return ESP_ERR_TIMEOUT if read times out.
 * @return other esp_err_t on bus/communication errors.
 */
esp_err_t mpu6050_read_raw(mpu6050_raw_t* raw);

/**
 * @brief Read scaled sensor data (converts raw to SI units, applies calibration offsets).
 *
 * Calls mpu6050_read_raw() then applies scale factors for default ranges:
 *   Accel: ±2g -> LSB/g = 16384 -> m/s^2 = (raw - offset) / 16384 * 9.80665
 *   Gyro:  ±250 dps -> LSB/dps = 131 -> rad/s = (raw - offset) / 131 * pi/180
 *   Temp:  degC = raw / 340 + 36.53
 *
 * @param[out] scaled Pointer to mpu6050_scaled_t struct to fill.
 *
 * @return ESP_OK on success, or error from mpu6050_read_raw().
 */
esp_err_t mpu6050_read_scaled(mpu6050_scaled_t* scaled);

/**
 * @brief Get accumulated error counters.
 *
 * @param[out] counts Pointer to mpu6050_error_count_t struct to fill.
 */
void mpu6050_get_error_counts(mpu6050_error_count_t* counts);

/**
 * @brief Reset error counters to zero.
 */
void mpu6050_reset_error_counts(void);

/**
 * @brief Load calibration offsets from NVS.
 *
 * If valid calibration exists in NVS, loads into internal state.
 * If not found or invalid, zeros offsets.
 *
 * @return ESP_OK on success (including not-found, offsets zeroed).
 * @return ESP_ERR_NOT_FOUND if NVS namespace/key doesn't exist.
 * @return other esp_err_t on NVS errors.
 */
esp_err_t mpu6050_load_calibration(void);

/**
 * @brief Save current calibration offsets to NVS.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_STATE if calibration not valid.
 * @return other esp_err_t on NVS errors.
 */
esp_err_t mpu6050_save_calibration(void);

/**
 * @brief Run calibration routine: collect samples, compute offsets, save to NVS.
 *
 * Device must be STATIONARY and LEVEL during calibration.
 * Collects MPU6050_CAL_SAMPLES samples at ~500 Hz.
 * Accel Z offset includes +1g (16384 LSB) so that level = 0 m/s^2.
 *
 * @param[out] cal Pointer to mpu6050_calibration_t to receive computed offsets.
 *
 * @return ESP_OK on success, calibration saved to NVS.
 * @return ESP_ERR_INVALID_ARG if cal is NULL.
 * @return ESP_ERR_TIMEOUT if read times out during sampling.
 * @return other esp_err_t on bus/communication/NVS errors.
 */
esp_err_t mpu6050_calibrate(mpu6050_calibration_t* cal);

/**
 * @brief Get current calibration offsets (from internal state).
 *
 * @param[out] cal Pointer to mpu6050_calibration_t to fill.
 */
void mpu6050_get_calibration(mpu6050_calibration_t* cal);

/**
 * @brief Set calibration offsets manually (bypasses NVS).
 *
 * Useful for testing or applying known-good offsets.
 *
 * @param[in] cal Pointer to mpu6050_calibration_t with new offsets.
 */
void mpu6050_set_calibration(const mpu6050_calibration_t* cal);