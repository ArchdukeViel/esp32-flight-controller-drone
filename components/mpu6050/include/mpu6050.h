#pragma once

#include "esp_err.h"

/**
 * @file mpu6050.h
 * @brief MPU6050 WHO_AM_I detection (Prompt 3).
 *
 * This component only implements WHO_AM_I register read for identity confirmation.
 * No raw accel/gyro read, no scaled read, no calibration, no FIFO.
 * Uses ESP-IDF v6 I2C master driver (i2c_master_bus_handle_t) via i2c_bus component.
 */

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