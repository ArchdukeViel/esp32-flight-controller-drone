#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @file i2c_bus.h
 * @brief I2C master bus initialization and scanning using ESP-IDF v6 handle-based API.
 *
 * This component wraps the ESP-IDF v6 I2C master driver (i2c_master_bus_handle_t).
 * It does NOT use the legacy driver (i2c_param_config, i2c_driver_install, etc.).
 */

/**
 * @brief Initialize the I2C master bus.
 *
 * Uses board_config constants for SDA (GPIO21), SCL (GPIO22), and frequency (400 kHz).
 *
 * @return ESP_OK on success, or an error code if bus creation fails.
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Scan I2C bus for devices.
 *
 * Probes addresses 0x03 through 0x77 using i2c_master_probe.
 * Logs found devices and annotates known sensor addresses.
 * Does NOT read WHO_AM_I or confirm sensor identity.
 *
 * @return ESP_OK on success (even if 0 devices found), or error code on bus failure.
 */
esp_err_t i2c_bus_scan(void);

/**
 * @brief Get the I2C master bus handle.
 *
 * @return The i2c_master_bus_handle_t, or NULL if not initialized.
 */
i2c_master_bus_handle_t i2c_bus_get_handle(void);