#pragma once

#include "esp_err.h"
#include "estimator_types.h"
#include "mpu6050_types.h"
#include "bmp280_types.h"

/**
 * @file estimator.h
 * @brief Sensor fusion estimator: complementary filter for attitude, baro+accel for altitude.
 *
 * Complementary filter (attitude):
 *   - Accel gives absolute roll/pitch (noisy, drift-free)
 *   - Gyro gives rate integration (precise short-term, drifts)
 *   - Fuse: angle = alpha * (angle + gyro*dt) + (1-alpha) * accel_angle
 *   - Typical alpha = 0.98 (gyro weight), 0.02 (accel weight)
 *
 * Altitude fusion:
 *   - Baro gives absolute altitude (noisy, slow, drift-free)
 *   - Accel Z (world frame) integrated twice gives relative altitude (precise short-term, drifts)
 *   - Simple complementary: alt = alpha * (alt + vel*dt + 0.5*acc*dt^2) + (1-alpha) * baro_alt
 *   - Velocity also fused: vel = alpha * (vel + acc*dt) + (1-alpha) * (baro_alt - prev_baro_alt)/dt
 */

#ifdef __cplusplus
extern "C" {
#endif

// Complementary filter alpha (gyro weight). 0.98 = 98% gyro, 2% accel.
#define ESTIMATOR_CF_ALPHA 0.98f
#define ESTIMATOR_ALT_ALPHA 0.96f

/**
 * @brief Initialize estimator state.
 * @return ESP_OK on success
 */
esp_err_t estimator_init(void);

/**
 * @brief Update estimator with new sensor data.
 * Call at fixed rate (e.g., 1 kHz control loop).
 * @param mpu_scaled Scaled MPU6050 data (accel in m/s^2, gyro in rad/s)
 * @param bmp_compensated Compensated BMP280 data (pressure, temperature, altitude)
 * @param dt_s Time step in seconds since last update
 * @return ESP_OK on success
 */
esp_err_t estimator_update(const mpu6050_scaled_t* mpu_scaled,
                           const bmp280_compensated_t* bmp_compensated,
                           float dt_s);

/**
 * @brief Get current estimated state.
 * @param state Output state struct
 */
void estimator_get_state(estimator_state_t* state);

/**
 * @brief Reset estimator state (e.g., on arming).
 */
void estimator_reset(void);

/**
 * @brief Set complementary filter alpha (0.0-1.0, higher = more gyro trust).
 * @param alpha New alpha value
 */
void estimator_set_cf_alpha(float alpha);

/**
 * @brief Set altitude filter alpha (0.0-1.0, higher = more accel trust).
 * @param alpha New alpha value
 */
void estimator_set_alt_alpha(float alpha);

#ifdef __cplusplus
}
#endif