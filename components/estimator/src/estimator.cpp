#include "estimator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char* TAG = "estimator";

static estimator_state_t s_state = {};
static float s_cf_alpha = ESTIMATOR_CF_ALPHA;
static float s_alt_alpha = ESTIMATOR_ALT_ALPHA;
static bool s_first_update = true;
static float s_prev_baro_alt = 0.0f;

esp_err_t estimator_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_first_update = true;
    s_prev_baro_alt = 0.0f;
    s_cf_alpha = ESTIMATOR_CF_ALPHA;
    s_alt_alpha = ESTIMATOR_ALT_ALPHA;
    ESP_LOGI(TAG, "Estimator initialized (cf_alpha=%.3f, alt_alpha=%.3f)", s_cf_alpha, s_alt_alpha);
    return ESP_OK;
}

static void accel_to_roll_pitch(const mpu6050_scaled_t* mpu, float* roll, float* pitch)
{
    // Accelerometer measures gravity vector in body frame.
    // When level: ax=0, ay=0, az=-g (or +g depending on convention).
    // Our MPU6050 scaled: Z up = +9.8 m/s^2 when level.
    // Roll = atan2(ay, az), Pitch = atan2(-ax, sqrt(ay^2 + az^2))
    float ax = mpu->ax;
    float ay = mpu->ay;
    float az = mpu->az;

    *roll  = atan2f(ay, az);
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
}

static void rotate_accel_to_world(const mpu6050_scaled_t* mpu, float roll, float pitch, float* world_z)
{
    // Rotate body-frame accel to world frame using current attitude estimate
    // World Z = -ax*sin(pitch) + ay*sin(roll)*cos(pitch) + az*cos(roll)*cos(pitch)
    // But simpler: since we just want gravity-compensated Z accel:
    // gravity in body frame = [0, 0, 9.80665] rotated by (roll, pitch)
    // body accel = measured - gravity_body
    // world Z = body_accel dot world_Z_axis

    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cr = cosf(roll);
    float sr = sinf(roll);

    // World Z axis in body frame: [ -sp, sr*cp, cr*cp ]
    // Body accel (gravity removed) = measured - [0, 0, 9.80665] in body frame
    // But easier: world_accel = R * body_accel, where body_accel includes gravity
    // So world Z = R_31*ax + R_32*ay + R_33*az - g
    // R_31 = -sp, R_32 = sr*cp, R_33 = cr*cp
    *world_z = -sp * mpu->ax + sr * cp * mpu->ay + cr * cp * mpu->az - 9.80665f;
}

esp_err_t estimator_update(const mpu6050_scaled_t* mpu_scaled,
                           const bmp280_compensated_t* bmp_compensated,
                           float dt_s)
{
    if (!mpu_scaled || dt_s <= 0.0f) return ESP_ERR_INVALID_ARG;

    // --- Attitude: Complementary Filter ---
    float accel_roll, accel_pitch;
    accel_to_roll_pitch(mpu_scaled, &accel_roll, &accel_pitch);

    if (s_first_update) {
        s_state.attitude.roll  = accel_roll;
        s_state.attitude.pitch = accel_pitch;
        s_state.attitude.yaw   = 0.0f;
        s_first_update = false;
    } else {
        // Gyro integration
        float gyro_roll  = s_state.attitude.roll  + mpu_scaled->gx * dt_s;
        float gyro_pitch = s_state.attitude.pitch + mpu_scaled->gy * dt_s;

        // Complementary filter
        s_state.attitude.roll  = s_cf_alpha * gyro_roll  + (1.0f - s_cf_alpha) * accel_roll;
        s_state.attitude.pitch = s_cf_alpha * gyro_pitch + (1.0f - s_cf_alpha) * accel_pitch;
        // Yaw not estimated (needs mag)
        s_state.attitude.yaw += mpu_scaled->gz * dt_s;  // Open-loop integration
    }

    // --- Altitude Fusion ---
    if (bmp_compensated) {
        float world_z_accel = 0.0f;
        rotate_accel_to_world(mpu_scaled, s_state.attitude.roll, s_state.attitude.pitch, &world_z_accel);

        if (s_first_update == false) { // already initialized above
            // Accel-based prediction
            float pred_vel = s_state.altitude.vertical_vel + world_z_accel * dt_s;
            float pred_alt = s_state.altitude.altitude + s_state.altitude.vertical_vel * dt_s
                           + 0.5f * world_z_accel * dt_s * dt_s;

            // Baro measurement
            float baro_alt = bmp_compensated->altitude;
            float baro_vel = (baro_alt - s_prev_baro_alt) / dt_s;
            s_prev_baro_alt = baro_alt;

            // Complementary fusion
            s_state.altitude.vertical_vel = s_alt_alpha * pred_vel + (1.0f - s_alt_alpha) * baro_vel;
            s_state.altitude.altitude     = s_alt_alpha * pred_alt + (1.0f - s_alt_alpha) * baro_alt;
            s_state.altitude.baro_alt     = baro_alt;
            s_state.altitude.accel_z      = world_z_accel;
        }
    }

    s_state.timestamp_us = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000);
    s_state.initialized = true;
    return ESP_OK;
}

void estimator_get_state(estimator_state_t* state)
{
    if (state) *state = s_state;
}

void estimator_reset(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_first_update = true;
    s_prev_baro_alt = 0.0f;
}

void estimator_set_cf_alpha(float alpha)
{
    if (alpha >= 0.0f && alpha <= 1.0f) s_cf_alpha = alpha;
}

void estimator_set_alt_alpha(float alpha)
{
    if (alpha >= 0.0f && alpha <= 1.0f) s_alt_alpha = alpha;
}