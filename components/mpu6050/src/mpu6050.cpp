#include "mpu6050.h"
#include "board_config.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "math.h"

static const char* TAG = "mpu6050";

#define MPU6050_READ_TIMEOUT_MS 5

// Static calibration offsets (internal state)
static mpu6050_calibration_t s_cal = {};
static bool s_cal_valid = false;

// Static error counters
static mpu6050_error_count_t s_error_counts = {};

static inline void increment_read_error(void) {
    s_error_counts.read_errors++;
    s_error_counts.last_error_tick = xTaskGetTickCount();
}

static inline void increment_init_error(void) {
    s_error_counts.init_errors++;
    s_error_counts.last_error_tick = xTaskGetTickCount();
}

esp_err_t mpu6050_detect(void)
{
    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        increment_init_error();
        return ESP_ERR_INVALID_STATE;
    }

    // Add device at MPU6050 I2C address
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s",
                 MPU6050_I2C_ADDR, esp_err_to_name(ret));
        increment_init_error();
        return ret;
    }

    // Read WHO_AM_I register (0x75)
    uint8_t reg = MPU6050_REG_WHO_AM_I;
    uint8_t who_am_i = 0;

    ret = i2c_master_transmit_receive(dev_handle, &reg, 1, &who_am_i, 1, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev_handle);
        increment_read_error();
        return ret;
    }

    // Compare with expected value
    if (who_am_i == MPU6050_WHO_AM_I_EXPECTED) {
        ESP_LOGI(TAG, "MPU6050 detected: WHO_AM_I = 0x%02X", who_am_i);
        i2c_master_bus_rm_device(dev_handle);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "MPU6050 WHO_AM_I mismatch: expected 0x%02X, got 0x%02X",
                 MPU6050_WHO_AM_I_EXPECTED, who_am_i);
        i2c_master_bus_rm_device(dev_handle);
        increment_read_error();
        return ESP_ERR_NOT_FOUND;
    }
}

esp_err_t mpu6050_read_raw(mpu6050_raw_t* raw)
{
    if (raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        increment_read_error();
        return ESP_ERR_INVALID_STATE;
    }

    // Add device at MPU6050 I2C address
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = false,
        },
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s",
                 MPU6050_I2C_ADDR, esp_err_to_name(ret));
        increment_init_error();
        return ret;
    }

    // Read 14 bytes starting from register 0x3B (ACCEL_XOUT_H)
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[14] = {};

    ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 14, pdMS_TO_TICKS(MPU6050_READ_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read 14-byte burst from 0x%02X: %s",
                 MPU6050_REG_ACCEL_XOUT_H, esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev_handle);
        increment_read_error();
        return ret;
    }

    // Parse big-endian int16_t values
    raw->ax = (int16_t)((data[0] << 8) | data[1]);
    raw->ay = (int16_t)((data[2] << 8) | data[3]);
    raw->az = (int16_t)((data[4] << 8) | data[5]);
    raw->t  = (int16_t)((data[6] << 8) | data[7]);
    raw->gx = (int16_t)((data[8] << 8) | data[9]);
    raw->gy = (int16_t)((data[10] << 8) | data[11]);
    raw->gz = (int16_t)((data[12] << 8) | data[13]);

    i2c_master_bus_rm_device(dev_handle);
    return ESP_OK;
}

esp_err_t mpu6050_read_scaled(mpu6050_scaled_t* scaled)
{
    if (scaled == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_raw_t raw;
    esp_err_t ret = mpu6050_read_raw(&raw);
    if (ret != ESP_OK) {
        return ret;
    }

    // Acceleration: raw LSB / 16384 * 9.80665 = m/s^2 (for ±2g)
    const float accel_scale = 9.80665f / MPU6050_ACCEL_LSB_PER_G;
    scaled->ax = raw.ax * accel_scale;
    scaled->ay = raw.ay * accel_scale;
    scaled->az = raw.az * accel_scale;

    // Temperature: raw / 340 + 36.53 = degC
    scaled->t = raw.t / MPU6050_TEMP_LSB_PER_DEGC + MPU6050_TEMP_OFFSET_DEGC;

    // Gyro: raw LSB / 131 * pi/180 = rad/s (for ±250 dps)
    const float gyro_scale = (3.14159265359f / 180.0f) / MPU6050_GYRO_LSB_PER_DPS;
    scaled->gx = raw.gx * gyro_scale;
    scaled->gy = raw.gy * gyro_scale;
    scaled->gz = raw.gz * gyro_scale;

    return ESP_OK;
}

void mpu6050_get_error_counts(mpu6050_error_count_t* counts)
{
    if (counts != NULL) {
        *counts = s_error_counts;
    }
}

void mpu6050_reset_error_counts(void)
{
    s_error_counts.init_errors = 0;
    s_error_counts.read_errors = 0;
    s_error_counts.crc_errors = 0;
    s_error_counts.last_error_tick = 0;
}

esp_err_t mpu6050_load_calibration(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MPU6050_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        s_cal_valid = false;
        memset(&s_cal, 0, sizeof(s_cal));
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "No calibration in NVS, using zeros");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    mpu6050_calibration_t cal;
    size_t len = sizeof(cal);
    ret = nvs_get_blob(handle, MPU6050_NVS_KEY_CAL, &cal, &len);
    if (ret != ESP_OK || len != sizeof(cal)) {
        nvs_close(handle);
        s_cal_valid = false;
        memset(&s_cal, 0, sizeof(s_cal));
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "No calibration blob in NVS, using zeros");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "NVS get failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_cal = cal;
    s_cal_valid = true;
    nvs_close(handle);
    ESP_LOGI(TAG, "Calibration loaded from NVS");
    return ESP_OK;
}

esp_err_t mpu6050_save_calibration(void)
{
    if (!s_cal_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MPU6050_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(handle, MPU6050_NVS_KEY_CAL, &s_cal, sizeof(s_cal));
    if (ret != ESP_OK) {
        nvs_close(handle);
        ESP_LOGE(TAG, "NVS set failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Calibration saved to NVS");
    return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_calibration_t* cal)
{
    if (cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < MPU6050_CAL_SAMPLES; i++) {
        mpu6050_raw_t raw;
        esp_err_t ret = mpu6050_read_raw(&raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration sample %d failed: %s", i, esp_err_to_name(ret));
            return ret;
        }
        sum_ax += raw.ax;
        sum_ay += raw.ay;
        sum_az += raw.az;
        sum_gx += raw.gx;
        sum_gy += raw.gy;
        sum_gz += raw.gz;
        vTaskDelay(pdMS_TO_TICKS(MPU6050_CAL_DELAY_MS));
    }

    // Average in raw LSB, cast to int16_t
    // Gyro offset = average (expecting zero motion)
    cal->gx_offset = (int16_t)(sum_gx / MPU6050_CAL_SAMPLES);
    cal->gy_offset = (int16_t)(sum_gy / MPU6050_CAL_SAMPLES);
    cal->gz_offset = (int16_t)(sum_gz / MPU6050_CAL_SAMPLES);

    // Accel: average minus 1g on Z (assuming level, Z-up)
    cal->ax_offset = (int16_t)(sum_ax / MPU6050_CAL_SAMPLES);
    cal->ay_offset = (int16_t)(sum_ay / MPU6050_CAL_SAMPLES);
    cal->az_offset = (int16_t)((sum_az / MPU6050_CAL_SAMPLES) - MPU6050_ACCEL_LSB_PER_G);

    cal->valid = true;

    // Store in internal state
    s_cal = *cal;
    s_cal_valid = true;

    // Save to NVS
    esp_err_t ret = mpu6050_save_calibration();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Calibration computed but NVS save failed");
    }

    ESP_LOGI(TAG, "Calibration complete: gyro_off=(%d,%d,%d) accel_off=(%d,%d,%d)",
             cal->gx_offset, cal->gy_offset, cal->gz_offset,
             cal->ax_offset, cal->ay_offset, cal->az_offset);
    return ESP_OK;
}

void mpu6050_get_calibration(mpu6050_calibration_t* cal)
{
    if (cal != NULL) {
        *cal = s_cal;
    }
}

void mpu6050_set_calibration(const mpu6050_calibration_t* cal)
{
    if (cal != NULL) {
        s_cal = *cal;
        s_cal_valid = true;
    }
}
