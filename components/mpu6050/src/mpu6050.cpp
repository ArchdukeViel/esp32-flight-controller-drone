#include "mpu6050.h"
#include "board_config.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char* TAG = "mpu6050";

esp_err_t mpu6050_detect(void)
{
    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
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
        return ret;
    }

    // Read WHO_AM_I register (0x75)
    uint8_t reg = 0x75;
    uint8_t who_am_i = 0;

    ret = i2c_master_transmit_receive(dev_handle, &reg, 1, &who_am_i, 1, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev_handle);
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
        return ESP_ERR_NOT_FOUND;
    }
}