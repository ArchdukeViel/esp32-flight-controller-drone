#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "board_config.h"
#include "i2c_bus.h"
#include "mpu6050.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    // Boot banner
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32 Flight Controller Drone");
    ESP_LOGI(TAG, "Project: esp32_flight_controller_drone");
    ESP_LOGI(TAG, "Target: ESP32 DevKit V1");
    ESP_LOGI(TAG, "Sensors: MPU6050, BMP280");
    ESP_LOGI(TAG, "ESCs: 30A, Motors: 2212 2200KV");
    ESP_LOGI(TAG, "Framework: ESP-IDF v6.0.1, C++17");
    ESP_LOGI(TAG, "Phase: Prompt 4 - MPU6050 raw accel/gyro burst read");
    ESP_LOGI(TAG, "========================================");

    // Safety message
    ESP_LOGW(TAG, "SAFETY: No motor output exists in this phase.");
    ESP_LOGW(TAG, "MCPWM, ESC pins, WiFi, PID, receiver, safety state machine are DISABLED.");
    ESP_LOGW(TAG, "Do not connect ESCs or propellers.");

    // Initialize I2C master bus
    ESP_ERROR_CHECK(i2c_bus_init());

    // Scan I2C bus for devices
    ESP_ERROR_CHECK(i2c_bus_scan());

    // Detect MPU6050 via WHO_AM_I register read
    esp_err_t mpu_ret = mpu6050_detect();
    if (mpu_ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 confirmed at 0x%02X", MPU6050_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "MPU6050 not detected or WHO_AM_I mismatch (will retry on next boot)");
    }

    // Optional LED blink if board_config defines a safe onboard LED pin
#ifdef BOARD_HAS_SAFE_ONBOARD_LED
    ESP_LOGI(TAG, "Blinking onboard LED on GPIO %d", BOARD_ONBOARD_LED_GPIO_NUM);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BOARD_ONBOARD_LED_GPIO_NUM);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    uint32_t read_count = 0;
    mpu6050_error_count_t error_counts = {0};

    while (1) {
        // Toggle LED for liveness
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BOARD_ONBOARD_LED_GPIO_NUM, 1));
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BOARD_ONBOARD_LED_GPIO_NUM, 0));

        // Read raw and scaled MPU6050 data
        mpu6050_raw_t raw = {0};
        mpu6050_scaled_t scaled = {0};

        esp_err_t raw_ret = mpu6050_read_raw(&raw);
        esp_err_t scaled_ret = mpu6050_read_scaled(&scaled);

        if (raw_ret == ESP_OK && scaled_ret == ESP_OK) {
            read_count++;
            if (read_count % 10 == 0) {
                // Log raw values every 10 reads (1 Hz at 10 Hz loop)
                ESP_LOGI(TAG, "RAW  (#%lu): ax=%6d ay=%6d az=%6d  t=%6d  gx=%6d gy=%6d gz=%6d",
                         read_count, raw.ax, raw.ay, raw.az, raw.t, raw.gx, raw.gy, raw.gz);

                // Log scaled values every 10 reads
                ESP_LOGI(TAG, "SCALED: ax=%7.3f ay=%7.3f az=%7.3f m/s^2  t=%5.2f C  gx=%7.3f gy=%7.3f gz=%7.3f rad/s",
                         scaled.ax, scaled.ay, scaled.az, scaled.t,
                         scaled.gx, scaled.gy, scaled.gz);
            }
        } else {
            ESP_LOGW(TAG, "MPU6050 read failed: raw=%s scaled=%s",
                     esp_err_to_name(raw_ret), esp_err_to_name(scaled_ret));
        }

        // Log error counts periodically
        if (read_count % 50 == 0) {
            mpu6050_get_error_counts(&error_counts);
            if (error_counts.read_errors > 0 || error_counts.init_errors > 0) {
                ESP_LOGW(TAG, "Error counts: init=%lu read=%lu last_tick=%lu",
                         error_counts.init_errors, error_counts.read_errors, error_counts.last_error_tick);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(900));  // ~1 Hz loop (100ms LED on + 900ms delay)
    }
#else
    ESP_LOGI(TAG, "No safe onboard LED defined. Idle loop.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}