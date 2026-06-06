#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "board_config.h"
#include "i2c_bus.h"

static const char* TAG = "main";

extern "C" void app_main(void) {
    // Boot banner
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32 Flight Controller Drone");
    ESP_LOGI(TAG, "Project: esp32_flight_controller_drone");
    ESP_LOGI(TAG, "Target: ESP32 DevKit V1");
    ESP_LOGI(TAG, "Sensors: MPU6050, BMP280");
    ESP_LOGI(TAG, "ESCs: 30A, Motors: 2212 2200KV");
    ESP_LOGI(TAG, "Framework: ESP-IDF v6.0.1, C++17");
    ESP_LOGI(TAG, "Phase: Prompt 2 - I2C bus scan");
    ESP_LOGI(TAG, "========================================");

    // Safety message
    ESP_LOGW(TAG, "SAFETY: No motor output exists in this phase.");
    ESP_LOGW(TAG, "MCPWM, ESC pins, WiFi, PID, receiver, safety state machine are DISABLED.");
    ESP_LOGW(TAG, "Do not connect ESCs or propellers.");

    // Initialize I2C master bus
    ESP_ERROR_CHECK(i2c_bus_init());

    // Scan I2C bus for devices
    ESP_ERROR_CHECK(i2c_bus_scan());

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

    while (1) {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BOARD_ONBOARD_LED_GPIO_NUM, 1));
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BOARD_ONBOARD_LED_GPIO_NUM, 0));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#else
    ESP_LOGI(TAG, "No safe onboard LED defined. Idle loop.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}