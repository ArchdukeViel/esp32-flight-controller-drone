#include "i2c_bus.h"
#include "board_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus_handle = NULL;

esp_err_t i2c_bus_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C master bus");
    ESP_LOGI(TAG, "  SDA GPIO: %d", I2C_MASTER_SDA_GPIO);
    ESP_LOGI(TAG, "  SCL GPIO: %d", I2C_MASTER_SCL_GPIO);
    ESP_LOGI(TAG, "  Frequency: %d Hz", I2C_MASTER_FREQ_HZ);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,  // let the driver pick
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_GPIO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
            .allow_pd = false,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C master bus initialized");
    } else {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_bus_scan(void)
{
    if (s_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized. Call i2c_bus_init() first.");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Scanning I2C bus for devices...");

    int found_count = 0;

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        esp_err_t ret = i2c_master_probe(s_bus_handle, addr, pdMS_TO_TICKS(50));
        if (ret == ESP_OK) {
            found_count++;
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);

            if (addr == 0x68) {
                ESP_LOGI(TAG, "  0x%02X -> possible MPU6050 (not confirmed)", addr);
            } else if (addr == 0x76) {
                ESP_LOGI(TAG, "  0x%02X -> possible BMP280 (not confirmed)", addr);
            } else if (addr == 0x77) {
                ESP_LOGI(TAG, "  0x%02X -> possible BMP280 (not confirmed)", addr);
            }
        }
    }

    if (found_count == 0) {
        ESP_LOGW(TAG, "I2C scan complete: %d device(s) found. Sensors may not be connected.", found_count);
    } else {
        ESP_LOGI(TAG, "I2C scan complete: %d device(s) found", found_count);
    }

    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get_handle(void)
{
    return s_bus_handle;
}