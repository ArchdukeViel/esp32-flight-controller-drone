#include "bmp280.h"
#include "board_config.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

static const char* TAG = "bmp280";

#define BMP280_READ_TIMEOUT_MS 10

// Static state
static bmp280_error_count_t s_error_counts = {0};
static bmp280_calib_t s_calib = {0};
static uint8_t s_addr = 0;
static bool s_initialized = false;
static float s_sea_level = BMP280_SEA_LEVEL_PRESSURE;

static inline void inc_read_err(void) {
    s_error_counts.read_errors++;
    s_error_counts.last_error_tick = xTaskGetTickCount();
}
static inline void inc_init_err(void) {
    s_error_counts.init_errors++;
    s_error_counts.last_error_tick = xTaskGetTickCount();
}

static i2c_master_dev_handle_t bmp280_create_device(uint8_t addr)
{
    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (!bus) return NULL;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = false },
    };

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) { inc_init_err(); return NULL; }
    return dev;
}

esp_err_t bmp280_detect(uint8_t* detected_addr)
{
    uint8_t addrs[2] = { BMP280_I2C_ADDR_PRIMARY, BMP280_I2C_ADDR_SECONDARY };

    for (int i = 0; i < 2; i++) {
        i2c_master_dev_handle_t dev = bmp280_create_device(addrs[i]);
        if (!dev) continue;

        uint8_t reg = BMP280_REG_CHIP_ID;
        uint8_t chip_id = 0;
        esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1, &chip_id, 1, pdMS_TO_TICKS(50));
        i2c_master_bus_rm_device(dev);

        if (ret == ESP_OK && chip_id == BMP280_CHIP_ID_EXPECTED) {
            s_addr = addrs[i];
            if (detected_addr) *detected_addr = s_addr;
            ESP_LOGI(TAG, "BMP280 detected at 0x%02X, chip_id=0x%02X", s_addr, chip_id);
            return ESP_OK;
        }
    }
    inc_read_err();
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t bmp280_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(BMP280_READ_TIMEOUT_MS));
}

static esp_err_t bmp280_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, 2, pdMS_TO_TICKS(BMP280_READ_TIMEOUT_MS));
}

esp_err_t bmp280_init(void)
{
    if (s_addr == 0) { inc_init_err(); return ESP_ERR_INVALID_STATE; }

    i2c_master_dev_handle_t dev = bmp280_create_device(s_addr);
    if (!dev) return ESP_ERR_INVALID_STATE;

    // Soft reset
    esp_err_t ret = bmp280_write_reg(dev, BMP280_REG_RESET, BMP280_RESET_CMD);
    if (ret != ESP_OK) { i2c_master_bus_rm_device(dev); inc_init_err(); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read calibration (24 bytes from 0x88)
    uint8_t calib_data[24];
    ret = bmp280_read_reg(dev, BMP280_REG_CALIB_START, calib_data, 24);
    if (ret != ESP_OK) { i2c_master_bus_rm_device(dev); inc_read_err(); return ret; }

    s_calib.dig_T1 = (uint16_t)(calib_data[1] << 8 | calib_data[0]);
    s_calib.dig_T2 = (int16_t)(calib_data[3] << 8 | calib_data[2]);
    s_calib.dig_T3 = (int16_t)(calib_data[5] << 8 | calib_data[4]);
    s_calib.dig_P1 = (uint16_t)(calib_data[7] << 8 | calib_data[6]);
    s_calib.dig_P2 = (int16_t)(calib_data[9] << 8 | calib_data[8]);
    s_calib.dig_P3 = (int16_t)(calib_data[11] << 8 | calib_data[10]);
    s_calib.dig_P4 = (int16_t)(calib_data[13] << 8 | calib_data[12]);
    s_calib.dig_P5 = (int16_t)(calib_data[15] << 8 | calib_data[14]);
    s_calib.dig_P6 = (int16_t)(calib_data[17] << 8 | calib_data[16]);
    s_calib.dig_P7 = (int16_t)(calib_data[19] << 8 | calib_data[18]);
    s_calib.dig_P8 = (int16_t)(calib_data[21] << 8 | calib_data[20]);
    s_calib.dig_P9 = (int16_t)(calib_data[23] << 8 | calib_data[22]);

    // Configure: ctrl_meas (osrs_t, osrs_p, mode)
    uint8_t ctrl_meas = (BMP280_DEFAULT_OSRS_T << 5) | (BMP280_DEFAULT_OSRS_P << 2) | BMP280_DEFAULT_MODE;
    ret = bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS, ctrl_meas);
    if (ret != ESP_OK) { i2c_master_bus_rm_device(dev); inc_init_err(); return ret; }

    // Configure: config (standby, filter)
    uint8_t config = (BMP280_DEFAULT_STANDBY << 5) | (BMP280_DEFAULT_FILTER << 2);
    ret = bmp280_write_reg(dev, BMP280_REG_CONFIG, config);
    if (ret != ESP_OK) { i2c_master_bus_rm_device(dev); inc_init_err(); return ret; }

    i2c_master_bus_rm_device(dev);
    s_initialized = true;
    ESP_LOGI(TAG, "BMP280 initialized at 0x%02X", s_addr);
    return ESP_OK;
}

// Bosch compensation formulas (integer math)
static int32_t t_fine;

static float compensate_temp(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * ((int32_t)s_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) * ((int32_t)s_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

static float compensate_press(int32_t adc_P)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 += ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 += (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)s_calib.dig_P1)) >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);
    return (float)p / 256.0f;
}

esp_err_t bmp280_read_compensated(bmp280_compensated_t* comp)
{
    if (!comp) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    i2c_master_dev_handle_t dev = bmp280_create_device(s_addr);
    if (!dev) return ESP_ERR_INVALID_STATE;

    uint8_t data[6];
    esp_err_t ret = bmp280_read_reg(dev, BMP280_REG_PRESS_MSB, data, 6);
    i2c_master_bus_rm_device(dev);

    if (ret != ESP_OK) { inc_read_err(); return ret; }

    uint32_t press_raw = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    uint32_t temp_raw  = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);

    int32_t t_fine_local = (int32_t)compensate_temp((int32_t)temp_raw);
    float temperature = t_fine_local / 100.0f;
    float pressure = compensate_press((int32_t)press_raw);
    float altitude = 44330.0f * (1.0f - powf(pressure / s_sea_level, 0.1903f));

    comp->temperature = temperature;
    comp->pressure = pressure;
    comp->altitude = altitude;
    return ESP_OK;
}

esp_err_t bmp280_read_raw(bmp280_raw_t* raw)
{
    if (!raw) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    i2c_master_dev_handle_t dev = bmp280_create_device(s_addr);
    if (!dev) return ESP_ERR_INVALID_STATE;

    uint8_t data[6];
    esp_err_t ret = bmp280_read_reg(dev, BMP280_REG_PRESS_MSB, data, 6);
    i2c_master_bus_rm_device(dev);

    if (ret != ESP_OK) { inc_read_err(); return ret; }

    raw->press_raw = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
    raw->temp_raw  = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
    return ESP_OK;
}

void bmp280_get_calibration(bmp280_calib_t* calib)
{
    if (calib) *calib = s_calib;
}

void bmp280_get_error_counts(bmp280_error_count_t* counts)
{
    if (counts) *counts = s_error_counts;
}

void bmp280_reset_error_counts(void)
{
    memset(&s_error_counts, 0, sizeof(s_error_counts));
}

esp_err_t bmp280_soft_reset(void)
{
    if (s_addr == 0) return ESP_ERR_INVALID_STATE;
    i2c_master_dev_handle_t dev = bmp280_create_device(s_addr);
    if (!dev) return ESP_ERR_INVALID_STATE;
    esp_err_t ret = bmp280_write_reg(dev, BMP280_REG_RESET, BMP280_RESET_CMD);
    i2c_master_bus_rm_device(dev);
    return ret;
}

void bmp280_set_sea_level_pressure(float sea_level_hpa)
{
    s_sea_level = sea_level_hpa * 100.0f;
}