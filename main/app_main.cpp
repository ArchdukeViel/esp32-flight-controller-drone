#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "board_config.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "estimator.h"
#include "pid.h"
#include "motor_mixer.h"
#include "motor_output.h"
#include "safety.h"

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
    ESP_LOGI(TAG, "Phase: Prompt 11 - Safety State Machine");
    ESP_LOGI(TAG, "========================================");

    // Safety message
    ESP_LOGW(TAG, "SAFETY: Safety State Machine is NOW ACTIVE.");
    ESP_LOGW(TAG, "Motor output gated by safety state (BOOT->DISARMED->PRE_ARM_CHECK->ARMED).");
    ESP_LOGW(TAG, "ESC pins (GPIO 18,19,23,25) forced to 1000 us idle in non-ARMED states.");
    ESP_LOGW(TAG, "Do NOT connect ESCs or propellers unless intentionally testing.");
    ESP_LOGW(TAG, "WiFi, receiver still DISABLED.");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C master bus
    ESP_ERROR_CHECK(i2c_bus_init());

    // Scan I2C bus for devices
    ESP_ERROR_CHECK(i2c_bus_scan());

    // Detect MPU6050 via WHO_AM_I register read
    esp_err_t mpu_ret = mpu6050_detect();
    if (mpu_ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not detected, halting");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "MPU6050 confirmed at 0x%02X", MPU6050_I2C_ADDR);

    // Load MPU6050 calibration from NVS
    mpu6050_load_calibration();
    mpu6050_calibration_t mpu_cal;
    mpu6050_get_calibration(&mpu_cal);

    if (!mpu_cal.valid) {
        ESP_LOGW(TAG, "No valid MPU6050 calibration found. Running calibration now...");
        ESP_LOGW(TAG, "PLACE DEVICE ON LEVEL SURFACE AND KEEP STILL!");
        vTaskDelay(pdMS_TO_TICKS(3000));
        mpu6050_calibrate(&mpu_cal);
        if (mpu_cal.valid) {
            ESP_LOGI(TAG, "MPU6050 calibration complete and saved to NVS");
        } else {
            ESP_LOGE(TAG, "MPU6050 calibration failed, continuing without offsets");
        }
    } else {
        ESP_LOGI(TAG, "Loaded MPU6050 calibration: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d",
                 mpu_cal.ax_offset, mpu_cal.ay_offset, mpu_cal.az_offset,
                 mpu_cal.gx_offset, mpu_cal.gy_offset, mpu_cal.gz_offset);
    }

    // Detect and initialize BMP280
    uint8_t bmp_addr = 0;
    esp_err_t bmp_ret = bmp280_detect(&bmp_addr);
    bool bmp_ok = false;
    if (bmp_ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP280 confirmed at 0x%02X", bmp_addr);
        ESP_ERROR_CHECK(bmp280_init());
        bmp280_calib_t bmp_cal;
        bmp280_get_calibration(&bmp_cal);
        ESP_LOGI(TAG, "BMP280 calibration loaded: dig_T1=%u dig_P1=%u", bmp_cal.dig_T1, bmp_cal.dig_P1);
        bmp_ok = true;
    } else {
        ESP_LOGW(TAG, "BMP280 not detected at 0x76 or 0x77");
    }

    // Initialize estimator
    ESP_ERROR_CHECK(estimator_init());

    // Initialize PID
    ESP_ERROR_CHECK(pid_init());

    // Initialize motor mixer
    ESP_ERROR_CHECK(motor_mixer_init());

    // Initialize motor output (MCPWM)
    ESP_ERROR_CHECK(motor_output_init(NULL));

    // Initialize safety state machine
    ESP_ERROR_CHECK(safety_init());

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

    uint32_t loop_count = 0;
    mpu6050_error_count_t mpu_errs = {};
    bmp280_error_count_t bmp_errs = {};
    estimator_state_t est_state = {};
    float pid_output[3] = {};
    motor_mixer_output_t motor_mixer_out = {};
    motor_output_state_t motor_output_state = {};
    float target_throttle = 0.0f;  // Disarmed
    estimator_attitude_t target_attitude = {};  // Level target

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t loop_period = pdMS_TO_TICKS(10);  // 100 Hz control loop

    while (1) {
        // Toggle LED for liveness (every 10 loops = 10 Hz blink)
        if (loop_count % 10 == 0) {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BOARD_ONBOARD_LED_GPIO_NUM, 1));
        } else if (loop_count % 10 == 1) {
            ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BOARD_ONBOARD_LED_GPIO_NUM, 0));
        }

        // Read MPU6050
        mpu6050_raw_t raw = {};
        mpu6050_scaled_t scaled = {};
        esp_err_t raw_ret = mpu6050_read_raw(&raw);
        esp_err_t scaled_ret = mpu6050_read_scaled(&scaled);

        // Read BMP280
        bmp280_compensated_t bmp = {};
        esp_err_t bmp_read = ESP_ERR_INVALID_STATE;
        if (bmp_ok) {
            bmp_read = bmp280_read_compensated(&bmp);
        }

        // Update estimator at 100 Hz (dt = 0.01s)
        if (scaled_ret == ESP_OK) {
            estimator_update(&scaled, bmp_ok ? &bmp : NULL, 0.01f);
        }

        // Update safety state machine
        safety_update();

        // Get current estimated attitude
        estimator_get_state(&est_state);

        // Compute PID (target = level, current = estimated)
        if (est_state.initialized && scaled_ret == ESP_OK) {
            pid_compute(&target_attitude, &est_state.attitude, 0.01f, pid_output);

            // Mix PID outputs with throttle
            motor_mixer_mix(pid_output, target_throttle, &motor_mixer_out);

            // Send to motor output (MCPWM) - only if safety allows
            if (safety_is_motor_output_allowed()) {
                motor_output_set(&motor_mixer_out);
            }
        }

        // Log at 10 Hz (every 10 loops)
        if (loop_count % 10 == 0) {
            if (raw_ret == ESP_OK && scaled_ret == ESP_OK) {
                ESP_LOGI(TAG, "MPU RAW  (#%lu): ax=%6d ay=%6d az=%6d  t=%6d  gx=%6d gy=%6d gz=%6d",
                         loop_count, raw.ax, raw.ay, raw.az, raw.t, raw.gx, raw.gy, raw.gz);
                ESP_LOGI(TAG, "MPU SCALED: ax=%7.3f ay=%7.3f az=%7.3f m/s^2  t=%5.2f C  gx=%7.3f gy=%7.3f gz=%7.3f rad/s",
                         scaled.ax, scaled.ay, scaled.az, scaled.t,
                         scaled.gx, scaled.gy, scaled.gz);
            } else {
                ESP_LOGW(TAG, "MPU6050 read failed: raw=%s scaled=%s",
                         esp_err_to_name(raw_ret), esp_err_to_name(scaled_ret));
            }

            if (bmp_ok) {
                if (bmp_read == ESP_OK) {
                    ESP_LOGI(TAG, "BMP280: T=%6.2f C  P=%10.0f Pa  Alt=%7.2f m",
                             bmp.temperature, bmp.pressure, bmp.altitude);
                } else {
                    ESP_LOGW(TAG, "BMP280 read failed: %s", esp_err_to_name(bmp_read));
                }
            }

            // Log estimator state
            if (est_state.initialized) {
                ESP_LOGI(TAG, "EST: roll=%7.3f pitch=%7.3f yaw=%7.3f deg  alt=%7.2f m  vel=%6.2f m/s  baro=%7.2f m",
                         est_state.attitude.roll * 180.0f / 3.14159f,
                         est_state.attitude.pitch * 180.0f / 3.14159f,
                         est_state.attitude.yaw * 180.0f / 3.14159f,
                         est_state.altitude.altitude,
                         est_state.altitude.vertical_vel,
                         est_state.altitude.baro_alt);
            }

            // Log PID + motor mixer + motor output
            if (est_state.initialized) {
                motor_output_get_state(&motor_output_state);
                uint16_t pwm[4];
                for (int i = 0; i < 4; i++) pwm[i] = motor_mixer_to_pwm_us(motor_mixer_out.motor[i]);
                ESP_LOGI(TAG, "PID:   roll=%7.3f pitch=%7.3f yaw=%7.3f  (norm)",
                         pid_output[0], pid_output[1], pid_output[2]);
                ESP_LOGI(TAG, "MIX:   FR=%5.3f FL=%5.3f RR=%5.3f RL=%5.3f  PWM: %u %u %u %u us",
                         motor_mixer_out.motor[0], motor_mixer_out.motor[1],
                         motor_mixer_out.motor[2], motor_mixer_out.motor[3],
                         pwm[0], pwm[1], pwm[2], pwm[3]);
                ESP_LOGI(TAG, "MCPWM: ARMED=%s  pulse_us: %u %u %u %u",
                         motor_output_state.armed ? "YES" : "NO",
                         motor_output_state.pulse_us[0], motor_output_state.pulse_us[1],
                         motor_output_state.pulse_us[2], motor_output_state.pulse_us[3]);
                
                // Log safety state
                safety_status_t safety_status = {};
                safety_get_status(&safety_status);
                const char* state_names[] = {"BOOT", "DISARMED", "PRE_ARM_CHECK", "ARMED", "FAILSAFE", "ERROR"};
                ESP_LOGI(TAG, "SAFETY: state=%s  can_arm=%s",
                         state_names[safety_status.state],
                         safety_status.can_arm ? "YES" : "NO");
            }
        }

        // Log error counts periodically (every 100 loops = 1 Hz)
        if (loop_count % 100 == 0 && loop_count > 0) {
            mpu6050_get_error_counts(&mpu_errs);
            bmp280_get_error_counts(&bmp_errs);
            if (mpu_errs.read_errors > 0 || mpu_errs.init_errors > 0 ||
                bmp_errs.read_errors > 0 || bmp_errs.init_errors > 0) {
                ESP_LOGW(TAG, "Errors: MPU(init=%lu read=%lu) BMP(init=%lu read=%lu)",
                         mpu_errs.init_errors, mpu_errs.read_errors,
                         bmp_errs.init_errors, bmp_errs.read_errors);
            }
        }

        loop_count++;
        vTaskDelayUntil(&last_wake, loop_period);
    }
#else
    ESP_LOGI(TAG, "No safe onboard LED defined. Idle loop.");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
#endif
}