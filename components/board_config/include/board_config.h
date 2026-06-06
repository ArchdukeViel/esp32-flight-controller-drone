#pragma once

#include <stdint.h>
#include "driver/gpio.h"

/**
 * @file board_config.h
 * @brief Hardware constants for ESP32 DevKit V1 flight controller.
 * 
 * This file defines constants only. No initialization, no drivers.
 */

// Project identity
#define PROJECT_NAME "esp32_flight_controller_drone"
#define TARGET_MCU "ESP32 DevKit V1"

// I2C bus defaults (configured later, not initialized here)
#define I2C_MASTER_SDA_GPIO 21
#define I2C_MASTER_SCL_GPIO 22
#define I2C_MASTER_FREQ_HZ 400000

// MPU6050 constants
#define MPU6050_I2C_ADDR 0x68
#define MPU6050_WHO_AM_I_EXPECTED 0x68

// BMP280 constants
#define BMP280_I2C_ADDR_PRIMARY 0x76
#define BMP280_I2C_ADDR_SECONDARY 0x77
#define BMP280_CHIP_ID_EXPECTED 0x58

// ESC / MCPWM constants (placeholders, not used in this phase)
#define ESC_PWM_FREQ_HZ 50
#define ESC_PULSE_MIN_US 1000
#define ESC_PULSE_MAX_US 2000
#define ESC_PULSE_DISARMED_US 1000

// ESC pins (placeholders, do not use)
#define ESC_M1_GPIO 18
#define ESC_M2_GPIO 19
#define ESC_M3_GPIO 23
#define ESC_M4_GPIO 25

// Receiver pins (placeholders, do not use)
#define RC_CH1_GPIO 34
#define RC_CH2_GPIO 35
#define RC_CH3_GPIO 32
#define RC_CH4_GPIO 33
#define RC_CH5_GPIO 26
#define RC_CH6_GPIO 27

// Control loop timing
#define CONTROL_LOOP_FREQ_HZ 250
#define CONTROL_LOOP_PERIOD_US 4000

// Onboard LED (optional, only if safe to use)
// ESP32 DevKit V1 typically has blue LED on GPIO2
// Comment out BOARD_HAS_SAFE_ONBOARD_LED to disable LED blink
#define BOARD_HAS_SAFE_ONBOARD_LED
#define BOARD_ONBOARD_LED_GPIO (gpio_num_t)2

// Safety constants
#define THROTTLE_MIN_NORMALIZED 0.0f
#define THROTTLE_MAX_NORMALIZED 1.0f
#define THROTTLE_IDLE_NORMALIZED 0.05f

// Motor numbering convention (Quad X)
/*
         FRONT
    M1 (CCW)     M2 (CW)
 front-left   front-right
      \        /
       \      /
        \    /
        /    \
       /      \
      /        \
 rear-left   rear-right
    M4 (CW)      M3 (CCW)
         REAR
*/

// Version info
#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 1
#define FIRMWARE_VERSION_PATCH 0