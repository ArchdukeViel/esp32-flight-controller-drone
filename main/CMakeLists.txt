idf_component_register(SRCS "app_main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES board_config esp_driver_gpio i2c_bus mpu6050 bmp280 estimator pid motor_mixer motor_output nvs_flash)
