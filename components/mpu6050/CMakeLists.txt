idf_component_register(SRCS "src/mpu6050.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES board_config i2c_bus esp_driver_i2c
                       PRIV_REQUIRES nvs_flash)
