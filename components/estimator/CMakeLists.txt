idf_component_register(SRCS "src/estimator.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES mpu6050 bmp280 esp_driver_i2c)