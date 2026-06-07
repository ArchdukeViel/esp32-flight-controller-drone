idf_component_register(SRCS "src/bmp280.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES board_config i2c_bus esp_driver_i2c nvs_flash)