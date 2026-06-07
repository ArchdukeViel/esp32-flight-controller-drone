idf_component_register(SRCS "src/motor_output.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES motor_mixer esp_driver_mcpwm esp_driver_gpio)