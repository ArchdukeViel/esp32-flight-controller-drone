idf_component_register(
    SRCS "src/safety.cpp"
    INCLUDE_DIRS "include"
    REQUIRES motor_output pid estimator mpu6050
)
