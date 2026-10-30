# the driver only touches hardware through the I2C/GPIO concepts, so the mock
# below builds and runs on the linux host target
idf_component_register(
    SRCS "mpu6050_test.cpp"
    INCLUDE_DIRS "."
    REQUIRES unity mpu6050 swarm_hal
)
