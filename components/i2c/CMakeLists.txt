# Written with Claude
idf_component_register(
    SRCS "i2c.cpp"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_common log swarm_queue swarm_hal driver
)
