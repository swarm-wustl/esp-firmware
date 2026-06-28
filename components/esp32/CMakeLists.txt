file(GLOB_RECURSE SOURCES "*.cpp")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES micro_ros_espidf_component swarm_hal swarm_logger
             esp_driver_gpio esp_driver_spi esp_driver_ledc esp_driver_uart
)

