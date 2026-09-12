file(GLOB_RECURSE SOURCES "*.cpp")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES swarm_hal esp_common log
             esp_driver_gpio esp_driver_spi esp_driver_ledc esp_driver_uart
)

