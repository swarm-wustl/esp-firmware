# Written with Claude
idf_component_register(
    SRCS "ros.cpp"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_common log micro_ros_espidf_component swarm_consumer
             swarm_queue
)
