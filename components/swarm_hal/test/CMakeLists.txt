# pure kinematics: no IDF, no micro-ROS, so these run on the linux host target
idf_component_register(
    SRCS "drive_test.cpp" "resources_test.cpp"
    INCLUDE_DIRS "."
    REQUIRES unity swarm_hal
)
