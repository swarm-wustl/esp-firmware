# pure kinematics: no IDF, no micro-ROS, so these run on the linux host target
idf_component_register(
    SRCS "drive_test.cpp" "forward_test.cpp" "motor_test.cpp" "odometry_test.cpp" "pid_test.cpp" "resources_test.cpp"
    INCLUDE_DIRS "."
    REQUIRES unity swarm_hal
)
