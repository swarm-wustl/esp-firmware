# the kinematics core is pure value types -- no IDF, no micro-ROS -- so it
# builds for the linux host target and its tests run without a board
idf_component_register(
    INCLUDE_DIRS "."
)
