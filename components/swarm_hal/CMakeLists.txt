# micro-ROS only builds for the ESP target; the host (linux) tests never pull
# motion_hal.h, so keep it out of the linux build closure
set(reqs "")
if(NOT "${IDF_TARGET}" STREQUAL "linux")
    list(APPEND reqs micro_ros_espidf_component)
endif()

idf_component_register(
    INCLUDE_DIRS "."
    REQUIRES ${reqs}
)
