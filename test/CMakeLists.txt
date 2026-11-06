# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../components")


# Set the components to include the tests for.
# This can be overridden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx" ..
#
set(TEST_COMPONENTS "dwm;encoders;mpu6050;swarm_hal;swarm_sched" CACHE STRING "List of components to test")

# On the host (linux) target, build only the closure the pure/mock tests need.
# The default "build every discovered component" pulls in micro-ROS, which
# requires esp_wifi/lwip and can't build for linux. Its idf_component.yml also
# lists esp32* targets only, and the component manager hard-errors on that as
# soon as the directory is searched -- so narrow the search path rather than
# relying on COMPONENTS to filter it out afterwards.
if("${IDF_TARGET}" STREQUAL "linux")
    set(EXTRA_COMPONENT_DIRS "../components/dwm" "../components/encoders"
                             "../components/mpu6050" "../components/swarm_hal"
                             "../components/swarm_queue" "../components/swarm_sched")
    set(COMPONENTS main dwm encoders mpu6050 swarm_hal swarm_queue swarm_sched)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unit_test)
