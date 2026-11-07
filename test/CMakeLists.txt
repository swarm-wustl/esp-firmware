# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Include the components directory of the main application:
#
set(EXTRA_COMPONENT_DIRS "../components")


# Set the components to include the tests for.
# The list is a plain variable, not a cache entry: a cache entry is only
# written on a build dir's first configure, so a default kept there goes stale
# the moment this file changes. To test a subset, override it instead:
# - idf.py -B build_host -DTEST_COMPONENTS_OVERRIDE="dwm;swarm_hal" ...
# - and -DTEST_COMPONENTS_OVERRIDE="" to go back to the full list
#
set(TEST_COMPONENTS_OVERRIDE "" CACHE STRING
    "Test only these components (;-separated); empty tests them all")

if(DEFINED CACHE{TEST_COMPONENTS})
    message(WARNING "TEST_COMPONENTS is no longer read from the cache; "
                    "dropping it. Use -DTEST_COMPONENTS_OVERRIDE for a subset")
    unset(TEST_COMPONENTS CACHE)
endif()

if(TEST_COMPONENTS_OVERRIDE)
    set(TEST_COMPONENTS "${TEST_COMPONENTS_OVERRIDE}")
else()
    set(TEST_COMPONENTS "dwm;encoder_bank;mpu6050;swarm_hal;swarm_sched")
endif()

# On the host (linux) target, build only the closure the pure/mock tests need.
# The default "build every discovered component" pulls in micro-ROS, which
# requires esp_wifi/lwip and can't build for linux. Its idf_component.yml also
# lists esp32* targets only, and the component manager hard-errors on that as
# soon as the directory is searched -- so narrow the search path rather than
# relying on COMPONENTS to filter it out afterwards.
if("${IDF_TARGET}" STREQUAL "linux")
    set(EXTRA_COMPONENT_DIRS "../components/dwm" "../components/encoder_bank"
                             "../components/mpu6050" "../components/swarm_hal"
                             "../components/swarm_queue" "../components/swarm_sched")
    set(COMPONENTS main dwm encoder_bank mpu6050 swarm_hal swarm_queue swarm_sched)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unit_test)
