cmake_minimum_required(VERSION 3.22)
project(swarm_sim CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# the firmware headers take their constants from the sdkconfig.h IDF
# generates. This is the same translation, so the sim builds against the same
# wheel geometry and namespace the board does
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${REPO}/sdkconfig)
file(STRINGS ${REPO}/sdkconfig entries REGEX "^CONFIG_[A-Za-z0-9_]+=")
set(defines "")
foreach(entry IN LISTS entries)
  string(REGEX MATCH "^(CONFIG_[A-Za-z0-9_]+)=(.*)$" _ "${entry}")
  set(value "${CMAKE_MATCH_2}")
  if(value STREQUAL "y")
    set(value 1)
  endif()
  string(APPEND defines "#define ${CMAKE_MATCH_1} ${value}\n")
endforeach()
file(CONFIGURE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config/sdkconfig.h
     CONTENT "${defines}" @ONLY)

add_executable(sim_base sim_base.cpp)
target_include_directories(sim_base PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${REPO}/main/src
  ${REPO}/components/swarm_hal
  ${CMAKE_CURRENT_BINARY_DIR}/config)
target_compile_options(sim_base PRIVATE -Wall -Wextra)
target_link_libraries(sim_base PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_msgs_TARGETS})
