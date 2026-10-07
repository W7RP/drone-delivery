cmake_minimum_required(VERSION 3.16)
project(dd_flight LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dd_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

# PX4's generated MAVLink 2 headers (setup_ws.sh passes the PX4 build path).
set(DD_MAVLINK_INCLUDE "" CACHE PATH "PX4 build/px4_sitl_default/mavlink")
if(NOT EXISTS "${DD_MAVLINK_INCLUDE}/common/mavlink.h")
  message(FATAL_ERROR "DD_MAVLINK_INCLUDE must point at PX4's generated MAVLink headers")
endif()

# Our code builds warning-free with these; third-party headers come in as SYSTEM.
set(DD_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Werror)

add_library(dd_core STATIC src/geometry.cpp)
target_include_directories(dd_core PUBLIC include)
target_compile_options(dd_core PRIVATE ${DD_WARNINGS})

add_library(dd_mavlink STATIC src/mavlink_target.cpp)
target_include_directories(dd_mavlink PUBLIC include)
target_include_directories(dd_mavlink SYSTEM PRIVATE ${DD_MAVLINK_INCLUDE})
# MAVLink's packing helpers trip -Wconversion and -Wpedantic; keep them out of -Werror here only.
target_compile_options(dd_mavlink PRIVATE -Wall -Wextra -Wno-address-of-packed-member)

function(dd_node name)
  add_executable(${name} src/${name}.cpp)
  target_include_directories(${name} PRIVATE include)
  target_compile_options(${name} PRIVATE ${DD_WARNINGS})
  target_link_libraries(${name} dd_core ${ARGN})
  ament_target_dependencies(${name} rclcpp px4_msgs dd_interfaces sensor_msgs geometry_msgs
    std_msgs actuator_msgs)
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

dd_node(marker_detector_node dd_mavlink ${OpenCV_LIBS})
target_include_directories(marker_detector_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
dd_node(precland_check_node)
dd_node(drone_agent_node)
dd_node(pad_controller_node)
dd_node(fleet_manager_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry dd_core)
endif()

ament_package()
