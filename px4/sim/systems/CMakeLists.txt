cmake_minimum_required(VERSION 3.16)
project(dd_gz_systems LANGUAGES CXX)
# Phase 8: Gazebo systems of this repo, built against the host's Gazebo Harmonic (gz-sim 8).
# px4/tools/build_gz_systems.sh builds them into px4/sim/systems/build (git-ignored).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
add_library(dd_chase_camera SHARED dd_chase_camera.cpp)
target_link_libraries(dd_chase_camera PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
target_compile_options(dd_chase_camera PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Werror)
