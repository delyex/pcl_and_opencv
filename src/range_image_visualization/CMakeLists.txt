﻿# CMakeList.txt : CMake project for point_cloud2png, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Find packages
set(PCL_ROOT "C:/Program Files (x86)/PCL 1.8.1")
find_package(PCL CONFIG REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# Add source to this project's executable.
add_executable (range_imager "range_image_visualization.cpp")

# Link
target_link_libraries(range_imager ${PCL_LIBRARIES} ${Boost_LIBRARIES})
# TODO: Add tests and install targets if needed.
