# CMakeList.txt : CMake project for point_cloud2png, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Find packages
# set(PCL_ROOT "C:/Program Files (x86)/PCL 1.8.1")
find_package(PCL CONFIG REQUIRED)
# set(OpenCV "C:/Program Files (x86)/opencv")
find_package(OpenCV CONFIG REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(./ SRC)
# Add source to this project's executable.
add_executable (rail_imager ${SRC}) 

# Link
target_link_libraries(rail_imager ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
# TODO: Add tests and install targets if needed.

