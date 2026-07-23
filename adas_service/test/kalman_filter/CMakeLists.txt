cmake_minimum_required(VERSION 3.10)
project(kalman_filter_one_state)

# Find Eigen and Boost packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)

# Define root directory of the SDK to avoid repeating the base path
set(ADAS_SDK_ROOT ../../../../)


# Add include directories for the Kalman filter and other dependencies
include_directories(
    ${ADAS_SDK_ROOT}/adas_service/utils/object_tracking/kalman_filter/linear_kalman_filter
    ${ADAS_SDK_ROOT}/adas_service/perception/common_yolov5/yolov5/src
    ${ADAS_SDK_ROOT}/common/src/inc
    ${Boost_INCLUDE_DIRS}
)

# Add the executable with source files
add_executable(kalman 
    ${ADAS_SDK_ROOT}/adas_service/test/kalman_filter/kalman.cpp
    ${ADAS_SDK_ROOT}/adas_service/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter.cpp
)

# Link Eigen and Boost Chrono libraries
target_link_libraries(kalman Eigen3::Eigen Boost::chrono)
