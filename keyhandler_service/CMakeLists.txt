cmake_minimum_required(VERSION 3.16.3)
project(keyhandler_service LANGUAGES C CXX)
message(STATUS "cmake build ${PROJECT_NAME}...")

# Find required packages
set(Boost_REQUIRED_COMPONENTS timer chrono thread)
find_package(Boost REQUIRED ${Boost_REQUIRED_COMPONENTS})


# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lrt -lboost_thread-mt -DENABLE_CAN")
if(UT_TEST)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

# IMPORTANT: Add dependencies to ensure OpenDDS files are generated before building the library

# Set include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/can_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/dbc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Build executables
###############
# keyhandler_service #
###############
add_executable(keyhandler_service
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/InputDeviceUtils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KeyWorker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/dbc/tesla.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/can_manager/can_manager.cpp
)

target_link_libraries(keyhandler_service
    ${Boost_LIBRARIES}
)