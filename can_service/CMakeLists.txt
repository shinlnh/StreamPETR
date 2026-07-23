cmake_minimum_required(VERSION 3.16.3)
project(can_service LANGUAGES C CXX)
message(STATUS "cmake build ${PROJECT_NAME}...")

# Find required packages
set(Boost_REQUIRED_COMPONENTS timer chrono thread)
find_package(Boost REQUIRED ${Boost_REQUIRED_COMPONENTS})
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ipc_helper REQUIRED)
find_package(message_filters REQUIRED)

# Make sure CAN Service always uses CAN
add_definitions(-DENABLE_CAN)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lrt -lboost_thread")
if(UT_TEST)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

# IMPORTANT: Add dependencies to ensure OpenDDS files are generated before building the library

# Set include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/can_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/dbc
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/ipc_helper/interface/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inc
)

# Build executables
###############
# can_service #
###############
add_executable(can_service
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can_service_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/src/can_service.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/src/can_service_if.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/dbc/tesla.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc/can_manager/can_manager.cpp
)

target_link_libraries(can_service
    ${Boost_LIBRARIES}
)

ament_target_dependencies(can_service
    rclcpp
    ipc_helper
    message_filters
)


############
# can_test #
############
# add_executable(can_test 
#     ${CMAKE_CURRENT_SOURCE_DIR}/test/can_test.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/src/can_service.cpp
# )

# # Add dependencies for test executable
# add_dependencies(can_test ${opendds_libs})

# target_link_libraries(can_test PUBLIC 
#     ${package}
#     OpenDDS::OpenDDS 
#     OpenDDS::Dcps
#     OpenDDS::Rtps
#     OpenDDS::Rtps_Udp
#     ${opendds_libs}
#     ${Boost_LIBRARIES}
# )
