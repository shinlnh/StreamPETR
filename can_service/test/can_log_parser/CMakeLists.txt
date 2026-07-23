cmake_minimum_required(VERSION 3.16.3)
project(can_service LANGUAGES C CXX)
message(STATUS "cmake build ${PROJECT_NAME}...")

find_package(OpenDDS REQUIRED)
set(Boost_REQUIRED_COMPONENTS timer chrono thread)

find_package(Boost REQUIRED ${Boost_REQUIRED_COMPONENTS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lrt -lboost_thread")

# include directory
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/can_manager/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/dbc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/inc
)

# add executable
add_executable(can_service
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/can_manager/can_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/dbc/tesla.c
)
    
    target_link_libraries(can_service PUBLIC OpenDDS::OpenDDS ${Boost_LIBRARIES})
    
add_executable(send_scenario
    ${CMAKE_CURRENT_SOURCE_DIR}/send.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/can_manager/can_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/dbc/tesla.c
)

target_link_libraries(send_scenario PUBLIC OpenDDS::OpenDDS ${Boost_LIBRARIES})