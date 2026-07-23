cmake_minimum_required(VERSION 3.16.3)
project(can_service LANGUAGES C CXX)
message(STATUS "cmake build ${PROJECT_NAME}...")

set(Boost_REQUIRED_COMPONENTS timer chrono thread)

find_package(Boost REQUIRED ${Boost_REQUIRED_COMPONENTS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lrt -pthread -lboost_thread")

# include directory
include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../
)

# add executable
add_executable(can_test
    ${CMAKE_CURRENT_SOURCE_DIR}/example.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/can_manager.cpp
)
target_link_libraries(can_test PUBLIC ${package})