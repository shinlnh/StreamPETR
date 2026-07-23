cmake_minimum_required(VERSION 3.16.3)
project(test_adas_service_if)

# ====== 1. SETUP VARIABLE SECTION ======
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
get_filename_component(SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../.." REALPATH)

# ====== 2. FIND PACKAGE SECTION ======
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ipc_helper REQUIRED)

# ====== 3. ADD DIRECTORIES SECTION ======
include_directories(
    ${SDK_PATH}/common/src/inc
    ${CMAKE_CURRENT_LIST_DIR}/../inc
)

# ====== 4. ADD EXECUTABLES SECTION ======
add_executable(
    publisher
    main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/adas_service_if.cpp
)

add_executable(
    subscriber
    main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../src/adas_service_if.cpp
)

# ====== 5. LINK LIBRARIES ======
ament_target_dependencies(
    publisher
    rclcpp
    ipc_helper
)

ament_target_dependencies(
    subscriber
    rclcpp
    ipc_helper
)

target_compile_definitions(
    publisher
    PRIVATE
    PUBLISHER
)