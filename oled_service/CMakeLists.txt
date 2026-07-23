cmake_minimum_required(VERSION 3.16.3)
project(oled_service LANGUAGES CXX)
message(STATUS "cmake build ${PROJECT_NAME}...")

find_package(OpenDDS REQUIRED)
find_package(ZLIB REQUIRED)

# -lz for zlib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lz")

# include directory
include_directories(
    ${DDS_ROOT}
    ${ACE_ROOT}
    ${TAO_ROOT}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/opendds_generated
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/src/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inc
)

# generating file
add_library(OledServiceTopic)
opendds_target_sources(OledServiceTopic
    ${CMAKE_CURRENT_SOURCE_DIR}/../ipc_helper/idl/OledServiceTopic.idl
)

set(opendds_libs
    OledServiceTopic
)

set(SRC_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/src/oled_service_if.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inc/oled_service_if.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/src/ssd1307.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/inc/ssd1307.h
)

# add static library
set(package "oled_service_if")
add_library(${package} STATIC ${SRC_FILE})
target_link_libraries(${package} PUBLIC ZLIB::ZLIB)

# add executable
add_executable(oled_service
    ${CMAKE_CURRENT_SOURCE_DIR}/src/oled_service_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/src/oled_service.cpp
)
target_link_libraries(
    oled_service PUBLIC 
    ${package}
    OpenDDS::Dcps
    ${opendds_libs}
)

add_executable(oled_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/oled_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/src/oled_service.cpp
)
target_link_libraries(
    oled_test PUBLIC 
    OpenDDS::Dcps
    ${package}
    ${opendds_libs}
)