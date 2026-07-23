cmake_minimum_required(VERSION 3.16.3)
project(adas)
message(STATUS "cmake build ${PROJECT_NAME}...")

# Capture Git information at build time
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    message(STATUS "Git Version: ${GIT_VERSION}")
    message(STATUS "Git Commit: ${GIT_COMMIT_HASH}")
    message(STATUS "Git Branch: ${GIT_BRANCH}")
else()
    set(GIT_VERSION "unknown")
    set(GIT_COMMIT_HASH "unknown")
    set(GIT_BRANCH "unknown")
    message(WARNING "Git not found, version info will be unavailable")
endif()

# Generate version header
configure_file(
    ${CMAKE_SOURCE_DIR}/common/src/inc/version.h.in
    ${CMAKE_BINARY_DIR}/generated/version.h
    @ONLY
)
include_directories(${CMAKE_BINARY_DIR}/generated)

# option
option(SET_DEBUG "Build with debug flag" ON)
option(BUILD_ADAS_SERVICE "Build ADAS Service" ON)
option(BUILD_ADAS_VISUALIZE_SERVICE "Build ADAS Visualization Service" ON)
option(BUILD_CAN_SERVICE "Build CAN Service" OFF)
option(BUILD_OLED_SERVICE "Build OLED Service" OFF)

# Build module manually
if(BUILD_ADAS_SERVICE)
    # ADAS Service
    add_subdirectory(${PROJECT_SOURCE_DIR}/adas_service)
    message(STATUS "finding the package adas_service")
endif()
if(BUILD_ADAS_VISUALIZE_SERVICE)
    # ADAS Visualize Service
    add_subdirectory(${PROJECT_SOURCE_DIR}/adas_visualize_service)
    message(STATUS "finding the package adas_visualize_service")
endif()
if(BUILD_CAN_SERVICE)
    # CAN Service
    add_subdirectory(${PROJECT_SOURCE_DIR}/can_service)
    message(STATUS "finding the package can_service")
endif()
if(BUILD_OLED_SERVICE)
    # OLED Service
    add_subdirectory(${PROJECT_SOURCE_DIR}/oled_service)
    message(STATUS "finding the package oled_service")
endif()

# Build all service
if (NOT BUILD_ADAS_SERVICE AND NOT BUILD_CAN_SERVICE AND NOT BUILD_OLED_SERVICE)
    # get src directory
    file(GLOB SRC_DIR "${PROJECT_SOURCE_DIR}/*")

    # check file CMakeLists.txt
    foreach(M ${SRC_DIR})
        if(IS_DIRECTORY ${M})
            if(EXISTS ${M}/CMakeLists.txt)
                add_subdirectory(${M})
                get_filename_component(PRJ_NAME ${M} NAME)
                message(STATUS "finding the package ${PRJ_NAME}")
            endif()
        endif()
    endforeach()
endif()
