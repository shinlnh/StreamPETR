cmake_minimum_required(VERSION 3.12)
project(test_it_control)

# ====== 1. SETUP VARIABLE SECTION ======
# SET C++ STANDARD
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread --coverage")

# ====== 2. SETUP VARIABLE DIRECTORY AND ENVIRONMENT ======
# MACROS -D
set(GENERATE_COVERAGE ON)
add_definitions(-DIT_TEST)

# ROOT PATH VARIABLE OF ADAS PROJECT
set(ADAS_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(ADAS_SERVICE_ROOT ${ADAS_SDK_ROOT}/adas_service)

# SET PATH VARIABLE OF CUDA
set(ENV{PATH} "/usr/local/cuda-11.1/bin:$ENV{PATH}")
set(ENV{LD_LIBRARY_PATH} "/usr/local/cuda-11.1/lib64:$ENV{LD_LIBRARY_PATH}")

# Set VARIABLE LIBRARIES 
set(GTEST_LIBRARIES 
    gtest
    gmock
)
set(Eigen3_LIBRARIES Eigen3::Eigen)

# LISTS ALL SUBPROJECTS
set(SUBPROJECTS 
        acc_controller
        tja_controller
        tjc_controller
)

# ====== 3. SETUP GTEST ======
# Add GoogleTest (gtest) as a subdirectory (or download it if not present)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.1
)
FetchContent_MakeAvailable(googletest)

# ===== 4. ENABLE TESTING =====
enable_testing()

# ====== 5. ADD CUSTOM TARGETS, DEPENDENCIES AND SUBDIRECTORIES SUBPROJECTS ========
# Define custom target 'build_all'
add_custom_target(build_all)

# Automatically add tests, subdirectories and dependencies
foreach(executable ${SUBPROJECTS})
    # Add subdirectories for tests
    add_subdirectory(${executable})

    # Add custom target
    add_custom_target(run_${executable}
        COMMAND ${executable}
        DEPENDS ${executable}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running ${executable}.........."
    )

    # Add test
    add_test(NAME ${executable} 
                COMMAND ${executable}
    )

    # Add dependencies for 'build_all' target
    add_dependencies(build_all run_${executable})
endforeach()

