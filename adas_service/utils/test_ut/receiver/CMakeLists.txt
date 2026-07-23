# Get the directory name
get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# ====== 1. SETUP VARIABLE SECTION ======
set(Receiver
    ${ADAS_SDK_ROOT}/common/communications/Receiver.cpp
)

# ====== 2. FIND PACKAGE SECTION ======
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# ====== 3. ADD INCLUDE DIRECTORIES ======
# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${OpenCV_INCLUDE_DIRS}

    ${ADAS_SERVICE_ROOT}/adas_main
    
    ${ADAS_SERVICE_ROOT}/control
    
    ${ADAS_SERVICE_ROOT}/perception/object_detection
    ${ADAS_SERVICE_ROOT}/perception/common_yolov5/yolov5/src
    ${ADAS_SERVICE_ROOT}/perception/lane_detection/inc
    
    ${ADAS_SERVICE_ROOT}/planning
    ${ADAS_SERVICE_ROOT}/planning/trajectory_generator
    
    ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter

    ${ADAS_SDK_ROOT}/common/communications/
    ${ADAS_SDK_ROOT}/common/ipc_helper/
    ${ADAS_SDK_ROOT}/common/src/inc
)

# ====== 4. ADD EXECUTABLE ======
# Add the executable for statevehicle
file(GLOB TEST_SOURCE "./*cpp")

add_executable(${TEST_NAME} 
                ${TEST_SOURCE}
                ${Receiver}
)

# ====== 5. ADD COMPILER OPTIONS ======
target_compile_definitions(${TEST_NAME} PRIVATE -DUT_TEST)

# ====== 6. LINK EXECUTABLE ======  
# Link the executable with gtest
target_include_directories(${TEST_NAME} PRIVATE
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
)

target_link_libraries(${TEST_NAME} 
                        ${GTEST_LIBRARIES}
                        ${Eigen3_LIBRARIES}
                        ${OpenCV_LIBS}
                        ${GSTREAMER_LIBRARIES}
                        ${GSTREAMER_APP_LIBRARIES}
)

# ====== 7. ENABLE and ADDTESTING ======
# # Enable testing
# enable_testing()
# # Add the test
# add_test(NAME ${TEST_NAME} 
#             COMMAND ${TEST_NAME}
# )
