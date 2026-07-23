# Get the directory name
get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# ====== 1. SETUP VARIABLE SECTION ======
set(linear_kalman_filter
    ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter.cpp
)

# ====== 2. FIND PACKAGE SECTION ======
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ====== 3. ADD INCLUDE DIRECTORIES ======
# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${OpenCV_INCLUDE_DIRS}

    ${ADAS_SERVICE_ROOT}
    ${ADAS_SDK_ROOT}

    ${ADAS_SERVICE_ROOT}/adas_main
    
    ${ADAS_SERVICE_ROOT}/control

    ${ADAS_SERVICE_ROOT}/perception/common_yolov5/yolov5/src
    ${ADAS_SERVICE_ROOT}/perception/lane_detection/inc
    
    ${ADAS_SERVICE_ROOT}/planning
    ${ADAS_SERVICE_ROOT}/planning/decision_making
    
    ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter
    
    ${ADAS_SDK_ROOT}/common/ipc_helper/
    ${ADAS_SDK_ROOT}/common/src/inc
)

# ====== 4. ADD EXECUTABLE ======
# Add the executable for statevehicle
file(GLOB TEST_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# Ignore test files
list(REMOVE_ITEM TEST_SOURCE 
    "${CMAKE_CURRENT_SOURCE_DIR}/test_prediction.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_update.cpp"
)
 
add_executable(${TEST_NAME}
                ${TEST_SOURCE}
                ${linear_kalman_filter}
)

# ====== 5. ADD COMPILER OPTIONS ======
target_compile_definitions(${TEST_NAME} PRIVATE -DUT_TEST)

# ====== 6. LINK EXECUTABLE ======  
# Link the executable with gtest
target_link_libraries(${TEST_NAME} 
                        ${GTEST_LIBRARIES}
                        ${Eigen3_LIBRARIES}
                        ${OpenCV_LIBRARIES}
)

# ====== 7. ENABLE and ADDTESTING ======
# # Enable testing
# enable_testing()
# # Add the test
# add_test(NAME ${TEST_NAME} 
#             COMMAND ${TEST_NAME}
# )

