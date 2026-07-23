# Get the directory name
get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# ====== 1. SETUP VARIABLE SECTION ======
set(cameramodel
    ${ADAS_SERVICE_ROOT}/perception/camera_model/cameramodel.cpp
)

# ====== 2. FIND PACKAGE SECTION ======
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ====== 3. ADD INCLUDE DIRECTORIES ======
# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${OpenCV_INCLUDE_DIRS}

    ${ADAS_SERVICE_ROOT}/adas_main
    
    ${ADAS_SERVICE_ROOT}/control

    ${ADAS_SERVICE_ROOT}/perception/camera_model
    ${ADAS_SERVICE_ROOT}/perception/object_detection
    ${ADAS_SERVICE_ROOT}/perception/common_yolov5/yolov5/src
    ${ADAS_SERVICE_ROOT}/perception/lane_detection/inc
    
    ${ADAS_SERVICE_ROOT}/planning
    
    ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter
    
    ${ADAS_SDK_ROOT}/common/ipc_helper/
    ${ADAS_SDK_ROOT}/common/src/inc
)

# ====== 4. ADD EXECUTABLE ======
# Add the executable for statevehicle
add_executable( ${TEST_NAME} 
                test_cameramodel.cpp
                ${cameramodel}
)

# ====== 5. ADD COMPILER OPTIONS ======
target_compile_definitions(${TEST_NAME} PRIVATE -DUT_TEST)

# ====== 6. LINK EXECUTABLE ======  
# Link the executable with gtest
target_link_libraries(  ${TEST_NAME} 
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

# ====== 8. Copy config files ======
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/birdview_calib.txt
          ${CMAKE_CURRENT_SOURCE_DIR}/undistort_calib.txt
          ${CMAKE_CURRENT_SOURCE_DIR}/camera_config.txt
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})