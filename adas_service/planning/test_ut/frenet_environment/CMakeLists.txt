# Get the directory name
get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
# ====== 1. SETUP VARIABLE SECTION ======
set(frenet_environment
    ${ADAS_SERVICE_ROOT}/planning/frenet_environment/frenet_environment.cpp
    # ${ADAS_SERVICE_ROOT}/adas_main/frenet_environment/visualization_helper.cpp
    # ${ADAS_SERVICE_ROOT}/planning/algorithms/line_segment_intersect.cpp
    # ${ADAS_SERVICE_ROOT}/planning/lane_environment/road_enviroment_define.cpp
    # ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter.cpp
    # ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter_for_cam.cpp
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

    ${ADAS_SERVICE_ROOT}/utils
    ${ADAS_SERVICE_ROOT}/utils/adas_service_if/inc
    ${ADAS_SERVICE_ROOT}/utils/maths
    ${ADAS_SERVICE_ROOT}/utils/multi_node
    ${ADAS_SERVICE_ROOT}/utils/object_tracking
    ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter
    ${ADAS_SERVICE_ROOT}/utils/object_tracking/hungarian_matching
    ${ADAS_SERVICE_ROOT}/utils/thread_pooling


    ${ADAS_SERVICE_ROOT}/adas_main
    ${ADAS_SERVICE_ROOT}/planning/frenet_environment

    # ${ADAS_SERVICE_ROOT}/control

    ${ADAS_SERVICE_ROOT}/perception/object_detection
    ${ADAS_SERVICE_ROOT}/perception/common_yolov5/yolov5/src
    ${ADAS_SERVICE_ROOT}/perception/lane_detection/inc

    ${ADAS_SERVICE_ROOT}/planning
    ${ADAS_SERVICE_ROOT}/planning/lane_environment
    ${ADAS_SERVICE_ROOT}/planning/algorithms
    ${ADAS_SERVICE_ROOT}/planning/frenet_planner


    # ${ADAS_SERVICE_ROOT}/utils/object_tracking/kalman_filter/linear_kalman_filter

    ${ADAS_SDK_ROOT}/common/ipc_helper/
    ${ADAS_SDK_ROOT}/common/src/inc
)

# ====== 4. ADD EXECUTABLE ======
# Add the executable for statevehicle
file(GLOB TEST_SOURCE "./*cpp")
add_executable(${TEST_NAME} 
                ${TEST_SOURCE}
                ${frenet_environment}
)
# ====== 5. ADD COMPILER OPTIONS ======
target_compile_definitions(${TEST_NAME} PRIVATE -DUT_TEST_FRENET)

# ====== 6. LINK EXECUTABLE ======  
# Link the executable with gtest
add_library (stub_find_line_intersect stub/stub_find_line_intersect)
target_link_libraries(${TEST_NAME} 
                        ${GTEST_LIBRARIES}
                        ${Eigen3_LIBRARIES}
                        ${OpenCV_LIBS}
                        stub_find_line_intersect
)

# ====== 7. ENABLE and ADDTESTING ======
# # Enable testing
# enable_testing()
# # Add the test
# add_test(NAME ${TEST_NAME} 
#             COMMAND ${TEST_NAME}
# )
