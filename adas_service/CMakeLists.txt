cmake_minimum_required(VERSION 3.12)

project(adas_service VERSION 0.1 LANGUAGES CXX)

# Define option
option(BUILD_TEST "Build for testing coverage" OFF)
option(ENABLE_CAN "Switch to CAN Mode" OFF)

# ====== 1. SETUP VARIABLE SECTION ======
# Set up CMake and CXX variable to build
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if (DEFINED ENV{PC_BUILD})
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Override compiler flags to remove problematic options
set(CMAKE_CXX_COMPILER_ARG1 "")
set(CMAKE_C_COMPILER_ARG1 "")

set(CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} \
    -pthread \
    -lboost_thread \
    -fpermissive \
    -Wno-deprecated-declarations \
    -fstack-protector-strong \
    -O2 \
    -D_FORTIFY_SOURCE=2 \
    -Wformat \
    -Wformat-security \
    -Werror=format-security"
)

# Set your own flags
if (NOT DEFINED ENV{PC_BUILD})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+crc --sysroot=$ENV{SDKTARGETSYSROOT}")
endif()

# SET Flags for Unit Test
if(BUILD_TEST)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

# ====== 2. FIND PACKAGE SECTION ======
# Find package that required for building
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)
set(Boost_REQUIRED_COMPONENTS timer chrono)

if (DEFINED ENV{PC_BUILD})
    list(APPEND Boost_REQUIRED_COMPONENTS thread)
endif()

find_package(Boost REQUIRED ${Boost_REQUIRED_COMPONENTS})
find_package(ZLIB REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)
pkg_check_modules(GST_RTP REQUIRED gstreamer-rtp-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# ROS package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ipc_helper REQUIRED)
find_package(carla_msgs_custom REQUIRED)

if (DEFINED ENV{PC_BUILD}) 
    link_directories(/prj/es-automotive/adas/common/TensorRT-8.6.1.6/lib)
endif()

find_package(CUDA REQUIRED)
if(NOT CMAKE_CUDA_DEVICE_LINK_LIBRARY)
    set(CMAKE_CUDA_DEVICE_LINK_LIBRARY "<CMAKE_CUDA_COMPILER> <CMAKE_CUDA_LINK_FLAGS> <LANGUAGE_COMPILE_FLAGS> ${CMAKE_CUDA_COMPILE_OPTIONS_PIC} ${_CMAKE_CUDA_EXTRA_DEVICE_LINK_FLAGS} -shared -dlink <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
endif()
if(NOT CMAKE_CUDA_DEVICE_LINK_EXECUTABLE)
    set(CMAKE_CUDA_DEVICE_LINK_EXECUTABLE "<CMAKE_CUDA_COMPILER> <FLAGS> <CMAKE_CUDA_LINK_FLAGS> ${CMAKE_CUDA_COMPILE_OPTIONS_PIC} ${_CMAKE_CUDA_EXTRA_DEVICE_LINK_FLAGS} -shared -dlink <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
endif()
# Add object detection shared library
add_subdirectory(${PROJECT_SOURCE_DIR}/perception/object_detection)

set(TENSORRT_DIRS "")

if (DEFINED ENV{PC_BUILD})
    list(APPEND TENSORRT_DIRS /prj/es-automotive/adas/common/TensorRT-8.6.1.6/include/)
endif()

# Setting include, lib directories and definitions 
include_directories(
    ${TENSORRT_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
    ${GST_RTSP_SERVER_INCLUDE_DIRS}
    ${GST_RTP_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}

    ${CMAKE_SOURCE_DIR}/common/communications/
    ${CMAKE_SOURCE_DIR}/common/config/
    ${CMAKE_SOURCE_DIR}/common/src/inc/
    ${CMAKE_SOURCE_DIR}/common/src/inc/can_manager/
    ${CMAKE_SOURCE_DIR}/common/src/inc/dbc/
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/interface/inc

    ${CMAKE_CURRENT_SOURCE_DIR}/utils/
    ${CMAKE_CURRENT_SOURCE_DIR}/tracker/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/tcp_ip_streaming
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/maths
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/thread_pooling
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/debug_substitution
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/object_tracking/kalman_filter/linear_kalman_filter
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/object_tracking/hungarian_matching/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/object_tracking
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/multi_node

    ${CMAKE_CURRENT_SOURCE_DIR}/ui/
    
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/streampetr/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/lane_detection/inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/camera_model/
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/tracker/
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/common_yolov5/yolov5/src 
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/common_yolov5/yolov5/plugin
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/object_detection/
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/trt_engine/
    ${CMAKE_CURRENT_SOURCE_DIR}/perception/

    ${CMAKE_CURRENT_SOURCE_DIR}/planning/
    ${CMAKE_CURRENT_SOURCE_DIR}/planning/decision_making/
    ${CMAKE_CURRENT_SOURCE_DIR}/planning/frenet_planner/
    ${CMAKE_CURRENT_SOURCE_DIR}/planning/planning_queue/
    ${CMAKE_CURRENT_SOURCE_DIR}/planning/safety_force_field/

    ${CMAKE_CURRENT_SOURCE_DIR}/control/

    ${CMAKE_CURRENT_SOURCE_DIR}/adas_main/
    
    ${Boost_INCLUDE_DIR}
)

if(ENABLE_CAN)
    add_definitions(-DENABLE_CAN)
endif()

set(PROJECT_SOURCES_MAIN
    ${PROJECT_SOURCE_DIR}/main.cpp
)

# Image processing and algorithms
set(PROJECT_SOURCES_OBJECT_DETECTION
    ${PROJECT_SOURCE_DIR}/perception/camera_model/cameramodel.cpp
    ${PROJECT_SOURCE_DIR}/perception/aeb_controller/aeb_controller.cpp
)
set(PROJECT_SOURCE_TRT_ENGINE
    ${PROJECT_SOURCE_DIR}/perception/trt_engine/engine.cpp
)
set(PROJECT_SOURCE_PERCEPTION_MODEL
    ${PROJECT_SOURCE_DIR}/perception/perception.cpp
)
set(PROJECT_SOURCE_PERCEPTION_TRACKING
    ${PROJECT_SOURCE_DIR}/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter.cpp
    ${PROJECT_SOURCE_DIR}/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter_for_cam.cpp
    ${PROJECT_SOURCE_DIR}/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter_for_radar.cpp
    ${PROJECT_SOURCE_DIR}/perception/tracker/tracker.cpp
)

# collect all planner source files
file(GLOB PLANNER_SOURCES 
    ${PROJECT_SOURCE_DIR}/planning/*.c 
    ${PROJECT_SOURCE_DIR}/planning/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/algorithms/*.c
    ${PROJECT_SOURCE_DIR}/planning/algorithms/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/decision_making/*.c 
    ${PROJECT_SOURCE_DIR}/planning/decision_making/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/planning_queue/*.c 
    ${PROJECT_SOURCE_DIR}/planning/planning_queue/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/curve_fitter/*.c 
    ${PROJECT_SOURCE_DIR}/planning/curve_fitter/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/trajectory_generator/*.c 
    ${PROJECT_SOURCE_DIR}/planning/trajectory_generator/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/frenet_environment/*.c 
    ${PROJECT_SOURCE_DIR}/planning/frenet_environment/*.cpp
    ${PROJECT_SOURCE_DIR}/planning/safety_force_field/*.cpp
)
set(PROJECT_SOURCES_PATH_PLANNING
    ${PLANNER_SOURCES}
)

# collect all controller source files
file(GLOB CONTROLLER_SOURCES 
     ${PROJECT_SOURCE_DIR}/control/*.c 
     ${PROJECT_SOURCE_DIR}/control/*.cpp
)

set(PROJECT_SOURCES_PATH_TRACKING
    ${CONTROLLER_SOURCES}
)

# collect all adas_main source files
file(GLOB ADAS_MAIN_SOURCES 
    ${PROJECT_SOURCE_DIR}/adas_main/*.c 
    ${PROJECT_SOURCE_DIR}/adas_main/*.cpp
)
set(PROJECT_SOURCES_ADAS_MAIN
    ${ADAS_MAIN_SOURCES}
)

# Communication
list(APPEND PROJECT_SOURCES_COMMUNICATION
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/interface/src/adas_service_if.cpp
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/interface/src/msg_utils.cpp
)

list(APPEND PROJECT_SOURCES_STREAM
    ${PROJECT_SOURCE_DIR}/utils/tcp_ip_streaming/tcp_ip_server.cpp
    ${CMAKE_SOURCE_DIR}/common/communications/ReceiverUdp.cpp
    ${CMAKE_SOURCE_DIR}/common/communications/ReceiverGTCP.cpp
    ${CMAKE_SOURCE_DIR}/common/communications/ReceiverRtsp.cpp
    ${CMAKE_SOURCE_DIR}/common/communications/Receiver.cpp
    ${CMAKE_SOURCE_DIR}/common/communications/RtspServer.cpp
)

list(APPEND PROJECT_SOURCES_UTILS
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/intermediate_representations.cpp
    ${CMAKE_SOURCE_DIR}/common/src/inc/can_manager/can_manager.cpp
    ${CMAKE_SOURCE_DIR}/common/src/inc/dbc/tesla.c
)

set(ADAS_SOURCE_CODE_LIST 
    ${PROJECT_SOURCES_MAIN}
    ${PROJECT_SOURCE_PERCEPTION_TRACKING}
    ${PROJECT_SOURCE_TRT_ENGINE}
    ${PROJECT_SOURCE_PERCEPTION_MODEL}
    ${PROJECT_SOURCES_PATH_PLANNING}
    ${PROJECT_SOURCES_PATH_TRACKING}
    ${PROJECT_SOURCES_ADAS_MAIN}
    ${PROJECT_SOURCES_COMMUNICATION}
    ${PROJECT_SOURCES_STREAM}
    ${PROJECT_SOURCES_UTILS}
)

list(APPEND ADAS_SOURCE_CODE_LIST ${PROJECT_SOURCES_OBJECT_DETECTION})

add_executable(
    ${PROJECT_NAME} 
    ${ADAS_SOURCE_CODE_LIST}
)

set(LINK_LIBS_LIST 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)

list(APPEND LINK_LIBS_LIST ${CUDA_LIBRARIES} adas_service_object_detector)


target_link_libraries(
    ${PROJECT_NAME}
    ${LINK_LIBS_LIST} 
    ${GST_LIBRARIES}
    ${GST_RTSP_SERVER_LIBRARIES}
    ${GST_RTP_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${X11_LIBRARIES}
    -lgstrtspserver-1.0
    ZLIB::ZLIB
)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    ipc_helper
    carla_msgs_custom
)
