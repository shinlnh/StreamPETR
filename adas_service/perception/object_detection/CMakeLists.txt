if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    set (CPP_FS_LIB "stdc++fs")
endif()

if (DEFINED ENV{PC_BUILD})
    set(TENSORRT_ROOT "/prj/es-automotive/adas/common/TensorRT-8.6.1.6/")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast")

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(QT NAMES Qt5 REQUIRED COMPONENTS Widgets Concurrent Core Quick)
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Concurrent Core Quick)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES include/)
message(STATUS "Found TensorRT headers at ${TENSORRT_INCLUDE_DIR}")

find_library(TENSORRT_LIBRARY_INFER nvinfer
        HINTS ${TENSORRT_ROOT} ${TENSORRT_BUILD} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)
message(STATUS "Found TensorRT libs ${TENSORRT_LIBRARY_INFER}")
list(APPEND CUDA_NVCC_FLAGS "-D_FORCE_INLINES -Xcompiler -fPIC")
list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -std=c++17")

find_package(OpenCV REQUIRED)
set(TENSORRT_LIBS "")

if (DEFINED ENV{PC_BUILD})
    list(APPEND TENSORRT_LIBS /prj/es-automotive/adas/common/TensorRT-8.6.1.6/include/)
endif()

link_directories(
    ${OpenCV_LIBRARIES_DIRS} 
    ${TENSORRT_LIBS} 
)

include_directories(
        ${CUDA_INCLUDE_DIRS}
        ${TENSORRT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../common_yolov5/yolov5/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../common_yolov5/yolov5/plugin
        ${CMAKE_CURRENT_SOURCE_DIR}/../streampetr/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/object_tracking/kalman_filter/linear_kalman_filter/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/ipc_helper/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/src/inc/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/config/
        ${CMAKE_CURRENT_SOURCE_DIR}/../lane_detection/inc/
        ${CMAKE_CURRENT_SOURCE_DIR}/../trt_engine/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/thread_pooling/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../adas_main/
)

file(GLOB CPP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/../common_yolov5/yolov5/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../streampetr/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../lane_detection/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../trt_engine/*.cpp
)
# Exclude standalone tools from the shared library
list(FILTER CPP_SRC EXCLUDE REGEX "gen_yolov5_engine\\.cpp$")

file(GLOB CU_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/../common_yolov5/yolov5/plugin/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/../common_yolov5/yolov5/src/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/../streampetr/src/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/../lane_detection/src/*.cu
)
cuda_add_library(adas_service_object_detector SHARED
    ${CPP_SRC}
    ${CU_SRC}
)

target_compile_features(adas_service_object_detector PRIVATE cxx_std_17)

get_filename_component(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/../../../lib REALPATH)

target_link_libraries(
        adas_service_object_detector
        ${TENSORRT_LIBRARY_INFER}
        ${OpenCV_LIBS}
        ${CPP_FS_LIB}
        nvonnxparser
        # Qt${QT_VERSION_MAJOR}::Core
        Eigen3::Eigen
)

# # Standalone tool: generate YOLOv5 TensorRT engine from .wts weights
# # Links against adas_service_object_detector to reuse compiled CUDA objects
# # (avoids FindCUDA race condition when two targets share .cu sources)
# add_executable(gen_yolov5_engine
#     ${CMAKE_CURRENT_SOURCE_DIR}/gen_yolov5_engine.cpp
# )
# target_compile_features(gen_yolov5_engine PRIVATE cxx_std_17)
# set_target_properties(gen_yolov5_engine PROPERTIES
#     RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../../../bin"
# )
# target_link_libraries(gen_yolov5_engine
#     adas_service_object_detector
#     ${TENSORRT_LIBRARY_INFER}
#     ${CPP_FS_LIB}
#     nvonnxparser
# )
# install(TARGETS gen_yolov5_engine DESTINATION bin)
