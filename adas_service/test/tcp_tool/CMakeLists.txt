cmake_minimum_required(VERSION 3.5)

project(adas_client VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


find_package(OpenCV REQUIRED)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/src/client.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)
