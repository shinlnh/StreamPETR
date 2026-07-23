cmake_minimum_required(VERSION 3.10)
project(CameraCapture)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(camera_capture camera_capture.cpp)

# Link OpenCV libraries
target_link_libraries(camera_capture ${OpenCV_LIBS})
