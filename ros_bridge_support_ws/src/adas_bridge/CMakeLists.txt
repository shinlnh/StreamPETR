cmake_minimum_required(VERSION 3.8)
project(adas_bridge)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -pthread")
add_definitions(-DADAS_BRIDGE_BUILD)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(carla_msgs_custom REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(ipc_helper REQUIRED)
find_package(nav_msgs REQUIRED)

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4.2.0 EXACT REQUIRED PATHS /usr/include/opencv4)
link_directories(/usr/lib/x86_64-linux-gnu)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
#------------------------GStreamer------------------------#
pkg_check_modules(GST REQUIRED gstreamer-1.0)

# Find the VPI package
find_package(vpi 2.3 REQUIRED)

#------------------------pcl library------------------------#
find_package(PCL REQUIRED COMPONENTS common filters tracking sample_consensus features)
find_package(pcl_conversions REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

get_filename_component(ADAS_ROOT "${PROJECT_SOURCE_DIR}/../../.." REALPATH)

include_directories(
  include
  ${ADAS_ROOT}/common/src/inc
  ${PROJECT_SOURCE_DIR}/src/Radar
  ${PROJECT_SOURCE_DIR}/src/Radar/DBSCAN
  ${PROJECT_SOURCE_DIR}/src/Radar/Debug
  ${PROJECT_SOURCE_DIR}/src/Radar/Utils
  ${ADAS_ROOT}/adas_service/utils/debug_substitution
)

# Add executables
add_executable(${PROJECT_NAME}_rgb_front_node 
  src/rgb_front.cpp
)

add_executable(${PROJECT_NAME}_radar_front_node 
  src/Radar/radar_front.cpp
  src/Radar/DBSCAN/dbscan.cpp  
  src/Radar/Utils/utilities.cpp 
  ${ADAS_ROOT}/common/ipc_helper/interface/src/adas_service_if.cpp
)

add_executable(${PROJECT_NAME}_control_msg_node 
  src/control_msg.cpp
  ${ADAS_ROOT}/common/ipc_helper/interface/src/adas_service_if.cpp
)

add_executable(${PROJECT_NAME}_vehicle_status_node 
  src/vehicle_status.cpp
  ${ADAS_ROOT}/common/ipc_helper/interface/src/adas_service_if.cpp
)

add_executable(${PROJECT_NAME}_gnss_node
  src/gnss.cpp
  ${ADAS_ROOT}/common/ipc_helper/interface/src/adas_service_if.cpp
)

add_executable(${PROJECT_NAME}_imu_node
  src/imu.cpp
  ${ADAS_ROOT}/common/ipc_helper/interface/src/adas_service_if.cpp
)

add_executable(${PROJECT_NAME}_odometer_node
  src/odometer.cpp
  ${ADAS_ROOT}/common/ipc_helper/interface/src/adas_service_if.cpp
)

# Link directory
target_include_directories(${PROJECT_NAME}_rgb_front_node PRIVATE ${OpenCV_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_radar_front_node PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${ADAS_ROOT}/common/ipc_helper/interface/inc)
target_include_directories(${PROJECT_NAME}_vehicle_status_node PRIVATE ${ADAS_ROOT}/common/ipc_helper/interface/inc)
target_include_directories(${PROJECT_NAME}_imu_node PRIVATE ${ADAS_ROOT}/common/ipc_helper/interface/inc)
target_include_directories(${PROJECT_NAME}_odometer_node PRIVATE ${ADAS_ROOT}/common/ipc_helper/interface/inc)
target_include_directories(${PROJECT_NAME}_gnss_node PRIVATE ${ADAS_ROOT}/common/ipc_helper/interface/inc)
target_include_directories(${PROJECT_NAME}_control_msg_node PRIVATE ${ADAS_ROOT}/common/ipc_helper/interface/inc)

## Specify libraries to link a library or executable target against
## The template:
## target_link_libraries(<executable_name> [<library>...])
## ament_target_dependencies(<executable_name> [<ROS related library>...])

target_link_libraries(${PROJECT_NAME}_radar_front_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS} 
)

target_link_libraries(${PROJECT_NAME}_rgb_front_node
  ${OpenCV_LIBS} 
  ${GST_LIBRARIES}
  vpi
  gstapp-1.0
  gstrtp-1.0 
)


ament_target_dependencies(${PROJECT_NAME}_radar_front_node
  rclcpp
  sensor_msgs
  pcl_conversions
  ipc_helper
  carla_msgs_custom
)

ament_target_dependencies(${PROJECT_NAME}_rgb_front_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  carla_msgs_custom
)

ament_target_dependencies(${PROJECT_NAME}_control_msg_node
  rclcpp
  carla_msgs
  ipc_helper
  carla_msgs_custom
)

ament_target_dependencies(${PROJECT_NAME}_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  ipc_helper
  carla_msgs_custom
)

ament_target_dependencies(${PROJECT_NAME}_odometer_node
  rclcpp
  nav_msgs
  ipc_helper
  carla_msgs_custom
)

ament_target_dependencies(${PROJECT_NAME}_vehicle_status_node
  rclcpp
  carla_msgs
  ipc_helper
  carla_msgs_custom
  message_filters
)

ament_target_dependencies(${PROJECT_NAME}_gnss_node
  rclcpp
  sensor_msgs
  ipc_helper
  carla_msgs_custom
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}_rgb_front_node
  ${PROJECT_NAME}_radar_front_node
  ${PROJECT_NAME}_control_msg_node
  ${PROJECT_NAME}_vehicle_status_node
  ${PROJECT_NAME}_gnss_node
  ${PROJECT_NAME}_imu_node
  ${PROJECT_NAME}_odometer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
