cmake_minimum_required(VERSION 3.12)

project(adas_visualize_service VERSION 0.1 LANGUAGES CXX)

# Define option
option(ENABLE_CAN "Switch to CAN Mode" OFF)

# ====== 1. SETUP VARIABLE SECTION ======
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

# ====== 2. FIND PACKAGE SECTION ======
# Find package that required for building
find_package(QT NAMES Qt5 REQUIRED COMPONENTS Widgets Concurrent Core Quick)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Concurrent Core Quick)
find_package(OpenCV REQUIRED)
find_package(X11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread chrono system REQUIRED)

# ROS package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ipc_helper REQUIRED)
find_package(carla_msgs_custom REQUIRED)

# Gstreamer
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)
pkg_check_modules(GST_RTP REQUIRED gstreamer-rtp-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GST_INCLUDE_DIRS}
    ${GST_RTSP_SERVER_INCLUDE_DIRS}
    ${GST_RTP_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
    # 
    ${PROJECT_SOURCE_DIR}/app
    ${PROJECT_SOURCE_DIR}/app/sensor_config
    ${PROJECT_SOURCE_DIR}/adas_main_visualize
    # Common
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/common/src/inc
    ${CMAKE_SOURCE_DIR}/common/src/inc/can_manager
    ${CMAKE_SOURCE_DIR}/common/src/inc/dbc
    ${CMAKE_SOURCE_DIR}/common/ipc_helper
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/interface/inc
    # Sensor
    ${CMAKE_SOURCE_DIR}/common/config
    # Perception Common
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/perception
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/perception/utils/object_tracking/hungarian_matching
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/perception/utils/object_tracking/kalman_filter/linear_kalman_filter
    # Planning Commonn
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/planning
    # Communication Common
    ${CMAKE_SOURCE_DIR}/common/communications
)

if(ENABLE_CAN)
    add_definitions(-DENABLE_CAN)
endif()

# UI
set(PROJECT_SOURCES_UI
    ${PROJECT_SOURCE_DIR}/app/MainViewController.cpp
    ${PROJECT_SOURCE_DIR}/app/key_handler.cpp
    ${PROJECT_SOURCE_DIR}/app/TopDownViewController.cpp
    ${PROJECT_SOURCE_DIR}/app/sensor_config/SensorViewConfig.cpp
    ${PROJECT_SOURCE_DIR}/app/sensor_config/RadarViewConfig.cpp
    ${PROJECT_SOURCE_DIR}/app/sensor_config/CameraViewConfig.cpp
    ${PROJECT_SOURCE_DIR}/app/PaintItem.cpp
    ${PROJECT_SOURCE_DIR}/app/AEBDebugWindow.cpp

    ${PROJECT_SOURCE_DIR}/app/qml.qrc
    ${PROJECT_SOURCE_DIR}/app/main.qml

    # Common
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/common/TextButton.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/common/ImageTextButton.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/common/CardContainer.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/common/CustomButton.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/common/ToggleSwitch.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/common/CustomTextInput.qml

    # Dialogs
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/dialogs/WarningDialog.qml

    # Header
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/header/Header.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/header/HeaderIcon.qml
    
    # Control Panel 
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/ControlPanel.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/StatusInfo.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/TopDownView.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/MiniTopDownView.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/AdjustButton.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/GirdRound.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/Objects.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/Lanes.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/PathPlanning.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/FOV.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/MiniFOV.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/SettingsStatusInfo.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/control/qmldir
    
    
    # Camera Panel
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/camera/CameraPanel.qml

    # Settings Panel
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/settings/SettingsPanel.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/settings/FpsWindow.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/settings/SettingsState.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/components/settings/qmldir

    # Style
    ${PROJECT_SOURCE_DIR}/app/resources/qml/style/Theme.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/style/ObjectStyles.qml
    ${PROJECT_SOURCE_DIR}/app/resources/qml/style/qmldir
)

list(APPEND PROJECT_SOURCES_MAIN
    ${PROJECT_SOURCE_DIR}/adas_main_visualize/adas_main_visualize.cpp
)

list(APPEND PROJECT_SOURCES_UTILS
    ${CMAKE_SOURCE_DIR}/common/src/inc/can_manager/can_manager.cpp
    ${CMAKE_SOURCE_DIR}/common/src/inc/dbc/tesla.c
)

list(APPEND PROJECT_SOURCES_COMMON
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/intermediate_representations.cpp
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/interface/src/adas_service_if.cpp
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/interface/src/msg_utils.cpp
    # Perception
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/perception/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter_for_cam.cpp
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/perception/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter_for_radar.cpp
    ${CMAKE_SOURCE_DIR}/common/ipc_helper/perception/utils/object_tracking/kalman_filter/linear_kalman_filter/linear_kalman_filter.cpp
    # Communications
    ${CMAKE_SOURCE_DIR}/common/communications/Receiver.cpp
    ${CMAKE_SOURCE_DIR}/common/communications/ReceiverRtsp.cpp
    
)

add_executable(${PROJECT_NAME}
                ${PROJECT_SOURCES_UI}
                ${PROJECT_SOURCES_UTILS}
                ${PROJECT_SOURCES_MAIN}
                ${PROJECT_SOURCES_COMMON}
                main.cpp)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES 
        AUTOMOC ON 
        AUTOUIC ON
        AUTORCC ON
)

set(LINK_LIBS_LIST 
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    Qt${QT_VERSION_MAJOR}::Core 
    Qt${QT_VERSION_MAJOR}::Quick
    Qt${QT_VERSION_MAJOR}::Qml
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    ${GST_LIBRARIES}
    ${GST_RTSP_SERVER_LIBRARIES}
    ${GST_RTP_LIBRARIES}
    ${GST_APP_LIBRARIES}
    -lgstrtspserver-1.0
)

target_link_libraries(${PROJECT_NAME}
                        ${LINK_LIBS_LIST} 
)

ament_target_dependencies(
    ${PROJECT_NAME}
    rclcpp
    ipc_helper
    carla_msgs_custom
)