cmake_minimum_required(VERSION 3.8)
project(ipc_helper)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
# Common message
set(COMMON_MSG
    "msg/Array1D.msg"
    "msg/Array2D.msg"
    "msg/Array3D.msg"
    "msg/Box.msg"
    "msg/Orientation.msg"
    "msg/Point2f.msg"
    "msg/RollPitchYaw.msg"
    "msg/Vector2f.msg"
    "msg/Vector3f.msg"
)

# IMU messsage
set(IMU_MSG
    "msg/ImuParameters.msg"
)

# Odometer message
set(ODOMETER_MSG
    "msg/Odometry.msg"
)

# GNSS messsage
set(GNSS_MSG
    "msg/GnssPoint.msg"
)

# Control command message
set(CONTROL_MSG
    "msg/Control.msg"
)

# Car status message
set(CAR_STATUS_MSG
    "msg/CarStatus.msg"
)

# Radar point message
set(RADAR_MSG
    "msg/Radar.msg"
    "msg/RadarCloudPoint.msg"
)

# Adas control message (IVI -> ADAS)
set(ADAS_CONTROL_MSG
    "msg/AdasControl.msg"
)

# Service command message (IVI -> ADAS)
set(SERIVCE_COMMAND_MSG
    "msg/ServiceCommand.msg"
)
 
# Service status message (ADAS -> IVI)
set(SERVICE_STATUS_MSG
    "msg/ServiceStatus.msg"
)

# SFF Debug message (AEB debug visualization)
set(SFF_DEBUG_MSG
    "msg/SFFClaimState.msg"
    "msg/SFFClaimSet.msg"
    "msg/SFFOverlapResult.msg"
    "msg/SFFDebugData.msg"
)

set(VISUALIZATION_MSG
    "msg/SensorSourcePair.msg"
    "msg/SensorSourceObject.msg"
    "msg/AdvanceFusionObject.msg"
    "msg/LaneMarking.msg"
    "msg/LanePointsRaw.msg"
    "msg/LaneDetection.msg"
    "msg/PlanningResults.msg"
    "msg/PerceptionResults.msg"
    "msg/PipelineResults.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${COMMON_MSG}
    ${IMU_MSG}
    ${ODOMETER_MSG}
    ${GNSS_MSG}
    ${CONTROL_MSG}
    ${RADAR_MSG}
    ${CAR_STATUS_MSG}
    ${ADAS_CONTROL_MSG}
    ${SERIVCE_COMMAND_MSG}
    ${SERVICE_STATUS_MSG}
    ${VISUALIZATION_MSG}
    ${SFF_DEBUG_MSG}
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()