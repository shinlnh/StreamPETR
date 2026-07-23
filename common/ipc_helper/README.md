- [IPC Helper - ROS message](#ipc-helper---ros-message)
  - [How to add the new ROS message](#how-to-add-the-new-ros-message)
    - [1. Create the ROS message](#1-create-the-ros-message)
    - [2. Update CMakeLists.txt](#2-update-cmakeliststxt)
    - [3. Update package.xml](#3-update-packagexml)

# IPC Helper - ROS message
<pre>
ipc_helper
├── src
│   ├── msg                        <------------ ROS messages directory
│   │   ├── ImuParameters.msg      <------------ ROS message file
│   │   ├── GnssPoint.msg
│   │   ├── Radar.msg
│   │   └── ...
│   ├── CMakeLists.txt             <------------ CMakeLists.txt file
│   └── package.xml                <------------ package configuration
└── README.md                      <------------ <b>You are here</b>
</pre>

## How to add the new ROS message
### 1. Create the ROS message
Define the new message and place at folder `msg`. Can create support ROS message when creating a complex message \
**Importance:** Using CamelCase when naming the ROS message. e.g. `ImuParameters.msg`

Example:
```
uint64 time_stamp
ipc_helper/Orientation imu_orientation
ipc_helper/Vector3f imu_angular_velocity
ipc_helper/Vector3f imu_linear_acceleration
ipc_helper/RollPitchYaw imu_roll_pitch_yaw
```
### 2. Update CMakeLists.txt
- Update package dependencies (Optional)
```cmake
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
```
- Define the variable to contain all message
```cmake
# Radar point message
set(RADAR_MSG
    "msg/Radar.msg"
    "msg/RadarCloudPoint.msg"
)
```
- Add this variable to `rosidl_generate_interfaces`
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
    ...
    ${RADAR_MSG}
    ...
    DEPENDENCIES std_msgs
)
```
### 3. Update package.xml
If no additional package dependencies are needed, please skip this section.
- Add the package dependencies
```xml
<depend>std_msgs</depend>
...
```