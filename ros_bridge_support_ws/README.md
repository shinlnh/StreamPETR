- [ADAS ROS bridge workspace](#adas-ros-bridge-workspace)
  - [The overall structure of this work space](#the-overall-structure-of-this-work-space)
  - [Build workspace](#build-workspace)
  - [How to run workspace](#how-to-run-workspace)
    - [1. Start the Carla server](#1-start-the-carla-server)
    - [2. Launch `bridge` in package `carla_ros_bridge` to create environment](#2-launch-bridge-in-package-carla_ros_bridge-to-create-environment)
    - [3. Launch the ADAS bridge](#3-launch-the-adas-bridge)
  - [Quick start guide to run ADAS bridge](#quick-start-guide-to-run-adas-bridge)
    - [Set up environment](#set-up-environment)
    - [Run ADAS Bridge](#run-adas-bridge)
  - [For individual node stream](#for-individual-node-stream)

# ADAS ROS bridge workspace
This is the support ROS bridge to stream and receive information between the carla server and adas app client.
The bridge stored in __ros_bridge_support_ws__ folder and built base on ROS 2 Foxy. 
## The overall structure of this work space
<pre>
ros_bridge_support_ws
├── build                       <--------------------- folder created when build
├── install                     <--------------------- contain script to install custom package
├── log                         <--------------------- log folder
├── config                      <--------------------- contain configuration file for CARLA 
├── script                      <--------------------- contain script for bridge execution 
├── launch                      <--------------------- launch configuration file
├── <b>src</b>                         <--------------------- contain source code
│   ├── <b>adas_bridge</b>             <--------------------- the adas_bridge package
│   │   ├── include
│   │   ├── script
│   │   │   └── carla_api       <--------------------- get CARLA data and publish to ROS node
│   │   ├── <b>src</b>
│   │   │   ├── control_msg     <--------------------- handle control for ego vehicle
│   │   │   ├── radar_front     <--------------------- handle radar streaming
│   │   │   ├── rgb_front       <--------------------- handle camera streaming
│   │   │   ├── imu             <--------------------- handle data from IMU sensor
│   │   │   ├── gnss            <--------------------- handle data from GNSS sensor
│   │   │   ├── carla_api       <--------------------- handle data from CARLA API node
│   │   │   └── vehicle_status  <--------------------- handle vehicle info streaming
│   │   │   └── odometer        <--------------------- handle data from Odometer sensor
│   │   ├── package.xml         <--------------------- package configuration
│   │   └── CMakeLists.txt      <--------------------- CMakeLists file
│   ├── carla_msgs_custom       <--------------------- custom message for sending info
│   └── example
└── README.md                   <--------------------- <b>you are here</b>
</pre>
If folders or files are not mentioned then those are depricated and unrelated to the bridge and will likely be removed in the future.
## Build workspace
```
./script/build.sh
```
The script accepts a single parameter: `-j`, which specifies the number of parallel jobs that colcon can use when building packages. For example, `./script/build.sh -j 4` allows up to 4 packages to be built in parallel, improving compilation efficiency.

**Importance note:** In the current project, there are only two packages: `adas_bridge` and `carla_msgs_custom`. Since `adas_bridge` depends on `carla_msgs_custom`, this feature is not applicable.

To quickly clean up the old build:
```
./script/clean.sh
```
## How to run workspace
***Note:*** Make sure that you are in this workspace. You can skip this section and go straight to the [Quick start guide to run ADAS bridge](#quick-start-guide-to-run-adas-bridge) section for an `all-in-one` setup instead of launching each component individually as described in this section.
### 1. Start the Carla server 
```
# Start with headless mode
# SDK_PATH refers to the root directory of the ADAS SDK workspace after cloning the full repository
./$SDK_PATH/script/carla_script/run_carla.sh -r OFF
```

### 2. Launch `bridge` in package `carla_ros_bridge` to create environment
```
./script/run_carla_bridge.sh
```
This script will create the environment, spawn object and start the python script to control the ego vehicle.

### 3. Launch the ADAS bridge
```
./script/run_adas_bridge.sh
```
This support 2 mode:
1. `local` mode
```
./script/run_adas_bridge.sh --mode local
```
2. `board` mode
```
./script/run_adas_bridge.sh --mode board
```
## Quick start guide to run ADAS bridge
To run all ROS bridge nodes, simply execute the following shell script. It will source the environment and run the bridge with the required parameters.

### Set up environment
Run script below to set several environments variables
```
source script/setup_runtime_env.sh
``` 

### Run ADAS Bridge
Command:
```
./script/run_all.sh --mode local
```
About the params and configuraiton of the launch file: 
 - __--mode:__ `local` or `board` the ip of local and board is define in configuration file, local for PC and board for streaming to the Jetson.

Examples:
```
# Local mode
./script/run_all.sh --mode local --ros-args town:=Town03 objects_definition_file:=$SDK_PATH/common/config/carla/vehicle_1.json
```

```
# Board mode
./script/run_all.sh --mode board --ros-args town:=Town03 objects_definition_file:=$SDK_PATH/common/config/carla/vehicle_1.json board_ip:=192.168.240.110
```

Incase you just want to run the bridge in default value, the default DDS domain ID will be 0, with default node name and default vehicle. If you want to deploy in difference DDS domain ID, please change the environment paramenter name `ROS_DOMAIN_ID`
```
# Example: Using DDS Domain ID = 42
export ROS_DOMAIN_ID=42
# Check current ROS_DOMAIN_ID
printenv ROS_DOMAIN_ID
```
When `ROS_DOMAIN_ID` is unset, DDS uses domain ID 0 by default

Running the shell script will automatically start all the nodes for you using your configuration, based on your current directory. Please ensure your directory structure and the configuration in the common folder are correct before running the launch script. The script will source all necessary files, so you don't need to source anything manually.

**Importance note:** To use a specific domain ID (as shown in the example above), ensure that `ROS_DOMAIN_ID` is set to the same value in all terminals
```
# Terminal for script run_all.sh
export ROS_DOMAIN_ID=42
./script/run_all.sh --mode local
```
```
# Terminal for running ADAS Service
# Ensure that you are in the top-level SDK folder
export ROS_DOMAIN_ID=42
./script/run_debug.sh
```

## For individual node stream
Incase you need to run one node only (for debugging purpose), you can do it as follow.
First, before run any node, you have to source: 
```
source script/setup_runtime_env.sh
```
For example, ROS node for sending IMU data:
```
ros2 run adas_bridge adas_bridge_imu_node --ros-args -p vehicle_name:="hero0"
```
- `vehicle_name`: Vehicle's name in CARLA Simulator
