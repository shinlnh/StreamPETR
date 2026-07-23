
# adas_sdk
SDK for Banvien ADAS hackathon

- [adas\_sdk](#adas_sdk)
  - [Project structure](#project-structure)
  - [Run the application and start the ROS for streaming](#run-the-application-and-start-the-ros-for-streaming)

## Project structure
<pre>
*adas_sdk
├── *adas_service
├── *adas_visualize_service
├── build                                   <------------ the build directory for adas apps
├── common                                  <------------ contain variable and configurations
├── ipc_helper
├── *ros_bridge_support_ws                  <---- ros bridge work space 
├── *script  
│   ├── ai_script                           <------------ script for training and deploy model
│   ├── *carla_script                       <--------- script for carla run and setup
│   │   └── *scenarios                      <------ script for scenario
│   ├── build.sh                            <------ cross-compilation script for the target board
│   ├── pc_build.sh                         <------ cross-compilation script for the target board
│   ├── setup_adas_service_runtime_env.bash <------ source script for board & pc run 
│   └── *test_scripts
│      ├── run_test_scripts.sh              <------ script for running unit test and integration test
│      ├── generate_test_scripts.sh         <------ script for generating coverage reports
│      └── README.md
├── CMakeLists.txt
└── README.md                               <------------ <b>right now README.md file</b>
</pre>
## Run the application and start the ROS for streaming
The application consist of three main components:
 - The adas service app.
 - The 3d simulate enviroment where we run our car. 
 - The bridge to connect between the app and the enviroment.

Application can be run on board and on PC:
- To launch the 3D environment CARLA and test scenarios please read the README file in: 
  - [Start up CARLA guide](./script/carla_script/README.md#-Quick-start-guide-on-how-to-use-Carla-scripts) for 3D environment setup
  - [Run scenarios guide](./script/carla_script/scenarios/README.md) for launching specific scenario.
- To run the application and start the ROS bridge to stream and control the car. Please read the README in the:
  - [__adas_service__](./adas_service/README.md#-ADAS-service) 
  - [__adas_visualize_service__](./adas_visualize_service/README.md#-ADAS-visualize-service) 
  - [__ros_bridge_support_ws__](./ros_bridge_support_ws/README.md#-ADAS-Bridge)
  - [__can_service__](can_service/README.md)
- To run Unit Test and Integration Test:
  - [__Build/Run Test Guide__](./script/test_scripts/README.md#GTEST-for-UT/IT) 

