- [0. Setup Enviroment](#0-setup-enviroment)
- [1. Run Scenario Script](#1-run-scenario-script)
  - [2. Directory Descriptions](#2-directory-descriptions)
    - [ACC](#acc)
    - [AEB](#aeb)
    - [LKS](#lks)
    - [Perception](#perception)
    - [TJA](#tja)
  - [3. Term Decoding](#3-term-decoding)

# 0. Setup Enviroment
1. Navigate to the carla_script directory:
```
cd adas_sdk/script/carla_script
```
2. Set up the environment for CARLA:
```
source ./setup_env_carla.sh
```
# 1. Run Scenario Script
1. Navigate to the scenario directory
```
cd scenario
```
2. Run scenario
```
/usr/bin/python /prj/common/carla/scenario_runner-0.9.13/scenario_runner.py --openscenario <your_scenario>.xosc --timeout 10000 --host localhost --port 2000 --waitForEgo
```
3. Example
```
/usr/bin/python /prj/common/carla/scenario_runner-0.9.13/scenario_runner.py --openscenario acc/ACC_MiddleLane_ForwardVehicleSwervingSpeed50.xosc --timeout 10000 --host localhost --port 2000 --waitForEgo
```
## 2. Directory Descriptions

### acc
This directory contains scenarios for **Adaptive Cruise Control (ACC)**.

For Example:
- **Left Lane:**
  - ACC_LeftLane_ForwardVehicleSpeed60.xosc
- **Middle Lane:**
  - ACC_MiddleLane_ForwardVehicleDistance150.xosc
  - ACC_MiddleLane_ForwardVehicleSpeed60.xosc
  - ACC_MiddleLane_ForwardVehicleSwervingSpeed50.xosc
- **Right Lane:**
  - ACC_RightLane_ForwardVehicle_Speed60.xosc

### aeb
This directory contains scenarios for **Autonomous Emergency Braking (AEB)**:

For Example:
- AEB_MiddleLane_3ForwardVehicleStop.xosc

### lks
This directory contains scenarios for **Lane Keeping Systems (LKS)**:

For Example:
- LKS_LeftLane.xosc
### perception
This directory contains **perception** scenarios:

For Example:
- Localization.xosc
- ObjectDetection.xosc
- ObjectTracking.xosc

### tja
This directory contains scenarios for **Traffic Jam Assist (TJA)**:
For Example:
- TJA_LeftLane_4ForwardVehicleSpeed7.xosc
  
&nbsp;
## 3. Term Decoding
**ForwardVehicleSpeedXX**: Indicates the speed of the forward vehicle in the scenario.
  - `ForwardVehicleSpeed20`: The forward vehicle is traveling at 20 km/h.
  - `ForwardVehicleSpeed40`: The forward vehicle is traveling at 40 km/h.
  - `ForwardVehicleSpeed60`: The forward vehicle is traveling at 60 km/h.

**ForwardVehicleDistanceXXX**: Indicates the distance to the forward vehicle in the scenario.
  - `ForwardVehicleDistance150`: The distance to the forward vehicle is 150 meters.

**SwervingSpeedXX**: Indicates the speed of the swerving vehicle in the scenario.
  - `SwervingSpeed50`: The swerving vehicle is traveling at 50 km/h.

**LaneXX**: Indicates the lane in which the scenario is set.
  - `LeftLane`: The scenario is set in the left lane.
  - `MiddleLane`: The scenario is set in the middle lane.
  - `RightLane`: The scenario is set in the right lane.