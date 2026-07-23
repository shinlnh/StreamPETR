# Quick start guide on how to use Carla scripts

- [Quick start guide on how to use Carla scripts](#quick-start-guide-on-how-to-use-carla-scripts)
  - [1. Start CARLA simulation enviroment in UE4](#1-start-carla-simulation-enviroment-in-ue4)
  - [2. Launch the desired Open Scenario files for testing and calibrating.](#2-launch-the-desired-open-scenario-files-for-testing-and-calibrating)
    - [For testing scenario](#for-testing-scenario)
    - [For calibrating scenario](#for-calibrating-scenario)

All the script should be done at directory: `script/carla_script/`.\
Several scripts
 - Start up the UE4 3D envrioment.
 - Start up the specific scenario for testing  


## 1. Start CARLA simulation enviroment in UE4
Run the CARLA server:
 - option `-g` stand for graphic level with 2 option L (Low) and E(Epic)
 - option `-r` stand for rendering mode, with 2 option ON and OFF, the OFF mode will not render carla world window.
```
./run_carla.sh -g L -r ON
```
Wait for the world load completely before moving to the next step. 

## 2. Launch the desired Open Scenario files for testing and calibrating.
Scripts folder provided with multiple scenarios for testing and a specific scenario for calibration camera.

### For testing scenario
To create a scenario for testing first, you also need to source the terminal first before running the following scenario: 

```
source setup_env_carla.sh
/usr/bin/python /prj/common/carla/scenario_runner-0.9.13/scenario_runner.py --openscenario <your_scenario> --timeout 10000 --host localhost --port 2000 --waitForEgo
```
The scenarios will be store in scenario folder in form of .xosc file, the folder will have structure like this:
```
└── carla_script  
    └── *scenarios
       ├── acc        <--------- .xosc file for ACC testing cases
       ├── aeb        <--------- .xosc file for AEB testing cases
       ├── lks        <--------- .xosc file for LKS testing cases
       ├── perception <--------- .xosc file for perception testing cases
       └── tja        <--------- .xosc file for TJA testing cases
```
The scenario file will have format: `ObjectTracking_Scenario.xosc`.
### For calibrating scenario
To run the Python script using to calibrate for camera, you will spawn your ego car, then run the following script roadPaing.py with acccordingly parameter.
```
usage: roadPain.py [-h] [--distance DISTANCE] [--width WIDTH] [--length LENGTH]

Draw a rectangle in front of a vehicle in CARLA simulator.

optional arguments:
  -h, --help            show this help message and exit
  --distance DISTANCE   Distance from the front of the vehicle to the rectangle (in meters)
  --width WIDTH         Width of the rectangle (in meters)
  --length LENGTH       Length of the rectangle (in meters)
```
The script will pain a rectangle with distance (from your car's rear), width and length according to your params. In the ADAS APPS, navigate to the calibrate tab and you can capture the image to create 4 points for calibrate.\
More more detail please read the README in the scenario folder.\
After start up the CARLA enviroment, you can go to `/adas_sdk/ros_bridge_support_ws` to start the bridge for streaming, more detail will be in the README file at the same directory.

