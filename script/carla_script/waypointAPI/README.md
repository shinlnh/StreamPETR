# Get Waypoints Module
It performs tasks such as collecting waypoints, transforming coordinates, and saving data to files.
## How It Works
1. Initialize the CARLA Client:

    - The program connects to the CARLA server running on localhost at port 2000.
2. Vehicle Identification:

    - The program identifies the vehicle with a specific `VEHICLE_ID` within the CARLA world. If the vehicle isn't found, it retries for up to 2 seconds.
3. Waypoint Collection:

    - The program collects waypoints along the vehicle's current lane for a specified distance (`max_distance`), storing them in the world coordinate system.

4. Coordinate Transformation:

    - The waypoints are transformed into the vehicle's local coordinate system using a change of basis matrix derived from the vehicle's current orientation.

5. Saving Waypoints:

    - The program writes both the raw and transformed waypoints to CSV files. It uses threads to handle file writing and waypoint collection simultaneously.

6. Real-Time Waypoint Tracking:

    - A real-time tracking function collects waypoints as the vehicle moves, storing them in a file. These waypoints are then transformed into the vehicle's coordinate system.
## Files
1. `data/raw_point.csv:` Stores the raw waypoint data in the world coordinate system.
2. `data/plan_point.csv:` Stores the transformed waypoint data in the vehicle's coordinate system.
3. `data/real_point.csv:` Stores the real-time collected waypoints, also transformed into the vehicle's coordinate system.

## Some importance variable
1. `TIMEOUT` : This is the time program retry to get all vehicle. (default: 2s)
2. `LANE_ERROR` : When it come to the end of current lane, it will try to jump into a near lane. This is maximum jump step. (default: 1m)
3. `VEHICLE_ID` : The ID of vehicle which you want to get waypoint (default: 145)
## Usage
1. Running the Program:
```
python get_waypoint.py
Enter the expected distance: 
```
The program will request user enter distance traveled by the car (unit: meter)

# Process Waypoint Module
This module will collect data from all file CSV and draw on single window.

To run this module:
```
python process_waypoint.py
```
This program will save a graph in folder `data/`. If you want to show this immediately, you will replace all lines contain `plt.savefig` with `plt.show()`