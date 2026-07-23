# CARLA Spawn Object Configuration

This file defines the spawn configuration for a simulation scenario in the CARLA Simulator, specifying the ego vehicle and its associated sensors.

## The general structure
```json
{
"objects":
    [
        {
            "type": "<SENSOR-TYPE>",
            "id": "<NAME>",
            "spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            <ADDITIONAL-SENSOR-ATTRIBUTES>
        },
        {
            "type": "<VEHICLE-TYPE>",
            "id": "<VEHICLE-NAME>",
            "spawn_point": {"x": -11.1, "y": 138.5, "z": 0.2, "roll": 0.0, "pitch": 0.0, "yaw": -178.7},
            "sensors":
                [
                <SENSORS-TO-ATTACH-TO-VEHICLE>
                ]
        }
        ...
    ]
}
```

# Explain example minimal_configuration
**Vehicle**
- Type: vehicle.tesla.model3
- ID: hero0
This is the main ego vehicle in the simulation.

**Attached Sensors**
1. RGB Camera
    - Type: sensor.camera.rgb
    - ID: rgb_front
    - Mount Position:
      - x: 2.2 meters (in front of vehicle)
      - y: 0.0 meters
      - z: 2.0 meters (above ground)
    - Field of View (FOV): 90°
    - Resolution: 800 × 600 pixels

   Purpose: Used for front-facing image capture.

2. Radar Sensor
   - Type: sensor.other.radar
   - ID: radar_front
   - Mount Position:
     - x: 2.2 meters
     - y: 0.0 meters
     - z: 0.5 meters
   - Field of View:
     - Horizontal: 30°
     - Vertical: 10°
   - Points Per Second: 50,000
   - Range: 250 meters

    Purpose: Detect objects and motion in front of the vehicle.
3. GNSS (GPS) Sensor
   - Type: sensor.other.gnss
   - ID: gnss
   - Mount Position:
     - x: 0.0
     - y: 0.0
     - z: 2.8
   - Noise: Disabled (no bias or standard deviation)

    Purpose: Provides geographic location with ideal precision (no noise).
4. IMU Sensor
   - Type: sensor.other.imu
   - ID: imu
   - Mount Position:
     - x: 0.0
     - y: 0.0
     - z: 2.8
   - Noise Parameters: All set to zero (ideal readings)

   Purpose: Measures acceleration and angular velocity of the vehicle.

> Find documentation about the CARLA Spawn Objects package [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/stable/carla_spawn_objects/).