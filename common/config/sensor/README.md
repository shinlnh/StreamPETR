# Sensor Configuration README

This document explains the sensor configuration file format and highlights an important difference in coordinate conventions for camera sensors compared to other sensors.

---

## 1. Configuration File Format

Each sensor is configured using the following parameters:

- **type**: The sensor type (e.g., `external_sensor`).
- **id**: A unique identifier for the sensor (e.g., `rgb_front`).
- **coordinate**: A list of six values defining the sensor’s position and orientation. The format is: x, y, z, roll, pitch, yaw

- **x, y, z**: These values specify the sensor's translation (position). In most cases, you can ignore these if you are only concerned with orientation.
- **roll, pitch, yaw**: These values specify the sensor's rotation in the local coordinate system.

- **horizontal_fov**: The sensor’s horizontal field of view in degrees.
- **vertical_fov**: The sensor’s vertical field of view in degrees.
- **image_size_x**: The width (in pixels) of the captured image.
- **image_size_y**: The height (in pixels) of the captured image.

---

## 2. Coordinate Orientation Conventions

### Coordinate Orientation Conversion

**Standard Sensors (e.g., Lidar, Radar):**
- **x-axis**: Forward
- **y-axis**: Left
- **z-axis**: Upward  

**Camera Sensors (e.g., `rgb_front`):**
- **x-axis**: Right
- **y-axis**: Downward
- **z-axis**: Forward  

**Conversion Equation:**
- **camera_roll**: -pitch
- **camera_pitch**: -yaw
- **camera_yaw**: roll
