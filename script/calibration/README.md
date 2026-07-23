# Camera Calibration and Bird's Eye View Calibration Instructions

This guide outlines the steps to capture images, perform camera calibration, and bird's eye view calibration in a simulated CARLA environment.

> **Note:** 
   - The term `calibration` folder refers to the directory located at: `adas_sdk/script/carla_script/scenarios/calibration` in the current system.
   - The images used for camera calibration and Bird's Eye View calibration are stored in the `calibration_images` branch of the `data` group at `playground/adas-hackathon1/development/adas/data`.
   
## Camera Calibration Procedure

1. **Position the Ego Vehicle**  
   Move the ego vehicle to a spacious area that allows vehicle to flexibly move around the calibration pattern.

2. **Generate a Calibration Pattern**  
   Use the provided script to generate a calibration pattern (e.g., chessboard or dot grid). The pattern should be:
   - Oriented perpendicular to the front camera's view.
   - Stationary throughout the calibration process.
   
   Example calibration patterns are available in the `calibration` folder: `draw_dot_grid.py` and `draw_chessboard.py`

3. **Capture Multiple Viewpoints**  
   Drive the ego vehicle around the pattern to capture images from a variety of angles and distances. This diversity is essential for accurate calibration.

4. **Save Images Using ROS**  
   Capture and save front camera images using the `image_view` ROS package. Images should be saved in PNG format to the `calibration/calibration_images` directory.

   First, source the ROS environment:
   Ensure you're in the SDK's top-level directory (`adas_sdk`)
   ```
   # Ignore the error like 'No such file or directory'
   source ros_bridge_support_ws/script/setenv.sh   
   ```
   Then, run the image saver:
   ```
   ros2 run image_view image_saver --ros-args \
      --remap image:=/carla/hero0/rgb_front/image \
      -p filename_format:=<output path>
   ```
   Example:
   ```
   ros2 run image_view image_saver --ros-args \
   --remap image:=/carla/hero0/rgb_front/image \
   -p filename_format:=$SCENARIOS_DIR/calibration/calibration_images/frame%04d.png
   ```
   **Tip**: Capture 10 to 15 frames for optimal calibration results.

5. **Execute the Calibration Script**      
   Run `camera_calibration.py`, located in the `calibration` folder, to compute the camera’s intrinsic and distortion parameters. The output will be saved to:
   ``` 
   adas_sdk/common/config/sensor/camera_calibration_parameters.txt
   ```

6. **Set the weather in CARLA**  
   To capture images from the CARLA camera under various weather conditions, run the script `set_carla_weather.py` located in the `adas_sdk/script/carla_script` folder. This script allows you to configure different weather settings.

## Top-down View Calibration Procedure

1. **Position the Ego Vehicle**  
   Move the ego vehicle to a spacious area free of objects that could obstruct the camera view.

2. **Generate a carpet in the simulation environment**  
   Use the provided script to generate a carpet in the CARLA simlation.
   An example pattern is available in the `calibration` folder: `roadPain.py`.

3. **Capture the carpet image**  
   Use `cv::imwrite()` function to capture an image of the carpet in the service.  
   The image should be saved in the folder: ` calibration/bird_eye_view_calibration_images` 

4. **Execute the Calibration Script**  
   - Open `bird_eye_view_calibration.py` in the `calibration` folder.  
   - Configure the script parameters according to the values used in `roadPain.py` and your ego vehicle settings.  
   - Run the script and manually select the four corners of the carpet.  
   - The resulting calibration parameters will be saved to:  
   `
   adas_sdk/common/config/sensor/camera_calib_carla.txt
   `

5. **Manual tunning calibration result**  
   As perspective transform is very sensitive to error. You should adjust the pixel value in calib file afterward.

   First, setup for debugging:  
   - Uncomment the 35m rectangle drawing in `Lanes.qml`
   - Run ADAS Service in debug mode
   - Run carla manual control
   - Press tab until switched to Top-down view
   - Run script to spawn 35x35 area
      ```
      # Source Carla Environment
      source script/carla_script/setup_env_carla.sh

      # Run script
      cd script/calibration
      python3 roadPain.py --debug
      ```
   - Run 
      ```
      xprop -f _NET_WM_WINDOW_OPACITY 32c -set _NET_WM_WINDOW_OPACITY $(printf 0x%x $((0xffffffff * 40 / 100)))
      ```
      then select manual control window to make it transparent
   - Drag the manual control window to match its rectangle with the rectangle on ADAS Service Top-downview

   Then, adjust pixel value in camera calib file to match ADAS Top-down view with carla Top-down view.