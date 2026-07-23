# Perception module for BV ADAS app

## Description
The Perception module is a critical part of our Advanced Driver Assistance System (ADAS), responsible for processing and interpreting sensor data to understand the vehicle's surroundings. By integrating multiple views and coordinate systems, it enables accurate environmental perception and supports to support both planning and control functions within the ADAS software.

Important Distinctions:

- **Camera View**: The raw image captured by the camera, represented purely in pixels.

- **Bird's-Eye/Top-Down View**: An image that has been transformed to provide a top-down perspective, but still represented in pixels. Two matrices, 𝑀 and 𝑀_inv, are used to convert between the camera view and the top-down view. The distance estimation module takes a pixel coordinate as input and returns the corresponding real-world distance (x, y offset in meters).

- **Real-World View**: Coordinates are given in meters, representing actual distances in the real world. Real-world distances can be estimated from a given image using calibration files.

## Overall folder structure:
- `./lane_detection`: source code for lane tracking algorithm and related script/tool for testing lane track algorithm.
- `./object_detect`: source code for implement object detection model.
- `./distance_estimator`: source code for calculating distance-to-object based on object detector's output.

For each algorithm/model, its folder organization is described sperately but share a common configuration below:
- `./<module_name>/src`: contains `.cpp` or `.c` file(s) for implement algorithm.
- `./<module_name>/inc`: contains heading file(s) `.hpp` or `.h`


## 1. `lane_detection`
Inside the `./lane_detection` directory:
- `/src`
- `/inc`
- `/test`: script/tool for testing purpose

The algorithm will take input as image from camera. This data then will be computed to give the **output** of *lane-tracking points* and *error in angle*.

## 2. `object_detect`
Inside the `./object_detect` directory:
- `/src`
- `/inc`
- `/model`: contains model file
- `/weight`:(if needed) weight of model
- `/test`: script/tool for testing purpose

The **output** of this module is vector-like data about center-point of objects.

## 3. `distance_estimator`
Inside the `./distance_estimator` directory:
- `/src`
- `/inc`
- `/test`: script/tool for testing purpose

The **output** of this module is distance to near objects combined with `object_detect` module

## 1. Undistortion
- void undistorPipeline(const cv::Mat &img)
    + Implement undistortion algorithm
- cv::Mat getUndistorImage(void)
    + Get undistored image

## 2. Lane detection
- void laneDetectorPipeline(const cv::Mat &img)
    + Implement lane detection algorithm
- void setOutModeLaneDetect(OUT_IMAGE_LANE_DETECT outMode) && OUT_IMAGE_LANE_DETECT getOutModeLaneDetect(void) {return outModeLaneDetect;};
    + Select output image type
    + OUT_IMAGE_LANE_DETECT::NONE             (Black background image)
    + OUT_IMAGE_LANE_DETECT::THRESH_IMAGE     (The input image is processed by filters - Camera view)
    + OUT_IMAGE_LANE_DETECT::SET_WINDOW_IMAGE (Image contains sliding windows - Top down view)
    + OUT_IMAGE_LANE_DETECT::SET_LANE_IMAGE   (Image with detected lane - Camera view)
- std::vector<std::vector<float>> getVerticesPoints(void)
    + Returns the four vertices of the detected lane region
- cv::Mat getLaneDetectImage(void)
    + Get the image of the lane detection results

## 3. Object detection
- void objectDetectionPipeline(const cv::Mat &img);
    + Implement object detection algorithm
- void setOutModeObjectDetect(OUT_IMAGE_OBJECT_DETECT outMode) && OUT_IMAGE_OBJECT_DETECT getOutModeObjectDetect(void)
    + Select output image type
    + OUT_IMAGE_OBJECT_DETECT::NONE             (Black background image)
    + OUT_IMAGE_OBJECT_DETECT::BBOX             (Image with all detected object bouding boxs - Camera view)
    + OUT_IMAGE_OBJECT_DETECT::BBOX_ID          (Image with all detected object bouding boxs and classID - Camera view)
    + OUT_IMAGE_OBJECT_DETECT::BBOX_ID_DISTANCE (Image with all detected object bouding boxs, classID and estimated distance - Camera view)
- std::vector<TrafficObject> getDetectedObjects(void)
    + Returns information about detected objects (bbox - classID - distance)
- cv::Mat getObjectDetectImage(void)
    + Get the image of the object detection results