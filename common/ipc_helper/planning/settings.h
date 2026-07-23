#ifndef __PATH_PLANNING_VISUALIZER_SETTINGS_H__
#define __PATH_PLANNING_VISUALIZER_SETTINGS_H__

//* Settings that impace performance. Comment below codes to disable
#define ENABLE_ANTI_ALIASING
#define VISUALIZE_WHEEL

//Car dimensions. (in meters). Model after the Toyota Corolla
#define CAR_LENGTH                  4.46
#define CAR_WIDTH                   1.82
#define CAR_WHEEL_DIAMETER          0.45
#define CAR_WHEEL_WIDTH             0.19
#define CAR_WHEEL_BASE              3.0046463 // [unit] = m

//Just some coloring.
#define BLACK_OPENCV_SCALAR         cv::Scalar(0,0,0)
#define BLUE_OPENCV_SCALAR          cv::Scalar(255,153,102)
#define GRAY_OPENCV_SCALAR          cv::Scalar(153,153,153)
#define GREEN_OPENCV_SCALAR         cv::Scalar(0,182,0)
#define RED_OPENCV_SCALAR           cv::Scalar(0,0,255)
#define YELLOW_OPENCV_SCALAR        cv::Scalar(0,230,230)
#define CANVAS_BACKGROUND_COLOR     cv::Scalar(47,47,47)
#define UI_BACKGROUND_WIDGET_COLOR  cv::Scalar(30,30,30)
#define CAR_COLOR                   cv::Scalar(217,217,217)
#define WHEEL_COLOR                 cv::Scalar(31,31,31)
#define EDGE_LANE_COLOR             cv::Scalar(0,254,249)
#define CROSSABLE_LANE_COLOR        cv::Scalar(97,97,97)
#define SELECTED_PATH_COLOR         cv::Scalar(247,244,57)
#define COLLIDED_PATH_COLOR         cv::Scalar(82,80,236)
#define PATH_COLOR                  cv::Scalar(217,199,127)
#define OBJECT_COLOR_CLOSE          cv::Scalar(63,113,254)
#define OBJECT_COLOR_MEDIUM         cv::Scalar(122,218,255)
#define OBJECT_COLOR_FAR            cv::Scalar(129,255,199)

#define RADAR_COLOR_CLOSE           cv::Scalar(255,0, 0)     // Dark Blue (Radar Close)
#define RADAR_COLOR_MEDIUM          cv::Scalar(255,0, 0)    // Moderate Blue (Radar Medium)
#define RADAR_COLOR_FAR             cv::Scalar(255,0, 0)   // Light Blue (Radar Far)

#define CAMERA_COLOR_CLOSE          cv::Scalar(0, 255, 0)     // Bright Red (Camera Close)
#define CAMERA_COLOR_MEDIUM         cv::Scalar(0, 255, 0)    // Bright Yellow-Orange (Camera Medium)
#define CAMERA_COLOR_FAR            cv::Scalar(0, 255, 0)    // Soft Yellow (Camera Far)

//* Setting Size of TOPDOWN Mini Map
#define TOPDOWN_MINIMAP_X           250     // Width in pixels
#define TOPDOWN_MINIMAP_Y           250     // Height in pixels
#define TOPDOWN_MINIMAP_SCALE_X     80      // Width in meters
#define TOPDOWN_MINIMAP_SCALE_Y     80     // Height in meters

#endif // __PATH_PLANNING_VISUALIZER_SETTINGS_H__