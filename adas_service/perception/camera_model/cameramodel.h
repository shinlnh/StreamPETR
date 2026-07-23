#ifndef CAMERAMODEL_H
#define CAMERAMODEL_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <fstream>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "common.h"
#ifndef UT_TEST
#else
#endif

// #define TOPDOWN_CALIB_PATH ((getenv("PC_BUILD") != nullptr) ? "/nfs/share/adas_data/camera_calib/camera_calib_carla.txt" : "/lib/banvien/camera_calib.txt")
#define TOPDOWN_CALIB_PATH      "common/config/sensor/camera_calib_carla.txt"
#define UNDISTORT_CALIB_PATH    "common/config/sensor/camera_calibration_parameters.txt"
#define CAMERA_CONFIG_PATH      "common/config/sensor/camera_config.txt"

class CameraModel
{
private:
    std::string topdown_calib_path = TOPDOWN_CALIB_PATH;
    std::string undistort_calib_path = UNDISTORT_CALIB_PATH;
    std::string camera_config_path = CAMERA_CONFIG_PATH;
    
    bool is_calibrated = false;
    float car_width = 0;

    // Transformation matrix from camera perspective to top-down view
    std::vector<cv::Point2f> four_image_points;
    std::vector<cv::Point2f> carpet_real_points;

    // Camera properties
    Eigen::Vector3f position = Eigen::Vector3f::Zero(3);  // x, y, z
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero(3);  // roll, pitch, yaw
    Eigen::Matrix3f camera_intrinsic_matrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix4f camera_extrinsic_matrix = Eigen::Matrix4f::Identity();
    
public:
    // Constructor
    CameraModel(std::string topdown_calib_path = "",
                std::string undistort_calib_path = "",
                std::string camera_config_path = "");

    // Update methods
    void calibTopdownView();
    void calcCameraIntrinsicMatrix();
    void calcCameraExtrinsicMatrix();
    
    // Set methods
    void setTopdownViewCalib(std::string topdown_calib_path = "");
    void setUndistortCalib(std::string undistort_calib_path = "");
    void setCameraConfig(std::string camera_config_path = "");

    // Get methods
    bool isCalibrated() {return is_calibrated;}
    float getCarWidth() {return this->car_width;}
    Eigen::Vector3f getPosition() {return position;}
    Eigen::Vector3f getOrientation() {return orientation;}
    Eigen::Matrix3f getCamIntrinsicMatrix() {return camera_intrinsic_matrix;}
    Eigen::Matrix4f getCamExtrinsicMatrix() {return camera_extrinsic_matrix;}

    // Calculate distance
    std::vector<cv::Point2f> getDistanceVectorToCar(std::vector<cv::Point2f> const &points, const float pitch);
    float getLateralDistanceToCar(const float x, const float y, const float pitch);
    float getLongitudinalDistanceToCar(const float x, const float y, const float pitch);
};

#endif // CAMERAMODEL_H
