#include "cameramodel.h"

using namespace std;
using namespace cv;

CameraModel::CameraModel(std::string topdown_calib_path,
                         std::string undistort_calib_path,
                         std::string camera_config_path)
{
    // Set file path
    if (!topdown_calib_path.empty())
        this->topdown_calib_path = topdown_calib_path;
    if (!undistort_calib_path.empty())
        this->undistort_calib_path = undistort_calib_path;
    if (!camera_config_path.empty())
        this->camera_config_path = camera_config_path;

    // Update calib
    calcCameraIntrinsicMatrix();
    calcCameraExtrinsicMatrix();
    calibTopdownView();  // Always calib topdown after reading camera config

    is_calibrated = true;
    INFO("Camera calibrated!");
}

void CameraModel::calibTopdownView()
{
    float car_width;
    float carpet_dist, carpet_width, carpet_length;
    uint16_t top_left_x, top_left_y;
    uint16_t top_right_x, top_right_y;
    uint16_t bottom_right_x, bottom_right_y;
    uint16_t bottom_left_x, bottom_left_y;
    std::string line;

    // Read data file
    std::ifstream data_file(this->topdown_calib_path);
    data_file >> line >> car_width;
    data_file >> line >> carpet_width;
    data_file >> line >> carpet_dist;
    data_file >> line >> carpet_length;
    data_file >> line >> top_left_x >> line >> top_left_y;
    data_file >> line >> top_right_x >> line >> top_right_y;
    data_file >> line >> bottom_right_x >> line >> bottom_right_y;
    data_file >> line >> bottom_left_x >> line >> bottom_left_y;

    // Carpet corners in the image
    four_image_points = {
        cv::Point2f(top_left_x, top_left_y),               // top-left
        cv::Point2f(top_right_x, top_right_y),             // top-right
        cv::Point2f(bottom_right_x, bottom_right_y),       // bottom-right
        cv::Point2f(bottom_left_x, bottom_left_y)          // bottom-left
    };

    // Show data after read data file
    DEBUG("%s", this->topdown_calib_path.c_str());
    DEBUG("car_width: %f", car_width);
    DEBUG("carpet_width: %f", carpet_width);
    DEBUG("car_to_carpet_distance: %f", carpet_dist);
    DEBUG("carpet_length: %f", carpet_length);
    DEBUG("top_left_x:  %hu", top_left_x);
    DEBUG("top_left_y:  %hu", top_left_y);
    DEBUG("top_right_x:  %hu", top_right_x);
    DEBUG("top_right_y:  %hu", top_right_y);
    DEBUG("bottom_right_x:  %hu", bottom_right_x);
    DEBUG("bottom_right_y:  %hu", bottom_right_y);
    DEBUG("bottom_left_x:  %hu", bottom_left_x);
    DEBUG("bottom_left_y:  %hu", bottom_left_y);

    // Calculate realworld cord of the carpet
    float xOffset = position[0], yOffset = position[1];
    float xLeft  = xOffset - (carpet_width / 2.0f),
          xRight = xOffset + (carpet_width / 2.0f),
          yTop   = yOffset + carpet_dist + carpet_length,
          yBot   = yOffset + carpet_dist;
    carpet_real_points = {  // four real world coords
        cv::Point2f(xLeft, yTop),   // top-left
        cv::Point2f(xRight, yTop),  // top-right
        cv::Point2f(xRight, yBot),  // bottom-right
        cv::Point2f(xLeft, yBot)    // bottom-left
    };

    // Update properties
    this->car_width = car_width;
}


void CameraModel::calcCameraIntrinsicMatrix()
{
    // Open calib file
    std::ifstream file(this->undistort_calib_path);
    
    if (!file.is_open()) {
        ERROR("Failed to open calibration file: %s", this->undistort_calib_path.c_str());
        throw std::runtime_error("Failed to open camera calibration file");
    }

    std::string line;
    // Find the start of the camera matrix block
    while (std::getline(file, line)) {
        if (line.find("Camera Matrix:") != std::string::npos) break;
    }

    std::vector<float> matrix_values;
    for (int i = 0; i < 3; ++i) {
        if (!std::getline(file, line)) {
            ERROR("Incomplete camera matrix in calibration file");
            throw std::runtime_error("Incomplete camera matrix");
        }

        // Parse float values from each line
        std::istringstream iss(line);
        float value;
        while (iss >> value) matrix_values.push_back(value);
    }

    if (matrix_values.size() < 9) {
        ERROR("Invalid camera matrix format: expected 9 values, got %zu", matrix_values.size());
        throw std::runtime_error("Invalid camera matrix format");
    }
    file.close();

    Eigen::Matrix3f intrinsic;
    // Assign the parsed values to the Eigen 3x3 camera intrinsic matrix
    // Because the pattern (dotted grid) in CARLA wasn't good enough,
    // the inferred intrinsic matrix parameters are not very close to the true value — so I adjusted them slightly by error values below
    matrix_values[0] = 400.0;
    matrix_values[2] = 400.0;
    matrix_values[4] = 400.0;
    matrix_values[5] = 400.0;
    intrinsic <<
        matrix_values[0], matrix_values[1], matrix_values[2],
        matrix_values[3], matrix_values[4], matrix_values[5],
        matrix_values[6], matrix_values[7], matrix_values[8];

    camera_intrinsic_matrix = intrinsic;
}


void CameraModel::calcCameraExtrinsicMatrix()
{
    // Open Calib file
    std::ifstream file(this->camera_config_path);
    if (!file.is_open()) {
        ERROR("Failed to open camera config file: %s", this->camera_config_path.c_str());
        throw std::runtime_error("Failed to open camera config file");
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string key, value;
        std::getline(iss, key, ':');
        std::getline(iss, value);

        if (key != "coordinate") continue;

        std::istringstream coord_stream(value);
        std::string token;
        std::vector<double> coords;

        while (std::getline(coord_stream, token, ',')) {
            coords.push_back(std::stod(token));
        }

        if (coords.size() >= 6) {
            position = Eigen::Vector3f(-coords[1], coords[0], coords[2]);  // (Y, -X, Z)
            orientation = Eigen::Vector3f(
                -coords[4] * M_PI / 180.0f,  // Roll
                 coords[3] * M_PI / 180.0f,  // Pitch
                 coords[5] * M_PI / 180.0f   // Yaw
            );
        }
        break;
    }
    file.close();

    // Compute rotation matrix from roll, pitch, yaw
    Eigen::Matrix3f R = (
        Eigen::AngleAxisf(orientation[0], Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(orientation[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(orientation[2], Eigen::Vector3f::UnitZ())
    ).toRotationMatrix();

    // Construct 4x4 extrinsic matrix: [R^T | -R^T * T]
    Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity();
    extrinsic.block<3,3>(0,0) = R.transpose();
    extrinsic.block<3,1>(0,3) = -R.transpose() * position;

    camera_extrinsic_matrix = extrinsic;
}


void CameraModel::setTopdownViewCalib(std::string topdown_calib_path)
{
    if (topdown_calib_path.empty()) return;
    this->topdown_calib_path = topdown_calib_path; // Set file path
    calibTopdownView();
}


void CameraModel::setUndistortCalib(std::string undistort_calib_path)
{
    if (undistort_calib_path.empty()) return;
    this->undistort_calib_path = undistort_calib_path; // Set file path
    calcCameraIntrinsicMatrix();
}


void CameraModel::setCameraConfig(std::string camera_config_path)
{
    if (camera_config_path.empty()) return;
    this->camera_config_path = camera_config_path; // Set file path
    calcCameraExtrinsicMatrix();
    calibTopdownView();  // Always calib topdown after reading camera config
}


std::vector<cv::Point2f> CameraModel::getDistanceVectorToCar(std::vector<cv::Point2f> const &cam_points, const float pitch)
{
    // Return empty if not calibrated
    if (!is_calibrated || cam_points.empty())
        return std::vector<cv::Point2f> ();

    // Static variables
    static constexpr float focal_length = 400;
    cv::Mat transform_matrix;

    // Shift source pixel to compensate for pitch change
    float src_pitch = this->orientation[1];
    float pixelShift = focal_length * std::tan(pitch - src_pitch);
    std::vector<cv::Point2f> adjustedSrc = this->four_image_points;
    for(auto &point: adjustedSrc) {
        point.y -= pixelShift; // Shift points up/down to compensate
    }

    // Calculate transform matrix
    transform_matrix = cv::getPerspectiveTransform(adjustedSrc, this->carpet_real_points);

    // Transform all points to top down view
    std::vector<cv::Point2f> topdown_points;
    cv::perspectiveTransform(cam_points, topdown_points, transform_matrix);

    return topdown_points;
}


float CameraModel::getLateralDistanceToCar(const float x, const float y, const float pitch)
{
    if (!is_calibrated)
        return -1;

    // Convert one points from the camera view to the top down view
    std::vector<cv::Point2f> camera_point = {cv::Point2f(x, y)};
    std::vector<cv::Point2f> topdown_points = this->getDistanceVectorToCar(camera_point, pitch);

    return topdown_points.at(0).x;
}


float CameraModel::getLongitudinalDistanceToCar(const float x, const float y, const float pitch)
{
    if (!is_calibrated)
        return -1;

    // Convert one points from the camera view to the top down view
    std::vector<cv::Point2f> camera_point = {cv::Point2f(x, y)};
    std::vector<cv::Point2f> topdown_points = this->getDistanceVectorToCar(camera_point, pitch);

    return topdown_points.at(0).y;
}