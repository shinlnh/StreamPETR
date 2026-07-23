#ifndef LINEAR_KALMAN_FILTER_FOR_CAM
#define LINEAR_KALMAN_FILTER_FOR_CAM

#include <array>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "linear_kalman_filter.hpp"
#include "types.h"

class LinearKalmanFilterCam : public LinearKalmanFilter
{
public:
    // ================ Linear Kalman Filter in the 3D real-world space ===========
    // The state vector = [x relative distance, x absolute velocity, x absolute acceleration, x absolute jerk,
    //                     y relative distance, y absolute velocity, y absolute acceleration, y absolute jerk]
    static constexpr uint16_t STATE_VECTOR_SIZE_3D_CAM = 8;
    // The measurement vector = [x relative distance,
    //                           y relative distance]
    static constexpr uint16_t MEASUREMENT_VECTOR_SIZE_3D_CAM = 2;
    // The input variable = [x our car's absolute velocity, x our car's absolute acceleration,
    //                       y our car's absolute velocity, y our car's absolute acceleration]
    static constexpr uint16_t INPUT_VARIABLE_SIZE_3D_CAM = 4;

    // A detailed explanation of the expansion of Kalman parameters will be documented on Confluence in the near future.


private:
    float sample_time_ = 1.0;
    
    // Initialize the standard deviation of jerk, distance in the x and y directions
    float jerk_standard_deviation_ = 0.302;                                //[unit] = meter/s^3
    float distance_error_standard_deviation_x_ = 0.132856;                 //[unit] = meter
    float distance_error_standard_deviation_y_ = 0.580834;                 //[unit] = meter
    float distance_error_covariance_x_y_ = 0.04031287;

    void recalcStateTransitionMatrix(const float sample_time);
    void recalcProcessNoiseCovarianceMatrix(const float sample_time);
    void recalcControlMatrix(const float sample_time);
    void initializeObservationMatrix();
    void initializeMeasurementNoiseMatrix();
    void initializeLksCameraStates();

public:
    LinearKalmanFilterCam();
    ~LinearKalmanFilterCam();
    void recalcKalmanFilterMatrices(const float sample_time) override;
    Eigen::MatrixXf getObservationErrorCovarianceMatrix(Eigen::Matrix<float, 4, 1> &observation, 
                                                        const Eigen::MatrixXf &ego_states, 
                                                        const float &sample_time) override;
};

class BboxLinearKalmanFilter : public LinearKalmanFilter
{
public:
    // ================ Linear Kalman Filter in the 2D pixel space ================
    // The state vector = [center x, center y, width, height, v_cx, v_cy, v_width, v_height]
    static constexpr uint16_t STATE_VECTOR_SIZE_2D_CAM = 8;

    // The measurement vector = [center x, center y, width, height]
    static constexpr uint16_t MEASUREMENT_VECTOR_SIZE_2D_CAM = 4;

    // Due to the use of image pixels for tracking, it is quite hard and complicated to transfer the control input into the LinearKalmanFilterCam.
    static constexpr uint16_t INPUT_VARIABLE_SIZE_2D_CAM = 0;

    // A detailed explanation of the expansion of Kalman parameters will be documented on Confluence in the near future.
    
private:
    float sample_time_ = 1.0;

    void recalcStateTransitionMatrix(const float sample_time);
    void recalcProcessNoiseCovarianceMatrix(const float sample_time);
    void initializeObservationMatrix();
    void initializeMeasurementNoiseMatrix();
    void initializeLksCameraStates();

public:
    BboxLinearKalmanFilter();
    ~BboxLinearKalmanFilter();
    void recalcKalmanFilterMatrices(const float sample_time) override;
};

#endif //LINEAR_KALMAN_FILTER_FOR_CAM