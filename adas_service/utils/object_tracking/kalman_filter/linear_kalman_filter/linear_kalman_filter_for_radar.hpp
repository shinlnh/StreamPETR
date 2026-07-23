#ifndef LINEAR_KALMAN_FILTER_FOR_RADAR
#define LINEAR_KALMAN_FILTER_FOR_RADAR

#include <array>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "linear_kalman_filter.hpp"
#include "types.h"

class LinearKalmanFilterRadar : public LinearKalmanFilter
{
public:
    // The state vector = [x relative distance, x absolute velocity, x absolute acceleration, x absolute jerk,
    //                     y relative distance, y absolute velocity, y absolute acceleration, y absolute jerk]
    static constexpr uint16_t STATE_VECTOR_SIZE_RADAR = 8;
    // The measurement vector = [x relative distance, x absolute velocity,
    //                           y relative distance, y absolute velocity]
    static constexpr uint16_t MEASUREMENT_VECTOR_SIZE_RADAR = 4;
    // The input variable = [x our car's absolute velocity, x our car's absolute acceleration,
    //                       y our car's absolute velocity, y our car's absolute acceleration]
    static constexpr uint16_t INPUT_VARIABLE_SIZE_RADAR = 4;

    // A detailed explanation of the expansion of Kalman parameters will be documented on Confluence in the near future.

private:
    float sample_time_ = 1.0;
    
    // Initialize the standard deviation of jerk, distance in the x and y directions, and velocity in the x and y directions
    float jerk_standard_deviation_ = 1.001;
    float distance_error_standard_deviation_x_ = 0.065105017678918235;  //[unit] = meter
    float velocity_error_standard_deviation_x_ = 0.2031268351725987;    //[unit] = meter/second
    float distance_error_standard_deviation_y_ = 0.3475902872219935;    //[unit] = meter
    float velocity_error_standard_deviation_y_ = 0.26320348615727307;   //[unit] = meter/second

    // Initialize the covariance between distance and velocity in the x and y directions
    float distance_velocity_error_covariance_x_ = 0.0011501839115877177;
    float distance_velocity_error_covariance_y_ = -0.0038435104413570764;

    void recalcStateTransitionMatrix(const float sample_time);
    void recalcProcessNoiseCovarianceMatrix(const float sample_time);
    void recalcControlMatrix(const float sample_time);
    void initializeObservationMatrix();
    void initializeMeasurementNoiseMatrix();
    void initializeLksRadarStates();

public:
    LinearKalmanFilterRadar();
    ~LinearKalmanFilterRadar();
    void recalcKalmanFilterMatrices(const float sample_time) override;
    Eigen::MatrixXf getObservationErrorCovarianceMatrix(Eigen::Matrix<float, 4, 1> &observation, 
                                                        const Eigen::MatrixXf &ego_states, 
                                                        const float &sample_time) override;
};

#endif //LINEAR_KALMAN_FILTER_FOR_RADAR