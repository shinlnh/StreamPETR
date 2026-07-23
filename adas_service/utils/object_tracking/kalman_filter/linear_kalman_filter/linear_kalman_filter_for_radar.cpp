#include "linear_kalman_filter_for_radar.hpp"
#include <cmath>
#include "common.h"
LinearKalmanFilterRadar::LinearKalmanFilterRadar()
: LinearKalmanFilter(STATE_VECTOR_SIZE_RADAR, MEASUREMENT_VECTOR_SIZE_RADAR, INPUT_VARIABLE_SIZE_RADAR)
{
    this->initializeObservationMatrix();
    this->initializeMeasurementNoiseMatrix();
    // Since initial state vector is a guess, we set a very high estimate uncertainty
    (this->P_cov) = (this->P_cov) * 500;
    // Predict after initialization 
    this->initializeLksRadarStates();
}

LinearKalmanFilterRadar::~LinearKalmanFilterRadar()
{

}

void LinearKalmanFilterRadar::recalcStateTransitionMatrix(const float dt)
{
    float dt2 = 0.5f * dt * dt;
    float dt3 = (1.0f / 6.0f) * dt * dt * dt;

    (this->F_trans) << 
            1.0f, dt,   dt2,   dt3, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, dt,    dt2, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f,   dt, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f,   dt,  dt2,  dt3,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,   dt,  dt2,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,   dt,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
}

void LinearKalmanFilterRadar::recalcProcessNoiseCovarianceMatrix(const float dt)
{   
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    float dt5 = dt4 * dt;
    float dt6 = dt4 * dt2;

    float jerk_var = std::pow(this->jerk_standard_deviation_, 2);

    (this->Q_noise) << 
        (1.0f / 36.0f) * dt6, (1.0f / 12.0f) * dt5, (1.0f / 6.0f) * dt4, (1.0f / 6.0f) * dt3,                 0.0f,                 0.0f,                0.0f,                0.0f,
        (1.0f / 12.0f) * dt5,  (1.0f / 4.0f) * dt4, (1.0f / 2.0f) * dt3, (1.0f / 2.0f) * dt2,                 0.0f,                 0.0f,                0.0f,                0.0f,
         (1.0f / 6.0f) * dt4,  (1.0f / 2.0f) * dt3,                 dt2,                  dt,                 0.0f,                 0.0f,                0.0f,                0.0f,
         (1.0f / 6.0f) * dt3,  (1.0f / 2.0f) * dt2,                  dt,                1.0f,                 0.0f,                 0.0f,                0.0f,                0.0f,
                        0.0f,                 0.0f,                0.0f,                0.0f, (1.0f / 36.0f) * dt6, (1.0f / 12.0f) * dt5, (1.0f / 6.0f) * dt4, (1.0f / 6.0f) * dt3,
                        0.0f,                 0.0f,                0.0f,                0.0f, (1.0f / 12.0f) * dt5,  (1.0f / 4.0f) * dt4, (1.0f / 2.0f) * dt3, (1.0f / 2.0f) * dt2,
                        0.0f,                 0.0f,                0.0f,                0.0f,  (1.0f / 6.0f) * dt4,  (1.0f / 2.0f) * dt3,                 dt2,                  dt,
                        0.0f,                 0.0f,                0.0f,                0.0f,  (1.0f / 6.0f) * dt3,  (1.0f / 2.0f) * dt2,                  dt,                1.0f;

    (this->Q_noise) *= jerk_var;
}

void LinearKalmanFilterRadar::recalcControlMatrix(const float dt)
{
    float dt2 = -0.5f * dt * dt;

    (this->G_control) << 
            -dt,    dt2, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f,  -dt,  dt2,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f; 
}

void LinearKalmanFilterRadar::initializeObservationMatrix()
{
    (this->H_observ) << 
            1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0;
}

void LinearKalmanFilterRadar::initializeMeasurementNoiseMatrix()
{
    (this->R_cov) << 
            pow(distance_error_standard_deviation_x_, 2),        distance_velocity_error_covariance_x_,                                            0,                                            0,
                   distance_velocity_error_covariance_x_, pow(velocity_error_standard_deviation_x_, 2),                                            0,                                            0,
                                                       0,                                            0, pow(distance_error_standard_deviation_y_, 2),        distance_velocity_error_covariance_y_,
                                                       0,                                            0,        distance_velocity_error_covariance_y_, pow(velocity_error_standard_deviation_y_, 2);
}

void LinearKalmanFilterRadar::recalcKalmanFilterMatrices(const float sample_time)
{
    this->sample_time_ = sample_time;
    recalcStateTransitionMatrix(this->sample_time_);
    recalcProcessNoiseCovarianceMatrix(this->sample_time_);
    recalcControlMatrix(this->sample_time_);
}

void LinearKalmanFilterRadar::initializeLksRadarStates()
{
    // During initialization, we do not know the measurement period, set the default value equal to 1 second
    recalcKalmanFilterMatrices(1.0); 
    this->predict();
}

Eigen::MatrixXf LinearKalmanFilterRadar::getObservationErrorCovarianceMatrix(Eigen::Matrix<float, 4, 1> &observation, 
                                                                             const Eigen::MatrixXf &ego_states, const float &sample_time)
{   
    recalcKalmanFilterMatrices(sample_time);
    Eigen::MatrixXf radar_observation =  (this->F_trans) * (this->x_n_n) + (this->G_control) * ego_states; 
    Eigen::MatrixXf sub_P_cov = (this->F_trans) * (this->P_cov) * (this->F_trans).transpose() + (this->Q_noise);
    observation << radar_observation.coeffRef(0),
                   radar_observation.coeffRef(1),
                   radar_observation.coeffRef(4),
                   radar_observation.coeffRef(5);
    Eigen::Matrix<float,4, 4> s_matrix = (this->H_observ) * sub_P_cov * (this->H_observ).transpose() + (this->R_cov);
    return s_matrix;      
}
