#include "linear_kalman_filter_for_cam.hpp"
#include <cmath>
#include "common.h"
LinearKalmanFilterCam::LinearKalmanFilterCam()
  : LinearKalmanFilter(STATE_VECTOR_SIZE_3D_CAM, MEASUREMENT_VECTOR_SIZE_3D_CAM, INPUT_VARIABLE_SIZE_3D_CAM)
{
    this->initializeObservationMatrix();
    this->initializeMeasurementNoiseMatrix();
    // Since initial state vector is a guess, we set a very high estimate uncertainty
    (this->P_cov) = (this->P_cov) * 500;
    // Predict after initialization 
    this->initializeLksCameraStates();
}

LinearKalmanFilterCam::~LinearKalmanFilterCam()
{

}

void LinearKalmanFilterCam::recalcStateTransitionMatrix(const float dt)
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

void LinearKalmanFilterCam::recalcProcessNoiseCovarianceMatrix(const float dt)
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
void LinearKalmanFilterCam::recalcControlMatrix(const float dt)
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

void LinearKalmanFilterCam::initializeObservationMatrix()
{
    (this->H_observ) << 
            1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0;
}

void LinearKalmanFilterCam::initializeMeasurementNoiseMatrix()
{
    (this->R_cov) << 
            pow(distance_error_standard_deviation_x_, 2), distance_error_covariance_x_y_,
                          distance_error_covariance_x_y_, pow(distance_error_standard_deviation_y_, 2);
}
void LinearKalmanFilterCam::recalcKalmanFilterMatrices(const float sample_time)
{
    this->sample_time_ = sample_time;
    recalcStateTransitionMatrix(this->sample_time_);
    recalcProcessNoiseCovarianceMatrix(this->sample_time_);
    recalcControlMatrix(this->sample_time_);
}

void LinearKalmanFilterCam::initializeLksCameraStates()
{   
    // During initialization, we do not know the measurement period, set the default value equal to 1 second
    recalcKalmanFilterMatrices(1.0);  
    this->predict();
}

Eigen::MatrixXf LinearKalmanFilterCam::getObservationErrorCovarianceMatrix(Eigen::Matrix<float, 4, 1> &observation, 
                                                                           const Eigen::MatrixXf &ego_states, const float &sample_time)
{   
    // since the camera only observes (x,y) position, but the radar observes (x,y,vx​,vy​),
    // we need to modify the Mahalanobis distance calculation to consider position (x,y) and velocity (vx​,vy​)
    Eigen::Matrix<float, 4, 8> H_observ_;
    H_observ_ <<
            1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0;

    Eigen::Matrix<float, 4, 4> R_cov_;        
    R_cov_ << 
            pow(distance_error_standard_deviation_x_, 2),            0,                                 distance_error_covariance_x_y_,                 0,
            0,                                                       pow(0, 2),                         0,                                              0,
            distance_error_covariance_x_y_,                          0,                                 pow(distance_error_standard_deviation_y_, 2),   0,
            0,                                                       0,                                 0,                                              pow(0, 2);

    recalcKalmanFilterMatrices(sample_time);
    Eigen::MatrixXf camera_observation =  (this->F_trans) * (this->x_n_n) + (this->G_control) * ego_states; 
    Eigen::MatrixXf sub_P_cov = (this->F_trans) * (this->P_cov) * (this->F_trans).transpose() + (this->Q_noise);
    observation << camera_observation.coeffRef(0),
                   camera_observation.coeffRef(1),
                   camera_observation.coeffRef(4),
                   camera_observation.coeffRef(5);

    Eigen::Matrix<float, 4, 4> s_matrix  = H_observ_ * sub_P_cov * H_observ_.transpose() + R_cov_;   
    return s_matrix;                           
}

BboxLinearKalmanFilter::BboxLinearKalmanFilter()
    : LinearKalmanFilter(STATE_VECTOR_SIZE_2D_CAM, MEASUREMENT_VECTOR_SIZE_2D_CAM, INPUT_VARIABLE_SIZE_2D_CAM)
{
    this->initializeObservationMatrix();
    this->initializeMeasurementNoiseMatrix();
    // Since initial state vector is a guess, we set a very high estimate uncertainty
    (this->P_cov) = (this->P_cov) * 500;
    // Predict after initialization 
    this->initializeLksCameraStates();
}

BboxLinearKalmanFilter::~BboxLinearKalmanFilter()
{

}

void BboxLinearKalmanFilter::recalcStateTransitionMatrix(const float dt)
{   
    (this->F_trans) << 
            1, 0, 0, 0, dt,  0,  0,  0,
            0, 1, 0, 0,  0, dt,  0,  0,
            0, 0, 1, 0,  0,  0, dt,  0,
            0, 0, 0, 1,  0,  0,  0, dt,
            0, 0, 0, 0,  1,  0,  0,  0,
            0, 0, 0, 0,  0,  1,  0,  0,
            0, 0, 0, 0,  0,  0,  1,  0,
            0, 0, 0, 0,  0,  0,  0,  1;
}

void BboxLinearKalmanFilter::recalcProcessNoiseCovarianceMatrix(const float dt)
{   
    //To be honest, the process noise covariance matrix needs to have the sample time as an input parameter, and this matrix is dynamic, not constant.
    // Since the pixel camera tracking works well with this matrix, I still include it in the system."  
    // This is a hard code for F (only adapt with the kinematic model of constant acceleration without the bbox params)
    (this->Q_noise) << 
            1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0, 0, 0, 0.0001;
}

void BboxLinearKalmanFilter::initializeObservationMatrix()
{
    (this->H_observ) << 
            1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0;
}

void BboxLinearKalmanFilter::initializeMeasurementNoiseMatrix()
{
    (this->R_cov) << 
            1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 10, 0,
            0, 0, 0,  10;
}
void BboxLinearKalmanFilter::recalcKalmanFilterMatrices(const float sample_time)
{
    this->sample_time_ = sample_time;
    recalcStateTransitionMatrix(this->sample_time_);
    recalcProcessNoiseCovarianceMatrix(this->sample_time_);
}

void BboxLinearKalmanFilter::initializeLksCameraStates()
{   
    // During initialization, we do not know the measurement period, set the default value equal to 1 second
    recalcKalmanFilterMatrices(1.0);  
    this->predict();
}

