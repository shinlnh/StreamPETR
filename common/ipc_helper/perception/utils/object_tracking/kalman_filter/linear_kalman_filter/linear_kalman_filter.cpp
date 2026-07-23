#include "linear_kalman_filter.hpp"
#include <cmath>
LinearKalmanFilter::LinearKalmanFilter(uint16_t state_vector_size_, uint16_t measurement_vector_size_, uint16_t input_variable_size_)
    : state_vector_size(state_vector_size_), measurement_vector_size(measurement_vector_size_), input_variable_size(input_variable_size_)
{   
    if (state_vector_size != 0 && measurement_vector_size != 0) {
        // Initialize matrices with appropriate sizes
        (this->x_n_n).resize(state_vector_size);
        (this->x_n1_n).resize(state_vector_size);

        (this->z_measurement).resize(measurement_vector_size);
        (this->u_input).resize(input_variable_size);

        (this->F_trans).resize(state_vector_size, state_vector_size);
        (this->G_control).resize(state_vector_size, input_variable_size);
        (this->Q_noise).resize(state_vector_size, state_vector_size);
        (this->K_gain).resize(state_vector_size, measurement_vector_size);
        (this->P_cov).resize(state_vector_size, state_vector_size);
        (this->R_cov).resize(measurement_vector_size, measurement_vector_size);
        (this->H_observ).resize(measurement_vector_size, state_vector_size);
        (this->eye_mat_ss).resize(state_vector_size, state_vector_size);

        // Initialize the matrices with default values
        init();
    }
    else {
        ERROR("Both state vector size and measurement vector size need to be non-zero");
        exit(1);
    }
}

LinearKalmanFilter::~LinearKalmanFilter() = default;

void LinearKalmanFilter::init()
{
    (this->x_n_n).setZero();
    (this->x_n1_n).setZero();

    (this->z_measurement).setZero();
    (this->u_input).setZero();

    (this->F_trans).setZero();
    (this->G_control).setZero();
    (this->Q_noise).setZero();
    (this->K_gain).setZero();
    (this->P_cov).setIdentity();
    (this->R_cov).setZero();
    (this->H_observ).setIdentity();
    (this->eye_mat_ss).setIdentity();
}

void LinearKalmanFilter::predict(const Eigen::MatrixXf &ego_states) 
{   
    if (input_variable_size == ego_states.rows()) {
        (this->u_input) = ego_states; // Update input

        (this->x_n1_n) = (this->F_trans) * (this->x_n_n) + (this->G_control) * (this->u_input); // Predict state

        (this->P_cov) = (this->F_trans) * (this->P_cov) * (this->F_trans).transpose() + (this->Q_noise); // Update covariance
    }
    else {
        ERROR("Incorrect input vector size");
        exit(1);
    }
}

void LinearKalmanFilter::predict() 
{   
    (this->x_n1_n) = (this->F_trans) * (this->x_n_n); // Predict state with no control input

    (this->P_cov) = (this->F_trans) * (this->P_cov) * (this->F_trans).transpose() + (this->Q_noise); // Update covariance
}

void LinearKalmanFilter::update(const Eigen::MatrixXf &z_measure) {
    if (measurement_vector_size == z_measure.rows()) {
        (this->z_measurement) = z_measure; // Set measurement

        (this->K_gain) = (this->P_cov) * (this->H_observ).transpose() * ((this->H_observ) * 
                         (this->P_cov) * (this->H_observ).transpose() + (this->R_cov)).inverse(); // Update the Kalman gain
        
        (this->x_n_n) = (this->x_n1_n) + (this->K_gain) * ((this->z_measurement) - 
                        (this->H_observ) * (this->x_n1_n)); // Estimate the current state

        Eigen::MatrixXf I_KH = ((this->eye_mat_ss) - (this->K_gain) * (this->H_observ));
        (this->P_cov) = I_KH *  (this->P_cov) * I_KH.transpose() + (this->K_gain) * 
                        (this->R_cov) * (this->K_gain).transpose(); // Update the current estimate uncertainty
    }
    else {
        ERROR("Incorrect measurement vector size");
        exit(1);
    }
}

void LinearKalmanFilter::getParametersForFusing(Eigen::Matrix<float, 8, 1> &estimated_state_vector, 
                                                Eigen::Matrix<float, 8, 8> &process_noise_matrix,
                                                const Eigen::MatrixXf &ego_states, const float &sample_time)
{   
    recalcKalmanFilterMatrices(sample_time);
    estimated_state_vector =  (this->F_trans) * (this->x_n_n) + (this->G_control) * ego_states; 
    process_noise_matrix = (this->F_trans) * (this->P_cov) * (this->F_trans).transpose() + (this->Q_noise);    
}

void LinearKalmanFilter::refreshObjectData(Eigen::Matrix<float, 3, 1> &location, 
                                           Eigen::Matrix<float, 3, 1> &velocity,
                                           Eigen::Matrix<float, 3, 1> &acceleration)
{
    float location_z = location.coeffRef(2);
    float velocity_z = velocity.coeffRef(2);
    location << this->x_n_n[0], this->x_n_n[4], location_z;
    velocity << this->x_n_n[1], this->x_n_n[5], velocity_z;
    acceleration << this->x_n_n[2], this->x_n_n[6], 0.0;
}

Eigen::MatrixXf LinearKalmanFilter::getPredictedState()
{
    return this->x_n1_n;
};

void LinearKalmanFilter::assignPredictedStatsToEstimatedStates()
{
    this->x_n_n = this->x_n1_n;
}

Eigen::MatrixXf LinearKalmanFilter::getMeasurementNoiseCovariance()
{
    return this->R_cov;
}