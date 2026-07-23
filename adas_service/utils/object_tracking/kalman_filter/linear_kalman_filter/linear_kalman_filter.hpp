#ifndef LINEAR_KALMAN_FILTER
#define LINEAR_KALMAN_FILTER

#include <array>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <math.h>
#include "types.h"
#include "common.h"

class LinearKalmanFilter
{
protected:
    uint16_t state_vector_size;                     //The number of states in a state vector
    uint16_t measurement_vector_size;               //The number of measured states
    uint16_t input_variable_size;                   //The number of elements of the input variable

    // State vectors
    Eigen::Matrix<float,Eigen::Dynamic,1> x_n_n;                        // Estimated state vector at step n
    Eigen::Matrix<float,Eigen::Dynamic,1> x_n1_n;                       // Predicted system state vector at step n + 1

    // Measurement and control input vectors
    Eigen::Matrix<float,Eigen::Dynamic,1> z_measurement;                // Measurement vector 
    Eigen::Matrix<float,Eigen::Dynamic,1> u_input;                      // Control input vector

    // Matrices for Kalman filter operations  
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> K_gain;        // Kalman gain matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> P_cov;         // Estimate error covariance matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> eye_mat_ss;    // Identity matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> F_trans;       // State transition matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> G_control;     // Control matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Q_noise;       // Process noise covariance matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_cov;         // Measurement noise covariance matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H_observ;      // Observation matrix
    void init(); // Initialization function
    
public:
    LinearKalmanFilter(){}
    LinearKalmanFilter(uint16_t state_vector_size, uint16_t measurement_vector_size, uint16_t input_variable_size);
    ~LinearKalmanFilter();
    
    // Runtime section
    void predict(const Eigen::MatrixXf &ego_states);
    void predict(); // for cases with no input variable
    void update(const Eigen::MatrixXf &z_measure);
    Eigen::MatrixXf getPredictedState();
    Eigen::MatrixXf getMeasurementNoiseCovariance();

    // Assign predicted states to estimated states.
    // If receiving data from the sensors, update the estimated states
    void assignPredictedStatsToEstimatedStates();

    void getParametersForFusing(Eigen::Matrix<float, 8, 1> &estimated_state_vector,
                                Eigen::Matrix<float, 8, 8> &process_noise_matrix,
                                const Eigen::MatrixXf &ego_states, const float &sample_time);  
    void refreshObjectData(Eigen::Matrix<float, 3, 1> &location, 
                           Eigen::Matrix<float, 3, 1> &velocity,
                           Eigen::Matrix<float, 3, 1> &acceleration);

    virtual void recalcKalmanFilterMatrices(const float sample_time) 
    {
        return;
    };

    virtual Eigen::MatrixXf getObservationErrorCovarianceMatrix(Eigen::Matrix<float, 4, 1> &observation, 
                                                                const Eigen::MatrixXf &ego_states, const float &sample_time)
    {
        return this->H_observ;
    }
};

#endif //LINEAR_KALMAN_FILTER

