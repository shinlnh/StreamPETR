#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>
#include <Eigen/Dense>

#define PREDICTED_STEPS 37

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: predictedTrajectoryMatrix(const float& x_coordinate, const float& y_coordinate,
                                   const float& yaw_angle, const float& steering_angle,
                                   const float& my_car_velocity, const float& sample_time_mpc, 
                                   const PlanningResults& planning_results, 
                                   Eigen::Matrix<float, 3 * Np, 1>& Rs)*
* Titile: For case vale of predicted trajectory columus 1 is smaller than 13.0     
* Input: x_coordinate, y_coordinate, yaw_angle, steering_angle, my_car_velocity, sample_time_mpc, planning_results, Rs
* Expected output:
*/
TEST(ModelPredictiveControlTest, predictedTrajectoryMatrixTest01) {
    ModelPredictiveControl mpc;

    // Define inputs for the test
    float x_coordinate = 1.0f;                  
    float y_coordinate = 1.0f;
    float yaw_angle = 0.1f;                                 // yan angle in radians
    float steering_angle = 0.1f;                            // steering angle in radians
    float my_car_velocity = 10.0f;                          // my car veloctity in m/s
    float car_acceleration = 0.0f;                          // acceleration in m/s
    float sample_time_mpc = 0.1f;                           // sample time in seconds
    PlanningResults planning_results;     // planning results
    planning_results.points.push_back({1.0f, 2.0f, 3.0f});

    Eigen::Matrix<float, 3 * PREDICTED_STEPS, 1> Rs;
    Rs.setZero();           // Declare initialize Eigen Matrix
    
    // Call fucntion to test 
    mpc.predictedTrajectoryMatrix(x_coordinate, y_coordinate, 
                                  yaw_angle, steering_angle,
                                  my_car_velocity, car_acceleration,
                                  sample_time_mpc,
                                  planning_results, Rs
                                  );
}

/*
* @brief TEST 2: predictedTrajectoryMatrix(const float& x_coordinate, const float& y_coordinate,
                                   const float& yaw_angle, const float& steering_angle,
                                   const float& my_car_velocity, const float& sample_time_mpc, 
                                   const PlanningResults& planning_results, 
                                   Eigen::Matrix<float, 3 * Np, 1>& Rs)*
* Titile: For case vale of predicted trajectory columus 1 is greater than 13.0                                    
* Input: x_coordinate, y_coordinate, yaw_angle, steering_angle, my_car_velocity, sample_time_mpc, planning_results, Rs
* Expected output:
*/
TEST(ModelPredictiveControlTest, predictedTrajectoryMatrixTest02) {
    ModelPredictiveControl mpc;

    // Define inputs for the test
    float x_coordinate = 1.0f;                  
    float y_coordinate = 23.0f;
    float yaw_angle = 0.1f;                                 // yan angle in radians
    float steering_angle = 0.1f;                            // steering angle in radians
    float my_car_velocity = 10.0f;                          // my car veloctity in m/s
    float car_acceleration = 0.0f;                          // acceleration in m/s
    float sample_time_mpc = 0.1f;                           // sample time in seconds
    PlanningResults planning_results;     // planning results
    planning_results.points.push_back({1.0f, 2.0f, 3.0f});

    Eigen::Matrix<float, 3 * PREDICTED_STEPS, 1> Rs;
    Rs.setZero();           // Declare initialize Eigen Matrix
    
    // Call fucntion to test 
    mpc.predictedTrajectoryMatrix(x_coordinate, y_coordinate, 
                                  yaw_angle, steering_angle,
                                  my_car_velocity, car_acceleration,
                                  sample_time_mpc,
                                  planning_results, Rs
                                  );
}