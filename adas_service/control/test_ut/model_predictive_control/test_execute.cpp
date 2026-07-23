#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>
#include <gmock/gmock.h>
#include <stdexcept>
#include <limits>

#define CONTROL_STEPS 20

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: execute(const PlanningResults& planning_results, const float& my_car_velocity,
*                                     const float& sample_time_mpc, const float& car_steering_angle)
*
* Input: PlanningResults& planning_results, float my_car_velocity, float sample_time_mpc, float car_steering_angle
* Expected output:
*
*/
TEST(ModelPredictiveControlTest, ExecuteTest_01) {
    ModelPredictiveControl mpc;
    
    // Define inputs for the test
    float steering_angle = 0.1f;                // steering angle in radians
    float car_steering_angle = 0.2f;            // car steering angle in radians
    float my_car_velocity = 10.0f;              // velocity in m/s
    float car_acceleration = 0.0f;              // acceleration in m/s
    float sample_time_mpc = 0.1f;               // sample time in seconds

    PlanningResults planning_results;

    planning_results.points.push_back({1.0f, 2.0f, 3.0f});

    // Call the method under test
    mpc.execute(planning_results, my_car_velocity, car_acceleration, sample_time_mpc, car_steering_angle);
}


/*
* @brief TEST 2: execute(const PlanningResults& planning_results, const float& my_car_velocity,
*                                     const float& sample_time_mpc, const float& car_steering_angle)
*
* Input: PlanningResults& planning_results, float my_car_velocity, float sample_time_mpc, float car_steering_angle
* Expected output:
*
*/
//// Test Mock function checkForConstraintViolations in the excute function 
TEST(ModelPredictiveControlTest, ExecuteTest_02) {
    //ModelPredictiveControl mpc;
    MockModelPredictiveControl mock_mpc;

    //mockcheckModelPredictiveControl mock_mpc;
    // Define inputs for the test
    float steering_angle = 0;                // steering angle in radians
    float car_steering_angle = 0.2f;            // car steering angle in radians
    float my_car_velocity = 10.0f;              // velocity in m/s
    float car_acceleration = 0.0f;              // acceleration in m/s
    float sample_time_mpc = 0.1f;               // sample time in seconds

    Eigen::Matrix<float, 2 * mock_mpc.control_steps, 1> gamma_matrix;
    Eigen::Matrix<float, 2 * mock_mpc.control_steps, mock_mpc.control_steps> constraint_matrix;
    Eigen::Matrix<float, mock_mpc.control_steps, 1> matrix_to_be_checked;
    PlanningResults planning_results;

    planning_results.points.push_back({1.0f, 2.0f, 3.0f});

    // Call the method under test
    EXPECT_CALL(mock_mpc, checkForConstraintViolations(_, _, _))
    .Times(1)
    .WillOnce(Return(true));

    mock_mpc.execute(planning_results, my_car_velocity, car_acceleration, sample_time_mpc, car_steering_angle);
}


