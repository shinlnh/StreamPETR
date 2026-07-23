#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>
#include "frenet_environment/settings.h"

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: enhancedLinearKinematicModel(_, _, _, _)
* Tittle:
* Input: float steering_angle, float yaw_angle, float my_car_velocity, float sample_time_mpc
* Expected output: Ae, Be, Ce and Xe 
*
*/
TEST(ModelPredictiveControlTest, EnhancedLinearKinematicModelTest) {
    ModelPredictiveControl mpc;

    // Define inputs for the test
    float steering_angle = 0.1f;                // steering angle in radians
    float yaw_angle = 0.2f;                     // yaw angle in radians
    float my_car_velocity = 10.0f;              // velocity in m/s
    float sample_time_mpc = 0.1f;               // sample time in seconds

    // Call the method under test
    mpc.enhancedLinearKinematicModel(steering_angle, yaw_angle, my_car_velocity, sample_time_mpc);

    // Define expected values for Ae, Be, Ce, and Xe matrices based on calculations
    Eigen::Matrix<float, 4, 4> expected_Ae;
    expected_Ae << 1,   0,   my_car_velocity * std::cos(yaw_angle) * sample_time_mpc,      0,
                   0,   1, - my_car_velocity * std::sin(yaw_angle) * sample_time_mpc,      0,
                   0,   0,   1,   my_car_velocity / (CAR_WHEEL_BASE * std::cos(steering_angle) * std::cos(steering_angle)) * sample_time_mpc,
                   0,   0,   0,   1;

    Eigen::Matrix<float, 4, 1> expected_Be;
    expected_Be << 0,
                   0,
                   my_car_velocity / (CAR_WHEEL_BASE * std::cos(steering_angle) * std::cos(steering_angle)) * sample_time_mpc,
                   1;

    Eigen::Matrix<float, 3, 4> expected_Ce;
    expected_Ce << 1,   0,   0,   0,
                   0,   1,   0,   0,
                   0,   0,   1,   0;

    Eigen::Matrix<float, 4, 1> expected_Xe;
    expected_Xe << 0, 
                   0,
                   0,
                   steering_angle;

    // Assertions to compare the actual results to the expected matrices
    EXPECT_TRUE(mpc.Ae.isApprox(expected_Ae, 0)) << "Ae matrix does not mismatch";
    EXPECT_TRUE(mpc.Be.isApprox(expected_Be, 0)) << "Be matrix does not mismatch";
    EXPECT_TRUE(mpc.Ce.isApprox(expected_Ce, 0)) << "Ce matrix does not mismatch";
    EXPECT_TRUE(mpc.Xe.isApprox(expected_Xe, 0)) << "Xe matrix does not mismatch";
}
