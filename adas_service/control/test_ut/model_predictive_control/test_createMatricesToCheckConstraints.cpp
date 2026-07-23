#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>
#include <Eigen/Dense>

#define CONTROL_STEPS 20

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: createMatricesToCheckConstraints(Eigen::Matrix<float, 2 * Nc, 1>& gamma_matrix, 
                                          Eigen::Matrix<float, 2 * Nc, Nc>& M,
                                          const float& steering_angle)) 
* Tittle:
* Input: Eigen::Matrix<float, 2 * Nc, 1> gamma_matrix, Eigen::Matrix<float, 2 * Nc, Nc> M, float& steering_angle)
* Expected output:  M, gamma_matrix
*/
TEST(ModelPredictiveControlTest, createMatricesToCheckConstraintsTest_01) {
    ModelPredictiveControl mpc;

    // Define inputs for the test
    Eigen::Matrix<float, 4 * CONTROL_STEPS, 1> gamma_matrix;
    Eigen::Matrix<float, 4 * CONTROL_STEPS, CONTROL_STEPS> M;
    float steering_angle = 5.0f / 180.0f;            // steering angle in radians
    const float sample_time = 0.05f;
    M.setZero();

    // Call the method under test
    mpc.createMatricesToCheckConstraints(gamma_matrix, M, steering_angle, sample_time);
    
    // Expect result
    Eigen::Matrix<float, 4 * CONTROL_STEPS, 1> expect_gamma_matrix;
    Eigen::Matrix<float, 4 * CONTROL_STEPS, CONTROL_STEPS> expect_M;
    expect_M.setZero();

    for(int i = 0; i < CONTROL_STEPS; i++) {
        expect_gamma_matrix.coeffRef(i,0) = mpc.max_steering_angle - steering_angle;
        expect_gamma_matrix.coeffRef(i + CONTROL_STEPS, 0) = - mpc.min_steering_angle + steering_angle;
        for(int j = 0; j <= i; j++) {
            expect_M.coeffRef(i,j) = 1;
            expect_M.coeffRef(i + CONTROL_STEPS, j) = -1;
        }
        expect_gamma_matrix.coeffRef(i + 2 * CONTROL_STEPS, 0) = mpc.max_steering_rate * sample_time;
        expect_M.coeffRef(i + 2 * CONTROL_STEPS, i) = 1.0f;
        expect_gamma_matrix.coeffRef(i + 3 * CONTROL_STEPS, 0) = mpc.max_steering_rate * sample_time;
        expect_M.coeffRef(i + 3 * CONTROL_STEPS, i) = -1.0f;
    }

    float scale = 10.0f;
    for (int i = 0; i < CONTROL_STEPS; i++)
    {
        expect_M.row(i + 2 * CONTROL_STEPS) *= scale;
        expect_M.row(i + 3 * CONTROL_STEPS) *= scale;

        expect_gamma_matrix.coeffRef(i + 2 * CONTROL_STEPS, 0) *= scale;
        expect_gamma_matrix.coeffRef(i + 3 * CONTROL_STEPS, 0) *= scale;
    }
    EXPECT_TRUE(gamma_matrix.isApprox(expect_gamma_matrix, 1e-3)) << "Ae matrix does not mismatch";
    //ASSERT_TRUE(M.isApprox(expect_M, 1e-3)) << "M matrix does not mismatch"; // Compare after 3 float point
    EXPECT_EQ(M, expect_M);
}
