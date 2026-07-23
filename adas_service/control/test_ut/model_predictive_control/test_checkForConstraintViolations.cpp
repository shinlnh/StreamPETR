#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>
#include <Eigen/Dense>

#define CONTROL_STEPS 20

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: checkForConstraintViolations(const Eigen::Matrix<float, Nc, 1>& matrix_to_be_checked,
                                                          const Eigen::Matrix<float, 2 * Nc, Nc>& constraint_matrix,
                                                          const Eigen::Matrix<float, 2 * Nc, 1>& gamma_matrix) 
* Tittle:
* Input: Eigen::Matrix<float, Nc, 1> matrix_to_be_checked, onst Eigen::Matrix<float, 2 * Nc, Nc> constraint_matrix, Eigen::Matrix<float, 2 * Nc, 1> gamma_matrix
* Expected output: TRUE 
*
*/
TEST(ModelPredictiveControlTest, checkForConstraintViolationsTest_01) {
    
    ModelPredictiveControl mpc;
    
    // Define inputs for the test
    Eigen::Matrix<float, 4 * CONTROL_STEPS, 1> gamma_matrix;
    Eigen::Matrix<float, 4 * CONTROL_STEPS, CONTROL_STEPS> constraint_matrix;
    Eigen::Matrix<float, CONTROL_STEPS, 1> matrix_to_be_checked;

    // Declare initiallize  for Engen Matrix
    gamma_matrix.setZero();
    constraint_matrix.setZero();
    matrix_to_be_checked.setZero();
    constraint_matrix.coeffRef(1,1) = 1;
    matrix_to_be_checked.coeffRef(1,0) = 1;

    // Call the method under test
    bool expect_result;
    expect_result = mpc.checkForConstraintViolations(matrix_to_be_checked, constraint_matrix, gamma_matrix);
    
    // Expcet result
    EXPECT_EQ(true, expect_result);
}

/*
* @brief TEST 2: checkForConstraintViolations(const Eigen::Matrix<float, Nc, 1>& matrix_to_be_checked,
                                                          const Eigen::Matrix<float, 2 * Nc, Nc>& constraint_matrix,
                                                          const Eigen::Matrix<float, 2 * Nc, 1>& gamma_matrix) 
* Tittle:
* Input: Eigen::Matrix<float, Nc, 1> matrix_to_be_checked, Eigen::Matrix<float, 2 * Nc, Nc> constraint_matrix, Eigen::Matrix<float, 2 * Nc, 1> gamma_matrix
* Expected output: FALSE 
*
*/
TEST(ModelPredictiveControlTest, checkForConstraintViolationsTest_02) {
    
    ModelPredictiveControl mpc;

    // Define inputs for the test
    Eigen::Matrix<float, 4 * CONTROL_STEPS, 1> gamma_matrix;
    Eigen::Matrix<float, 4 * CONTROL_STEPS, CONTROL_STEPS> constraint_matrix;
    Eigen::Matrix<float, CONTROL_STEPS, 1> matrix_to_be_checked;

    // Declare initiallize  for Engen Matrix
    gamma_matrix.setZero();
    constraint_matrix.setZero();
    matrix_to_be_checked.setZero();

    // Call the method under test
    bool expect_result;
    expect_result = mpc.checkForConstraintViolations(matrix_to_be_checked, constraint_matrix, gamma_matrix);
    
    // Expcet result
    EXPECT_EQ(false, expect_result);
}
