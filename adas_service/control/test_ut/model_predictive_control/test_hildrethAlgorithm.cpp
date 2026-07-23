#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>
#include <Eigen/Dense>

#define CONTROL_STEPS 20

using ::testing::_;
using ::testing::Return;

/* @brief TEST 1: hildrethAlgorithm(const Eigen::Matrix<float, Nc, Nc>& H,
                            const Eigen::Matrix<float, Nc, 1>& f_matrix,
                            const Eigen::Matrix<float, 2 * Nc, Nc>& M,
                            const Eigen::Matrix<float, 2 * Nc, 1>& gamma_matrix,
                            const Eigen::Matrix<float, Nc, 1>& delta_control_signal)
*
* Input: Eigen::Matrix<float, Nc, Nc> H, Eigen::Matrix<float, Nc, 1> f_matrix, Eigen::Matrix<float, 2 * Nc, Nc> M, Eigen::Matrix<float, 2 * Nc, 1> gamma_matrix, Eigen::Matrix<float, Nc, 1> delta_control_signal
* Expected output: 1
*/
TEST(ModelPredictiveControlTest, hildrethAlgorithmTest_01) {
  ModelPredictiveControl mpc;
  // Define inputs for the test
  Eigen::Matrix<float, CONTROL_STEPS, CONTROL_STEPS> H;
  Eigen::Matrix<float, CONTROL_STEPS, 1> f_matrix;
  Eigen::Matrix<float, 4 * CONTROL_STEPS, CONTROL_STEPS> M;
  Eigen::Matrix<float, 4 * CONTROL_STEPS, 1> gamma_matrix;
  Eigen::Matrix<float, CONTROL_STEPS, 1> delta_control_signal;    

  // Declare initialize matrix
  H = Eigen::MatrixXf::Identity(CONTROL_STEPS, CONTROL_STEPS);
  f_matrix.setZero();
  M.setZero();
  
  gamma_matrix.setZero();
  for (int i = 0; i < CONTROL_STEPS; i++)
  {
    gamma_matrix.coeffRef(i, 0) = i*-15;
  }

  delta_control_signal.setOnes();

  // Call fucntion to test 
  Eigen::MatrixXf algo_result =  mpc.hildrethAlgorithm(H, f_matrix, M, gamma_matrix, delta_control_signal);
  float steer_value = algo_result(0,0);

  // Compare result of the test
  EXPECT_EQ(steer_value, 1);
}

/*
* @brief TEST 2: hildrethAlgorithm(const Eigen::Matrix<float, Nc, Nc>& H,
                            const Eigen::Matrix<float, Nc, 1>& f_matrix,
                            const Eigen::Matrix<float, 2 * Nc, Nc>& M,
                            const Eigen::Matrix<float, 2 * Nc, 1>& gamma_matrix,
                            const Eigen::Matrix<float, Nc, 1>& delta_control_signal)
*
* Input: Eigen::Matrix<float, Nc, Nc> H, Eigen::Matrix<float, Nc, 1> f_matrix, Eigen::Matrix<float, 2 * Nc, Nc> M, Eigen::Matrix<float, 2 * Nc, 1> gamma_matrix, Eigen::Matrix<float, Nc, 1> delta_control_signal
* Expected output: 0
*/
TEST(ModelPredictiveControlTest, hildrethAlgorithmTest_02) {
  ModelPredictiveControl mpc;

  // Define inputs for the test
  Eigen::Matrix<float, CONTROL_STEPS, CONTROL_STEPS> H;
  Eigen::Matrix<float, CONTROL_STEPS, 1> f_matrix;
  Eigen::Matrix<float, 4 * CONTROL_STEPS, CONTROL_STEPS> M;
  Eigen::Matrix<float, 4 * CONTROL_STEPS, 1> gamma_matrix;
  Eigen::Matrix<float, CONTROL_STEPS, 1> delta_control_signal;   

  // Declare initialize matrix
  H = Eigen::MatrixXf::Identity(CONTROL_STEPS, CONTROL_STEPS);
  f_matrix.setOnes();
  delta_control_signal.setZero();
  gamma_matrix.setZero();
  M.setOnes();

  // Call fucntion to test 
  Eigen::MatrixXf algo_result =  mpc.hildrethAlgorithm(H, f_matrix, M, gamma_matrix, delta_control_signal);
  float steer_value = algo_result(0,0);

  // Compare result of the test
  EXPECT_EQ(steer_value, 0);
}