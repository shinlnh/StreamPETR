#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>

#define PREDICTED_STEPS 37
#define CONTROL_STEPS 20

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: createQuadraticProgrammingAlgorithmMatrices(Eigen::Matrix<float, 3 * Np, 4>& F,
                                                     Eigen::Matrix<float, 3 * Np, Nc>& G)
* Tittle:
* Input: Eigen::Matrix<float, 3 * Np, 4> F, Eigen::Matrix<float, 3 * Np, Nc> G
* Expected output: 
*/
TEST(ModelPredictiveControlTest, createQuadraticProgrammingAlgorithmMatricesTest_01) {  
    ModelPredictiveControl mpc;

    // Define inputs for the test
    Eigen::Matrix<float, 3 * PREDICTED_STEPS, 4> F;
    Eigen::Matrix<float, 3 * PREDICTED_STEPS, CONTROL_STEPS> G;

    // Call the method under test
    mpc.createQuadraticProgrammingAlgorithmMatrices(F, G);   
}
