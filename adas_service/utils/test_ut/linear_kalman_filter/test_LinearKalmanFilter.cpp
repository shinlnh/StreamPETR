#include <gtest/gtest.h>
#include "fixture.h"

/**
 * @brief  TEST 1: Constructor_1()
 * 
 * Input: an invalid constructor
 * Expected output: terminated or throw an exception
 * EXPECT_DEATH or EXPECT_ANY_THROW
*/
TEST_F(LKF_TEST, LKF_TEST_Con_Destructor_1)
{
    // EXPECT_ANY_THROW({
    // LinearKalmanFilter object;
    // });
}

/**
 * @brief  TEST 2: Constructor_2(parameters)
 * 
 * Input: valid constructor with valid parameters
 * Expected output: run correctly
 *
*/
TEST_F(LKF_TEST, LKF_TEST_Con_Destructor_2)
{
    int state_vector_size = 4;
    int measurement_vector_size = 2;
    int input_variable_size = 3;

    EXPECT_NO_THROW({
        LinearKalmanFilter filter(state_vector_size, measurement_vector_size, input_variable_size);
        EXPECT_EQ(filter.x_n_n.size(), state_vector_size);
        EXPECT_EQ(filter.x_n1_n.size(), state_vector_size);
        EXPECT_EQ(filter.z_measurement.size(), measurement_vector_size);
        EXPECT_EQ(filter.u_input.size(), input_variable_size);
        EXPECT_EQ(filter.F_trans.rows(), state_vector_size);
        EXPECT_EQ(filter.F_trans.cols(), state_vector_size);
    });
}

/** BOUNDARY METHOD
 * @brief  TEST 3: Constructor(parameters)
 * 
 * Input: valid constructor with invalid parameters
 * Smallest valid inputs
 * 
 * Expected output: run successfully
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_Con_Destructor_3) {
    EXPECT_NO_THROW({
        LinearKalmanFilter object(1, 1, 1);
    });
}

/** BOUNDARY METHOD
 * @brief  TEST 4: Constructor(parameters)
 * 
 * Input: valid constructor with invalid parameters
 * very large valid inputs
 * 
 * Expected output: run successfully
 * 
*/
// TEST_F(LKF_TEST, LKF_TEST_Con_Destructor_4) {
//     /* Test with large valid input */
//     EXPECT_NO_THROW({
//         LinearKalmanFilter object(10000, 10000, 10000);
//     });

//     /*  Stress test with values close to INT_MAX (if system allows) */
//     EXPECT_NO_THROW({
//         LinearKalmanFilter object(INT_MAX / 10, INT_MAX / 10, INT_MAX / 10);
//     });
// }

/**FAULT INJECTION
 * @brief  TEST 6: Constructor(parameters)
 * 
 * Input: valid constructor with invalid parameters
 * negative value
 * Expected output: terminated or throw an exception
 * EXPECT_DEATH or EXPECT_ANY_THROW
*/
// TEST_F(LKF_TEST, LKF_TEST_Con_Destructor_6)
// {
//     EXPECT_ANY_THROW({
//         LinearKalmanFilter filter(-1, 1, 1);
//     });
//     EXPECT_ANY_THROW({
//         LinearKalmanFilter filter(1, -1, 1);
//     });
//     EXPECT_ANY_THROW({
//         LinearKalmanFilter filter(1, 1, -1);
//     });
// }
