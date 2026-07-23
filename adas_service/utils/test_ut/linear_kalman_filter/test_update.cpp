#include <gtest/gtest.h>
#include "fixture.h"
#include <iostream>

/**
 * @brief  TEST 1: update(const Eigen::MatrixXf &z_measure)
 * 
 * Input: an valid constructor with a valid z_measure != MEASUREMENT_VECTOR_SIZE
 * Expected output: log ERROR
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_UPDATE_1)
{
    LinearKalmanFilter object(1,3,0);
    Eigen::MatrixXf z_measure(0, 1);
    EXPECT_NO_THROW
    ({      
        object.update(z_measure);
    });
}

/**
 * @brief  TEST 2: update(const Eigen::MatrixXf &z_measure)
 * 
 * Input: an valid constructor with a valid z_measure = MEASUREMENT_VECTOR_SIZE
 * Expected output: run successfully
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_UPDATE_2)
{
    LinearKalmanFilter object(1,3,0);
    Eigen::MatrixXf z_measure(3, 1);
    z_measure << 1.0, 2.0, 3.0;
    EXPECT_NO_THROW
    ({  
        object.update(z_measure);
    });
}

/**FAULT INJECTION: Null z_measure
 * @brief  TEST 3: update(const Eigen::MatrixXf &z_measure)
 * 
 * Input: an valid constructor with a invalid z_measure != MEASUREMENT_VECTOR_SIZE
 * Expected output: log ERROR
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_UPDATE_3)
{
    LinearKalmanFilter object(1,3,0);
    Eigen::MatrixXf z_measure;
    EXPECT_NO_THROW
    ({      
        object.update(z_measure);
    });
}

