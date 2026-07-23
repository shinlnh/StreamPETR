#include <gtest/gtest.h>
#include "fixture.h"


/**
 * @brief  TEST 1: PREDICTION()
 * 
 * Input: an valid constructor
 * Expected output: run successfully
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_PREDICTION_1)
{
    EXPECT_NO_THROW
    ({      
        LinearKalmanFilter object(4,2,3);
        object.prediction();
    });
}

/**
 * @brief  TEST 2: prediction(const Eigen::MatrixXf &ego_states) 
 * 
 * Input: an valid constructor with a valid ego_states = INPUT_VARIABLE_SIZE
 * Expected output: run successfully
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_PREDICTION_2)
{
    Eigen::MatrixXf ego_states(3, 1);
    ego_states << 1.0, 2.0, 3.0;
    LinearKalmanFilter object(4,2,3);
    EXPECT_NO_THROW
    ({  
        object.prediction(ego_states);
    });
}

/**
 * @brief  TEST 3: prediction(const Eigen::MatrixXf &ego_states) 
 * 
 * Input: an valid constructor with a valid ego_states != INPUT_VARIABLE_SIZE
 * Expected output: log ERROR
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_PREDICTION_3)
{
    LinearKalmanFilter object(4,2,2);
    Eigen::MatrixXf ego_states(3, 1);
    ego_states << 1.0, 2.0, 3.0;
    EXPECT_NO_THROW
    ({  
        object.prediction(ego_states);
    });
}
/**
 * @brief  TEST 4: prediction(const Eigen::MatrixXf &ego_states) 
 * 
 * Input: an valid constructor with a valid ego_states = INPUT_VARIABLE_SIZE = 0
 * Expected output: No throw
 * Actual output: log ERROR
*/
TEST_F(LKF_TEST, LKF_TEST_PREDICTION_4)
{
    LinearKalmanFilter object(4,2,0);
    EXPECT_ANY_THROW
    ({  
        Eigen::MatrixXf ego_states(0, 1);
        object.prediction(ego_states);
    });
}

/** Fault Injection: prediction with null ego_states
 * @brief  TEST 5: prediction(const Eigen::MatrixXf &ego_states) 
 * 
 * Input: prediction with null ego_states
 * Expected output: No throw
 * Actual output: log ERROR
*/               
TEST_F(LKF_TEST, LKF_TEST_PREDICTION_5) {
    LinearKalmanFilter object(3, 2, 2);

    Eigen::MatrixXf ego_states;
    EXPECT_NO_THROW(object.prediction(ego_states));
}
