#include <gtest/gtest.h>
#include "fixture.h"

/**
 * @brief  TEST 1: init()
 * 
 * Input: init() with valid constructor
 * Expected output: run successfully
 * 
*/
TEST_F(LKF_TEST, LKF_TEST_INIT_1)
{
    EXPECT_NO_THROW
    ({      
        LinearKalmanFilter object(4,3,2);
        object.init();
    });
}
