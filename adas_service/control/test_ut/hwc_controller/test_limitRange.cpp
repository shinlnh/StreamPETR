#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: limitRange(float* x, float upper_limit, float lower_limit)
* Input: float *x, float upper_limit, float lower_limit
* Expected output: 5 
*/
TEST(HWcController, LimitRangeTest)
{
    // Define inputs for the constructor of function HWCController
    ACCController acc_controller;
    LKSController lks_controller;
    HWCController hwc_controller(acc_controller, lks_controller);

    // Define inputs for the test
    float x = 0;
    float upper_limit = 10;
    float lower_limit = 5;

    // Define expect resut for the test
    float expect_result = 5;
    
    // Call the method under test
    hwc_controller.limitRange(&x, upper_limit, lower_limit);

    // Compare expect result with actual resutl
    EXPECT_EQ(x, expect_result);
}
