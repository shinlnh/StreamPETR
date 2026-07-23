#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: calculateVelCoef(float car_velocity, float max_heading_point, float car_acceleration) 
* Input: float car_velocity, float max_heading_point, float car_acceleration
* Expected output: 1
*/
TEST(HWcController, calculateVelCoefTest)
{
    // Define inputs for the constructor of function HWCController
    ACCController acc_controller;
    LKSController lks_controller;
    HWCController hwc_controller(acc_controller, lks_controller);

    // Define inputs for the test
    float car_velocity = 1.0;
    float max_heading_point = 0;
    float car_acceleration = 5;

    // Define expect resut for the test
    float expect_result = 1;
    
    // Call the method under test
    float result = hwc_controller.calculateVelCoef(car_velocity, max_heading_point, car_acceleration);

    // Compare expect result with actual resutl
    EXPECT_EQ(result, expect_result);
}
