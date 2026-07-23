#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/*
/**
 * @brief  TEST 1: execute()
 * 
 * Description: created default case
 * Input:
 * Expected output: ControlResults
 * 
 * steering_value: 0.0f
 * throttle_value: 0.0f
 * brake_value: 0.0f
*/
TEST_F(ACCController_Default_Case, test_excute_01)
{
    ControlResults result = mock.execute(planning_results, distance_velocity, configs);

    EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
    EXPECT_FLOAT_EQ(result.throttle_value, 0.0f);
    EXPECT_FLOAT_EQ(result.brake_value, 0.0f);
}

/**
 * @brief  TEST 2: execute()
 * 
 * Description: created default case
 * Input:
 * Expected output: ControlResults
 * 
 * steering_value: 0.0f
 * throttle_value: 0.0f
 * brake_value: 1.0f
 * 
 */
TEST_F(ACCController_Default_Case, test_excute_02)
{    
    // Define input for the test
    mock.reference_velocity = -2.0f;

    // Call the function for the test
    ControlResults result = mock.execute(planning_results, distance_velocity, configs);
       
     // Compare result 
    EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
    EXPECT_FLOAT_EQ(result.throttle_value, 0.0f);
    EXPECT_FLOAT_EQ(result.brake_value, 1.0f);
}

/**
 * @brief  TEST 3: execute()
 * 
 * Description: created default case
 * Input:
 * Expected output: ControlResults
 * 
 * steering_value: 0.0f
 * throttle_value: 0.0f
 * brake_value: 0.0f
 * 
 */
TEST_F(ACCController_Default_Case, test_excute_03)
{
    // Define input for the test
    mock.detected_vehicle = true;

    // Call the function for the test
    ControlResults result = mock.execute(planning_results, distance_velocity, configs);

    // Compare result 
    EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
    EXPECT_FLOAT_EQ(result.throttle_value, 0.0f);
    EXPECT_FLOAT_EQ(result.brake_value, 0.0f);
}

/**
 * @brief  TEST 4: execute()
 * 
 * Description: created default case
 * Input:
 * Expected output: ControlResults
 * 
 * steering_value: 0.0f
 * throttle_value: 0.0f
 * brake_value: 2.0f
 * 
 */
TEST_F(ACCController_Default_Case, test_excute_04)
{
    // Define input for the test
    float my_car_velocity = 3.0f;
    mock.reference_velocity = -2.0f;

    // Call the function for the test
    ControlResults result = mock.execute(planning_results, my_car_velocity, configs);

    // Compare result 
    EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
    EXPECT_FLOAT_EQ(result.throttle_value, 0.0f);
    EXPECT_FLOAT_EQ(result.brake_value, 2.0f);
}
