#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: execute(const PlanningResults &planning_results, 
 *                         const float &my_car_velocity, ControlConfigs configs)
 * Input: PlanningResults planning_results, float my_car_velocity, ControlConfigs configs
 * Expected output: control_results.steering_value = 5; control_results.brake_value = 5; control_results.throttle_value = 5
*/
TEST(tjc_controller, execute_test01)
{
    // Define inputs for the constructor of function TJCController
    TJAController tja_controller;
    LKSController lks_controller;
    TJCController tjc_controller(tja_controller, lks_controller);
        
    // Define inputs for the test
    PlanningResults planning_results;
    float my_car_velocity = 10;
    ControlConfigs configs;
    tjc_controller.car_acceleration = -1;   
    planning_results.points = {
        {1.0f, 2.0f, 3.0f},
        {4.0f, 5.0f, 6.0f},
        {7.0f, 8.0f, 9.0f}
    };

    // Define expect resut for the test
    ControlResults expect_control_result;
    
    // Call the method under test
    expect_control_result = tjc_controller.execute(planning_results, my_car_velocity, configs);

    // Compare expect result with actual resutl
    EXPECT_EQ(expect_control_result.steering_value, 5);
    EXPECT_EQ(expect_control_result.brake_value, 5);
    EXPECT_EQ(expect_control_result.throttle_value, 5);
}


/**
 * @brief  TEST 2: execute(const PlanningResults &planning_results, 
 *                         const float &my_car_velocity, ControlConfigs configs)
 * Input: PlanningResults planning_results, float my_car_velocity, ControlConfigs configs
 * Expected output: control_results.steering_value = 5, control_results.brake_value = 5, control_results.throttle_value = 5
*/
TEST(tjc_controller, execute_test02)
{
    // Define inputs for the constructor of function TJCController
    TJAController tja_controller;
    LKSController lks_controller;
    
    TJCController tjc_controller(tja_controller, lks_controller);
        
    // Define inputs for the test
    PlanningResults planning_results;
    float my_car_velocity = 10;
    ControlConfigs configs;
    tjc_controller.car_acceleration = 3;

    // Define expect resut for the test
    ControlResults expect_control_result;
    
    // Call the method under test
    expect_control_result = tjc_controller.execute(planning_results, my_car_velocity, configs);

    // Compare expect result with actual resutl
    EXPECT_EQ(expect_control_result.steering_value, 5);
    EXPECT_EQ(expect_control_result.brake_value, 5);
    EXPECT_EQ(expect_control_result.throttle_value, 5);
}
