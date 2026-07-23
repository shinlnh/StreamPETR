#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: execute(const PlanningResults &planning_results, 
 *                         const float &my_car_velocity, ControlConfigs configs)
 * Input: PlanningResults planning_results, float my_car_velocity, ControlConfigs configs
 * Expected output: control_result.steering_value = 2.45553374, control_result.brake_value = 0, control_result.throttle_value = 0
*/
// TEST(lks_controller, execute_test01)
// {
//     // Define inputs for the constructor of function LKSController
//     LKSController lks_controller;   
    
//     // Define inputs for the test
//     PlanningResults planning_results;
//     float my_car_velocity = 10;
//     ControlConfigs configs; 
//     planning_results.push_back({1.0f, 2.0f, 3.0f}); 
    
//     // Define expect resut for the test
//     ControlResults expect_control_result;
        
//     // Call the method under test
//     expect_control_result = lks_controller.execute(planning_results, my_car_velocity, configs); 

//     // Compare expect result with actual resutl
//     EXPECT_FLOAT_EQ(expect_control_result.steering_value, 2.45553374);
//     EXPECT_EQ(expect_control_result.brake_value, 0);
//     EXPECT_EQ(expect_control_result.throttle_value, 0);
// }
