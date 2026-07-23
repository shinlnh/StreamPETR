#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: getYaw()
 * Title: getting yaw smaller than 180 degree
 * Input: float car_yaw, float root_yaw 
 * Expected output: 15
*/
TEST(PlanningQueue, getYaw_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    planning_queue.car_yaw =  M_PI / 4;
    planning_queue.root_yaw = M_PI / 6;

    // Define expect resut for the test
    float expect_yaw = 15;

    // Call the method under tests
    float yaw = planning_queue.getYaw();

    // Compare between expected result and actual result
    EXPECT_EQ(yaw, expect_yaw);
}

/**
 * @brief  TEST 2: getYaw()
 * Title: getting yaw greater than 180 degree
 * Input: float car_yaw, float root_yaw 
 * Expected output: 285
*/
TEST(PlanningQueue, getYaw_test02)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    planning_queue.car_yaw = -M_PI / 6;
    planning_queue.root_yaw = M_PI / 4;

    // Define expect resut for the test
    float expect_yaw = 285;

    // Call the method under tests
    float yaw = planning_queue.getYaw();

    // Compare between expected result and actual result
    EXPECT_EQ(yaw, expect_yaw);
}