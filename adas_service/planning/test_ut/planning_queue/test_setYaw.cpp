#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: setYaw()
 * Title: set Yaw
 * Input: float yaw
 * Expected output: 
*/
TEST(PlanningQueue, setYaw_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    float yaw;

    // Call the method under tests
    planning_queue.setYaw(yaw);
}