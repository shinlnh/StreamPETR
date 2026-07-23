#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: setCarLocation(float x, float y, float z)
 * Title: set CarL Location
 * Input: float x, float y, float z
 * Expected output: 
*/
TEST(PlanningQueue, setCarLocation_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    float y;
    float z;
    float x = y = z = 3.0;

    // Call the method under tests
    planning_queue.setCarLocation(x, y, z);
}

