#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: getCarLocation()
 * Title: 
 * Input: 
 * Expected output: 
*/
TEST(PlanningQueue, getCarLocation_test)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;
    
    // Call the method under tests
    planning_queue.getCarLocation();
}