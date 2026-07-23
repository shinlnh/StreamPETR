#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: reset()
 * Title: reset
 * Input: 
 * Expected output: 
*/
TEST(PlanningQueue, reset_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Call the method under tests
    planning_queue.reset();
}