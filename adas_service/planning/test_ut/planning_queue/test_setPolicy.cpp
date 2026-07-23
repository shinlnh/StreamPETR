#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: setPolicy(planning_queue_policy policy)
 * Title: setPolicy
 * Input: planning_queue_policy policy
 * Expected output: 
*/
TEST(PlanningQueue, setPolicy_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    planning_queue_policy policy;

    // Call the method under tests
    planning_queue.setPolicy(policy);
}