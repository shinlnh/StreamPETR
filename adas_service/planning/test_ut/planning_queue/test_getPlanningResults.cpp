#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: getPlanningResults()
 * Title: check if planning_results has enough elements
 * Input: 
 * Expected output: 
*/
TEST(PlanningQueue, getPlanningResults_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    planning_queue.planning_results.resize(1, std::array<float, 3>{1.0f, 0.5f, 3.0f});

    // Call the method under tests
    planning_queue.getPlanningResults();
}

/**
 * @brief  TEST 2: getPlanningResults()
 * Title: Erase only if there are enough elements in planning results
 * Input: 
 * Expected output: 
*/
TEST(PlanningQueue, getPlanningResults_test02)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    planning_queue.planning_results.resize(2, std::array<float, 3>{1.0f, 0.5f, 3.0f});
    
    // Call the method under tests
    planning_queue.getPlanningResults();
}