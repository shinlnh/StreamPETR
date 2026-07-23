#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: enqueue(std::vector<std::array<float, 3>> new_result)
 * Input: std::vector<std::array<float, 3>> new_result
 * Expected output: 
*/
TEST(PlanningQueue, enqueue_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_result;
    new_result.resize(0);

    // Call the method under test
    planning_queue.enqueue(new_result);
}

/**
 * @brief  TEST 2: enqueue(std::vector<std::array<float, 3>> new_result)
 * Titile: policy was merge statue
 * Input: std::vector<std::array<float, 3>> new_result
 * Expected output: 
*/
TEST(PlanningQueue, enqueue_test02)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_result;
    new_result.resize(3, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.planning_results.resize(0, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.policy = MERGING;

    // Call the method under test
    planning_queue.enqueue(new_result);
}

/**
 * @brief  TEST 3: enqueue(std::vector<std::array<float, 3>> new_result)
 * Titile: policy was NEWEST statue
 * Input: std::vector<std::array<float, 3>> new_result
 * Expected output: 
*/
TEST(PlanningQueue, enqueue_test03)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_result;
    new_result.resize(3, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.planning_results.resize(10, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.policy = NEWEST;

    // Call the method under test
    planning_queue.enqueue(new_result);
}

/**
 * @brief  TEST 4: enqueue(std::vector<std::array<float, 3>> new_result)
 * Titile: policy was REPLAN statue
 * Input: std::vector<std::array<float, 3>> new_result
 * Expected output: 
*/
TEST(PlanningQueue, enqueue_test04)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_result;
    new_result.resize(3, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.planning_results.resize(3, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.policy = REPLAN;

    // Call the method under test
    planning_queue.enqueue(new_result);
}

/**
 * @brief  TEST 5: enqueue(std::vector<std::array<float, 3>> new_result)
 * Titile: policy wasn't REPLAN, NEWEST, MERGE status
 * Input: std::vector<std::array<float, 3>> new_result
 * Expected output: 
*/
TEST(PlanningQueue, enqueue_test05)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_result;
    new_result.resize(3, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.planning_results.resize(3, std::array<float, 3>{1.0f, 3.0f, 3.0f});
    planning_queue.policy = static_cast <planning_queue_policy> (-10);

    // Call the method under test
    planning_queue.enqueue(new_result);
}