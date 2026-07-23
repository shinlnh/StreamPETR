#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"
#include <pthread.h>

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: merge(PlanningResults& new_points)
 * Title: For case current_distance is smaller than est_distance
 * Input: PlanningResults new_points
 * Expected output: 
*/
TEST(PlanningQueue, merge_test01)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_points;
    planning_queue.root_position =  std::array<float, 3>{4.0f, 0.0f, 3.0f};
    planning_queue.current_position = std::array<float, 3>{0.0f, 0.0f, 3.0f};
    planning_queue.planning_results.resize(3, std::array<float, 3>{-2.0f, 0.0f, -3.0f});

    // Call the method under tests
    planning_queue.merge(new_points);
}

/**
 * @brief  TEST 2: merge(PlanningResults& new_points)
 * Title: For case current_distance is greater than est_distance 
 * and size of planning_result is smaller than 20 (The number of points is not enough for a merge)
 * Input: PlanningResults new_points
 * Expected output: 
*/
TEST(PlanningQueue, merge_test02)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;
    planning_queue.root_position =  std::array<float, 3>{4.0f, 0.0f, 3.0f};
    planning_queue.current_position = std::array<float, 3>{0.0f, 0.0f, 3.0f};

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_points;
    planning_queue.planning_results.resize(3, std::array<float, 3>{1.0f, 0.5f, 3.0f});

    // Call the method under tests
    planning_queue.merge(new_points);
}

/**
 * @brief  TEST 3: merge(PlanningResults& new_points)
 * Title: For case size of planning_result is greater than 20.
 * and jump the case our heading angle with the planning is not safe enough for a merge
 * Input: PlanningResults new_points
 * Expected output: 
*/
TEST(PlanningQueue, merge_test03)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_points;
    planning_queue.root_position =  std::array<float, 3>{0.0f, 0.0f, 3.0f};
    planning_queue.current_position = std::array<float, 3>{0.0f, 0.0f, 3.0f};
    planning_queue.planning_results.resize(22);
    planning_queue.planning_results[0] = std::array<float, 3> {2.0f, 3.0f, 3.0f};
    // Call the method under tests
    planning_queue.merge(new_points);
}

/**
 * @brief  TEST 4: merge(PlanningResults& new_points)
 * Title: For case new point only be marked if its distance with root point
 * is larger than the distance between the root point and the last planning point
 * Input: PlanningResults new_points
 * Expected output: 
*/
TEST(PlanningQueue, merge_test04)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_points;
    planning_queue.root_position =  std::array<float, 3>{1.0f, 1.0f, 3.0f};
    planning_queue.current_position = std::array<float, 3>{1.0f, 1.0f, 3.0f};

    //planning_queue.planning_results.resize(22, std::array<float, 3>{2.0f, 25.0f, 3.0f});
    planning_queue.planning_results.resize(22);
    planning_queue.planning_results[0] = std::array<float, 3>{2.0f, 25.0f, 3.0f};
    planning_queue.planning_results.push_back(std::array<float, 3>{1.0f, 1.0f, 2.0f}); 
    
    new_points.resize(2, std::array<float, 3>{0.0f, 0.0f, 0.0f});
    new_points.push_back(std::array<float, 3>{1.0f, 1.0f, 3.0f}); 
    new_points.push_back(std::array<float, 3>{2.0f, 2.11f, 180.0f}); 
    new_points.push_back(std::array<float, 3>{2.0f, 2.11f, 210.0f}); 
    new_points.push_back(std::array<float, 3>{2.0f, 2.11f, 1.0f}); 
    new_points.push_back(std::array<float, 3>{2.0f, 2.11f, 2.0f}); 

    // Call the method under tests
    planning_queue.merge(new_points);
}

/**
 * @brief  TEST 5: merge(PlanningResults& new_points)
 * Title: For case remove passed points from previous planning results
 * Input: PlanningResults new_points
 * Expected output: 
*/
// Thread 1 - calls printValue() of class A
void* thread1_function(void* arg) {
    PlanningQueueTest* obj = static_cast<PlanningQueueTest*>(arg);  // Cast argument to Class A
    std::vector<std::array<float, 3>> new_points;
    obj->merge(new_points);  // Call printValue() method
    return nullptr;
}

// Thread 2 - calls modifyValue() of class B
void* thread2_function(void* arg) {
    PlanningQueueTest* obj = static_cast<PlanningQueueTest*>(arg);  // Cast argument to Class B
    obj->change(obj->planning_results);  // Call modifyValue() method
    return nullptr;
}

TEST(PlanningQueue, merge_test05)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueueTest planning_queue;
    pthread_t thread1, thread2;

    // Define inputs for the test
    std::vector<std::array<float, 3>> new_points;
    planning_queue.root_position =  std::array<float, 3>{4.0f, 0.0f, 3.0f};
    planning_queue.current_position = std::array<float, 3>{0.0f, 0.0f, 3.0f};
    planning_queue.planning_results.resize(10000, std::array<float, 3>{2.0f, 0.0f, 3.0f});

    // Call the method under tests
    pthread_create(&thread1, nullptr, thread1_function, &planning_queue);
    pthread_create(&thread2, nullptr, thread2_function, &planning_queue);
    
    // Join threads to ensure both threads complete before proceeding
    pthread_join(thread1, nullptr);
    pthread_join(thread2, nullptr);
}

