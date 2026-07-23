#include <gtest/gtest.h>
#include "fixture.h"
#include <math.h>

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: findMinVectorLength(const PlanningResults& planning_results,
                                const std::array<float, 3>& trajectory_vector, 
                                const uint8_t& index_begin)
*
* Input: PlanningResults planning_results, std::array<float, 3> trajectory_vector, uint8_t index_begin
* Expected output: 1
*/
TEST(ModelPredictiveControlTest, findMinVectorLengthTest_01) {
    ModelPredictiveControl mpc;

    // Define inputs for the test
    PlanningResults planning_results;
    planning_results.points.push_back({1012.0f, 2.0f, 3.0f}); 
    planning_results.points.push_back({2.0f, 3.0f, 4.0f});

    std::array<float, 3> trajectory_vector = {1.0f, 2.0f, 3.0f};
    uint8_t index_begin = 0;        // value of index begin

    // Expect output index
    uint8_t expect_index;
    expect_index = mpc.findMinVectorLength(planning_results, trajectory_vector, index_begin);
    
    EXPECT_EQ(1, expect_index);
}
