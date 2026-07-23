#include <gtest/gtest.h>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 2_1: TrajectoryGenerator()
 * Input:
 * Expect output:
*/
TEST(TrajectoryGeneratorTest, test_TrajectoryGenerator_02_1)
{
    FrenetEnvironment _frenet_environment;
    TrajectoryGenerator TrafficGenerator(&_frenet_environment);
}

/**
 * @brief  TEST 2_2: TrajectoryGenerator()
 * Input:
 * Expect output:
*/
TEST(TrajectoryGeneratorTest, test_TrajectoryGenerator_02_2)
{
    TrajectoryGenerator TrafficGenerator(nullptr);
}

