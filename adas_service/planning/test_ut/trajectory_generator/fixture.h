#include <gtest/gtest.h>
#include "mocking.h"

class TrajectoryGenerator_Default_Case : public ::testing::Test
{
protected:
    TrajectoryGenerator Trajectory_Generator;
    FrenetEnvironment _frenet_environment;
    MockTrajectory_generator mock_Trajectory_Generator;

    void SetUp() override
    {

    }
    void TearDown() override
    {
    }
};
