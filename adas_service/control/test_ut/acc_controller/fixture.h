#include <gtest/gtest.h>
#include "mocking.h"

class ACCController_Default_Case : public ::testing::Test
{
protected:
    ACCController mock;
    PlanningResults planning_results;
    ControlConfigs configs;
    ControlResults returnValue;
    float distance_velocity = 0.0F;

    void SetUp() override
    {
        configs.custom = false;
        configs.acc_following_enabled = false;
        configs.acc_error_following = 0.0F;

        mock.detected_vehicle = false;
    }
    void TearDown() override
    {
    }
};
