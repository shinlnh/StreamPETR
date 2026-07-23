#include <gtest/gtest.h>
#include "mocking.h"

class ACCController_Default_Case : public ::testing::Test
{
protected:
    ACCController acc;
    ACCController::FuzzyParameters fuzzy_parameters;
    PlanningResults planning_results;
    ControlConfigs configs;
    ControlResults result;

    int episode = 100000;
    float actual_distance = 0.0f;
    float actual_velocity = 0.0f;
    float velocity_of_car_ahead = 0.0f;
    float reference_velocity = 0.0f;
    float reference_distance = 0.0f;
    bool detected_vehicle;

    void SetUp() override
    {
        configs.custom = false;
        configs.acc_following_enabled = false;
        configs.acc_error_following = 0.0F;

        detected_vehicle = false;
    }
    void TearDown() override
    {
    }
};
