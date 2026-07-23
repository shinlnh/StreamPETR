#include <gtest/gtest.h>
#include "mocking.h"

class AebController_Default_Case : public ::testing::Test
{
public:
    AEBController aeb;
    ControlResults control_results;
    
    float x;
    float result;
    float expected;

    void SetUp() override
    {
        x = 0.0f;
        result = 0.0f;
        expected = 0.0f;
    }
    void TearDown() override
    {
    }
};

class AebController_Check_Bool : public ::testing::Test
{
public:
    AEBController aeb;
    ControlResults control_results;
    
    bool result;

    void SetUp() override
    {
        result = false;
    }
    void TearDown() override
    {
    }
};

class AebController_timeToCollision_Case : public ::testing::Test
{
public:
    AEBController aeb;
    ControlResults control_results;
    
    float result;
    float expected;
    float distance_to_car_ahead;
    float leading_car_velocity;
    float following_car_velocity;
    float relative_acceleration;

    void SetUp() override
    {
        result = 0.0f;
        expected = 0.0f;
        distance_to_car_ahead = 0.0f;
        leading_car_velocity = 0.0f;
        following_car_velocity = 0.0f;
        relative_acceleration = 0.0f;
    }
    void TearDown() override
    {
    }
};

class AebController_timeStop_Case : public ::testing::Test
{
public:
    AEBController aeb;
    ControlResults control_results;
    
    float result;
    float expected;
    float distance_to_car_ahead;
    float leading_car_velocity;
    float following_car_velocity;

    void SetUp() override
    {
        result = 0.0f;
        expected = 0.0f;
        distance_to_car_ahead = 0.0f;
        leading_car_velocity = 0.0f;
        following_car_velocity = 0.0f;
    }
    void TearDown() override
    {
        ASSERT_GE(leading_car_velocity, 0) << "Expected: " << expected << " Result: " << result;
        EXPECT_GT(result, 0) << "Expected: " << expected << " Result: " << result;
    }
};

class AebController_dangerCheck_Case : public ::testing::Test
{
public:
    MockAEBController mock_aeb;
    float distance_to_car_ahead;
    float leading_car_velocity;
    float following_car_velocity;
    float longitudinal_acceleration;
    bool is_reverse;
    bool result;

    void SetUp() override
    {
        distance_to_car_ahead = 0.0f;
        leading_car_velocity = 0.0f;
        following_car_velocity = 0.0f;
        longitudinal_acceleration = 0.0f;
        is_reverse = false;
        result = false;
    }
    void TearDown() override
    {
    }
};