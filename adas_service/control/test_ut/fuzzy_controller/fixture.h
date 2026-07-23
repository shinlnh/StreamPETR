#include <gtest/gtest.h>
#include "mocking.h"

class FuzzyController_Default_Case : public ::testing::Test
{
protected:
    FuzzyController fuzzy;
    float result;
    float expected;

    void SetUp() override
    {
        result = 0.0f;
        expected = 0.0f;
    }
    void TearDown() override
    {
    }
};

class MockFuzzyController_Default_Case : public ::testing::Test
{
protected:
    MockFuzzyController mock;
    float result;
    float expected;

    void SetUp() override
    {
        result = 0.0f;
        expected = 0.0f;
    }
    void TearDown() override
    {
    }
};

class MockFuzzyController_Custom_Default_Case : public ::testing::Test
{
protected:
    MockFuzzyController_Custom mock;
    float result;
    float expected;

    void SetUp() override
    {
        result = 0.0f;
        expected = 0.0f;
    }
    void TearDown() override
    {
    }
};