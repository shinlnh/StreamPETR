#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mocking.h>

#include "frenet_environment.hpp"

using ::testing::_;
using ::testing::Return;
using ::testing::IsEmpty;
using ::testing::ElementsAre;

class FrenetEnvironment_Default_Case : public ::testing::Test
{
protected:
    FrenetEnvironment FrEn;
    FrenetEnvironment frenet_env;
    MockFrenetEnvironment mock_frenet_env;

    void SetUp() override {}
    void TearDown() override {}
};

class FrenetEnvironmentTest : public ::testing::Test {
protected:
    FrenetEnvironment env;
   
    void SetUp() override {

    }

    void TearDown() override {

    }
};

