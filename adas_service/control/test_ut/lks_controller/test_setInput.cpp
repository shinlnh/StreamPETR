#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: setInput(const PlanningResults& planning_results, ControlConfigs configs)
 * Input:
 * Expected output:
*/
TEST(lks_controller, setInput_test)
{
    // Define inputs for the constructor of function LKSController
    LKSController lks_controller;

    // Define inputs for the test
    PlanningResults planning_results;
    ControlConfigs configs;

    // Call the method under test
    lks_controller.setInput(planning_results, configs);

}