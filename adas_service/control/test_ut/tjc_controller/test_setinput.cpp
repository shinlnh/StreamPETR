#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;


/**
 * @brief  TEST 1: setInput(const PlanningResults& planning_results, ControlConfigs configs)
 * Input: planning_results, configs
 * Expected output:
*/
TEST(tjc_controller, setInputTest)
{
    // Define inputs for the constructor of function TJCController
    TJAController tja_controller;
    LKSController lks_controller;
    TJCController tjc_controller(tja_controller, lks_controller);

    // Define inputs for the test
    PlanningResults planning_results;
    ControlConfigs configs;
    
    // Call the method under test
    tjc_controller.setInput(planning_results, configs);
}