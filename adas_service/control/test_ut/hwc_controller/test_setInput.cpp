#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: setInput(const PlanningResults& planning_results, ControlConfigs configs)
* Input: PlanningResults planning_results, ControlConfigs configs
* Expected output: 
*/
TEST(HWcController, setInputTest)
{
    // Define inputs for the constructor of function HWCController
    ACCController acc_controller;
    LKSController lks_controller;
    HWCController hwc_controller(acc_controller, lks_controller);

    // Define inputs for the test
    PlanningResults planning_results;
    ControlConfigs configs; 
    
    // Call the method under test
    hwc_controller.setInput(planning_results, configs);
}
