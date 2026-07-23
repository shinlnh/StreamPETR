#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: HWcControllerT()
 * Input:
 * Expected output:
*/
TEST(HWcController, HWcControllerTest)
{
    // Define inputs for the constructor of function HWCController
    ACCController acc_controller;
    LKSController lks_controller;
    HWCController hwc_controller(acc_controller, lks_controller);
}
