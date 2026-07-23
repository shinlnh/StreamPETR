#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: tjc_controller()
 * Input:
 * Expected output:
*/
TEST(tjc_controller, test_01)
{
    TJAController tja_controller;
    LKSController lks_controller;

    TJCController tjc_controller(tja_controller, lks_controller);
}