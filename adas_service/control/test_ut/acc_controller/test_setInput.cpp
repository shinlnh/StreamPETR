#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: setInput()
 * 
 * Description: 
 * Input:
 * Expected output:
 * 
*/
TEST_F(ACCController_Default_Case, test_setInput)
{
    mock.setInput(planning_results, configs);
}