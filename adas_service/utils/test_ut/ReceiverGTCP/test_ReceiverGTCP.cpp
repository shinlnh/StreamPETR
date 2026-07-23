#include <gtest/gtest.h>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @file        test_ReceiverGTCP.cpp
 * @brief       Test implementation for constructor of ReceiverGTCP 
 * @details     Includes Coverage tests
 */

/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_ReceiverGTCP_coverage_01
 * 
 * -----------------------------------------------------------------------------
 * Test Category:   Code Coverage
 * Test Type:       Unit Test
 * Coverage Type:   C0 (Statement Coverage)
 * -----------------------------------------------------------------------------
 * 
 * Prerequisites:
 *
 * Test Steps:
 *      1. Prepare input width and height data
 *      2. Declare object of class ReceiverGTCP
 *
 * Input:
 *      - Width             800                 
 *      - Height            600               
 *      - appsink           != nullptr 
 *      - pipeline          != nullptr
 * 
 * Expected Output:
 *      - No error codes returned
 *
 * Actual Output:
 *      - No error codes returned
 * 
 * Pass/Fail Criteria:
 *      - No exceptions thrown
 */
TEST(ReceiverGTCPTest, TC_ReceiverGTCP_Coverage_01)
{
    // Define the input
    unsigned int width = 800;
    unsigned int height = 600; 
    ReceiverGTCP regtcp(width, height);
}

