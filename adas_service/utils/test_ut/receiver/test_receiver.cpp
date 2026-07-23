#include <gtest/gtest.h>
#include "fixture.h"
#include <iostream>

using ::testing::_;
using ::testing::Return;

/**
 * @file        test_receiver.cpp
 * @brief       Test implementation for constructor of Receiver 
 * @details     Includes Coverage tests
 */

/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_receiver_coverage_01
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
 *      2. Declare object of class Receiver
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
TEST(receiverTest, TC_receiver_coverage_01)
{
    // Define input for the test
    unsigned int width = 800;
    unsigned int height = 600; 
    Receiver rc(width, height);
    
    gst_init(nullptr, nullptr);
    rc.pipeline = gst_pipeline_new("example-pipeline");
    rc.appsink = gst_element_factory_make("fakesink", "sink");
}
