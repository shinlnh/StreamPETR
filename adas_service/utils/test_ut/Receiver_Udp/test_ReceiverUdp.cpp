// #include <gtest/gtest.h>
// #include <iostream>
// #include "fixture.h"
// #include "mocking.h"
// #include <gst/gst.h>
// #include <gst/rtp/gstrtpbuffer.h>


// using ::testing::_;
// using ::testing::Return;

// /**
//  * @file        test_ReceiverUdp.cpp
//  * @brief       Test implementation for constructor of ReceiverUdp
//  * @details     Includes Coverage tests
//  */

// /**
//  * =============================================================================
//  * Test Case Group: Coverage Testing
//  * =============================================================================
//  * @test        TC_constructor_coverage_01
//  * 
//  * -----------------------------------------------------------------------------
//  * Test Category:   Code Coverage
//  * Test Type:       Unit Test
//  * Coverage Type:   C0 (Statement Coverage)
//  * -----------------------------------------------------------------------------
//  * 
//  * Prerequisites:
//  * 
//  * Test Steps:
//  *      1. Prepare input data vector
//  *      2. Create object of class ReceiverUdp
//  *      3. Delete the object
//  * 
//  * Input:
//  *      - Width:             1920
//  *      - Height:            1080
//  * 
//  * Expected Output:
//  *      - No error codes returned
//  *      - No exceptions thrown
//  * 
//  * Actual Output:
//  *      - No error codes returned
//  * 
//  * Pass/Fail Criteria:
//  *      - Function returns successfully
//  */
// TEST_F(ReceiverUdpTest, TC_constructor_coverage_01) {
//     ReceiverUdp* receiver = new ReceiverUdp(1920, 1080);
//     delete receiver;
//     ReceiverUdp* receiver1 = new ReceiverUdp(1920, 1080);
//     delete receiver1;
// }

// /**
//  * =============================================================================
//  * Test Case Group: Coverage Testing
//  * =============================================================================
//  * @test        TC_constructor_coverage_02
//  * 
//  * -----------------------------------------------------------------------------
//  * Test Category:   Code Coverage
//  * Test Type:       Unit Test
//  * Coverage Type:   C0 (Statement Coverage)
//  * -----------------------------------------------------------------------------
//  * 
//  * Prerequisites:
//  * 
//  * Test Steps:
//  *      1. Create object of class ReceiverUdp with invalid pipeline parameters
//  * 
//  * Input:
//  *      - Width:             0
//  *      - Height:            0
//  * 
//  * Expected Output:
//  *      - invalidReceiver.pipeline = nullptr
//  *      - No exceptions thrown
//  * 
//  * Actual Output:
//  *      - invalidReceiver.pipeline = nullptr
//  * 
//  * Pass/Fail Criteria:
//  *      - Function returns successfully
//  */
// TEST_F(ReceiverUdpTest, TC_constructor_coverage_02) {
//     ReceiverUdp invalidReceiver(0, 0);
//     EXPECT_EQ(invalidReceiver.pipeline, nullptr);
// }

// /**
//  * =============================================================================
//  * Test Case Group: Coverage Testing
//  * =============================================================================
//  * @test        TC_constructor_coverage_03
//  * 
//  * -----------------------------------------------------------------------------
//  * Test Category:   Code Coverage
//  * Test Type:       Unit Test
//  * Coverage Type:   C0 (Statement Coverage)
//  * -----------------------------------------------------------------------------
//  * 
//  * Prerequisites:
//  * 
//  * Test Steps:
//  *      1. Create object of class ReceiverUdp with valid pipeline parameters
//  *      2. Verify that all required components are initialized
//  * 
//  * Input:
//  *      - Width:             1920
//  *      - Height:            1080
//  * 
//  * Expected Output:
//  *      - receiver.pipeline != nullptr
//  *      - receiver.appsink != nullptr
//  *      - receiver.depayloader != nullptr
//  *      - receiver.src_pad != nullptr
//  *      - No exceptions thrown
//  * 
//  * Actual Output:
//  *      - All components initialized successfully
//  * 
//  * Pass/Fail Criteria:
//  *      - Function returns successfully
//  */
// TEST_F(ReceiverUdpTest, TC_constructor_coverage_03) {
//     ReceiverUdp* receiver = new ReceiverUdp(1920, 1080);
//     EXPECT_NE(receiver->pipeline, nullptr);
//     EXPECT_NE(receiver->appsink, nullptr);
//     EXPECT_NE(receiver->depayloader, nullptr);
//     EXPECT_NE(receiver->src_pad, nullptr);
//     delete receiver;
// }
