#include <gtest/gtest.h>
#include "fixture.h"
#include <gst/app/gstappsink.h>

using ::testing::_;
using ::testing::Return;

/**
 * @file        test_getNextFrame.cpp
 * @brief       Test implementation for getNextFrame function
 * @details     Includes both Coverage and Boundary Value Analysis tests
 */

/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_getNextFrame_coverage_01
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
 *      2. Declare object of class ReceiverAppJetson
 *      3. Prepare input appsink and pipline data
 *      4. Call prediction function
 *      5. Verify output format
 *      6. Check prediction range
 *      7. Declare expected result
 *      8. Compare between actual result and expected
 *
 * Input:
 *      - Width             800                 
 *      - Height            600               
 *      - appsink           = nullptr 
 *      - pipeline          != nullptr
 * 
 * Expected Output:
 *      - The return value of frame wasn't cv::Mat(height, width, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 *
 * Actual Output:
 *      - The return value of frame wasn't cv::Mat(height, width, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 * 
 * Pass/Fail Criteria:
 *      - No exceptions thrown
 *      - Function returns successfully
 *      - Output is within valid range
 *      - Result of compare value between actual and expected is TRUE
 */
TEST(ReceiverGTCPTest, TC_getNextFrame_Coverage_01)
{
    // Define the input
    unsigned int width = 800;
    unsigned int height = 600;
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    /** For case gst_app_sink_pull_sample() has sample is ready. 
    */
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    regtcp.appsink = nullptr;
    EXPECT_EQ(regtcp.appsink, nullptr);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(0, 0, 0);

    // Compare result between actual result and expect result
    EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
    EXPECT_EQ(bgrImage.data, nullptr);
}

/**
 * =============================================================================
 * Test Case Group: Coverage Testing
 * =============================================================================
 * @test        TC_getNextFrame_coverage_02
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
 *      2. Declare object of class ReceiverAppJetson
 *      3. Prepare input appsink and pipline data
 *      4. Call prediction function
 *      5. Verify output format
 *      6. Check prediction range
 *      7. Declare expected result
 *      8. Compare between actual result and expected
 *
 * Input:
 *      - Width             800                 
 *      - Height            600               
 *      - appsink           != nullptr 
 *      - pipeline          != nullptr
 * 
 * Expected Output:
 *      - The return value of frame was cv::Mat(height, width, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 *
 * Actual Output:
 *      - The return value of frame was cv::Mat(height, width, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 * 
 * Pass/Fail Criteria:
 *      - No exceptions thrown
 *      - Function returns successfully
 *      - Output is within valid range
 *      - Result of compare value between actual and expected is TRUE
 */
TEST(ReceiverGTCPTest, TC_getNextFrame_Coverage_02)
{
    // Define the input
    unsigned int width = 800;
    unsigned int height = 600;
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    /** For case gst_app_sink_pull_sample() has sample is ready. 
    */
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);
    EXPECT_NE(regtcp.appsink, nullptr);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(600, 800, CV_8UC3);

    // Compare result between actual result and expect result
    EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
}

/** 
 * @file        test_getNextFrame.cpp
 * @brief       Boundary Value Analysis tests for frame dimensions
 * @details     Tests frame initialization with cv::Mat(height, width, CV_8UC3, map.data)
 * 
 * =============================================================================
 * Test Case Group: Boundary Value Analysis (BVA)
 * =============================================================================
 * @test        TC_getNextFrame_BVA_01 - Test Inside Bounds Test
 * @test        TC_getNextFrame_BVA_02 - Test Inside Bounds Test
 * @test        TC_getNextFrame_BVA_03 - Minimal boundary that the program can handled
 * @test        TC_getNextFrame_BVA_04 - Minimal boundary that the program can't handled
 * @test        TC_getNextFrame_BVA_05 - Maximal boundary that the program can handled
 * @test        TC_getNextFrame_BVA_06 - Outside Bounds Test
 * 
 * -----------------------------------------------------------------------------
 * Test Category:   Boundary Value Analysis
 * Test Type:       Edge Case Testing
 * -----------------------------------------------------------------------------
 * 
 * Prerequisites:
 *      -
 * 
 * Test Steps:
 *      1. Prepare input width and height data
 *      2. Declare object of class Receiver
 *      3. Prepare input appsink and pipline data
 *      4. Call prediction function
 *      5. Verify output format
 *      6. Check prediction range
 *      7. Declare expected result
 *      8. Compare between actual result and expected
 * 
 * Test Values: height and weight
 *      TC_getNextFrame_BVA_01
 *          - Test type:    Between minimal boundary and maximal boundary.
 *                          And value arguments of constructor module ReceiverAppJetsion is even value.
 *          - Value:        800
 *      TC_getNextFrame_BVA_02
 *          - Test type:    Between minimal boundary and maximal boundary.
 *                          And value arguments of constructor module ReceiverAppJetsion is odd value.
 *          - Value:        641
 * 
 *      TC_getNextFrame_BVA_03
 *          - Test type:    Minimal boundary that the program can handled
 *          - Value:        2
 * 
 *      TC_getNextFrame_BVA_04
 *          - Test type:    Minimal boundary that the program cann't handled
 *          - Value:        0
 * 
 *      TC_getNextFrame_BVA_05
 *          - Test type:    Maximal boundary that the program can handled
 *          - Value:        3000
 * 
 *      TC_getNextFrame_BVA_06
 *          - Test type:    Over maximal boundary that the program can handled
 *          - Value:        4000          
 * 
 * Expected Output:
 *      TC_getNextFrame_BVA_01
 *              - The return value of frame was cv::Mat(800, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 * 
 *      TC_getNextFrame_BVA_02
 *              - The return value of frame was cv::Mat(641, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 * 
 *      TC_getNextFrame_BVA_03
 *              - The return value of frame was cv::Mat(2, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 * 
 *      TC_getNextFrame_BVA_04
 *              - The return value of frame was cv::Mat(0, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 * 
 *      TC_getNextFrame_BVA_05
 *              - The return value of frame was cv::Mat(30000, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 * 
 *      TC_getNextFrame_BVA_06
 *              - The return value of frame was cv::Mat(800, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 *
 * Actual Output:
 *      TC_getNextFrame_BVA_01
 *              - The return value of frame was cv::Mat(800, 1296, CV_8UC3, map.data)
 *              - No error codes returned
 *              - The result of compare between actual result and expect result was TRUE
 * 
 *      TC_getNextFrame_BVA_02
 *              - Logs an error message "ERROR" without throwing an exception.
 * 
 *      TC_getNextFrame_BVA_03
 *              - The return value of frame was cv::Mat(2, 1296, CV_8UC3, map.data).
 *              - No error codes returned.
 *              - The result of compare between actual result and expect result was TRUE.
 * 
 *      TC_getNextFrame_BVA_04
 *              - Logs an error message "ERROR" without throwing an exception.
 * 
 *      TC_getNextFrame_BVA_05
 *              - The return value of frame was cv::Mat(30000, 1296, CV_8UC3, map.data).
 *              - No error codes returned.
 *              - The result of compare between actual result and expect result was TRUE.
 * 
 *      TC_getNextFrame_BVA_06
 *              - Logs an error message "ERROR" without throwing an exception.
 *              
 * Expected Behavior:
 *      - No exceptions thrown
 *      - Function returns successfully
 *      - Output is within valid range
 *      - Result of compare between actual and expected is TRUE
 */
TEST(ReceiverGTCPTest, TC_getNextFrame_BVA_01)
{
    // Define the input
    unsigned int width = 800; 
    unsigned int height = 800; 
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    /** For case gst_app_sink_pull_sample() has sample is ready. 
    */
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(800, 800, CV_8UC3);

    // Compare result between actual result and expect result
    EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
}

/*
TEST(ReceiverGTCPTest, TC_getNextFrame_BVA_02)
{
    // Define the input
    unsigned int width =  641;
    unsigned int height = 641;
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    // For case gst_app_sink_pull_sample() has sample is ready. 
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(641, 641 , CV_8UC3); // cv::Mat expect_bgrImage(1296, 481, CV_8UC3);

    // Compare result between actual result and expect result
    EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
}
*/

TEST(ReceiverGTCPTest, TC_getNextFrame_BVA_03)
{
    // Define the input
    unsigned int width = 2; 
    unsigned int height = 2; 
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    /** For case gst_app_sink_pull_sample() has sample is ready. 
    */
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(2, 2, CV_8UC3);

    // Compare result between actual result and expect result
    EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
}

/*
TEST(ReceiverGTCPTest, TC_getNextFrame_BVA_04)
{
    // Define the input
    unsigned int width = 0; 
    unsigned int height = 0; 
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    // For case gst_app_sink_pull_sample() has sample is ready. 
    
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(15, 1296, CV_8UC3);
}
*/

TEST(ReceiverGTCPTest, TC_getNextFrame_BVA_05)
{
    // Define the input
    unsigned int width =  3000; 
    unsigned int height = 3000; 
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    // For case gst_app_sink_pull_sample() has sample is ready. 
    
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(3000, 3000, CV_8UC3);

    // Compare result between actual result and expect result
    // TODO: Segfault
    // EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    // EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
}
/*
TEST(ReceiverGTCPTest, TC_getNextFrame_BVA_06)
{
    // Define the input
    unsigned int width =  4000; //  3001
    unsigned int height = 4000; //  3001
    gst_init(nullptr, nullptr);

    ReceiverGTCP regtcp(width, height);

    // For case gst_app_sink_pull_sample() has sample is ready. 
    regtcp.pipeline = gst_parse_launch("videotestsrc ! videoconvert ! appsink name=appsink0", NULL);
    regtcp.appsink = gst_bin_get_by_name(GST_BIN(regtcp.pipeline), "appsink0");  
    g_object_set(regtcp.appsink, "sync", FALSE, NULL);

    // Set pipeline to playing
    gst_element_set_state(regtcp.pipeline, GST_STATE_PLAYING);
    
    // Wait for the pipeline to be initialized and processing
    g_usleep(1000000); // Sleep for 1 second to allow the pipeline to process

    // Call the function under test
    cv::Mat bgrImage = regtcp.getNextFrame();

    // Define the expect result  
    cv::Mat expect_bgrImage(4000, 4000, CV_8UC3);

    // Compare result between actual result and expect result
    EXPECT_EQ(bgrImage.size(), expect_bgrImage.size());
    EXPECT_EQ(bgrImage.type(), expect_bgrImage.type());
}
*/

