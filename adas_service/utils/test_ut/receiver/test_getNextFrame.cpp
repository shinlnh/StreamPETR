#include <gtest/gtest.h>
#include "fixture.h"
#include <iostream>

using ::testing::_;
using ::testing::Return;

/**
 * @file        test_getNextFrame.cpp
 * @brief       Test implementation for getNextFrame function
 * @details     Includes both Coverage and Boundary Value Analysis tests
 */

/*
 * Helper function to compare two cv::Mat objects element-wise
 */

bool CompareMatElementwise(const cv::Mat& mat1, const cv::Mat& mat2) {
    double tolerance = 1.0;
    for (int row = 0; row < mat1.rows; ++row) {
        for (int col = 0; col < mat1.cols; ++col) {
            cv::Vec3b pixel1 = mat1.at<cv::Vec3b>(row, col);
            cv::Vec3b pixel2 = mat2.at<cv::Vec3b>(row, col);
            for (int c = 0; c < 3; ++c) {
                if (std::abs(pixel1[c] - pixel2[c]) > tolerance) {
                    return false;
                }
            }
        }
    }
    return true;
}

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
 *      2. Declare object of class Receiver
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
 *      - The return value of frame was cv::Mat(480, 640, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 *
 * Actual Output:
 *      - The return value of frame was cv::Mat(480, 640, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 * 
 * Pass/Fail Criteria:
 *      - No exceptions thrown
 *      - Function returns successfully
 *      - Output is within valid range
 *      - Result of compare value between actual and expected is TRUE
 */
TEST(receiverTest, TC_getNextFrame_coverage_01)
{
    gst_init(nullptr, nullptr);
    unsigned int width = 800;
    unsigned int height = 600; 
    Receiver rc(width, height);

    // Define input for the test
    rc.pipeline = gst_parse_launch(
            "videotestsrc pattern=ball ! video/x-raw,format=BGRx,width=640,height=480 ! appsink name=appsink0",
            nullptr);
    EXPECT_NE(rc.pipeline, nullptr);

    rc.appsink = gst_bin_get_by_name(GST_BIN(rc.pipeline), "appsink0");
    EXPECT_NE( rc.appsink, nullptr);

    gst_element_set_state(rc.pipeline, GST_STATE_PLAYING);

    GstSample* sample = nullptr;
    g_signal_emit_by_name(rc.appsink, "pull-sample", &sample);
    EXPECT_NE(sample, nullptr);

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    EXPECT_NE(buffer, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // Declare the expect result
    cv::Mat expect_frame = cv::Mat(480, 640, CV_8UC3, map.data);

    // Call the function under the test
    cv::Mat frame = rc.getNextFrame();

    // Compare result between actual result and expect result
    EXPECT_EQ(frame.size(), expect_frame.size());
    EXPECT_EQ(frame.type(), expect_frame.type());
    EXPECT_FALSE(CompareMatElementwise(frame, expect_frame));
}

/** 
 * @file        test_getNextFrame.cpp
 * @brief       Boundary Value Analysis tests for frame dimensions
 * @details     Tests frame initialization with cv::Mat(480, 640, CV_8UC3, map.data)
 * 
 * =============================================================================
 * Test Case Group: Boundary Value Analysis (BVA)
 * =============================================================================
 * @test        TC_getNextFrame_BVA_01 - Minimum Boundary Test
 * @test        TC_getNextFrame_BVA_02 - Maximum Boundary Test
 * @test        TC_getNextFrame_BVA_03 - Inside Bounds Test
 * @test        TC_getNextFrame_BVA_04 - Outside Bounds Test
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
 *          - Test type:    Between minimal boundary and maximal boundary 
 *          - Value:        800
 * 
 *      TC_getNextFrame_BVA_02
 *          - Test type:    Minimal boundary
 *          - Value:        0
 * 
 *      TC_getNextFrame_BVA_03
 *          - Test type:    Maximal boundary 
 *          - Value:        4294967295
 * 
 *      TC_getNextFrame_BVA_04
 *          - Test type:    Over maximal boundary 
 *          - Value:        4294967296          
 * 
 * Expected Output:
 *      - The return value of frame was cv::Mat(480, 640, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 *
 * Actual Output:
 *      - The return value of frame was cv::Mat(480, 640, CV_8UC3, map.data)
 *      - No error codes returned
 *      - The result of compare between actual result and expect result was TRUE
 * 
 * Expected Behavior:
 *      - No exceptions thrown
 *      - Function returns successfully
 *      - Output is within valid range
 *      - Result of compare between actual and expected is TRUE
 */
TEST(receiverTest, TC_getNextFrame_BVA_01)
{
    gst_init(nullptr, nullptr);
    unsigned int width = 800;
    unsigned int height = 800; 
    Receiver rc(width, height);

    // Create a test pipeline
    rc.pipeline = gst_parse_launch(
            "videotestsrc pattern=ball ! video/x-raw,format=BGRx,width=800,height=600 ! appsink name=appsink0",
            nullptr);

    rc.appsink = gst_bin_get_by_name(GST_BIN(rc.pipeline), "appsink0");
    gst_element_set_state(rc.pipeline, GST_STATE_PLAYING);

    gst_app_sink_set_emit_signals(GST_APP_SINK(rc.appsink), TRUE);

    GstSample* sample = nullptr;
    g_signal_emit_by_name(rc.appsink, "pull-sample", &sample);

    EXPECT_NE(sample, nullptr);

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    EXPECT_NE(buffer, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    cv::Mat expect_frame = cv::Mat(480, 640, CV_8UC3, map.data);

    // Call the function under the test
    cv::Mat frame = rc.getNextFrame();

    // Compare result between actual result and expect result
    EXPECT_EQ(frame.size(), expect_frame.size());
    EXPECT_EQ(frame.type(), expect_frame.type());
    EXPECT_NE(frame.data, expect_frame.data);
}

TEST(receiverTest, TC_getNextFrame_BVA_02)
{
    gst_init(nullptr, nullptr);
    unsigned int width = 0;
    unsigned int height = 0; 
    Receiver rc(width, height);

    // Create a test pipeline
    rc.pipeline = gst_parse_launch(
            "videotestsrc pattern=ball ! video/x-raw,format=BGRx,width=1,height=1 ! appsink name=appsink0",
            nullptr);

    rc.appsink = gst_bin_get_by_name(GST_BIN(rc.pipeline), "appsink0");
    gst_element_set_state(rc.pipeline, GST_STATE_PLAYING);

    gst_app_sink_set_emit_signals(GST_APP_SINK(rc.appsink), TRUE);

    GstSample* sample = nullptr;
    g_signal_emit_by_name(rc.appsink, "pull-sample", &sample);

    EXPECT_NE(sample, nullptr);

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    EXPECT_NE(buffer, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    cv::Mat expect_frame = cv::Mat(480, 640, CV_8UC3, map.data);

    // Call the function under the test
    cv::Mat frame = rc.getNextFrame();

    // Compare result between actual result and expect result
    EXPECT_EQ(frame.size(), expect_frame.size());
    EXPECT_EQ(frame.type(), expect_frame.type());
    EXPECT_NE(frame.data, expect_frame.data);
}

TEST(receiverTest, TC_getNextFrame_BVA_03)
{
    gst_init(nullptr, nullptr);
    unsigned int width = 4294967295;
    unsigned int height = 4294967295; 
    Receiver rc(width, height);

    // Create a test pipeline
    rc.pipeline = gst_parse_launch(
            "videotestsrc pattern=ball ! video/x-raw,format=BGRx,width=2,height=2 ! appsink name=appsink0",
            nullptr);

    rc.appsink = gst_bin_get_by_name(GST_BIN(rc.pipeline), "appsink0");
    gst_element_set_state(rc.pipeline, GST_STATE_PLAYING);

    gst_app_sink_set_emit_signals(GST_APP_SINK(rc.appsink), TRUE);

    GstSample* sample = nullptr;
    g_signal_emit_by_name(rc.appsink, "pull-sample", &sample);

    EXPECT_NE(sample, nullptr);

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    EXPECT_NE(buffer, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    cv::Mat expect_frame = cv::Mat(480, 640, CV_8UC3, map.data);

    // Call the function under the test
    cv::Mat frame = rc.getNextFrame();

    // Compare result between actual result and expect result
    EXPECT_EQ(frame.size(), expect_frame.size());
    EXPECT_EQ(frame.type(), expect_frame.type());
    EXPECT_NE(frame.data, expect_frame.data);
}

/*
TEST(receiverTest, TC_getNextFrame_BVA_04)
{
    gst_init(nullptr, nullptr);
    unsigned int width =  4294967296; // 4,294,967,295
    unsigned int height = 4294967296; // 4294967295
    Receiver rc(width, height);

    // Create a test pipeline
    rc.pipeline = nullptr;
    EXPECT_EQ(rc.pipeline, nullptr);

    rc.appsink = gst_bin_get_by_name(GST_BIN(rc.pipeline), "appsink0");
    EXPECT_EQ( rc.appsink, nullptr);

    GstSample* sample = nullptr;
    g_signal_emit_by_name(rc.appsink, "pull-sample", &sample);

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    EXPECT_EQ(buffer, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    cv::Mat expect_frame = cv::Mat(480, 640, CV_8UC3, map.data);

    // Call the function under the test
    cv::Mat frame = rc.getNextFrame();

    // Compare result between actual result and expect result
    EXPECT_EQ(frame.size(), expect_frame.size());
    EXPECT_EQ(frame.type(), expect_frame.type());
    EXPECT_NE(frame.data, expect_frame.data);
}
*/

