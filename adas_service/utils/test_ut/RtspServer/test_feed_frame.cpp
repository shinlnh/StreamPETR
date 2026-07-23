#include "fixture.h"

using ::testing::_;
using ::testing::Return;
/*
* @brief TEST 1: feed_frame()
* Description: 
* Input: 
* Expected output: run successfully
*
 */
TEST_F(RtspServer_Test,feed_frame_1)
{
    RtspServer module;
    uint64_t timestamp = 50;
    cv::Mat new_frame = cv::Mat(cv::Size(800, 600), CV_8UC3, cv::Scalar(0, 0, 0));
    module.feed_frame(new_frame, timestamp);

    EXPECT_EQ(timestamp, module.timestamp);
    EXPECT_EQ(new_frame.cols, module.frame.cols);
    EXPECT_EQ(new_frame.rows, module.frame.rows);

    cv::Mat diff;
    std::vector<cv::Mat> channels;
    cv::compare(new_frame, module.frame, diff, cv::CMP_NE);
    cv::split(diff, channels);
    int numDiffPixel = 0;
    for (auto mat: channels) {
        numDiffPixel += cv::countNonZero(mat);
    }
    EXPECT_EQ(numDiffPixel, 0);
}
