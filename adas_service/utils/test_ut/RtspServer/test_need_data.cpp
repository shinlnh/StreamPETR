#include "fixture.h"

using ::testing::_;
using ::testing::Return;
/*
* @brief TEST 1: need_data()
* Description: ctx valid and frame_pointer valid
* Input: 
* Expected output: run successfully
*
 */
TEST_F(RtspServer_Test,need_data_1)
{
    RtspServer server;
    cv::Mat frame1 = cv::Mat::ones(1200, 1600, CV_8UC3); 
    std::mutex frame_mutex;
    std::condition_variable frame_ready; 
    
    ctx1.frame_pointer = &frame1;
    ctx1.frame_mutex = &frame_mutex;
    ctx1.frame_ready = &frame_ready;
    ctx1.timestamp = 0;
    server.need_data(appsrc, unused, &ctx1);
}

/*
* @brief TEST 2: need_data()
* Description: ctx valid and frame_pointer not null
* Input: 
* Expected output: run successfully
*
 */
TEST_F(RtspServer_Test,need_data_2)
{
    RtspServer server;
    ctx1.frame_pointer = new cv::Mat(); 
    ctx1.timestamp = 0;
    EXPECT_NO_THROW(server.need_data(appsrc, unused, &ctx1));
    delete ctx1.frame_pointer;
}

/*
* @brief TEST 3: need_data()
* Description: ctx valid and frame_pointer null
* Input: 
* Expected output: run successfully
*
 */
TEST_F(RtspServer_Test,need_data_3)
{
    RtspServer server;
    ctx1.frame_pointer = nullptr;
    ctx1.timestamp = 0;
    EXPECT_NO_THROW(server.need_data(appsrc, unused, &ctx1));
}

/*
* @brief TEST 4: need_data()
* Description: ctx null -> core dump
* Input: 
* Expected output: run successfully
*
 */
// TEST_F(RtspServer_Test,need_data_4)
// {
//     RtspServer server;
//     RtspServer::MyContext* ctx2= nullptr;
//     EXPECT_THROW(server.need_data(appsrc, unused, ctx2));
// }