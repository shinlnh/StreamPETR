#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/*
* @brief TEST 1: MediaConfigure()
* Description:
* Input: 
* Expected output: run successfully
*
 */
// Test Case: Valid test with mocked GStreamer interactions
TEST_F(RtspServer_Test, MediaConfigure_1) {
    GstRTSPMediaFactory* factory= gst_rtsp_media_factory_new();

    GstRTSPMedia* media = GST_RTSP_MEDIA(g_object_new(GST_TYPE_RTSP_MEDIA,NULL)); // Mocked media
    
    gpointer user_data = &server;
    EXPECT_NO_THROW(server.media_configure(factory,media,user_data));
    g_object_unref(factory);
    g_object_unref(media);
}
