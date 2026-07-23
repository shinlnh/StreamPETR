#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mocking.h>
#include <iostream>

using ::testing::_;
using ::testing::Return;

class RtspServer_Test : public ::testing::Test
{
protected:
    GstElement* appsrc;
    guint unused;

    // MockGstElement mock_gst;
    RtspServer server;

    RtspServer::MyContext ctx1;
    void SetUp() override 
    {
        gst_init(nullptr, nullptr);
        appsrc = gst_element_factory_make("appsrc","test_source");
        ctx1.frame_pointer = nullptr;
        ctx1.timestamp= 0;
    }
    void TearDown() override 
    {
        if (appsrc)
        {
            gst_object_unref(appsrc);
        }
    }
};
