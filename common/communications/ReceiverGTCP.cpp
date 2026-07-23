#include "ReceiverGTCP.hpp"
#include <stdlib.h>

ReceiverGTCP::ReceiverGTCP(unsigned int width, unsigned int height)
: Receiver(width, height)
{
    gst_init(nullptr, nullptr);

    GError *error = nullptr;
    pipeline = gst_parse_launch(
    	"tcpclientsrc host=192.168.240.100 port=8888 ! tsparse ! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! appsink",
        &error);

    if (error)
    {
        g_printerr("[ERROR]: %s\n", error->message);
        g_clear_error(&error);
        return;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink0");
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

ReceiverGTCP::~ReceiverGTCP()
{
}

cv::Mat ReceiverGTCP::getNextFrame()
{
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    if (sample != NULL)
    {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        unsigned int uvWidth = this->width / 2;
        unsigned int uvHeight = this->height / 2;

        // Assuming I420 format with separate planes for Y, U, and V
        cv::Mat yFrame(height, width, CV_8UC1, (void *)map.data);  // Y channel
        cv::Mat uFrame(uvHeight, uvWidth, CV_8UC1, (void *)(map.data + width * height));  // U channel
        cv::Mat vFrame(uvHeight, uvWidth, CV_8UC1, (void *)(map.data + width * height * 5 / 4));  // V channel

        // Create an empty BGR image
        cv::Mat bgrImage(height, width, CV_8UC3);

        // Convert YUV to BGR
        cv::cvtColor(cv::Mat(height + uvHeight, width, CV_8UC1, map.data), bgrImage, cv::COLOR_YUV2BGR_I420);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        return bgrImage;
    }
    return cv::Mat(); // Return an empty Mat if sample is null
}

