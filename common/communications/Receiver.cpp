#include "Receiver.hpp"

Receiver::Receiver(unsigned int width, unsigned int height)
{
    this->width = width;
    this->height = height;
}

Receiver::~Receiver()
{
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

cv::Mat Receiver::getNextFrame()
{
    /*
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    unsigned int width = 1920;
    unsigned int height = 1080;
    unsigned int channels = 3;

    cv::Mat frame(height, width, CV_8UC3, (void *)map.data, width * channels);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return frame;
    */
    cv::Mat frame;

    GstSample *sample;

    g_signal_emit_by_name(appsink, "pull-sample", &sample);

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    frame = cv::Mat(480, 640, CV_8UC3, map.data);

    return frame;
}
