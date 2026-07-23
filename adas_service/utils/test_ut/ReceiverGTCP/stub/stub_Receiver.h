#ifndef STUB_RECEIVER_H
#define STUB_RECEIVER_

#include <opencv2/opencv.hpp>
#include <gst/gst.h>

class Receiver
{
protected:
    GstElement *pipeline;
    GstElement *appsink;
    unsigned int width = 800;
    unsigned int height = 600;

public:
    Receiver(unsigned int width, unsigned int height)
    {
        this->width = width;
        this->height = height;  
    };
    ~Receiver()
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    };
    virtual cv::Mat getNextFrame(){
        cv::Mat frame;

        GstSample *sample;

        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        frame = cv::Mat(480, 640, CV_8UC3, map.data);
        
        return frame;
    };
};

#endif // STUB_RECEIVER_HPP_
