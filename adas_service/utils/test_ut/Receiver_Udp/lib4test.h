#ifndef LIB4TEST_H
#define LIB4TEST_H

#include <string.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class Receiver
{
public:
    GstElement *pipeline;
    GstElement *appsink;
    unsigned int width = 800;
    unsigned int height = 600;
    Receiver(unsigned int width, unsigned int height){}
    ~Receiver(){}
    virtual cv::Mat getNextFrame() = 0;
};

#endif