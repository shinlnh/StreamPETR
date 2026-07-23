#ifndef RECEIVER_HPP_
#define RECEIVER_HPP_

#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

/**
 * @brief This is the Receiver interface, which must be subclassed to implement different receiving pipelines.
 * Currently UDP, TCP and RTSP are all supported. 
 * The interface exports a simple constructor that sets up the receiving resolution and a simple getNextFrame() function that returns a cv::Mat.
 * You can infer from the class name which class you want to use depends on the specific stream you are receiving.
 * ReceiverUdp, ReceiverGTCP and ReceiverRtsp are used to receive streams served with the UDP, TCP and RTSP respectively.
 * 
 */
class Receiver
{
protected:
    GstElement *pipeline;
    GstElement *appsink;
    // Sizes of the original image
    unsigned int width = 800;
    unsigned int height = 800;

public:
    Receiver(unsigned int width, unsigned int height);
    ~Receiver();
    #ifndef UT_TEST
        virtual cv::Mat getNextFrame() = 0;
    #else  
        virtual cv::Mat getNextFrame();
    #endif
};

#endif // RECEIVER_HPP_