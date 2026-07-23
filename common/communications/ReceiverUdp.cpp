#include "ReceiverUdp.hpp"
#include <stdlib.h>
#include <boost/chrono.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <opencv2/core.hpp>
#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>
#include "common.h"

static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;
    ReceiverUdp* self = static_cast<ReceiverUdp*>(user_data);

    if (!gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
        g_printerr("Failed to map RTP buffer\n");
        return GST_PAD_PROBE_OK;
    }
    // Check if the RTP packet has an extension
    if (gst_rtp_buffer_get_extension(&rtp_buffer)) {
        guint16 extension_bits;
        guint8 *extension_data;
        guint extension_length_words;
        if(gst_rtp_buffer_get_extension_data(&rtp_buffer, &extension_bits, (gpointer*)&extension_data, &extension_length_words))
        {
            guint extension_length_bytes = extension_length_words * 4; // Convert words to bytes

            // Process the data by skipping the first two bytes
            if (extension_length_bytes > 2) {
                guint64 value = 0;
                for (guint i = 2; i < extension_length_bytes; i++) {
                    value = (value << 8) | extension_data[i];
                }
                self->timeStamp = value;
                DEBUG("Decimal value after skipping first two bytes: %lld", self->timeStamp);
            }
        }
    } 
    gst_rtp_buffer_unmap(&rtp_buffer);
    return GST_PAD_PROBE_OK;
}

ReceiverUdp::ReceiverUdp(unsigned int width, unsigned int height)
    : Receiver(width, height)
{
    gst_init(nullptr, nullptr);

    GError *error = nullptr;
    std::string pipe_line = 
        "udpsrc name=udpsrc_element port=" + std::to_string(getPort()) + " ! "
        "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 ! "
        "rtph264depay name=depayloader ! h264parse ! avdec_h264 ! "
        "appsink name=appsink0 max-buffers=1 drop=true sync=false";

    pipeline = gst_parse_launch(pipe_line.c_str(), &error);
    if (error)
    {
        g_printerr("[ERROR]: %s\n", error->message);
        g_clear_error(&error);
        return;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink0");

    // Get the depayloader element by name
    depayloader = gst_bin_get_by_name(GST_BIN(pipeline), "udpsrc_element");

    // Get the src pad from the depayloader element
    src_pad = gst_element_get_static_pad(depayloader, "src");

    // Add a pad probe on the src pad to read the RTP header extensions
    gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, pad_probe_callback, this, NULL);
    
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

}

ReceiverUdp::~ReceiverUdp()
{
    if (pipeline)
    {
        // Set the pipeline and appsink to NULL state
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_element_set_state(appsink, GST_STATE_NULL);

        // Block until the pipeline is NULL state (or the timeout occurs)
        gst_element_get_state(pipeline, NULL, NULL, GST_CLOCK_TIME_NONE);
        gst_element_get_state(appsink, NULL, NULL, GST_CLOCK_TIME_NONE);

        // Unreference and release resources
        gst_object_unref(appsink);
        gst_object_unref(pipeline);
        gst_object_unref(src_pad);
        gst_object_unref(depayloader);
    }

    gst_deinit(); // Deinitialize GStreamer
}

cv::Mat ReceiverUdp::getNextFrame()
{
    cv::Mat nextFrame;

    // Check if pipeline and appsink is created
    if (!pipeline || !appsink) {
        INFO("Invalid pipeline or appsink");
        return nextFrame;
    }

    // Get current pipeline and appsink state
    GstState state, appSinkState;
    gst_element_get_state(pipeline, &state, NULL, GST_CLOCK_TIME_NONE);
    gst_element_get_state(GST_ELEMENT(appsink), &appSinkState, NULL, GST_CLOCK_TIME_NONE);

    if (state != GST_STATE_PLAYING || appSinkState != GST_STATE_PLAYING) {
        INFO("Pipeline or appsink not in playing state");
        return nextFrame;
    }

    // Pull data from appsink
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    if (sample == NULL) {
        INFO("Could not pull sample from appsink");
        return nextFrame;
    }

    // Retrieve frame buffer
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    unsigned int uvWidth = this->width / 2;
    unsigned int uvHeight = this->height / 2;

    // Convert YUV to BGR
    cv::Mat yuvImage(height + uvHeight, width, CV_8UC1, map.data);
    cv::cvtColor(yuvImage, nextFrame, cv::COLOR_YUV2BGR_I420);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

// Enable this section to put timstamp on the img
#if 0
    std::string timeStr = "Recv: " + std::to_string(this->timeStamp);
    cv::putText(nextFrame, timeStr, cv::Point(10, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, cv::LINE_AA);
#endif

    return nextFrame;
}
