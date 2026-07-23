#include "ReceiverRtsp.hpp"
#include "rtsp_settings.h"
#include "common.h"
#include <gst/rtp/gstrtpbuffer.h>

ReceiverRtsp::ReceiverRtsp(unsigned int width, unsigned int height, std::string server_ip)
: Receiver(width, height)
{
    gst_init(nullptr, nullptr);

    GError *error = nullptr;

    std::string pipelineString = 
        "rtspsrc latency=0 location=rtsp://" + server_ip + ":8554/test ! "
        "rtpgstdepay name=depay0 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink max-buffers=1 sync=false";

    pipeline = gst_parse_launch(
        pipelineString.c_str(),
        &error);

    if (error)
    {
        g_printerr("[ERROR]: %s\n", error->message);
        g_clear_error(&error);
        return;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    
    // Get the depayloader element by name
    GstElement *depayloader = gst_bin_get_by_name(GST_BIN(pipeline), "depay0");

    // Get the src pad from the depayloader element
    GstPad *sink_pad = gst_element_get_static_pad(depayloader, "sink");

    // Add a pad probe on the src pad to read the RTP header extensions
    gst_pad_add_probe(sink_pad, GST_PAD_PROBE_TYPE_BUFFER, pad_probe_callback, this, NULL);
    gst_clear_object(&sink_pad);
    gst_clear_object(&depayloader);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    // Check if the pipeline is in the playing state
    GstState state;
    GstStateChangeReturn ret = gst_element_get_state(pipeline, &state, NULL, GST_CLOCK_TIME_NONE);

    if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING)
    {
        INFO("Failed to start the pipeline. Connection error or other issues.");
        return;
    }

    allow_grabbing = true;
}

ReceiverRtsp::~ReceiverRtsp()
{
    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
}

cv::Mat ReceiverRtsp::getNextFrame()
{
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 5 * GST_SECOND);

    if (sample != nullptr) 
    {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);

        // Assuming BGR format, adjust accordingly
        int width = RTSP_FRAME_WIDTH;
        int height = RTSP_FRAME_HEIGHT;

        // Convert to OpenCV Mat
        cv::Mat frame(height, width, CV_8UC3, map.data);

        // Cleanup
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

#if 0
        // Enable this section to put timstamp on the img
        std::string timeStr = "Recv: " + std::to_string(this->timestamp);
        cv::putText(frame, timeStr, cv::Point(300, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, cv::LINE_AA);
#endif

        return frame.clone();
    }

    // Check if the timeout occurred
    if (gst_app_sink_is_eos(GST_APP_SINK(appsink)))
    {
        // EOS (End of Stream) condition
        INFO("End of stream (EOS) reached.");
        allow_grabbing = false;
    }
    else
    {
        // Handle the timeout
        INFO("Timeout occurred. No frame received within the specified timeout.");
        allow_grabbing = false;
    }

    return cv::Mat();
}

GstPadProbeReturn ReceiverRtsp::pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    ReceiverRtsp* self = static_cast<ReceiverRtsp*>(user_data);
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;

    if (!gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
        INFO("Failed to map RTP buffer");
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
                self->timestamp = value;
                DEBUG("Decimal value after skipping first two bytes: %lld", self->timeStamp);
            }
        }
    } 
    gst_rtp_buffer_unmap(&rtp_buffer);
    return GST_PAD_PROBE_OK;
}