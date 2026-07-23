#ifndef __RTSPSERVER_H__
#define __RTSPSERVER_H__

#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <condition_variable>


/**
 * @brief The server object is the object listening for connections on a port and creating GstRTSPClient objects to handle those connections.
 * This design provides a centralized model where multiple clients can access a stream through a single server.
 */
class RtspServer
{
private:
    bool is_running = false;
    bool has_frames = false;
    GstRTSPServer* server;
    GMainLoop* loop;
    GstElement* appsrc;
    GstRTSPMediaFactory* factory;

    typedef struct {
        GstClockTime pts;
        uint64_t *timestamp;
        cv::Mat *frame_pointer;
        std::mutex *frame_mutex;
        std::condition_variable* frame_ready;
    } MyContext;

    static void need_data(GstElement* appsrc, guint unused, MyContext* ctx);
    static void media_configure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data);
    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);

public:
    RtspServer(/* args */);
    ~RtspServer();

    std::mutex frame_mutex;
    std::condition_variable frame_ready; 
    cv::Mat frame;
    uint64_t timestamp;
    void start_server();
    void stop_server();
    void feed_frame(cv::Mat new_frame, uint64_t timestamp);
    void setRunning(bool value) {
        this->is_running = value;
    }
    bool getRunning() const {
        return this->is_running;
    }
    void setHasFrames(bool value) {
        this->has_frames = value;
    }
    bool getHasFrames() const {
        return this->has_frames;
    }
};



#endif // __RTSPSERVER_H__W