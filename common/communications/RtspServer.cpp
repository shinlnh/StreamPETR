#include "RtspServer.hpp"
#include "rtsp_settings.h"
#include "common.h"
#include <boost/format.hpp>

/**
 * @brief Construct a new Rtsp Server:: Rtsp Server object
 * The RtspServer constructor initializes the GStreamer library, creates a new RTSP server, and configures a media factory to handle a specific streaming pipeline. 
 * It must be noted that this specific server assumes the server is deployed on NVIDIA hardware with the installed GStreamer components.
 * The pipeline is designed to stream video frames in the BGRx format which is tested successfully by the Jetson Nano and Xavier.
 * However this pipeline is in no way mandatory. User can configure the pipeline in the gst_rtsp_media_factory_set_launch() function.
 * After all this is done a server is created locally on the machine and can handle connection from other clients in the network.
 */
RtspServer::RtspServer()
{
    gst_init(NULL, NULL);
    loop = g_main_loop_new(NULL, FALSE);
    server = gst_rtsp_server_new();
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    factory = gst_rtsp_media_factory_new();

    boost::format pipeline_fmt = boost::format(
        "appsrc name=mysrc ! "
        "videoconvert ! video/x-raw,format=BGR,width=%1%,height=%2% ! "
        "jpegenc ! rtpgstpay name=pay0 pt=96")
        % RTSP_FRAME_WIDTH % RTSP_FRAME_HEIGHT;

    gst_rtsp_media_factory_set_launch(factory, pipeline_fmt.str().c_str());
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure), this);
    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);
    g_object_unref(mounts);
    gst_rtsp_server_attach(server, NULL);
}

RtspServer::~RtspServer()
{
    g_main_loop_unref(loop);
}

void RtspServer::start_server()
{
    //g_print("stream ready at rtsp://127.0.0.1:8554/test\n");
    g_main_loop_run(loop);
}

void RtspServer::stop_server()
{
    g_main_loop_quit(loop);
}

void RtspServer::feed_frame(cv::Mat new_frame, uint64_t timestamp)
{
    std::lock_guard<std::mutex> lock(this->frame_mutex);
    this->frame = new_frame;
    this->timestamp = timestamp;
    this->frame_ready.notify_all();
}

/**
 * @brief This callback is called everytime the pipeline needs new data.
 * It generates new data and serve this data to the streaming pipeline.
 * The 2 first variables are filled automatically by GStreamer.
 * The ctx argument is passed in from the media_configure function.
 * 
 * @param appsrc 
 * @param unused 
 * @param ctx User created context (that has data and timestamp), this aids in the data generatation process.
 */
void RtspServer::need_data(GstElement* appsrc, guint unused, MyContext* ctx)
{
    GstBuffer* buffer;
    guint size;
    GstMapInfo map;
    GstFlowReturn ret;

    // Define image properties
    int width = RTSP_FRAME_WIDTH;
    int height = RTSP_FRAME_HEIGHT;
    int channels = 3;

    size = width * height * channels;

    buffer = gst_buffer_new_allocate(NULL, size, NULL);

    // Map the buffer for writing
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);

    cv::Mat frame;
    try {
        if (ctx && ctx->frame_pointer) {

            std::unique_lock<std::mutex> lock(*(ctx->frame_mutex));
            ctx->frame_ready->wait(lock, [ctx] {return !ctx->frame_pointer->empty(); });

            frame = ctx->frame_pointer->clone();

            // Check if the frame is empty before resizing
            if (frame.empty()) {
                DEBUG("Received an empty frame");
                frame = cv::Mat::zeros(height, width, CV_8UC3);
            } else {
                // Resize the image to the desired height and width
                cv::resize(frame, frame, cv::Size(width, height));

            #if 0
                // Enable this section to put timstamp on the img
                std::string timeStr = "Send: " + std::to_string(*(ctx->timestamp));
                cv::putText(frame, timeStr, cv::Point(300, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, cv::LINE_AA);
            #endif
            }
        } else {
            DEBUG("Invalid frame pointer");
            frame = cv::Mat::zeros(height, width, CV_8UC3);
        }
    } catch (const cv::Exception& ex) {
        DEBUG("Unable to grab frame");
        // Handle OpenCV exception
        frame = cv::Mat::zeros(height, width, CV_8UC3);
    } catch (const std::exception& ex) {
        DEBUG("Code encounters undefined error, please check.");
        frame = cv::Mat::zeros(height, width, CV_8UC3);
    }

    // Copy the frame data to the GStreamer buffer
    std::memcpy(map.data, frame.data, size);

    // Unmap the buffer
    gst_buffer_unmap(buffer, &map);

    // Set presentation timestamp and duration
    GST_BUFFER_PTS(buffer) = ctx->pts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30);
    ctx->pts += GST_BUFFER_DURATION(buffer);

    // Push the buffer to appsrc
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);

    // Unref the buffer
    gst_buffer_unref(buffer);
}

/**
 * @brief This callback is called whenever there is a new connection to the server. 
 * The 2 first variables are filled automatically by GStreamer.
 * user_data can be passed in using any type of pointer. 
 * This, combined with casting, allows users to pass in pointers to generating classes which will generate data for the need_data function.
 * 
 * @param factory 
 * @param media 
 * @param user_data
 */
void RtspServer::media_configure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data)
{
    INFO("New connection created.");
    RtspServer* self = static_cast<RtspServer*>(user_data);

    GstElement *element, *appsrc;
    MyContext *ctx;

    /* get the element used for providing the streams of the media */
    element = gst_rtsp_media_get_element(media);

    /* get our appsrc, we named it 'mysrc' with the name property */
    appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
    /* configure the caps of the video */
    g_object_set(G_OBJECT(appsrc), "caps",
                gst_caps_new_simple("video/x-raw",
                                    "format", G_TYPE_STRING, "BGR",
                                    "width", G_TYPE_INT, RTSP_FRAME_WIDTH,
                                    "height", G_TYPE_INT, RTSP_FRAME_HEIGHT,
                                    "framerate", GST_TYPE_FRACTION, 30, 1, NULL), NULL);

    ctx = g_new0(MyContext, 1);
    ctx->frame_pointer = &(self->frame);
    ctx->frame_mutex = &(self->frame_mutex);
    ctx->frame_ready = &(self->frame_ready);
    ctx->timestamp = &(self->timestamp);
    ctx->pts = 0;
    /* make sure the data are freed when the media is gone */
    g_object_set_data_full(G_OBJECT(media), "my-extra-data", ctx,
                            (GDestroyNotify)g_free);

    /* install the callback that will be called when a buffer is needed */
    g_signal_connect(appsrc, "need-data", (GCallback)need_data, ctx);
    gst_clear_object(&appsrc);

    // Setup RTP payloader and probe
    GstElement *payloader = gst_bin_get_by_name(GST_BIN(element), "pay0");
    GstPad *src_pad = gst_element_get_static_pad(payloader, "src");

    gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER_LIST, pad_probe_callback, ctx, NULL);
    gst_clear_object(&src_pad);
    gst_clear_object(&payloader);

    gst_clear_object(&element);
}


GstPadProbeReturn RtspServer::pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    (void)pad;    // avoid warning
    auto ctx = static_cast<MyContext*>(user_data);

    GstBufferList *list = gst_pad_probe_info_get_buffer_list(info);
    guint len = gst_buffer_list_length(list);
    if (len == 0) {
        return GST_PAD_PROBE_OK;
    }

    // Add timestamp to the last one
    GstBuffer *buffer = gst_buffer_list_get(list, len - 1);

    GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;
    if (!gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
        INFO("Sender Failed to map RTP buffer");
        return GST_PAD_PROBE_OK;
    }

    guint64 timestamp = *(ctx->timestamp);
    // Fill timestamp bytes (8 bytes)
    guint8 data[11] = {0};
    for (size_t i = 0; i < sizeof(timestamp); ++i) {
        data[10 - i] = static_cast<guint8>((timestamp >> (i * 8)) & 0xFF);
    }

    // Add extension header to RTP packet
    gst_rtp_buffer_add_extension_onebyte_header(&rtp_buffer, 1, data, sizeof(data));
    gst_rtp_buffer_unmap(&rtp_buffer);

    return GST_PAD_PROBE_OK;
}