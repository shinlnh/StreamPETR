// ROS2 header
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

// GStreamer header
#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/rtp/gstrtpbuffer.h>

// System header
#include <iostream>
#include <fstream>
#include <thread>
#include <condition_variable>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <limits.h>
#include <unistd.h>
#include <libgen.h>

// VPI library
#include <vpi/VPI.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Remap.h>

// Macro for VPI status checking
#define CHECK_STATUS(STMT) \
    do { \
        VPIStatus status = (STMT); \
        if (status != VPI_SUCCESS) { \
            RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"),"VPI error at %s:%d: %s", __FILE__, __LINE__, vpiStatusGetName(status)); \
            throw std::runtime_error("VPI error"); \
        } \
    } while (0)


/**
 * @brief Get absolute path to ADAS SDK in run-time to compatible with separated build and deploy
 */
std::string getAdasSdkPath()
{
    /**
     *  Executable path should be like: 
     *  <workspace>/adas_sdk/ros_bridge_support_ws/install/adas_bridge/lib/adas_bridge/<executable name>
     */ 
    char exe_path[PATH_MAX] = {};
    char temp_adas_sdk_path[PATH_MAX] = {};
    char adas_sdk_path[PATH_MAX] = {};
    char postfix_path[] = "/../../../../..";
    
    // Get executable path
    ssize_t exe_path_len = readlink("/proc/self/exe", exe_path, PATH_MAX - 1);
    if (exe_path_len != -1) {
        exe_path[exe_path_len] = '\0';   // double check here to avoid memory overflow
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rgb_debug"), "Cannot get executable path");
        return "";
    }
    
    // Extract path
    char *last_slash = strrchr(exe_path, '/');
    if (!last_slash) {
        RCLCPP_ERROR(rclcpp::get_logger("rgb_debug"), "Cannot get path to executable directory");
        return "";
    }
    
    // Copy path and postfix to temp_adas_sdk_path
    size_t prefix_len = last_slash - exe_path;
    size_t postfix_len = strlen(postfix_path);
    size_t final_len = prefix_len + postfix_len;

    memcpy(temp_adas_sdk_path, exe_path, prefix_len);
    memcpy(temp_adas_sdk_path + prefix_len, postfix_path, postfix_len);
    temp_adas_sdk_path[final_len] = '\0';  

    if (realpath(temp_adas_sdk_path, adas_sdk_path) == nullptr) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rgb_debug"), "Cannot get ADAS SDK path");
    }
    RCLCPP_INFO(rclcpp::get_logger("rgb_debug"), "ADAS SDK path: %s", adas_sdk_path);
    return std::string(adas_sdk_path);
}

// Main video streaming class
class Sender {
private:
    GstElement *pipeline, *appsrc, *payloader;
    GstPad *src_pad;
    GstBus *bus;
    std::mutex frameMutex;
    cv::Mat frame;
    std::string ip;
    int width, height;
    rclcpp::Time current_timestamp;

    // VPI resources
    VPIStream stream;
    VPIPayload remap;
    VPIImage tmpIn, tmpOut, vimg;
    VPIWarpMap warpMap = {};

public:
    Sender(const std::string &ip, int width, int height, int port)
        : ip(ip), width(width), height(height), 
          stream(nullptr), remap(nullptr), tmpIn(nullptr), 
          tmpOut(nullptr), vimg(nullptr) {
        gst_init(NULL, NULL);

        // Initialize VPI resources
        initializeVPI();

        // Get port for GStreamer
        std::string dynamic_port = std::to_string(port);

        // Construct GStreamer pipeline
        std::string pipeline_desc = 
            "appsrc name=source ! video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency bitrate=5000 speed-preset=superfast ! "
            "rtph264pay name=payloader ! udpsink host=" + ip + " port=" + dynamic_port;

        // Initialize GStreamer pipeline
        pipeline = gst_parse_launch(pipeline_desc.c_str(), NULL);
        if (!pipeline) {
            RCLCPP_ERROR(rclcpp::get_logger("rgb_debug"), "Failed to create pipeline.");
            cleanupVPI();
            return;
        }

        // Get and configure appsrc element
        appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");
        if (!appsrc) {
            RCLCPP_ERROR(rclcpp::get_logger("rgb_debug"), "Failed to get appsrc element.");
            cleanupVPI();
            return;
        }

        // Set video format capabilities
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "I420",
            "width", G_TYPE_INT, width,
            "height", G_TYPE_INT, height,
            "framerate", GST_TYPE_FRACTION, 30, 1,
            NULL);
        g_object_set(appsrc, "caps", caps, NULL);
        gst_caps_unref(caps);

        // Configure appsrc properties
        g_object_set(appsrc, "format", GST_FORMAT_TIME, NULL);
        g_object_set(appsrc, "stream-type", 0, NULL);

        // Setup pipeline bus for monitoring
        bus = gst_element_get_bus(pipeline);
        gst_bus_add_watch(bus, [](GstBus *bus, GstMessage *msg, gpointer user_data) -> gboolean {
            (void)bus;    // avoid warning
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_EOS:
                case GST_MESSAGE_ERROR:
                    g_main_loop_quit((GMainLoop *)user_data);
                    break;
                default:
                    break;
            }
            return TRUE;
        }, NULL);

        // Setup RTP payloader and probe
        payloader = gst_bin_get_by_name(GST_BIN(pipeline), "payloader");
        src_pad = gst_element_get_static_pad(payloader, "src");

        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, pad_probe_callback, this, NULL);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    // Destructor for cleanup
    ~Sender() {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(bus);
        gst_object_unref(pipeline);
        cleanupVPI();
    }
    
    // Read camera calibration parameters from file
    void updateCameraClibParam(float& fx, float& fy, float& cx, float& cy, VPIPolynomialLensDistortionModel& distModel) 
    {   
        std::string adas_root_path = getAdasSdkPath(); 
        // Path to the camera calibration parameters file 
        std::string file_path = adas_root_path + "/common/config/sensor/camera_calibration_parameters.txt";

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "Failed to open calibration file: %s", file_path.c_str());
        }

        std::string line;
        // Skip header lines
        while (std::getline(file, line)) {
            if (line.find("Camera Matrix:") != std::string::npos) {
                break;
            }
        }

        // Read camera matrix (3 lines, use first 2)
        std::vector<float> matrix_values;
        for (int i = 0; i < 3; ++i) {
            if (!std::getline(file, line)) {
                RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "Incomplete camera matrix in calibration file");
                file.close();
                throw std::runtime_error("Incomplete camera matrix");
            }
            std::istringstream iss(line);
            float val;
            while (iss >> val) {
                matrix_values.push_back(val);
            }
        }

        if (matrix_values.size() < 9) {
            RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "Invalid camera matrix format: expected 9 values, got %zu", matrix_values.size());
            file.close();
            throw std::runtime_error("Invalid camera matrix format");
        }

        // Assign intrinsics
        fx = matrix_values[0]; // fx
        cx = matrix_values[2]; // cx
        fy = matrix_values[4]; // fy
        cy = matrix_values[5]; // cy

        // Skip to distortion coefficients
        while (std::getline(file, line)) {
            if (line.find("Distortion Coefficients:") != std::string::npos) {
                break;
            }
        }

        // Read distortion coefficients
        if (!std::getline(file, line)) {
            RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "Missing distortion coefficients in calibration file");
            file.close();
            throw std::runtime_error("Missing distortion coefficients");
        }

        std::istringstream iss(line);
        std::vector<float> dist_values;
        float val;
        while (iss >> val) {
            dist_values.push_back(val);
        }

        if (dist_values.size() != 5) {
            RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "Invalid distortion coefficients: expected 5 values, got %zu", dist_values.size());
            file.close();
            throw std::runtime_error("Invalid distortion coefficients");
        }

        // Assign distortion coefficients
        distModel.k1 = dist_values[0];
        distModel.k2 = dist_values[1];
        distModel.p1 = dist_values[2];
        distModel.p2 = dist_values[3];
        distModel.k3 = dist_values[4];
        distModel.k4 = 0.0f;
        distModel.k5 = 0.0f;
        distModel.k6 = 0.0f;

        file.close();
    }

    // Initialize VPI resources
    void initializeVPI() {
        // Camera intrinsics (will be updated from file)
        float fx = 473.549975f;
        float fy = 474.542340f;
        float cx = 640.278717f;
        float cy = 480.537785f;

        // Polynomial distortion coefficients (will be updated from file)
        VPIPolynomialLensDistortionModel distModel = {};
        distModel.k1 = -0.015329;
        distModel.k2 = 0.040248;
        distModel.p1 = -0.000010;
        distModel.p2 = 0.000432;
        distModel.k3 = -0.015245;
        distModel.k4 = 0.0f;
        distModel.k5 = 0.0f;
        distModel.k6 = 0.0f;

        // Load calibration parameters
        try {
            updateCameraClibParam(fx, fy, cx, cy, distModel);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "Failed to load calibration parameters: %s", e.what());
        }
        
        // Create stream
        CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA, &stream));

        // Allocate warp map
        warpMap.grid.numHorizRegions = 1;
        warpMap.grid.numVertRegions = 1;
        warpMap.grid.regionWidth[0] = width;
        warpMap.grid.regionHeight[0] = height;
        warpMap.grid.horizInterval[0] = 1;
        warpMap.grid.vertInterval[0] = 1;
        CHECK_STATUS(vpiWarpMapAllocData(&warpMap));

        // Camera intrinsics
        VPICameraIntrinsic K = {
            {fx, 0.0f, cx},
            {0.0f, fy, cy}
        };

        // Camera extrinsics (identity)
        VPICameraExtrinsic X = {};
        X[0][0] = X[1][1] = X[2][2] = 1.0f;

        // Generate warp map
        CHECK_STATUS(vpiWarpMapGenerateFromPolynomialLensDistortionModel(K, X, K, &distModel, &warpMap));

        // Create remap payload
        CHECK_STATUS(vpiCreateRemap(VPI_BACKEND_CUDA, &warpMap, &remap));

        // Free warp map
        vpiWarpMapFreeData(&warpMap);

        // Create temporary images
        CHECK_STATUS(vpiImageCreate(width, height, VPI_IMAGE_FORMAT_NV12_ER, 0, &tmpIn));
        CHECK_STATUS(vpiImageCreate(width, height, VPI_IMAGE_FORMAT_NV12_ER, 0, &tmpOut));
    }

    // Cleanup VPI resources
    void cleanupVPI() {
        if (tmpOut) vpiImageDestroy(tmpOut);
        if (tmpIn) vpiImageDestroy(tmpIn);
        if (vimg) vpiImageDestroy(vimg);
        if (remap) vpiPayloadDestroy(remap);
        if (stream) vpiStreamDestroy(stream);
    }

    // Update frame data and timestamp
    void updateFrame(const cv::Mat &newFrame) {
        std::lock_guard<std::mutex> lock(frameMutex);
        this->frame = newFrame.clone();
    }
    // Set ROS timestamp
    void setTimestamp(const rclcpp::Time &ts) {
        current_timestamp = ts;
    }

    // Process image with VPI undistortion
    void processImage(const cv::Mat &inputImage, cv::Mat &outputImage) {
        // Wrap input image
        if (!vimg) {
            CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(inputImage, 0, &vimg));
        } else {
            CHECK_STATUS(vpiImageSetWrappedOpenCVMat(vimg, inputImage));
        }

        // Convert BGR to NV12
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, vimg, tmpIn, NULL));

        // Undistort
        CHECK_STATUS(vpiSubmitRemap(stream, VPI_BACKEND_CUDA, remap, tmpIn, tmpOut, VPI_INTERP_CATMULL_ROM, VPI_BORDER_ZERO, 0));

        // Convert NV12 to BGR
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, tmpOut, vimg, NULL));

        // Sync stream
        CHECK_STATUS(vpiStreamSync(stream));

        // Output is already in inputImage (wrapped by vimg)
        outputImage = inputImage.clone(); // Clone to ensure independent memory
    }

    void sendFrameOnce() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (frame.empty()) {
            return;
        }

        GstBuffer *buffer = gst_buffer_new_allocate(NULL, frame.total() * frame.elemSize(), NULL);
        if (!buffer) {
            RCLCPP_INFO(rclcpp::get_logger("rgb_debug"), "Failed to create GstBuffer.");
            return;
        }

        gst_buffer_fill(buffer, 0, frame.data, frame.total() * frame.elemSize());
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK) {
            RCLCPP_INFO(rclcpp::get_logger("rgb_debug"), "Failed to push buffer: %d", ret);
        }

        frame.release();
    }

    // Callback for RTP packet processing
    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        (void)pad;    // avoid warning
        Sender *sender = static_cast<Sender *>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        GstRTPBuffer rtp_buffer = GST_RTP_BUFFER_INIT;

        if (!gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp_buffer)) {
            RCLCPP_INFO(rclcpp::get_logger("rgb_debug"), "Failed to map RTP buffer");
            return GST_PAD_PROBE_OK;
        }

        guint64 timestamp_value = sender->current_timestamp.nanoseconds();
         // Fill timestamp bytes (8 bytes)
        guint8 data[11] = {0};
        for (size_t i = 0; i < 8; ++i) {
            data[10 - i] = static_cast<guint8>((timestamp_value >> (i * 8)) & 0xFF);
        }

        // Add extension header to RTP packet
        gst_rtp_buffer_add_extension_onebyte_header(&rtp_buffer, 1, data, sizeof(data));
        gst_rtp_buffer_unmap(&rtp_buffer);
        return GST_PAD_PROBE_OK;
    }
};

class RgbFrontVpiNode : public rclcpp::Node {
private:
    std::shared_ptr<Sender> sender_;
    std::thread sender_thread_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;

public:
    RgbFrontVpiNode() : Node("adas_bridge_rgb_front_node") {
        // Declare parameter
        this->declare_parameter("board_ip", "127.0.0.1");
        this->declare_parameter("vehicle_name", "hero0");
        this->declare_parameter("image_port", 3445);

        // Get parameter value
        std::string ip = this->get_parameter("board_ip").as_string();
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();

        RCLCPP_DEBUG(this->get_logger(), "Target stream IP: %s", ip.c_str());
        RCLCPP_DEBUG(this->get_logger(), "The name of ego vehicle: %s", vehicle_name.c_str());

        // Create subscriber
        std::string sub_topic = "/carla/" + vehicle_name + "/rgb_front/image";
        sub_img_ = create_subscription<sensor_msgs::msg::Image>(
            sub_topic, 1,
            std::bind(&RgbFrontVpiNode::imageCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "RGB front node intialize successful");
    }

    ~RgbFrontVpiNode() override {
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received image [W:%d][H:%d]", msg->width, msg->height);

        static double timestamp_bench = rclcpp::Time(msg->header.stamp).seconds();
        RCLCPP_DEBUG(
            this->get_logger(), 
            "Duration call imageCallback: %.4lf s", 
            rclcpp::Time(msg->header.stamp).seconds() - timestamp_bench
        );
        timestamp_bench = rclcpp::Time(msg->header.stamp).seconds();

        if (sender_ == nullptr) {
            // Initialize sender with size of image
            std::string ip = this->get_parameter("board_ip").as_string();
            int port = this->get_parameter("image_port").as_int();

            sender_ = std::make_shared<Sender>(ip, msg->width, msg->height, port);
        }

        try {
            rclcpp::Time ros_time = msg->header.stamp;
            sender_->setTimestamp(ros_time);

            cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Undistort the image using VPI
            cv::Mat undistorted_image;
            sender_->processImage(cv_image, undistorted_image);
            if (undistorted_image.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("vpi_logger"), "VPI undistortion failed, skipping frame");
                return;
            }
            
            #if 0
            // Put timestamp on image
            std::string timeStr = "Send: " + std::to_string(ros_time.nanoseconds());
            cv::putText(undistorted_image, timeStr, cv::Point(10, 70),
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250), 1, cv::LINE_AA);
            #endif

            // Convert to YUV420 format
            cv::Mat yuv_image;
            cv::cvtColor(undistorted_image, yuv_image, cv::COLOR_BGR2YUV_I420);
            sender_->updateFrame(yuv_image);
            sender_->sendFrameOnce();
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
    });

    auto rgb_node = std::make_shared<RgbFrontVpiNode>();
    rclcpp::spin(rgb_node);
    rclcpp::shutdown();
    return 0;
}