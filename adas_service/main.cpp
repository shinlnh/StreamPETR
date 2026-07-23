#include "common.h"
#include "string.h"
#include <opencv2/opencv.hpp>
#include "adas_main.h"
#include "service_handler.h"
#include "version.h"
#include <X11/Xlib.h>
using namespace cv;

#include <iostream>

int main(int argc, char *argv[])
{
    XInitThreads();
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int sig_num) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
        exit(sig_num);
    });
    const std::string keys =
        "{help h usage ? |             | print this message   }"
        "{src_type       | camera      | source type 'camera', 'video', 'tcp', 'udp'}"
        "{src            | /dev/video0 | source device, video path, url}"
        "{debug_aeb      | false       | Enable/Disable debug AEB}"
        ;

    CommandLineParser parser(argc, argv, keys);
    parser.about("BV ADAS APP v1.0.0");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Display version information
    INFO("==============================================");
    INFO("BV ADAS Service %s", GIT_VERSION);
    INFO("Commit: %s", GIT_COMMIT_HASH);
    INFO("Branch: %s", GIT_BRANCH);
    INFO("Build: %s %s", BUILD_DATE, BUILD_TIME);
    INFO("==============================================");

    AdasMain master (parser.get<std::string>("src_type")
                , parser.get<std::string>("src")
                , parser.get<bool>("debug_aeb"));

    cv::startWindowThread();
    ServiceHandler service_handler(&master);
    while (service_handler.getHandlerAliveStatus() == true) 
    {
        // DEBUG("Service Alive");

        cv::Mat img = master.getDebugImage();

        if (!img.empty()) {
            cv::imshow("Debug View", img);
        }

        boost::this_thread::yield();
    }

    INFO("Service handler exited");
    return 0;

    rclcpp::shutdown();
}
