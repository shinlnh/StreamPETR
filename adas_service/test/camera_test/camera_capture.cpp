#include <opencv2/opencv.hpp>
#include <iostream>
#include "common.h"

int main() {
    // Define the GStreamer pipeline for OpenCV
    std::string pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=960, height=540, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    INFO("OpenCV Build Information:\n%s", cv::getBuildInformation().c_str());

    if (!cap.isOpened()) {
        ERROR("Error: Cannot open camera!");
        return -1;
    }


    cv::Mat frame;
    while (true) {
        cap >> frame;  // Capture a frame
        if (frame.empty()) {
            ERROR("Error: Blank frame grabbed!");
            break;
        }

        cv::imshow("Camera Output", frame);  // Display the frame

        if (cv::waitKey(10) == 27) {  // Wait for 'Esc' key press for 10 ms
            break;
        }
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
