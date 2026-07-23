#ifndef __ENGINE_H__
#define __ENGINE_H__

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include "laneatt_utils.h"  // This is temporary
#include "common.h"

// #define LANE_SEGMENT_DEBUG

// Logger class for TensorRT
class LoggerTRT : public nvinfer1::ILogger {
public:
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override {
        // Log messages based on severity level
        if (severity != nvinfer1::ILogger::Severity::kINFO)
            INFO("msg: %s", msg);
    }
};

// Template struct for destroying TensorRT objects
template<typename T>
struct Destroy {
    void operator()(T* obj) const {
        if (obj)
            obj->destroy();
    }
};

class TRTEngine
{
private:
    /* data */
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    LoggerTRT logger;
    cudaStream_t stream;
public:
    TRTEngine(/* args */);
    ~TRTEngine();

    nvinfer1::ICudaEngine* loadEngine(const std::string& engineFile);
    cv::Mat runInference(const cv::Mat& input_frame);
    void runInferenceWithThread(const cv::Mat& input_frame, cv::Mat& output_frame);
};

#endif // __ENGINE_H__