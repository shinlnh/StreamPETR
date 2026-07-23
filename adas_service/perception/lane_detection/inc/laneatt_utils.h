#ifndef __LANEATT_UTILS_H__
#define __LANEATT_UTILS_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include "common.h"

// These are model's config, in future release, config will be read from file instead of hardcode.
#define TRT_WEIGHT_PATH ((getenv("PC_BUILD") != nullptr) ? "/nfs/share/adas_data/weights/laneatt_cls.engine" : "/lib/banvien/laneatt_cls.engine")
// LaneATT Anchors config
#define N_ANCHOR_LINE       1000
#define N_POINT_PER_LINE    72
#define N_ANCHOR_PROP       (2 + 3 + N_POINT_PER_LINE) // 2 for class, conf and 3 for x0, y0, Length
#define N_STRIP             (N_POINT_PER_LINE - 1)  

// LaneATT network input shape
#define INPUT_C             3
#define INPUT_H             360
#define INPUT_W             640

// Crop setting
#define CROP_X              0
#define CROP_Y              175
#define CROP_W              800
#define CROP_H              450

// Post processing setting
#define CONF_THRESH         0.3     // Filter proposals with conf bellow this
#define IOU_THRESH          50.     // If average point distance (pixel) of 2 proposals less than this then one is removed
#define TOPK                4       // The max number of lane markings to keep

#define N_PROP 77
#define DIVUP(m,n) (((m)+(n)-1) / (n))
int64_t const threadsPerBlock = sizeof(unsigned long long) * 8;
extern cudaStream_t NMS_Stream;

/**
 * @brief
 * Perform NMS on lane proposals from LaneATT model
 * Return a cv::Mat that store lanes after applying NMS, one lane per row
*/
cv::Mat nms_lane_cuda(cv::Mat &proposals, // Proposals (class, conf, origins, length, offsets)
                      cv::Mat &idx,       // Indexes of proposals sorted by confidence (descending)
                      float iou_thresh,   // IoU Threshold
                      int32_t top_k       // Max # detection
);

cv::Mat classFilter(cv::Mat &proposals, std::vector<float> filter = {0.});
cv::Mat confThresholding(cv::Mat &proposals, float confThresh);
cv::Mat LaneATTPostProcess(cv::Mat &proposals, float confThresh, float IoUThresh, int topK);
std::vector<std::vector<cv::Point2f>> LaneATTLanesToPoints(cv::Mat const &lanes);

#endif