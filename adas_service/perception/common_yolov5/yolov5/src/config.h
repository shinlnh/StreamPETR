#pragma once

/* --------------------------------------------------------
 * These configs are related to tensorrt model, if these are changed,
 * please re-compile and re-serialize the tensorrt model.
 * --------------------------------------------------------*/

// For INT8, you need prepare the calibration dataset, please refer to
// https://github.com/wang-xinyu/tensorrtx/tree/master/yolov5#int8-quantization
#define USE_FP16  // set USE_INT8 or USE_FP16 (now for PC) or USE_FP32
// #define USE_INT8  // set USE_INT8 or USE_FP16 or USE_FP32

// These are used to define input/output tensor names,
// you can set them to whatever you want.
#define OD_INPUT_TENSOR_NAME  "data"
#define OD_OUTPUT_TENSOR_NAME "prob"

// Detection model and Segmentation model' number of classes
#define OD_NUM_CLASS 80

// Classfication model's number of classes
#define OD_CLS_OD_NUM_CLASS 1000

#define OD_BATCH_SIZE 1

// Yolo's input width and height must by divisible by 32
#define OD_INPUT_H 640
#define OD_INPUT_W 640

// Classfication model's input shape
#define OD_CLS_INPUT_H 224
#define OD_CLS_INPUT_W 224

// Maximum number of output bounding boxes from yololayer plugin.
// That is maximum number of output bounding boxes before NMS.
#define OD_MAX_NUM_OUTPUT_BBOX 1000

#define OD_NUM_ANCHOR 3

// The bboxes whose confidence is lower than OD_IGNORE_THRESH will be ignored in yololayer plugin.
#define OD_IGNORE_THRESH 0.1f

/* --------------------------------------------------------
 * These configs are NOT related to tensorrt model, if these are changed,
 * please re-compile, but no need to re-serialize the tensorrt model.
 * --------------------------------------------------------*/

// NMS overlapping thresh and final detection confidence thresh
#define OD_NMS_THRESH 0.5f
#define OD_CONF_THRESH 0.4f

#define OD_GPU_ID 0

// The maximum size of the input image
#define OD_MAX_INPUT_IMAGE_SIZE 800 * 800

