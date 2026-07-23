#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <iostream>
#include <memory>
#include <string>
#include <cstdlib>

#ifndef DISABLE_OBJECT_DETECTOR
#include "utils.h"
#include "cuda_utils.h"
#include "logging.h"
#include "preprocess.h"
#include "postprocess.h"
#include "model.h"
#endif

#include "traffic_object.h"

using namespace nvinfer1;

#define ENGINE_FILE_PATH ((getenv("PC_BUILD") != nullptr) ? "/nfs/share/adas_data/weights/yolov5s_seg_on_pc.engine" : "/lib/banvien/adas_data/obj_dect_model/weights/yolov5s_seg_on_board.engine")
#define LATERAL_NOI_OUTSIDE 10
#define LATERAL_NOI_INSIDE  180
enum class OUT_IMAGE_OBJECT_DETECT
{
    NONE,
    BBOX,
    BBOX_ID,
    BBOX_ID_DISTANCE
};

static Logger gLogger;
const static int kOutputSize1 = OD_MAX_NUM_OUTPUT_BBOX * sizeof(Detection) / sizeof(float) + 1;
const static int kOutputSize2 = 32 * (OD_INPUT_H / 4) * (OD_INPUT_W / 4);

class ObjectDetector {
private:
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;
    cudaStream_t stream;
    float* gpu_buffers[3];
    float* cpu_output_buffer1 = nullptr;
    float* cpu_output_buffer2 = nullptr;

    void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context);
    void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer1, float** gpu_output_buffer2, float** cpu_output_buffer1, float** cpu_output_buffer2);
    void infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output1, float* output2, int batchsize);

public:
    ObjectDetector();
    ~ObjectDetector();

    std::string engine_name = ENGINE_FILE_PATH;

    std::vector<TrafficObject> detect(cv::Mat img);
};

#endif  // OBJECT_DETECTOR_H
