#include <cstdlib>
#include <chrono>

#include "engine.h"

// #define INFERENCE_DEBUG

TRTEngine::TRTEngine(/* args */)
{
    // Load the TensorRT engine
    nvinfer1::ICudaEngine* engine = loadEngine(TRT_WEIGHT_PATH);
    if (!engine) {
        ERROR("Failed to load TensorRT engine");
        throw std::runtime_error("Unable to load engine");
    }
    cudaStreamCreate(&stream);
     // Create an execution context
    context = engine->createExecutionContext();
    if (!context) {
        ERROR("Failed to create execution context");
        throw std::runtime_error("Failed to create execution context");
    }
    INFO("Loaded engine from %s", TRT_WEIGHT_PATH);
}

TRTEngine::~TRTEngine()
{
    cudaStreamDestroy(stream);
    delete engine;
    delete context;
    // Free GPU memory
}

// Function to load TensorRT engine
nvinfer1::ICudaEngine* TRTEngine::loadEngine(const std::string& engineFile)
{
    std::ifstream file(engineFile, std::ios::binary);
    if (!file) {
        ERROR("Error opening engine file: %s", engineFile.c_str());
        return nullptr;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::unique_ptr<char[]> engineData(new char[size]);
    file.read(engineData.get(), size);
    file.close();

    std::unique_ptr<nvinfer1::IRuntime, Destroy<nvinfer1::IRuntime>> runtime{nvinfer1::createInferRuntime(this->logger)};
    return runtime->deserializeCudaEngine(engineData.get(), size, nullptr);
}

cv::Mat TRTEngine::runInference(const cv::Mat& input_frame)
{
#ifdef INFERENCE_DEBUG
    auto start_time = std::chrono::high_resolution_clock::now();
#endif
    // Preprocess the input frame
    cv::Mat input_data;
    input_data = input_frame(cv::Rect(CROP_X, CROP_Y, CROP_W, CROP_H)); // Crop
    cv::resize(input_data, input_data, cv::Size(INPUT_W, INPUT_H)); // Resize
    input_data.convertTo(input_data, CV_32F, 1.0 / 255.0); // Convert data type & scale pixel value

    // Convert memory layout from HWC to CHW
    std::vector<cv::Mat> channels(INPUT_C); // Vector to hold channels
    cv::split(input_data, channels); // Splits into separate channel matrices
    float* chw_data = new float[INPUT_C * INPUT_H * INPUT_W]; // Allocate memory
    for (int i = 0; i < INPUT_C; ++i) {
        memcpy(chw_data + i * INPUT_H * INPUT_W,
               channels[i].data, INPUT_H * INPUT_W * sizeof(float)); // Copy
    }

    // Create matrix for output from GPU
    cv::Mat output_data(N_ANCHOR_LINE, N_ANCHOR_PROP, CV_32FC1);

    // Allocate GPU memory for input and output
    void* device_input;
    void* device_output;
    cudaMalloc(&device_input, INPUT_C * INPUT_H * INPUT_W * sizeof(float));
    cudaMalloc(&device_output, output_data.total() * output_data.elemSize());

    // Create binding pointer to hold the input and output data
    void* bindings[] = {device_input, device_output};

    // Transfer input data to GPU
    cudaMemcpyAsync(device_input, chw_data,
                    INPUT_C * INPUT_H * INPUT_W * sizeof(float),
                    cudaMemcpyHostToDevice, stream);

    // Run inference
    context->enqueueV2(bindings, stream, nullptr);

    // Transfer output data from GPU to CPU
    cudaMemcpyAsync(output_data.data, device_output,
                    output_data.total() * output_data.elemSize(),
                    cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    delete(chw_data);
    cudaFree(device_input);
    cudaFree(device_output);

#ifdef INFERENCE_DEBUG
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration = end_time - start_time;
    INFO("Inference took: %ld  microseconds", duration.count());
#endif    

    return output_data;
}


// Implementation of runInferenceWithThread function
void TRTEngine::runInferenceWithThread(const cv::Mat& input_frame, cv::Mat& output_frame) {
    output_frame = runInference(input_frame);
}