#include "object_detector.h"
#include <unordered_set>
#include <algorithm>
#include <boost/chrono.hpp>
#include "common.h"
// We use this set to filter out fault detections
std::unordered_set<int> valid_class_ids = {
    0, 1, 2, 3, 5, 7, 9, 11 
};

ObjectDetector::ObjectDetector()
{
    // Read engine file of yolov5s model
    std::ifstream file(engine_name, std::ios::binary);

    if (!file.good())
    {
        ERROR("Engine file not found: %s. Please generate using gen_yolov5_engine tool.", engine_name.c_str());
        throw std::runtime_error("Engine file not found: " + engine_name);
    }

    // Deserialize the engine from file
    deserialize_engine(engine_name, &runtime, &engine, &context);
    CUDA_CHECK(cudaStreamCreate(&stream));

    // Init CUDA preprocessing
    cuda_preprocess_init(OD_MAX_INPUT_IMAGE_SIZE);

    // Prepare cpu and gpu buffers
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &gpu_buffers[2], &cpu_output_buffer1, &cpu_output_buffer2);
}

ObjectDetector::~ObjectDetector(void)
{
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    CUDA_CHECK(cudaFree(gpu_buffers[2]));
    delete[] cpu_output_buffer1;
    delete[] cpu_output_buffer2;
    cuda_preprocess_destroy();

    // Destroy the engine
    // context->destroy();
    delete context;
    //engine->destroy();
    delete engine;
    // runtime->destroy(); //Deprecated, use delete instead
    delete runtime;
} 

void ObjectDetector::deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
    ERROR("Read %s error!", engine_name.c_str());
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char* serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}

void ObjectDetector::prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer1, float** gpu_output_buffer2, float** cpu_output_buffer1, float** cpu_output_buffer2) {
  assert(engine->getNbBindings() == 3);

  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(OD_INPUT_TENSOR_NAME);
  const int outputIndex1 = engine->getBindingIndex(OD_OUTPUT_TENSOR_NAME);
  const int outputIndex2 = engine->getBindingIndex("proto");

  assert(inputIndex == 0);
  assert(outputIndex1 == 1);
  assert(outputIndex2 == 2);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, OD_BATCH_SIZE * 3 * OD_INPUT_H * OD_INPUT_W * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer1, OD_BATCH_SIZE * kOutputSize1 * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer2, OD_BATCH_SIZE * kOutputSize2 * sizeof(float)));

  // Alloc CPU buffers
  *cpu_output_buffer1 = new float[OD_BATCH_SIZE * kOutputSize1];
  *cpu_output_buffer2 = new float[OD_BATCH_SIZE * kOutputSize2];
}

void ObjectDetector::infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output1, float* output2, int batchsize) {
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output1, gpu_buffers[1], batchsize * kOutputSize1 * sizeof(float), cudaMemcpyDeviceToHost, stream));
  CUDA_CHECK(cudaMemcpyAsync(output2, gpu_buffers[2], batchsize * kOutputSize2 * sizeof(float), cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

std::vector<TrafficObject> ObjectDetector::detect(cv::Mat img)
{
    // Crop the received image that captures the area of interest.
    // Excluding the car hood and the upper sky, which helps prevent undesired detection.
    static constexpr uint32_t crop_x = 0, crop_y = 150, crop_w = 800, crop_h = 500;
    static const cv::Rect cropRegion(crop_x, crop_y, crop_w, crop_h);

    // Check if the crop rectangle is within image bounds
    if ((crop_x + crop_w > img.cols) || (crop_y + crop_h > img.rows)) {
        ERROR("Crop region exceeds image bounds!");
        return {};
    }

    // Get a batch of images (clone to make contiguous memory)
    std::vector<cv::Mat> img_batch;
    img_batch.push_back(img(cropRegion).clone());

    // Preprocess
    boost::chrono::system_clock::time_point start;
    cuda_batch_preprocess(img_batch, gpu_buffers[0], OD_INPUT_W, OD_INPUT_H, stream);

    // Run inference and conculate processing time
    infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer1, cpu_output_buffer2, OD_BATCH_SIZE);

    // Non-max suppression for several bounding boxs
    std::vector<std::vector<Detection>> res_batch;
    batch_nms(res_batch, cpu_output_buffer1, img_batch.size(), kOutputSize1, OD_CONF_THRESH, OD_NMS_THRESH);

    // Add all detected information into object 
    std::vector<TrafficObject> traffic_objects;
    for (size_t i = 0; i < img_batch.size(); i++) {
        cv::Mat image = img_batch[i];
        auto& res = res_batch[i];
        traffic_objects.reserve(traffic_objects.size() + res.size());
        for (const auto &detection: res)
        {
            // Remove all TrafficObjects with classID not in valid_class_ids
            if (valid_class_ids.find(detection.class_id) == valid_class_ids.end()) continue;
            
            cv::Rect r = get_rect(image, detection.bbox);
            // Because of the wrong position estimation from the Bounding box, objects which locate at the edge of FOV carmera
            // will be ignore to eliminate the wrong noise object
            if ((r.x <= LATERAL_NOI_OUTSIDE && (r.width + r.x) <= LATERAL_NOI_INSIDE) ||
                (r.x >= (image.cols - LATERAL_NOI_INSIDE) && (r.width + r.x) >= (image.cols - LATERAL_NOI_OUTSIDE)))
                continue;

            TrafficObject obj;
            obj.bbox.x1 = r.x + crop_x;
            obj.bbox.y1 = r.y + crop_y;
            obj.bbox.x2 = r.width + r.x + crop_x;
            obj.bbox.y2 = r.height + r.y + crop_y;
            obj.classId = detection.class_id;
            obj.prob = detection.conf;

            cv::Mat mask_mat = cv::Mat::zeros(OD_INPUT_H / 4, OD_INPUT_W / 4, CV_32FC1);
            // The additional output buffer 2 is the mask
            const float* proto = &cpu_output_buffer2[i * kOutputSize2];
            int proto_size= kOutputSize2;
            r = get_downscale_rect(detection.bbox, 4);
            for (int x = r.x; x < r.x + r.width; x++) {
                for (int y = r.y; y < r.y + r.height; y++) {
                    float e = 0.0f;
                    for (int k = 0; k < 32; k++) {
                        // In here we travelling each of 32 160x160 coeff mask and iterate through each mask pixel
                        e += detection.mask[k] * proto[k * proto_size / 32 + y * mask_mat.cols + x];
                    }
                    e = 1.0f / (1.0f + expf(-e));
                    mask_mat.at<float>(y, x) = e;
                }
            }
            int x, y, w, h;
            float r_w = OD_INPUT_W / (image.cols * 1.0);
            float r_h = OD_INPUT_H / (image.rows * 1.0);
            if (r_h > r_w) {
                w = OD_INPUT_W;
                h = r_w * image.rows;
                x = 0;
                y = (OD_INPUT_H - h) / 2;
            } else {
                w = r_h * image.cols;
                h = OD_INPUT_H;
                x = (OD_INPUT_W - w) / 2;
                y = 0;
            }
            cv::Rect r_resize(x, y, w, h);
            cv::resize(mask_mat, mask_mat, cv::Size(OD_INPUT_W, OD_INPUT_H));
            cv::resize(mask_mat(r_resize), mask_mat, cv::Size(image.cols, image.rows));
            obj.mask = mask_mat;
            obj.distance_to_my_car = -1.0f;

            traffic_objects.push_back(obj);
        } 
    }

    return traffic_objects;
}
