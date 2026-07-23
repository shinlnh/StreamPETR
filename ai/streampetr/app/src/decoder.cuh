#ifndef DECODER_CUH
#define DECODER_CUH

#include <cuda_runtime_api.h>
#include <vector>
#include <array>

class Decoder
{
public:
    struct Detection
    {
        int cls_id;
        float conf;
        float cx, cy, cz;
        float w, l, h;
        float yaw;
        float vx, vy;
    }; // struct Detection
    
    struct Detection_GPU
    {
        Detection data;

        // Padding to 64 bytes
        char _padd[64 - sizeof(Detection)];
    }; // struct Detection_GPU

private:
    cudaStream_t ext_stream;  // the stream of StreamPETR

    // Postprocessing configs
    int num_proposal = 428;
    int num_cls = 10;
    int topk = -1;
    float threshold = -1.0f;
    std::array<float, 6> post_center_range = {0};
    std::array<float, 16> lidar2ego;

    // Buffer for postprocessing on GPU
    int *count_dev;
    Detection_GPU *proposals_dev;
    int *count_host;
    Detection_GPU *proposals_host;

public:
    // output proposal buffer of detector
    float *all_cls_scores;
    float *all_bbox_preds;

public:
    Decoder(
        cudaStream_t stream,
        int num_proposal,
        int num_cls,
        int topk,
        float threshold,
        std::array<float, 6> post_center_range,
        float *lidar2ego
    );
    ~Decoder();

    std::vector<Detection> decode();
}; // class Decoder

#endif // DECODER_CUH