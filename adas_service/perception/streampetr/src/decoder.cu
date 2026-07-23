#include "decoder.cuh"
#include <algorithm>

#define thrdPerBlk 256

using Mat4x4f = const std::array<float, 16>;

__global__ void PostProcess(
    const float* __restrict__ all_cls_scores,
    const float* __restrict__ all_bbox_preds,
    const int num_proposal,
    const int num_cls,
    const float threshold,
    const std::array<float, 6> post_center_range,
    const Mat4x4f lidar2ego,
    int* __restrict__ count,
    Decoder::Detection_GPU* __restrict__ proposals
)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < num_proposal) {
        const float *cls_ptr = all_cls_scores + idx * num_cls;
        const float *box_ptr = all_bbox_preds + idx * num_cls;

        /* ---- Argmax & max of classes ---- */
        int cls_id = 0;
        float conf = cls_ptr[0];
        for (int i = 1; i < num_cls; i++) {
            if (conf < cls_ptr[i]) {
                conf = cls_ptr[i];
                cls_id = i;
            }
        }

        /* ---- Filter ---- */
        // Confidence threshold filter
        if (conf < threshold) return;

        // Range filter
        bool range_filter_enabled = false;
        #pragma unroll
        for (const float& val: post_center_range) {
            if (val != 0.0f) {
                range_filter_enabled = true;
                break;
            }
        }

        const float &cx = box_ptr[0], &cy = box_ptr[1], &cz = box_ptr[2];
        if (range_filter_enabled) {
            // check if x, y, z is within range
            const float &min_x = post_center_range[0], &min_y = post_center_range[1], &min_z = post_center_range[2];
            const float &max_x = post_center_range[3], &max_y = post_center_range[4], &max_z = post_center_range[5];

            if ((cx < min_x || max_x < cx) || (cy < min_y && max_y < cy) || (cz < min_z && max_z < cz)) {
                return;
            }
        }

        /* ---- Add proposal ---- */
        // Get write slot
        int pos = atomicAdd(count, 1);
        Decoder::Detection &proposal = proposals[pos].data;

        // Transform Position
        float x = lidar2ego[0] * cx + lidar2ego[1] * cy + lidar2ego[2] * cz + lidar2ego[3];
        float y = lidar2ego[4] * cx + lidar2ego[5] * cy + lidar2ego[6] * cz + lidar2ego[7];
        float z = lidar2ego[8] * cx + lidar2ego[9] * cy + lidar2ego[10] * cz + lidar2ego[11];

        // Transform Orientation
        const float &rot_sine = box_ptr[6], &rot_cosine = box_ptr[7];
        float new_cos = lidar2ego[0] * rot_cosine + lidar2ego[1] * rot_sine; // + transform[2] * 0
        float new_sin = lidar2ego[4] * rot_cosine + lidar2ego[5] * rot_sine; // + transform[6] * 0

        float yaw = atan2(new_sin, new_cos);

        // Box size
        float w = __expf(box_ptr[3]), l = __expf(box_ptr[4]), h = __expf(box_ptr[5]);

        // Velocity
        const float &vx = box_ptr[8], &vy = box_ptr[9];

        // Write data
        proposal.cls_id = cls_id;
        proposal.conf = conf;
        proposal.cx = x;
        proposal.cy = y;
        proposal.cz = z;
        proposal.w = w;
        proposal.l = l;
        proposal.h = h;
        proposal.yaw = yaw;
        proposal.vx = vx;
        proposal.vy = vy;
    }
}

Decoder::Decoder(
    cudaStream_t stream,
    int num_proposal,
    int num_cls,
    int topk,
    float threshold,
    std::array<float, 6> post_center_range,
    float *lidar2ego
)
{
    this->ext_stream = stream;
    this->num_proposal = num_proposal;
    this->num_cls = num_cls;
    this->topk = topk;
    this->threshold = threshold;
    this->post_center_range = post_center_range;

    std::copy(lidar2ego, lidar2ego + 16, this->lidar2ego.data());

    cudaMalloc(&count_dev, sizeof(int) * 1);
    cudaMalloc(&proposals_dev, sizeof(Detection_GPU) * this->num_proposal);

    cudaHostAlloc(&count_host, sizeof(int) * 1, cudaHostAllocDefault);
    cudaHostAlloc(&proposals_host, sizeof(Detection_GPU) * this->num_proposal, cudaHostAllocDefault);
}

Decoder::~Decoder()
{
    cudaFree(count_dev);
    cudaFree(proposals_dev);

    cudaFreeHost(count_host);
    cudaFreeHost(proposals_host);
}


/**
 * Perform post process and copy detection to CPU.
 * Operations:
 * - Argmax & max for class ID and confidence
 * - Confidence Threshold filter
 * - Denormalize bbox
 * - Range filter
 * - Transform to ego space
 * - Topk filter
 * 
 * NOTE: This call will block to wait for GPU's data.
 * 
 * @return Vector of Detections
 */
std::vector<Decoder::Detection> Decoder::decode()
{
    cudaMemsetAsync(this->count_dev, 0, sizeof(int), this->ext_stream);
    PostProcess<<<(this->num_proposal + thrdPerBlk) / thrdPerBlk, thrdPerBlk, 0, this->ext_stream>>>(
        this->all_cls_scores,
        this->all_bbox_preds,
        this->num_proposal,
        this->num_cls,
        this->threshold,
        this->post_center_range,
        this->lidar2ego,
        this->count_dev,
        this->proposals_dev
    );
    cudaMemcpyAsync(
        count_host,
        count_dev,
        sizeof(int) * 1,
        cudaMemcpyDeviceToHost,
        this->ext_stream
    );
    cudaMemcpyAsync(
        proposals_host,
        proposals_dev,
        sizeof(Detection_GPU) * this->num_proposal,
        cudaMemcpyDeviceToHost,
        this->ext_stream
    );

    cudaStreamSynchronize(this->ext_stream);

    int num_detect = this->topk;
    if (this->topk < *this->count_host) {
        // Got weird issue causing program to crash. Could not find the cause.
        // You can try this instead of stable_sort to improve performance a bit.
        // std::nth_element(
        //     proposals_host,                      // Start
        //     proposals_host + (this->topk - 1),   // The "Pivot" point (k-th element)
        //     proposals_host + *this->count_host,  // End of data
        //     [](const Detection_GPU& a, const Detection_GPU& b) {
        //         if (std::isnan(a.data.conf)) return false;
        //         if (std::isnan(b.data.conf)) return true;
        //         if (std::isinf(a.data.conf)) return true;
        //         if (std::isinf(b.data.conf)) return false;
        //         return a.data.conf > b.data.conf; // Descending
        //     }
        // );
        std::stable_sort(
            proposals_host,
            proposals_host + *this->count_host,
            [](const Detection_GPU& a, const Detection_GPU& b) {
                if (std::isnan(a.data.conf)) return false;
                if (std::isnan(b.data.conf)) return true;
                if (std::isinf(a.data.conf)) return true;
                if (std::isinf(b.data.conf)) return false;
                return a.data.conf > b.data.conf; // Descending
            }
        );
    }
    else {
        num_detect = *this->count_host;
    }

    std::vector<Detection> result(num_detect);
    for (int i = 0; i < num_detect; i++) {
        result[i] = proposals_host[i].data;
    }

    return result;
}