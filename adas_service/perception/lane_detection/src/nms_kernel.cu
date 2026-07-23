#include "laneatt_utils.h"

template <typename scalar_t>
__device__ inline bool devIoU(int32_t const n_points,
                              scalar_t const * const a,
                              scalar_t const * const b,
                              float const threshold)
{
  // Find y overlapping range
  const int start_a = (int) (a[2] * (n_points - 1) + 0.5); // 0.5 rounding trick
  const int start_b = (int) (b[2] * (n_points - 1) + 0.5);
  const int start = max(start_a, start_b);
  const int end_a = start_a + a[4] - 1 + 0.5 - ((a[4] - 1) < 0); //  - (x<0) trick to adjust for negative numbers (in case length is 0)
  const int end_b = start_b + b[4] - 1 + 0.5 - ((b[4] - 1) < 0);
  const int end = min(min(end_a, end_b), n_points - 1);
  
  if (end < start) return false; // Lines stay on differen y range (no overlap)

  // Calculate total distance between lines
  scalar_t dist = 0;
  for(unsigned char i = 5 + start; i <= 5 + end; ++i) {
    if (a[i] < b[i]) {
      dist += b[i] - a[i];
    } else {
      dist += a[i] - b[i];
    }
  }
  
  // Return true if these proposals are too close (total dist < total threshold)
  return dist < (threshold * (end - start + 1));
}

__global__ void nms_kernel(int32_t const n_proposals,
                           int32_t const n_prop,
                           float const nms_overlap_thresh,
                           float const *dev_boxes,
                           int32_t const *idx,
                           uint64 *dev_mask)
{
  int64_t const row_start = blockIdx.y;
  int64_t const col_start = blockIdx.x;

  if (row_start > col_start) return; // Stop blocks passing the diagonal line

  int const row_size = min(n_proposals - row_start * threadsPerBlock, threadsPerBlock);
  int const col_size = min(n_proposals - col_start * threadsPerBlock, threadsPerBlock);

  // Copy proposals (x-axis) for current block
  extern __shared__ float block_boxes[];
  if (threadIdx.x < col_size) {
    for (int i = 0; i <  n_prop; ++i) {
      block_boxes[threadIdx.x * n_prop + i] = dev_boxes[idx[(threadsPerBlock * col_start + threadIdx.x)] * n_prop + i];
    }
  }
  __syncthreads();

  // Calculate IoU between all anchors
  if (threadIdx.x < row_size) // stop threads on rows thats outside of block's matrix
  {
    // Get current thread's base box (y-axis)
    int const cur_box_idx = threadsPerBlock * row_start + threadIdx.x;
    float const *cur_box = dev_boxes + idx[cur_box_idx] * n_prop;

    int32_t i = 0;
    uint64_t t = 0;
    int32_t start = 0;

    // Adjust start position from the diagonal line
    if (row_start == col_start)
    {
      start = threadIdx.x + 1;
    }

    // Calculate IoU for current row
    for (i = start; i < col_size; i++) {
      if (devIoU(n_prop - 5, cur_box, block_boxes + i * n_prop, nms_overlap_thresh)) {
        t |= 1ULL << i;
      }
    }

    // Save into mask
    int const col_blocks = DIVUP(n_proposals, threadsPerBlock);
    dev_mask[cur_box_idx * col_blocks + col_start] = t;
  }
}


__global__ void nms_collect(int32_t const n_proposals,
                            int32_t const num_blocks,
                            int64_t const top_k,
                            int32_t const *idx,
                            uint64_t *mask,
                            int32_t *keep,
                            int32_t *num_to_keep)
{
  int64_t num_to_keep_ = 0;

  /******** NOTE: ********
   * I've removed the remv[] array, instead I use the mask array of the proposal
   *  with highest confidence since we will always keep it anyway.
   * Also allocating remv[] array require a #define max which is a hassle and
   *  waste memory anyway.
   * This change doesn't improve performance, but it keep the code simplier.
   */

  // Loop through each proposal
  for (int i = 0; i < n_proposals; i++)
  {
    // values for indexing
    int nblock = i / threadsPerBlock;
    int inblock = i % threadsPerBlock;

    if (!(mask[nblock] & (1ULL << inblock)))  // If not removed
    {
      keep[num_to_keep_] = idx[i];  // Save this proposal's index
      const uint64_t *p = mask + i * num_blocks;  // Ptr to IoU relation with others proposals
      for (int j = nblock; j < num_blocks; j++) {
        mask[j] |= p[j];  // Record proposals overlap with to remove
      }

      num_to_keep_++;

      if (num_to_keep_==top_k)
          break;
    }
  }

  *num_to_keep = min(top_k,num_to_keep_);
}

cv::Mat nms_lane_cuda(cv::Mat &proposals,
                      cv::Mat &idx,
                      float iou_thresh,
                      int32_t top_k)
{
  // Verify input
  int32_t const num_proposals = proposals.rows;
  int32_t const num_prop = proposals.cols;
  if(num_proposals <= 1) return proposals;  // Return input if there is less than 1 proposal
  if(iou_thresh <= 0. || top_k <= 0) return cv::Mat(); // If IoU or topk is zero or less, return empty
  if(num_proposals != (int32_t) idx.total()) { // Check if idx size and No. proposals match
    ERROR("Number of proposals and Sorted index does not match!");
    throw std::runtime_error("NMS - Input dimension error!");
  }

  // Guard: make data continuous
  if(proposals.isContinuous() == false) proposals = proposals.clone();
  if(idx.isContinuous() == false) idx = idx.clone();

  // Init cuda Stream (once)
  cudaError_t stream_check = cudaStreamQuery(NMS_Stream);
  if(stream_check != cudaSuccess && stream_check != cudaErrorNotReady) {
    if(cudaStreamCreate(&NMS_Stream) != cudaSuccess) {
      ERROR("NMS Stream can't be created! Exiting!");
      throw std::runtime_error("NMS Stream cant be created!");
    }
  }

  // Calculate No. blocks
  const int32_t num_blocks = DIVUP(num_proposals, threadsPerBlock);
  dim3 blocks(num_blocks, num_blocks);
  dim3 threads(threadsPerBlock);

  // Allocate buffers for kernel
  float *k_proposals;  // device pointer to proposals
  cudaMalloc(&k_proposals, proposals.total() * proposals.elemSize());
  
  int32_t *k_idx;  // device pointer to sorted indexes
  cudaMalloc(&k_idx, idx.total() * idx.elemSize());

  uint64_t *mask;  // shared pointer to mask
  cudaMalloc(&mask, num_proposals * num_blocks * sizeof(uint64_t));

  int32_t *keep;  // shared pointer to array of indexes to keep
  cudaMalloc(&keep, num_proposals * sizeof(int32_t));

  int32_t *num_to_keep = nullptr;  // shared pointer to number of indexes to keep
  cudaMalloc(&num_to_keep, 1 * sizeof(int32_t));


  // Upload data
  cudaMemcpyAsync(k_proposals, proposals.data,
                  proposals.total() * proposals.elemSize(),
                  cudaMemcpyHostToDevice, NMS_Stream);
  cudaMemcpyAsync(k_idx, idx.data,
                  idx.total() * idx.elemSize(),
                  cudaMemcpyHostToDevice, NMS_Stream);
  
  // Call NMS kernel to perform on GPU
  nms_kernel<<<blocks, threads, threadsPerBlock * N_PROP * sizeof(float), NMS_Stream>>>
    (num_proposals, num_prop, iou_thresh,
     k_proposals, k_idx, mask);

  nms_collect<<<1, 1, 0, NMS_Stream>>>(num_proposals, num_blocks, top_k,
                                       k_idx, mask,
                                       keep, num_to_keep);

  // Download data
  int32_t *keep_cpu = (int32_t*) malloc(num_proposals * sizeof(int32_t));
  int32_t *num_to_keep_cpu = (int32_t*) malloc(sizeof(int32_t));
  cudaMemcpyAsync(keep_cpu, keep, num_proposals * sizeof(int32_t), cudaMemcpyDeviceToHost, NMS_Stream);
  cudaMemcpyAsync(num_to_keep_cpu, num_to_keep, sizeof(int32_t), cudaMemcpyDeviceToHost, NMS_Stream);

  // Sync up
  cudaStreamSynchronize(NMS_Stream);

  // Filter Lanes to return
  cv::Mat output(*num_to_keep_cpu, num_prop, CV_32F);
  for(int i = 0; i < *num_to_keep_cpu; i++) {
    int index = keep_cpu[i];
    proposals.row(index).copyTo(output.row(i));
  }

  // Free allocated buffers
  cudaFree(k_proposals);
  cudaFree(k_idx);
  cudaFree(mask);
  cudaFree(keep);
  cudaFree(num_to_keep);
  free(keep_cpu);
  free(num_to_keep_cpu);

  return output;
}
