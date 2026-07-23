/*
 * SPDX-FileCopyrightText: Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MEMORY_CUH
#define MEMORY_CUH

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
// #include <NvInferRuntime.h>
 
class Memory {
private:
    cudaStream_t ext_stream;  // the stream of StreamPETR
    cudaStream_t mem_stream;  // memory's internal stream

    int num_frame;      // The number of old frame to keep
    int num_propagated; // The number of detection kept per frame
    int memory_len;     // Total number of memory entries
    int embed_dims;     // Dim size of detection's embedding

    // Memory buffers (in global space)
    double *timestamp_buf;
    double *egopose_buf;
    double *reference_point_buf;
    float *embedding_buf;
    float *velo_buf;

public:
    // Pointers to Detector's memory input
    float *memory_timestamp;
    float *memory_egopose;
    float *memory_reference_point;
    float *memory_embedding;
    float *memory_velo;

    // Pointers to new memory entries from Detector
    float *rec_reference_points;
    float *rec_memory;
    float *rec_velo;

    // Check if init_memory() is called atleast once
    bool is_initialized = false;
    double lastest_timestamp = 0;

public:
    Memory(cudaStream_t stream, int memory_len, int num_propagated, int embed_dims);
    ~Memory();

    void init_memory(double timestamp, const double *ego_pose, const float *pseudo_reference_points);
    void read_memory(double timestamp, const double *ego_pose_inv);
    void update_memory(double timestamp, const double *ego_pose);
    void sync();

    void DebugPrint();
}; // class Memory

#endif // MEMORY_CUH