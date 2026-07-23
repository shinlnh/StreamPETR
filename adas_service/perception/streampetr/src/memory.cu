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

#include "memory.cuh"
#include <stdio.h>
#include <algorithm>
#include <array>
#include <iostream>

#define thrdPerBlk 256

using Mat4x4d = std::array<double, 16>;

template <typename T>
__global__ void MemShiftRight(T* __restrict__ buf, int len, int n, T fill_val = T())
{
    // Check if shifting value is correct
    if (n < 1) return;

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < n) {
        // Shift to right
        int i = len - 1 - n - idx;
        for (; i >= 0; i -= n) {
            buf[i + n] = buf[i];
        }
        // Check and fill first n
        i += n;
        if (i >= 0) {
            buf[i] = fill_val;
        }
    }
}

__global__ void SetIdentity(float* __restrict__ buf, int s, int n, bool set_zeros = false)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < n) {
        float *mat = buf + (idx * s * s);
        // Set entire mat to zero if required
        // This is very slow, you should use memset 0 before calling this kernel
        if (set_zeros) {
            for (int i = 0; i < s; i++) {
                for (int j = 0; j < s; j++) {
                    mat[i * j] = 0.0f;
                }
            }
        }

        // Set the diagonal to 1
        for (int i = 0; i < s; i++) {
            mat[i * i] = 1.0f;
        }
    }
}

/** NOTE: Block * Thread should equal n */
__global__ void TransformReferencePointsToEgo(
    const double* __restrict__ points,
    const Mat4x4d transform,
    float* __restrict__ outbuf,
    int n
)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < n) {
        const double &x = points[idx * 3 + 0];
        const double &y = points[idx * 3 + 1];
        const double &z = points[idx * 3 + 2];

        float &new_x = outbuf[idx * 3 + 0];
        float &new_y = outbuf[idx * 3 + 1];
        float &new_z = outbuf[idx * 3 + 2];

        // The transform is affine so we will skip w
        new_x = double(transform[0] * x + transform[1] * y + transform[2] * z + transform[3]);
        new_y = double(transform[4] * x + transform[5] * y + transform[6] * z + transform[7]);
        new_z = double(transform[8] * x + transform[9] * y + transform[10] * z + transform[11]);
        // float w = double(transform[12] * x + transform[13] * y + transform[14] * z + transform[15]);

        // if (w != 0) {
        //     new_x /= w;
        //     new_y /= w;
        //     new_z /= w;
        // }
    }
}

/** NOTE: Block * Thread should equal n */
__global__ void TransformReferencePointsToGlobal(
    const float* __restrict__ points,
    const Mat4x4d transform,
    double* __restrict__ outbuf,
    int n
)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < n) {
        const double x = (double)points[idx * 3 + 0];
        const double y = (double)points[idx * 3 + 1];
        const double z = (double)points[idx * 3 + 2];

        double &new_x = outbuf[idx * 3 + 0];
        double &new_y = outbuf[idx * 3 + 1];
        double &new_z = outbuf[idx * 3 + 2];

        // The transform is affine so we will skip w
        new_x = transform[0] * x + transform[1] * y + transform[2] * z + transform[3];
        new_y = transform[4] * x + transform[5] * y + transform[6] * z + transform[7];
        new_z = transform[8] * x + transform[9] * y + transform[10] * z + transform[11];
        // float w = transform[12] * x + transform[13] * y + transform[14] * z + transform[15];

        // if (w != 0) {
        //     new_x /= w;
        //     new_y /= w;
        //     new_z /= w;
        // }
    }
}

/** NOTE: Make sure to use <<<n, dim3(4, 4)>>> for launch */
__global__ void PoseGlobalToEgo(
    const Mat4x4d G2E_Mat,
    const double* __restrict__ G_Pose,
    float* __restrict__ E_Pose,
    int n
)
{
    int i = blockIdx.x;
    int row = threadIdx.y; // 0 to 3
    int col = threadIdx.x; // 0 to 3 

    if (i < n && row < 4 && col < 4) {
        float &value = E_Pose[(i * 4 + row) * 4 + col];
        const double *rowA = G2E_Mat.data() + row * 4;
        const double *colB = G_Pose + ((i * 4 * 4) + col);

        value = float(rowA[0] * colB[0] + rowA[1] * colB[4] + rowA[2] * colB[8] + rowA[3] * colB[12]);
    }
}

__global__ void MemRepeatPrepare(float* __restrict__ membuf, int n, int m, int s)
{
    if (s < 2 || n < 1 || m < 1) return;

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Check if aliasing can happen, if not then multi thread, else single thread
    bool no_aliasing = n < s;
    if (no_aliasing) {
        // Multi thread
        if (idx < n) {
            int src_idx = idx * m;
            int dest_idx = src_idx * s;
            for (int i = 0; i < m; i++) {
                membuf[dest_idx + i] = membuf[src_idx + i];
            }
        }
    }
    else {
        // Single thread
        if (idx == 0) {
            for (int i = n - 1; i >= 0; i++) {
                int src_idx = i * m;
                int dest_idx = src_idx * s;
                for (int j = 0; j < m; j++) {
                    membuf[dest_idx + j] = membuf[src_idx + j];
                }
            }
        }
    }
}

__global__ void MemRepeat(float* __restrict__ membuf, int n, int m, int s)
{
    if (s < 2 || n < 1 || m < 1) return;

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int src_idx = idx / s;

    if (src_idx < n) {
        float *src = membuf + (src_idx * s * m);
        float *dest = membuf + (idx * m);

        for (int i = 0; i < m; i++) {
            dest[i] = src[i];
        }
    }
}

__global__ void CalculateTimeDelta(double timestamp, double* membuf, float* outbuf, int n)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        outbuf[idx] = (float)(timestamp - membuf[idx]);
    }
}


template <typename T>
void debug_cpu_buf(T* cpu_buf, int size)
{
    for (int i = 0; i < size; i++){
        std::cout << cpu_buf[i] << " ";
    }
    std::cout << std::endl;
}

template <typename T>
void debug_gpu_buf(T* gpu_buf, int size)
{
    T debug_buffer[size];
    // copy from memory
    cudaMemcpy(
        debug_buffer,
        gpu_buf,
        sizeof(T) * size,
        cudaMemcpyDeviceToHost
    );
    for (int i = 0; i < size; i++){
        std::cout << debug_buffer[i] << " ";
    }
    std::cout << std::endl;
}

Memory::Memory(cudaStream_t stream, int memory_len, int num_propagated, int embed_dims)
{
    cudaStreamCreate(&this->mem_stream);

    this->ext_stream = stream;
    this->memory_len = memory_len;
    this->num_propagated = num_propagated;
    this->embed_dims = embed_dims;
    this->num_frame = memory_len / num_propagated;

    cudaMalloc(&timestamp_buf       , sizeof(double) * num_frame * 1);
    cudaMalloc(&egopose_buf         , sizeof(double) * num_frame * 4 * 4);
    cudaMalloc(&reference_point_buf , sizeof(double) * memory_len * 3);
    cudaMalloc(&embedding_buf       , sizeof(float) * memory_len * embed_dims);
    cudaMalloc(&velo_buf            , sizeof(float) * memory_len * 2);
}

Memory::~Memory()
{
    cudaStreamDestroy(this->mem_stream);

    cudaFree(timestamp_buf);
    cudaFree(egopose_buf);
    cudaFree(reference_point_buf);
    cudaFree(embedding_buf);
    cudaFree(velo_buf);
}

void Memory::init_memory(double timestamp, const double *ego_pose, const float *pseudo_reference_points)
{
    Mat4x4d ego_pose_mat;
    std::copy(ego_pose, ego_pose + 16, ego_pose_mat.begin());
    
    // zero init
    cudaMemsetAsync(egopose_buf,         0, sizeof(double) * num_frame * 4 * 4,      mem_stream);
    cudaMemsetAsync(embedding_buf,       0, sizeof(float) * memory_len * embed_dims, mem_stream);
    cudaMemsetAsync(velo_buf,            0, sizeof(float) * memory_len * 2,          mem_stream);

    // init timestamp
    double ts_buf[this->num_frame];
    for (int i = 0; i < this->num_frame; i++) {
        ts_buf[i] = timestamp;
    }
    cudaMemcpyAsync(
        timestamp_buf,
        ts_buf,
        sizeof(double) * num_frame,
        cudaMemcpyHostToDevice,
        mem_stream
    );

    // init reference point (local)
    cudaMemsetAsync(memory_reference_point, 0, sizeof(float) * memory_len * 3, mem_stream);
    cudaMemcpyAsync(
        memory_reference_point,
        pseudo_reference_points,
        sizeof(float) * num_propagated * 3,
        cudaMemcpyHostToDevice,
        mem_stream
    );
    // then transform back to global
    TransformReferencePointsToGlobal<<<1, memory_len, 0, mem_stream>>>(
        memory_reference_point, ego_pose_mat, reference_point_buf, memory_len);

    // init ego pose
    cudaMemcpyAsync(
        egopose_buf,
        ego_pose,
        sizeof(double) * 4 * 4,
        cudaMemcpyHostToDevice,
        mem_stream
    );

    // Set init flag to true
    cudaStreamSynchronize(this->mem_stream);
    this->lastest_timestamp = timestamp;
    this->is_initialized = true;
}

void Memory::read_memory(double timestamp, const double * ego_pose_inv)
{
    Mat4x4d ego_pose_inv_mat;
    std::copy(ego_pose_inv, ego_pose_inv + 16, ego_pose_inv_mat.begin());

    // convert clock time to time delta
    CalculateTimeDelta<<<1, num_frame, 0, ext_stream>>>(
        timestamp, timestamp_buf, memory_timestamp, num_frame);
    MemRepeatPrepare<<<1, num_frame, 0, ext_stream>>>(
        memory_timestamp, num_frame, 1, num_propagated);
    MemRepeat<<<(memory_len + thrdPerBlk) / thrdPerBlk, thrdPerBlk, 0, ext_stream>>>(
        memory_timestamp, num_frame, 1, num_propagated);

    // convert global to ego coord
    PoseGlobalToEgo<<<num_frame, {4, 4, 1}, 0, ext_stream>>>(
        ego_pose_inv_mat, egopose_buf, memory_egopose, num_frame);
    MemRepeatPrepare<<<1, num_frame, 0, ext_stream>>>(
        memory_egopose, num_frame, 16, num_propagated);
    MemRepeat<<<(memory_len + thrdPerBlk) / thrdPerBlk, thrdPerBlk, 0, ext_stream>>>(
        memory_egopose, num_frame, 16, num_propagated);

    TransformReferencePointsToEgo<<<1, memory_len, 0, ext_stream>>>(
        reference_point_buf, ego_pose_inv_mat, memory_reference_point, memory_len);

    // copy from memory
    cudaMemcpyAsync(
        memory_embedding,
        embedding_buf,
        sizeof(float) * memory_len * embed_dims,
        cudaMemcpyDeviceToDevice,
        ext_stream
    );

    cudaMemcpyAsync(
        memory_velo,
        velo_buf,
        sizeof(float) * memory_len * 2,
        cudaMemcpyDeviceToDevice,
        ext_stream
    );
}

void Memory::update_memory(double timestamp, const double * ego_pose)
{
    Mat4x4d ego_pose_mat;
    std::copy(ego_pose, ego_pose + 16, ego_pose_mat.begin());

    // slide buffers
    MemShiftRight<<<1, 1, 0, ext_stream>>>(
        timestamp_buf, num_frame, 1, timestamp  // insert new timestamp
    );
    MemShiftRight<<<1, 1, 0, ext_stream>>>(
        (Mat4x4d*)egopose_buf, num_frame, 1, ego_pose_mat  // insert new ego pose
    );
    MemShiftRight<<<(num_propagated * 3 + thrdPerBlk) / thrdPerBlk, thrdPerBlk, 0, ext_stream>>>(
        reference_point_buf, memory_len * 3, num_propagated * 3
    );
    MemShiftRight<<<(num_propagated * embed_dims + thrdPerBlk) / thrdPerBlk, thrdPerBlk, 0, ext_stream>>>(
        embedding_buf, memory_len * embed_dims, num_propagated * embed_dims
    );
    MemShiftRight<<<(num_propagated * 2 + thrdPerBlk) / thrdPerBlk, thrdPerBlk, 0, ext_stream>>>(
        velo_buf, memory_len * 2, num_propagated * 2
    );

    // convert from ego to global coord and insert
    TransformReferencePointsToGlobal<<<1, num_propagated, 0, ext_stream>>>(
        rec_reference_points, ego_pose_mat, reference_point_buf, num_propagated);

    // insert new topk memory
    cudaMemcpyAsync(
        embedding_buf,
        rec_memory,
        sizeof(float) * num_propagated * embed_dims,
        cudaMemcpyDeviceToDevice,
        ext_stream
    );
    // debug_gpu_buf(embedding_buf, embed_dims);
    // debug_gpu_buf(rec_memory, embed_dims);
    // throw std::runtime_error("");
    cudaMemcpyAsync(
        velo_buf,
        rec_velo,
        sizeof(float) * num_propagated * 2,
        cudaMemcpyDeviceToDevice,
        ext_stream
    );

    this->lastest_timestamp = timestamp;
}

void Memory::sync()
{
    cudaStreamSynchronize(this->mem_stream);
}

void Memory::DebugPrint()
{
    // double temp_buf[16];
    // cudaMemcpy(reinterpret_cast<void*>(temp_buf), mem_buf, sizeof(double) * 16, cudaMemcpyDeviceToHost);
    // for( int i=0; i<16; i++ ) {
    //     printf("%f ", temp_buf[i]);
    // }
}