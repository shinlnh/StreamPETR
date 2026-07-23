#ifndef STREAMPETR_HPP
#define STREAMPETR_HPP

#include "trt_engine.hpp"
#include "memory.cuh"
#include "decoder.cuh"
#include "utils.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <array>
#include <vector>

class StreamPETR
{
public:
    enum CAM_CHANNEL: int
    {
        F = 0,
        FR,
        FL,
        B,
        BL,
        BR,
        NUM_CHANNEL
    }; // enum CAMERA_CHANNEL

    struct Config
    {
        // Path to model's weights
        std::string weights_path;

        // Preprocessing & Position embedding
        std::array<float, 6> position_range;
        uint32_t embed_dims;
        float depth_start;
        uint32_t depth_num;
        bool LID;

        std::array<uint32_t, NUM_CHANNEL> img_width;
        std::array<uint32_t, NUM_CHANNEL> img_height;
        std::array<Eigen::Matrix3fRM, NUM_CHANNEL> cam2imgs;
        std::array<Eigen::Matrix4fRM, NUM_CHANNEL> ego2cams;
        Eigen::Matrix4fRM lidar2ego;

        // Memory
        uint32_t num_propagated;
        uint32_t memory_len;

        // Postprocessing & Decoder
        uint32_t topk;
        float threshold;
        std::array<float, 6> post_center_range;
    };

    struct Input
    {
        double timestamp = 0;
        std::array<cv::Mat, NUM_CHANNEL> rawImgs;
        Eigen::Matrix4dRM ego_pose;
    };

    struct Output
    {
        std::vector<Decoder::Detection> objects;
    };

public:
    /* ---- Model configs ---- */
    std::string weights_path;

    uint32_t stride;
    std::array<uint32_t, 5> input_shape;
    std::array<Eigen::Matrix3fRM, NUM_CHANNEL> cam2imgs;
    std::array<Eigen::Matrix4fRM, NUM_CHANNEL> ego2cams;
    Eigen::Matrix4fRM lidar2ego;

    std::array<float, 6> position_range;
    float depth_start;
    uint32_t depth_num;
    bool LID;

    std::array<Eigen::Matrix3fRM, NUM_CHANNEL> ida_mats;
    std::array<Eigen::Matrix3fRM, NUM_CHANNEL> compensated_cam2imgs;
    std::array<Eigen::Matrix4fRM, NUM_CHANNEL> lidar2imgs;

    Eigen::MatrixXfRM static_pos_embed;
    Eigen::MatrixXfRM static_cone;

    uint32_t embed_dims;
    uint32_t num_propagated;
    uint32_t memory_len;

    uint32_t topk;
    float threshold;
    std::array<float, 6> post_center_range;

    /* ---- Model engine ---- */
    // TensorRT
    RuntimePtr runtime;
    CudaStreamPtr stream_;

    // Engines
    std::unique_ptr<SubNetwork> backbone_;
    std::unique_ptr<SubNetwork> detector_;

    // Profilers
    std::unique_ptr<Duration> dur_backbone_;
    std::unique_ptr<Duration> dur_detector_;

    /* ---- Model memory ---- */
    std::unique_ptr<Memory> memory;

    /* ---- Output decoder ---- */
    std::unique_ptr<Decoder> decoder;

public:
    StreamPETR(Config cfg);
    ~StreamPETR();

    Output infer(Input input);

private:
    void read_input_shape();
    void _calc_stride();
    void _calc_ida_mats(
        std::array<uint32_t, NUM_CHANNEL> img_height,
        std::array<uint32_t, NUM_CHANNEL> img_width
    );
    void _calc_img_lidar_transform();
    std::vector<Eigen::MatrixXfRM> prepare_location();
    void position_embedding(std::vector<Eigen::MatrixXfRM> memory_centers);
    Eigen::MatrixXfRM position_encoder(Eigen::MatrixXfRM x);

    std::vector<float> img_preprocess(std::array<cv::Mat, NUM_CHANNEL> rawImgs);
};

#endif // STREAMPETR_HPP