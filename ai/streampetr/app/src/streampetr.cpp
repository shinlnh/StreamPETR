#include "streampetr.hpp"

Logger gLogger;

StreamPETR::StreamPETR(Config cfg)
{
    /* ---- Model configurations ---- */
    this->weights_path = cfg.weights_path;

    this->cam2imgs = cfg.cam2imgs;
    this->ego2cams = cfg.ego2cams;
    this->lidar2ego = cfg.lidar2ego;

    this->position_range = cfg.position_range;
    this->depth_start = cfg.depth_start;
    this->depth_num = cfg.depth_num;
    this->LID = cfg.LID;

    this->embed_dims = cfg.embed_dims;
    this->num_propagated = cfg.num_propagated;
    this->memory_len = cfg.memory_len;

    this->topk = cfg.topk;
    this->threshold = cfg.threshold;
    this->post_center_range = cfg.post_center_range;

    /* ---- Model engine ---- */
    // TensorRT
    cudaSetDevice(0);
    cudaStream_t raw_stream;
    cudaStreamCreate(&raw_stream);
    this->stream_.reset(raw_stream);
    this->runtime.reset(createInferRuntime(gLogger));

    // Engines
    std::string backbone_path = this->weights_path + "simplify_extract_img_feat.engine";
    std::string detector_path = this->weights_path + "simplify_pts_head_memory.engine";
    
    this->backbone_ = std::make_unique<SubNetwork>(backbone_path, this->runtime.get());
    this->detector_ = std::make_unique<SubNetwork>(detector_path, this->runtime.get());

    this->backbone_->EnableCudaGraph(this->stream_.get());
    this->detector_->EnableCudaGraph(this->stream_.get());

    // Profiler
    this->dur_backbone_ = std::make_unique<Duration>("backbone");
    this->dur_detector_ = std::make_unique<Duration>("detector");

    /* ---- Model memory ---- */
    this->memory = std::make_unique<Memory>(
        this->stream_.get(), this->memory_len, this->num_propagated, this->embed_dims);

    memory->memory_timestamp       = static_cast<float*>(detector_->bindings.at("memory_timestamp")->ptr);
    memory->memory_egopose         = static_cast<float*>(detector_->bindings.at("memory_egopose")->ptr);
    memory->memory_reference_point = static_cast<float*>(detector_->bindings.at("memory_reference_point")->ptr);
    memory->memory_embedding       = static_cast<float*>(detector_->bindings.at("memory_embedding")->ptr);
    memory->memory_velo            = static_cast<float*>(detector_->bindings.at("memory_velo")->ptr);

    memory->rec_reference_points = static_cast<float*>(detector_->bindings.at("rec_reference_points")->ptr);
    memory->rec_memory           = static_cast<float*>(detector_->bindings.at("rec_memory")->ptr);
    memory->rec_velo             = static_cast<float*>(detector_->bindings.at("rec_velo")->ptr);

    /* ---- Output decoder ---- */
    int num_cls = detector_->bindings["all_cls_scores"]->shape(-1);
    int num_proposals = detector_->bindings["all_cls_scores"]->shape(-2);
    this->decoder = std::make_unique<Decoder>(
        this->stream_.get(), num_proposals, num_cls, this->topk, this->threshold, this->post_center_range, this->lidar2ego.data());

    decoder->all_cls_scores = static_cast<float*>(detector_->bindings.at("all_cls_scores")->ptr);
    decoder->all_bbox_preds = static_cast<float*>(detector_->bindings.at("all_bbox_preds")->ptr);

    /* ---- Init static variables ---- */
    // Calculate ida mats
    read_input_shape();
    _calc_stride();
    _calc_ida_mats(cfg.img_height, cfg.img_width);

    // Calculate pos_embed and cone
    _calc_img_lidar_transform();
    auto memory_centers = prepare_location();
    position_embedding(memory_centers);
        
    // Copy to GPU (one time only as it is static)
    this->detector_->bindings["pos_embed"]->copy(this->static_pos_embed.data(), this->stream_.get());
    this->detector_->bindings["cone"]->copy(this->static_cone.data(), this->stream_.get());
}

StreamPETR::~StreamPETR()
{
}

void StreamPETR::read_input_shape()
{
    auto input_dims = this->backbone_->bindings["img"]->dim;
    if (input_dims.nbDims != 5) {
        throw std::runtime_error(
            "Expected input dim of 5, got " + std::to_string(input_dims.nbDims) +"!!!"
        );
    }

    for (int32_t i = 0; i < input_dims.nbDims; i++) {
        this->input_shape[i] = input_dims.d[i];
    }

    if (this->input_shape[0] != 1) {
        throw std::runtime_error(
            "Expect Batch size = 1, got " + std::to_string(this->input_shape[0]) + "!!!"
        );
    }
}

void StreamPETR::_calc_stride()
{
    auto feat_dims = this->detector_->bindings["img_feats"]->dim;
    if (feat_dims.nbDims != 5) {
        throw std::runtime_error(
            "Expected feature dim of 5, got " + std::to_string(feat_dims.nbDims) +"!!!"
        );
    }
    
    uint32_t fB = feat_dims.d[0];
    uint32_t fH = feat_dims.d[3];
    uint32_t fW = feat_dims.d[4];

    if (fB != 1) {
        throw std::runtime_error(
            "Expect Batch size = 1, got " + std::to_string(fB) + "!!!"
        );
    }

    uint32_t H = this->input_shape[3];
    uint32_t W = this->input_shape[4];

    if ((H / fH) != (W / fW)) {
        throw std::runtime_error(
            "Mismatch stride!!! H: " + std::to_string(H / fH) + ", W: " + std::to_string(W / fW)
        );
    }

    this->stride = H / fH;
}

void StreamPETR::_calc_ida_mats(
    std::array<uint32_t, NUM_CHANNEL> img_height,
    std::array<uint32_t, NUM_CHANNEL> img_width
)
{
    const uint32_t N = this->input_shape[1];
    const uint32_t H = this->input_shape[3];
    const uint32_t W = this->input_shape[4];

    for (uint32_t cam_idx = 0; cam_idx < N; cam_idx++) {
        const uint32_t img_H = img_height[cam_idx];
        const uint32_t img_W = img_width[cam_idx];

        // resize
        const float scale_factor = std::max((float)H / img_H, (float)W / img_W);
        const int resize_H = img_H * scale_factor + 0.5;
        const int resize_W = img_W * scale_factor + 0.5;
        
        // crop (bottom center)
        const int crop_y = resize_H - H;
        const int crop_x = (resize_W - W) / 2;

        // calculate ida mat
        Eigen::Matrix3fRM ida_mat = Eigen::Matrix3fRM::Identity();
        ida_mat.topLeftCorner<2, 2>() *= scale_factor;
        ida_mat(0, 2) = -crop_x;
        ida_mat(1, 2) = -crop_y;
        this->ida_mats[cam_idx] = ida_mat;
    }
}

void StreamPETR::_calc_img_lidar_transform()
{
    const uint32_t N = this->input_shape[1];
    for (uint32_t cam_idx = 0; cam_idx < N; cam_idx++) {
        Eigen::Matrix4fRM ego2cam = this->ego2cams[cam_idx];
        Eigen::Matrix3fRM ida_mat = this->ida_mats[cam_idx];

        // create 4x4 camera to image transformation matrix
        Eigen::Matrix4fRM cam2img = Eigen::Matrix4fRM::Identity();
        auto cam2img_3x3 = cam2img.topLeftCorner<3, 3>();
        cam2img_3x3 = this->cam2imgs[cam_idx];

        // compensate for resize, crop
        cam2img_3x3 = ida_mat * cam2img_3x3;
        this->compensated_cam2imgs[cam_idx] = cam2img_3x3;

        // get the lidar to image pixel matrix
        Eigen::Matrix4fRM lidar2img = cam2img * ego2cam * this->lidar2ego;
        this->lidar2imgs[cam_idx] = lidar2img;
    }
}

std::vector<Eigen::MatrixXfRM> StreamPETR::prepare_location()
{
    const uint32_t N = this->input_shape[1];
    const uint32_t H = this->input_shape[3];
    const uint32_t W = this->input_shape[4];
    const uint32_t feat_h = H / this->stride;
    const uint32_t feat_w = W / this->stride;

    Eigen::MatrixXfRM locations(feat_h * feat_w, 2);
    // Calculate feature centers
    for (uint32_t i = 0; i < feat_h; i++) {
        float y = float(i * stride + stride / 2) / H;

        for (uint32_t j = 0; j < feat_w; j++) {
            float x = float(j * stride + stride / 2) / W;

            locations.row(i * feat_w + j) << x, y;
        }
    }

    // Copy across cameras
    std::vector<Eigen::MatrixXfRM> locations_list(N);
    for (uint32_t cam_idx = 0; cam_idx < N; cam_idx++) {
        locations_list[cam_idx] = locations;
    }
    
    return locations_list;
}

void StreamPETR::position_embedding(std::vector<Eigen::MatrixXfRM> memory_centers)
{
    const float eps = 1e-5;

    const uint32_t N = this->input_shape[1];
    const uint32_t H = this->input_shape[3];
    const uint32_t W = this->input_shape[4];
    const uint32_t mem_HW = (H / this->stride) * (W / this->stride);

    const uint32_t D = this->depth_num;
    const float D0 = this->depth_start;
    const float D_max = std::max(this->position_range[3], this->position_range[4]);

    auto min_range = Eigen::Map<const Eigen::RowVector3f>(this->position_range.data());
    auto max_range = Eigen::Map<const Eigen::RowVector3f>(this->position_range.data() + 3);

    /* ---- Calculate depth bins ---- */
    Eigen::ArrayXf coords_d;
    Eigen::ArrayXf index = Eigen::ArrayXf::LinSpaced(D, 0, D - 1);
    if (this->LID) {
        float bin_size = (D_max - D0) / (D * (D + 1));
        coords_d = D0 + bin_size * index * (index + 1.0f);
    }
    else {
        float bin_size = (D_max - D0) / D;
        coords_d = D0 + bin_size * index;
    }

    /* ---- Position embedding ---- */
    if (memory_centers[0].rows() != mem_HW) {
        throw std::runtime_error("Feature's center matrix size mismatched");
    }

    // 1. Transform from image space to lidar space
    Eigen::MatrixXfRM feat_coords_3D;
    feat_coords_3D.resize(N * mem_HW * D, 3);
    for (uint32_t cam_idx = 0; cam_idx < N; cam_idx++) {
        Eigen::Matrix4fRM img2lidars = this->lidar2imgs[cam_idx].inverse();
        Eigen::MatrixXfRM feat_coords_2D = memory_centers[cam_idx];

        // Convert from normalized value to pixel coord value
        feat_coords_2D.col(0) *= W;
        feat_coords_2D.col(1) *= H;

        // Make homogeneous (add depth and ones dimension)
        Eigen::MatrixXfRM feat_coords;
        feat_coords.resize(mem_HW * D, 4);
        for (uint32_t feat_idx = 0; feat_idx < mem_HW; feat_idx++) {
            // Copy xy value
            auto feat_xy = feat_coords_2D.row(feat_idx);
            feat_coords.block(feat_idx * D, 0, D, 2).rowwise() = feat_xy;

            // Add depth
            feat_coords.col(2).segment(feat_idx * D, D) = coords_d.matrix();

            // Adjust xy based on depth
            feat_coords.block(feat_idx * D, 0, D, 2).array().colwise() *= coords_d.max(eps);
        }
        // Add ones
        feat_coords.col(3).setOnes();

        // Perspective transform to lidar 3D space
        feat_coords = feat_coords * img2lidars.transpose();
        feat_coords_3D.middleRows(cam_idx * mem_HW * D, mem_HW * D) = feat_coords.leftCols(3);
    }

    // Normalize
    feat_coords_3D.array().rowwise() -= min_range.array();
    feat_coords_3D.array().rowwise() /= (max_range - min_range).array();

    // 2. Prepare the cone
    this->static_cone.resize(N * mem_HW, 8);
    for (uint32_t feat_idx(0), cam_idx(0); feat_idx < N * mem_HW; feat_idx++, ++cam_idx %= N) {
        /** NOTE:
         * THIS IS MATCHING FEATURES WITH WRONG INTRINSIC
         * I'M KEEPING IT THIS WAY BECAUSE THE ORIGINAL MODEL WAS TRAINED
         *  WITH THIS ERROR AND THUS CAN'T BE CORRECTED RIGHT NOW
         * IF YOU RETRAIN MODEL, CORRECT IT BOTH HERE AND IN THE TRAINING
        */
        const Eigen::Matrix3fRM &intrinsic = this->compensated_cam2imgs[cam_idx % N];
        this->static_cone(feat_idx, 0) = intrinsic(0, 0) / 1e3f;
        this->static_cone(feat_idx, 1) = intrinsic(1, 1) / 1e3f;

        static const uint32_t mid_depth = 34, last_depth = D - 1;
        this->static_cone.block(feat_idx, 2, 1, 3) = feat_coords_3D.row(feat_idx * D + last_depth);
        this->static_cone.block(feat_idx, 5, 1, 3) = feat_coords_3D.row(feat_idx * D + mid_depth);
    }

    // 3. Encode into embeddings
    Eigen::Map<Eigen::MatrixXfRM> reshaped_feat_coords_3D(feat_coords_3D.data(), N * mem_HW, D * 3);
    feat_coords_3D = inverse_sigmoid(reshaped_feat_coords_3D);
    
    this->static_pos_embed = position_encoder(feat_coords_3D);
}

Eigen::MatrixXfRM StreamPETR::position_encoder(Eigen::MatrixXfRM x)
{
    /* ---- Load weights ---- */
    std::string w1_path = this->weights_path + "position_encoder_w1.bin";
    std::string b1_path = this->weights_path + "position_encoder_b1.bin";
    std::string w2_path = this->weights_path + "position_encoder_w2.bin";
    std::string b2_path = this->weights_path + "position_encoder_b2.bin";

    Eigen::MatrixXfRM w1 = load_npy_matrix(w1_path, this->depth_num * 3, this->embed_dims * 4);
    Eigen::MatrixXfRM b1 = load_npy_matrix(b1_path, 1, this->embed_dims * 4);
    Eigen::MatrixXfRM w2 = load_npy_matrix(w2_path, this->embed_dims * 4, this->embed_dims);
    Eigen::MatrixXfRM b2 = load_npy_matrix(b2_path, 1, this->embed_dims);

    /* ---- MLP forward ---- */
    // Linear 1 + ReLU
    x = (x * w1).rowwise() + b1.row(0);
    x = x.cwiseMax(0.0f);

    // Linear 2
    x = (x * w2).rowwise() + b2.row(0);

    return x;
}

StreamPETR::Output StreamPETR::infer(Input input)
{
    /* ---- 1. Pre-processing ---- */
    std::vector<float> img_data = img_preprocess(input.rawImgs);

    /* ---- 2. Backbone ---- */
    // Copy input
    this->backbone_->bindings["img"]->copy(img_data, this->stream_.get());
    
    // Inference
    this->dur_backbone_->MarkBegin(this->stream_.get());
    this->backbone_->Enqueue(this->stream_.get());
    this->dur_backbone_->MarkEnd(this->stream_.get());

    /* ---- 3. Detector ---- */
    // Init memory on first run
    if (this->memory->is_initialized == false) {
        std::string ref_point_path = this->weights_path + "pseudo_reference_points.bin";
        Eigen::MatrixXfRM ref_points = load_npy_matrix(ref_point_path, this->num_propagated, 3);
        this->memory->init_memory(input.timestamp, input.ego_pose.data(), ref_points.data());
    }

    // Read memory inputs
    Eigen::Matrix4dRM ego_pose_inv = invert_transform_matrix(input.ego_pose);
    this->memory->read_memory(input.timestamp, ego_pose_inv.data());

    // Copy inputs from backbone
    this->detector_->bindings["img_feats"]->copy(this->backbone_->bindings["img_feats"], this->stream_.get());

    // Inference
    this->dur_detector_->MarkBegin(this->stream_.get());
    this->detector_->Enqueue(this->stream_.get());
    this->dur_detector_->MarkEnd(this->stream_.get());

    // Update memory
    static constexpr double min_delta_t = 0.49; // the memory time step that model was trained on
    double delta_t = input.timestamp - this->memory->lastest_timestamp;
    if (delta_t > min_delta_t) {
        this->memory->update_memory(input.timestamp, input.ego_pose.data());
    }

    /* ---- 4. Post-processing ---- */
    Output output;
    output.objects = this->decoder->decode();

    return output;
}

std::vector<float> StreamPETR::img_preprocess(std::array<cv::Mat, NUM_CHANNEL> rawImgs)
{
    uint32_t final_size = this->backbone_->bindings["img"]->volume;
    std::vector<float> data_buffer(final_size);

    const uint32_t N = this->input_shape[1];
    const uint32_t C = this->input_shape[2];
    const uint32_t H = this->input_shape[3];
    const uint32_t W = this->input_shape[4];

    for (uint32_t cam_idx = 0; cam_idx < N; cam_idx++) {
        cv::Mat img = rawImgs[cam_idx]; //.clone();  // clone if perform inplace modification
        uint32_t img_H = img.rows;
        uint32_t img_W = img.cols;

        // 1. Resize (Maintain Aspect Ratio)
        double scale_factor = std::max((float)H / img_H, (float)W / img_W);
        uint32_t resize_H = img_H * scale_factor + 0.5;  // Round up 0.5
        uint32_t resize_W = img_W * scale_factor + 0.5;
        
        cv::resize(img, img, cv::Size(resize_W, resize_H), 0, 0, cv::INTER_LINEAR);

        // 2. Crop (Bottom Center)
        int crop_y = resize_H - H;
        int crop_x = (resize_W - W) / 2;
        cv::Rect roi(crop_x, crop_y, W, H);
        img = img(roi);

        // 3. Convert BGR to RGB, normalize, HWC to CHW
        static constexpr float mean[] = {123.675f, 116.28f, 103.53f};
        static constexpr float std[] = {58.395f, 57.12f, 57.375f};
        uint32_t channel_size = H * W;
        uint32_t image_size = C * H * W;

        for (uint32_t y = 0; y < H; ++y) {
            // Get pointer to the start of the row in the source BGR image
            const uint8_t* row_ptr = img.ptr<uint8_t>(y);

            for (uint32_t x = 0; x < W; ++x) {
                // Get pixel value and normalize (Note that mean and std is in RGB order)
                float b = (static_cast<float>(row_ptr[x * C + 0]) - mean[2]) / std[2];
                float g = (static_cast<float>(row_ptr[x * C + 1]) - mean[1]) / std[1];
                float r = (static_cast<float>(row_ptr[x * C + 2]) - mean[0]) / std[0];

                // Calculate destination indices for CHW ordering
                uint32_t pixel_idx = cam_idx * image_size + y * W + x;

                // Converting BGR -> RGB and HWC to CHW
                data_buffer[pixel_idx]                    = r;
                data_buffer[pixel_idx + channel_size]     = g;
                data_buffer[pixel_idx + 2 * channel_size] = b;
            }
        }
    }

    return data_buffer;
}
