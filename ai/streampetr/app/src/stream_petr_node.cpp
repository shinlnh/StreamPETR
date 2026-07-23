// Copyright 2024 Koji Minoda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "stream_petr_node.hpp"

#include "utils.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <algorithm>

namespace tensorrt_stream_petr
{
DetectedObject bbox_to_ros_msg(const Decoder::Detection& obj) const
{
    // cx, cy, cz, w, l, h, rot, vx, vy
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = obj.cx;
    object.kinematics.pose_with_covariance.pose.position.y = obj.cy;
    object.kinematics.pose_with_covariance.pose.position.z = obj.cz;
    object.shape.dimensions.x = obj.w;
    object.shape.dimensions.y = obj.l;
    object.shape.dimensions.z = obj.h;
    const double yaw = obj.yaw;
    object.kinematics.pose_with_covariance.pose.orientation.w = cos(yaw * 0.5);
    object.kinematics.pose_with_covariance.pose.orientation.x = 0;
    object.kinematics.pose_with_covariance.pose.orientation.y = 0;
    object.kinematics.pose_with_covariance.pose.orientation.z = sin(yaw * 0.5);

    object.existence_probability = obj.conf;
    object.kinematics.has_position_covariance = false;
    object.kinematics.has_twist = false;
    object.shape.type = 0;
    
    // autoware_perception_msgs::msg::ObjectClassification classification;
    // classification.probability = 1.0f;
    // classification.label = getSemanticType(config_.class_names[bbox.label]);
    // object.classification.push_back(classification);

    return object;
}

StreamPetrNode::StreamPetrNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("tensorrt_stream_petr", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    /* ---- Read parameters ---- */
    // Read from config file
    const auto param_weights_path = declare_parameter<std::string>("model_params.weights_path");
    const auto param_position_range = declare_parameter<std::vector<double>>("model_params.position_range");
    const auto param_embed_dims = declare_parameter<int>("model_params.embed_dims");
    const auto param_depth_start = declare_parameter<double>("model_params.depth_start");
    const auto param_depth_num = declare_parameter<int>("model_params.depth_num");
    const auto param_LID = declare_parameter<bool>("model_params.LID");

    const auto param_num_propagated = declare_parameter<int>("model_params.num_propagated");
    const auto param_memory_len = declare_parameter<int>("model_params.memory_len");

    const auto param_topk = declare_parameter<int>("model_params.topk");
    const auto param_threshold = declare_parameter<double>("model_params.threshold");
    const auto param_point_cloud_range = declare_parameter<std::vector<double>>("model_params.point_cloud_range");

    // Store
    this->model_cfg.weights_path = param_weights_path;
    this->model_cfg.position_range = cast_to_float<CAM_CHANNEL::NUM_CHANNEL>(param_position_range);
    this->model_cfg.embed_dims = param_embed_dims;
    this->model_cfg.depth_start = param_depth_start;
    this->model_cfg.depth_num = param_depth_num;
    this->model_cfg.LID = param_LID;

    this->model_cfg.num_propagated = param_num_propagated;
    this->model_cfg.memory_len = param_memory_len;

    this->model_cfg.topk = param_topk;
    this->model_cfg.threshold = param_threshold;
    this->model_cfg.point_cloud_range = cast_to_float<CAM_CHANNEL::NUM_CHANNEL>(param_point_cloud_range);

    /* ---- Read hardware info ---- */
    // Subscribe for camera info
    this->subscribeCameraInfo();

    // Create timer to check if got all camera info received,
    // then continue initialization
    initialization_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { this->checkInitialization(); }
    );
}

void StreamPetrNode::checkInitialization()
{
    if (camera_info_received_flag_ == false)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Waiting for Camera Info and TF Transform Initialization..."
        );
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Finished reading Camera Info!");

    // Stop timer
    initialization_check_timer_->cancel();
    initialization_check_timer_.reset();
    
    // Init model
    this->initModel();

    // Start Communication
    this->initPublishers();
    this->initSubscribers()
}

void StreamPetrNode::subscribeCameraInfo()
{
    auto cam_info_qos = rclcpp::SensorDataQoS{}.keep_last(1);

    sub_caminfo_[CAM_CHANNEL::F] = this->create_subscription<CameraInfo>(
        "~/input/topic_img_front/camera_info",
        cam_info_qos,
        [this](CameraInfo::ConstSharedPtr msg) { cameraInfoCallback(CAM_CHANNEL::F, msg); });

    sub_caminfo_[CAM_CHANNEL::FR] = this->create_subscription<CameraInfo>(
        "~/input/topic_img_front_right/camera_info",
        cam_info_qos,
        [this](CameraInfo::ConstSharedPtr msg) { cameraInfoCallback(CAM_CHANNEL::FR, msg); });

    sub_caminfo_[CAM_CHANNEL::FL] = this->create_subscription<CameraInfo>(
        "~/input/topic_img_front_left/camera_info",
        cam_info_qos,
        [this](CameraInfo::ConstSharedPtr msg) { cameraInfoCallback(CAM_CHANNEL::FL, msg); });

    sub_caminfo_[CAM_CHANNEL::B] = this->create_subscription<CameraInfo>(
        "~/input/topic_img_back/camera_info",
        cam_info_qos,
        [this](CameraInfo::ConstSharedPtr msg) { cameraInfoCallback(CAM_CHANNEL::B, msg); });

    sub_caminfo_[CAM_CHANNEL::BL] = this->create_subscription<CameraInfo>(
        "~/input/topic_img_back_left/camera_info",
        cam_info_qos,
        [this](CameraInfo::ConstSharedPtr msg) { cameraInfoCallback(CAM_CHANNEL::BL, msg); });

    sub_caminfo_[CAM_CHANNEL::BR] = this->create_subscription<CameraInfo>(
        "~/input/topic_img_back_right/camera_info",
        cam_info_qos,
        [this](CameraInfo::ConstSharedPtr msg) { cameraInfoCallback(CAM_CHANNEL::BR, msg); });

    return;
}

void StreamPetrNode::initPublishers()
{
    // Detected objects
    this->pub_objects_ = this->create_publisher<DetectedObjects>("~/output/boxes", rclcpp::QoS{1});
}

void StreamPetrNode::initSubscribers()
{
    // Camera Image topics
    auto img_qos = rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile();

    sub_img_[CAM_CHANNEL::F].subscribe(this, "~/input/topic_img_front", img_qos);
    sub_img_[CAM_CHANNEL::FR].subscribe(this, "~/input/topic_img_front_right", img_qos);
    sub_img_[CAM_CHANNEL::FL].subscribe(this, "~/input/topic_img_front_left", img_qos);

    sub_img_[CAM_CHANNEL::B].subscribe(this, "~/input/topic_img_back", img_qos);
    sub_img_[CAM_CHANNEL::BL].subscribe(this, "~/input/topic_img_back_left", img_qos);
    sub_img_[CAM_CHANNEL::BR].subscribe(this, "~/input/topic_img_back_right", img_qos);

    // Localization topic
    sub_localization_.subscribe(this, "~/input/odometry", rclcpp::QoS{1});

    // Start synchronizer and add callback
    sync_ = std::make_shared<Sync>(
        MultiCameraApproxSync(10),
        sub_f_img_, sub_fr_img_, sub_fl_img_,
        sub_b_img_, sub_bl_img_, sub_br_img_,
        sub_localization_
    );

    sync_->registerCallback(
        [this] (auto f_img, auto fr_img, auto fl_img,
                auto b_img, auto bl_img, auto br_img,
                auto pose)
        {
            this->StreamPetrNode::callback(
                {f_img, fr_img, fl_img, b_img, bl_img, br_img},
                pose
            );
        }
    );

    return;
}

void StreamPetrNode::initModel()
{
    RCLCPP_INFO(get_logger(), "nvinfer: %d.%d.%d\n", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH);

    /* ---- Prepare needed params ---- */
    // Assign to a constant value as system doesn't have lidar
    this->lidar2ego <<
        1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12;

    this->model_cfg.img_width = this->img_width;
    this->model_cfg.img_height = this->img_height;
    this->model_cfg.cam2imgs = this->cam2imgs;
    this->model_cfg.ego2cams = this->ego2cams;
    this->model_cfg.lidar2ego = this->lidar2ego;

    /* ---- Create model ---- */
    this->model = std::make_unique<StreamPETR>(this->model_cfg);
}

void StreamPetrNode::cameraInfoCallback(int idx, CameraInfo::ConstSharedPtr msg)
{
    if (this->caminfo_received_[idx]) return; // already received;

    // Get the image size
    uint32_t img_W;
    uint32_t img_H;
    if (msg->roi.width == 0 && msg->roi.height == 0) {
        img_W = msg->width;
        img_H = msg->height;
    }
    else {
        img_W = msg->roi.width;
        img_H = msg->roi.height;
    }
    if (msg->binning_x > 0) {
        img_W /= msg->binning_x;
    }
    if (msg->binning_y > 0) {
        img_H /= msg->binning_y;
    }
    this->img_width[idx] = img_W;
    this->img_height[idx] = img_H;

    // Get intrinsic matrix
    Eigen::Matrix3fRM cam2img = getEigenCameraIntrinsics(msg);
    this->cam2imgs[idx] = cam2img;

    // Get extrinsic matrix
    Eigen::Matrix4fRM ego2cam;
    try {
        ego2cam = getEigenTransformMatrix(
            tf_buffer_->lookupTransform(msg->header.frame_id, "base_link", rclcpp::Time(0))
        );
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return;
    }
    this->ego2cams[idx] = ego2cam;

    // Finished, check if all cameras info have been read
    this->caminfo_received_[idx] = true;
    this->camera_info_received_flag_ = std::all_of(
        caminfo_received_.begin(),
        caminfo_received_.end(),
        [](bool i) { return i; }
    );
}

void StreamPetrNode::callback(
    std::array<Image::ConstSharedPtr, CAM_CHANNEL::NUM_CHANNEL> img_msg,
    Odometry::ConstSharedPtr odo_msg
)
{
    /* ---- Prepare Input ---- */
    StreamPETR::Input input_data;

    // Get image and latest timestamp
    input_data.rawImgs.reserve(CAM_CHANNEL::NUM_CHANNEL);
    for (const auto& msg: img_msg) {
        if (!msg) {
            throw std::runtime_error("Received invalid Image message!");
        }

        // Get image data as cv::Mat
        cv::Mat img;
        try {
            img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(logger_, "Image conversion failed: %s", e.what());
            throw std::runtime_error("Unsupported image encoding for conversion: " + msg->encoding);
        }

        input_data.rawImgs.push_back(img);

        // Get latest timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
        if (input_data.timestamp < timestamp) input_data.timestamp = timestamp;
    }

    // Get ego pose
    input_data.ego_pose = get_ego_pose_vector(odo_msg);

    /* ---- Inference ---- */
    StreamPETR::Output output_data = this->model->infer(input_data);

    /* ---- Publish ---- */
    std::vector<DetectedObject> raw_objects(output_data.objects.size());
    for (const auto& obj: output_data.objects) {
        raw_objects.push_back(bbox_to_ros_msg(obj));
    }

    DetectedObjects output_msg;
    output_msg.objects = raw_objects;
    output_msg.header.frame_id = "base_link";
    pub_objects_->publish(output_msg);
    return;
}

Eigen::Matrix4dRM StreamPetrNode::get_ego_pose_vector(Odometry::ConstSharedPtr odo) const
{
    const auto& orientation = odo->pose.pose.orientation;
    const auto& position = odo->pose.pose.position;

    // Convert quaternion to rotation matrix
    tf2::Quaternion quat(orientation.w, orientation.x, orientation.y, orientation.z);
    tf2::Matrix3x3 rotation;
    rotation.setRotation(latest_quat);

    // Final ego pose
    Eigen::Matrix4dRM egopose <<
        rotation[0][0], rotation[0][1], rotation[0][2], position.x,
        rotation[1][0], rotation[1][1], rotation[1][2], position.y,
        rotation[2][0], rotation[2][1], rotation[2][2], position.z,
        0.0, 0.0, 0.0, 1.0;

    return egopose;
}

}  // namespace tensorrt_stream_petr


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_stream_petr::StreamPetrNode)