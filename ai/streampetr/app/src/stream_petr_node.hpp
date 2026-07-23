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

#ifndef TENSORRT_STREAM_PETR__STREAM_PETR_NODE_HPP__
#define TENSORRT_STREAM_PETR__STREAM_PETR_NODE_HPP__

#include <image_transport/image_transport.hpp>
#include <memory>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "trt_engine.hpp"
#include "streampetr.hpp"
#include "camera_data_store.hpp"

namespace tensorrt_stream_petr
{
class StreamPetrNode : public rclcpp::Node
{
    using CAM_CHANNEL = StreamPETR::CAM_CHANNEL;

    using Odometry = nav_msgs::msg::Odometry;
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
    using DetectedObject = autoware_perception_msgs::msg::DetectedObject;

private:
    // Transform tree
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Variables for storing hardware configs
    std::array<uint32_t, CAM_CHANNEL::NUM_CHANNEL> img_width;
    std::array<uint32_t, CAM_CHANNEL::NUM_CHANNEL> img_height;
    std::array<Eigen::Matrix3fRM, CAM_CHANNEL::NUM_CHANNEL> cam2imgs;
    std::array<Eigen::Matrix4fRM, CAM_CHANNEL::NUM_CHANNEL> ego2cams;
    Eigen::Matrix4fRM lidar2ego;

    std::array<bool, CAM_CHANNEL::NUM_CHANNEL> caminfo_received_;
    bool camera_info_received_flag_ = false;

    // Timer for checking initialization
    rclcpp::TimerBase::SharedPtr initialization_check_timer_;

    /* ---- Subscribers ---- */
    // Subscribers of camera info for each camera, no need to synchronize
    std::array<rclcpp::Subscription<CameraInfo>::SharedPtr, CAM_CHANNEL::NUM_CHANNEL> sub_caminfo_;

    // Subscribers of images for each camera, synchronized
    std::array<message_filters::Subscriber<Image>, CAM_CHANNEL::NUM_CHANNEL> sub_img_;

    // Subscriber for localization, synchronized
    message_filters::Subscriber<Odometry> sub_localization_;

    // Synchronizer for image callbacks
    typedef message_filters::sync_policies::ApproximateTime<
        Image, Image, Image, Image, Image, Image, Odometry
    > MultiCameraApproxSync;

    typedef message_filters::Synchronizer<MultiCameraApproxSync> Sync;
    std::shared_ptr<Sync> sync_;

    /* ---- Publishers ---- */
    // Detected objects
    rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;

    /* ---- Model ---- */
    std::unique_ptr<StreamPETR> model;
    StreamPETR::Config model_cfg;

public:
    explicit StreamPetrNode(const rclcpp::NodeOptions &node_options);

private:
    void checkInitialization();
    void subscribeCameraInfo();
    void initPublishers();
    void initSubscribers();
    void initModel();
    void cameraInfoCallback(int idx, CameraInfo::ConstSharedPtr msg);
    void callback(
        std::array<Image::ConstSharedPtr, CAM_CHANNEL::NUM_CHANNEL> img_msg,
        Odometry::ConstSharedPtr odo_msg
    );
    Eigen::Matrix4dRM get_ego_pose_vector(Odometry::ConstSharedPtr odo) const;
};

} // namespace tensorrt_stream_petr

#endif // TENSORRT_STREAM_PETR__STREAM_PETR_NODE_HPP__
