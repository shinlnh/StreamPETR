#ifndef TOPIC_IMU_H
#define TOPIC_IMU_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/imu_parameters.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicImuHandler : public TopicBase<ipc_helper::msg::ImuParameters>
{
public:
    TopicImuHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/imu"
    ) : TopicBase<ipc_helper::msg::ImuParameters>(node_ptr, topic_name) {}
    ~TopicImuHandler(){}
};

#endif // TOPIC_IMU_H