#ifndef TOPIC_ODOMETER_H
#define TOPIC_ODOMETER_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/odometry.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicOdometerHandler : public TopicBase<ipc_helper::msg::Odometry>
{
public:
    TopicOdometerHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/odometer"
    ) : TopicBase<ipc_helper::msg::Odometry>(node_ptr, topic_name) {}
    ~TopicOdometerHandler(){}
};

#endif // TOPIC_ODOMETER_H