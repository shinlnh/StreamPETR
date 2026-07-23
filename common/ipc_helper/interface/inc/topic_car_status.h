#ifndef TOPIC_CARLA_STATUS_H
#define TOPIC_CARLA_STATUS_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/car_status.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicCarStatusHandler : public TopicBase<ipc_helper::msg::CarStatus>
{
public:
    TopicCarStatusHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/car_status"
    ) : TopicBase<ipc_helper::msg::CarStatus>(node_ptr, topic_name) {}
    ~TopicCarStatusHandler() {}
};

#endif // TOPIC_CARLA_STATUS_H