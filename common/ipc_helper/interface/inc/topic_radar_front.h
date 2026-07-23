#ifndef TOPIC_RADAR_FRONT_H
#define TOPIC_RADAR_FRONT_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/radar.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicRadarFrontHandler : public TopicBase<ipc_helper::msg::Radar>
{
public:
    TopicRadarFrontHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/radar_front"
    ) : TopicBase<ipc_helper::msg::Radar>(node_ptr, topic_name) {}
    ~TopicRadarFrontHandler() {}
};

#endif // TOPIC_RADAR_FRONT_H