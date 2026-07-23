#ifndef TOPIC_GNSS_H
#define TOPIC_GNSS_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/gnss_point.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicGnssHandler : public TopicBase<ipc_helper::msg::GnssPoint>
{
public:
    TopicGnssHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/gnss"
    ) : TopicBase<ipc_helper::msg::GnssPoint>(node_ptr, topic_name) {}
    ~TopicGnssHandler() {}
};

#endif // TOPIC_GNSS_H