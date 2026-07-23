#ifndef TOPIC_SFF_DEBUG_H
#define TOPIC_SFF_DEBUG_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/sff_debug_data.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicSFFDebugHandler : public TopicBase<ipc_helper::msg::SFFDebugData>
{
public:
    TopicSFFDebugHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/sff_debug"
    ) : TopicBase<ipc_helper::msg::SFFDebugData>(node_ptr, topic_name) {}
    ~TopicSFFDebugHandler() {}
};

#endif // TOPIC_SFF_DEBUG_H
