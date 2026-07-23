#ifndef TOPIC_VISUALIZATION_H
#define TOPIC_VISUALIZATION_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/pipeline_results.hpp>

// System header
#include <functional>
#include <string>
#include <vector>
#include <array>

// User header
#include "topic_base.h"

class TopicVisualizationHandler : public TopicBase<ipc_helper::msg::PipelineResults>
{
public:
    TopicVisualizationHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/visualization"
    ) : TopicBase<ipc_helper::msg::PipelineResults>(node_ptr, topic_name) {}
    ~TopicVisualizationHandler() {}
};

#endif // TOPIC_VISUALIZATION_H