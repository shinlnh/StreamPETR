#ifndef TOPIC_CARLA_API_H
#define TOPIC_CARLA_API_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <carla_msgs_custom/msg/vehicle_data.hpp>
#include <carla_msgs_custom/msg/vehicle_data_array.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicCarlaApiHandler : public TopicBase<carla_msgs_custom::msg::VehicleDataArray>
{
public:
    TopicCarlaApiHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/carla_api"
    ) : TopicBase<carla_msgs_custom::msg::VehicleDataArray>(node_ptr, topic_name) {}
    ~TopicCarlaApiHandler() {}
};

#endif // TOPIC_CARLA_API_H