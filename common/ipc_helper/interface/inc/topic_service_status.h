#ifndef TOPIC_SERVICE_STATUS_H
#define TOPIC_SERVICE_STATUS_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/service_status.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicServiceStatusHandler : public TopicBase<ipc_helper::msg::ServiceStatus>
{
private:
    ipc_helper::msg::ServiceStatus cache_msg_;

public:
    TopicServiceStatusHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/service_status",
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(3)).reliable().durability_volatile()
    ) : TopicBase<ipc_helper::msg::ServiceStatus>(node_ptr, topic_name, qos)
    {
        initCacheMessage();
    }
    ~TopicServiceStatusHandler() {}

    void publish(const ipc_helper::msg::ServiceStatus &msg) override
    {
        cache_msg_.header.stamp = msg.header.stamp;
        cache_msg_.velocity = msg.velocity;
        cache_msg_.fps = msg.fps;
        cache_msg_.controller_theta = msg.controller_theta;
        cache_msg_.controller_error = msg.controller_error;
        cache_msg_.controller_tau = msg.controller_tau;
        cache_msg_.controller_angle = msg.controller_angle;
        cache_msg_.state_id = msg.state_id;
        cache_msg_.distance_captured = msg.distance_captured;

        publisher_->publish(msg);
    }

    // Helper function initialize
    void initCacheMessage()
    {
        if (node_ptr_ != nullptr) {
            rclcpp::Time stamp = node_ptr_->now();
            cache_msg_.header.stamp.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000);
            cache_msg_.header.stamp.nanosec = stamp.nanoseconds() % 1000000000;
        }
        cache_msg_.velocity = 0.0f;
        cache_msg_.fps = 0.0f;
        cache_msg_.controller_theta = 0.0f;
        cache_msg_.controller_error = 0.0f;
        cache_msg_.controller_tau = 0.0f;
        cache_msg_.controller_angle = 0.0f;
        cache_msg_.state_id = 0;
        cache_msg_.distance_captured = 0;
    }
};

#endif // TOPIC_SERVICE_STATUS_H