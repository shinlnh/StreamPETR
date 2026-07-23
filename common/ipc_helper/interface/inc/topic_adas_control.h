#ifndef TOPIC_ADAS_CONTROL_H
#define TOPIC_ADAS_CONTROL_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/adas_control.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicAdasControlHandler : public TopicBase<ipc_helper::msg::AdasControl>
{
private:
    ipc_helper::msg::AdasControl cache_msg_;

public:
    TopicAdasControlHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/vehicle_adas_control_cmd"
    ) : TopicBase<ipc_helper::msg::AdasControl>(node_ptr, topic_name)
    {
        initCacheMessage();
    }
    ~TopicAdasControlHandler() {}

    void publish(const ipc_helper::msg::AdasControl &msg) override
    {
        cache_msg_.header.stamp = msg.header.stamp;
        cache_msg_.throttle = msg.throttle;
        cache_msg_.brake = msg.brake;
        cache_msg_.steer = msg.steer;

        publisher_->publish(msg);
    }
    
    // Helper function when changing one field
    void publishThrottle(const float &throttle)
    {
        cache_msg_.throttle = throttle;
        publisher_->publish(cache_msg_);
    }
    void publishBrake(const float &brake)
    {
        cache_msg_.brake = brake;
        publisher_->publish(cache_msg_);
    }
    void publishSteer(const float &steer)
    {
        cache_msg_.steer = steer;
        publisher_->publish(cache_msg_);
    }

    // Helper function initialize
    void initCacheMessage()
    {
        if (node_ptr_ != nullptr) {
            rclcpp::Time stamp = node_ptr_->now();
            cache_msg_.header.stamp.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000);
            cache_msg_.header.stamp.nanosec = stamp.nanoseconds() % 1000000000;
        }
        cache_msg_.throttle = 0.0f;
        cache_msg_.brake = 0.0f;
        cache_msg_.steer = 0.0f;
    }
};

#endif // TOPIC_ADAS_CONTROL_H