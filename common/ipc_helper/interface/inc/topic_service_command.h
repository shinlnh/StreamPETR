#ifndef TOPIC_SERVICE_COMMAND_H
#define TOPIC_SERVICE_COMMAND_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// Message header
#include <ipc_helper/msg/service_command.hpp>

// System header
#include <functional>
#include <string>

// User header
#include "topic_base.h"

class TopicServiceCommandHandler : public TopicBase<ipc_helper::msg::ServiceCommand>
{
private:
    ipc_helper::msg::ServiceCommand cache_msg_;

public:
    enum SignalFlagID : uint8_t {  // Bit ID for signal flags
        CAPTURE = 0,
        CONTROL_LKS,
        CONTROL_ACC,
        CONTROL_AEB,
        NUM_SIGNAL_FLAG
    };

public:
    TopicServiceCommandHandler(
        rclcpp::Node::SharedPtr node_ptr,
        std::string topic_name = "/service_command",
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(3)).reliable().durability_volatile()
    ) : TopicBase<ipc_helper::msg::ServiceCommand>(node_ptr, topic_name, qos)
    {
        initCacheMessage();
    }
    ~TopicServiceCommandHandler() {}

    void publish(const ipc_helper::msg::ServiceCommand &msg) override
    {
        cache_msg_.header.stamp = msg.header.stamp;
        cache_msg_.capture_enabled = msg.capture_enabled;
        cache_msg_.control_lks_enabled = msg.control_lks_enabled;
        cache_msg_.control_acc_enabled = msg.control_acc_enabled;
        cache_msg_.control_aeb_enabled = msg.control_aeb_enabled;
        cache_msg_.velocity_ref = msg.velocity_ref;
        cache_msg_.distance_ref = msg.distance_ref;
        cache_msg_.flag_modified = ~0U;

        publisher_->publish(msg);
        cache_msg_.flag_modified = 0U; // Reset after sending
    }
    
    // Helper function when changing one field
    void publishCaptureEnabled(bool state)
    {
        cache_msg_.capture_enabled = state;
        cache_msg_.flag_modified |= 1U << CAPTURE;

        publisher_->publish(cache_msg_);
        cache_msg_.flag_modified = 0U; // Reset after sending
    }
    void publishControlLksEnabled(bool state)
    {
        cache_msg_.control_lks_enabled = state;
        cache_msg_.flag_modified |= 1U << CONTROL_LKS;
        
        publisher_->publish(cache_msg_);
        cache_msg_.flag_modified = 0U; // Reset after sending
    }
    void publishControlAccEnabled(bool state)
    {
        cache_msg_.control_acc_enabled = state;
        cache_msg_.flag_modified = 1U << CONTROL_ACC;
        
        publisher_->publish(cache_msg_);
        cache_msg_.flag_modified = 0U; // Reset after sending
    }
    void publishControlAebEnabled(bool state)
    {
        cache_msg_.control_aeb_enabled = state;
        cache_msg_.flag_modified = 1U << CONTROL_AEB;
        
        publisher_->publish(cache_msg_);
        cache_msg_.flag_modified = 0U; // Reset after sending
    }
    void publishAebTakeover()
    {
        cache_msg_.aeb_takeover = true;
        publisher_->publish(cache_msg_);
        cache_msg_.aeb_takeover = false;  // Reset after sending
    }
    void publishVelocityRef(float velocity)
    {
        cache_msg_.velocity_ref = velocity;
        publisher_->publish(cache_msg_);
    }
    void publishDistanceRef(float distance)
    {
        cache_msg_.distance_ref = distance;
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
        cache_msg_.capture_enabled = false;
        cache_msg_.control_lks_enabled = false;
        cache_msg_.control_acc_enabled = false;
        cache_msg_.control_aeb_enabled = true;
        cache_msg_.aeb_takeover = false;
        cache_msg_.velocity_ref = 0.0f;
        cache_msg_.distance_ref = 0.0f;
        cache_msg_.flag_modified = 0U;
    }
};

#endif // TOPIC_SERVICE_COMMAND_H