#ifndef TOPIC_BASE_H
#define TOPIC_BASE_H

#include <rclcpp/rclcpp.hpp>

#include <functional>

class TopicInterface
{
public:
    virtual ~TopicInterface() = default;

    virtual rclcpp::QoS getQoS() = 0;
};

template <typename MessageTypeName>
class TopicBase : public TopicInterface
{
protected:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::QoS qos_;
    typename rclcpp::Publisher<MessageTypeName>::SharedPtr publisher_;
    typename rclcpp::Subscription<MessageTypeName>::SharedPtr subscriber_;
    std::string topic_name_;

    void createPubSub()
    {
        if (node_ptr_ == nullptr) {
            return;
        }
        publisher_ = node_ptr_->create_publisher<MessageTypeName>(topic_name_, qos_);
        subscriber_ = node_ptr_->create_subscription<MessageTypeName>(
            topic_name_, qos_, 
            [](const std::shared_ptr<MessageTypeName> msg){
                (void)msg;  // avoid warning unused parameter
            }
        );
    }
public:
    TopicBase() : node_ptr_(nullptr), qos_(rclcpp::KeepLast(1)), publisher_(nullptr), subscriber_(nullptr), topic_name_("") {}
    TopicBase(rclcpp::Node::SharedPtr node_ptr, std::string topic_name) : node_ptr_(node_ptr), qos_(rclcpp::KeepLast(1)), topic_name_(std::move(topic_name)) 
    {
        /**
         * Default QoS:
            * History: Keep last
            * Depth: 1
            * Reliability: Best effort
            * Durability: Volatile
            * Deadline: Default
            * Lifespan: Default
            * Liveliness: Automatic
            * Lease Duration: Default
         * */ 
        /**
         * Note:
            * For each of the policies that is a duration, 
            * there also exists a “default” option that means 
            * the duration is unspecified, which the underlying 
            * middleware will usually interpret as an infinitely 
            * long duration.
         */
        qos_.best_effort().durability_volatile();

        // Create publisher and subscriber
        createPubSub();
    }
    TopicBase(rclcpp::Node::SharedPtr node_ptr, std::string topic_name, rclcpp::QoS qos) : 
              node_ptr_(node_ptr), qos_(qos), topic_name_(std::move(topic_name))
    {
        createPubSub();
    }
    virtual ~TopicBase() {}

    virtual void registerCallback(std::function<void (const std::shared_ptr<MessageTypeName>)> callback)
    {
        if (subscriber_ == nullptr) {
            return;
        }
        subscriber_ = node_ptr_->create_subscription<MessageTypeName>(topic_name_, qos_, callback);
    }
    virtual void publish(const MessageTypeName &msg)
    {
        if (publisher_ == nullptr) {
            return;
        }
        publisher_->publish(msg);
    }

    rclcpp::QoS getQoS() override { return qos_; }
};

#endif  // TOPIC_BASE_H