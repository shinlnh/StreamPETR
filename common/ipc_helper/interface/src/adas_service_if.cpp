#include "adas_service_if.h"

/**
 * @brief Default constructor
 */
AdasServiceIF::AdasServiceIF(std::string vehicle_name, std::string node_name) : node_ptr_(nullptr), vehicle_name_(vehicle_name)
{
    if (node_name != "") {
        node_ptr_ = std::make_shared<rclcpp::Node>(node_name);
    }
    // Declare topic_list_ and assign nullptr to each topic
    int topic_size = static_cast<int>(TopicNameT::COUNT);
    topic_list_.resize(topic_size);
    for (int i = 0; i < topic_size; ++i) {
        topic_list_[i] = nullptr;
    }
}

/**
 * @brief Destructor
 */
AdasServiceIF::~AdasServiceIF() 
{
    executor_.cancel();
    if (executor_handler_.joinable()) {
        executor_handler_.join();
    }
}

/**
 * @brief Run all topic handler
 */
void AdasServiceIF::run()
{
    if (node_ptr_ == nullptr) {
        return;
    }
    executor_.add_node(node_ptr_);
    executor_handler_ = std::thread([this](){
        this->executor_.spin();
    });
}

/**
 * @brief Get the name that matches each topic type
 * @param topic_name_id
 */
std::string AdasServiceIF::getTopicName(const TopicNameT &topic_name_id)
{
    int topic_index = static_cast<int>(topic_name_id);
    auto topic_name = topic_name_str_.at(topic_index);
    return "/adas/" + vehicle_name_ + "/" + topic_name;
}


/**
 * @brief Get QoS of specific topic
 */
rclcpp::QoS AdasServiceIF::getQoS(const TopicNameT &topic_name_id)
{
    return topic_list_.at(static_cast<int>(topic_name_id))->getQoS();
}