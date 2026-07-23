#ifndef ADAS_SERVICE_IF_H
#define ADAS_SERVICE_IF_H

// ROS header
#include <rclcpp/rclcpp.hpp>

// System header
#include <vector>
#include <thread>

// User header
#include "common.h"

// Topic handler header
#include "topic_base.h"
#include "topic_imu.h"
#include "topic_odometer.h"
#include "topic_gnss.h"
#include "topic_control.h"
#include "topic_adas_control.h"
#include "topic_service_status.h"
#include "topic_service_command.h"
#include "topic_car_status.h"
#include "topic_radar_front.h"
#include "topic_carla_api.h"
#include "topic_visualization.h"
#include "topic_sff_debug.h"

#ifndef ADAS_BRIDGE_BUILD
// Util headers
#include "msg_utils.h"
#endif

class AdasServiceIF
{
private:
    rclcpp::Node::SharedPtr node_ptr_;
    std::vector<std::shared_ptr<TopicInterface>> topic_list_;
    std::thread executor_handler_;
    rclcpp::executors::MultiThreadedExecutor executor_;

    std::string vehicle_name_;
    const std::vector<std::string> topic_name_str_ = {
        "imu",
        "odometer",
        "gnss",
        "radar_front",
        "vehicle_control_cmd",
        "vehicle_adas_control_cmd",
        "service_status",
        "service_command",
        "visualization",
        "car_status",
        "carla_api",
        "sff_debug"
    };
public:
    enum class TopicNameT : int {
        IMU = 0,
        ODOMETER,
        GNSS,
        RADAR,
        CONTROL,
        ADAS_CONTROL,
        SERVICE_STATUS,
        SERVICE_COMMAND,
        VISUALIZATION,
        CAR_STATUS,
        CARLA_API,
        SFF_DEBUG,
        COUNT
    };

    AdasServiceIF(std::string vehicle_name = "hero0", std::string node_name = "adas");
    ~AdasServiceIF();
    
    template <typename TopicType>
    void registerTopic(const TopicNameT &topic_type);
    template <typename TopicType>
    std::shared_ptr<TopicType> getTopicHandler(const TopicNameT &topic_type);
    void run();

    /* Specific method for ROS interface */
    std::string getTopicName(const TopicNameT &topic_type);
    rclcpp::QoS getQoS(const TopicNameT &topic_type);
};


/**
 * @brief Register the topic which use to communicate 
 * with other components 
 */
template <typename TopicType>
void AdasServiceIF::registerTopic(const TopicNameT &topic_name_id)
{
    // If topic not listed in Enum, skip
    if (topic_name_id >= TopicNameT::COUNT) {
        return;
    }

    // Create topic
    int topic_index = static_cast<int>(topic_name_id);
    auto topic_name = getTopicName(topic_name_id);
    topic_list_[topic_index] = std::make_shared<TopicType>(node_ptr_, topic_name);
}

/**
 * @brief Get topic 
 */
template <typename TopicType>
std::shared_ptr<TopicType> AdasServiceIF::getTopicHandler(const TopicNameT &topic_name_id)
{
    auto handler = topic_list_.at(static_cast<int>(topic_name_id));
    return std::dynamic_pointer_cast<TopicType>(handler);
}

#endif  // ADAS_SERVICE_IF