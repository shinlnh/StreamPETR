#ifndef CAN_SERVICE_IF_H
#define CAN_SERVICE_IF_H

#include "common.h"
#include <string.h>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <topic_control.h>
#include <topic_car_status.h>
#include <topic_imu.h>
#include <topic_odometer.h>


class CanServiceIF 
{
private:
    /*define struct to register rx data
    struct register_handler
    {
        char topic_name[50];
        void *arg;
        std::function<void(void *, const char *)> fucnt_handle;
    };
    */

    // Interface DDS
    rclcpp::Node::SharedPtr node_ptr_;
    std::thread executor_handler_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::shared_ptr<TopicControlHandler> topic_control_;
    std::shared_ptr<TopicCarStatusHandler> topic_car_status_;
    std::shared_ptr<TopicImuHandler> topic_imu_;
    std::shared_ptr<TopicOdometerHandler> topic_odometer_;
    // Interface CAN (Have not moved from CAN Service)

public:
    CanServiceIF(std::string vehicle_name = "hero0", std::string node_name = "can_service_node");
    ~CanServiceIF();
    
    // Carla Control Directly
    bv_err_return_t publishControls(ipc_helper::msg::Control control_msg);
    std::shared_ptr<TopicControlHandler>& getTopicControl() { return topic_control_; }
    std::shared_ptr<TopicCarStatusHandler>& getTopicCarStatusHandler() { return topic_car_status_; }
    std::shared_ptr<TopicImuHandler>& getTopicIMUHandler() { return topic_imu_; }
    std::shared_ptr<TopicOdometerHandler>& getTopicOdometerHandler() { return topic_odometer_; }
};
#endif
