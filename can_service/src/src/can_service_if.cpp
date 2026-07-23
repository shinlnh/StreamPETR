#include "can_service_if.h"


/**
 * @brief  constructer of CanService
 * @retval void
 */
CanServiceIF::CanServiceIF(std::string vehicle_name, std::string node_name)
{   
    if (node_name == "") {
        node_name = "can_service_node";
    }
    if (vehicle_name == "") {
        vehicle_name = "hero0";
    }

    node_ptr_ = std::make_shared<rclcpp::Node>(node_name);

    // Create topics handler
    topic_control_ = std::make_shared<TopicControlHandler>(
        node_ptr_,
        "/adas/" + vehicle_name + "/vehicle_control_cmd");
    topic_car_status_ = std::make_shared<TopicCarStatusHandler>(
        node_ptr_, 
        "/adas/" + vehicle_name + "/car_status");
    topic_imu_ = std::make_shared<TopicImuHandler>(
        node_ptr_, 
        "/adas/" + vehicle_name + "/imu");
    topic_odometer_ = std::make_shared<TopicOdometerHandler>(
        node_ptr_, 
        "/adas/" + vehicle_name + "/odometer");

    executor_.add_node(node_ptr_);
    executor_handler_ = std::thread(
        std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_)
    );
}

/**
 * @brief  destructer of CanService
 * @retval void
 */
CanServiceIF::~CanServiceIF()
{
    DEBUG("destructer %s", "CanServiceIF");
    if (executor_handler_.joinable()) {
        executor_handler_.join();
    }
}

/*......................................USER FUNCTION......................................*/
bv_err_return_t CanServiceIF::publishControls(ipc_helper::msg::Control control_msg)
{
    bv_err_return_t return_code = BV_RETURN_OK;

    DEBUG("Sending reverse %s to ADAS Service", control_msg.reverse ? "true" : "false");

    topic_control_->publish(control_msg);

    return return_code;
}
