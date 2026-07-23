#define _USE_MATH_DEFINES
// ROS2 header
#include <rclcpp/rclcpp.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <carla_msgs_custom/msg/wheel_sensor.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// User header
#include "adas_service_if.h"

class VehicleStatusNode : public rclcpp::Node {
public:
    VehicleStatusNode()
        : Node("adas_bridge_vehicle_status_node")
    {
        // Declare and get parameters
        this->declare_parameter("vehicle_name", "hero0");

        // Get parameter value
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();

        std::string topic_status = "/carla/" + vehicle_name + "/vehicle_status";
        std::string topic_wheel = "/carla/" + vehicle_name + "/wheel_sensor";

        // Initialize message_filters subscribers for exact time synchronization
        vehicle_status_sub_.subscribe(this, topic_status);
        steering_angle_sub_.subscribe(this, topic_wheel);

        // Use ExactTime policy for absolute timestamp synchronization
        // Both messages must have exactly the same timestamp from Carla
        typedef message_filters::sync_policies::ExactTime<
            carla_msgs::msg::CarlaEgoVehicleStatus,
            carla_msgs_custom::msg::WheelSensor
        > SyncPolicy;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), vehicle_status_sub_, steering_angle_sub_);

        sync_->registerCallback(
            std::bind(&VehicleStatusNode::syncCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Initialize API for ADAS Service Interface
        adas_service_if_ = std::make_unique<AdasServiceIF>(vehicle_name, "");
        adas_service_if_->registerTopic<TopicCarStatusHandler>(AdasServiceIF::TopicNameT::CAR_STATUS);
        adas_publisher_ = this->create_publisher<ipc_helper::msg::CarStatus>(
            adas_service_if_->getTopicName(AdasServiceIF::TopicNameT::CAR_STATUS),
            adas_service_if_->getQoS(AdasServiceIF::TopicNameT::CAR_STATUS)
        );

        RCLCPP_INFO(this->get_logger(), "Vehicle status node initialized with exact time synchronization");
    }

private:
    void syncCallback(
        const carla_msgs::msg::CarlaEgoVehicleStatus::ConstSharedPtr& vehicle_status_msg,
        const carla_msgs_custom::msg::WheelSensor::ConstSharedPtr& wheel_sensor_msg)
    {
        // Publish synchronized message to ADAS
        ipc_helper::msg::CarStatus car_status_msg;

        // Use timestamp from vehicle_status (exactly synchronized with steering_angle)
        car_status_msg.header.stamp = vehicle_status_msg->header.stamp;

        // Get steering angle from synchronized message
        car_status_msg.steering = wheel_sensor_msg->steering_angle;

        // Get wheels' position
        size_t num_wheel = wheel_sensor_msg->wheel_position.size();
        car_status_msg.wheel_position.resize(num_wheel);
        for (size_t i = 0; i < num_wheel; i++) {
            car_status_msg.wheel_position[i].x = wheel_sensor_msg->wheel_position[i].y * -1.0f;
            car_status_msg.wheel_position[i].y = wheel_sensor_msg->wheel_position[i].x;
            car_status_msg.wheel_position[i].z = wheel_sensor_msg->wheel_position[i].z;
        }

        // Get control parameters
        auto &current_control = vehicle_status_msg->control;
        car_status_msg.steer = current_control.steer;
        car_status_msg.throttle = current_control.throttle;
        car_status_msg.brake = current_control.brake;

        // Get current gear from current control
        if (current_control.gear < 0 && current_control.reverse == true) {
            car_status_msg.gear = static_cast<int>(GEAR::R);
        }
        else if (current_control.gear == 0) {
            if (current_control.hand_brake == true) {
                car_status_msg.gear = static_cast<int>(GEAR::P);
            }
            else {
                car_status_msg.gear = static_cast<int>(GEAR::N);
            }
        }
        else if (current_control.gear > 0) {
            car_status_msg.gear = static_cast<int>(GEAR::D);
        }

        adas_publisher_->publish(car_status_msg);
    }

    // Message filters subscribers for exact time synchronization
    message_filters::Subscriber<carla_msgs::msg::CarlaEgoVehicleStatus> vehicle_status_sub_;
    message_filters::Subscriber<carla_msgs_custom::msg::WheelSensor> steering_angle_sub_;

    // Synchronizer with ExactTime policy
    typedef message_filters::sync_policies::ExactTime<
        carla_msgs::msg::CarlaEgoVehicleStatus,
        carla_msgs_custom::msg::WheelSensor
    > SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // ADAS Service Interface
    std::unique_ptr<AdasServiceIF> adas_service_if_;
    rclcpp::Publisher<ipc_helper::msg::CarStatus>::SharedPtr adas_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
    });
    std::shared_ptr<VehicleStatusNode> vehicle_status_node = std::make_shared<VehicleStatusNode>();
    rclcpp::spin(vehicle_status_node);
    rclcpp::shutdown();
    return 0;
}
