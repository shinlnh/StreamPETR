#define _USE_MATH_DEFINES
// ROS2 header
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// System header
#include <cmath>
#include <csignal>

// User header
#include "adas_service_if.h"

// Convert a geometry_msgs::msg::Quaternion to roll, pitch, yaw (uint: radian)
void quaternionToRPY(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
}

class ImuProcessorNode : public rclcpp::Node
{
public:
    ImuProcessorNode()
    : rclcpp::Node("adas_bridge_imu_node")
    {
        // Declare and get parameters
        this->declare_parameter("vehicle_name", "hero0");
        
        // Get parameter value
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();

        // Create subscription
        std::string topic_name = "/carla/" + vehicle_name + "/imu";
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, 1,
            std::bind(&ImuProcessorNode::imuCallback, this, std::placeholders::_1));

        // Create ADAS Service Interface
        adas_service_if_ = std::make_unique<AdasServiceIF>(vehicle_name, "");
        adas_service_if_->registerTopic<TopicImuHandler>(AdasServiceIF::TopicNameT::IMU);
        adas_publisher_ = this->create_publisher<ipc_helper::msg::ImuParameters>(
            adas_service_if_->getTopicName(AdasServiceIF::TopicNameT::IMU), 
            adas_service_if_->getQoS(AdasServiceIF::TopicNameT::IMU)
        );

        RCLCPP_INFO(this->get_logger(), "IMU node initialize successfully");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        const geometry_msgs::msg::Quaternion& orientation = msg->orientation;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        quaternionToRPY(orientation, roll, pitch, yaw);

        const geometry_msgs::msg::Vector3& angular_velocity = msg->angular_velocity;
        const geometry_msgs::msg::Vector3& linear_acceleration = msg->linear_acceleration;

        ipc_helper::msg::ImuParameters imu_msg;
        imu_msg.header.stamp = msg->header.stamp;
        imu_msg.imu_orientation.x = orientation.x;
        imu_msg.imu_orientation.y = orientation.y;
        imu_msg.imu_orientation.z = orientation.z;
        imu_msg.imu_orientation.w = orientation.w;

        imu_msg.imu_angular_velocity.x = angular_velocity.x;
        imu_msg.imu_angular_velocity.y = angular_velocity.y;
        imu_msg.imu_angular_velocity.z = angular_velocity.z;

        imu_msg.imu_linear_acceleration.x = linear_acceleration.x;
        imu_msg.imu_linear_acceleration.y = linear_acceleration.y;
        imu_msg.imu_linear_acceleration.z = linear_acceleration.z;

        imu_msg.imu_roll_pitch_yaw.roll = roll;
        imu_msg.imu_roll_pitch_yaw.pitch = pitch;
        imu_msg.imu_roll_pitch_yaw.yaw = yaw;

        adas_publisher_->publish(imu_msg);
    }

    // ROS bridge
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

    // ADAS Service Interface
    std::unique_ptr<AdasServiceIF> adas_service_if_;
    rclcpp::Publisher<ipc_helper::msg::ImuParameters>::SharedPtr adas_publisher_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
    });
    std::shared_ptr<ImuProcessorNode> imu_node = std::make_shared<ImuProcessorNode>();
    rclcpp::spin(imu_node);
    rclcpp::shutdown();
    return 0;
}
