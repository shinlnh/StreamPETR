#define _USE_MATH_DEFINES
// ROS2 header
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

// System header
#include <cmath>
#include <csignal>
#include <random>

// User header
#include "adas_service_if.h"

class OdometerProcessorNode : public rclcpp::Node
{
public:
    OdometerProcessorNode()
    : rclcpp::Node("adas_bridge_odometer_node")
    , generator(this->rd())
    , linear_vel_x_dist(linear_vel_x_mean, linear_vel_x_stddev)
    , linear_vel_y_dist(linear_vel_y_mean, linear_vel_y_stddev)
    , linear_vel_z_dist(linear_vel_z_mean, linear_vel_z_stddev)
    , angular_vel_x_dist(angular_vel_x_mean, angular_vel_x_stddev)
    , angular_vel_y_dist(angular_vel_y_mean, angular_vel_y_stddev)
    , angular_vel_z_dist(angular_vel_z_mean, angular_vel_z_stddev)
    {
        // Declare and get parameters
        this->declare_parameter("vehicle_name", "hero0");
        
        // Get parameter value
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();

        // Create subscription
        std::string topic_name = "/carla/" + vehicle_name + "/odometry";
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name, 1,
            std::bind(&OdometerProcessorNode::odometerCallback, this, std::placeholders::_1));

        // Create ADAS Service Interface
        adas_service_if_ = std::make_unique<AdasServiceIF>(vehicle_name, "");
        adas_service_if_->registerTopic<TopicOdometerHandler>(AdasServiceIF::TopicNameT::ODOMETER);
        adas_publisher_ = this->create_publisher<ipc_helper::msg::Odometry>(
            adas_service_if_->getTopicName(AdasServiceIF::TopicNameT::ODOMETER), 
            adas_service_if_->getQoS(AdasServiceIF::TopicNameT::ODOMETER)
        );

        RCLCPP_INFO(this->get_logger(), "Odometer node initialize successfully");
    }

private:
    void odometerCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const geometry_msgs::msg::Vector3& linear_velocity = msg->twist.twist.linear;
        const geometry_msgs::msg::Vector3& angular_velocity = msg->twist.twist.angular;

        // Generate gaussian noise
        float linear_vel_x_error = linear_vel_x_dist(generator);
        float linear_vel_y_error = linear_vel_y_dist(generator);
        float linear_vel_z_error = linear_vel_z_dist(generator);
        float angular_vel_x_error = angular_vel_x_dist(generator);
        float angular_vel_y_error = angular_vel_y_dist(generator);
        float angular_vel_z_error = angular_vel_z_dist(generator);

        // Add noise to message and publish
        ipc_helper::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;

        // Add Gaussian noise to velocity measurements
        odom_msg.linear_velocity.x = -linear_velocity.y + linear_vel_x_error;
        odom_msg.linear_velocity.y = linear_velocity.x + linear_vel_y_error;
        odom_msg.linear_velocity.z = linear_velocity.z + linear_vel_z_error;

        odom_msg.angular_velocity.x = -angular_velocity.y + angular_vel_x_error;
        odom_msg.angular_velocity.y = angular_velocity.x + angular_vel_y_error;
        odom_msg.angular_velocity.z = angular_velocity.z + angular_vel_z_error;

        // NOTE: Set twist covariance for sensor fusion packages (localization, SLAM, etc.)
        // Covariance matrix should match the stddev used for noise generation
        // Format: 6x6 matrix stored as 36-element array [row-major order]
        // Order: [vx, vy, vz, vroll, vpitch, vyaw]
        // Currently set to 0 - UPDATE these values when integrating with fusion packages!
        //
        // EXAMPLE for realistic odometry sensor:
        // odom_msg.twist_covariance[0]  = 0.05 * 0.05;  // vx variance (m/s)^2
        // odom_msg.twist_covariance[7]  = 0.05 * 0.05;  // vy variance (m/s)^2
        // odom_msg.twist_covariance[14] = 0.02 * 0.02;  // vz variance (m/s)^2
        // odom_msg.twist_covariance[21] = 0.01 * 0.01;  // vroll variance (rad/s)^2
        // odom_msg.twist_covariance[28] = 0.01 * 0.01;  // vpitch variance (rad/s)^2
        // odom_msg.twist_covariance[35] = 0.02 * 0.02;  // vyaw variance (rad/s)^2
        
        // Off-diagonal elements are 0 (assuming no correlation between axes)
        // If sensor has correlated noise, set appropriate off-diagonal values

        adas_publisher_->publish(odom_msg);
    }

    // ROS bridge
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    // ADAS Service Interface
    std::unique_ptr<AdasServiceIF> adas_service_if_;
    rclcpp::Publisher<ipc_helper::msg::Odometry>::SharedPtr adas_publisher_;

    // Gaussian noise generator
    std::random_device rd;
    std::mt19937 generator;
    std::normal_distribution<float> linear_vel_x_dist;
    std::normal_distribution<float> linear_vel_y_dist;
    std::normal_distribution<float> linear_vel_z_dist;
    std::normal_distribution<float> angular_vel_x_dist;
    std::normal_distribution<float> angular_vel_y_dist;
    std::normal_distribution<float> angular_vel_z_dist;
    
    // NOTE: Noise parameters - Currently set to 0 (perfect sensor simulation)
    // UPDATE these values to add realistic sensor noise:
    // - Typical odometry linear velocity noise: 0.01-0.1 m/s (stddev)
    // - Typical odometry angular velocity noise: 0.01-0.05 rad/s (stddev)
    // 
    // IMPORTANT: When changing stddev values, also update twist_covariance in callback!
    // Covariance diagonal elements should be: variance = stddev^2
    static constexpr float linear_vel_x_mean = 0.0f, linear_vel_x_stddev = 0.0f;    // m/s
    static constexpr float linear_vel_y_mean = 0.0f, linear_vel_y_stddev = 0.0f;    // m/s
    static constexpr float linear_vel_z_mean = 0.0f, linear_vel_z_stddev = 0.0f;    // m/s
    static constexpr float angular_vel_x_mean = 0.0f, angular_vel_x_stddev = 0.0f;  // rad/s
    static constexpr float angular_vel_y_mean = 0.0f, angular_vel_y_stddev = 0.0f;  // rad/s
    static constexpr float angular_vel_z_mean = 0.0f, angular_vel_z_stddev = 0.0f;  // rad/s
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
    });
    std::shared_ptr<OdometerProcessorNode> odometer_node = std::make_shared<OdometerProcessorNode>();
    rclcpp::spin(odometer_node);
    rclcpp::shutdown();
    return 0;
}