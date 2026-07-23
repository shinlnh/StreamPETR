// ROS2 header
#include <rclcpp/rclcpp.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_status.hpp>
#include <std_msgs/msg/bool.hpp>

// System header
#include <cmath>
#include <csignal>
#include <chrono>
#include <atomic>

// User header
#include "adas_service_if.h"

using std::placeholders::_1;

class CarlaControlNode : public rclcpp::Node
{
public:
    CarlaControlNode()
        : Node("adas_bridge_control_msg_node")
        , server_reverse_(false)
    {
        // Declare and get parameters
        this->declare_parameter("vehicle_name", "hero0");

        // Get parameter value
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();

        std::string pub_control_topic = "/carla/" + vehicle_name + "/vehicle_control_cmd";
        std::string pub_override_sig_topic = "/carla/" + vehicle_name + "/vehicle_control_manual_override";
        std::string sub_vehicle_status_topic = "/carla/" + vehicle_name + "/vehicle_status";

        pub_control_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(pub_control_topic, 1);
        pub_override_sig_ = this->create_publisher<std_msgs::msg::Bool>(pub_override_sig_topic, 1);
        
        // Subscribe to vehicle status to get server's reverse state
        // This is a workaround for Carla issue #7857: R->N/P shifts to D instead
        // See: https://github.com/carla-simulator/carla/issues/7857
        sub_vehicle_status_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
            sub_vehicle_status_topic,
            rclcpp::QoS(1).best_effort(),
            std::bind(&CarlaControlNode::cbVehicleStatus, this, std::placeholders::_1)
        );
        
        // Initialzie API for ADAS Service Interface
        adas_service_if_ = std::make_unique<AdasServiceIF>(vehicle_name, "");
        adas_service_if_->registerTopic<TopicControlHandler>(AdasServiceIF::TopicNameT::CONTROL);
        adas_subscriber_ = this->create_subscription<ipc_helper::msg::Control>(
            adas_service_if_->getTopicName(AdasServiceIF::TopicNameT::CONTROL),
            adas_service_if_->getQoS(AdasServiceIF::TopicNameT::CONTROL),
            std::bind(&CarlaControlNode::publishControlMessage, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Carla control node intialize successfully (with R->N/P workaround)");
    }

private:
    void cbVehicleStatus(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
    {
        // Store server's current reverse state
        // This is used as workaround for Carla issue #7857
        server_reverse_.store(msg->control.reverse);
    }

    void publishControlMessage(const ipc_helper::msg::Control::SharedPtr msg)
    {
        static carla_msgs::msg::CarlaEgoVehicleControl control_msg;
        control_msg.header.stamp = msg->header.stamp;
        control_msg.brake = msg->brake;
        control_msg.steer = msg->steer;
        control_msg.throttle = msg->throttle;
        control_msg.hand_brake = false;
        control_msg.manual_gear_shift = true;

        // Determine desired reverse state based on gear
        bool desired_reverse = (msg->gear == static_cast<int>(GEAR::R));
        
        // Convert automatic transmission Gear to Carla control
        switch (static_cast<GEAR>(msg->gear)) {
            case (GEAR::P):
                control_msg.gear = 0;
                control_msg.hand_brake = true;
                break;
            case (GEAR::R):
                control_msg.gear = -1;
                break;
            case (GEAR::N):
                control_msg.gear = 0;
                break;
            case (GEAR::D):
                control_msg.gear = 1;
                break;
            default:
                control_msg.gear = msg->gear;
                WARN("Received undefined gear!!! Skipping conversion!!!");
        }

        // WORKAROUND for Carla issue #7857:
        // When reverse flag changes in DefaultMovementComponent.cpp, Carla forces gear to -1 or 1,
        // ignoring the actual gear value (breaking N/P when switching from R).
        // Fix: Use server's reverse state to prevent the flag from "changing" in Carla's eyes,
        // then Carla will respect our gear value.
        // 
        // However, we still need to eventually sync reverse state. Strategy:
        // - If desired_reverse matches server state: use server state (no change = Carla respects gear)
        // - If desired_reverse differs: set it (Carla will force gear, but that's ok for R/D)
        //   For R->N/P transition, we detect and handle specially.
        
        bool current_server_reverse = server_reverse_.load();
        
        if (!desired_reverse && current_server_reverse && control_msg.gear == 0) {
            // Special case: R -> N/P transition (reverse true->false, gear=0)
            // This triggers Carla bug. Workaround: keep reverse=true for this message,
            // but with gear=0. Next message when server_reverse becomes false, it will work.
            // Actually, better approach: always use server reverse to avoid triggering the bug.
            control_msg.reverse = current_server_reverse;
            RCLCPP_DEBUG(this->get_logger(), 
                "R->N/P workaround: using server reverse=%d to avoid Carla bug #7857", 
                current_server_reverse);
        } else {
            control_msg.reverse = desired_reverse;
        }

        // Publish
        this->pub_control_->publish(control_msg);
        this->pub_override_sig_->publish(false);
        RCLCPP_DEBUG(this->get_logger(), "Publishing CARLA control: gear=%d, reverse=%d", 
            control_msg.gear, control_msg.reverse);
    }

    // ROS bridge
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_control_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_override_sig_;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_vehicle_status_;

    // Server state cache (for Carla bug workaround)
    std::atomic<bool> server_reverse_;

    // ADAS Service Interface
    std::unique_ptr<AdasServiceIF> adas_service_if_;
    rclcpp::Subscription<ipc_helper::msg::Control>::SharedPtr adas_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<CarlaControlNode> node = std::make_shared<CarlaControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
