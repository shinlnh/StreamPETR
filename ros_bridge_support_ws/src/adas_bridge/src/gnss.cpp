#define _USE_MATH_DEFINES
// ROS2 header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// System header
#include <signal.h>
#include <cmath>

// User header
#include "adas_service_if.h"

#define a 6378137
#define b 6356752.31424518
#define f ((a - b) / a)
#define e (f * (2 - f))

std::vector<float> geodetic_to_ecef(float lat, float lon, float h)
{
    // (lat, lon) in WSG-84 degrees
    // h in meters
    float lamb = lat * M_PI / 180;
    float phi = lon * M_PI / 180;
    float s = sin(lamb);
    float N = a / sqrt(1 - e*e * s*s);

    float sin_lambda = sin(lamb);
    float cos_lambda = cos(lamb);
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    float X = (h + N) * cos_lambda * cos_phi;
    float Y = (h + N) * cos_lambda * sin_phi;
    float Z = (h + (1 - e*e) * N) * sin_lambda;

    return {X, Y, Z};
}

std::vector<float> ecef_to_enu(float x, float y, float z, float lat0, float lon0, float h0)
{
    float lamb = lat0 * M_PI / 180;
    float phi = lon0 * M_PI / 180;
    float s = sin(lamb);
    float N = a / sqrt(1 - e*e * s*s);

    float sin_lambda = sin(lamb);
    float cos_lambda = cos(lamb);
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    float x0 = (h0 + N) * cos_lambda * cos_phi;
    float y0 = (h0 + N) * cos_lambda * sin_phi;
    float z0 = (h0 + (1 - e*e) * N) * sin_lambda;

    float xd = x - x0;
    float yd = y - y0;
    float zd = z - z0;

    float xEast = -sin_phi * xd + cos_phi * yd;
    float yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    float zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    return {xEast, yNorth, zUp};
}

class GnssProcessorNode : public rclcpp::Node
{
public:
    GnssProcessorNode()
        : Node("adas_bridge_gnss_node"), ref_latitude(0.0f), ref_longtitude(0.0f), ref_altitude(0.0f), ref_flag(false)
    {
        // Declare and get parameters
        this->declare_parameter("vehicle_name", "hero0");
        
        // Get parameter value
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();

        // Create subscription
        std::string topic_name = "/carla/" + vehicle_name + "/gnss";
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            topic_name, 1,
            std::bind(&GnssProcessorNode::gnss_callback, this, std::placeholders::_1)
        );

        // Initialzie API for ADAS Service Interface
        adas_service_if_ = std::make_unique<AdasServiceIF>(vehicle_name, "");
        adas_service_if_->registerTopic<TopicGnssHandler>(AdasServiceIF::TopicNameT::GNSS);
        adas_publisher_ = this->create_publisher<ipc_helper::msg::GnssPoint>(
            adas_service_if_->getTopicName(AdasServiceIF::TopicNameT::GNSS),
            adas_service_if_->getQoS(AdasServiceIF::TopicNameT::GNSS)
        );

        RCLCPP_INFO(this->get_logger(), "GNSS node initialize successfully");
    }

private:
    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        static double timestamp_bench = rclcpp::Time(msg->header.stamp).seconds();
        RCLCPP_DEBUG(
            this->get_logger(), 
            "Duration call gnssCallback: %.4lf s", 
            rclcpp::Time(msg->header.stamp).seconds() - timestamp_bench
        );
        timestamp_bench = rclcpp::Time(msg->header.stamp).seconds();

        float latitude = msg->latitude;
        float longitude = msg->longitude;
        float altitude = msg->altitude;

        if (!ref_flag)
        {
            ref_latitude = latitude;
            ref_longtitude = longitude;
            ref_altitude = altitude;
            ref_flag = true;
        }

        std::vector<float> ecef = geodetic_to_ecef(latitude, longitude, altitude);
        std::vector<float> enu = ecef_to_enu(ecef[0], ecef[1], ecef[2], ref_latitude, ref_longtitude, ref_altitude);

        // Create messaege
        ipc_helper::msg::GnssPoint gnss_msg;
        gnss_msg.header.stamp = msg->header.stamp;
        gnss_msg.x = enu[0];
        gnss_msg.y = enu[1];
        gnss_msg.z = enu[2];
        gnss_msg.latitude = latitude;
        gnss_msg.longitude = longitude;
        gnss_msg.altitude = altitude;

        adas_publisher_->publish(gnss_msg);
    }

    // ROS bridge
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;

    // ADAS Service Interface
    std::unique_ptr<AdasServiceIF> adas_service_if_;
    rclcpp::Publisher<ipc_helper::msg::GnssPoint>::SharedPtr adas_publisher_;
    
    // Reference parameter
    float ref_latitude;
    float ref_longtitude;
    float ref_altitude;
    bool ref_flag;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
    });
    std::shared_ptr<GnssProcessorNode> gnss_node = std::make_shared<GnssProcessorNode>();
    rclcpp::spin(gnss_node);
    rclcpp::shutdown();
    return 0;
}
