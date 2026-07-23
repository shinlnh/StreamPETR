#include "adas_service_if.h"
#include <unistd.h>

int main(int argc, char **argv)
{
    bool is_publisher = false;
#ifdef PUBLISHER
    is_publisher = true;
    INFO("I am publisher");
#else
    INFO("I am subscriber");
#endif  // PUBLISHER
    rclcpp::init(argc, argv);
    AdasServiceIF adas_service_if;
    adas_service_if.registerTopic<TopicImuHandler>(AdasServiceIF::TopicNameT::IMU);
    auto topic_imu = adas_service_if.getTopicHandler<TopicImuHandler>(AdasServiceIF::TopicNameT::IMU);
#ifdef PUBLISHER
#else
    topic_imu->registerCallback([](auto msg){
        INFO("Hello from IMU callback");
    });
#endif  // PUBLISHER

    adas_service_if.run();

#ifdef PUBLISHER
    for (int i = 0; i < 10; ++i) {
        ipc_helper::msg::ImuParameters msg;
        topic_imu->publish(msg);
        sleep(1);
    }
#endif  // PUBLISHER

    while (rclcpp::ok());
    rclcpp::shutdown();
    return 0;
}