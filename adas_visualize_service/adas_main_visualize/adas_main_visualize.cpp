#include "adas_main_visualize.h"

#include <string>
#include "ReceiverRtsp.hpp"
#include "rtsp_settings.h"
#include "settings.h"


AdasMainVisualize::AdasMainVisualize()
{
    // Variable Initialization
    this->carla_speed = 0.0;

    // Detached specific setup
    /* New ADAS Service Interface */
    using TopicNameT = AdasServiceIF::TopicNameT;
#ifndef ENABLE_CAN
    interface.registerTopic<TopicControlHandler>(TopicNameT::CONTROL);
#endif
    interface.registerTopic<TopicAdasControlHandler>(TopicNameT::ADAS_CONTROL);
    interface.registerTopic<TopicServiceStatusHandler>(TopicNameT::SERVICE_STATUS);
    interface.registerTopic<TopicServiceCommandHandler>(TopicNameT::SERVICE_COMMAND);
    interface.registerTopic<TopicVisualizationHandler>(TopicNameT::VISUALIZATION);
    interface.registerTopic<TopicCarStatusHandler>(TopicNameT::CAR_STATUS);
    interface.registerTopic<TopicSFFDebugHandler>(TopicNameT::SFF_DEBUG);

    auto topic_adas_control =
        interface.getTopicHandler<TopicAdasControlHandler>(TopicNameT::ADAS_CONTROL);
    auto topic_service_status =
        interface.getTopicHandler<TopicServiceStatusHandler>(TopicNameT::SERVICE_STATUS);
    auto topic_visualization =
        interface.getTopicHandler<TopicVisualizationHandler>(TopicNameT::VISUALIZATION);
    auto topic_car_status =
        interface.getTopicHandler<TopicCarStatusHandler>(TopicNameT::CAR_STATUS);

    topic_adas_control->registerCallback([this](auto msg) { cbAdasControlIF(msg); });
    topic_service_status->registerCallback([this](auto msg) { cbServiceStatusIF(msg); });
    topic_visualization->registerCallback([this](auto msg) { cbVisualizationIF(msg); });
    topic_car_status->registerCallback([this](auto msg) { cbCarStatusIF(msg); });

    auto topic_sff_debug =
        interface.getTopicHandler<TopicSFFDebugHandler>(TopicNameT::SFF_DEBUG);
    topic_sff_debug->registerCallback([this](auto msg) { cbSFFDebugIF(msg); });

    interface.run();
}

AdasMainVisualize::~AdasMainVisualize() {
    udp_receiving_handler->join();
    delete udp_receiving_handler;
}

void AdasMainVisualize::setDstImage(cv::Mat newMat)
{
    boost::unique_lock<boost::mutex> lock(dst_image_mutex);
    dst_image = newMat;
}

cv::Mat AdasMainVisualize::getDstImage()
{
    boost::unique_lock<boost::mutex> lock(dst_image_mutex);
    return dst_image;
}

cv::Mat AdasMainVisualize::getGrapCam(graphType type)
{
    if (!is_stream)
    {
        DEBUG("No image");
        return cv::Mat(RTSP_FRAME_HEIGHT, RTSP_FRAME_WIDTH, CV_8UC3, UI_BACKGROUND_WIDGET_COLOR);
    }
    cv::Mat image;
    switch (type) {
        case CAM:
            image = getDstImage();
            break;
    }
    return image;
}

void AdasMainVisualize::TriggerSignal(bool is_start)
{
    auto topic_service_command =
        interface.getTopicHandler<TopicServiceCommandHandler>(AdasServiceIF::TopicNameT::SERVICE_COMMAND);

    is_stream = is_start;
    topic_service_command->publishCaptureEnabled(is_start);
}

void AdasMainVisualize::handleConnection(std::string server_ip)
{
    // Must create this thread because the MainWindow's updateGUI doesn't spin fast enough.
    TriggerSignal(true); // Service be run to get correct behavior.

    udp_receiving_handler = new boost::thread([this, server_ip]() {
        graphicsReceivingThread(server_ip);
    });
    udp_receiving_handler->detach();
}

void AdasMainVisualize::graphicsReceivingThread(std::string server_ip)
{
    ReceiverRtsp image_stream(RTSP_FRAME_WIDTH, RTSP_FRAME_HEIGHT, server_ip);
    while (image_stream.allow_grabbing) {
        // This is where we will grab the main window and the top down window.
        cv::Mat frame = image_stream.getNextFrame();
        if (!frame.empty())
        {
            setDstImage(frame);
            // cv::Rect roi1(0, 0, this->width, this->height);
        }
    }
}

void AdasMainVisualize::setValue(boolObjects boolEnum, bool value)
{
#ifdef ENABLE_CAN
#else
    controlMsgAttributeType type = ATTRIBUTE_DEFAULT;
    auto topic_service_command =
        interface.getTopicHandler<TopicServiceCommandHandler>(AdasServiceIF::TopicNameT::SERVICE_COMMAND);
    switch (boolEnum)
    {
    case STATUS_BTN_LANE_KEEPING_SYSTEM:
        topic_service_command->publishControlLksEnabled(value);
        break;
    case STATUS_BTN_ADAPTIVE_CRUISE_CONTROL:
        topic_service_command->publishControlAccEnabled(value);
        break;
    case STATUS_BTN_AUTO_EMERGENCY_BRAKING:
        topic_service_command->publishControlAebEnabled(value);
        break;
    default:
        break;
    }
    this->handleControlMsg(type, value);
#endif
}

void AdasMainVisualize::setValue(intObjects intEnum, int value)
{
#ifdef ENABLE_CAN
#else
    controlMsgAttributeType type = ATTRIBUTE_DEFAULT;
    float value_converted = value;
    switch (intEnum)
    {
    case VALUE_SLIDER_STEER:
        type = ATTRIBUTE_STEER;
        value_converted = value_converted / 100.0;
        break;
    case VALUE_SLIDER_BRAKE:
        type = ATTRIBUTE_BRAKE;
        value_converted = value_converted / 100.0;
        break;
    case VALUE_SLIDER_THROTTLE:
        type = ATTRIBUTE_THROTTLE;
        value_converted = value_converted / 100.0;
        break;
    default:
        break;
    }
    this->handleControlMsg(type, value_converted);
#endif
}

int AdasMainVisualize::getStateID()
{
    return this->stateID;
}

float AdasMainVisualize::getFps()
{
    return this->fps;
}

float AdasMainVisualize::getSpeed()
{
    return this->carla_speed;
}

float AdasMainVisualize::getRefVel()
{
    return this->reference_velocity;
}

void AdasMainVisualize::setRefVel(float newRefVel)
{
#ifdef ENABLE_CAN
#else
    auto topic_service_command =
        interface.getTopicHandler<TopicServiceCommandHandler>(AdasServiceIF::TopicNameT::SERVICE_COMMAND);
    topic_service_command->publishVelocityRef(newRefVel);
#endif
}

float AdasMainVisualize::getRefDist()
{
    return this->reference_distance;
}

void AdasMainVisualize::setRefDistance(float newRefDistance)
{
#ifdef ENABLE_CAN
#else
    auto topic_service_command =
        interface.getTopicHandler<TopicServiceCommandHandler>(AdasServiceIF::TopicNameT::SERVICE_COMMAND);
    topic_service_command->publishDistanceRef(newRefDistance);
#endif
}

float AdasMainVisualize::getDistanceCaptured()
{
    return this->distance_captured_from_service;
}

bool AdasMainVisualize::getDisableDriveAssist()
{
    return this->disable_drive_assist;
}

bool AdasMainVisualize::getAEBState()
{
    return this->aeb_state;
}

void AdasMainVisualize::sendAEBTakeover()
{
#ifdef ENABLE_CAN
#else
    auto topic_service_command =
        interface.getTopicHandler<TopicServiceCommandHandler>(AdasServiceIF::TopicNameT::SERVICE_COMMAND);
    topic_service_command->publishAebTakeover();
#endif
}

void AdasMainVisualize::setGear(GEAR gear)
{
    this->current_gear = gear;
}

GEAR AdasMainVisualize::getGear()
{
    return this->current_gear;
}

void AdasMainVisualize::doSteer(float steering_value)
{
#ifdef ENABLE_CAN
#else
    auto topic_manual_control =
        interface.getTopicHandler<TopicControlHandler>(AdasServiceIF::TopicNameT::CONTROL);
    topic_manual_control->publishSteer(steering_value);
#endif
}

void AdasMainVisualize::doThrottle(float throttle_value)
{
#ifdef ENABLE_CAN
#else
    auto topic_manual_control =
        interface.getTopicHandler<TopicControlHandler>(AdasServiceIF::TopicNameT::CONTROL);
    topic_manual_control->publishThrottle(throttle_value);
#endif
}

void AdasMainVisualize::doBrake(float brake_value)
{
#ifdef ENABLE_CAN
#else
    auto topic_manual_control =
        interface.getTopicHandler<TopicControlHandler>(AdasServiceIF::TopicNameT::CONTROL);
    topic_manual_control->publishBrake(brake_value);
#endif
}

void AdasMainVisualize::doGear(int gear)
{
#ifdef ENABLE_CAN
#else
    auto topic_manual_control =
        interface.getTopicHandler<TopicControlHandler>(AdasServiceIF::TopicNameT::CONTROL);
    topic_manual_control->publishGear(gear);
#endif
}

void AdasMainVisualize::cbAdasControlIF(const ipc_helper::msg::AdasControl::SharedPtr msg)
{
#ifdef ENABLE_CAN
#else
    int mode_id = this->getStateID();
    if (mode_id == LKS_STATE_ID || mode_id == HWC_STATE_ID || mode_id == TJC_STATE_ID) {
        this->doSteer(msg->steer);
    }
    if (mode_id == ACC_STATE_ID || mode_id == TJA_STATE_ID ||
        mode_id == HWC_STATE_ID || mode_id == TJC_STATE_ID ||
        mode_id == BRAKE_STATE_ID)
    {
        this->doThrottle(msg->throttle);
        this->doBrake(msg->brake);
    }
#endif
}

void AdasMainVisualize::cbServiceStatusIF(const ipc_helper::msg::ServiceStatus::SharedPtr msg)
{
    this->carla_speed = msg->velocity;
    this->fps = msg->fps;
    // this->theta = msg->controller_theta;
    // this->error = msg->controller_error;
    // this->tau = msg->controller_tau;
    // this->angle = msg->controller_angle;
    this->reference_velocity = msg->reference_velocity;
    this->reference_distance = msg->reference_distance;
    this->stateID = msg->state_id;
    this->distance_captured_from_service = msg->distance_captured;
    this->disable_drive_assist = msg->disable_drive_assist;
    this->aeb_state = msg->aeb_state;
}

void AdasMainVisualize::cbVisualizationIF(const ipc_helper::msg::PipelineResults::SharedPtr msg)
{
    // Copy Planning Results
    size_t size_planning_result_points = msg->planning_results.points.size();
    this->planning_results.points.resize(size_planning_result_points);
    for (size_t i = 0; i < size_planning_result_points; ++i) {
        this->planning_results.points[i][0] = msg->planning_results.points[i].x;
        this->planning_results.points[i][1] = msg->planning_results.points[i].y;
    }
    this->planning_results.is_danger_aeb = msg->planning_results.is_danger_aeb;
    this->planning_results.is_warning_aeb = msg->planning_results.is_warning_aeb;

    // Copy Lane markings
    this->perception_results.laneResult << msg->perception_results.lane_result;

    // Copy Advance Fusion Object
    this->perception_results.adv_objs << msg->perception_results.adv_objs;
    
    // Copy Sensor source object
    size_t num_src_objs = msg->perception_results.src_objs.size();
    this->perception_results.src_objs.resize(num_src_objs);
    for (int i = 0; i < num_src_objs; i++) {
        // Update value for sensor source
        size_t num_sensor =  msg->perception_results.src_objs[i].sensor_source_object.size();
        for (size_t j = 0; j < num_sensor; ++j) {
            // Access the key and value
            ipc_helper::msg::SensorSourcePair sensor_source = msg->perception_results.src_objs[i].sensor_source_object[j];
            this->perception_results.src_objs[i][static_cast<SensorType>(sensor_source.key)] = sensor_source.value;
        }
    }

    // Copy status
    this->perception_results.lane_status = msg->perception_results.lane_status;

    // Push data into queue
    this->PlanningResultsQueue.pushData(0, std::make_shared<PlanningResults>(this->planning_results));
    this->perceptionResultsQueue.pushData(0, std::make_shared<PerceptionResults>(this->perception_results));
}

void AdasMainVisualize::cbCarStatusIF(const ipc_helper::msg::CarStatus::SharedPtr msg)
{
    // Get wheel angle
    // DEBUG("Car status arrive time: %ld", msg->timestamp);
    // SteeringParameters steering_angle;
    // steering_angle.time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    // steering_angle.steering = msg->steering;
    // this->steerAngleQueue.push_back(steering_angle);

    // Get current control
    // this->cacheSteer(msg->steer);
    // this->cacheThrottle(msg->throttle);
    // this->cacheBrake(msg->brake);
    this->setGear(static_cast<GEAR>(msg->gear));
}

template <typename T>
void AdasMainVisualize::handleControlMsg(controlMsgAttributeType type, T value)
{
#ifdef ENABLE_CAN
#else
    auto topic_control =
        interface.getTopicHandler<TopicControlHandler>(AdasServiceIF::TopicNameT::CONTROL);
    switch (type)
    {
    case ATTRIBUTE_BRAKE: {
        DEBUG("Modified ATTRIBUTE_BRAKE");
        topic_control->publishBrake(value);
        break;
    }

    case ATTRIBUTE_STEER: {
        DEBUG("Modified ATTRIBUTE_STEER");
        topic_control->publishSteer(value);
        break;
    }

    case ATTRIBUTE_THROTTLE: {
        DEBUG("Modified ATTRIBUTE_THROTTLE");
        topic_control->publishThrottle(value);
        break;
    }

    case ATTRIBUTE_GEAR:
        DEBUG("Modified ATTRIBUTE_GEAR");
        topic_control->publishGear(value);
        break;
    case ATTRIBUTE_DEFAULT:
        DEBUG("Non control type attribute, no control is sent");
        break;
    default:
        DEBUG("Invalid Control Attribute");
        break;
    }
#endif
}

void AdasMainVisualize::cbSFFDebugIF(const ipc_helper::msg::SFFDebugData::SharedPtr msg)
{
    this->sffDebugQueue.pushData(0, std::make_shared<ipc_helper::msg::SFFDebugData>(*msg));
}
