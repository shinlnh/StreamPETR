#ifndef __ADAS_MAIN_VISUALIZE_H__
#define __ADAS_MAIN_VISUALIZE_H__

#include <string> 
#include <boost/thread.hpp>

#include "adas_service_if.h"
#include "intermediate_representations.h"
#include "data_sharing_queue.h"
#include "status_button.h"
#include "state_id.h"
#include "tesla.h"

typedef enum graphType_t {
    CAM
} graphType;

class AdasMainVisualize {
private:
    // TODO
    PerceptionResults perception_results;
    PlanningResults planning_results;

    bool is_stream = false;

    int stateID = 0;
    float carla_speed = 0.0f;
    float fps = 0.0f;
    float reference_velocity = 0.0f;
    float reference_distance = 0.0f;
    float distance_captured_from_service = 0.0f;
    bool disable_drive_assist = false;
    GEAR current_gear = GEAR::N;
    bool aeb_state = true;

    cv::Mat dst_image;
    
    // Thread
    boost::thread *udp_receiving_handler;
    boost::mutex dst_image_mutex;

    // Callback for new ADAS Service Interface
    void cbAdasControlIF(const ipc_helper::msg::AdasControl::SharedPtr msg);
    void cbServiceStatusIF(const ipc_helper::msg::ServiceStatus::SharedPtr msg);
    void cbVisualizationIF(const ipc_helper::msg::PipelineResults::SharedPtr msg);
    void cbCarStatusIF(const ipc_helper::msg::CarStatus::SharedPtr msg);
    void cbSFFDebugIF(const ipc_helper::msg::SFFDebugData::SharedPtr msg);
protected:
    // Interface section
    AdasServiceIF interface;

    template <typename T>
    void handleControlMsg(controlMsgAttributeType type, T value);

public:
    AdasMainVisualize();
    ~AdasMainVisualize();

    void setDstImage(cv::Mat newMat);
    cv::Mat getDstImage();

    cv::Mat getGrapCam(graphType type);
    void TriggerSignal(bool is_start);
    void handleConnection(std::string server_ip);
    void graphicsReceivingThread(std::string server_ip);

    void setValue(boolObjects boolEnum, bool value);
    void setValue(intObjects intEnum, int value);

    DataSharingQueue<PerceptionResults> perceptionResultsQueue;
    DataSharingQueue<PlanningResults> PlanningResultsQueue;
    DataSharingQueue<ipc_helper::msg::SFFDebugData> sffDebugQueue;

    int getStateID();
    float getFps();
    float getSpeed();
    float getRefVel();
    void setRefVel(float newRefVel);
    float getRefDist();
    void setRefDistance(float newRefDistance);
    float getDistanceCaptured();
    bool getDisableDriveAssist();
    bool getAEBState();
    void sendAEBTakeover();
    void setGear(GEAR gear);
    GEAR getGear();

    void doSteer(float steering_value);
    void doThrottle(float throttle_value);
    void doBrake(float brake_value);
    void doGear(int gear);

    PlanningResults getPlanningResults() const
    {
        return this->planning_results;
    };
};

#endif // __ADAS_MAIN_VISUALIZE_H__