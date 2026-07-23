#include "adas_main.h"
#include "policy_state_machine.h"
#include <algorithm>

#define LOOP_RATE 40

// Initialize the static loggers
LogUtility AdasMain::cb_camera("log/", "cb_camera", false);
LogUtility AdasMain::cb_carla_status_logger("log/", "cb_carla_status", false);
LogUtility AdasMain::cb_radar_logger("log/", "cb_radar", false);
LogUtility AdasMain::cb_gnss_logger("log/", "cb_gnss", false);
LogUtility AdasMain::cb_imu_logger("log/", "cb_imu", false);
LogUtility AdasMain::cb_odometer_logger("log/", "cb_odometer", false);
LogUtility AdasMain::cb_carla_api_logger("log/", "cb_carla_api", false);
LogUtility AdasMain::cb_service_status_logger("log/", "cb_service_status", false);
LogUtility AdasMain::cb_manual_logger("log/", "cb_manual", false);
LogUtility AdasMain::profiling_logger("log/", "profiling", false);

std::map<int, std::shared_ptr<SensorConfig>> SensorConfig::sensorConfigurations = {};
int looptimes = 0;


AdasMain::AdasMain(std::string src_type
                  , std::string src
                  , bool debug_aeb)
: src_type(src_type)
, src(src)
, planner_(debug_aeb)
, is_running(false)
// , can_if("can service", 42, false)
, carla_speed(0.0)
, hwc_controller(acc_controller, lks_controller)
, tjc_controller(tja_controller, lks_controller)
, policy_state_(nullptr)
, LaneDetectionsQueue(1)   // only 1 producer (Lane Detection Thread)
, perceptionResultsQueue(1) // only 1 producer (Policy State Machine) temporary
, PlanningResultsQueue(1)   // only 1 producer (method plan in AdasMain)
, perceptionOutputQueue(1)  // only 1 producer (Perception Processing Thread)
, rawSrcImageQueue(1)
, radar_detection_queue(1)
, camera_detection_queue(1)
, camera_inference_queue(1)
#ifdef ENABLE_CAN
, can_manager("can0")
#endif  // ENABLE_CAN
{
    INFO("Source type %s!", src_type.c_str());

    /*---------------------------- SENSOR CONFIG ----------------------------*/
    // Update config into sensor configuration map base on each sensor type 
    SensorConfig* camera_config = new SensorConfig();
    if (camera_config->read_sensor_config("common/config/sensor/camera_config.txt")) {
        SensorConfig::sensorConfigurations[static_cast<int>(SensorType::CAMERA)] = std::shared_ptr<SensorConfig>(camera_config); 
    }

    SensorConfig* radar_config = new SensorConfig();
    if (radar_config->read_sensor_config("common/config/sensor/radar_config.txt")) {
        SensorConfig::sensorConfigurations[static_cast<int>(SensorType::RADAR)] = std::shared_ptr<SensorConfig>(radar_config); 
    }

    // Create Image capture receiver
    if("udp" == src_type) {
        // Receive frames from UDP source. In our use case it's CARLA ROS bridge.
        receiver_app_handler = new ReceiverUdp(ORININAL_IMAGE_WIDTH, ORININAL_IMAGE_HEIGHT);
    }

    /*---------------------------- QUEUES CONFIG ----------------------------*/
    // Assign queues' type
    rawSrcImageQueue.type = "rawSrcImage";
    LaneDetectionsQueue.type = "LaneImage";
    perceptionOutputQueue.type = "PerceptionOutput";
    perceptionResultsQueue.type = "PerceptionResults";

    // Register sensors consumer
    if (camera_tracker_id < 1)   camera_tracker_id   = camera_detection_queue.createConsumer();
    if (radar_tracker_id < 1)    radar_tracker_id    = radar_detection_queue.createConsumer();
    if (object_detection_id < 1) object_detection_id = rawSrcImageQueue.createConsumer();
    if (raw_src_cons < 0)        raw_src_cons        = rawSrcImageQueue.createConsumer();
    if (lane_detection_id < 0)   lane_detection_id   = rawSrcImageQueue.createConsumer();
    
    /*---------------------------- ROS INTERFACE ----------------------------*/
    using TopicNameT = AdasServiceIF::TopicNameT;
    /* Carla -> ADAS Service Interface */
    // Register topics
    interface.registerTopic<TopicImuHandler>(TopicNameT::IMU);
    interface.registerTopic<TopicOdometerHandler>(TopicNameT::ODOMETER);
    interface.registerTopic<TopicGnssHandler>(TopicNameT::GNSS);
    interface.registerTopic<TopicCarStatusHandler>(TopicNameT::CAR_STATUS);
    interface.registerTopic<TopicRadarFrontHandler>(TopicNameT::RADAR);
    interface.registerTopic<TopicCarlaApiHandler>(TopicNameT::CARLA_API);

    // Get Topic Handlers
    auto topic_imu         = interface.getTopicHandler<TopicImuHandler>(TopicNameT::IMU);
    auto topic_odometer    = interface.getTopicHandler<TopicOdometerHandler>(TopicNameT::ODOMETER);
    auto topic_gnss        = interface.getTopicHandler<TopicGnssHandler>(TopicNameT::GNSS);
    auto topic_car_status  = interface.getTopicHandler<TopicCarStatusHandler>(TopicNameT::CAR_STATUS);
    auto topic_radar_front = interface.getTopicHandler<TopicRadarFrontHandler>(TopicNameT::RADAR);
    auto topic_carla_api   = interface.getTopicHandler<TopicCarlaApiHandler>(TopicNameT::CARLA_API);

    // Register callback
    topic_imu->         registerCallback([this](auto msg) { cbImuIF(msg); });
    topic_odometer->    registerCallback([this](auto msg) { cbOdometerIF(msg); });
    topic_gnss->        registerCallback([this](auto msg) { cbGnssIF(msg); });
    topic_car_status->  registerCallback([this](auto msg) { cbCarStatusIF(msg); });
    topic_radar_front-> registerCallback([this](auto msg) { cbRadarIF(msg); });
    topic_carla_api->   registerCallback([this](auto msg) { cbCarlaApiIF(msg); });

    /* Client Manual Control -> ADAS Service Interface */
    // Additional DDS interfaces for receiving clients manual controls commands.
    interface.registerTopic<TopicServiceCommandHandler>(TopicNameT::SERVICE_COMMAND);
    auto topic_service_command = interface.getTopicHandler<TopicServiceCommandHandler>(TopicNameT::SERVICE_COMMAND);
    topic_service_command->registerCallback([this](auto msg) { cbServiceCommandIF(msg); });
    
    /* ADAS Service Outbound Interfaces */
    // Interfaces for ADAS Service to publish data to other modules
    interface.registerTopic<TopicAdasControlHandler>(TopicNameT::ADAS_CONTROL);
    interface.registerTopic<TopicServiceStatusHandler>(TopicNameT::SERVICE_STATUS);
    interface.registerTopic<TopicVisualizationHandler>(TopicNameT::VISUALIZATION);
    interface.registerTopic<TopicSFFDebugHandler>(TopicNameT::SFF_DEBUG);

    /* Start ROS communication */
    interface.run();

    /*---------------------------- CAN INTERFACE ----------------------------*/
#ifdef ENABLE_CAN
    // Register CAN calbacks
    can_manager.registerCallback({TESLA_EPS_BUTTON_STATUS_FRAME_ID},
                                 [this](auto frame) { cbEPSButtonStatus(frame); });
    can_manager.registerCallback({TESLA_EPS_TAKE_OVER_STATUS_FRAME_ID},
                                 [this](auto frame) { cbEPSTakeOverStatus(frame); });
    can_manager.registerCallback({TESLA_EPS_ALERT_MATRIX_FRAME_ID},
                                 [this](auto frame) { cbEPSAlertMatrix(frame); });
    can_manager.registerCallback({TESLA_EPS_DRIVER_CONTROL_FRAME_ID},
                                 [this](auto frame) { cbEPSDriverControl(frame); });
                                 
#endif  // ENABLE_CAN

    /*------------------------------- THREADS -------------------------------*/
    //> Master thread
    master_thread_handler = new boost::thread(&AdasMain::masterThread, this);

    //> Perception processing thread
    perception_processing_thread_handler = new boost::thread(&AdasMain::PerceptionProcessingThread, this);

    //> Perception results DDS publish thread
    perception_visualize_thread_handler = new boost::thread(&AdasMain::perceptionResultsPublishThread, this);

    //> Initialize tracker module, object detection, and lane detection threads  
    radar_tracker_thread = new boost::thread(&AdasMain::radarTrackerThread, this);
    camera_tracker_thread = new boost::thread(&AdasMain::cameraTrackerThread, this);
    object_detection_thread = new boost::thread(&AdasMain::objectDetectionThread, this);
    lane_detection_thread = new boost::thread(&AdasMain::laneDetectionThread, this);
    
    /*-------------------------- RTSP STREAM SERVER -------------------------*/
    rtsp_handler = new RtspServer();
    INFO("Starting up the server");
    rtsp_handler->setRunning(true);
    rtsp_server_thread_handler = new boost::thread(&RtspServer::start_server, rtsp_handler);

    // Initialization Done!
    // Start capture immediately
    TriggerSignal(true);
}

AdasMain::~AdasMain()
{
    delete master_thread_handler;
    delete perception_visualize_thread_handler;
    // delete policy_state_thread_handler;
    rtsp_server_thread_handler->join();
    delete rtsp_server_thread_handler;
    rtsp_handler->setRunning(false);
}

void AdasMain::resetAutoDrivingState()
{
    // Reset all button states
    setValue(STATUS_BTN_LANE_KEEPING_SYSTEM, false);
    setValue(STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, false);
}


#ifdef ENABLE_CAN
/**
 * @brief Send control message via CAN
 * 
 * @param control_results Control result from controllers
 */
void AdasMain::sendCanControlMsg(ControlResults control_results) 
{
    // Prepare values
    float throttle = control_results.throttle_value * 100;
    float brake    = control_results.brake_value * 100;
    float steer    = control_results.steering_value * MAX_CAR_STEERING_ANGLE;

    // Create CAN frame
    can_frame frame;
    frame.can_id = TESLA_ADAS_CONTROL_FRAME_ID;
    frame.can_dlc = TESLA_ADAS_CONTROL_LENGTH;

    // Pack value
    tesla_adas_control_t msg;
    msg.adas_throttle_request       = tesla_adas_control_adas_throttle_request_encode(throttle);
    msg.adas_brake_request          = tesla_adas_control_adas_brake_request_encode(brake);
    msg.adas_steering_angle_request = tesla_adas_control_adas_steering_angle_request_encode(steer);
    tesla_adas_control_pack(frame.data, &msg, frame.can_dlc);

    // Send message to CAN bus
    can_manager.sendCanFrame(frame);
}


/**
 * @brief Send AEB state via CAN
 */
void AdasMain::sendCanAdasWarning(bool isWarning, bool isDanger, bool aebEnabled, int laneStatus)
{
    // Prepare values
    int aebWarning = 0;  // Default is Safe
    if (!aebEnabled)    aebWarning = 3;
    else if (isDanger)  aebWarning = 2;
    else if (isWarning) aebWarning = 1;
    bool laneExists = (laneStatus != LaneStatus::MISSING_LANE);
    bool laneDepart = (laneStatus != LaneStatus::IN_LANE);

    // Create CAN frame
    can_frame frame;
    frame.can_id = TESLA_ADAS_WARNING_FRAME_ID;
    frame.can_dlc = TESLA_ADAS_WARNING_LENGTH;

    // Pack value
    tesla_adas_warning_t msg;
    msg.adas_aeb_warning    = tesla_adas_warning_adas_aeb_warning_encode(aebWarning);
    msg.adas_lane_exists    = tesla_adas_warning_adas_lane_exists_encode(laneExists);
    msg.adas_lane_departure = tesla_adas_warning_adas_lane_departure_encode(laneDepart);
    tesla_adas_warning_pack(frame.data, &msg, frame.can_dlc);

    // Send message to CAN bus
    can_manager.sendCanFrame(frame);
}


/**
 * @brief Send current ADAS Mode via CAN
 */
void AdasMain::sendCanAdasState(int stateID)
{
    // Create CAN frame
    can_frame frame;
    frame.can_id = TESLA_ADAS_STATE_FRAME_ID;
    frame.can_dlc = TESLA_ADAS_STATE_LENGTH;

    // Pack value
    tesla_adas_state_t msg;
    msg.adas_idle_state  = tesla_adas_state_adas_idle_state_encode(stateID == IDLE_STATE_ID);
    msg.adas_brake_state = tesla_adas_state_adas_idle_state_encode(stateID == BRAKE_STATE_ID);
    msg.adas_lks_state   = tesla_adas_state_adas_idle_state_encode(stateID == LKS_STATE_ID);
    msg.adas_tja_state   = tesla_adas_state_adas_idle_state_encode(stateID == TJA_STATE_ID);
    msg.adas_acc_state   = tesla_adas_state_adas_idle_state_encode(stateID == ACC_STATE_ID);
    msg.adas_hwc_state   = tesla_adas_state_adas_idle_state_encode(stateID == HWC_STATE_ID);
    msg.adas_tjc_state   = tesla_adas_state_adas_idle_state_encode(stateID == TJC_STATE_ID);
    tesla_adas_state_pack(frame.data, &msg, frame.can_dlc);

    // Send message to CAN bus
    can_manager.sendCanFrame(frame);
}


/**
 * @brief Send current controller settings via CAN
 */
void AdasMain::sendCanControlSetting(float refVel, float refDist)
{
    // Create CAN frame
    can_frame frame;
    frame.can_id = TESLA_ADAS_CONTROL_SETTING_FRAME_ID;
    frame.can_dlc = TESLA_ADAS_CONTROL_SETTING_LENGTH;

    // Pack value
    tesla_adas_control_setting_t control_setting_msg;
    control_setting_msg.adas_speed_limit = tesla_adas_control_setting_adas_speed_limit_encode(refVel);
    control_setting_msg.adas_dis_limit = tesla_adas_control_setting_adas_dis_limit_encode(refDist);
    tesla_adas_control_setting_pack(frame.data, &control_setting_msg, frame.can_dlc);

    // Send message to CAN bus
    can_manager.sendCanFrame(frame);
}
#endif  // ENABLE_CAN


void AdasMain::detectLane(
    const cv::Mat& src_image,
    std::shared_ptr<LaneDetection> lane_result)
{
    // Check if the input image is empty.
    if (src_image.empty()) {
        return;
    }
    
    // Run the lane tracking pipeline synchronously.
    perceptionModel.laneDetectorPipeline(src_image);

    // Retrieve lane detection.
    perceptionModel.retrieveLaneResult(lane_result);
}


void AdasMain::detectObject(
    cv::Mat src_image,
    std::shared_ptr<SensorData<SensorType::CAMERA>> camera_data)
{
    // Check if the input image is empty.
    if (src_image.empty()) 
    {
        return;
    }
    
    // Run the object detection pipeline synchronously.
    uint64_t timestamp = camera_data->time_stamp;
    cv::Mat debug_img = perceptionModel.objectDetectionPipeline(
        timestamp,
        src_image,
        this->odometerQueue.at_ts(timestamp)
    );
    this->setDebugImage(debug_img);
    camera_data->objects = perceptionModel.getDetectedObjects();
}

void AdasMain::masterThread(void)
{
    INFO("Spawn thread!");
    pthread_setname_np(pthread_self(), "Master");
    boost::chrono::nanoseconds loop_delay(static_cast<int>(1e9/LOOP_RATE));
    boost::chrono::system_clock::time_point start_time_point, end_time;

    while (1) {
        // Fix: merge the master thread with policy state thread, expected behavior is that any incoming sensor will notify for lock. 
        start_time_point = boost::chrono::system_clock::now();
        end_time = start_time_point + loop_delay;

        /** Execute the current policy state, handle exceptions if any occur. */
        if (this->policy_state_ == nullptr) this->transitionToState(new AdasMain::IdleState);
        try {   
            // Adas main doesn't know PSM current state, it just 'execute'
            if (this->policy_state_) this->policy_state_->execute();
        }
        catch(const std::exception& e) {
            ERROR("Catch exception from state machine thread: %s", e.what());
        }

        // Publish Visualization
        perception_visualize_thread_trigger.notify_one();  //notify visualization thread

#ifdef ENABLE_CAN
        // Publish current mode to CAN
        this->sendCanAdasState(policy_state_->getId());
        this->sendCanControlSetting(this->getRefVel(), this->getRefDist());
#endif

        // Might need to wrap a around service here.
        ipc_helper::msg::ServiceStatus msg;
        msg.velocity = carla_speed;
        msg.fps = fps;
        msg.controller_theta = 0.0f;
        msg.controller_error = 0.0f;
        msg.controller_tau = getThrotlle();
        msg.controller_angle = 0.0f;
        msg.reference_velocity = getRefVel();
        msg.reference_distance = getRefDist();
        msg.state_id = policy_state_->getId();
        msg.distance_captured = getDistanceCaptured();
        msg.disable_drive_assist = getDisableDriveAssist();
        msg.aeb_state = getAEBState();

        auto topic_service_status = 
            interface.getTopicHandler<TopicServiceStatusHandler>(AdasServiceIF::TopicNameT::SERVICE_STATUS);
        topic_service_status->publish(msg);

        looptimes++;

        while ( boost::chrono::system_clock::now() < end_time) {
            // Yield the thread to avoid busy waiting
            boost::this_thread::yield();
        }
    }
    INFO("Kill thread!");
}

void AdasMain::packVisualizationMessage(
    ipc_helper::msg::PipelineResults &msg, 
    PerceptionResults const &perception_results, 
    PlanningResults const &planning_results)
{
    /* pack perception results */
    // Advance Fusion objects
    msg.perception_results.adv_objs << perception_results.adv_objs;

    // Sensor source of each object
    size_t size_sensor_source = perception_results.src_objs.size();
    msg.perception_results.src_objs.resize(size_sensor_source);
    for (size_t i = 0; i < size_sensor_source; ++i) {
        size_t num_src = perception_results.src_objs[i].size();
        msg.perception_results.src_objs[i].sensor_source_object.resize(num_src);
        size_t j = 0;
        for (auto source : perception_results.src_objs[i]) {
            msg.perception_results.src_objs[i].sensor_source_object[j].key = static_cast<int>(source.first);
            msg.perception_results.src_objs[i].sensor_source_object[j].value = static_cast<bool>(source.second);
            ++j;
        }
    }

    // Status
    msg.perception_results.lane_status = perception_results.lane_status;

    // Lane markings
    msg.perception_results.lane_result << perception_results.laneResult;

    /* pack planning results */ 
    size_t size_planning_result_points = planning_results.points.size();
    msg.planning_results.points.resize(size_planning_result_points);
    for (size_t i = 0; i < size_planning_result_points; ++i) {
        msg.planning_results.points[i].x = planning_results.points[i][0];
        msg.planning_results.points[i].y = planning_results.points[i][1];
    }
    msg.planning_results.is_danger_aeb = planning_results.is_danger_aeb;
    msg.planning_results.is_warning_aeb = planning_results.is_warning_aeb;
}

static void packClaimState(ipc_helper::msg::SFFClaimState &out,
                           const SafetyForceField::ClaimState &cs) {
    out.pos_x = cs.pos.x;
    out.pos_y = cs.pos.y;
    out.yaw = cs.yaw;
    out.t = cs.t;
    out.path_length = cs.path_length;
    out.v = cs.v;
    out.a = cs.a;
    out.b = cs.b;
    out.shape_type = static_cast<int32_t>(cs.shape_type);
    out.circle_center_x = cs.circle.center.x;
    out.circle_center_y = cs.circle.center.y;
    out.circle_radius = cs.circle.radius;
    out.obb_center_x = cs.OBB.center.x;
    out.obb_center_y = cs.OBB.center.y;
    out.obb_half_l = cs.OBB.half_l;
    out.obb_half_w = cs.OBB.half_w;
    out.obb_yaw = cs.OBB.yaw;
}

static void packClaimSet(ipc_helper::msg::SFFClaimSet &out,
                         const SafetyForceField::ClaimSet &cs) {
    // Downsample: every step_size-th state, plus first 3 for low velocity range
    const int step_size = cs.in_stop_state ? 1 : 30;
    std::vector<size_t> indices;

    // First 3 states for low velocity range
    if (cs.in_low_vel_range) {
        for (size_t i = 0; i < std::min<size_t>(3, cs.claim_state_vector.size()); ++i) {
            indices.push_back(i);
        }
    }
    // Sampled states
    for (size_t i = 0; i < cs.claim_state_vector.size(); i += step_size) {
        indices.push_back(i);
    }
    // Last state
    if (!cs.claim_state_vector.empty()) {
        indices.push_back(cs.claim_state_vector.size() - 1);
    }
    // Remove duplicates and sort
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

    out.claim_states.resize(indices.size());
    for (size_t k = 0; k < indices.size(); ++k) {
        packClaimState(out.claim_states[k], cs.claim_state_vector[indices[k]]);
    }

    out.state_type = static_cast<int32_t>(cs.state_type);
    out.time_danger = cs.time_danger;
    out.time_reaction = cs.time_reaction;
    out.time_warning = cs.time_warning;
    out.time_hit = cs.time_hit;
    out.distance_brake_threshold = cs.distance_brake_threshold;
    out.in_low_vel_range = cs.in_low_vel_range;
    out.in_stop_state = cs.in_stop_state;
}

void AdasMain::packSFFDebugMessage(ipc_helper::msg::SFFDebugData &msg,
                                   const SafetyForceField &sff) {
    // Pack ego sets
    msg.ego_sets.resize(sff.sffEgoSet_.size());
    for (size_t i = 0; i < sff.sffEgoSet_.size(); ++i) {
        packClaimSet(msg.ego_sets[i], sff.sffEgoSet_[i]);
    }

    // Pack object sets
    msg.obj_sets.resize(sff.sffObjSet_.size());
    for (size_t i = 0; i < sff.sffObjSet_.size(); ++i) {
        packClaimSet(msg.obj_sets[i], sff.sffObjSet_[i]);
    }

    // Pack decision
    msg.decision = static_cast<int32_t>(sff.decisionChecking);

    // Pack overlap result
    msg.overlap.hit = sff.sffOverlap.hit;
    msg.overlap.object_index = sff.sffOverlap.object_index;
    msg.overlap.ego_sample_index = sff.sffOverlap.ego_sample_index;
    msg.overlap.obj_sample_index = sff.sffOverlap.obj_sample_index;
    msg.overlap.ego_in_stop_state = sff.sffOverlap.ego_in_stop_state;
    msg.overlap.t_hit_ego = sff.sffOverlap.t_hit_ego;
    msg.overlap.t_hit_obj = sff.sffOverlap.t_hit_obj;
    msg.overlap.level = static_cast<int32_t>(sff.sffOverlap.level);
    packClaimState(msg.overlap.ego_overlap, sff.sffOverlap.ego_overlap);
    packClaimState(msg.overlap.obj_overlap, sff.sffOverlap.obj_overlap);
}

void AdasMain::perceptionResultsPublishThread() {
    pthread_setname_np(pthread_self(), "Perception_Pub");
    PerceptionResults perceptionResults;
    if (consumer_perception_results_id_ < 0) {
        consumer_perception_results_id_ = this->perceptionResultsQueue.createConsumer();
    }
    PlanningResults planningResults;
    if (consumer_planning_results_id_ < 0) {
        consumer_planning_results_id_ = this->PlanningResultsQueue.createConsumer();
    }

    while (1) {
        boost::unique_lock<boost::mutex> lock(perception_visualize_thread_mut);
        perception_visualize_thread_trigger.wait(lock, [&]{ 
            return this->perceptionResultsQueue.sizeOf(consumer_perception_results_id_) > 0; 
        } );

        // Retrieve data from queues
        std::queue<std::shared_ptr<PerceptionResults>> all_perception_results = 
            this->perceptionResultsQueue.getNext(consumer_perception_results_id_);

        std::queue<std::shared_ptr<PlanningResults>> all_planning_results = 
            this->PlanningResultsQueue.getNext(consumer_planning_results_id_, false);

        perceptionResults = *(all_perception_results.back());
        
        if (all_planning_results.empty() == false) {
            planningResults = *(all_planning_results.back());
        } else {
            planningResults.points.clear();
            planningResults.is_danger_aeb = this->getPlanningModel()->isDangerAEB();
            planningResults.is_warning_aeb = this->getPlanningModel()->isWarningAEB();
        }

        // Send image to Rtsp server.
        sendImageToRtspServer(perceptionResults);

        // Send perception and planning result over DDS.
        ipc_helper::msg::PipelineResults msg;
        packVisualizationMessage(msg, perceptionResults, planningResults);
        auto topic_visualization = 
            interface.getTopicHandler<TopicVisualizationHandler>(AdasServiceIF::TopicNameT::VISUALIZATION);
        topic_visualization->publish(msg);

        // Publish SFF debug data if AEB debug is enabled
        if (planner_.isAebDebug()) {
            ipc_helper::msg::SFFDebugData sff_msg;
            packSFFDebugMessage(sff_msg, planner_.getSFF());
            sff_msg.header = msg.header; // same timestamp as visualization
            auto topic_sff = interface.getTopicHandler<TopicSFFDebugHandler>(
                AdasServiceIF::TopicNameT::SFF_DEBUG);
            topic_sff->publish(sff_msg);
        }

#ifdef ENABLE_CAN
        // Send Warning status over CAN
        this->sendCanAdasWarning(planningResults.is_warning_aeb,
                                 planningResults.is_danger_aeb,
                                 getAEBState(),
                                 perceptionResults.lane_status);
#endif  // ENABLE_CAN
    }
}

void AdasMain::TriggerSignal(bool is_start)
{
    this->setIsRunning(is_start);
    if (true == is_start) {
        if ("tcp" == src_type) {
            capture_thread_handler = 
                new boost::thread(&AdasMain::captureTcpRemoteResourceThread, this, src);
        }
        else if ("udp" == src_type) {
            capture_thread_handler =
                new boost::thread(&AdasMain::captureUdpRemoteResourceThread, this, src);
        }
        else if ("gtcp" == src_type) {
            capture_thread_handler =
                new boost::thread(&AdasMain::captureGtcpRemoteResourceThread, this, src);
        }
        else if ("video" == src_type) {
            capture_thread_handler =
                new boost::thread(&AdasMain::captureVideoThread, this, src);
        }
        else {
            capture_thread_handler =
                new boost::thread(&AdasMain::captureCameraThread, this, src);
        }
    }
    else {
        capture_thread_handler->join();
        delete capture_thread_handler;
    }
}

void AdasMain::setValue(boolObjects boolEnum, bool value) {
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    if (boolVariableMap.find(boolEnum) != boolVariableMap.end()) {
        *boolVariableMap[boolEnum] = value;
        DEBUG("Modify bool at index %d", boolEnum);
    }
    else {
        ERROR("Bool enum not found in map.");
    }
}

void AdasMain::setValue(intObjects intEnum, int value) {
    if (intVariableMap.find(intEnum) != intVariableMap.end()) {
        *intVariableMap[intEnum] = value;
        DEBUG("Modify int at index %d with the value of %d", intEnum, value);
        switch (intEnum)
        {
        case VALUE_SLIDER_STEER:
            this->doSteer(value / 100);
            break;
        case VALUE_SLIDER_BRAKE:
            this->doBrake(value / 100);
            break;
        case VALUE_SLIDER_THROTTLE:
            this->doThrottle(value / 100);
            break;
        default:
            break;
        }
    } else {
        ERROR("Int enum not found in map.");
    }
}

template <typename T>
void AdasMain::handleControlMsg(controlMsgAttributeType type, T value)
{
#ifdef ENABLE_CAN
#else
    auto topic_control = interface.getTopicHandler<TopicAdasControlHandler>(AdasServiceIF::TopicNameT::ADAS_CONTROL);
    switch (type)
    {
    case controlMsgAttributeType::ATTRIBUTE_BRAKE: {
        DEBUG("Modified ATTRIBUTE_BRAKE");
        topic_control->publishBrake(value);
        break;
    }
    case controlMsgAttributeType::ATTRIBUTE_STEER: {
        DEBUG("Modified ATTRIBUTE_STEER");
        topic_control->publishSteer(value);
        break;
    }
    case controlMsgAttributeType::ATTRIBUTE_THROTTLE: {
        DEBUG("Modified ATTRIBUTE_THROTTLE");
        topic_control->publishThrottle(value);
        break;
    }
    default:
        DEBUG("Invalid Control Attribute");
        break;
    }
#endif  // ENABLE_CAN
}


void AdasMain::sendImageToRtspServer(PerceptionResults perceptionResults) 
{
    // Resize images

    // Combine images vertically
    cv::Mat finalImage;
    std::vector<cv::Mat> images{perceptionResults.src_image};
    cv::vconcat(images, finalImage);

    // Send to Rtsp server.
    rtsp_handler->feed_frame(finalImage, perceptionResults.time_stamp);
}


AdasMain::AdasMain()
: hwc_controller(acc_controller, lks_controller)
, tjc_controller(tja_controller, lks_controller)
#ifdef ENABLE_CAN
, can_manager("can0")
#endif
{
    INFO("Constructor with no arguments");
}

cv::Mat AdasMain::getDebugImage()
{
    boost::lock_guard<boost::mutex> lock(debug_img_mut);
    if (!this->new_img) {
        return cv::Mat();
    }
    return this->debug_img;
}

void AdasMain::setDebugImage(cv::Mat img)
{
    boost::lock_guard<boost::mutex> lock(debug_img_mut);
    this->debug_img = img;
    this->new_img = true;
}


float AdasMain::getSpeed()
{
    return carla_speed;
}

float AdasMain::getFps()
{
    return fps;
}

float AdasMain::getTau()
{
    return static_cast<float>(getThrotlle());
}

void AdasMain::setIsRunning(bool is_running)
{
    boost::lock_guard<boost::mutex> lock(is_running_mut);
    this->is_running = is_running;
}

bool AdasMain::getIsRunning()
{
    boost::lock_guard<boost::mutex> lock(is_running_mut);
    return this->is_running;
}

void AdasMain::setGear(GEAR gear)
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    this->current_gear = gear;
}

GEAR AdasMain::getGear()
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    return this->current_gear;
}

bool AdasMain::isReverse()
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    return this->current_gear == GEAR::R;
}
    
bool AdasMain::getAEBState()
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    return this->statusBtnAEB;
};

bool AdasMain::updateDisableDriveAssist(bool is_missing_lane)
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    static constexpr int maxDisableDriveAssistCount = 60;
    if (is_missing_lane) {
        this->disable_drive_assist_counter = std::min(this->disable_drive_assist_counter + 1, maxDisableDriveAssistCount);
        if (this->disable_drive_assist_counter >= maxDisableDriveAssistCount) {
            this->disable_drive_assist = true;
        }
    }
    else if (this->disable_drive_assist || this->disable_drive_assist_counter) {
        this->disable_drive_assist = false;
        this->disable_drive_assist_counter = 0;
    }

    return this->disable_drive_assist_counter;
}

void AdasMain::setDisableDriveAssist(bool disable_drive_assist)
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    this->disable_drive_assist = disable_drive_assist;
}

bool AdasMain::getDisableDriveAssist()
{
    boost::lock_guard<boost::mutex> lock(control_state_mutex_);
    return this->disable_drive_assist;
}

void AdasMain::captureCameraThread(std::string path)
{
    INFO("Spawn thread with %s!", path.c_str());
    cv::VideoCapture frameCaptureDevice;
    std::string gst_pipeline = 
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540, format=(string)NV12, framerate=(fraction)10/1 ! "
        "nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink";
    if (!frameCaptureDevice.open(gst_pipeline)) {
       ERROR("Cound not read from camera device");
       return;
    }
    // Define a rate to get frame from camera to avoid oversampling the hardware
    const int camera_fps = 30;
    const int sleep_duration_mili = 1000/camera_fps; //(in miliseconds)
    DEBUG("Done setup camera !");

    while (true == this->getIsRunning()) {
        DEBUG("Running!");

        cv::Mat new_frame;
        bool frame_ok = frameCaptureDevice.read(new_frame);
        if (!frame_ok) {
            INFO("Camera not working! (Or something went wrong)");
            break;
        }
        
        if (new_frame.empty()) {
            continue;
        }

        auto camera_image = std::make_shared<CameraImageData>();
        camera_image->image = new_frame;
        camera_image->time_stamp = receiver_app_handler->timeStamp;
        rawSrcImageQueue.pushData(0, camera_image);

        boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_duration_mili));
    }

    //Release connection with camera device
    frameCaptureDevice.release();
    INFO("Kill thread!");
}

void AdasMain::captureVideoThread(std::string path)
{
    INFO("Spawn thread with %s!", path.c_str());
    cv::VideoCapture frameCaptureDevice;
    if (!frameCaptureDevice.open(path)) {
        ERROR("Cound not open video file");
        return;
    }

    //Get the fps from the video file
    double video_fps = frameCaptureDevice.get(cv::CAP_PROP_FPS);
    int n_frames = frameCaptureDevice.get(cv::CAP_PROP_FRAME_COUNT);
    size_t current_frame_id = 0;

    DEBUG("video with %d frames, %f video_fps", n_frames, video_fps);
	if (video_fps <=0) {
		WARN("CAN NOT read video FPS, using default value");
		video_fps = 50.0;
	}
    
    frameCaptureDevice.set(cv::CAP_PROP_POS_FRAMES, current_frame_id);

	const int sleep_duration_mili = 1000/video_fps; //(in miliseconds)
    while (true == this->getIsRunning()) {
        current_frame_id++;
        if (current_frame_id > n_frames) {
            break;
        }

        cv::Mat new_frame;
        bool frame_ok = frameCaptureDevice.read(new_frame);
        if (!frame_ok) {
            INFO("Video ended! (Or something went wrong)");
            break;
        }
        DEBUG("Running! - Ready pop out 1 frame from video");
        
        if (!new_frame.empty()) {
            continue;
        }

        auto camera_image = std::make_shared<CameraImageData>();
        camera_image->image = new_frame;
        rawSrcImageQueue.pushData(0, camera_image);

        boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_duration_mili));
    }
    //Release (close video file)
    frameCaptureDevice.release();
    INFO("Kill thread!");
}

void AdasMain::captureTcpRemoteResourceThread(std::string url)
{
    INFO("Spawn thread with %s!", url.c_str());
    const int sleep_duration_mili = 1000/LOOP_RATE; //(in miliseconds)
    TcpIpServer StreamingSerer = TcpIpServer();
    StreamingSerer.startServer();
    while (true == this->getIsRunning()) {
        DEBUG("Running!");
        // Fix: need to fex camera queue here

        boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_duration_mili));
    }
    INFO("Kill thread!");
}

void AdasMain::captureUdpRemoteResourceThread(std::string url)
{
    INFO("Spawn thread with %s!", url.c_str());
    LogUtility log = AdasMain::getCbCameraLogger();
    log.disableLogFile();

    const int sleep_duration_mili = 1000/LOOP_RATE; //(in miliseconds)
    while (true == this->getIsRunning()) {
		DEBUG("Running!");
        cv::Mat new_frame = receiver_app_handler->getNextFrame();
        
        if (!new_frame.empty()) {
            auto camera_image = std::make_shared<CameraImageData>();
            camera_image->image = new_frame;
            camera_image->time_stamp = receiver_app_handler->timeStamp;
            rawSrcImageQueue.pushData(0, camera_image);
        }
    }

    INFO("Kill thread!");
}

void AdasMain::captureGtcpRemoteResourceThread(std::string url)
{
    INFO("Spawn thread with %s!", url.c_str());
    const int sleep_duration_mili = 1000/30; //(in miliseconds)
    ReceiverGTCP image_stream(800, 600);
    while (true == this->getIsRunning()) {
        DEBUG("Running!");

        boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_duration_mili));
    }
    INFO("Kill thread!");
}

/**
 * @brief The PerceptionProcessingThread function performs a series of processing steps on the input image and imu parameters.
 * It includes undistortion, lane detection, object detection, distance estimation,
 * localization, Frenet conversion, and path planning. The processed information is stored in
 * respective models for further use.
 * @param configs 
 * @param IMU_parameters
 * @return PerceptionOutputObject 
 */
void AdasMain::PerceptionProcessingThread() {
    INFO("Create perception thread");
    pthread_setname_np(pthread_self(), "Perception");
    while(1) {
        auto start_time = boost::chrono::steady_clock::now();

        std::queue<std::shared_ptr<TrackedSensorData>> current_data_queue = this->SynchronizeQueue.getNext();

        // Stop timer after getNext() returns
        auto end_time = boost::chrono::steady_clock::now();
        long duration_ms = boost::chrono::duration_cast<boost::chrono::milliseconds>(end_time - start_time).count();
        DEBUG("SynchronizeQueue.getNext() took %ld ms", duration_ms);

        PerceptionOutputObject perception_results = perceptionModel.execute(current_data_queue);
        uint64_t timestamp = perception_results.time_stamp;
        perception_results.is_reverse      = this->isReverse();
        perception_results.my_car_width    = this->perceptionModel.getCarWidth();
        perception_results.my_car_speed    = *(this->odometerQueue.at_ts(timestamp));
        perception_results.my_car_state    = *(this->imuQueue.at_ts(timestamp));
        perception_results.my_car_location = *(this->gnssQueue.at_ts(timestamp));
        perception_results.steering_angle  = *(this->steerAngleQueue.at_ts(timestamp));

        perceptionOutputQueue.pushData(0, std::make_shared<PerceptionOutputObject>(perception_results));
    }
    INFO("Kill thread!");
}

void AdasMain::cameraTrackerThread()
{
    INFO("Create camera tracker thread");
    pthread_setname_np(pthread_self(), "Camera_Tracker");
    while(true) {
        std::queue<std::shared_ptr<SensorDataType>> result = this->camera_detection_queue.getNext(this->camera_tracker_id);
        std::shared_ptr<SensorDataType> sensor_data = result.back();
        uint64_t timestamp = sensor_data->time_stamp;
        std::shared_ptr<TrackedSensorData> tracked_sensor_data = 
            this->perceptionModel.Tracking(sensor_data,
                                           this->steerAngleQueue.at_ts(timestamp),
                                           this->imuQueue.at_ts(timestamp),
                                           this->gnssQueue.at_ts(timestamp),
                                           this->odometerQueue.at_ts(timestamp),
                                           this->isReverse()
                                           );
        if (tracked_sensor_data) {
            int sensor_id = static_cast<int>(tracked_sensor_data->getSensorType());
            SynchronizeQueue.pushData(sensor_id, tracked_sensor_data);
        }
    }
    INFO("Kill thread!");
}
void AdasMain::radarTrackerThread()
{
    INFO("Create radar tracker thread");
    pthread_setname_np(pthread_self(), "Radar_Tracker");
    while(true) {
        std::queue<std::shared_ptr<SensorDataType>> result = this->radar_detection_queue.getNext(this->radar_tracker_id);
        std::shared_ptr<SensorDataType> sensor_data = result.back();
        uint64_t timestamp = sensor_data->time_stamp;
        std::shared_ptr<TrackedSensorData> tracked_sensor_data = 
            this->perceptionModel.Tracking(sensor_data,
                                           this->steerAngleQueue.at_ts(timestamp),
                                           this->imuQueue.at_ts(timestamp),
                                           this->gnssQueue.at_ts(timestamp),
                                           this->odometerQueue.at_ts(timestamp),
                                           this->isReverse()
                                           );
        if(tracked_sensor_data) {
            int sensor_id = static_cast<int>(tracked_sensor_data->getSensorType());
            SynchronizeQueue.pushData(sensor_id, tracked_sensor_data);
        }
    }
    INFO("Kill thread!");
}

void AdasMain::objectDetectionThread()
{
    INFO("Create object detection thread");
    
    pthread_setname_np(pthread_self(), "Obj_Detect");

    while(true) {
        std::queue<std::shared_ptr<CameraImageData>> result = this->rawSrcImageQueue.getNext(this->object_detection_id);
        cv::Mat src_image = result.back()->image.clone();

        std::shared_ptr<SensorData<SensorType::CAMERA>> camera_data = std::make_shared<SensorData<SensorType::CAMERA>>();
        camera_data->time_stamp = result.back()->time_stamp;
        camera_data->config = SensorConfig::sensorConfigurations[static_cast<int>(SensorType::CAMERA)];

        // Perform inference, the result is update directly into camera_data to reduce copy
        this->detectObject(src_image, camera_data);
        // Using the share queue to push the radar frame here. 
        std::shared_ptr<SensorDataType> current_sensor_data = std::static_pointer_cast<SensorDataType>(camera_data);
        camera_detection_queue.pushData(0, current_sensor_data);
    }
    INFO("Kill thread!");
}

void AdasMain::laneDetectionThread()
{
    DEBUG("Create lane detection thread");
    pthread_setname_np(pthread_self(), "Lane_Detect");
    while(true) {
        std::queue<std::shared_ptr<CameraImageData>> result = this->rawSrcImageQueue.getNext(this->lane_detection_id);
        static uint64_t time; 

        if(!result.empty()) {
            if (time == result.back()->time_stamp && result.back()->time_stamp != 0) {
                DEBUG("consume the same frame: %ld", time);
            }
            std::shared_ptr<LaneDetection> lane_result = std::make_shared<LaneDetection>();
            cv::Mat src_image = result.back()->image.clone();
            lane_result->time_stamp = result.back()->time_stamp;
            // Perform inference, the result is update directly into camera_data to reduce copy
            this->detectLane(src_image, lane_result);
            // Using the share queue to push the radar frame here. 
            LaneDetectionsQueue.pushData(0, lane_result);
            time = result.back()->time_stamp;
        }
    }
    INFO("Kill thread!");
}

PlanningResults AdasMain::plan( PerceptionOutputObject &perception_output,
                                LaneDetection &lane_detection,
                                TrajectoryGenerationPolicy trajectory_generation_policy, 
                                DecisionMakingPolicy decision_policy)
{
    PlanningResults planning_results = planner_.execute(perception_output, lane_detection, trajectory_generation_policy, decision_policy);
    this->planning_queue.enqueue(planning_results.points);
    planning_results.points = this->planning_queue.getPlanningResults();
    this->PlanningResultsQueue.pushData(0, std::make_shared<PlanningResults>(planning_results));
    return planning_results;
}


/**
 * @brief Wrapper for the LKS Controller's execute function.
 * 
 * @param planning_results 
 * @param velocity 
 * @param configs 
 * @return ControlResults 
 */
ControlResults AdasMain::controlLKS(const PlanningResults &planning_results, const float &velocity, ControlConfigs configs)
{
    ControlResults control_results = lks_controller.execute(planning_results, 
                                                            velocity, 
                                                            configs);
    return control_results;
}

ControlResults AdasMain::controlACC(const PlanningResults &planning_results, const float &velocity, ControlConfigs configs)
{
    ControlResults control_results = acc_controller.execute(planning_results, 
                                                            velocity, 
                                                            configs);
    return control_results;
}

/**
 * @brief Wrapper for the TJA Controller's execute function.
 * 
 * @param planning_results 
 * @param velocity 
 * @param configs 
 * @return ControlResults 
 */
ControlResults AdasMain::controlTJA(const PlanningResults &planning_results, const float &velocity, ControlConfigs configs)
{
    ControlResults control_results = tja_controller.execute(planning_results, 
                                                            velocity, 
                                                            configs);
    return control_results;
}

ControlResults AdasMain::controlTJC(const PlanningResults &planning_results, const float &velocity, ControlConfigs configs)
{
    ControlResults control_results = tjc_controller.execute(planning_results, 
                                                            velocity, 
                                                            configs);
    return control_results;
}

ControlResults AdasMain::controlHWC(const PlanningResults &planning_results, const float &velocity, ControlConfigs configs)
{
    ControlResults control_results = hwc_controller.execute(planning_results, 
                                                            velocity, 
                                                            configs);
    return control_results;
}

ControlResults AdasMain::controlAEB()
{
    ControlResults control_results = aeb_controller.execute();
    return control_results;
}

void AdasMain::drive(ControlResults control_results)
{
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_STEER, control_results.steering_value);
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_THROTTLE, control_results.throttle_value);
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_BRAKE, control_results.brake_value);
}

void AdasMain::driveACC(ControlResults control_results) {
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_THROTTLE, control_results.throttle_value);
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_BRAKE, control_results.brake_value);
}

void AdasMain::driveAEB(ControlResults control_results) {
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_BRAKE, control_results.brake_value);
}

void AdasMain::driveTJA(ControlResults control_results) {
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_THROTTLE, control_results.throttle_value);
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_BRAKE, control_results.brake_value);
}

void AdasMain::driveLKS(ControlResults control_results) {
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_STEER, control_results.steering_value);
}

void AdasMain::doSteer(float steering_value)
{
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_STEER, steering_value);
}

void AdasMain::doThrottle(float throttle_value)
{
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_THROTTLE, throttle_value);
}

void AdasMain::doBrake(float brake_value)
{
    handleControlMsg(controlMsgAttributeType::ATTRIBUTE_BRAKE, brake_value);
}

/**
 * @brief The transitionToState function transitions the state machine to a new PolicyState.
 * 
 * This is a state-machine's design rule to ensure the machine has full-right to switch to a new state.
 * It deletes the current state (if any) and sets the new state, also updating the state machine reference.
 *
 * @param state Pointer to the new PolicyState to transition to.
 */
void AdasMain::transitionToState(PolicyState *state)
{
    // Reset cached drive command message
    // Preserve current control values based on the new state's capabilities
    // to avoid momentary control disruption during mode transition
    ControlResults controlResult;
    if (state->speedKeepEnabled()) {
        controlResult.throttle_value = this->getCachedThrottle();
    }
    if (state->laneKeepEnabled()) {
        controlResult.steering_value = this->getCachedSteer();
    }
#ifdef ENABLE_CAN
    sendCanControlMsg(controlResult);
#else
    drive(controlResult);
#endif  // ENABLE_CAN

    // Reset Pipeline results
    this->initPipelineResults();

    // Reset planning queue
    this->planning_queue.reset();

    // Delete the current PolicyState to avoid memory leaks.
    std::unique_lock<boost::mutex> lock(policy_transition_state_mut);
    if (this->policy_state_ != nullptr) {
        delete this->policy_state_;
    }

    // Set the new state and add this adas_main object for ref.
    this->policy_state_ = state;
    this->policy_state_->setMachineRef(this);
    this->policy_state_->enter(this);
}

/**
 * @brief This function check and release the Brake state.
 * If system not in Brake state then nothing happen.
 */
void AdasMain::releaseBrakeState()
{
    if (this->getStateID() == BRAKE_STATE_ID) {
        static_cast<BrakeState*>(this->policy_state_)->notifyReleaseAEB();
    }
}

PlanningConfigs AdasMain::createPlanningConfigs(int width, int height)
{
    PlanningConfigs configs;

    // Extend configs here.
    configs.frame_width = width;
    configs.frame_height = height;

    return configs;
}

ControlConfigs AdasMain::createControlConfigs()
{
    ControlConfigs configs;

    // Extend configs here.
    configs.custom = false;

    return configs;
}


int AdasMain::getStateID()
{
    std::unique_lock<boost::mutex> lock(policy_transition_state_mut);
    if (this->policy_state_ == nullptr)
        return -1;
    return this->policy_state_->getId();
}

float AdasMain::getDistanceCaptured()
{
    return this->distance_captured;
}

void AdasMain::enableStateMachineLogFile() 
{
    PolicyState::enableLogFile();
}

/*===================Callback for new ADAS Service Interface ===================*/
void AdasMain::cbImuIF(const ipc_helper::msg::ImuParameters::SharedPtr msg)
{
    // Get logger
    LogUtility& log = AdasMain::getCbImuLogger();
    log.disableLogFile();

    // Logging Imu parameters
    DEBUG("  x: %f", msg->imu_orientation.x);
    DEBUG("  y: %f", msg->imu_orientation.y);
    DEBUG("  z: %f", msg->imu_orientation.z);
    DEBUG("  w: %f", msg->imu_orientation.w);

    DEBUG("  y: %f", msg->imu_angular_velocity.y);
    DEBUG("  z: %f", msg->imu_angular_velocity.z);
    DEBUG("  x: %f", msg->imu_angular_velocity.x);

    DEBUG("  y: %f", msg->imu_linear_acceleration.y);
    DEBUG("  z: %f", msg->imu_linear_acceleration.z);
    DEBUG("  x: %f", msg->imu_linear_acceleration.x);

    // Create imu parameter instance
    ImuParameters imu_parameters;

    imu_parameters.Imu_orientation.x = msg->imu_orientation.x;
    imu_parameters.Imu_orientation.y = msg->imu_orientation.y;
    imu_parameters.Imu_orientation.z = msg->imu_orientation.z;
    imu_parameters.Imu_orientation.w = msg->imu_orientation.w;

    imu_parameters.Imu_angular_velocity.x = msg->imu_angular_velocity.x;
    imu_parameters.Imu_angular_velocity.y = msg->imu_angular_velocity.y;
    imu_parameters.Imu_angular_velocity.z = msg->imu_angular_velocity.z;

    imu_parameters.Imu_linear_acceleration.x = msg->imu_linear_acceleration.x;
    imu_parameters.Imu_linear_acceleration.y = msg->imu_linear_acceleration.y;
    imu_parameters.Imu_linear_acceleration.z = msg->imu_linear_acceleration.z;
    imu_parameters.time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();

    // Add the new element to the imuQueue
    this->imuQueue.push(imu_parameters);

    // Set yaw for the planning queue
    this->planning_queue.setYaw(msg->imu_roll_pitch_yaw.yaw);
}

void AdasMain::cbOdometerIF(const ipc_helper::msg::Odometry::SharedPtr msg)
{
    // Get logger
    LogUtility& log = AdasMain::getCbOdometerLogger();
    log.disableLogFile();

    // Logging Odometer values
    DEBUG("Linear velocity [ y: %f; z: %f; x: %f ]", msg->linear_velocity.y, msg->linear_velocity.z, msg->linear_velocity.x);
    DEBUG("Angular velocity [ y: %f; z: %f; x: %f ]", msg->angular_velocity.y, msg->angular_velocity.z, msg->angular_velocity.x);

    // Create odometry parameter instance
    OdometryParameters odometry_parameters;
    odometry_parameters.time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();

    odometry_parameters.angular_velocity.x = msg->angular_velocity.x;
    odometry_parameters.angular_velocity.y = msg->angular_velocity.y;
    odometry_parameters.angular_velocity.z = msg->angular_velocity.z;

    odometry_parameters.linear_velocity.x = msg->linear_velocity.x;
    odometry_parameters.linear_velocity.y = msg->linear_velocity.y;
    odometry_parameters.linear_velocity.z = msg->linear_velocity.z;

    this->carla_speed = (msg->linear_velocity.y * 3.6f);
    

    // Add the new element to the odometerQueue
    this->odometerQueue.push(odometry_parameters);
}

void AdasMain::cbGnssIF(const ipc_helper::msg::GnssPoint::SharedPtr msg)
{
    LogUtility& log = AdasMain::getCbGnssLogger();
    log.disableLogFile();

    DEBUG("X: %f - Y: %f Z: %f", msg->x, msg->y, msg->z);
    DEBUG("\n");

    this->planning_queue.setCarLocation(msg->x, msg->y, msg->z);
    GnssPoint gnss_point;
    gnss_point.east = msg->x;
    gnss_point.north = msg->y;
    gnss_point.up = msg->z;
    gnss_point.time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();

    // Add the new element to the gnssQueue
    this->gnssQueue.push(gnss_point);
}

void AdasMain::cbCarStatusIF(const ipc_helper::msg::CarStatus::SharedPtr msg)
{
    LogUtility& log = AdasMain::getCbCarlaStatusLogger();
    log.disableLogFile();

    // Get wheel angle
    DEBUG("Car status arrive time: %ld", msg->timestamp);
    SteeringParameters steering_angle;
    steering_angle.time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    steering_angle.steering = msg->steering;
    this->steerAngleQueue.push(steering_angle);

    // Get current control
    this->cacheSteer(msg->steer);
    this->cacheThrottle(msg->throttle);
    this->cacheBrake(msg->brake);
    this->setGear(static_cast<GEAR>(msg->gear));

    // Calculate camera_pitch to ground
    auto fl_wheel = msg->wheel_position[0];
    auto fr_wheel = msg->wheel_position[1];
    auto rl_wheel = msg->wheel_position[2];
    auto rr_wheel = msg->wheel_position[3];
    float front_height = (fl_wheel.z + fr_wheel.z) / 2;
    float rear_height = (rl_wheel.z + rr_wheel.z) / 2;
    float delta_height = front_height - rear_height;
    float front_y = (fl_wheel.y + fr_wheel.y) / 2;
    float rear_y = (rl_wheel.y + rr_wheel.y) / 2;
    float delta_y = front_y - rear_y;

    float camera_ground_pitch = std::atan(delta_height / delta_y);

    this->perceptionModel.setPitch(camera_ground_pitch);
}

void AdasMain::cbRadarIF(const ipc_helper::msg::Radar::SharedPtr msg)
{
    LogUtility& log = AdasMain::getCbRadarLogger();
    log.disableLogFile();
    
    // Retrieve sensor configs
    auto radar_config = SensorConfig::sensorConfigurations[static_cast<int>(SensorType::RADAR)];
    auto camera_config = SensorConfig::sensorConfigurations[static_cast<int>(SensorType::CAMERA)];
    // Process radar cloud point and extract cluster data.
    // Fix: ajust the data sensor input to map it with radar data, note that the radar data is in 3D 
    auto radar_data = std::make_shared<SensorData<SensorType::RADAR>>();
    radar_data->time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    radar_data->config     = radar_config;
    for (const auto &obj: msg->radar_clouds){
        // Sometimes, radar_data returns NaN velocity values. We really need to remove them before tracking --> This issue will be addressed in the ros_bridge in the near future
        if (std::isnan(obj.velocity.x) || std::isnan(obj.velocity.y) || std::isnan(obj.velocity.z))
        {                
            continue;
        }
        
        std::shared_ptr<InputObject> detection = std::make_shared<InputObject>();
        detection->bbox     << obj.min_point.x, obj.min_point.z, obj.max_point.x, obj.max_point.z; 
        detection->velocity << obj.velocity.x, obj.velocity.y, obj.velocity.z;
        detection->location << obj.location.x, obj.location.y, obj.location.z;
        detection->closest_point << obj.closest_point.x, obj.closest_point.y, obj.closest_point.z;
        this->perceptionModel.convertRadarBboxTo2D(detection, radar_config, camera_config);
        if(obj.velocity.x != 0 || obj.velocity.y != 0) {
            // Vector magnitude
            float magnitude = std::sqrt(square(obj.velocity.x) + square(obj.velocity.y)); 
            detection->orientation << obj.velocity.x / magnitude, obj.velocity.y / magnitude;
        }
        else {
            detection->orientation << 0.0f, 0.0f;
        }
        
        detection->segment_mask = cv::Mat();
        detection->classID      = -1;
        detection->sensor_source[SensorType::RADAR] = true;
        
        radar_data->objects.push_back(detection);
    }

    // Using the share queue to push the radar frame here. 
    std::shared_ptr<SensorDataType> current_sensor_data = std::static_pointer_cast<SensorDataType>(radar_data);
    this->radar_detection_queue.pushData(0, current_sensor_data);
}

void AdasMain::cbCarlaApiIF(const carla_msgs_custom::msg::VehicleDataArray::SharedPtr msg)
{
    LogUtility& log = AdasMain::getCbCarlaApiLogger();
    log.disableLogFile();

    SensorData<SensorType::API_SENSOR> carlaAPI_data = SensorData<SensorType::API_SENSOR>();
    carlaAPI_data.vehicle_count = msg->vehicle_count;
    carlaAPI_data.time_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
    
    for (size_t i = 0; i < msg->vehicles.size(); ++i)
    {
        std::shared_ptr<APIObject> vehicle = std::make_shared<APIObject>();
        vehicle->vehicleID   = msg->vehicles[i].id;
        vehicle->type        = msg->vehicles[i].type;
        vehicle->location    = {
            static_cast<float>(msg->vehicles[i].location.x), 
            static_cast<float>(msg->vehicles[i].location.y), 
            static_cast<float>(msg->vehicles[i].location.z)
        };
        vehicle->velocity    = {
            static_cast<float>(msg->vehicles[i].velocity.x), 
            static_cast<float>(msg->vehicles[i].velocity.y), 
            static_cast<float>(msg->vehicles[i].velocity.z)
        };
        vehicle->speed_kph   = static_cast<float>(msg->vehicles[i].speed_kph);
        vehicle->speed_kph_  = {
            static_cast<float>(msg->vehicles[i].vector_speed_kph.x), 
            static_cast<float>(msg->vehicles[i].vector_speed_kph.y), 
            static_cast<float>(msg->vehicles[i].vector_speed_kph.z)
        };
        vehicle->centripetal_location = msg->vehicles[i].centripetal_location;
        vehicle->sensor_source[SensorType::API_SENSOR] = true;

        // Logging vehicle data
        INFO("Vehicle %zu:", i);
        INFO("  ID: %d", vehicle->vehicleID);
        INFO("  Type: %s", vehicle->type.c_str());             
        INFO("  Location: (%f, %f, %f)", vehicle->location.x(), vehicle->location.y(), vehicle->location.z());
        INFO("  Velocity: (%f, %f, %f)", vehicle->speed_kph_.x(), vehicle->speed_kph_.y(), vehicle->speed_kph_.z());
        INFO("  Speed (kph): %f", vehicle->speed_kph);
        INFO("  Centripetal Location: %f", vehicle->centripetal_location);

        // Add the vehicle to the parameters
        carlaAPI_data.objects.push_back(vehicle);
    }
    // Add the new element to the carlaApiQueue
    this->carlaApiQueue.push(carlaAPI_data);
}

void AdasMain::cbServiceCommandIF(const ipc_helper::msg::ServiceCommand::SharedPtr msg)
{
    LogUtility& log = AdasMain::getCbServiceStatusLogger();
    log.disableLogFile();

    DEBUG("Ego info arrive time: %ld", boost::chrono::system_clock::now());

    // Capture button
    if (msg->flag_modified & (1U << TopicServiceCommandHandler::CAPTURE)) {
        bool new_running_state = msg->capture_enabled;
        bool current_running_state = this->getIsRunning();
        if (new_running_state != current_running_state) {
            INFO("Capture Enabled Value Sent from Client: %d", new_running_state);
            this->TriggerSignal(new_running_state);
        }
    }
    
    // Reference values
    bool updateButton = msg->flag_modified != 0U;  /** @todo This is a trick to avoid sending outdated values, remove ASAP */
    if (! updateButton) {
        this->setRefVel(msg->velocity_ref);
        this->setRefDistance(msg->distance_ref);
    }
    
    // Control buttons.
    if (msg->flag_modified & (1U << TopicServiceCommandHandler::CONTROL_AEB)) {
        if (this->getSpeed() <= SPEED_AUTO_DISABLE_AEB) {
            this->setValue(STATUS_BTN_AUTO_EMERGENCY_BRAKING, msg->control_aeb_enabled);
            // If manually disabled, ensure logic doesn't auto-reenable thinking it was a speed-limit toggle
            if (msg->control_aeb_enabled == false) {
                 boost::lock_guard<boost::mutex> lock(control_state_mutex_);
                 auto_disabled_aeb = false;
            }
        } else {
             // If speed > SPEED_AUTO_DISABLE_AEB, we generally ignore. 
             // Edge case: If user sends "OFF" command while we are auto-disabled, 
             // we technically are already OFF. But we might want to clear the 'auto' flag so it stays OFF.
             // However, strictly following "Do not change state", we do nothing.
             WARN("Ignored AEB toggle request (DDS) due to high speed: %.2f km/h", this->getSpeed());
        }
    }
    if (msg->flag_modified & (1U << TopicServiceCommandHandler::CONTROL_LKS))
        this->setValue(STATUS_BTN_LANE_KEEPING_SYSTEM, msg->control_lks_enabled);
    if (msg->flag_modified & (1U << TopicServiceCommandHandler::CONTROL_ACC))
        this->setValue(STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, msg->control_acc_enabled);

    // AEB release signal
    if (msg->aeb_takeover) {
        this->releaseBrakeState();
    }
}


#ifdef ENABLE_CAN
void AdasMain::cbEPSButtonStatus(const can_frame& frame)
{
    // Unpack
    tesla_eps_button_status_t button_status;
    tesla_eps_button_status_unpack(&button_status, frame.data, frame.can_dlc);

    // Get values
    auto lksState      = bool(tesla_eps_button_status_eps_lks_state_decode(button_status.eps_lks_state));
    auto accState      = bool(tesla_eps_button_status_eps_acc_state_decode(button_status.eps_acc_state));
    auto aebState      = bool(tesla_eps_button_status_eps_aeb_state_decode(button_status.eps_aeb_state));
    auto disableAdas   = bool(tesla_eps_button_status_eps_disable_adas_decode(button_status.eps_disable_adas));
    auto incSpeedLimit = uint8_t(tesla_eps_button_status_eps_inc_speed_limit_decode(button_status.eps_inc_speed_limit));
    auto decSpeedLimit = uint8_t(tesla_eps_button_status_eps_dec_speed_limit_decode(button_status.eps_dec_speed_limit));
    auto incDisLimit   = uint8_t(tesla_eps_button_status_eps_inc_dis_limit_decode(button_status.eps_inc_dis_limit));
    auto decDisLimit   = uint8_t(tesla_eps_button_status_eps_dec_dis_limit_decode(button_status.eps_dec_dis_limit));

    // Handle state controls
    if (aebState == true) {
        if (this->getSpeed() <= SPEED_AUTO_DISABLE_AEB) {
            bool newAebState = !this->getAEBState();
            this->setValue(STATUS_BTN_AUTO_EMERGENCY_BRAKING, newAebState);
            if (newAebState == false) {
                 boost::lock_guard<boost::mutex> lock(control_state_mutex_);
                 auto_disabled_aeb = false;
            }
        } else {
             WARN("Ignored AEB toggle request (CAN) due to high speed: %.2f km/h", this->getSpeed());
        }
    }
    if (disableAdas) {
        // Reset ADAS back to Idle
        this->resetAutoDrivingState();
        return;
    }
    if (lksState == true) {
        bool newLksState = !this->getStatusBtnLKS();
        this->setValue(STATUS_BTN_LANE_KEEPING_SYSTEM, newLksState);
    }
    if (accState == true) {
        bool newACCState = !this->getStatusBtnACC();
        this->setValue(STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, newACCState);
    }

    // Handle speed controls
    static constexpr float CHANGE_SPEED_LIMIT_SHORT_PRESS = 1.0f;
    static constexpr float CHANGE_SPEED_LIMIT_LONG_PRESS  = 5.0f;
    if (incSpeedLimit != TESLA_EPS_BUTTON_STATUS_EPS_INC_SPEED_LIMIT_DONT_PRESS_CHOICE
        && decSpeedLimit != TESLA_EPS_BUTTON_STATUS_EPS_DEC_SPEED_LIMIT_DONT_PRESS_CHOICE) {
        // Both buttons are pressed, do nothing
    }
    else if (incSpeedLimit == TESLA_EPS_BUTTON_STATUS_EPS_INC_SPEED_LIMIT_SHORT_PRESS_CHOICE) {
        this->setRefVel(this->getRefVel() + CHANGE_SPEED_LIMIT_SHORT_PRESS);
    }
    else if (incSpeedLimit == TESLA_EPS_BUTTON_STATUS_EPS_INC_SPEED_LIMIT_LONG_PRESS_CHOICE) {
        this->setRefVel(this->getRefVel() + CHANGE_SPEED_LIMIT_LONG_PRESS);
    }
    else if (decSpeedLimit == TESLA_EPS_BUTTON_STATUS_EPS_DEC_SPEED_LIMIT_SHORT_PRESS_CHOICE) {
        this->setRefVel(this->getRefVel() - CHANGE_SPEED_LIMIT_SHORT_PRESS);
    }
    else if (decSpeedLimit == TESLA_EPS_BUTTON_STATUS_EPS_DEC_SPEED_LIMIT_LONG_PRESS_CHOICE) {
        this->setRefVel(this->getRefVel() - CHANGE_SPEED_LIMIT_LONG_PRESS);
    }

    // Handle distance control
    static constexpr float CHANGE_DIS_LIMIT_SHORT_PRESS = 1.0f;
    static constexpr float CHANGE_DIS_LIMIT_LONG_PRESS  = 5.0f;
    if (incDisLimit != TESLA_EPS_BUTTON_STATUS_EPS_INC_DIS_LIMIT_DONT_PRESS_CHOICE
        && decDisLimit != TESLA_EPS_BUTTON_STATUS_EPS_DEC_DIS_LIMIT_DONT_PRESS_CHOICE) {
        // Both buttons are pressed, do nothing
    }
    else if (incDisLimit == TESLA_EPS_BUTTON_STATUS_EPS_INC_DIS_LIMIT_SHORT_PRESS_CHOICE) {
        this->setRefDistance(this->getRefDist() + CHANGE_DIS_LIMIT_SHORT_PRESS);
    }
    else if (incDisLimit == TESLA_EPS_BUTTON_STATUS_EPS_INC_DIS_LIMIT_LONG_PRESS_CHOICE) {
        this->setRefDistance(this->getRefDist() + CHANGE_DIS_LIMIT_LONG_PRESS);
    }
    else if (decDisLimit == TESLA_EPS_BUTTON_STATUS_EPS_DEC_DIS_LIMIT_SHORT_PRESS_CHOICE) {
        this->setRefDistance(this->getRefDist() - CHANGE_DIS_LIMIT_SHORT_PRESS);
    }
    else if (decDisLimit == TESLA_EPS_BUTTON_STATUS_EPS_DEC_DIS_LIMIT_LONG_PRESS_CHOICE) {
        this->setRefDistance(this->getRefDist() - CHANGE_DIS_LIMIT_LONG_PRESS);
    }
}


void AdasMain::cbEPSTakeOverStatus(const can_frame& frame)
{
    // Unpack
    tesla_eps_take_over_status_t takeOverStatus;
    tesla_eps_take_over_status_unpack(&takeOverStatus, frame.data, frame.can_dlc);

    // Get values
    bool CONTROL_takeOver = bool(tesla_eps_take_over_status_eps_control_take_over_decode(takeOverStatus.eps_control_take_over));
    bool AEB_takeOver     = bool(tesla_eps_take_over_status_eps_aeb_take_over_decode(takeOverStatus.eps_aeb_take_over));

    // Handle takeOver
    if (CONTROL_takeOver == true) {
        this->resetAutoDrivingState();
    }
    if (AEB_takeOver == true) {
        this->releaseBrakeState();
    }
}


void AdasMain::cbEPSAlertMatrix(const can_frame& frame)
{
    // Unpack
    tesla_eps_alert_matrix_t alertMatrix;
    tesla_eps_alert_matrix_unpack(&alertMatrix, frame.data, frame.can_dlc);

    // Get values
    auto notOkToStartDrive = bool(tesla_eps_alert_matrix_eps_not_ok_to_start_drive_decode(
                                    alertMatrix.eps_not_ok_to_start_drive));

    /**
     * @note Currently only receive message. Logic will be implemented later.
     */
}

void AdasMain::cbEPSDriverControl(const can_frame& frame)
{
    // Unpack
    tesla_eps_driver_control_t epsDriverControl;
    tesla_eps_driver_control_unpack(&epsDriverControl, frame.data, frame.can_dlc);

    // Get values
    float EPS_steeringAngle = float(tesla_eps_driver_control_eps_steering_angle_decode(epsDriverControl.eps_steering_angle));

    // Set eps steering angle for controller
    this->setSteeringAngle(EPS_steeringAngle);
    // INFO("EPS steering_CB: %f", EPS_steeringAngle);
}
#endif // ENABLE_CAN
