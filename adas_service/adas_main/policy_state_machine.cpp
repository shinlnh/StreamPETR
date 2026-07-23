#include "policy_state_machine.h"
#include "algorithm"
#include <fstream>

/**
 * @brief Set the reference to the AdasMain instance.
 *
 * This ensure the nested - class having the ability to access member variables and functions from AdasMain state-machine.
 * Using bare-pointer for reference purpose.
 *
 * @param adas_main_ptr Pointer to the AdasMain instance.
 */
void AdasMain::PolicyState::setMachineRef(AdasMain *adas_main_ptr)
{
    boost::lock_guard<boost::mutex> lock(state_machine_mut);
    this->adas_main_ = adas_main_ptr;
}

// Initialize static logger
LogUtility AdasMain::PolicyState::log("log/", "state_machine");
namespace {
    /**
     * Since we can't call the log.enable() globally like this, this is a work around that creates a lambda
     * and execute it directly. The effect of this block is that the state machine's logger's log to file 
     * functionality is enabled. If you don't want to log to file, simply comment the lambda below.
     * This block is intended to provide a centralized space to control logging location.
    */
    auto logInitializer = [](){
        // AdasMain::enableStateMachineLogFile();
        return 0;
    }();
}

template <typename T>
std::shared_ptr<T> findClosest(const std::vector<std::shared_ptr<T>>& buffer, uint64_t target_timestamp)
{
    // If buffer is empty, return null
    if (buffer.empty()) {
        return nullptr;
    }

    // Iterate and find the closest one to the matching timestamp.
    std::shared_ptr<T> closest = *(buffer.rbegin());
    auto min_time_diff = std::abs(static_cast<int64_t>(closest->time_stamp - target_timestamp));
    for (auto it = buffer.rbegin() + 1; it != buffer.rend(); ++it) {
        auto current_time_diff = std::abs(static_cast<int64_t>((*it)->time_stamp - target_timestamp));
        if (current_time_diff < min_time_diff) {
            closest = *it;
            min_time_diff = current_time_diff;
        }
    }
    return closest;
}

/**
 * @brief The executeState() function processes the behavior of current state of the PolicyState.
 * 
 * It selects a task to execute based on which state, which action is requested,
 * issuing commands for different driving assistance features.
 * 
 * It is the only function that handle task choosing's logic. No function is allowed to modify
 * task switch's logic except this function.
 */
void AdasMain::PolicyState::execute()
{    
    // Cache pointer to AdasMain.
    AdasMain* adas_main_ptr = this->adas_main_;

    // AEB Auto Logic (Speed Limiter)
    // CRITICAL SAFETY: Do active this logic if we are performing AEB Braking action.
    // If we are braking, we must ignore speed threshold to ensure the braking maneuver finishes safely.
    if (this->getId() != BRAKE_STATE_ID) 
    {
        float current_speed = adas_main_ptr->getSpeed();
        // Use lock to ensure thread safety when modifying control states
        boost::lock_guard<boost::mutex> lock(adas_main_ptr->control_state_mutex_);
        if (current_speed > SPEED_AUTO_DISABLE_AEB) {
            if (adas_main_ptr->statusBtnAEB) {
                adas_main_ptr->statusBtnAEB = false;
                adas_main_ptr->auto_disabled_aeb = true;
                WARN("Policy: Auto disable AEB due to speed > (%.2f) km/h (%.2f)", SPEED_AUTO_DISABLE_AEB, current_speed);
            }
        } else {
            // Speed <= SPEED_AUTO_DISABLE_AEB
            if (adas_main_ptr->auto_disabled_aeb) {
                adas_main_ptr->statusBtnAEB = true;
                adas_main_ptr->auto_disabled_aeb = false;
                INFO("Policy: Auto re-enable AEB (speed <= SPEED_AUTO_DISABLE_AEB km/h)");
            }
        }
    }

    // Create queue consumer if not yet
    if (adas_main_ptr->consumer_lane_detection_id_ < 0) {
        adas_main_ptr->consumer_lane_detection_id_ = adas_main_ptr->LaneDetectionsQueue.createConsumer();
    }
    if (adas_main_ptr->consumer_perception_output_objects_id_ < 0) {
        adas_main_ptr->consumer_perception_output_objects_id_ = adas_main_ptr->perceptionOutputQueue.createConsumer();
    }

    // --- Retrieve lane detections first (blocking) ---
    std::queue<std::shared_ptr<LaneDetection>> lane_detection_queue =
        adas_main_ptr->LaneDetectionsQueue.getNext(adas_main_ptr->consumer_lane_detection_id_, true);

    // Consume the entire lane detection queue and take the latest lane.
    std::shared_ptr<LaneDetection> latest_lane = lane_detection_queue.back();

    // Use the latest lane's timestamp as the target for the other streams.
    double target_timestamp = latest_lane->time_stamp;

    // --- Retrieve perception output objects ---
    std::queue<std::shared_ptr<PerceptionOutputObject>> perception_output_objects_queue =
        adas_main_ptr->perceptionOutputQueue.getNext(adas_main_ptr->consumer_perception_output_objects_id_, false);
    while (!perception_output_objects_queue.empty()) {
        if (this->perception_output_objects_queue_.size() >= 10){
            this->perception_output_objects_queue_.erase(this->perception_output_objects_queue_.begin());
        }
        this->perception_output_objects_queue_.push_back(perception_output_objects_queue.front());
        perception_output_objects_queue.pop();
    }

    // --- Retrieve raw source images ---
    std::queue<std::shared_ptr<CameraImageData>> camera_image_data_queue =
        adas_main_ptr->rawSrcImageQueue.getNext(adas_main_ptr->raw_src_cons, false);
    while (!camera_image_data_queue.empty()) {
        if (this->camera_image_data_queue_.size() >= 10) {
            this->camera_image_data_queue_.erase(this->camera_image_data_queue_.begin());
        }
        this->camera_image_data_queue_.push_back(camera_image_data_queue.front());
        camera_image_data_queue.pop();
    }

    // --- Select the closest data from each buffer using the lane's timestamp ---
    std::shared_ptr<LaneDetection> selected_lane = latest_lane;  // Use the latest lane directly.
    std::shared_ptr<PerceptionOutputObject> selected_obj = findClosest(this->perception_output_objects_queue_, target_timestamp);
    std::shared_ptr<CameraImageData> selected_img = findClosest(this->camera_image_data_queue_, target_timestamp);

    // --- Build perception results ---
    PerceptionResults perception_results;
    PerceptionOutputObject perception_output;
    LaneDetection lane_detection;

    // Update image data if available.
    if (selected_img && !selected_img->image.empty()) {
        perception_results.src_image_width  = selected_img->image.cols;
        perception_results.src_image_height = selected_img->image.rows;
        perception_results.src_image = selected_img->image.clone();
        perception_results.time_stamp = selected_img->time_stamp;
        DEBUG("Raw image time: %ld", selected_img->time_stamp);
    }

    // Update object results
    if (selected_obj) {
        perception_output = *selected_obj;
        perception_output.my_car_width = adas_main_ptr->perceptionModel.getCarWidth();
        perception_results.adv_objs.clear();
        // Process detected objects.
        for (const auto &object: perception_output.objects) {
            AdvanceFusionObject temp;
            temp.x_offset = object->location.x();
            temp.y_offset = object->location.y();
            temp.z_offset = object->location.z();
            temp.x_velocity = object->velocity.x();
            temp.y_velocity = object->velocity.y();
            temp.center_point.x = object->location.x();
            temp.center_point.y = object->location.y();
            temp.id_ = object->trackingID;
            temp.mask = object->segment_mask;
            temp.classId = object->classID;
            temp.bbox = { object->bbox[0], object->bbox[1],
                          object->bbox[2], object->bbox[3] };
            perception_results.adv_objs.push_back(temp);
            perception_results.src_objs.push_back(object->sensor_source);
        }
        perception_results.my_car_speed = perception_output.my_car_speed.linear_velocity.y * 3.6;
        perception_results.my_car_width = adas_main_ptr->perceptionModel.getCarWidth();
        DEBUG("Object detection time: %ld", selected_obj->time_stamp);
    }

    // Update lane detection data.
    if (selected_lane) {
        lane_detection = *selected_lane;
        perception_results.laneResult = lane_detection;
        DEBUG( "Lane detection time: %ld", selected_lane->time_stamp);
    }

    // Process if at least one stream has new data.
    if (selected_img && selected_lane) {
        // Check and update lane status
        perception_results.lane_status = adas_main_ptr->perceptionModel.updateLaneStatus(selected_lane);
        bool missingLane = perception_results.lane_status == LaneStatus::MISSING_LANE;

        // Get the distance to closest car in current lane
        auto result = adas_main_ptr->getPlanningModel()->getClosestVehicleInLane(perception_output, lane_detection);
        const auto vehicle_ptr = result.first;
        const float distance_in_lane = result.second;
        if (vehicle_ptr)
            perception_results.closest_car_distance = distance_in_lane;

        // Store data and publish final Perception result
        adas_main_ptr->updateDisableDriveAssist(missingLane);
        adas_main_ptr->setDistanceCaptured(perception_results.closest_car_distance);
        adas_main_ptr->savePerceptionResults(perception_results);
        adas_main_ptr->savePerceptionOutput(perception_output);
        adas_main_ptr->saveLaneDetection(lane_detection);
        adas_main_ptr->perceptionResultsQueue.pushData(0, std::make_shared<PerceptionResults>(perception_results));

        // Count FPS
        static auto prev = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - prev).count();
        prev = now;
        adas_main_ptr->fps = std::round(1e9f / elapsed);

        // If vehicle is not in lane, do not switch LKS on
        bool isInLane = perception_results.lane_status == LaneStatus::IN_LANE;
        if (!isInLane && !this->laneKeepEnabled()) {
            adas_main_ptr->setStatusBtnLKS(false);
        }

        // If gear not in Drive, ADAS is disabled or is in AEB braking, reset mode
        GEAR current_gear = adas_main_ptr->getGear();
        if (current_gear != GEAR::D
            || adas_main_ptr->getDisableDriveAssist()
            || this->getId() == BRAKE_STATE_ID)
        {
            adas_main_ptr->resetAutoDrivingState();
            if (this->laneKeepEnabled() || this->speedKeepEnabled()) {
                if (current_gear != GEAR::D) {
                    INFO("Vehicle not in Drive gear. Handing control over to user...");
                }
                if (adas_main_ptr->getDisableDriveAssist()) {
                    INFO("Driving assist is disabled. Handing control over to user...");            
                }
            }
        }
        
        
        // Reset AEB state when leaving D/N gears to prevent false brake state trigger
        // This prevents: D (brake) → R → N → brake state reappearing
        if (current_gear != GEAR::D && current_gear != GEAR::N) {
            adas_main_ptr->getPlanningModel()->resetAEBState();
        }

        // Check for collision - Only run dangerCheck in D/N gears
        // Skip entirely in P/R to avoid false warning indicators
        bool danger_check = false;
        
        // Only run danger detection if in appropriate gear
        if (current_gear == GEAR::D || current_gear == GEAR::N) {
            danger_check = adas_main_ptr->getPlanningModel()->dangerCheck(perception_output);
            if (!adas_main_ptr->statusBtnAEB) {
                danger_check = false;
                adas_main_ptr->getPlanningModel()->resetAEBState();
            }
        }
        else {
            // P or R gear: Don't run danger check at all
            // This prevents publishing false danger/warning flags
            adas_main_ptr->getPlanningModel()->resetAEBState();
        }


        // Determine if we should enter brake state based on gear
        // AEB module (planner.cpp) handles negative velocity check
        // So we only need to check if in appropriate gear
        bool should_enter_brake_state = false;
        
        if (danger_check) {
            if (current_gear == GEAR::D || current_gear == GEAR::N) {
                // D or N gear: AEB active
                // Negative velocity (backward movement) is handled by AEB module
                should_enter_brake_state = true;
            }
            // P or R gear: Don't trigger AEB (already filtered in execute())
        }
        
        if (should_enter_brake_state) {
            return this->doBrake();
        }






        // Retrieve mode request
        bool laneKeepRequest = adas_main_ptr->getStatusBtnLKS();
        bool speedKeepRequest = adas_main_ptr->getStatusBtnACC();
        bool isTrafficJam = adas_main_ptr->getPlanningModel()->detectTrafficJam(perception_results);
        if (speedKeepRequest && laneKeepRequest && isTrafficJam)
            return this->doTJC();
        if (speedKeepRequest && laneKeepRequest)
            return this->doHWC();
        if (speedKeepRequest && isTrafficJam)
            return this->doTJA();
        if (speedKeepRequest)
            return this->doACC();
        if (laneKeepRequest)
            return this->doLKS();
    }
    return this->doIdle();
}


/* ============== BASE POLICY STATE ============== */
/**
  * @brief Perform switch state to the IdleState.
  */
void AdasMain::PolicyState::doIdle()
{
    INFO("Change from %s to Idle State", this->getStateName());
    this->adas_main_->resetRefValues();
    this->adas_main_->transitionToState(new AdasMain::IdleState);
}

/**
  * @brief Perform switch state to the BrakeState.
  *
  * @note Should log something to indicate a fail in logic if this switch state occurs ?
  */
void AdasMain::PolicyState::doBrake()
{
    INFO("Change from %s to Brake State", this->getStateName());
    this->adas_main_->resetRefValues();
    this->adas_main_->transitionToState(new AdasMain::BrakeState);
}

/**
  * @brief Perform switch state to the LKSState.
  * 
  * Requests AdasMain to switch to new LKSState - which is main state for perform this doLKS() task.
  * Log message to indicate this state change.
  */
void AdasMain::PolicyState::doLKS() 
{
    INFO("Change from %s to Lane Keeping System", this->getStateName());
    this->adas_main_->resetRefValues();
    this->adas_main_->transitionToState(new AdasMain::LKSState);
}

/**
  * @brief Perform switch state to the TJAState.
  * 
  * Requests AdasMain to switch to new TJAState - which is main state for perform this doTJA() task.
  * Log message to indicate this state change.
  */
void AdasMain::PolicyState::doTJA() 
{
    INFO("Change from %s to Traffic Jam Assist", this->getStateName());
    if (this->getId() == IDLE_STATE_ID || this->getId() == LKS_STATE_ID) {
        this->adas_main_->updateRefValues(this->adas_main_->getSpeed(), this->adas_main_->getDistanceCaptured());
    }
    this->adas_main_->transitionToState(new AdasMain::TJAState);
}

/**
  * @brief Perform switch state to the ACCState.
  * 
  * Requests AdasMain to switch to new ACCState - which is main state for perform this doACC() task.
  * Log message to indicate this state change.
  */
void AdasMain::PolicyState::doACC()
{
    INFO("Change from %s to Adaptive Cruise Control", this->getStateName());
    if (this->getId() == IDLE_STATE_ID || this->getId() == LKS_STATE_ID) {
        this->adas_main_->updateRefValues(this->adas_main_->getSpeed(), this->adas_main_->getDistanceCaptured());
    }
    this->adas_main_->transitionToState(new AdasMain::ACCState);
}

/**
  * @brief Perform switch state to the TJCState.
  * 
  * Requests AdasMain to switch to new TJCState - which is main state for perform this doTJC() task.
  * Log message to indicate this state change.
  */
void AdasMain::PolicyState::doTJC() 
{
    INFO("Change from %s to Traffic Jam Chauffeur", this->getStateName());
    if (this->getId() == IDLE_STATE_ID || this->getId() == LKS_STATE_ID) {
        this->adas_main_->updateRefValues(this->adas_main_->getSpeed(), this->adas_main_->getDistanceCaptured());
    }
    this->adas_main_->transitionToState(new AdasMain::TJCState);
}

/**
  * @brief Perform switch state to the HWCState.
  * 
  * Requests AdasMain to switch to new TJAState - which is main state for perform this doHWC() task.
  * Log message to indicate this state change.
  */
void AdasMain::PolicyState::doHWC()
{
    INFO("Change from %s to Highway Chauffeur", this->getStateName());
    if (this->getId() == IDLE_STATE_ID || this->getId() == LKS_STATE_ID) {
        this->adas_main_->updateRefValues(this->adas_main_->getSpeed(), this->adas_main_->getDistanceCaptured());
    }
    this->adas_main_->transitionToState(new AdasMain::HWCState);
}


/* ============== IDLE STATE ============== */
/**
 * @brief Performs IdleState enter function, currently only sets up the top down view.
 * 
 * @param adas_main_ptr 
 */
void AdasMain::IdleState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    adas_main_ptr->controller_ptr = nullptr;
}

/**
  * @brief Doing nothing in IdleState.
  *
  * In doIdle(), don't need to do anything. All preprocessing tasks is handled by `execute()` function (of PolicyState class)
  */
void AdasMain::IdleState::doIdle() {}


/* ============== BRAKE STATE ============== */
/**
 * @brief Performs BrakeState enter function, currently only sets up the top down view.
 * 
 * @param adas_main_ptr 
 */
void AdasMain::BrakeState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    adas_main_ptr->controller_ptr = nullptr;
}

/**
 * @brief Check whether to stop braking or not.
 */
bool AdasMain::BrakeState::checkReleaseAEB()
{
    boost::lock_guard<boost::mutex> lock(state_machine_mut);
    return this->release;
}

/**
 * @brief Notify to release AEB state.
 */
void AdasMain::BrakeState::notifyReleaseAEB()
{
    boost::lock_guard<boost::mutex> lock(state_machine_mut);
    this->release = true;
}

/**
  * @brief Perform switch state from BrakeState to the IdleState.
  */
void AdasMain::BrakeState::doIdle() 
{
    this->doBrake();
}

/**
  * @brief Perform handle doBrake() task.
  */
void AdasMain::BrakeState::doBrake() 
{
    ControlResults control_results = this->adas_main_->controlAEB();
    control_results.limit();
#ifdef ENABLE_CAN
    this->adas_main_->sendCanControlMsg(control_results);
#else
    this->adas_main_->driveAEB(control_results);
#endif // ENABLE_CAN

    // Update AEB status through planning result
    PlanningResults planning_result;
    planning_result.is_danger_aeb = this->adas_main_->getPlanningModel()->isDangerAEB();
    planning_result.is_warning_aeb = this->adas_main_->getPlanningModel()->isWarningAEB();
    this->adas_main_->PlanningResultsQueue.pushData(0, std::make_shared<PlanningResults>(planning_result));

    // Check if vehicle has stopped for atleast 2 seconds
    static constexpr int MAX_FRAME_COUNT = 40; // 2s = 40 frame
    static int releaseFrameCount = -1;
    if (releaseFrameCount < MAX_FRAME_COUNT) {
        if (std::abs(this->adas_main_->getSpeed()) < 0.5f) {
            releaseFrameCount += 1;
        }
        else {
            releaseFrameCount = -1;
        }
        return;
    }

    // Check if AEB button was disabled - immediate exit
    if (!this->adas_main_->statusBtnAEB) {
        releaseFrameCount = -1;
        INFO("AEB button disabled. Exiting brake state...");
        return this->PolicyState::doIdle();
    }

    // Check if received takeover signal from user (gear P/R or throttle press)
    if (this->checkReleaseAEB() == true) {
        releaseFrameCount = -1;
        // Call base implementation to return to idle
        return this->PolicyState::doIdle();
    }

}

/**
  * @brief Perform switch state from BrakeState to the ACCState.
  */
void AdasMain::BrakeState::doLKS()
{
    this->doBrake();
}

/**
  * @brief Perform switch state from BrakeState to the ACCState.
  */
void AdasMain::BrakeState::doACC()
{
    this->doBrake();
}

/**
  * @brief Perform switch state from BrakeState to the TJAState.
  */
void AdasMain::BrakeState::doTJA() 
{
    this->doBrake();
}

/**
  * @brief Perform switch state from BrakeState to the HWCState.
  */
void AdasMain::BrakeState::doHWC() 
{
    this->doBrake();
}

/**
  * @brief Perform switch state from BrakeState to the TJCState.
  */
void AdasMain::BrakeState::doTJC() 
{
    this->doBrake();
}


/* ============== LKS STATE ============== */
/**
 * @brief Performs LKSState enter function, currently only sets up the top down view.
 * 
 * @param adas_main_ptr 
 */
void AdasMain::LKSState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    std::ifstream file("./common/src/inc/lks_controller_params.txt");
    if (file.is_open()) {
        float Q_11, Q_22, Q_33, r_value;
        file >> Q_11 >> Q_22 >> Q_33 >> r_value;
        adas_main_ptr->lks_controller.updateParameters(Q_11, Q_22, Q_33, r_value);
        file.close();
    } else {
        ERROR("Unable to open file 'lks_controller_params.txt'");
    }
    // Initialize controller with current steering to avoid steering jerk on mode transition
    adas_main_ptr->lks_controller.initializeWithSteering(adas_main_ptr->getCachedSteer());
    adas_main_ptr->controller_ptr = &(adas_main_ptr->lks_controller);
}

/**
 * @brief Default handle function for doLKS() task.
 */
void AdasMain::LKSState::doLKS()
{   
    // Grab perception results from AdasMain (Every mode has one common perception results).
    PerceptionOutputObject perception_output = this->adas_main_->getPerceptionOutput();
    LaneDetection lane_detection = this->adas_main_->getLaneDetection();

    //> LKS PLANNING.
    PlanningResults planning_results = this->adas_main_->plan(perception_output, lane_detection, DEFAULT_GENERATION, ONLY_GO_STRAIGHT);

    //> LKS CONTROL.
    ControlConfigs control_configs   = this->adas_main_->createControlConfigs();

    float car_steering_angle =  this->adas_main_->getCachedSteer();
    this->adas_main_->lks_controller.setSteeringAngle(car_steering_angle);
    this->adas_main_->lks_controller.setWheelAngle(perception_output.steering_angle);
    this->adas_main_->lks_controller.setCarAcceleration(perception_output.my_car_state.Imu_linear_acceleration.x);
    ControlResults control_results   = this->adas_main_->controlLKS(planning_results, perception_output.my_car_speed.linear_velocity.y *3.6, control_configs);
    control_results.limit();

    //> DRIVE.
    float steering_value = control_results.steering_value;
    float throttle_value = control_results.throttle_value;
    float brake_value = control_results.brake_value;
#ifdef ENABLE_CAN
    this->adas_main_->sendCanControlMsg(control_results);
#else
    this->adas_main_->driveLKS(control_results);
#endif  // ENABLE_CAN

    //> SAVE.
    this->adas_main_->savePlanningResults(planning_results);
    this->adas_main_->saveControlResults(control_results);
}

/**
  * @brief Perform switch state from LKSState to the TJAState.
  */
void AdasMain::LKSState::doTJA()
{
    WARN("Can not change from LKS State to Traffic Jam Assist");
}

/**
  * @brief Perform switch state from LKSState to the ACCState.
  */
void AdasMain::LKSState::doACC()
{
    WARN("Can not change from LKS State to ACC State");
}


/* ============== TJA STATE ============== */
/**
 * @brief Performs TJAState enter function, currently only sets up the top down view.
 * 
 * @param adas_main_ptr 
 */
void AdasMain::TJAState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    std::ifstream file("./common/src/inc/tja_controller_params.txt");
        if (file.is_open()) {
        float new_ke, new_ke_dot, new_ku, new_a, new_b, new_c ;
        file >> new_ke >> new_ke_dot >> new_ku >> new_a >> new_b >> new_c;
        adas_main_ptr->tja_controller.updateParameters(new_ke, new_ke_dot, new_ku, new_a, new_b, new_c);
        file.close();
    } else {
        ERROR("Unable to open file 'tja_controller_params.txt'");
    }
    // Initialize controller with current throttle to avoid speed drop on mode transition
    adas_main_ptr->tja_controller.initializeWithThrottle(adas_main_ptr->getCachedThrottle());
    adas_main_ptr->controller_ptr = &(adas_main_ptr->tja_controller);
}

/**
  * @brief Perform switch state from TJAState to the LKSState.
  */
void AdasMain::TJAState::doLKS()
{
    WARN("Can not change from TJA State to LKS State");
}

/**
 * @brief Main function for handle doTJA() task.
 */
void AdasMain::TJAState::doTJA()
{
    // Grab perception results from AdasMain (Every mode has one common perception results).
    PerceptionOutputObject perception_output = this->adas_main_->getPerceptionOutput();
    LaneDetection lane_detection = this->adas_main_->getLaneDetection();
    
    // TJA PLANNING.
    PlanningResults planning_results = this->adas_main_->plan(perception_output, lane_detection, DEFAULT_GENERATION, ONLY_GO_STRAIGHT);
    
    //> TJA CONTROL.
    ControlConfigs control_configs   = this->adas_main_->createControlConfigs();
 
    bool vehicle_seen = false;
    std::pair<std::shared_ptr<OutputObject>, float> result = 
    this->adas_main_->getPlanningModel()->getClosestVehicleInLane(perception_output, lane_detection);

    const std::shared_ptr<OutputObject>& vehicle_ptr = result.first;
    float distance_in_lane = result.second;

    if (vehicle_ptr) {
        const OutputObject closest_vehicle_in_lane = *vehicle_ptr;
        if (this->adas_main_->tja_controller.getRefVel() > closest_vehicle_in_lane.velocity.y() * 3.6) { // The condition returns true if the detected object has a velocity smaller than that of our car
            vehicle_seen = true;
            this->adas_main_->tja_controller.setVelCoef(1.0);
            this->adas_main_->tja_controller.setVelCarHead(closest_vehicle_in_lane.velocity.y() * 3.6);
            this->adas_main_->tja_controller.setDisCarHead(distance_in_lane);
        }
    }
    
    this->adas_main_->tja_controller.setDetectedVehicle(vehicle_seen);
    ControlResults control_results = this->adas_main_->controlTJA(planning_results, perception_output.my_car_speed.linear_velocity.y *3.6, control_configs);
    control_results.limit();

    //> DRIVE.
    float steering_value = control_results.steering_value;
    float throttle_value = control_results.throttle_value;
    float brake_value = control_results.brake_value;

#ifdef ENABLE_CAN
    this->adas_main_->sendCanControlMsg(control_results);
#else
    this->adas_main_->driveTJA(control_results);   
#endif  // ENABLE_CAN

    //> SAVE.
    this->adas_main_->savePlanningResults(planning_results);
    this->adas_main_->saveControlResults(control_results);
}



/* ============== ACC STATE ============== */
/**
 * @brief Performs ACCState enter function, currently only sets up the top down view.
 * 
 * @param adas_main_ptr 
 */
void AdasMain::ACCState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    std::ifstream file("./common/src/inc/acc_controller_params.txt");
        if (file.is_open()) {
        float new_ke, new_ke_dot, new_ku, new_a, new_b, new_c;
        file >> new_ke >> new_ke_dot >> new_ku >> new_a >> new_b >> new_c;
        adas_main_ptr->acc_controller.updateParameters(new_ke, new_ke_dot, new_ku, new_a, new_b, new_c);
        file.close();
    } else {
        ERROR("Unable to open file 'acc_controller_params.txt'");
    }
    // Initialize controller with current throttle to avoid speed drop on mode transition
    adas_main_ptr->acc_controller.initializeWithThrottle(adas_main_ptr->getCachedThrottle());
    adas_main_ptr->controller_ptr = &(adas_main_ptr->acc_controller);
}

void AdasMain::ACCState::doLKS()
{
    WARN("Can not change from ACC State to LKS State");
}

void AdasMain::ACCState::doACC()
{
    // Grab perception results from AdasMain (Every mode has one common perception results).
    PerceptionOutputObject perception_output = this->adas_main_->getPerceptionOutput();
    LaneDetection lane_detection = this->adas_main_->getLaneDetection();
    
    // ACC PLANNING.
    PlanningResults planning_results = this->adas_main_->plan(perception_output, lane_detection, DEFAULT_GENERATION, ONLY_GO_STRAIGHT);
    
    //> ACC CONTROL.
    ControlConfigs control_configs   = this->adas_main_->createControlConfigs();

    bool vehicle_seen = false;
    std::pair<std::shared_ptr<OutputObject>, float> result = 
    this->adas_main_->getPlanningModel()->getClosestVehicleInLane(perception_output, lane_detection);

    const std::shared_ptr<OutputObject>& vehicle_ptr = result.first;
    float distance_in_lane = result.second;

    if (vehicle_ptr) {
        const OutputObject closest_vehicle = *vehicle_ptr;
        if (this->adas_main_->acc_controller.getRefVel() > closest_vehicle.velocity.y()*3.6) { // The condition returns true if the detected object has a velocity smaller than that of our car
            vehicle_seen = true;
            this->adas_main_->acc_controller.setVelCoef(1.0);
            this->adas_main_->acc_controller.setVelCarHead(closest_vehicle.velocity.y()*3.6);
            this->adas_main_->acc_controller.setDisCarHead(distance_in_lane);
        }
    }

    this->adas_main_->acc_controller.setDetectedVehicle(vehicle_seen);
    ControlResults control_results = this->adas_main_->controlACC(planning_results, perception_output.my_car_speed.linear_velocity.y *3.6, control_configs);
    control_results.limit();

    //> DRIVE.
    float steering_value = control_results.steering_value;
    float throttle_value = control_results.throttle_value;
    float brake_value = control_results.brake_value;
#ifdef ENABLE_CAN
    this->adas_main_->sendCanControlMsg(control_results);
#else
    this->adas_main_->driveACC(control_results);
#endif  // ENABLE_CAN

    //> SAVE.
    this->adas_main_->savePlanningResults(planning_results);
    this->adas_main_->saveControlResults(control_results);
}


/* ============== HWC STATE ============== */
void AdasMain::HWCState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    std::ifstream file("./common/src/inc/hwc_controller_params.txt");
    if (file.is_open()) {
        float new_heading_exponent, new_velocity_exponent, new_acceleration_exponent, new_total_exponent, new_velocity_weight;
        file >> new_heading_exponent >> new_velocity_exponent >> new_acceleration_exponent >> new_total_exponent >> new_velocity_weight;
        adas_main_ptr->hwc_controller.updateParameters(new_heading_exponent, new_velocity_exponent, new_acceleration_exponent, new_total_exponent, new_velocity_weight);
        file.close();
    } else {
        ERROR("Unable to open file 'hwc_controller_params.txt'");
    }
    // Initialize ACC controller (used internally by HWC) with current throttle to avoid speed drop
    adas_main_ptr->acc_controller.initializeWithThrottle(adas_main_ptr->getCachedThrottle());
    // Initialize LKS controller (used internally by HWC) with current steering to avoid steering jerk
    adas_main_ptr->lks_controller.initializeWithSteering(adas_main_ptr->getCachedSteer());
    adas_main_ptr->controller_ptr = &(adas_main_ptr->hwc_controller);
}

void AdasMain::HWCState::doHWC()
{
    // Grab perception results from AdasMain (Every mode has one common perception results).
    PerceptionOutputObject perception_output = this->adas_main_->getPerceptionOutput();
    LaneDetection lane_detection = this->adas_main_->getLaneDetection();
    
    // HWC PLANNING.
    PlanningResults planning_results = this->adas_main_->plan(perception_output, lane_detection, DEFAULT_GENERATION, ONLY_GO_STRAIGHT);

    //> HWC CONTROL.
    ControlConfigs control_configs   = this->adas_main_->createControlConfigs(); 
    
    bool vehicle_seen = false;
    std::pair<std::shared_ptr<OutputObject>, float> result = 
    this->adas_main_->getPlanningModel()->getClosestVehicleInLane(perception_output, lane_detection);

    const std::shared_ptr<OutputObject>& vehicle_ptr = result.first;
    float distance_in_lane = result.second;

    if (vehicle_ptr) {
        const OutputObject closest_vehicle = *vehicle_ptr;
        if (this->adas_main_->hwc_controller.getRefVel() > closest_vehicle.velocity.y()*3.6) { // The condition returns true if the detected object has a velocity smaller than that of our car
            vehicle_seen = true;
            this->adas_main_->hwc_controller.setVelCarHead(closest_vehicle.velocity.y()*3.6);
            this->adas_main_->hwc_controller.setDisCarHead(distance_in_lane);
        }
    }
    
    this->adas_main_->hwc_controller.setCarAcceleration(perception_output.my_car_state.Imu_linear_acceleration.x);
    this->adas_main_->hwc_controller.setDetectedVehicle(vehicle_seen);

    float car_steering_angle =  this->adas_main_->getCachedSteer();
    this->adas_main_->hwc_controller.setSteeringAngle(car_steering_angle);
    this->adas_main_->hwc_controller.setWheelAngle(perception_output.steering_angle);
    ControlResults control_results = this->adas_main_->controlHWC(planning_results, perception_output.my_car_speed.linear_velocity.y *3.6, control_configs);
    control_results.limit();

    //> DRIVE.
    float steering_value = control_results.steering_value;
    float throttle_value = control_results.throttle_value;
    float brake_value = control_results.brake_value;
#ifdef ENABLE_CAN
    this->adas_main_->sendCanControlMsg(control_results);
#else
    this->adas_main_->drive(control_results);   
#endif  // ENABLE_CAN

    //> SAVE
    this->adas_main_->savePlanningResults(planning_results);
    this->adas_main_->saveControlResults(control_results);
}


/* ============== TJC STATE ============== */
void AdasMain::TJCState::enter(AdasMain *adas_main_ptr)
{
    DEBUG("Performing enter method...");
    std::ifstream file("./common/src/inc/tjc_controller_params.txt");
    if (file.is_open()) {
        float new_heading_exponent, new_velocity_exponent, new_acceleration_exponent, new_total_exponent, new_velocity_weight;
        file >> new_heading_exponent >> new_velocity_exponent >> new_acceleration_exponent >> new_total_exponent >> new_velocity_weight;
        adas_main_ptr->tjc_controller.updateParameters(new_heading_exponent, new_velocity_exponent, new_acceleration_exponent, new_total_exponent, new_velocity_weight);
        file.close();
    } else {
        ERROR("Unable to open file 'tjc_controller_params.txt'");
    }
    // Initialize TJA controller (used internally by TJC) with current throttle to avoid speed drop
    adas_main_ptr->tja_controller.initializeWithThrottle(adas_main_ptr->getCachedThrottle());
    // Initialize LKS controller (used internally by TJC) with current steering to avoid steering jerk
    adas_main_ptr->lks_controller.initializeWithSteering(adas_main_ptr->getCachedSteer());
    adas_main_ptr->controller_ptr = &(adas_main_ptr->tjc_controller);
}

void AdasMain::TJCState::doTJC()
{
    // Grab perception results from AdasMain (Every mode has one common perception results).
    PerceptionOutputObject perception_output = this->adas_main_->getPerceptionOutput();
    LaneDetection lane_detection = this->adas_main_->getLaneDetection();

    //> TJC PLANNING.
    PlanningResults planning_results = this->adas_main_->plan(perception_output, lane_detection, DEFAULT_GENERATION, ONLY_GO_STRAIGHT);

    //> TJC CONTROL.
    ControlConfigs control_configs   = this->adas_main_->createControlConfigs();
    
    bool vehicle_seen = false;

    std::pair<std::shared_ptr<OutputObject>, float> result = 
    this->adas_main_->getPlanningModel()->getClosestVehicleInLane(perception_output, lane_detection);

    const std::shared_ptr<OutputObject>& vehicle_ptr = result.first;
    float distance_in_lane = result.second;


    if (vehicle_ptr) {
        const OutputObject closest_vehicle_in_lane = *vehicle_ptr;
        if (this->adas_main_->tjc_controller.getRefVel() > closest_vehicle_in_lane.velocity.y() * 3.6) { // The condition returns true if the detected object has a velocity smaller than that of our car
            vehicle_seen = true;
            this->adas_main_->tjc_controller.setVelCarHead(closest_vehicle_in_lane.velocity.y() * 3.6);
            this->adas_main_->tjc_controller.setDisCarHead(distance_in_lane);
        }
    }

    this->adas_main_->tjc_controller.setCarAcceleration(perception_output.my_car_state.Imu_linear_acceleration.x);
    this->adas_main_->tjc_controller.setDetectedVehicle(vehicle_seen);
    float car_steering_angle =  this->adas_main_->getCachedSteer();
    this->adas_main_->tjc_controller.setSteeringAngle(car_steering_angle);
    this->adas_main_->tjc_controller.setWheelAngle(perception_output.steering_angle);
    ControlResults control_results = this->adas_main_->controlTJC(planning_results, perception_output.my_car_speed.linear_velocity.y *3.6, control_configs);
    control_results.limit();

    //> DRIVE.
    float steering_value = control_results.steering_value;
    float throttle_value = control_results.throttle_value;
    float brake_value = control_results.brake_value;
#ifdef ENABLE_CAN
    this->adas_main_->sendCanControlMsg(control_results);
#else
    this->adas_main_->drive(control_results);   
#endif  // ENABLE_CAN

    //> SAVE.
    this->adas_main_->savePlanningResults(planning_results);
    this->adas_main_->saveControlResults(control_results);
}