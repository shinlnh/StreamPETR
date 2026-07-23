#ifndef ADAS_MAIN_H
#define ADAS_MAIN_H

#include <deque>
#include <map>
#include <unordered_map>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <semaphore.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <zlib.h>

#include "tcp_ip_streaming/tcp_ip_server.h"
#include "RtspServer.hpp"
#include "ReceiverUdp.hpp"
#include "ReceiverGTCP.hpp"
#include "ReceiverRtsp.hpp"
#include "adas_service_if.h"
#include "perception.h"
#include "planner.h"
#include "lks_controller.h"
#include "tja_controller.h"
#include "acc_controller.h"
#include "tjc_controller.h"
#include "hwc_controller.h"
#include "ctpl_stl.h"
#include "planning_queue.h"
#include "logger/general_logger.h"
#include "debug_to_dumplog.h"
#include "general_maths.h"
#include "common.h"
#ifdef ENABLE_CAN
#include "can_manager/can_manager.h"
#include "tesla.h"      // This library use for CAN message. Comment this line if don't integrate CAN
#endif


// Define size parameters of the original image received from the ROS bridge
#define ORININAL_IMAGE_WIDTH  800  // in pixels
#define ORININAL_IMAGE_HEIGHT 800  // in pixels

// Policy state machine defines
#define HIGH_SPEED_MODE_THRESHOLD 40  // km/h

// Reference Control Limits
#define MIN_REF_SPEED_KMH 5.0f   // Minimum reference speed in km/h
#define MAX_REF_SPEED_KMH 120.0f // Maximum reference speed in km/h
#define MIN_REF_DISTANCE_M   5.0f   // Minimum reference distance in meters
#define MAX_REF_DISTANCE_M   60.0f  // Maximum reference distance in meters
#define SPEED_AUTO_DISABLE_AEB 100.0f // km/h

enum PolicyStateID : int {
    IDLE_STATE_ID = 1,
    BRAKE_STATE_ID = 2,
    LKS_STATE_ID = 3,
    TJA_STATE_ID = 4,
    ACC_STATE_ID = 5,
    HWC_STATE_ID = 6,
    TJC_STATE_ID = 7
};

template<typename T>
class ThreadSafeQueue {
public:
    bool push(const T& item) {
        bool is_unbalanced = false;
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(item);
        if (queue_.size() > 2) {
            queue_.pop();
            WARN("queue size is %ld of type %s please ensure the consumer is fast enough", queue_.size(), type.c_str());
            is_unbalanced = true;
        }
        // cv_.notify_one();
        return is_unbalanced;
    }

    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mtx_);
        // cv_.wait(lock, [this] { return !queue_.empty(); });
        if (queue_.empty())
            return false;
        item = queue_.front();
        queue_.pop();
        return true;
    }

    size_t size() {
        return queue_.size();
    }

private:
    std::queue<T> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
public:
    std::string type;
};

template <typename T>
class DataSynchronizerQueue {
private:
    size_t expectedSensors_;
    boost::chrono::nanoseconds timeout_;
    boost::mutex mtx_;
    boost::condition_variable cv_;
    sem_t semaphore_;

    std::unordered_map<int, bool> hasData_;
    std::unordered_map<int, std::shared_ptr<T>> pendingData_;
    std::queue<std::shared_ptr<T>> outputQueue_;

    bool combineData() {
        if (pendingData_.size() == 0) return false;

        for (auto& entry : pendingData_) {
            outputQueue_.push(entry.second);
        }
        reset(); 
        return true;
    }

    void reset() {
        for(auto& entry : hasData_)
        {
            entry.second = false;
        }
        pendingData_.clear();
    }

public:
    DataSynchronizerQueue(size_t expectedSensors = 2, boost::chrono::milliseconds timeout = boost::chrono::milliseconds(50))
        : expectedSensors_(expectedSensors), timeout_(timeout) {
        // Allow up to the expected number of sensor inputs
        sem_init(&semaphore_, 0, expectedSensors_); 

        // Preallocate
        hasData_.reserve(expectedSensors_);

        for (size_t i = 0; i < expectedSensors_; ++i)
        {
            hasData_[i] = false;
        }
    }

    ~DataSynchronizerQueue() {
        sem_destroy(&semaphore_);
    }

    // Push data into the synchronizer
    // Block if too many sensors are pushing data
    void pushData(int sensorId, std::shared_ptr<T> data) 
    {
        sem_wait(&semaphore_); 
        boost::unique_lock<boost::mutex> lock(mtx_);

        if (hasData_[sensorId] == true) {
            ERROR("Warning: Radar data for sensorId %d is already coming.", sensorId);

        }

        pendingData_[sensorId] = data;
        hasData_[sensorId] = true;
        // Wait for all expected sensor data or timeout
        cv_.wait_for(lock, timeout_, [&] { return pendingData_.size() >= expectedSensors_;});
        bool processed = combineData(); // All sensors provided data within the timeout

        sem_post(&semaphore_); // Release the semaphore
        if(processed) cv_.notify_one();      // Notify others that new data has been processed
    }

    // Retrieve the next combined data
    std::queue<std::shared_ptr<T>> getNext() {
        boost::unique_lock<boost::mutex> lock(mtx_);

        cv_.wait(lock, [&]() { return !outputQueue_.empty(); });
        std::queue<std::shared_ptr<T>> return_queue;
        while(!outputQueue_.empty())
        {
            return_queue.push(std::dynamic_pointer_cast<T>(outputQueue_.front()->clone()));
            outputQueue_.pop();
        }
        return return_queue;
    }
};

template <typename T>
class DataSharingQueue {
private:
    size_t expectedSensors_;
    boost::chrono::nanoseconds timeout_;
    boost::mutex mtx_;
    boost::condition_variable cv_;
    sem_t semaphore_;

    bool block_if_empty_;

    std::unordered_map<int, bool> hasData_;
    std::unordered_map<int, std::shared_ptr<T>> pendingData_;

    // Each consumer is identified by an integer.
    static int consumers;
    std::unordered_map<int, std::queue<std::shared_ptr<T>>> subscribers_;
    unsigned int max_size_subscriber_;

    bool combineData() {
        if (pendingData_.empty()) return false;
        // Distribute each sensor's data to all registered consumers.
        for (auto& entry : pendingData_) {
            for (auto& sub : subscribers_) {
                // sub is a pair (consumerId, queue), so push to the queue:
                sub.second.push(entry.second);
                if (sub.second.size() > max_size_subscriber_) {
                    sub.second.pop();
                }
            }
        }
        reset();
        return true;
    }

    void reset() {
        for(auto& entry : hasData_) {
            entry.second = false;
        }
        pendingData_.clear();
    }

public:
    /* Attribute */
    std::string type = "";

    /* Method */
    DataSharingQueue(size_t expectedSensors = 2,
                          boost::chrono::milliseconds timeout = boost::chrono::milliseconds(1),
                          unsigned int max_size_subscriber = 100)
        : expectedSensors_(expectedSensors), timeout_(timeout), max_size_subscriber_(max_size_subscriber)
    {
        // Allow up to expectedSensors threads to push concurrently.
        sem_init(&semaphore_, 0, expectedSensors_); 

        // Preallocate the hasData_ map with sensor IDs 0 .. expectedSensors_-1.
        hasData_.reserve(expectedSensors_);
        for (size_t i = 0; i < expectedSensors_; ++i) {
            hasData_[i] = false;
        }

        // Initiale with non blocking mode
        block_if_empty_ = false;
    }

    ~DataSharingQueue() {
        sem_destroy(&semaphore_);
    }

    // Create a new consumer (subscriber) and return its consumer id.
    int createConsumer() {
        boost::unique_lock<boost::mutex> lock(mtx_);
        int consumerId = ++DataSharingQueue::consumers;
        subscribers_[consumerId] = std::queue<std::shared_ptr<T>>();
        return consumerId;
    }

    // Return the total number of consumers.
    int getConsumer() {
        return subscribers_.size();
    }

    // Push sensor data into the synchronizer.
    // Blocks (via a semaphore) if too many sensors are pushing concurrently.
    void pushData(int sensorId, std::shared_ptr<T> data) {
        sem_wait(&semaphore_); 
        boost::unique_lock<boost::mutex> lock(mtx_);

        if (DataSharingQueue::consumers <= 0) {
            DEBUG("Have no consumer in Data Sharing Queue");
        }

        if (hasData_[sensorId] == true) {
            DEBUG("Warning: Data for sensorId %d is already coming.", sensorId);
        }

        pendingData_[sensorId] = data;
        hasData_[sensorId] = true;
        // Wait for either all expected sensor data to arrive or until timeout.
        cv_.wait_for(lock, timeout_, [&] { return pendingData_.size() >= expectedSensors_; });
        bool processed = combineData();
        sem_post(&semaphore_); // Release the semaphore.
        if (processed)
            cv_.notify_all(); // Notify all waiting consumers.
    }

    // Retrieve the next combined set of data for the given consumer.
    // This function will block until there is data available in the consumer's queue.
    std::queue<std::shared_ptr<T>> getNext(int consumerId, bool block = true, bool drop = true) {
        boost::unique_lock<boost::mutex> lock(mtx_);
        if(block) cv_.wait(lock, [&]() { return !subscribers_[consumerId].empty(); });
        std::queue<std::shared_ptr<T>> return_queue = subscribers_[consumerId];
        while (drop && !subscribers_[consumerId].empty()) {
            subscribers_[consumerId].pop();
        }
        return return_queue;
    }

    /**
     * @brief Unregister customer
     * @param customer_id
     * @return status
     */
    bool unregisterCustomer(int customer_id) {
        boost::lock_guard<boost::mutex> lock(mtx_);
        return subscribers_.erase(customer_id);
    }

    void setBlockIfEmpty(const bool &enable) {
        boost::unique_lock<boost::mutex> lock(mtx_);
        block_if_empty_ = enable;
    }
    
    /**
     * @brief Get number of element contained in queue of customer
     */
    int sizeOf(int customer_id) {
        boost::lock_guard<boost::mutex> lock(mtx_);
        return subscribers_[customer_id].size();
    }
};

/**
 * @brief This is a thread-safe ring buffer that only allow adding new data.
 * Processes can view and take a snapshot for internal use without affecting
 * others.
 */
template<typename T, size_t _size>
class DataSharingPool
{
protected:
    std::array<std::shared_ptr<const T>, _size> buffer;
    mutable boost::shared_mutex mut;
    int head_idx = -1;
    bool filled = false;

    /**
     * @brief This is the core function to reset the buffer's status.
     * @warning This function is NOT Thread-safe!!! It is meant for internal
     * use by other member function that is thread-safe (already got lock).
     * !!! DO NOT MAKE IT PUBLIC !!!
     */
    virtual void _reset()
    {
        this->head_idx = -1;
        this->filled = false;
    }

    /**
     * @brief This is the core function to take snapshot of the buffer.
     * @warning This function is NOT Thread-safe!!! It is meant for internal
     * use by other member function that is thread-safe (already got lock).
     * !!! DO NOT MAKE IT PUBLIC !!!
     */
    std::vector<std::shared_ptr<const T>> _snapshot() const
    {
        std::vector<std::shared_ptr<const T>> result;
        result.reserve(_size);

        // Copy older data first
        if (this->filled) {
            auto start_it = this->buffer.begin() + this->head_idx + 1;
            auto end_it = this->buffer.end();
            result.insert(result.end(), start_it, end_it);
        }
        
        // Copy newer data
        auto start_it = this->buffer.begin();
        auto end_it = this->buffer.begin() + this->head_idx + 1;
        result.insert(result.end(), start_it, end_it);

        return result;
    }

public:
    DataSharingPool() = default;
    virtual ~DataSharingPool() = default;

    size_t size() const {
        if (this->filled) {
            return _size;
        }
        else {
            return this->head_idx +1 ;
        }
    }

    virtual void push(const T &data)
    {
        this->push(std::make_shared<const T>(data));
    }
    
    virtual void push(std::shared_ptr<const T> data_ptr)
    {
        boost::unique_lock<boost::shared_mutex> lock(this->mut);
        this->head_idx++;
        if (this->head_idx == _size) {
            this->head_idx = 0;
            this->filled = true;
        }
        this->buffer.at(this->head_idx) = data_ptr;
    }

    virtual std::vector<std::shared_ptr<const T>> snapshot() const
    {
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        return this->_snapshot();
    }

    virtual std::shared_ptr<const T> at(size_t index) const
    {
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        return this->buffer.at(index % _size);
    }

    virtual std::shared_ptr<const T> latest() const
    {
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        if (this->head_idx == -1) {
            return nullptr;
        }
        return this->buffer.at(this->head_idx);
    }

    virtual std::shared_ptr<const T> oldest() const
    {
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        size_t tail_idx = 0;
        if (this->filled) {
            tail_idx = this->head_idx + 1;
            if (tail_idx == _size) {
                tail_idx = 0;
            }
        }
        return this->buffer.at(tail_idx);
    }
};

/**
 * @details Specialized version of DataSharingPool for messages with time_stamp.
 * Add function to wait and grab snapshot when message reach certain time point.
 */
template<typename T, size_t _size>
class MessageSharingPool : public DataSharingPool<T, _size>
{
private:
    mutable boost::condition_variable_any cv;
    boost::chrono::milliseconds timeout_;
    uint64_t latest_timestamp = 0;
    
public:
    MessageSharingPool(boost::chrono::milliseconds timeout = boost::chrono::milliseconds(50))
    : timeout_(timeout) {}
    ~MessageSharingPool() = default;

    uint64_t getTimestamp() const
    {
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        return this->latest_timestamp;
    }
    
    using DataSharingPool<T, _size>::push;

    void push(std::shared_ptr<const T> data_ptr) override
    {
        boost::unique_lock<boost::shared_mutex> lock(this->mut);
        if (data_ptr->time_stamp < latest_timestamp) {
            // Check if need to reset pool (due to Carla restart)
            static uint64_t reset_thresh = 10e9;
            int64_t timestamp_delta = int64_t(latest_timestamp) - int64_t(data_ptr->time_stamp);
            if (timestamp_delta > reset_thresh) {
                this->_reset();
            }
            else {
                WARN("Outdated data! Dropping!");
                return;
            }
        }
        this->latest_timestamp = data_ptr->time_stamp;

        this->head_idx++;
        if (this->head_idx == _size) {
            this->head_idx = 0;
            this->filled = true;
        }
        this->buffer.at(this->head_idx) = data_ptr;
        this->cv.notify_all();
    }

    std::vector<std::shared_ptr<const T>> snapshot_ts(uint64_t timestamp) const
    {
        return this->snapshot_ts(timestamp, this->timeout_);
    }

    std::vector<std::shared_ptr<const T>> snapshot_ts(
        uint64_t timestamp,
        boost::chrono::milliseconds timeout
    ) const
    {
        // Wait for message before taking snapshot
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        if (timeout == boost::chrono::milliseconds(0)) {
            this->cv.wait(lock, [&]() {
                return this->latest_timestamp >= timestamp && !this->buffer.empty();
            });
        }
        else {
            this->cv.wait_for(lock, timeout, [&]() {
                return this->latest_timestamp >= timestamp && !this->buffer.empty();
            });
        }
        return this->_snapshot();
    }
    
    std::shared_ptr<const T> at_ts(uint64_t timestamp, bool no_null = true) const
    {
        std::shared_ptr<const T> msg = this->at_ts(timestamp, this->timeout_, no_null);
        return msg;
    }
    
    std::shared_ptr<const T> at_ts(
        uint64_t timestamp,
        boost::chrono::milliseconds timeout,
        bool no_null = true
    ) const
    {
        std::shared_ptr<const T> msg = nullptr;

        // Get snapshot of messages at the timestamp
        auto messages = this->snapshot_ts(timestamp, timeout);
        
        // Find the first one that has timestamp equal or greater than required
        auto itr = std::lower_bound(messages.begin(), messages.end(), timestamp,
            [](const auto& item, uint64_t ts) { return item->time_stamp < ts; });

        // Return if found, if not found then last one if not empty, else default if no null
        if (itr != messages.end()) {
            msg = *(itr);
        }
        else if (!messages.empty()) {
            msg = messages.back();
        }
        else if (no_null) {
            msg = std::make_shared<const T>();
        }

        return msg;
    }

    std::shared_ptr<const T> latest_ts(uint64_t timestamp, bool no_null = true) const
    {
        std::shared_ptr<const T> msg = this->latest_ts(timestamp, this->timeout_, no_null);
        return msg;
    }

    std::shared_ptr<const T> latest_ts(
        uint64_t timestamp,
        boost::chrono::milliseconds timeout,
        bool no_null = true
    ) const
    {
        boost::shared_lock<boost::shared_mutex> lock(this->mut);
        if (timeout == boost::chrono::milliseconds(0)) {
            this->cv.wait(lock, [&]() {
                return this->latest_timestamp >= timestamp && !this->buffer.empty();
            });
        }
        else {
            this->cv.wait_for(lock, timeout, [&]() {
                return this->latest_timestamp >= timestamp && !this->buffer.empty();
            });
        }
        if (this->head_idx == -1) {
            if (no_null) {
                return std::make_shared<const T>();
            }
            return nullptr;
        }
        return this->buffer.at(this->head_idx);
    }
};

// Initialize the static consumer counter.
template <typename T>
int DataSharingQueue<T>::consumers = 0;

typedef enum boolObjects_t {
    STATUS_BTN_CTRL,
    STATUS_BTN_CALIB,
    STATUS_BTN_PARAM,
    STATUS_BTN_CAPTURE,
    STATUS_BTN_LANE_GRAYSCALE,
    STATUS_BTN_SLIDING_WINDOW,
    STATUS_BTN_LOCALIZATION,
    STATUS_BTN_LANE_DETECT,
    STATUS_BTN_OBJ_ENABLE,
    STATUS_BTN_OBJ_CLASS,
    STATUS_BTN_OBJ_DISTANCE,
    STATUS_BTN_UNDISTOR,
    STATUS_BTN_SEND_CONTROL_MSG,
    STATUS_BTN_START_SERVER,
    STATUS_BTN_LANE_KEEPING_SYSTEM,
    STATUS_BTN_ADAPTIVE_CRUISE_CONTROL,
    STATUS_BTN_AUTO_EMERGENCY_BRAKING,
    STATUS_BTN_DETACH_AUTO_BRAKE
}boolObjects;

typedef enum intObjects_t {
    VALUE_SLIDER_STEER,
    VALUE_LINE_EDIT_STEER,
    VALUE_LINE_EDIT_THROTTLE,
    VALUE_SLIDER_THROTTLE,
    VALUE_LINE_EDIT_BRAKE,
    VALUE_SLIDER_BRAKE
} intObjects;

enum class controlMsgAttributeType
{
    ATTRIBUTE_BRAKE,
    ATTRIBUTE_STEER,
    ATTRIBUTE_THROTTLE,
};


class AdasMain
{
private:
    // AutoDrivingStateHandle* auto_drive_state_ui_handler;
    // Select tab.
    bool statusBtnCtrl = false;
    bool statusBtnCalib = false;
    bool statusBtnParam = false;
    // Image processing.
    bool statusBtnCapture = false;
    // Undistortion
    bool statusBtnUndistor = false;
    // Lane tracking.
    bool statusBtnLaneGrayscale = false;
    bool statusBtnSlidingWindow = false;
    bool statusBtnLocalization = false;
    bool statusBtnLaneDetect = false;
    bool statusBtnLaneDetectPre = false;
    // Object detection.
    bool statusBtnObjEnable = false;
    bool statusBtnObjClass = false;
    bool statusBtnObjDistance = false;
    // Path planning's section.
    int valueSliderSteer = 0;
    int valueLineEditSteer = 0;
    int valueLineEditThrottle = 0;
    int valueSliderThrottle = 0;
    int valueLineEditBrake = 0;
    int valueSliderBrake = 0;
    bool statusBtnSendControlMsg = false;
    bool statusBtnStartServer = false;
    // Path tracking's section.
    bool statusBtnLKS = false;
    bool statusBtnACC = false;
    bool statusBtnAEB = true;
    bool statusBtnDetachAutoBrake = false;
    // Loop rate time measured 
    long long execute_duration;
    
    // Distance Capture
    float distance_captured = 0.0f;

    // Auto Disabled AEB flag
    bool auto_disabled_aeb = false;

    // Cached Control Values
    float cached_throttle_value = 0.0f;
    float cached_brake_value = 0.0f;
    float cached_steer_value = 0.0f;

    // Debug image variable
    boost::mutex debug_img_mut;
    bool new_img = false;
    cv::Mat debug_img;

    // Master thread processing variables
    boost::mutex is_running_mut;
    boost::thread *master_thread_handler;

    // Event handler thread variables
    boost::thread *capture_thread_handler;
    boost::thread *keyboard_thread_handler;
    boost::thread *carla_car_status_receiving_thread_handler;
    boost::thread *sending_thread_handler;
    boost::thread *udp_sending_handler;
    boost::thread *rtsp_server_thread_handler;

    // Perception visualize thread variables
    boost::thread *perception_visualize_thread_handler;
    boost::condition_variable perception_visualize_thread_trigger;
    boost::mutex perception_visualize_thread_mut;

    // Policy state machine processing thread variables
    boost::thread *policy_state_thread_handler;
    boost::condition_variable policy_state_thread_trigger;
    boost::mutex policy_state_thread_mut;
    boost::mutex policy_transition_state_mut;
    boost::mutex policy_state_mut;

    // Perception module processing thread variables
    boost::thread *perception_processing_thread_handler;

    // Tracker module, object detection, and lane detection threads  
    boost::thread *radar_tracker_thread;
    boost::thread *camera_tracker_thread;
    boost::thread *object_detection_thread;
    boost::thread *lane_detection_thread;

    //
    boost::mutex control_state_mutex_;
    
    //> Relating functions hold by thread handlers
    // Master thread
    void masterThread(void);

    // Tracker module, object detection, and lane detection thread execution functions
    void radarTrackerThread(void);
    void cameraTrackerThread(void);
    void objectDetectionThread(void);
    void laneDetectionThread(void);

    // Perception processing thread
    void PerceptionProcessingThread(void);

    // Event handler thread
    void captureCameraThread(std::string path);
    void captureVideoThread(std::string path);
    void captureTcpRemoteResourceThread(std::string url);
    void captureUdpRemoteResourceThread(std::string url);
    void captureGtcpRemoteResourceThread(std::string url);

    ReceiverUdp* receiver_app_handler;

    Perception perceptionModel;

    void perceptionResultsPublishThread();
    void packVisualizationMessage(ipc_helper::msg::PipelineResults &msg,
                                  PerceptionResults const &perception_result,
                                  PlanningResults const &planning_result);
    void packSFFDebugMessage(ipc_helper::msg::SFFDebugData &msg,
                             const SafetyForceField &sff);

    Planner planner_;

    PlanningQueue planning_queue;
    // Controller section
    
    // Loggers
    static LogUtility cb_camera;
    static LogUtility cb_carla_status_logger;
    static LogUtility cb_radar_logger;
    static LogUtility cb_gnss_logger;
    static LogUtility cb_imu_logger;
    static LogUtility cb_odometer_logger;
    static LogUtility cb_carla_api_logger;
    static LogUtility cb_service_status_logger;
    static LogUtility cb_manual_logger;
    static LogUtility profiling_logger;

    //Sender.
    RtspServer* rtsp_handler;

    // Policy State Machine section

    class PolicyState; //Forward declared inner class, implement inside policy state machine;
    // All posible states forward declaration
    class IdleState;
    class BrakeState;
    class LKSState;
    class TJAState;
    class ACCState;
    class HWCState;
    class TJCState;

    PolicyState *policy_state_;
    void transitionToState(PolicyState *state);
    void releaseBrakeState();

    // Perception wrappers.
    void detectLane(const cv::Mat& src_image, std::shared_ptr<LaneDetection> lane_result);
    void detectObject(cv::Mat src_image, std::shared_ptr<SensorData<SensorType::CAMERA>> camera_data);

    // Planning wrappers.
    PlanningResults plan(   PerceptionOutputObject &perception_output,
                            LaneDetection &lane_detection, 
                            TrajectoryGenerationPolicy trajectory_generation_policy, 
                            DecisionMakingPolicy decision_policy);

    // Control wrappers.
    ControlResults controlLKS(const PlanningResults &planning_results, const float &speed, ControlConfigs configs);
    ControlResults controlTJA(const PlanningResults &planning_results, const float &speed, ControlConfigs configs);
    ControlResults controlACC(const PlanningResults &planning_results, const float &speed, ControlConfigs configs);
    ControlResults controlTJC(const PlanningResults &planning_results, const float &speed, ControlConfigs configs);
    ControlResults controlHWC(const PlanningResults &planning_results, const float &speed, ControlConfigs configs);
    ControlResults controlAEB();

    // Drive help function.
    void drive(ControlResults control_results);
    void driveACC(ControlResults control_results);
    void driveAEB(ControlResults control_results);
    void driveTJA(ControlResults control_results);
    void driveLKS(ControlResults control_results);

#ifdef ENABLE_CAN
    // CAN Callbacks.
    void cbEPSButtonStatus(const can_frame& frame);
    void cbEPSTakeOverStatus(const can_frame& frame);
    void cbEPSAlertMatrix(const can_frame& frame);
    void cbEPSDriverControl(const can_frame& frame);
#endif  // ENABLE_CAN

    // Callback for new ADAS Service Interface
    void cbImuIF(const ipc_helper::msg::ImuParameters::SharedPtr msg);
    void cbOdometerIF(const ipc_helper::msg::Odometry::SharedPtr msg);
    void cbGnssIF(const ipc_helper::msg::GnssPoint::SharedPtr msg);
    void cbRadarIF(const ipc_helper::msg::Radar::SharedPtr msg);
    void cbCarStatusIF(const ipc_helper::msg::CarStatus::SharedPtr msg);
    void cbCarlaApiIF(const carla_msgs_custom::msg::VehicleDataArray::SharedPtr msg);
    void cbServiceCommandIF(const ipc_helper::msg::ServiceCommand::SharedPtr msg);

    // Send Image.
    void sendImageToRtspServer(PerceptionResults perceptionResults);

protected:
    std::map<boolObjects, bool*> boolVariableMap = {
        {STATUS_BTN_CTRL, &statusBtnCtrl},
        {STATUS_BTN_CALIB, &statusBtnCalib},
        {STATUS_BTN_PARAM, &statusBtnParam},
        {STATUS_BTN_CAPTURE, &statusBtnCapture},
        {STATUS_BTN_LANE_GRAYSCALE, &statusBtnLaneGrayscale},
        {STATUS_BTN_SLIDING_WINDOW, &statusBtnSlidingWindow},
        {STATUS_BTN_LOCALIZATION, &statusBtnLocalization},
        {STATUS_BTN_LANE_DETECT, &statusBtnLaneDetect},
        {STATUS_BTN_OBJ_ENABLE, &statusBtnObjEnable},
        {STATUS_BTN_OBJ_CLASS, &statusBtnObjClass},
        {STATUS_BTN_OBJ_DISTANCE, &statusBtnObjDistance},
        {STATUS_BTN_UNDISTOR, &statusBtnUndistor},
        {STATUS_BTN_SEND_CONTROL_MSG, &statusBtnSendControlMsg},
        {STATUS_BTN_START_SERVER, &statusBtnStartServer},
        {STATUS_BTN_LANE_KEEPING_SYSTEM, &statusBtnLKS},
        {STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, &statusBtnACC},
        {STATUS_BTN_AUTO_EMERGENCY_BRAKING, &statusBtnAEB},
        {STATUS_BTN_DETACH_AUTO_BRAKE, &statusBtnDetachAutoBrake}
    };
    std::map<intObjects, int*> intVariableMap = {
        {VALUE_SLIDER_STEER, &valueSliderSteer},
        {VALUE_LINE_EDIT_STEER, &valueLineEditSteer},
        {VALUE_LINE_EDIT_THROTTLE, &valueLineEditThrottle},
        {VALUE_SLIDER_THROTTLE, &valueSliderThrottle},
        {VALUE_LINE_EDIT_BRAKE, &valueLineEditBrake},
        {VALUE_SLIDER_BRAKE, &valueSliderBrake}
    };

    std::string src_type;
    std::string src;
    bool is_running;
    bool disable_drive_assist = false;
    int disable_drive_assist_counter = 0;
    float carla_speed;
    GEAR current_gear = GEAR::N;
    MessageSharingPool<SteeringParameters, 20> steerAngleQueue;
    MessageSharingPool<ImuParameters, 20> imuQueue;
    MessageSharingPool<OdometryParameters, 20> odometerQueue;
    MessageSharingPool<GnssPoint, 20> gnssQueue;
    uint16_t fps = 0;
   
    // Fix: currently fixing here 
    // Let the time stamp come into the sensor queue
    // Not putting more trash variable here
    MessageSharingPool<SensorData<SensorType::API_SENSOR>, 20> carlaApiQueue; 

    Controller* controller_ptr = nullptr;
    LKSController lks_controller;
    TJAController tja_controller;
    ACCController acc_controller;
    HWCController hwc_controller;
    TJCController tjc_controller;
    AEBController aeb_controller;

    // Interface section
    // ROS Interface
    AdasServiceIF interface;

#ifdef ENABLE_CAN
    // CAN Interface
    CanManager can_manager;
    void sendCanControlMsg(ControlResults control_results);
    void sendCanAdasWarning(bool isWarning, bool isDanger, bool aebEnabled, int laneStatus);
    void sendCanAdasState(int mode);
    void sendCanControlSetting(float refVel, float refDist);
#endif  // ENABLE_CAN
protected:

    PlanningConfigs createPlanningConfigs(int width, int height);
    ControlConfigs createControlConfigs();
    template <typename T>
    void handleControlMsg(controlMsgAttributeType type, T value);

    // Central struct for storing results from different steps.
    PipelineResults pipeline_results;

public:
    AdasMain(std::string src_type
            , std::string src
            , bool debug_aeb=false);
    ~AdasMain();
    AdasMain(); // Empty Contructor to allow for polymorphic call.
    void TriggerSignal(bool is_start);
    void setValue(boolObjects boolEnum, bool value);
    void setValue(intObjects intEnum, int value);
    cv::Mat getDebugImage();
    void setDebugImage(cv::Mat img);
    float getSpeed();
    float getFps();
    float getTau();
    int getStateID();
    float getDistanceCaptured();
    void setIsRunning(bool is_running);
    bool getIsRunning();
    bool getAEBState();
    bool updateDisableDriveAssist(bool is_missing_lane);
    bool getDisableDriveAssist();
    void setDisableDriveAssist(bool disable_drive_assist);
    bool isReverse();
    void setGear(GEAR gear);
    GEAR getGear();
    // void setAutoDriveRestarter(AutoDrivingStateHandle* handler);
    void resetAutoDrivingState();
    
    void setRefVel(float newRefVel)
    {
        // Clamp speed within valid range [MIN, MAX]
        if (newRefVel < MIN_REF_SPEED_KMH) newRefVel = MIN_REF_SPEED_KMH;
        if (newRefVel > MAX_REF_SPEED_KMH) newRefVel = MAX_REF_SPEED_KMH;

        lks_controller.setRefVel(newRefVel);
        acc_controller.setRefVel(newRefVel);
        tja_controller.setRefVel(newRefVel);
        hwc_controller.setRefVel(newRefVel);
        tjc_controller.setRefVel(newRefVel);
    }

    void setRefDistance(float newRefDist)
    {
        // Clamp distance within valid range [MIN, MAX]
        if (newRefDist < MIN_REF_DISTANCE_M) newRefDist = MIN_REF_DISTANCE_M;
        if (newRefDist > MAX_REF_DISTANCE_M) newRefDist = MAX_REF_DISTANCE_M;

        lks_controller.setRefDistance(newRefDist);
        acc_controller.setRefDistance(newRefDist);
        tja_controller.setRefDistance(newRefDist);
        hwc_controller.setRefDistance(newRefDist);
        tjc_controller.setRefDistance(newRefDist);
    }

    float getRefVel(void)
    {
        if (controller_ptr == nullptr) return 0.0f;
        return controller_ptr->getRefVel();
    }

    float getRefDist(void)
    {
        if (controller_ptr == nullptr) return 0.0f;
        return controller_ptr->getRefDist();
    }

    void updateRefValues(float newRefVel, float newRefDist)
    {
        if (newRefVel < MIN_REF_SPEED_KMH) {
            WARN("Invalid input, set to minimum speed of %.0f km/h", MIN_REF_SPEED_KMH);
            newRefVel = MIN_REF_SPEED_KMH;
        }
        else if (newRefVel > MAX_REF_SPEED_KMH) {
            WARN("Invalid input, set to maximum speed of %.0f km/h", MAX_REF_SPEED_KMH);
            newRefVel = MAX_REF_SPEED_KMH;
        }
        newRefVel = std::round(newRefVel);
        this->setRefVel(newRefVel);

        // If within [0, min), then set to min
        if (newRefDist >= 0 && newRefDist < MIN_REF_DISTANCE_M) {
            WARN("Invalid input, set to minimum distance of %.0fm", MIN_REF_DISTANCE_M);
            newRefDist = MIN_REF_DISTANCE_M;
        }
        // Else if within [min, max], keep it
        else if (MIN_REF_DISTANCE_M <= newRefDist && newRefDist <= MAX_REF_DISTANCE_M) {
        }
        // Else out side of [0, max], set to max
        else {
            WARN("Invalid input, set to maximum distance of %.0fm", MAX_REF_DISTANCE_M);
            newRefDist = MAX_REF_DISTANCE_M;
        }
        newRefDist = std::round(newRefDist);
        this->setRefDistance(newRefDist);
    }

    void resetRefValues()
    {
        this->setRefVel(0.0f);
        this->setRefDistance(0.0f);
    }

    virtual void setSteeringAngle(float epsSteeringAngle)
    {
        lks_controller.setEpsSteeringAngle(epsSteeringAngle);
        hwc_controller.setEpsSteeringAngle(epsSteeringAngle);
        tjc_controller.setEpsSteeringAngle(epsSteeringAngle);
    }

    float getSteeringAngle(void)
    {
        if (controller_ptr != nullptr)
            return controller_ptr->getSteeringAngle();
        return 0.0f;
    }
    float getThrotlle(void)
    {
        if (controller_ptr != nullptr)
            return controller_ptr->getThrotlle();
        return 0.0f;
    }    

    Perception* getPerceptionModel()
    {
        return &this->perceptionModel;
    }
    Planner* getPlanningModel()
    {
        return &this->planner_;
    }
    void initPipelineResults()
    {
        // Clear the PipelineResults struct.
        this->pipeline_results.init();
    }

    void savePerceptionResults(PerceptionResults& perception_results)
    {
        this->pipeline_results.perception_results = perception_results;
    };
    void savePerceptionOutput(PerceptionOutputObject& perception_output)
    {
        this->pipeline_results.perception_output = perception_output;
    };
    void saveLaneDetection(LaneDetection& lane_detection)
    {
        this->pipeline_results.lane_detection = lane_detection;
    };
    void savePlanningResults(PlanningResults planning_results)
    {
        this->pipeline_results.planning_results = planning_results;
    };
    void saveControlResults(ControlResults control_results)
    {
        this->pipeline_results.control_results = control_results;
    };
    PerceptionResults getPerceptionResults() const
    {
        return this->pipeline_results.perception_results;
    };
    PerceptionOutputObject getPerceptionOutput() const
    {
        return this->pipeline_results.perception_output;
    };
    LaneDetection getLaneDetection() const
    {
        return this->pipeline_results.lane_detection;
    };
    PlanningResults getPlanningResults() const
    {
        return this->pipeline_results.planning_results;
    };
    ControlResults getControlResults() const
    {
        return this->pipeline_results.control_results;
    };
    void doSteer(float steering_value);
    void doThrottle(float throttle_value);
    void doBrake(float brake_value);

    void setDistanceCaptured(float new_distance_captured) {
        this->distance_captured = new_distance_captured;
    }
    void cacheThrottle(float value) { this->cached_throttle_value = value; }
    void cacheBrake(float value) { this->cached_brake_value = value; }
    void cacheSteer(float value) { this->cached_steer_value = value; }
    float getCachedThrottle() const { return this->cached_throttle_value; }
    float getCachedBrake() const { return this->cached_brake_value; }
    float getCachedSteer() const { return this->cached_steer_value; }

    // Getters for Loggers
    static LogUtility& getCbCameraLogger() { return cb_camera; }
    static LogUtility& getCbCarlaStatusLogger() { return cb_carla_status_logger; }
    static LogUtility& getCbRadarLogger() { return cb_radar_logger; }
    static LogUtility& getCbGnssLogger() { return cb_gnss_logger; }
    static LogUtility& getCbImuLogger() { return cb_imu_logger; }
    static LogUtility& getCbOdometerLogger() { return cb_odometer_logger; }
    static LogUtility& getCbCarlaApiLogger() { return cb_carla_api_logger; }
    static LogUtility& getCbServiceStatusLogger() { return cb_service_status_logger; }
    static LogUtility& getCbManualLogger() { return cb_manual_logger; }

    // Helper function to enable log to file in state machine logger
    static void enableStateMachineLogFile();

    // Thread-safe queue for storing results
    DataSharingQueue<CameraImageData> rawSrcImageQueue;
    DataSynchronizerQueue<TrackedSensorData> SynchronizeQueue;
    DataSharingQueue<PerceptionResults> perceptionResultsQueue;
    DataSharingQueue<LaneDetection> LaneDetectionsQueue;
    DataSharingQueue<PerceptionOutputObject> perceptionOutputQueue;
    DataSharingQueue<PlanningResults> PlanningResultsQueue;

    // Raw data thread safe queue
    DataSharingQueue<SensorDataType> radar_detection_queue;
    DataSharingQueue<SensorDataType> camera_detection_queue;
    DataSharingQueue<SensorDataType> camera_inference_queue;

    // Consumer ID
    int consumer_lane_detection_id_ = -1;   /**< Consumer ID of Lane Detection Queue */
    int consumer_perception_results_id_ = -1;   /**< Consumer ID of Perception Results Queue */
    int consumer_perception_output_objects_id_ = -1;   /**< Consumer ID of Object Detection Queue */
    int consumer_planning_results_id_ = -1;   /**< Consumer ID of Planning Results Queue */
    int raw_src_cons = -1;
    int radar_tracker_id = -1;
    int camera_tracker_id = -1;
    int object_detection_id = -1;
    int lane_detection_id = -1;

    // Thread-safe Getters/Setters
    bool getStatusBtnLKS() {
        boost::lock_guard<boost::mutex> lock(control_state_mutex_);
        return statusBtnLKS;
    }

    void setStatusBtnLKS(bool value) {
        boost::lock_guard<boost::mutex> lock(control_state_mutex_);
        statusBtnLKS = value;
    }

    bool getStatusBtnACC() {
        boost::lock_guard<boost::mutex> lock(control_state_mutex_);
        return statusBtnACC;
    }

    void setStatusBtnACC(bool value) {
        boost::lock_guard<boost::mutex> lock(control_state_mutex_);
        statusBtnACC = value;
    }
};

#endif // ADAS_MAIN_H