#ifndef _TRACKER_HPP_
#define _TRACKER_HPP_

#include "intermediate_representations.h"
#include "hungarian_algorithm.hpp"
#include <queue>
#include <deque>
#include <numeric>
#include "logger/general_logger.h"

class TrackedSensorData : public SensorDataType {
public:
    SensorType sensor_type = SensorType::UNDEFINED;
    std::map<uint32_t, std::shared_ptr<InputObject>> matched_objects;
    SensorType getSensorType() const override
    {
        return this->sensor_type;
    }

    std::shared_ptr<SensorDataType> clone() const override {
        std::shared_ptr<TrackedSensorData> cloned = std::make_shared<TrackedSensorData>(*this);

        for (std::shared_ptr<InputObject> object : cloned->objects)
        {
            object = object->clone();
        }

        for (auto object : cloned->matched_objects)
        {
            object.second = object.second->clone();
        }
        return std::static_pointer_cast<SensorDataType>(cloned);
    }
};

class Tracker {
private:
    // Internal state of Tracker, consist of 3 object type after fusing
    // Note that the 
    std::map<SensorType, std::shared_ptr<TrackedSensorData>> sensor_data_;
    std::map<SensorType, Eigen::Matrix<float, 4, 1>> ego_states;

    std::map<SensorType, std::map<uint32_t, std::shared_ptr<InputObject>>> objects_for_fusion;
    std::vector<std::shared_ptr<OutputObject>> object_data_synthesis_; 

    // Min max/hit, id setting for each sensor type
    std::map<SensorType, uint32_t> max_age;          // The life number required for an object to be considered disappeared
    std::map<SensorType, uint32_t> min_hit;          // The minimum number of hits required for an object to be considered valid
    std::map<SensorType, int> matching_threshold;    // The threshold for object matching
    std::map<SensorType, uint64_t> latest_time_stamp;
    
    std::map<SensorType, uint32_t> lastest_id_;

    std::map<
        std::pair<std::pair<uint32_t, int>, int>, // key: {{FusionID, classID}, Age}
        std::map<
            SensorType,
            std::pair<uint32_t, int> // value: {trackingID, classID}
        >
    > tracking_manager;  

    // Window size of Medium filter
    int Medium_Filter_Window_size = 5;
    // Bounding value to filter unreasonabl acceleration value
    float filter_bounding_Acceleration = 10;
    // Filter noise of velocity of static objects
    float filter_noise_velocity = 0.5;
    // Threshold for detecting that the dynamic object can slow down
    float slow_down_threshold = 1.0;
    // Delta Time value between 2 frames, as setting in simulation frequency 
    float delta_time = 0.05;

    // Store Velocity of Objects from previous frame
    std::map<int, std::pair<float, float>> old_velocity; // {Tracking ID , {Velocity_x, Velocity_y}}
    // Store Acceleration of Objects from previous frame
    std::map<int, std::pair<std::deque<float>, std::deque<float>>> old_acceleration; // {Tracking ID , {Acceleration_x, Acceleration_y}}

    void smoothVelocity(uint32_t fusionID, float & new_velocity, int choose_threshold);

    float correlation_scaling_factor = 1.0;       
    // The parameter for determining the correlation coefficient between radar and camera. It ranges from zero to one, 
    // with values closer to one indicating that radar and camera parameters are more independent.
    std::shared_ptr<std::mutex> tracker_mutex_;
    // Support functions
    void hungarianMatching(const std::vector<std::vector<int>> &cost_matrix, std::vector<std::vector<int>> &association_mat);
    int calculateCost(SensorType sensor_type, const std::shared_ptr<InputObject>& object_detection, const std::shared_ptr<InputObject>& tracked_object);
    void matchingObjects(std::shared_ptr<SensorDataType> object_detecions, std::shared_ptr<TrackedSensorData> current_queue);
    int calcIOU(const std::shared_ptr<InputObject>& object_detection, const std::shared_ptr<InputObject>& tracked_object);
    int calcMatchingOfRadarObjects(const std::shared_ptr<InputObject>& object_detection, 
                                   const std::shared_ptr<InputObject>& tracked_object);
    int calcMatchingOfFusionObjects(const std::shared_ptr<InputObject>& camera_object, 
                                    const std::shared_ptr<InputObject>& radar_object,
                                    const float &camera_sample_time,  const float &radar_sample_time);
    std::vector<std::shared_ptr<OutputObject>> executeSensorFusion(std::map<uint32_t, std::shared_ptr<InputObject>> matched_camera_objects, 
                                                                    std::map<uint32_t, std::shared_ptr<InputObject>> matched_radar_objects,
                                                                    const float &camera_sample_time,  const float &radar_sample_time);
    std::shared_ptr<OutputObject> getFusedObject(std::shared_ptr<InputObject> camera_object, 
                                                  std::shared_ptr<InputObject> radar_object, 
                                                  const float &camera_sample_time,  const float &radar_sample_time);

    int getInternalId(SensorType sensor_type);

    // Function to delete elements in the specified row and column of a matrix
    void deleteRowAndColumn(std::vector<std::vector<int>>& matrix, int row_idx, int col_idx);

    std::pair<bool, std::pair<std::pair<uint32_t, int>, int>> findExistedTrackingObject(SensorType sensor_type, uint32_t trackingID);
    uint32_t findCorrespondEstimateObject(SensorType sensor_type, std::pair<uint32_t, int> fusion_key);
    std::shared_ptr<OutputObject> updateSeparateOutputObjects(std::pair<const uint32_t, std::shared_ptr<InputObject>> input_object, 
                                                            SensorType sensor_type);
    void updateLifeCycleofManagerTracking();
    void updateAge(uint32_t fusionID, int classID, int newAge);
public:
    // Initialize an Tracker
    Tracker();
    ~Tracker() = default;

    // Update the Tracker with recent object and execute
    void execute(std::shared_ptr<SensorDataType> &object_detections, const Eigen::Matrix<float, 4, 1>& ego_states = Eigen::Matrix<float, 4, 1>::Zero());
    // Add additional function since this need to be paralleled 

    std::shared_ptr<TrackedSensorData> processSensorTracking(std::shared_ptr<SensorDataType>& object_detections,
                                            const Eigen::Matrix<float, 4, 1>& ego_states,
                                            SensorType sensor_type);

    void performFusion(std::queue<std::shared_ptr<TrackedSensorData>>& current_data_queue);
    std::shared_ptr<PerceptionOutputObject> getTrackingObjects();
};

#endif // _TRACKER_HPP_