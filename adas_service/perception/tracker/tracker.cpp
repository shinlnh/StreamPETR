#include "tracker.hpp"
#include <Eigen/Core>

//> Constructor =========================================================================================================

Tracker::Tracker() {
    // Initialize the Tracker max and min threshold
    this->max_age[SensorType::CAMERA]   = 1;
    this->max_age[SensorType::RADAR]    = 2;

    this->min_hit[SensorType::CAMERA]   = 2;
    this->min_hit[SensorType::RADAR]    = 0;

    // The parameters below are assigned based on benchmark data. 
    // Detailed information is availabe in https://confluence.banvien.com.vn/display/ESRNDAUT/SORT%3A+Simple+Online+and+Realtime+Tracking 
    // and https://confluence.banvien.com.vn/display/ESRNDAUT/Camera+and+radar+fusion%3A+Implementation
    this->matching_threshold[SensorType::CAMERA] = 75;      // The IoU value is multiplied by 1000 when returned as an integer cost
    this->matching_threshold[SensorType::RADAR] = -9210;   // The Mahalanobis distance is multiplied by -1000 when returned as an integer cost
    this->matching_threshold[SensorType::FUSION] = 13280;    // The Mahalanobis distance is multiplied by 1000 when returned as an integer cost

    this->lastest_id_[SensorType::CAMERA] = 0;
    this->lastest_id_[SensorType::RADAR] = 0;
    this->lastest_id_[SensorType::FUSION] = 0;

    this->latest_time_stamp[SensorType::CAMERA] = 0;
    this->latest_time_stamp[SensorType::RADAR] = 0;
    this->latest_time_stamp[SensorType::FUSION] = 0;

    sensor_data_[SensorType::CAMERA] = std::make_shared<TrackedSensorData>(); 
    sensor_data_[SensorType::RADAR] = std::make_shared<TrackedSensorData>(); 
    tracker_mutex_ = std::make_shared<std::mutex>();
}

//> HELPER FUNCTION =========================================================================================================
void Tracker::hungarianMatching(const std::vector<std::vector<int>> &cost_matrix,
                                      std::vector<std::vector<int>> &association_mat)
{
    std::vector<std::vector<int>> neg_mat;
    size_t nrows = cost_matrix.size();
    size_t ncols = cost_matrix.empty() ? 0 : cost_matrix[0].size();
    neg_mat.resize(nrows, std::vector<int>(ncols));
    // Init matrix with negative cost
    for (size_t row = 0; row < nrows; ++row)
    {
        for (size_t col = 0; col < ncols; ++col)
        {
            neg_mat[row][col] = -1 * cost_matrix[row][col];
        }
    }

    // Apply Hungarian with negative allowed
    Munkres::hungarian(neg_mat);

    // Copy result into association_mat
    for (size_t row = 0; row < nrows; ++row)
    {
        for (size_t col = 0; col < ncols; ++col)
        {
            association_mat[row][col] = neg_mat[row][col];
        }
    }
}

int Tracker::calcMatchingOfRadarObjects(const std::shared_ptr<InputObject>& object_detection, 
                                        const std::shared_ptr<InputObject>& tracked_object)
{   
    // Observation error covariance matrix of object_detection
    // This covariance is assigned the value of the radar measurement noise covariance matrix that is measured.
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR> object_detection_s_matrix;
    object_detection_s_matrix = tracked_object->kalmanFilter[0]->getMeasurementNoiseCovariance();
    // Extract (x, y) distance and velocity (vx​,vy​) from object detection parameters
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1>  object_detection_observation; 
    object_detection_observation << object_detection->location.x(),
                                    object_detection->velocity.x() + this->ego_states[SensorType::RADAR].coeffRef(0),
                                    object_detection->location.y(),
                                    object_detection->velocity.y() + this->ego_states[SensorType::RADAR].coeffRef(2);
    
    // Observation error covariance matrix of tracked radar object
    Eigen::Matrix<float,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR> tracked_object_s_matrix;
    // Extract radar (x, y) distance and velocity (vx​,vy​ )from Kalman state
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1>  tracked_object_observation;
    
    // Since the tracked radar object is predicted by the previous predict function, assign radar_sample_time to 0.0
    tracked_object_s_matrix = tracked_object->kalmanFilter[0]->getObservationErrorCovarianceMatrix(tracked_object_observation, this->ego_states[SensorType::RADAR], 0.0);
    // Compute the difference (innovation) between object_detection and tracked_object
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1>  innovation = object_detection_observation - tracked_object_observation;
    // Compute Observation Error Covariance
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR> s_matrix = object_detection_s_matrix + tracked_object_s_matrix;
    // Manually reduce weight of Observation Error Covariance Matrix -> Reduce the raw Mahalonobis Distance
    Eigen::MatrixXf  scaled_s_matrix = s_matrix.inverse();
    scaled_s_matrix.coeffRef(0,0) /= 10; 

    // Compute Mahalanobis distance:
    float mahalanobis_distance = std::sqrt(std::fabs(innovation.transpose() * scaled_s_matrix * innovation));
    // Return as integer cost
    return static_cast<int>(mahalanobis_distance * 1000);
}

int Tracker::calcIOU(const std::shared_ptr<InputObject>& object_detection, 
                     const std::shared_ptr<InputObject>& tracked_object)
{
    Eigen::MatrixXf predicted_camera_bbox_state = tracked_object->kalmanFilter[1]->getPredictedState();
    Eigen::Vector4f predicted_box;

    predicted_box[0] = predicted_camera_bbox_state(0, 0) - predicted_camera_bbox_state(2, 0)/ 2;  // x_min
    predicted_box[2] = predicted_camera_bbox_state(0, 0) + predicted_camera_bbox_state(2, 0)/ 2;  // x_max
    predicted_box[1] = predicted_camera_bbox_state(1, 0) - predicted_camera_bbox_state(3, 0)/ 2;  // y_min
    predicted_box[3] = predicted_camera_bbox_state(1, 0) + predicted_camera_bbox_state(3, 0)/ 2;  // y_max
                     
    float iou_x1 = std::max(object_detection->bbox[0], predicted_box[0]);
    float iou_y1 = std::max(object_detection->bbox[1], predicted_box[1]);
    float iou_x2 = std::min(object_detection->bbox[2], predicted_box[2]);
    float iou_y2 = std::min(object_detection->bbox[3], predicted_box[3]);
    float iou_width = std::max(0.0f, iou_x2 - iou_x1);
    float iou_height = std::max(0.0f, iou_y2 - iou_y1);

    // Calculate area of intersection and union
    float det_area = (object_detection->bbox[2] - object_detection->bbox[0]) * (object_detection->bbox[3] - object_detection->bbox[1]); // Assuming area() is a method of InputObject
    float track_area = std::abs(predicted_box[2] - predicted_box[0]) * std::abs(predicted_box[3] - predicted_box[1]);
    float intersection_area = iou_width * iou_height;
    float union_area = det_area + track_area - intersection_area;
    float iou = intersection_area / union_area;


    // Weights for different aspects of the cost function
    const float x_distance_weight = 0.001;    // Weight for x-distance difference
    const float y_distance_weight = 0.1f;     // Weight for y-distance difference
    const float iou_weight = 0.899;           // Weight for iou

    // Safeguard against division by zero or near-zero values
    float epsilon = 1e-6;

    // Compute normalized differences
    Eigen::MatrixXf predicted_3d_state = tracked_object->kalmanFilter[0]->getPredictedState();
    float normalized_x_distance_diff = std::fabs(object_detection->location.x() - predicted_3d_state(0, 0)) / 
                                       std::max(std::fabs(object_detection->location.x() + predicted_3d_state(0, 0)), epsilon);

    float normalized_y_distance_diff = std::fabs(object_detection->location.y() - predicted_3d_state(4, 0)) / 
                                       std::max(std::fabs(object_detection->location.y() + predicted_3d_state(4, 0)), epsilon);

    // Weighted cost
    float weight_cost = - (x_distance_weight * normalized_x_distance_diff)
                        - (y_distance_weight * normalized_y_distance_diff)
                        + (iou_weight * iou);

    weight_cost = std::max<float>(weight_cost, 0.0f);
    // Return as integer cost
    return static_cast<int>(weight_cost * 1000);
}

int Tracker::calculateCost(
    SensorType sensor_type, 
    const std::shared_ptr<InputObject>& object_detection, 
    const std::shared_ptr<InputObject>& tracked_object)
{
    if (sensor_type == SensorType::RADAR) {
        return -calcMatchingOfRadarObjects(object_detection, tracked_object);
    }
    else if (sensor_type == SensorType::CAMERA) {
        return calcIOU(object_detection, tracked_object);
    }
    else {
        return std::numeric_limits<int>::infinity();
    }
}

void Tracker::matchingObjects(std::shared_ptr<SensorDataType> object_detections, 
                              std::shared_ptr<TrackedSensorData> current_queue) 
{
    // Big note: in this function we tried to update the NEWEST data here which is not the optimal way, 
    // this this section when assign for match and unmatch should be consider updated using the kalman prediction 
    // No detections from this sensor - early return 
    if (object_detections->objects.empty()) return;
    SensorType sensor_type = object_detections->getSensorType();

    // No memory - nothing to be matched
    if (current_queue->matched_objects.empty())
    {
        for (const auto& object : object_detections->objects)
        {
            int lastest_index = getInternalId(sensor_type);
            current_queue->matched_objects[lastest_index] = std::make_shared<InputObject>(*object);
            // New object initialize it kalman filter for latter update
            current_queue->matched_objects[lastest_index]->init(object, this->ego_states[sensor_type]);
            if(sensor_type == SensorType::RADAR)
            {
                // Hard set value
                current_queue->matched_objects[lastest_index]->velocity.x() = object->velocity.x() + this->ego_states[sensor_type].coeffRef(0);
                current_queue->matched_objects[lastest_index]->velocity.y() = object->velocity.y() + this->ego_states[sensor_type].coeffRef(2);
                current_queue->matched_objects[lastest_index]->location.x() = object->location.x();
                current_queue->matched_objects[lastest_index]->location.y() = object->location.y();
                current_queue->matched_objects[lastest_index]->closest_point.x() = object->closest_point.x();
                current_queue->matched_objects[lastest_index]->closest_point.y() = object->closest_point.y();
                // Store velocity for next frame
                old_velocity[lastest_index] = std::make_pair(object->velocity.x(), object->velocity.y());
                old_acceleration[lastest_index].first.push_back(0);
                old_acceleration[lastest_index].second.push_back(0);
            }
        } 
        return;
    }
    // If two above conditions not satisfied we perform matching

    std::vector<std::vector<int>> cost_matrix;
    std::vector<std::vector<int>> matching_result;
    int detection_input_size = object_detections->objects.size();
    int current_matching_queue_size = current_queue->matched_objects.size();
    cost_matrix.resize(detection_input_size, std::vector<int>(current_matching_queue_size));
    matching_result.resize(detection_input_size, std::vector<int>(current_matching_queue_size));
    for (size_t row = 0; row < detection_input_size; ++row)
    {
        size_t col = 0;
        for (const auto &track_record : current_queue->matched_objects)
        {
            cost_matrix[row][col] = calculateCost(sensor_type, object_detections->objects[row], track_record.second);
            ++col;
        }
    }
    
    hungarianMatching(cost_matrix, matching_result);
    // Loop through all detect to seek the matching with previous tracked object
    for (size_t row = 0; row < detection_input_size; ++row)
    {
        size_t col = 0;
        bool matched_flag = false;
        for (const auto &trk : current_queue->matched_objects)
        {   
            if(1 == matching_result[row][col])
            {
                // Check if the matching result is qualified by matching threshold
                if (cost_matrix[row][col] >= this->matching_threshold[sensor_type])
                {
                    matched_flag = true;
                    // Insert new value into matched placeholder          
                    float dvx_avg, dvy_avg;       
                    float vel_x = object_detections->objects.at(row)->velocity.x() + this->ego_states[sensor_type].coeffRef(0);
                    float vel_y = object_detections->objects.at(row)->velocity.y() + this->ego_states[sensor_type].coeffRef(2);
                    current_queue->matched_objects[trk.first]->update(object_detections->objects.at(row), this->ego_states[sensor_type]);

                    // Because of the slow response of LKF when estimate Velocity and Acceleration in scenarios that following car brake immediately
                    // It is neccessary to hard update the measurement data to get the fast response of data tracking to support AEB can check overlapp exactly
                    if (sensor_type == SensorType::RADAR)
                    {
                        // Hard update zero value for all when dynamic car act like Braking
                        if (this->ego_states[sensor_type].coeffRef(2) > 0 && this->ego_states[sensor_type].coeffRef(3) < -10)
                        {
                            unsigned int object_id = trk.first;
                            vel_x = 0; vel_y = 0; dvx_avg = 0; dvx_avg = 0;
                            old_acceleration[object_id].first.push_back(0);
                            old_acceleration[object_id].second.push_back(0);
                            old_velocity[object_id] = std::make_pair(0, 0);
                            // Hard update
                            current_queue->matched_objects[trk.first]->acceleration.x() = 0;
                            current_queue->matched_objects[trk.first]->acceleration.y() = 0;
                            current_queue->matched_objects[trk.first]->velocity.x() = 0;
                            current_queue->matched_objects[trk.first]->velocity.y() = 0;
                        }
                        else
                        {
                            unsigned int object_id = trk.first;
                            auto &object = trk.second;
                            // If have old_velocity
                            if (old_velocity.find(object_id) != old_velocity.end()) 
                            {
                                bool accept_new_accX = true;
                                bool accept_new_accY = true;
                                // Calculate Acceleration X
                                float acceleration_x = (vel_x - old_velocity[object_id].first) / delta_time;
                                // Filter to eliminate the unreasonable value
                                if (std::fabs(acceleration_x) > filter_bounding_Acceleration) 
                                {
                                    dvx_avg = old_acceleration[object_id].first.back();
                                    acceleration_x = old_acceleration[object_id].first.back();
                                    accept_new_accX = false;
                                }
                                // Calculate Acceleration Y
                                float acceleration_y = (vel_y - old_velocity[object_id].second) / delta_time;
                                // Filter to eliminate the unreasonable value
                                if (std::fabs(acceleration_y) > filter_bounding_Acceleration) 
                                {
                                    dvy_avg = old_acceleration[object_id].second.back();
                                    acceleration_y = old_acceleration[object_id].second.back();
                                    accept_new_accY = false;
                                }
                                // Restore acceleration value
                                old_acceleration[object_id].first.push_back(acceleration_x);
                                old_acceleration[object_id].second.push_back(acceleration_y);
                                
                                // Check if Data is sufficient for Medium filter
                                if (old_acceleration[object_id].first.size() > Medium_Filter_Window_size) old_acceleration[object_id].first.pop_front();
                                if (old_acceleration[object_id].second.size() > Medium_Filter_Window_size) old_acceleration[object_id].second.pop_front();
    
                                auto dvx_history = old_acceleration[object_id].first;
                                auto dvy_history = old_acceleration[object_id].second;
                                
                                // Medium filter reduce noise
                                if (accept_new_accX)
                                    dvx_avg = std::accumulate(dvx_history.begin(), dvx_history.end(), 0.0f) / dvx_history.size();
                                if (accept_new_accY)
                                    dvy_avg = std::accumulate(dvy_history.begin(), dvy_history.end(), 0.0f) / dvy_history.size();

                                // Hard update
                                if (old_velocity[object_id].first != 0 || old_velocity[object_id].second != 0)
                                {
                                    // Hard filter for case that ego Car brake and following car brake immediately
                                    if (std::fabs(old_velocity[object_id].first) < slow_down_threshold)
                                    {
                                        if (std::fabs(vel_x) < filter_noise_velocity && acceleration_y < -2)
                                        {
                                            old_acceleration[object_id].first.back() = 0;
                                            vel_x = 0;
                                            dvx_avg = 0;
                                        }
                                    }
                                    if (std::fabs(old_velocity[object_id].second) < slow_down_threshold)
                                    {
                                        if (std::fabs(vel_y) < filter_noise_velocity && acceleration_y < -2) 
                                        {
                                            old_acceleration[object_id].second.back() = 0;
                                            vel_y = 0;
                                            dvy_avg = 0;
                                        }
                                    }
                                }
                                else
                                {
                                    // Filter noise for static object after hard update zero value
                                    float delta_velX = std::fabs(vel_x - old_velocity[object_id].first);
                                    float delta_velY = std::fabs(vel_y - old_velocity[object_id].second);
                                    if (delta_velX < filter_noise_velocity) 
                                    {
                                        vel_x = 0;
                                        old_acceleration[object_id].first.back() = 0;
                                        dvx_avg = 0;
                                    }
                                    if (delta_velY < filter_noise_velocity) 
                                    {
                                        vel_y = 0;
                                        old_acceleration[object_id].second.back() = 0;
                                        dvy_avg = 0;
                                    }
                                }
                                // Hard update data
                                current_queue->matched_objects[trk.first]->velocity.x() = vel_x;
                                current_queue->matched_objects[trk.first]->velocity.y() = vel_y;
                                current_queue->matched_objects[trk.first]->acceleration.x() = dvx_avg;
                                current_queue->matched_objects[trk.first]->acceleration.y() = dvy_avg;

                                // Update old_velocity for next_frame
                                old_velocity[object_id] = std::make_pair(vel_x, vel_y);
                            }
                        }
                    }
                }
                break;
            }
            ++col;
        }
        // if detection not matched with any track --> insert unmatched placeholder
        if (!matched_flag)
        {
            int lastest_index = getInternalId(sensor_type);
            // current_queue->matched_objects[lastest_index] = object_detections->objects.at(row);
            current_queue->matched_objects[lastest_index] = std::make_shared<InputObject>(*object_detections->objects.at(row));
            current_queue->matched_objects[lastest_index]->init(object_detections->objects.at(row), this->ego_states[sensor_type]);

            if(sensor_type == SensorType::RADAR)
            {
                // Hard set value
                current_queue->matched_objects[lastest_index]->velocity.x() = object_detections->objects.at(row)->velocity.x() + this->ego_states[sensor_type].coeffRef(0);
                current_queue->matched_objects[lastest_index]->velocity.y() = object_detections->objects.at(row)->velocity.y() + this->ego_states[sensor_type].coeffRef(2);
                current_queue->matched_objects[lastest_index]->location.x() = object_detections->objects.at(row)->location.x();
                current_queue->matched_objects[lastest_index]->location.y() = object_detections->objects.at(row)->location.y();
                current_queue->matched_objects[lastest_index]->closest_point.x() = object_detections->objects.at(row)->closest_point.x();
                current_queue->matched_objects[lastest_index]->closest_point.y() = object_detections->objects.at(row)->closest_point.y();
                // Store velocity for next frame
                old_velocity[lastest_index] = std::make_pair(object_detections->objects.at(row)->velocity.x() + this->ego_states[sensor_type].coeffRef(0),
                                                            object_detections->objects.at(row)->velocity.y() + this->ego_states[sensor_type].coeffRef(2));
                old_acceleration[lastest_index].first.push_back(0);
                old_acceleration[lastest_index].second.push_back(0);
            }
        }
    }

    return;
}

//> ENTRY POINT ===========================================================================================================
/**
 * processSensorTracking
 *
 * This function is executed within the Perception module's dedicated tracking thread. It processes incoming
 * sensor detection data (from either a camera or radar) and updates the internal tracking state accordingly.
 * 
 * The processing steps include:
 *   1. Calculating the time step since the last update for the specified sensor.
 *   2. Predicting the future state of all existing tracks using a Kalman filter (or similar prediction algorithm).
 *   3. Matching new sensor detections to existing tracks and updating the state of matched objects.
 *   4. Initializing new tracks for any unmatched detections and assigning them unique IDs.
 *   5. Removing stale tracks that have exceeded the maximum allowed age.
 *
 * Input Parameters:
 *   - object_detections: A shared pointer to the new sensor detection data (SensorDataType).
 *   - ego_states: A 4x1 Eigen matrix containing the current state of the ego vehicle.
 *   - sensor_type: The sensor type (e.g., SensorType::CAMERA or SensorType::RADAR), which determines
 *                  which internal state (tracked data, timestamps, counters) to update.
 *
 * Output:
 *   - Returns a shared pointer to a TrackedSensorData object containing the updated tracking information
 *     for the specified sensor.
 *
 * Note: This function is designed to be called repeatedly in a loop within the tracking thread,
 * ensuring that sensor data is processed concurrently as it becomes available.
 */

std::shared_ptr<TrackedSensorData> Tracker::processSensorTracking(
    std::shared_ptr<SensorDataType>& object_detections,
    const Eigen::Matrix<float, 4, 1>& ego_states,
    SensorType sensor_type)
{
    std::lock_guard<std::mutex> lock(*tracker_mutex_);
    this->ego_states[sensor_type] = ego_states;
    auto &sensor_data = this->sensor_data_[sensor_type];
    // After benchmarking the ego_state parameters when starting new scenarios, 
    // velocity_y usually fluctuates suddenly and for the longest time among the remaining parameters — it really needs to be filtered. 
    // Additionally, the maximum velocity of my car is 240 km/h, so 240 / 3.6 is the chosen filter value.
    if (std::abs(this->ego_states[sensor_type].coeffRef(2)) > 240 / 3.6) 
    {
        return nullptr;
    }

    // Backup current matched objects before matching
    std::unordered_map<uint32_t, std::shared_ptr<InputObject>> prev_matched_objects;

    for (const auto& pair : sensor_data->matched_objects) {
        const auto& id = pair.first;
        const auto& obj = pair.second;
        if (obj)
            prev_matched_objects[id] = obj->clone(); 
    }    

    // Calculate time delta in seconds.
    float time_step = (object_detections->time_stamp - latest_time_stamp[sensor_type]) / 1e9f;
    latest_time_stamp[sensor_type] = object_detections->time_stamp;
    
    // Predict update for all existing tracks for this sensor.
    for (auto& obj : sensor_data->matched_objects) 
    {
        obj.second->predict(time_step, this->ego_states[sensor_type]);
    }
    
    // Perform matching: update internal sensor state using the new detections.
    this->matchingObjects(object_detections, sensor_data);

    // Filter for unreasonable tracking data from Radar LKF. Because of wrong Object matching, the radar object has unreasonable
    // pair of closest point and location.
    if (sensor_type == SensorType::RADAR)
    {
        auto& objects   = sensor_data->matched_objects;
        auto& proc_objs = object_detections->objects;

        for (auto it = objects.begin(); it != objects.end(); )
        {
            auto& obj = it->second;
            bool keep = false;
            bool erase_processing = false;

            for (const auto& pobj : proc_objs)
            {
                if (obj->closest_point.x() == pobj->closest_point.x())
                {
                    float dx = obj->location.x() - pobj->location.x();
                    float dy = obj->location.y() - pobj->location.y();
                    float distance = std::sqrt(dx * dx + dy * dy);
                    if (distance < 4.0f)
                        keep = true;
                    else
                        erase_processing = true;

                    break;
                }
            }

            if (!keep)
            {
                float target_x = obj->closest_point.x();
                it = objects.erase(it);
                if (erase_processing)
                {
                    proc_objs.erase(
                        std::remove_if(proc_objs.begin(), proc_objs.end(),
                            [&](const auto& o)
                            {
                                return o->closest_point.x() == target_x;
                            }),
                        proc_objs.end());
                }
            }
            else
            {
                ++it;
            }
        }
    }

    const float LOCATION_JUMP_THRESHOLD = 6.0f;     // meters
    const float CLOSEST_POINT_JUMP_THRESHOLD = 6.0f; // meters

    auto& objects = sensor_data->matched_objects;

    for (auto it = objects.begin(); it != objects.end();) {
        uint32_t id = it->first;
        auto& new_obj = it->second;

        auto prev_it = prev_matched_objects.find(id);
        if (prev_it != prev_matched_objects.end()) {
            const auto& old_obj = prev_it->second;

            float dist_loc = (new_obj->location - old_obj->location).norm();
            float dist_cp  = (new_obj->closest_point - old_obj->closest_point).norm();

            if (dist_loc > LOCATION_JUMP_THRESHOLD || dist_cp > CLOSEST_POINT_JUMP_THRESHOLD) 
            {
                it = objects.erase(it);
                continue;
            }
        }
        ++it;
    }

    // Remove stale tracks that exceed the maximum allowed age.
    for (auto it = sensor_data->matched_objects.begin();
        it != sensor_data->matched_objects.end(); )
    {
        if (it->second->death_hit_streak_ > this->max_age[sensor_type]) 
        {
            old_velocity.erase(it->first);
            old_acceleration.erase(it->first);
            it = sensor_data->matched_objects.erase(it);
        }
        else {
            ++it;
        }
    }

    // Prepare output: clone objects with sufficient life hit streak.
    TrackedSensorData tracked_sensor_output;
    tracked_sensor_output.sensor_type = sensor_type;
    tracked_sensor_output.time_stamp = object_detections->time_stamp;
    for (auto& object : sensor_data->matched_objects) {
        // if (object.second->life_hit_streak_ >= this->min_hit[sensor_type] && object.second->death_hit_streak_ == 0) 
        if (object.second->life_hit_streak_ >= this->min_hit[sensor_type]) 
        {
            tracked_sensor_output.matched_objects[object.first] = object.second->clone();
        }
    }
    return std::make_shared<TrackedSensorData>(tracked_sensor_output);
}

void Tracker::performFusion(std::queue<std::shared_ptr<TrackedSensorData>>& current_data_queue)
{
    // Build fusion-eligible object maps for both sensors.
    std::lock_guard<std::mutex> lock(*tracker_mutex_);
    
    while(!current_data_queue.empty())
    {
        // Get external sensor data (only external sensor data triggers perception).
        std::shared_ptr<const TrackedSensorData> current_sensor_data = current_data_queue.front();
        current_data_queue.pop();
        objects_for_fusion[current_sensor_data->sensor_type] = current_sensor_data->matched_objects;
    }

    float camera_sample_time = 0.0;
    float radar_sample_time = 0.0;
    if (latest_time_stamp[SensorType::CAMERA] >= latest_time_stamp[SensorType::RADAR] )
    {
        latest_time_stamp[SensorType::FUSION]  = latest_time_stamp[SensorType::CAMERA];
        radar_sample_time = (latest_time_stamp[SensorType::CAMERA] - latest_time_stamp[SensorType::RADAR]) / 1e9;
    }
    else
    {
        latest_time_stamp[SensorType::FUSION]  = latest_time_stamp[SensorType::RADAR];
        camera_sample_time = (latest_time_stamp[SensorType::RADAR] - latest_time_stamp[SensorType::CAMERA]) / 1e9;
    }
    std::vector<std::shared_ptr<OutputObject>> fused_output = executeSensorFusion(objects_for_fusion[SensorType::CAMERA], objects_for_fusion[SensorType::RADAR], camera_sample_time,  radar_sample_time);
    this->object_data_synthesis_ = fused_output;
}

std::vector<std::shared_ptr<OutputObject>>Tracker::executeSensorFusion(std::map<uint32_t, std::shared_ptr<InputObject>> matched_camera_objects, 
                                                                  std::map<uint32_t, std::shared_ptr<InputObject>> matched_radar_objects,
                                                                  const float &camera_sample_time,  const float &radar_sample_time)
{
    std::vector<std::shared_ptr<OutputObject>> fusion_result;

    int camera_objects_size = 1;
    int radar_objects_size = 1;
    if (!matched_camera_objects.empty()) {
        camera_objects_size = matched_camera_objects.size();
    } 

    if (!matched_radar_objects.empty()) {
        radar_objects_size = matched_radar_objects.size();
    }

    std::vector<std::vector<int>> cost_matrix(camera_objects_size, std::vector<int>(radar_objects_size, 0));
    std::vector<std::vector<int>> matching_matrix(camera_objects_size, std::vector<int>(radar_objects_size, 0));

    size_t camera_idx  = 0;
    for (const auto &camera_object : matched_camera_objects) {
        size_t radar_idx = 0;
        for (const auto &radar_object : matched_radar_objects) {
            cost_matrix[camera_idx][radar_idx] = -calcMatchingOfFusionObjects(camera_object.second, radar_object.second, camera_sample_time, radar_sample_time);
            ++radar_idx;
        }
        ++camera_idx;
    }
    hungarianMatching(cost_matrix, matching_matrix);

    // Loop through all detect to seek the fused objects
    camera_idx  = 0;
    for (auto camera_it = matched_camera_objects.begin(); camera_it != matched_camera_objects.end(); ++camera_idx) {
        size_t radar_idx = 0;
        bool is_matched = false;

        // Loop to find matched radar object
        for (auto radar_it = matched_radar_objects.begin(); radar_it != matched_radar_objects.end(); ++radar_idx, ++radar_it) {   
            // Check if 2 objects are matched and their cost is below threshold
            is_matched = matching_matrix[camera_idx][radar_idx] == 1;
            is_matched &= -cost_matrix[camera_idx][radar_idx] <= this->matching_threshold[SensorType::FUSION];
            if (! is_matched) {
                // If not then skip this radar object
                continue;
            }

            // Create output fusion object
            std::shared_ptr<OutputObject> fused_object;
            fused_object = getFusedObject(camera_it->second, radar_it->second, camera_sample_time, radar_sample_time);

            // Find if there is exist tracking object in fusion tracking manager
            auto fusionID_camera = findExistedTrackingObject(SensorType::CAMERA, camera_it->first); 
            auto fusionID_radar = findExistedTrackingObject(SensorType::RADAR, radar_it->first);
            if (fusionID_camera.first && fusionID_radar.first) {
                if (fusionID_camera.second.first.first == fusionID_radar.second.first.first) {
                    // Get Fusion tracking ID
                    fused_object->trackingID = fusionID_camera.second.first.first;
                    // Reset Age
                    updateAge(fused_object->trackingID, fused_object->classID, 0);
                }
                else {
                    // False match
                    is_matched = false;
                }
            }
            else if (fusionID_camera.first) {
                // Get Fusion tracking ID
                fused_object->trackingID = fusionID_camera.second.first.first;
                // Update the Radar Object tracking for tracking manager
                std::pair<uint32_t, int> radar_result;
                radar_result.first = radar_it->first;
                radar_result.second = radar_it->second->classID;
                tracking_manager[fusionID_camera.second][SensorType::RADAR] = radar_result;
                updateAge(fused_object->trackingID, fused_object->classID, 0);
            }
            else if (fusionID_radar.first) {
                // Get Fusion tracking ID
                fused_object->trackingID = fusionID_radar.second.first.first;
                // Update the Camera Object traking for tracking manager
                std::pair<uint32_t, int> camera_result;
                camera_result.first = camera_it->first;
                camera_result.second = camera_it->second->classID;
                tracking_manager[fusionID_radar.second][SensorType::CAMERA] = camera_result;
                updateAge(fused_object->trackingID, fused_object->classID, 0);
            }
            else  // if 2 objects has not fused
            {
                // Get Fusion tracking ID
                fused_object->trackingID = getInternalId(SensorType::FUSION);

                // Update the tracking manager
                std::pair<uint32_t, int> id_result;
                id_result.first = fused_object->trackingID;
                id_result.second = fused_object->classID;

                std::pair<uint32_t, int> camera_result;
                camera_result.first = camera_it->first;
                camera_result.second = camera_it->second->classID;
                tracking_manager[{id_result, 0}][SensorType::CAMERA] = camera_result;

                std::pair<uint32_t, int> radar_result;
                radar_result.first = radar_it->first;
                radar_result.second = radar_it->second->classID;
                tracking_manager[{id_result, 0}][SensorType::RADAR] = radar_result;

                updateAge(fused_object->trackingID, fused_object->classID, 0);
            }

            // Add fusion object to result
            if (is_matched) {
                fusion_result.push_back(fused_object);
            
                // Delete row and column from matrices
                deleteRowAndColumn(matching_matrix, camera_idx, radar_idx);
                deleteRowAndColumn(cost_matrix, camera_idx, radar_idx);

                // Erase current radar and camera objects
                radar_it = matched_radar_objects.erase(radar_it);
                camera_it = matched_camera_objects.erase(camera_it);

                --camera_idx;
                break;
            }
        }

        if (!is_matched) {
            ++camera_it;
        }
    }

    // Re-catch the Matching object pairs by Radar
    for (auto radar_it = matched_radar_objects.begin(); radar_it != matched_radar_objects.end(); ) {
        // Check if any fusion track use this radar track
        auto fusionID_radar = findExistedTrackingObject(SensorType::RADAR, radar_it->first);
        if (!fusionID_radar.first) {
            ++radar_it;
            continue;
        }

        // Check if fused with any camera track
        uint32_t camera_object_ID = findCorrespondEstimateObject(SensorType::CAMERA, fusionID_radar.second.first);
        if (camera_object_ID == 0) {
            ++radar_it;
            continue;
        }

        // Create Fusion output object
        std::shared_ptr<OutputObject> fused_object;

        // Find corresponding Camera object
        static auto predicative = [&] (const std::pair<uint32_t, std::shared_ptr<InputObject>> &entry) -> bool {
            return entry.first == camera_object_ID;
        };
        auto camera_it = std::find_if(matched_camera_objects.begin(), matched_camera_objects.end(), predicative);

        // If found, fuse radar and camera
        if (camera_it != matched_camera_objects.end()) {
            fused_object = getFusedObject(camera_it->second, radar_it->second, camera_sample_time, radar_sample_time);
            fused_object->trackingID = fusionID_radar.second.first.first;

            // Erase Camera object
            matched_camera_objects.erase(camera_it);
        }
        // If not then update separately
        else {
            fused_object = updateSeparateOutputObjects(*radar_it, SensorType::FUSION);
            fused_object->trackingID = fusionID_radar.second.first.first;
            fused_object->classID = fusionID_radar.second.first.second;
        }

        // Reset age
        updateAge(fused_object->trackingID, fused_object->classID, 0);

        // Remove this radar object
        radar_it = matched_radar_objects.erase(radar_it);

        // Push into list
        fusion_result.push_back(fused_object);
    }

    // Re-catch the Matching object pairs by Camera
    for (auto camera_it = matched_camera_objects.begin(); camera_it != matched_camera_objects.end();) {
        auto fusionID_camera = findExistedTrackingObject(SensorType::CAMERA, camera_it->first); 
        if (fusionID_camera.first) {
            std::shared_ptr<OutputObject> fused_object = updateSeparateOutputObjects(*camera_it, SensorType::FUSION);
            fused_object->trackingID = fusionID_camera.second.first.first;
            fused_object->classID = fusionID_camera.second.first.second;
            updateAge(fused_object->trackingID, fused_object->classID, 0);
            fusion_result.push_back(fused_object);
            camera_it = matched_camera_objects.erase(camera_it);
        }
        else
            ++camera_it;
    }

    // Process remaining unmatched camera objects
    for (auto &camera_object : matched_camera_objects) {
        std::shared_ptr<OutputObject> object = std::make_shared<OutputObject>();
        object = updateSeparateOutputObjects(camera_object, SensorType::CAMERA);

        fusion_result.push_back(object);
    }

    // Process remaining unmatched radar objects
    for (auto &radar_object : matched_radar_objects) {
        std::shared_ptr<OutputObject> object = std::make_shared<OutputObject>();
        object = updateSeparateOutputObjects(radar_object, SensorType::RADAR);
        fusion_result.push_back(object);
    }

    updateLifeCycleofManagerTracking();
    return fusion_result;
}   

uint32_t Tracker::findCorrespondEstimateObject(SensorType sensor_type, std::pair<uint32_t, int> fusion_key)
{
    for (const auto& kv : tracking_manager)
    {
        const auto& full_key = kv.first;  
        const auto& sensor_map = kv.second;

        const auto& fusion_id_class = full_key.first;

        if (fusion_id_class == fusion_key) 
        {
            auto sensor_it = sensor_map.find(sensor_type);
            if (sensor_it != sensor_map.end())
                return sensor_it->second.first; 
            else
                return 0;
        }
    }
    return 0;
}

std::pair<bool, std::pair<std::pair<uint32_t, int>, int>> Tracker::findExistedTrackingObject(SensorType sensor_type, uint32_t trackingID)
{
    for (const auto& fusionPair : tracking_manager) {
        const auto& fusionKey = fusionPair.first;
        const auto& fusionValue = fusionPair.second;

        auto it = fusionValue.find(sensor_type);
        if (it != fusionValue.end()) {
            if (it->second.first == trackingID) {
                return {true, fusionKey};
            }
        }
    }
    return {false, {{0, -1}, -1}};
}


std::shared_ptr<OutputObject> Tracker::updateSeparateOutputObjects(std::pair<const uint32_t, std::shared_ptr<InputObject>> input_object,
    SensorType sensor_type)
{
    std::shared_ptr<OutputObject> output_object = std::make_shared<OutputObject>();;
    output_object->location         = input_object.second->location;
    output_object->velocity         = input_object.second->velocity;
    output_object->acceleration     = input_object.second->acceleration;
    output_object->orientation      = input_object.second->orientation;
    output_object->bbox             = input_object.second->bbox;
    output_object->closest_point    = input_object.second->closest_point;
    std::pair<bool, std::pair<uint32_t, int>> id_result;
    if (sensor_type == SensorType::FUSION)
    {
        id_result = {true, {input_object.first, 2}};
        output_object->sensor_source[SensorType::FUSION] = true;
    }
    else if (sensor_type == SensorType::CAMERA)
    {
        id_result = {false, {input_object.first, 2}};
        output_object->sensor_source = input_object.second->sensor_source;
        output_object->sensor_source[SensorType::FUSION] = id_result.first;
        output_object->sensor_source[SensorType::CAMERA] = !id_result.first;
    }
    else
    {
        id_result = {false, {input_object.first, -1}};
        output_object->sensor_source = input_object.second->sensor_source;
        output_object->sensor_source[SensorType::FUSION] = id_result.first;
        output_object->sensor_source[SensorType::RADAR] = !id_result.first;
    }
    output_object->trackingID  = id_result.second.first;
    output_object->classID     = id_result.second.second;
    return output_object;
}

void Tracker::updateAge(uint32_t fusionID, int classID, int newAge)
{
    for (auto it = tracking_manager.begin(); it != tracking_manager.end(); ++it) {
        const auto& key = it->first; // key = {{fusionID, classID}, Age}
        if (key.first.first == fusionID) {
            std::pair<std::pair<uint32_t, int>, int> newKey = key;
            newKey.first.second = classID;
            newKey.second = newAge;
            auto value = it->second;
            tracking_manager.erase(it);
            tracking_manager.emplace(std::move(newKey), std::move(value));
            return;
        }
    }
}


void Tracker::updateLifeCycleofManagerTracking()
{
    decltype(tracking_manager) new_map;

    for (const auto& kv : tracking_manager)
    {
        auto new_key = kv.first;
        new_key.second += 1; // Increase age each processing cycle

        if (new_key.second <= 5) // If ID tracking has age > 5, eliminate it
            new_map[new_key] = kv.second;
    }

    tracking_manager.swap(new_map);
}

int Tracker::calcMatchingOfFusionObjects(const std::shared_ptr<InputObject>& camera_object, const std::shared_ptr<InputObject>& radar_object, 
                                         const float &camera_sample_time,  const float &radar_sample_time)
{
    // Observation error covariance matrix of camera
    // since the camera only observes (x,y) position, but the radar observes (x,y,vx​,vy​)
    // we need to modify the Mahalanobis distance calculation that is based on position (x,y) and velocity (vx​,vy​)
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR> camera_s_matrix;
    // Extract camera (x, y) distance and velocity (vx​,vy​) from Kalman state
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1>  cam_observation; 
    
    // Observation error covariance matrix of radar
    Eigen::Matrix<float,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR> radar_s_matrix;
    // Extract radar (x, y) distance and velocity (vx​,vy​)from Kalman state
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1>  rad_observation;
    
    camera_s_matrix = camera_object->kalmanFilter[0]->getObservationErrorCovarianceMatrix(cam_observation, this->ego_states[SensorType::CAMERA], camera_sample_time);
    radar_s_matrix = radar_object->kalmanFilter[0]->getObservationErrorCovarianceMatrix(rad_observation, this->ego_states[SensorType::RADAR], radar_sample_time);
    // Compute the difference (innovation) between camera and radar distance
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1>  innovation = cam_observation - rad_observation;

    // Compute Observation Error Covariance
    Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR> s_matrix = camera_s_matrix + radar_s_matrix;

    // Manually reduce weight of Observation Error Covariance Matrix -> Reduce the raw Mahalonobis Distance
    Eigen::MatrixXf  scaled_s_matrix = s_matrix.inverse();
    // scaled_s_matrix.coeffRef(0,0) /= 25; 
    // scaled_s_matrix.coeffRef(0,1) /= 25; 
    // scaled_s_matrix.coeffRef(1,0) /= 25; 
    // scaled_s_matrix.coeffRef(1,1) /= 25; 
    
    float delta_x = std::fabs(cam_observation(0) - rad_observation(0));
    float delta_y = std::fabs(cam_observation(2) - rad_observation(2));
    float distance_euler = sqrt(delta_x * delta_x + delta_y * delta_y);
    if (distance_euler > 20)
        return 100000;
    else
    {
        Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1> scaled_innovation = innovation;
        scaled_innovation(0) /= 2;
        scaled_innovation(1) /= 5;
        scaled_innovation(2) /= 2;
        scaled_innovation(3) /= 5;
        // Compute Mahalanobis distance
        float mahalanobis_distance = std::sqrt(std::fabs(scaled_innovation.transpose() * scaled_s_matrix * scaled_innovation));

        // Return as integer cost
        return static_cast<int>(mahalanobis_distance * 1000);
    }
}

std::shared_ptr<OutputObject> Tracker::getFusedObject(std::shared_ptr<InputObject> camera_object, std::shared_ptr<InputObject> radar_object, 
                                                      const float &camera_sample_time, const float &radar_sample_time)
{   
    std::shared_ptr<OutputObject> fused_object = std::make_shared<OutputObject>();
    fused_object->sensor_source[SensorType::FUSION] = true;
    fused_object->classID = camera_object->classID;
    fused_object->bbox = camera_object->bbox;
    fused_object->closest_point = radar_object->closest_point;
    fused_object->segment_mask = camera_object->segment_mask;
    // reserve for future object size if needed
    // fused_object->size = radar_object->size;
    Eigen::Matrix<float, LinearKalmanFilterCam::STATE_VECTOR_SIZE_3D_CAM,  LinearKalmanFilterCam::STATE_VECTOR_SIZE_3D_CAM> camera_process_noise_matrix;
    Eigen::Matrix<float, LinearKalmanFilterCam::STATE_VECTOR_SIZE_3D_CAM, 1>  camera_state_vector; 
    Eigen::Matrix<float, LinearKalmanFilterRadar::STATE_VECTOR_SIZE_RADAR,  LinearKalmanFilterRadar::STATE_VECTOR_SIZE_RADAR> radar_process_noise_matrix;
    Eigen::Matrix<float, LinearKalmanFilterRadar::STATE_VECTOR_SIZE_RADAR, 1>  radar_state_vector;

    camera_object->kalmanFilter[0]->getParametersForFusing(camera_state_vector, camera_process_noise_matrix, this->ego_states[SensorType::CAMERA], camera_sample_time);
    radar_object->kalmanFilter[0]->getParametersForFusing(radar_state_vector, radar_process_noise_matrix, this->ego_states[SensorType::RADAR], radar_sample_time);

    Eigen::MatrixXf cross_covariance_matrix = (1 - this->correlation_scaling_factor) * (radar_state_vector* camera_state_vector.transpose());

    Eigen::MatrixXf fusion_weight_matrix = (radar_process_noise_matrix - cross_covariance_matrix)*
                                           (radar_process_noise_matrix + camera_process_noise_matrix - 
                                           cross_covariance_matrix - cross_covariance_matrix.transpose()).inverse();                                       
    fusion_weight_matrix *= 0.0f;
    Eigen::MatrixXf fused_state_vector = radar_state_vector + fusion_weight_matrix* (camera_state_vector - radar_state_vector);
    // The fused_covariance is used to evaluate the fused values --> used to debug
    // Eigen::MatrixXf fused_covariance =  radar_process_noise_matrix - (radar_process_noise_matrix - cross_covariance_matrix)*
    //                                     (radar_process_noise_matrix + camera_process_noise_matrix - cross_covariance_matrix - cross_covariance_matrix.transpose()).inverse()*
    //                                     (radar_process_noise_matrix - cross_covariance_matrix.transpose());
    fused_object->location << fused_state_vector.coeffRef(0), fused_state_vector.coeffRef(4), radar_object->location.coeffRef(2);
    fused_object->velocity << radar_object->velocity.coeffRef(0), radar_object->velocity.coeffRef(1), radar_object->velocity.coeffRef(2);
    fused_object->acceleration << radar_object->acceleration.coeffRef(0), radar_object->acceleration.coeffRef(1), radar_object->acceleration.coeffRef(2);
    return fused_object;
}               

std::shared_ptr<PerceptionOutputObject> Tracker::getTrackingObjects()
{
    std::lock_guard<std::mutex> lock(*tracker_mutex_);
    std::shared_ptr<PerceptionOutputObject> tracking_result = std::make_shared<PerceptionOutputObject>();
    for (auto it = this->object_data_synthesis_.begin(); it != this->object_data_synthesis_.end(); it++)
    {
        tracking_result->objects.push_back(*it);
    }
    tracking_result->time_stamp = latest_time_stamp[SensorType::FUSION];
    return tracking_result;
}

int Tracker::getInternalId(SensorType sensor_type)
{
    int internal_id = ++this->lastest_id_[sensor_type];
    // Reset the internal ID if it exceeds the threshold to avoid overflow
    if (internal_id > 1000)
    {
        this->lastest_id_[sensor_type] = 0;
        internal_id = 0;
    }

    return internal_id;
}

void Tracker::deleteRowAndColumn(std::vector<std::vector<int>>& matrix, int row_idx, int col_idx) 
{
    // Check if row index is valid, and erase the row
    if (row_idx < matrix.size()) {
        matrix.erase(matrix.begin() + row_idx);
    }

    // Check if column index is valid and erase the column from remaining rows
    for (auto& row : matrix) {
        if (col_idx < row.size()) {
            row.erase(row.begin() + col_idx);
        }
    }
}