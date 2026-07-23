#include "perception.h"

// Pre-applied crop, since I haven't made preprocess of StreamPETR configurable.
#define STREAMPETR_CROP_X 0
#define STREAMPETR_CROP_Y 300
#define STREAMPETR_CROP_W 800
#define STREAMPETR_CROP_H 290

// Initiate the logger
LogUtility Perception::log("log/", "perception");

// #define NUSCENE_MODE

#ifdef NUSCENE_MODE
#include <fstream>
#include <sstream>

std::vector<std::array<std::string, 3>> readNuSceneSamples(std::string dataset_path)
{
    std::vector<std::array<std::string, 3>> samples;

    std::ifstream record_file(dataset_path + "samples.txt");
    if (record_file.is_open()) {
        std::array<std::string, 3> temp_sample;
        for (size_t i = 0; std::getline(record_file, temp_sample[i]);) {
            if (temp_sample[i].empty()) {
                continue;
            }

            i = (i + 1) % 3;
            if (i == 0) samples.push_back(temp_sample);
        }
        record_file.close();
    }
    
    return samples;
}

#endif

cv::Mat visualize_BEV_ego_history(std::vector<float> ego_history)
{
#ifdef NUSCENE_MODE
    static uint32_t view_size = 900;
#else
    static uint32_t view_size = 800;
#endif

    size_t n = ego_history.size() / 16;
    if (n < 1 || ego_history.size() != n * 16) {
        return cv::Mat(view_size, view_size, CV_8UC3, cv::Scalar(255, 255, 255));
    }

    static const cv::Point center(view_size / 2, view_size / 2);
    static const int pixel_per_meter = view_size / 80;
    std::vector<cv::Point> ego_points(n);
    for (int i = 0; i < n; i++) {
        Eigen::Map<Eigen::Matrix4fRM> ego_pose(ego_history.data() + i * 16);
        static Eigen::Vector4f origin(0, 0, 0, 1);
        Eigen::Vector4f transformed_origin = ego_pose * origin;
        ego_points[i] = {int(transformed_origin(0)), int(-transformed_origin(1))};
        ego_points[i] = ego_points[i] * pixel_per_meter + center;
    }

    cv::Mat bev_view(view_size, view_size, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Point prev_point = center;
    cv::Scalar line_color(0, 255, 0);
    cv::Scalar color_delta(0, 255 / n, 0);
    for (auto &point : ego_points) {
        cv::line(bev_view, prev_point, point, line_color, 3);
        prev_point = point;
        line_color -= color_delta;
    }

    return bev_view;
}

Perception::Perception()
: dist_est_thread_pool(4),
  sensor_tracking_thread_pool(2),
  laneMarkingWorld(4)  // TEMPORARY
{
    /* ---- Init StreamPETR model ---- */
    using CAM_CHANNEL = StreamPETR::CAM_CHANNEL;
    StreamPETR::Config model_cfg;
    // Path to model's weights
    model_cfg.weights_path = "/opt/adas/adas_sdk/weights/streampetr/";

    // Model's basic attribute
    model_cfg.position_range = {-61.2f, -61.2f, -10.0f, 61.2f, 61.2f, 10.0f};
    model_cfg.embed_dims = 256;
    model_cfg.depth_start = 1.0f;
    model_cfg.depth_num = 64;
    model_cfg.LID = true;

    // Memory
    model_cfg.num_propagated = 128;
    model_cfg.memory_len = 512;

    // Postprocessing & Decoder
    model_cfg.topk = 128;
    model_cfg.threshold = 0.5f;
    model_cfg.post_center_range = {-61.2f, -61.2f, -10.0f, 61.2f, 61.2f, 10.0f};

#ifndef NUSCENE_MODE
    // Camera Intrinsic & Extrinsics
    model_cfg.img_width[CAM_CHANNEL::F] = STREAMPETR_CROP_W;
    model_cfg.img_height[CAM_CHANNEL::F] = STREAMPETR_CROP_H;
    model_cfg.cam2imgs[CAM_CHANNEL::F] = this->camModel->getCamIntrinsicMatrix();
    model_cfg.ego2cams[CAM_CHANNEL::F] = this->camModel->getCamExtrinsicMatrix();

    // Compensate intrinsic for default crop
    model_cfg.cam2imgs[CAM_CHANNEL::F](1, 2) -= STREAMPETR_CROP_Y;

    // Apply rotation to convert to camera conventional coordinate
    Eigen::Matrix4fRM XrYfZu_to_XrYdZf_mat;
    XrYfZu_to_XrYdZf_mat <<
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -1.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    model_cfg.ego2cams[CAM_CHANNEL::F] = XrYfZu_to_XrYdZf_mat * model_cfg.ego2cams[CAM_CHANNEL::F];

    // Matrix to rotate camera extrinsic from front to back
    Eigen::AngleAxisf rot(M_PI, Eigen::Vector3f::UnitY());
    Eigen::Translation3f trans(0.0f, 0.0f, -3.0f);
    Eigen::Matrix4fRM toRearCam = (trans * rot).matrix();
    // These camera channels are unused, set them to default
    std::vector<CAM_CHANNEL> unused_channel = {
        CAM_CHANNEL::FR,
        CAM_CHANNEL::FL,
        CAM_CHANNEL::B,
        CAM_CHANNEL::BL,
        CAM_CHANNEL::BR
    };
    for (auto channel : unused_channel) {
        model_cfg.img_width[channel] = STREAMPETR_CROP_W;
        model_cfg.img_height[channel] = STREAMPETR_CROP_H;
        model_cfg.cam2imgs[channel] = model_cfg.cam2imgs[CAM_CHANNEL::F];
        model_cfg.ego2cams[channel] = toRearCam * model_cfg.ego2cams[CAM_CHANNEL::F];
    }

    // Virtual lidar space transform
    model_cfg.lidar2ego <<
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 1.8f,
        0.0f, 0.0f, 0.0f, 1.0f;

#else
    model_cfg.img_width[CAM_CHANNEL::F] = 1600;
    model_cfg.img_height[CAM_CHANNEL::F] = 900;
    model_cfg.cam2imgs[CAM_CHANNEL::F] << 
        1266.417203f, 0.f, 816.26702f,
        0.f, 1266.417203f, 491.507066f,
        0.f, 0.f, 1.f;
    model_cfg.ego2cams[CAM_CHANNEL::F] << 
        0.005685f, -0.999983f, 0.000805f, 0.00506f,
        -0.005637f, -0.000837f, -0.999984f, 1.520533f,
        0.999968f, 0.00568f, -0.005641f, -1.692303f,
        0.f, 0.f, 0.f, 1.f;
    
    // These camera channels are unused, set them to default
    std::vector<CAM_CHANNEL> unused_channel = {
        CAM_CHANNEL::FR,
        CAM_CHANNEL::FL,
        CAM_CHANNEL::B,
        CAM_CHANNEL::BL,
        CAM_CHANNEL::BR
    };
    for (auto channel : unused_channel) {
        model_cfg.img_width[channel] = 1600;
        model_cfg.img_height[channel] = 900;
        model_cfg.cam2imgs[channel] = model_cfg.cam2imgs[CAM_CHANNEL::F];
        model_cfg.ego2cams[channel] << 
            0.002422f, 0.999989f, -0.004f, 0.002797f,
            -0.016754f, -0.003959f, -0.999852f, 1.579358f,
            -0.999857f, 0.002488f, 0.016744f, 0.001873f,
            0.f, 0.f, 0.f, 1.f;
    }

    model_cfg.lidar2ego <<
        0.002033f, 0.999704f, 0.024242f, 0.943713f,
        -0.999981f, 0.002176f, -0.005849f, 0.f,
        -0.0059f, -0.024229f, 0.999689f, 1.84023f,
        0.f, 0.f, 0.f, 1.f;
#endif

    // Init model with the config
    this->objectDetector = std::make_unique<StreamPETR>(model_cfg);

    /* ---- Init Tracker ---- */
    this->tracker = Tracker();
    // Perception::log.enableLogFile();    
}

void Perception::convertRadarBboxTo2D(
    std::shared_ptr<InputObject> detection, 
    std::shared_ptr<SensorConfig> radar_config,
    std::shared_ptr<SensorConfig> camera_config
) {
    if (!camera_config || !radar_config) {
        return;
    }
    // ===== Projection Equation =====
    // image_point = K * [R | t] * [X, Y, Z, 1]^T
    //
    // Where:
    // - (X, Y, Z) is the 3D point in world coordinates.
    // - [R | t] is the 3x4 extrinsic matrix composed of:
    //     R: 3x3 rotation matrix
    //     t: 3x1 translation vector
    // - K is the 3x3 intrinsic camera matrix
    // - The result image_point is in homogeneous image coordinates (u, v, w)
    //
    // To get pixel coordinates:
    //     u_pixel = u / w
    //     v_pixel = v / w

    // Extract radar configuration
    Eigen::Matrix3f rotation_radar; 
    rotation_radar = 
        Eigen::AngleAxisf(radar_config->orientation[0], Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(radar_config->orientation[1], Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(radar_config->orientation[2], Eigen::Vector3f::UnitZ());
    Eigen::Vector3f radar_position = radar_config->position;

    // Use detection->bbox for min and max points
    float min_x = detection->bbox[0];  // min_point.x
    float max_x = detection->bbox[2];  // max_point.x
    float min_z = detection->bbox[1];  // min_point.z
    float max_z = detection->bbox[3];  // max_point.z

    // Convert radar bounding box to global coordinates
    // The y value is taken from detection->location, assumed common for both min and max.
    Eigen::Vector3f radar_coords_min(min_x, detection->location.y(), min_z);
    Eigen::Vector3f radar_coords_max(max_x, detection->location.y(), max_z);

    // Transform from radar (local) to global coordinate system.
    Eigen::Vector3f global_coords_min = rotation_radar * radar_coords_min + radar_position;
    Eigen::Vector3f global_coords_max = rotation_radar * radar_coords_max + radar_position;

    // Convert world points to homogeneous coordinates
    Eigen::Vector4f global_min_h(global_coords_min[0], global_coords_min[1], global_coords_min[2], 1.0f);
    Eigen::Vector4f global_max_h(global_coords_max[0], global_coords_max[1], global_coords_max[2], 1.0f);

    // Transform to camera coordinates
    Eigen::Vector4f camera_min_h = camModel->getCamExtrinsicMatrix() * global_min_h;
    Eigen::Vector4f camera_max_h = camModel->getCamExtrinsicMatrix() * global_max_h;

    Eigen::Vector3f camera_coords_min = camera_min_h.head<3>();
    Eigen::Vector3f camera_coords_max = camera_max_h.head<3>();

    // Coordinate adjustment (camera axes conventions)
    camera_coords_min = Eigen::Vector3f(camera_coords_min[0], - camera_coords_min[2], camera_coords_min[1]);
    camera_coords_max = Eigen::Vector3f(camera_coords_max[0], - camera_coords_max[2], camera_coords_max[1]);

    // Project to 2D using intrinsic matrix
    Eigen::Vector3f projected_point_min = camModel->getCamIntrinsicMatrix() * camera_coords_min;
    Eigen::Vector3f projected_point_max = camModel->getCamIntrinsicMatrix() * camera_coords_max;

    projected_point_min[0] /= projected_point_min[2];
    projected_point_min[1] /= projected_point_min[2];

    projected_point_max[0] /= projected_point_max[2];
    projected_point_max[1] /= projected_point_max[2];

    // Convert to screen space (flip x-axis for visual_x)
    // visual_x (min_x)
    // visual_y (min_z)
    // visual_x (max_x)
    // visual_y (max_z)
    detection->bbox << projected_point_min[0], projected_point_max[1], projected_point_max[0], projected_point_min[1];
}


std::shared_ptr<TrackedSensorData> Perception::Tracking(
    std::shared_ptr<SensorDataType> current_sensor_data,
    std::shared_ptr<const SteeringParameters> steeringAngle,
    std::shared_ptr<const ImuParameters> imu,
    std::shared_ptr<const GnssPoint> gnss,
    std::shared_ptr<const OdometryParameters> odometer,
    const bool is_reverse
)
{
    std::shared_ptr<SensorDataType> processing_data = current_sensor_data->clone();
    uint64_t current_ts = processing_data->time_stamp;

    // Compute ego state from the matched IMU and ego velocity data.
    // Filter for State Ego Car values which is unreasonable when spam scenarios
    static constexpr float ego_bounding_vel = 90.0;
    static constexpr float ego_bounding_acc = 50.0;
    Eigen::Matrix<float, 4, 1> ego_states;
    if (abs(-imu->Imu_linear_acceleration.y) <= ego_bounding_acc
        && abs(-imu->Imu_linear_acceleration.x) <= ego_bounding_acc
        && abs(odometer->linear_velocity.x) <= ego_bounding_vel
        && abs(odometer->linear_velocity.y) <= ego_bounding_vel)
    {
        ego_states << odometer->linear_velocity.x,
                      -imu->Imu_linear_acceleration.y,
                      odometer->linear_velocity.y,
                      imu->Imu_linear_acceleration.x;           
        pre_ego_states = ego_states;
    }
    else
        ego_states = pre_ego_states;

    // Convert sensor data into the target coordinate system.
    processing_data->converTo();
    // Determine sensor type (e.g., CAMERA or RADAR).
    SensorType sensor_type = processing_data->getSensorType();

    // Update the tracker state for the given sensor type.
    // This function encapsulates prediction, matching, update, new track initialization, and cleanup.
    std::shared_ptr<TrackedSensorData> temp;
    {
        std::lock_guard<std::mutex> guard(perceptionInternalMutex);
        temp =  this->tracker.processSensorTracking(processing_data, ego_states, sensor_type);
    }

    if (sensor_type == SensorType::RADAR)
    {
        auto& objects = temp->matched_objects;
        for (auto it = objects.begin(); it != objects.end(); ) 
        {
            if (isfinite(it->second->location.x()) && isfinite(it->second->location.y()))
                ++it;
            else
                it = objects.erase(it);        
        }
    }

    // Filter out objects based on class ID for Camera Tracking in initial phase of LKF Camera when spam new scenarios
    // Bounding Velocity value to filter the Camera Object velocity
    static constexpr float obj_bounding_vel = 40.0f;
    static constexpr std::array<int, 6> valid_class_ids = {0, 1, 2, 3, 5, 7};  // person, bicycle, car, motorcycle, bus, truck
    if (sensor_type == SensorType::CAMERA) {
        auto& objects = temp->matched_objects;
        for (auto it = objects.begin(); it != objects.end(); ) {
            bool valid_id = std::find(valid_class_ids.begin(), valid_class_ids.end(), it->second->classID) != valid_class_ids.end();
            bool valid_velocity = it->second->velocity.x() < obj_bounding_vel && it->second->velocity.y() < obj_bounding_vel;
            if (valid_id && valid_velocity)
                ++it;
            else
                it = objects.erase(it);
        }
    }

    return temp;
}


/**
 * @brief Main execution pipeline, this function is called every iteration regardless of the state of the system and imu parameters are passed in.
 * 
 * @param srcImageQueue 
 * @param velocityQueue 
 * @param radar_pointscloud_queue
 * @param configs 
 * @param imu_parameters
 * @return PerceptionOutputObject 
 */
PerceptionOutputObject Perception::execute(
    std::queue<std::shared_ptr<TrackedSensorData>>& current_data_queue
)
{
    // Perform fusion and return the fusion enviroment
    {
        std::lock_guard<std::mutex> guard(perceptionInternalMutex);
        this->tracker.performFusion(current_data_queue);
    }

    // Get fusion data after performing fusion
    auto result_object_pointer = this->tracker.getTrackingObjects();
    for (auto it = result_object_pointer->objects.begin(); it != result_object_pointer->objects.end(); ) 
    {
        auto &object = *it;
        float valid_dist = object->location.norm() > object->closest_point.norm();
        if (!valid_dist && object->sensor_source[SensorType::RADAR] && object->classID == -1) 
        {
            it = result_object_pointer->objects.erase(it);
            continue;
        }
        // Just apply Average Filter for the moving Object cars. Because when the car stop, the Average filter will slow down the reponse of velocity status.
        if (object->classID == 2 && object->sensor_source[SensorType::FUSION])
        {
            if (object->velocity.x() != 0)
            {
                // Determine the range of velocity
                float velocity = std::sqrt(object->velocity.x() * object->velocity.x() + object->velocity.y() * object->velocity.y());
                int choose_threshold = -1;      // choose_threshold = 0 -> Low range velocity -> Need high window size to keep the stable of value
                                                // choose_threshold = 1 -> Medium to High range velocity -> Need low window size to keep the fast response
                if(abs(velocity) < threshold_filter_velocityX)    
                    choose_threshold = 0;
                else
                    choose_threshold = 1;
                // Filter for Velocity X of object tracking car
                float & vel_x = object->velocity.x();
                // object->velocity.x() = filterVelocityX(vel_x);
                smoothVelocity(object->trackingID, vel_x, choose_threshold);
                // object->velocity.x() = median_filter(vel_x);
            }
            if (object->acceleration.x() != 0)
            {
                // Determine the range of velocity
                float velocity = std::sqrt(object->velocity.x() * object->velocity.x() + object->velocity.y() * object->velocity.y());
                int choose_threshold = -1;      // choose_threshold = 0 -> Low range velocity -> Need high window size to keep the stable of value
                                                // choose_threshold = 1 -> Medium to High range velocity -> Need low window size to keep the fast response
                if(abs(velocity) < threshold_filter_velocityX)    
                    choose_threshold = 0;
                else
                    choose_threshold = 1;
                // Filter for Velocity X of object tracking car
                float & acc_x = object->acceleration.x();
                // object->velocity.x() = filterVelocityX(vel_x);
                smoothAcceleration(object->trackingID, acc_x, choose_threshold);
            }
        }
        ++it;
    }

    // Cleanup stale history to prevent memory leaks
    std::vector<uint32_t> active_ids;
    active_ids.reserve(result_object_pointer->objects.size());
    for (const auto& obj : result_object_pointer->objects) {
        active_ids.push_back(obj->trackingID);
    }
    std::sort(active_ids.begin(), active_ids.end());

    // Remove entries for objects that are no longer tracked
    auto cleanup_map = [&](auto& map) {
        for (auto it = map.begin(); it != map.end(); ) {
            if (!std::binary_search(active_ids.begin(), active_ids.end(), it->first)) {
                it = map.erase(it);
            } else {
                ++it;
            }
        }
    };

    cleanup_map(object_VelocityX_OldValue_LPF);
    cleanup_map(object_MedianFilter_Buffer);
    cleanup_map(object_AccelerationX_History);

    return *result_object_pointer;
}

std::string Perception::debugFusionObjects(const std::vector<FusionObject>& fusion_objects) {
    std::stringstream ss;
    for (const auto& obj : fusion_objects) {
        if (obj.from_camera && obj.from_radar) {
            ss << "Both: " << obj.x_offset << " " << obj.y_offset << "\n";
        } else if (obj.from_camera) {
            ss << "Camera: " << obj.x_offset << " " << obj.y_offset << "\n";
        } else if (obj.from_radar) {
            ss << "Radar: " << obj.x_offset << " " << obj.y_offset << "\n";
        } else {
            ss << "None: " << obj.x_offset << " " << obj.y_offset << "\n";
        }
    }
    return ss.str();
}


cv::Mat Perception::objectDetectionPipeline(
    uint64_t timestamp,
    cv::Mat img,
    std::shared_ptr<const OdometryParameters> odo
)
{
    // std::lock_guard<std::mutex> guard(objectDetectionMutex);

    if(img.empty()) {
        return img;
    }

    /* ---- Prepare input ---- */
    using CAM_CHANNEL = StreamPETR::CAM_CHANNEL;
    StreamPETR::Input input_data;

#ifndef NUSCENE_MODE
    // Set timestamp
    input_data.timestamp = static_cast<double>(timestamp) / 1e9;

    // Set images (only front channel, other images default to blank)
    // Apply a precrop here
    static const cv::Rect roi(STREAMPETR_CROP_X, STREAMPETR_CROP_Y, STREAMPETR_CROP_W, STREAMPETR_CROP_H);
    input_data.rawImgs[CAM_CHANNEL::F] = img(roi);

    static const cv::Mat blank_img(STREAMPETR_CROP_H, STREAMPETR_CROP_W, CV_8UC3, cv::Scalar(103.53f, 116.28f, 123.675f));
    input_data.rawImgs[CAM_CHANNEL::FR] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::FL] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::B] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::BL] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::BR] = blank_img;

    // Set ego pose
    input_data.ego_pose = getEgoPoseTransform(odo);
#else
    boost::this_thread::sleep_for(boost::chrono::milliseconds(400));
    static std::string dataset_path = "/opt/adas/adas_sdk/nuscenes/";
    static std::vector<std::array<std::string, 3>> samples = readNuSceneSamples(dataset_path);
    static size_t sample_idx = 0;
    if (sample_idx >= samples.size()) {
        return cv::Mat();
    }
    std::array<std::string, 3> &current_sample = samples[sample_idx];
    sample_idx++;

    // timestamp
    input_data.timestamp = std::stoull(current_sample[1]) / 1e6;

    // images
    img = cv::imread(dataset_path + current_sample[0], cv::IMREAD_COLOR);
    input_data.rawImgs[CAM_CHANNEL::F] = img;

    static const cv::Mat blank_img(900, 1600, CV_8UC3, cv::Scalar(103.53f, 116.28f, 123.675f));
    input_data.rawImgs[CAM_CHANNEL::FR] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::FL] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::B] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::BL] = blank_img;
    input_data.rawImgs[CAM_CHANNEL::BR] = blank_img;

    // pose
    std::stringstream ss(current_sample[2]);
    for (size_t i = 0; i < 16; i++) {
        ss >> input_data.ego_pose.data()[i];
    }
#endif

    /* ---- Objects detection ---- */
    StreamPETR::Output output_data = objectDetector->infer(input_data);
    auto ego_history = this->objectDetector->detector_->bindings["memory_egopose"]->cpu();
    for (int i = 1; i < 4; i++) {
        std::copy(ego_history.begin() + i * 128 * 16, ego_history.begin() + i * 128 * 16 + 16, ego_history.begin() + i * 16);
    }
    ego_history.resize(4 * 16);
    cv::Mat bev_view = visualize_BEV_ego_history(ego_history);

    /* ---- Visualize ---- */
    static Eigen::Matrix4fRM ego2img;
    static bool _init_ego2cam_ = false;
    if (_init_ego2cam_ == false) {
#ifndef NUSCENE_MODE
        Eigen::Matrix3fRM intrinsic = this->camModel->getCamIntrinsicMatrix();
        Eigen::Matrix4fRM extrinsic = this->camModel->getCamExtrinsicMatrix();

        Eigen::Matrix4fRM XrYfZu_to_XrYdZf_mat;
        XrYfZu_to_XrYdZf_mat <<
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, -1.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;

        Eigen::Matrix4fRM ego2cam = XrYfZu_to_XrYdZf_mat * extrinsic;
#else

        Eigen::Matrix3fRM intrinsic;
        Eigen::Matrix4fRM extrinsic;
        intrinsic << 
            1266.417203f, 0.f, 816.26702f,
            0.f, 1266.417203f, 491.507066f,
            0.f, 0.f, 1.f;
        extrinsic << 
            0.005685f, -0.999983f, 0.000805f, 0.00506f,
            -0.005637f, -0.000837f, -0.999984f, 1.520533f,
            0.999968f, 0.00568f, -0.005641f, -1.692303f,
            0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix4fRM ego2cam = extrinsic;
#endif
        Eigen::Matrix4fRM cam2img = Eigen::Matrix4fRM::Identity();
        cam2img.topLeftCorner<3, 3>() = intrinsic;
        ego2img = cam2img * ego2cam;
        _init_ego2cam_ = true;
    }

    img = img.clone();
    draw3DBoundingBoxes(img, output_data, ego2img);
    
    /* ---- Convert to InputObject ---- */
    this->objects.clear();
    // // Disabled since model return 3D output while system only handle 2D
    // this->objects.reserve(detected_objects.size());
    // for (auto& object : detected_objects) {
    //     int x_center = static_cast<int>((object.bbox.x1 + object.bbox.x2) / 2);
    //     int y_bottom = static_cast<int>(object.bbox.y2);

    //     std::shared_ptr<InputObject> detection = std::make_shared<InputObject>();
    //     detection->bbox << object.bbox.x1, object.bbox.y1, 
    //                        object.bbox.x2, object.bbox.y2;
    //     detection->location << perceptionModel.estimateXDistance(x_center, y_bottom),
    //                            perceptionModel.estimateYDistance(x_center, y_bottom),
    //                            0.0f;
    //     detection->velocity << UNKNOWN_VELOCITY, UNKNOWN_VELOCITY, UNKNOWN_VELOCITY;
    //     detection->classID = object.classId;
    //     detection->sensor_source[SensorType::CAMERA] = true;
    //     detection->segment_mask = object.mask;
    //     this->objects.push_back(detection);
    // }
    cv::Mat combined;
    cv::hconcat(bev_view, img, combined);
    return combined;
}


Eigen::Matrix4dRM Perception::getEgoPoseTransform(
    std::shared_ptr<const OdometryParameters> odo
)
{
    // 0. Memory of last pose and timestamp
    static uint64_t timestamp = odo->time_stamp;
    static Eigen::Matrix4dRM ego_pose = Eigen::Matrix4dRM::Identity();
    
    // 1. Calculate time delta
    double dt = static_cast<double>(odo->time_stamp - timestamp) / 1e9;
    if (dt <= 0) {
        // return last ego pose if dt is invalid
        return ego_pose;
    }

    // 2. Extract velocities from odo message
    double vx = odo->linear_velocity.x;  // right
    double vy = odo->linear_velocity.y;  // forward
    double vz = odo->linear_velocity.z;  // up

    double wp = odo->angular_velocity.x; // pitch
    double wr = odo->angular_velocity.y; // roll
    double wy = odo->angular_velocity.z; // yaw

    // 3. Create the Translation Delta (Local Frame)
    Eigen::Vector3d translation_delta(vx * dt, vy * dt, vz * dt);

    // 4. Create the Rotation Delta (Correct Order: Yaw -> Pitch -> Roll)
    Eigen::AngleAxisd pitch_delta(wp * dt,  Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd roll_delta(wr * dt, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_delta(wy * dt,   Eigen::Vector3d::UnitZ());

    // Combined rotation: Applied Right-to-Left
    Eigen::Quaterniond rotation_delta = yaw_delta * pitch_delta * roll_delta;

    // 5. Update the Ego to Global Transformation
    // We use a temporary Affine3d to represent this specific step's movement
    Eigen::Affine3d T_step = Eigen::Affine3d::Identity();
    T_step.translate(translation_delta);
    T_step.rotate(rotation_delta);

    // Apply the update: Ego2Global = Ego2Global * Local_Step
    timestamp = odo->time_stamp;
    ego_pose = ego_pose * T_step.matrix();

    return ego_pose;
}


void Perception::draw3DBoundingBoxes(
    cv::Mat &image,
    const StreamPETR::Output &output_data,
    const Eigen::Matrix4fRM &ego2img
)
{
    for (const auto& det : output_data.objects) {
        // 1. Define the 8 corners of the box in the local object coordinate system
        // Assuming: w is along x, l is along y, h is along z
        const float dx = det.w / 2.0f;
        const float dy = det.l / 2.0f;
        const float dz = det.h / 2.0f;

        Eigen::Matrix<float, 8, 4, Eigen::RowMajor> corners_local;
        corners_local <<
            dx,  dy,  dz, 1.0f,
            dx, -dy,  dz, 1.0f,
           -dx, -dy,  dz, 1.0f,
           -dx,  dy,  dz, 1.0f,
            dx,  dy, -dz, 1.0f,
            dx, -dy, -dz, 1.0f,
           -dx, -dy, -dz, 1.0f,
           -dx,  dy, -dz, 1.0f;

        // 2. Construct the Object-to-Ego Transform (Translation + Rotation)
        Eigen::Translation3f translation(det.cx, det.cy, det.cz);
        Eigen::AngleAxisf rotation(det.yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4fRM obj2ego = (translation * rotation).matrix();
        Eigen::Matrix4fRM obj2img = ego2img * obj2ego;

        // 3. Project corners to Image Plane
        int num_in_view = 0;
        std::vector<cv::Point> pts_2d(8);
        for (int i = 0; i < 8; ++i) {
            Eigen::Vector4f pt_img = obj2img * corners_local.row(i).transpose();

            // Check if valid depth
            if (pt_img.z() <= 0) {
                num_in_view = 0;
                break;
            }

            // Perspective division
            auto &p = pts_2d[i];
            p = {int(pt_img.x() / pt_img.z()), int(pt_img.y() / pt_img.z())};

            // Count if point within image
            if ((0 <= p.x && p.x < image.cols) && (0 <= p.y && p.y < image.rows)) {
                num_in_view++;
            }
        }

        if (num_in_view == 0 || pts_2d.size() < 8) continue;

        // 4. Draw the edges
        static const cv::Scalar color(0, 255, 0); // Green
        static constexpr int thickness = 2;

        // Draw top and bottom rectangles
        for (int i = 0; i < 4; ++i) {
            // Bottom face (0-3)
            cv::line(image, pts_2d[i], pts_2d[(i + 1) % 4], color, thickness);
            // Top face (4-7)
            cv::line(image, pts_2d[i + 4], pts_2d[((i + 1) % 4) + 4], color, thickness);
            // Vertical pillars
            cv::line(image, pts_2d[i], pts_2d[i + 4], color, thickness);
        }
        
        // Optional: Draw a "front" indicator to show heading
        cv::line(image, pts_2d[0], pts_2d[5], cv::Scalar(0, 0, 255), thickness); // Red front line
    }
}


void Perception::fitPolyToPoints(std::vector<cv::Point2f> const &points,
                                 int const degree,
                                 std::vector<float> &coeffs)
{
    coeffs.clear();
    
    // Return if not enough points
    if (points.size() <= degree) {
        return;
    }
    
    // Extract xs and ys mat
    cv::Mat pointsMat(points);
    std::vector<cv::Mat> xys(2); // 2 mats to hold xs and ys
    cv::split(pointsMat, xys);
    cv::Mat xs = xys[0], ys = xys[1];

    // Prepare Y, Yt, inv(Yt * Y)
    cv::Mat Y = cv::Mat::ones(ys.rows, degree+1, CV_32FC1);
    cv::Mat Yt, Yt_Y_inv;
    for(int i = 1; i < degree+1; i++) {
        cv::pow(ys, i, Y.col(i));
    }
    cv::transpose(Y, Yt);
    cv::invert(Yt * Y, Yt_Y_inv);

    // Coeff = inv(Yt * Y) * Yt * xs
    cv::Mat coeffsMat = Yt_Y_inv * Yt * xs;
    coeffs.resize(degree + 1);
    for (int i = 0; i < degree + 1; i++)
        coeffs[i] = coeffsMat.at<float>(i, 0);
}


void Perception::laneDetectorPipeline(const cv::Mat &img)
{
    // std::lock_guard<std::mutex> guard(laneDetectorMutex);

    if(img.empty()) {
        return;
    }

    // boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
    laneDetector->detect(img);
}


void Perception::retrieveLaneResult(std::shared_ptr<LaneDetection> lane_result)
{
    lane_result->laneLinesCount      = laneDetector->getLaneLinesCount();
    lane_result->averageLanePosition = laneDetector->getAverageLanePosition();
    lane_result->laneMarkingsPoint   = laneDetector->getLanes();
    getRealWorldLanes(lane_result);
}


/**
 * @brief Update lane status based on the lane detection.
 * Checking if lane is missing, not in lane or is in lane.
 * 
 * @return LaneStatus value after updating.
 * Check intermediate representation for detailed value info.
 */
int Perception::updateLaneStatus(const std::shared_ptr<LaneDetection> lane_result)
{
    // Static settings
    static constexpr int maxHasLaneCount = 40;    // Max number of frame
    static constexpr int maxInLaneCount  = 20;
    static constexpr float point0_y      = 4.0f;  // meters
    static constexpr float point1_y      = 6.0f;  // meters
    static constexpr float center_y      = 1.0f;  // meters
    static constexpr float distThresh    = 0.8f;  // meters
    static constexpr float angleThresh   = 15 * (M_PI / 180);

    // Information from lane detection
    const auto &leftLaneMark = lane_result->laneMarkingsWorld[LaneMarkingID::LEFT_MID];
    const auto &rightLaneMark = lane_result->laneMarkingsWorld[LaneMarkingID::RIGHT_MID];

    // Check if middle lane is visible
    bool hasLeftMid = leftLaneMark.type != LaneMarkingType::NOT_EXIST;
    bool hasRightMid = rightLaneMark.type != LaneMarkingType::NOT_EXIST;
    bool laneVisible = hasLeftMid && hasRightMid;

    // Update missing lane status
    if (!laneVisible) {
        this->frameHasLaneCounter--;
        if (this->frameHasLaneCounter <= 0) {
            this->laneStatus = LaneStatus::MISSING_LANE;
            this->frameHasLaneCounter = 0;
        }
    }
    else if (this->laneStatus == LaneStatus::MISSING_LANE) {
        this->laneStatus = LaneStatus::NOT_IN_LANE;
        this->frameHasLaneCounter = maxHasLaneCount;
    }

    // Check if lane is visible
    if (this->laneStatus == LaneStatus::MISSING_LANE) {
        inLaneCounter = 0;
        return this->laneStatus;
    }

    // Check if vehicle is align and close to center of current middle lane
    bool directionAlign = false;
    bool nearCenter = false;
    if (laneVisible) {
        float point0_x = (leftLaneMark.x(point0_y) + rightLaneMark.x(point0_y)) / 2;
        float point1_x = (leftLaneMark.x(point1_y) + rightLaneMark.x(point1_y)) / 2;
        float laneAngle = std::atan((point1_x - point0_x) / (point1_y - point0_y));
        directionAlign = (std::abs(laneAngle) < angleThresh);

        float center_x = (leftLaneMark.x(center_y) + rightLaneMark.x(center_y)) / 2;
        nearCenter = (std::abs(center_x) < distThresh);
    }

    // Increase counter if lane and vehicle is align
    if (directionAlign && nearCenter) {
        inLaneCounter++;
    }
    else {
        inLaneCounter--;
    }

    // Change state if counter reach min or max
    if (inLaneCounter >= maxInLaneCount) {
        this->laneStatus = LaneStatus::IN_LANE;
        inLaneCounter = maxInLaneCount;
    }
    else if (inLaneCounter <= 0) {
        this->laneStatus = LaneStatus::NOT_IN_LANE;
        inLaneCounter = 0;
    }

    return this->laneStatus;
}


void Perception::getRealWorldLanes(const std::shared_ptr<LaneDetection> lane_result)
{
    /** Transform lane points from camera view to Realworld Oxy coordinate */
    std::vector<std::vector<cv::Point2f>> pointsWorld(LaneMarkingID::NUM_LANE_MARKING);
    for (int i = 0; i < LaneMarkingID::NUM_LANE_MARKING; i++) {
        convertPointsToWorld(lane_result->laneMarkingsPoint[i].points, pointsWorld[i]);

        // Drop points too far away
        static constexpr float rawDistLimit = 60.0f;
        for (auto it = pointsWorld[i].begin(); it != pointsWorld[i].end(); it++) {
            if (0 < it->y && it->y < rawDistLimit) {
                pointsWorld[i].erase(pointsWorld[i].begin(), it);
                break;
            }
        }
    }

    /** Fit polynomial to represent lanes in Realworld coordinate */
    std::vector<std::vector<float>> coeffsWorld(LaneMarkingID::NUM_LANE_MARKING);
    for (int i = 0; i < LaneMarkingID::NUM_LANE_MARKING; i++) {
        fitPolyToPoints(pointsWorld[i], POLYNOMIAL_DEGREE, coeffsWorld[i]);
    }

    /** @note THIS IS TEMPORARY HARD LIMIT */
    static constexpr float endYHardLimit = 42.0f;

    /** Update lane result */
    lane_result->laneMarkingsWorld.resize(LaneMarkingID::NUM_LANE_MARKING);
    for (int i = 0; i < LaneMarkingID::NUM_LANE_MARKING; i++) {
        if (!coeffsWorld[i].empty()) {
            lane_result->laneMarkingsWorld[i].type = lane_result->laneMarkingsPoint[i].type;
            lane_result->laneMarkingsWorld[i].startY = pointsWorld[i].back().y;
            lane_result->laneMarkingsWorld[i].endY = pointsWorld[i].front().y;
            lane_result->laneMarkingsWorld[i].xCoeffs = coeffsWorld[i];
            // lane_result->laneMarkingsWorld[i].zCoeffs = std::vector<float>(degree + 1, 0);
            
            /** THIS IS TEMPORARY ********************************************
             * As Policy state machine can't handle case when there is no lane,
             * so old data is needed to provide lane info. Otherwise the policy
             * state machine will fail.
             * This section should be removed and replaced ASAP
             */
            if (lane_result->laneMarkingsWorld[i].endY > endYHardLimit) {
                lane_result->laneMarkingsWorld[i].endY = endYHardLimit;
            }
            laneMarkingWorld[i].startY = lane_result->laneMarkingsWorld[i].startY;
            laneMarkingWorld[i].endY = lane_result->laneMarkingsWorld[i].endY;
            laneMarkingWorld[i].xCoeffs = lane_result->laneMarkingsWorld[i].xCoeffs;
        }
        else {
            if (!laneMarkingWorld[i].xCoeffs.size()) {
                laneMarkingWorld[i].startY = 0.0f;
                laneMarkingWorld[i].endY = 0.0f;
                laneMarkingWorld[i].xCoeffs = std::vector<float>(POLYNOMIAL_DEGREE + 1, 0.0f);
            }
            lane_result->laneMarkingsWorld[i].type = lane_result->laneMarkingsPoint[i].type;
            lane_result->laneMarkingsWorld[i].startY = laneMarkingWorld[i].startY;
            lane_result->laneMarkingsWorld[i].endY = laneMarkingWorld[i].endY;
            lane_result->laneMarkingsWorld[i].xCoeffs = laneMarkingWorld[i].xCoeffs;
        }
            /************************* END OF TEMPORARY SECTION **********************/
    }

    // Debug
    // {
        // unsigned int width = 675;
        // unsigned int height = 675;
        // float max_dist = 60.0f;
        // float ppm_ratio = max(width, height) / max_dist;
        // float x_offset = -1 * (width / 2.0f);
        // float y_offset = 2.0f * ppm_ratio;

        // cv::Mat debug = cv::Mat::zeros(width, height, CV_8UC3);

        // // 35m rect
        // cv::rectangle(debug, cv::Point2f(-1 * (35.0f / 2.0f) * ppm_ratio - x_offset, (height -1) - (35.0f * ppm_ratio) - y_offset),
        //                 cv::Point2f((35.0f / 2.0f) * ppm_ratio - x_offset, (height - 1) - (0.0f * ppm_ratio) - y_offset),
        //                 cv::Scalar(200, 200, 240),
        //                 2);

        // // Converted points
        // for (int i = 0; i < LaneMarkingID::NUM_LANE_MARKING; i++) {
        //     if (lane_result->laneMarkingsWorld[i].type == LaneMarkingType::NOT_EXIST) 
        //         continue;
        //     for (auto &point: pointsWorld[i]) {
        //         point.x = point.x * ppm_ratio - x_offset;
        //         point.y = (height - 1) - (point.y * ppm_ratio) - y_offset;
        //         cv::circle(debug, point, 2, cv::Scalar(200, 200, 200), cv::FILLED);
        //     }
        // }

        // cv::imshow("lane_world_point", debug);
    // }
}

// Helper -- Average Filter to Smooth velocity value
void Perception::smoothVelocity(uint32_t fusionID, float & new_velocity, int choose_threshold) 
{
    float alpha = 0;
    if (choose_threshold == 0)
        alpha = 0.05;
    else if (choose_threshold == 1)
        alpha = 0.20;
    
    new_velocity = median_filter(fusionID, new_velocity);

    float old_velocity = new_velocity; // Default initialization (no filtering on first frame)

    if (object_VelocityX_OldValue_LPF.find(fusionID) != object_VelocityX_OldValue_LPF.end()) {
        old_velocity = object_VelocityX_OldValue_LPF[fusionID];
        
        // Apply filter logic
        new_velocity = new_velocity * alpha + (1.0f - alpha) * old_velocity;
    }
    
    // Update history
    object_VelocityX_OldValue_LPF[fusionID] = new_velocity;
}

float Perception::median_filter(uint32_t fusionID, float x)
{
    // Use deque to maintain a sliding window of size 5 per object
    auto& buffer = object_MedianFilter_Buffer[fusionID];

    buffer.push_back(x);
    if (buffer.size() > 5) {
        buffer.pop_front();
    }
    
    // Copy to vector for sorting
    std::vector<float> temp(buffer.begin(), buffer.end());
    size_t size = temp.size();
    
    if (size == 0) return x; // Should not happen given push_back above

    std::sort(temp.begin(), temp.end());

    // Return median
    if (size % 2 == 0) {
        return (temp[size/2 - 1] + temp[size/2]) / 2.0f;
    } else {
        return temp[size/2];
    }
}

void Perception::smoothAcceleration(uint32_t fusionID, float & new_accleretion, int choose_threshold) 
{
    size_t window_size = 0;
    if (choose_threshold == 0) window_size = filter_window_size_low;
    else if (choose_threshold == 1) window_size = filter_window_size_high;

    auto& history_velocity = object_AccelerationX_History[fusionID];
    history_velocity.push_back(new_accleretion);
    if (history_velocity.size() > window_size)  history_velocity.pop_front();

    double sum = 0.0;
    for (auto velocity : history_velocity) sum += velocity;
    new_accleretion = sum / static_cast<double>(history_velocity.size());
}