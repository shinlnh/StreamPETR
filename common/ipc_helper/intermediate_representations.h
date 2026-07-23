#ifndef INTERMEDIATE_REPRESENTATION_H
#define INTERMEDIATE_REPRESENTATION_H

#include <array>
#include <vector>
// #include <QObject>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "common.h"
#include "traffic_object.h"
#include "sensor/config.h"
#include "linear_kalman_filter.hpp"
#include "linear_kalman_filter_for_cam.hpp"
#include "linear_kalman_filter_for_radar.hpp"

// ------------------------------ SENSOR DATA ---------------------------------
/*
This code defines a flexible and extensible framework for handling sensor data from various sources like cameras and radars. 

1. **Base Class: SensorData**
   - The `SensorData` class is a generic base class designed to store common attributes shared by all sensor types. 
   - It includes:
     - `time_stamp`: A time_stamp to track when the data was captured.
     - `objects`: A collection of detected objects represented as shared pointers for efficient memory management.
   - The base class constructor allows initialization of these common fields, while the virtual destructor ensures proper cleanup in derived classes.

2. **Specialized Classes for Each Sensor Type**
   - The framework uses template specialization to define sensor-specific data structures by extending the base `SensorData` class:
   
   **a. Camera Sensor (`SensorData<SensorType::CAMERA>`)**
   - Adds fields specific to Camera data, such as:
     - `cv::Mat image`: The raw image captured by the camera.
     - `std::shared_ptr<LaneDetection> laneResults`: Results of lane detection for camera data.
   - This specialization ensures that only fields relevant to the Camera sensor are included.

   **b. Radar Sensor (`SensorData<SensorType::RADAR>`)**
   - Adds fields specific to Radar data, such as:
     - `std::shared_ptr<RadarPointCloud> radarClouds`: Point cloud data captured by the radar.
   - This specialization similarly isolates Radar-specific data to avoid unnecessary fields.
*/
// Fix: currently fixing here 
enum class SensorType : int8_t {
    UNDEFINED = 0,
    CAMERA,
    RADAR,
    API_SENSOR,
    FUSION,
    SENSOR_TYPE_COUNT
};

/** 
 * SensorObject: Represents raw detections from sensors (camera, LiDAR, radar). 
 * Contains basic attributes like position, size, and classification (e.g., vehicle, pedestrian). 
 */
struct Object {
    Eigen::Vector3f location      = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity      = Eigen::Vector3f::Zero();
    Eigen::Vector3f acceleration  = Eigen::Vector3f::Zero();
    Eigen::Vector3f closest_point = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    Eigen::Vector2f orientation   = Eigen::Vector2f::Zero();
    Eigen::Vector4f bbox          = Eigen::Vector4f::Zero();
    int classID                   = -1;
    int classCount                = 1;
    std::map<SensorType, bool> sensor_source;
    
    cv::Mat segment_mask;

    Object()
    {
        for(SensorType sensor = static_cast<SensorType>(0);
            sensor < SensorType::SENSOR_TYPE_COUNT;
            sensor = SensorType(static_cast<int>(sensor) + 1))
        {
            sensor_source[sensor] = false;
        }
    }
    virtual ~Object () = default;

    std::shared_ptr<Object> clone() const {
        auto cloned = std::make_shared<Object>(*this);
        cloned->segment_mask = this->segment_mask.clone();
        return cloned;
    }
};

/** 
 * InputObject: Processed sensor data used for tracking. 
 * Includes Kalman filter states and functions for prediction. 
 */
struct InputObject : Object {

    // kalman filter injection for tracking
    std::vector<std::shared_ptr<LinearKalmanFilter>> kalmanFilter;
    
    int32_t life_hit_streak_ = 0;
    int32_t death_hit_streak_ = 0;

    std::shared_ptr<InputObject> clone() const {
        auto cloned = std::make_shared<InputObject>(*this);
        cloned->segment_mask = this->segment_mask.clone();
        for (std::shared_ptr<LinearKalmanFilter> filter : cloned->kalmanFilter) {
            if(filter) filter = std::make_shared<LinearKalmanFilter>(*filter);
        }
        return cloned;
    }

    void update(std::shared_ptr<InputObject> obj, const Eigen::Matrix<float, 4, 1>& ego_states = Eigen::Matrix<float, 4, 1>::Zero());
    void predict(const float &radar_sample_time, const Eigen::Matrix<float, 4, 1> &ego_states = Eigen::Matrix<float, 4, 1>::Zero());
    void init(std::shared_ptr<InputObject> obj, const Eigen::Matrix<float, 4, 1> &ego_states = Eigen::Matrix<float, 4, 1>::Zero());
};

/** 
 * OutputObject: Fused and environment-aware object. 
 * Contains Frenet coordinates for motion planning. 
 */
struct OutputObject : Object {
    uint32_t trackingID              = -1;

    std::shared_ptr<OutputObject> clone() const {
        auto cloned = std::make_shared<OutputObject>(*this);
        cloned->segment_mask = this->segment_mask.clone();

        return cloned;
    }
};

/** 
 * APIObject: Carla's API object for internal use, return all the objects include ego car not just surrounding. 
 * - Type: Defines object role (ego, adversary). 
 * - Speed (kph): Velocity in global space. 
 * - Global Location: Absolute position of the vehicle. 
 */
struct APIObject : Object {
    int vehicleID               = -1;
    std::string type            = "";
    float speed_kph             = 0.0f;
    Eigen::Vector3f speed_kph_  = Eigen::Vector3f::Zero();
    float centripetal_location  = 0.0f;  
    std::shared_ptr<APIObject> clone() const {
        return std::make_shared<APIObject>(*this);
    }
};

class CameraImageData {
public:
    uint64_t time_stamp = 0;
    cv::Mat image;
    int width  = 0;
    int height = 0;

    std::shared_ptr<CameraImageData> clone() const {
        std::shared_ptr<CameraImageData> cloned = 
            std::make_shared<CameraImageData>(*this);
        // Clone the image properly.
        cloned->image = image.clone();
        return cloned;
    }
};

struct RadarPoint
{
    float x;        // Horizontal displacement
    float y;        // Distance to our car
    float z;        // Height of cloud point
    int ClusterID;
    float velocity; // relative velocity
    float x_2d;
    float y_2d;

    RadarPoint (
        float _x = 0.0f,
        float _y = 0.0f,
        float _z = 0.0f,
        int _id = -1,
        float _velocity = 0.0f,
        float _x2d = 0.0f,
        float _y2d = 0.0f
    )
    : x(_x), y(_y), z(_z), ClusterID(_id), velocity(_velocity), x_2d(_x2d), y_2d(_y2d) {}
};

struct RadarPointCloud
{
    uint64_t time_stamp;
    std::vector<RadarPoint> radarVector;

    RadarPointCloud(
        uint64_t time_stamp_ = -1,
        std::vector<RadarPoint> radarVector_ = std::vector<RadarPoint>()
    )
    : time_stamp(time_stamp_), radarVector(radarVector_) {}
};

// Note you can add a template to utilize this class for both internal external detecion of camera and radar data. 
class SensorDataType {
public:
    uint64_t time_stamp = 0;
    
    std::vector<std::shared_ptr<InputObject>> objects;
    std::shared_ptr<SensorConfig> config;
    
    virtual ~SensorDataType() = default;
    virtual SensorType getSensorType() const = 0;

    // Clone method 
    virtual std::shared_ptr<SensorDataType> clone() const = 0;

    void converTo(std::shared_ptr<SensorConfig> targer_sensor = nullptr);
    void convertFrom(std::shared_ptr<SensorConfig> source_sensor = nullptr);
};

/*
 *Template class to handle specific sensor types
 */ 
template <SensorType T>
class SensorData : public SensorDataType {
public:
    SensorType sensor_type = SensorType::UNDEFINED;
    virtual ~SensorData() = default;
    SensorType getSensorType() override
    {
        return this->sensor_type;
    }

    std::shared_ptr<SensorDataType> clone() const override {
        std::shared_ptr<SensorData<T>> cloned = std::make_shared<SensorData<T>>(*this);
        
        for (std::shared_ptr<InputObject> object : cloned->objects)
        {
            object = object->clone();
        }
        return std::static_pointer_cast<SensorDataType>(cloned);
    }
};

template<>
class SensorData<SensorType::CAMERA> : public SensorDataType {
public:
    cv::Mat src_image;
    int src_length = 0;
    int src_width  = 0;
    
    //place holder for camera model if needed
    SensorType sensor_type = SensorType::CAMERA;

    SensorType getSensorType() const override
    {
        return this->sensor_type;
    }

    std::shared_ptr<SensorDataType> clone() const override {
        std::shared_ptr<SensorData<SensorType::CAMERA>> cloned = std::make_shared<SensorData<SensorType::CAMERA>>(*this);

        cloned->src_image = src_image.clone();

        for (std::shared_ptr<InputObject> object : cloned->objects)
        {
            object = object->clone();
        }
        return std::static_pointer_cast<SensorDataType>(cloned);
    }
};

template<>
class SensorData<SensorType::RADAR> : public SensorDataType {
public:
    SensorType sensor_type = SensorType::RADAR;
    std::shared_ptr<RadarPointCloud> radarClouds = nullptr;

    SensorType getSensorType() const override
    {
        return this->sensor_type;
    }

    std::shared_ptr<SensorDataType> clone() const override{
        std::shared_ptr<SensorData<SensorType::RADAR>> cloned = std::make_shared<SensorData<SensorType::RADAR>>(*this);
        
        if(radarClouds) {
            cloned->radarClouds = std::make_shared<RadarPointCloud>();
            for(RadarPoint point : radarClouds->radarVector)
            {
                cloned->radarClouds->radarVector.push_back(point);
            }
        }

        for (std::shared_ptr<InputObject> object : cloned->objects)
        {
            object = object->clone();
        }

        return std::static_pointer_cast<SensorDataType>(cloned);
    }
};

template<>
class SensorData<SensorType::API_SENSOR> {
public:

    uint64_t time_stamp = 0;
    uint vehicle_count;
    SensorType sensor_type = SensorType::API_SENSOR;
    std::vector<std::shared_ptr<APIObject>> objects;

    SensorData<SensorType::API_SENSOR>() = default;
    SensorType getSensorType()
    {
        return this->sensor_type;
    }

    std::shared_ptr<SensorData<SensorType::API_SENSOR>> clone() const {
        std::shared_ptr<SensorData<SensorType::API_SENSOR>> cloned = std::make_shared<SensorData<SensorType::API_SENSOR>>(*this);
        
        for (std::shared_ptr<APIObject> object : cloned->objects)
        {
            object = object->clone();
        }

        return cloned;
    }
};

struct PlanningConfigs
{
    int frame_width;
    int frame_height;
    int acc_following_enabled = false;
};

struct ControlConfigs
{
    bool custom = false;
    bool acc_following_enabled = false; // Set to true if some vehicle inside safe following range, ACC turn to following mode
    float acc_error_following = 0.0f;
};

// Define the struct: ImuParameters
struct Orientation
{
    float x;
    float y;
    float z;
    float w;
    Orientation(float x = 0, float y = 0, float z = 0, float w = 0)
    : x(x), y(y), z(z), w(w) {}
};

struct AngularVelocity
{
    float x;
    float y;
    float z;
    AngularVelocity(float x = 0, float y = 0, float z = 0)
    : x(x), y(y), z(z) {}
};

struct LinearVelocity
{
    float x;
    float y;
    float z;
    LinearVelocity(float x = 0, float y = 0, float z = 0)
    : x(x), y(y), z(z) {}
};

struct LinearAcceleration
{
    float x;
    float y;
    float z;
    LinearAcceleration(float x = 0, float y = 0, float z = 0)
    : x(x), y(y), z(z) {}
};

struct ImuParameters 
{
    uint64_t time_stamp;
    Orientation Imu_orientation;
    AngularVelocity Imu_angular_velocity;
    LinearAcceleration Imu_linear_acceleration;
};

struct OdometryParameters
{
    uint64_t time_stamp;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
};

struct GnssPoint 
{
    uint64_t time_stamp;
    float east;          
    float north;         
    float up;           
    GnssPoint() : time_stamp(0), east(0.0f), north(0.0f), up(0.0f) {}
};

struct SteeringParameters
{
    uint64_t time_stamp;
    float steering;

    SteeringParameters (
        float steering_ = 0,
        uint64_t time_stamp_ = 0
    ) : time_stamp(time_stamp_), steering(steering_)  {}
};

// ---------------------------- LANE DETECTION --------------------------------
/**
 * @brief Class that hold enum for lane marking ids
 */
class LaneMarkingID /* : public QObject */
{
    // Q_OBJECT

public:
    enum LaneMarkingIDEnum : int8_t {
        LEFT_MID = 0,       // Left lane marking of middle lane
        RIGHT_MID,          // Right lane marking of middle lane
        LEFT_SIDE,          // Left lane marking of left lane
        RIGHT_SIDE,         // Right lane marking of right lane
        NUM_LANE_MARKING    // Count number of lane marking in use
    };
    // Q_ENUM(LaneMarkingIDEnum)

    explicit LaneMarkingID(/* QObject *parent = nullptr */) /* : QObject(parent) */ {}
};

/**
 * @brief
 * Class that hold enum for lane marking types.
 * Also a method to decode the type to attribute for drawing.
 */
class LaneMarkingType /* : public QObject */
{
    // Q_OBJECT

public:
    enum LaneMarkingTypeEnum : int8_t{
        NOT_EXIST = 0,
        OTHERS,
        BROKEN,
        SOLID,
        BROKEN_BROKEN,
        BROKEN_SOLID,
        SOLID_BROKEN,
        SOLID_SOLID,
        CURB
    };
    // Q_ENUM(LaneMarkingTypeEnum)

    explicit LaneMarkingType(/* QObject *parent = nullptr */) /* : QObject(parent) */ {}
    static int decode(int type, bool &double_line, bool line_types[2]);
};

/**
 * @brief This struct hold raw points and type output from lane detector
 */
struct LanePointsRaw{
    short type = LaneMarkingType::NOT_EXIST;
    std::vector<cv::Point2f> points;

    void clear()
    {
        type = LaneMarkingType::NOT_EXIST;
        points.clear();
    }
};

struct LaneMarking{
    short type = 0;                 // Lane marking type
    float startY = 0.0f,            // The y value where the lane start and end
          endY = 0.0f;
    std::vector<float> xCoeffs;     // Coeffs of the polynomial
    //                   zCoeffs;

    void clear()
    {
        type = 0;
        xCoeffs.clear();
        // zCoeffs.clear();
    }
    float x(float y) const;
    // float z(float y) const;
    // void xz(float y, float &x, float &z) const;
};

// laneResults
struct LaneDetection{
    uint64_t time_stamp = 0;
    short laneLinesCount = 4;
    float averageLanePosition = 400;

    // Lane points output from model
    std::vector<LanePointsRaw> laneMarkingsPoint;

    // Lane markings represented by polynomial function
    std::vector<LaneMarking> laneMarkingsWorld;
};


// ------------------------------ PERCEPTION ----------------------------------
/**
 * @brief Class that hold enum for different lane status
 */
class LaneStatus /* : public QObject */
{
    // Q_OBJECT

public:
    enum LaneStatusEnum : int8_t {
        MISSING_LANE = 0,  // Detect no lane
        NOT_IN_LANE = 1,   // Detected lane but vehicle is not in lane
        IN_LANE = 2,       // Detected lane and vehicle is in lane
        NUM_LANE_STATUS    // Count number of available lane status
    };
    // Q_ENUM(LaneStatusEnum)

    explicit LaneStatus(/* QObject *parent = nullptr */) /* : QObject(parent) */ {}
};

struct PerceptionOutputObject
{
    uint64_t time_stamp = 0;
    std::vector<std::shared_ptr<OutputObject>> objects;
    OdometryParameters my_car_speed;
    ImuParameters my_car_state;
    GnssPoint my_car_location;
    SteeringParameters steering_angle;

    bool is_reverse = false;

    int src_image_width = 0; 
    int src_image_height = 0; 
    float closest_car_distance = 0.0f;
    cv::Mat src_image; 
    float my_car_width;

    void init()
    {
        src_image_width = 0;
        src_image_height = 0;
        objects.clear();
    }
};
struct PerceptionResults
{
    // General information.
    uint64_t time_stamp = 0;
    int src_image_width = 0; 
    int src_image_height = 0; 
    float closest_car_distance = -1.0f;
    float closest_car_speed = 0.0f;
    cv::Mat src_image; 
    float my_car_speed;
    float my_car_width;
    ImuParameters my_car_state; 
    
    // Calculations results.
    std::vector<FusionObject> objects;
    std::vector<AdvanceFusionObject> adv_objs;
    std::vector<std::map<SensorType, bool>> src_objs;
    std::vector<RadarPoint> radar_object;
    std::unordered_map<size_t, size_t> map_obj_radar;
    std::vector<float> kalman_bbox;
    int lane_status = LaneStatus::MISSING_LANE;

    // Lane detection result
    LaneDetection laneResult;

    void init()
    {
        src_image_width = 0;
        src_image_height = 0;
        lane_status = LaneStatus::MISSING_LANE;
        objects.clear();
    }
};


// ------------------------------- PLANNING -----------------------------------
// TODO: Might need to expand this to include the old result which will be drawn on top of .
struct PlanningResults{
    uint64_t time_stamp = 0;
    std::vector<std::array<float, 3>> points;
    bool is_danger_aeb = false;
    bool is_warning_aeb = false;

    void init()
    {
        points.clear();
        is_danger_aeb = false;
        is_warning_aeb = false;
    }
};

struct ControlResults
{
    uint64_t time_stamp = 0;
    float steering_value = 0.0f;
    float throttle_value = 0.0f;
    float brake_value = 0.0f;

    void init()
    {
        steering_value = 0.0f;
        throttle_value = 0.0f;
        brake_value = 0.0f;
    }

    /**
     * @brief Limit the control values within boundary.
     */
    void limit()
    {
        steering_value = clamp(steering_value, -1.0f, 1.0f);
        throttle_value = clamp(throttle_value, 0.0f, 1.0f);
        brake_value = clamp(brake_value, 0.0f, 1.0f);
    }
};


// ------------------------------- PIPELINE -----------------------------------

struct PipelineResults
{
    uint64_t time_stamp = 0;
    PerceptionResults perception_results;
    PerceptionOutputObject perception_output;
    LaneDetection lane_detection;
    PlanningResults planning_results;
    ControlResults control_results;

    void init()
    {
        perception_results.init();
        planning_results.init();
        control_results.init();
    }
};


#endif // INTERMEDIATE_REPRESENTATION_H