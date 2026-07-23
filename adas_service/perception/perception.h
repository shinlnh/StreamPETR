#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <string>
#include <limits>
#include <deque>

// #include "traffic_object.h"
#include "streampetr.hpp"
// #include "object_detector.h"
#include "lane_detector.h"
#include "cameramodel.h"
#include "sensor/config.h"
#include "intermediate_representations.h"
#include "tracker.hpp"
#include "engine.h"
#include "ctpl_stl.h"
#include "logger/general_logger.h"
#include "common.h"

#define POLYNOMIAL_DEGREE 3
#define THRESHOLD_INSIDE_LANE 0.8
#define SYNC_FRAMES 5

class Perception
{
// ATTRIBUTES
private:
    // Environment status
    float pitch = 0.0f;

    // Object detection model
    std::shared_ptr<StreamPETR> objectDetector;
    std::vector<std::shared_ptr<InputObject>> objects;

    // std::shared_ptr<ObjectDetector> objectDetector = std::make_shared<ObjectDetector>(); 
    // std::mutex objectDetectionMutex;
    
    // Lane detection model
    std::shared_ptr<LaneDetector> laneDetector = std::make_shared<LaneDetector>();
    std::mutex laneDetectorMutex;
    std::vector<LaneMarking> laneMarkingWorld;  // TEMPORARY
    int frameHasLaneCounter = 0;                // Count number of frame has middle lane
    int inLaneCounter = 0;
    int laneStatus = 0;                         // System status related to lane

    // Restore velocity X value to apply Average filter
    std::unordered_map<uint32_t, std::deque<float>> object_AccelerationX_History;

    // Window size of filter based on range of velocity
    static constexpr size_t filter_window_size_high = 5;
    static constexpr size_t filter_window_size_low = 30;
    static constexpr float threshold_filter_velocityX = 6.0;
    
    // LPF and Median filter history
    std::unordered_map<uint32_t, float> object_VelocityX_OldValue_LPF;
    std::unordered_map<uint32_t, std::deque<float>> object_MedianFilter_Buffer;
    
    void smoothVelocity(uint32_t fusionID, float & new_velocity, int choose_threshold);
    void smoothAcceleration(uint32_t fusionID, float & new_accleretion, int choose_threshold);
    float median_filter(uint32_t fusionID, float x);
    // Camera model for estimate distance
    std::shared_ptr<CameraModel> camModel = std::make_shared<CameraModel>();

    std::shared_ptr<TrackedSensorData> FusionResult = std::make_shared<TrackedSensorData>();
    /**
     * @brief Frenet environment objects. Units: meters. Theses objects are very compilcated and 
     * these representations are not as clear as possible because of time constraints. A Frenet object
     * is represented by a vector of 5 elements {x, y , s, d, index}
     * x and y are differential values in discrete steps taken on the real world lane curves
     * s and d represents a distance along the path and the displacement to it respectively.
     * index is only used on the right lane to search for the corresponding point on the left lane
     */
    std::vector<std::vector<float>> mapPointsFrenet, rightPointsFrenet; /* Representation of the 2 lanes in frenet space. Maps mean left here. Unit: meters*/
    std::vector<std::vector<std::vector<float>>> objectsPointsFrenet;   /* Representation of objects. Unit: meters*/

    // Carla API results for reference, should be passed in from ADAS main loop.
    std::vector<std::shared_ptr<SensorDataType>> api_sensor_queue;

    Tracker tracker;
    std::mutex perceptionInternalMutex;

    Eigen::Matrix<float, 4, 1> pre_ego_states = Eigen::Matrix<float, 4, 1>::Zero();

    // Logger
    static LogUtility log;

public:
    // Thread Pooling
    ctpl::thread_pool sensor_tracking_thread_pool;
    ctpl::thread_pool dist_est_thread_pool;
    
    std::vector<std::shared_ptr<SensorDataType>> api_data_vector;
    
// METHODS
public:
    Perception();
    std::shared_ptr<TrackedSensorData> Tracking(
        std::shared_ptr<SensorDataType> current_sensor_data,
        std::shared_ptr<const SteeringParameters> steeringAngleQueue,
        std::shared_ptr<const ImuParameters> imuQueue,
        std::shared_ptr<const GnssPoint> gnssQueue,
        std::shared_ptr<const OdometryParameters> odometerQueue,
        const bool is_reverse
    );

    PerceptionOutputObject execute(
        std::queue<std::shared_ptr<TrackedSensorData>>& current_data_queue
    );

    std::string debugFusionObjects(const std::vector<FusionObject>& fusion_objects);

    // Distance estimation
    void setPitch(float pitch) { this->pitch = pitch; }
    float getPitch() { return this->pitch; }
    float estimateXDistance(int x, int y)
    {
        return camModel->getLateralDistanceToCar(x, y, this->pitch);
    }
    float estimateYDistance(int x, int y)
    {
        return camModel->getLongitudinalDistanceToCar(x, y, this->pitch);
    }
    void convertPointsToWorld(std::vector<cv::Point2f> const &points,
                              std::vector<cv::Point2f> &pointsWorld)
    {
        pointsWorld = camModel->getDistanceVectorToCar(points, this->pitch);
    }
    float getCarWidth(void) {return camModel->getCarWidth();}

    // Object detection methods
    cv::Mat objectDetectionPipeline(
        uint64_t timestamp,
        cv::Mat img,
        std::shared_ptr<const OdometryParameters> odo
    );
    Eigen::Matrix4dRM getEgoPoseTransform(
        std::shared_ptr<const OdometryParameters> odo
    );
    void draw3DBoundingBoxes(
        cv::Mat& image,
        const StreamPETR::Output &output_data,
        const Eigen::Matrix4fRM &ego2img
    );
    std::vector<std::shared_ptr<InputObject>> getDetectedObjects(void) {
        return objects;
    }

    // Lane detection methods
    void resetLaneDetection(void) { laneDetector->reset(); }
    void laneDetectorPipeline(const cv::Mat &img);
    void retrieveLaneResult(std::shared_ptr<LaneDetection> lane_result);
    int updateLaneStatus(const std::shared_ptr<LaneDetection> lane_result);
    int getLaneStatus() { return this->laneStatus; }
    void fitPolyToPoints(std::vector<cv::Point2f> const &points,
                         int const degree,
                         std::vector<float> &coeffs);
    void getRealWorldLanes(const std::shared_ptr<LaneDetection> lane_result);

    // Others helper functions
    void convertRadarBboxTo2D(std::shared_ptr<InputObject> detection, 
                              std::shared_ptr<SensorConfig> radar_config,
                              std::shared_ptr<SensorConfig> camera_config);
};

#endif // PERCEPTION_H



