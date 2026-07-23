#ifndef UTILS_H
#define UTILS_H

#include "dbscan.h"
#include "radar_front.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <mutex>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Velocity epsilon for sequential clustering (m/s)
// Points with velocity difference <= VELOCITY_EPS belong to same subcluster
#define VELOCITY_EPS 2.0f

// UI State structure
struct UIState {
    bool show_menu = false;
    bool is_logging = false;
    std::ofstream log_file;
    int frame_counter = 0;
    
    bool use_segmented_clusters_mode = false;  // false = objects (default), true = segmented_clusters
    
    // DBSCAN parameters
    int temp_min_points = MINIMUM_POINTS;
    double temp_epsilon = EPSILON;
    std::string min_points_text;
    std::string epsilon_text;
    
    // Point cloud filter parameters
    float temp_cutoff_distance = CUTOFF_DISTANCE;
    float temp_z_min = -0.2f;
    float temp_z_max = 2.5f;
    std::string cutoff_distance_text;
    std::string z_min_text;
    std::string z_max_text;
    
    // Logging filter parameters
    bool use_logging_filter = false;
    float log_x_min = -1000.0f;
    float log_x_max = 1000.0f;
    float log_y_min = -1000.0f;
    float log_y_max = 1000.0f;
    float log_z_min = -1000.0f;
    float log_z_max = 1000.0f;
    std::string log_x_min_text;
    std::string log_x_max_text;
    std::string log_y_min_text;
    std::string log_y_max_text;
    std::string log_z_min_text;
    std::string log_z_max_text;
    
    enum TextBoxID { 
        NONE = 0, 
        MIN_POINTS_BOX, 
        EPSILON_BOX,
        CUTOFF_DISTANCE_BOX,
        Z_MIN_BOX,
        Z_MAX_BOX,
        LOG_X_MIN_BOX,
        LOG_X_MAX_BOX,
        LOG_Y_MIN_BOX,
        LOG_Y_MAX_BOX,
        LOG_Z_MIN_BOX,
        LOG_Z_MAX_BOX
    };
    TextBoxID active_textbox = NONE;
    
    std::mutex log_mutex;
};

// Global UI state declaration
extern UIState g_ui_state;

//> Utilities function to seperate the cluster from the surrounding road
Eigen::VectorXd quadratic_regression(const std::vector<Point>& points);
bool is_slope_cluster(const Eigen::VectorXd& coeffs, double slope_threshold = 9.0);
std::vector<Point> extract_points_from_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
                                              float cutoff_distance, 
                                              float z_min, 
                                              float z_max);
std::vector<std::vector<Point>> segment_clusters_by_id(const std::vector<Point>& points, int detected_clusters);
std::vector<int> find_near_origin_cluster_ids(const std::vector<std::vector<Point>>& segmented_clusters);
void mark_slope_clusters(std::vector<std::vector<Point>>& segmented_clusters, const std::vector<int>& near_origin_cluster_ids);
pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZ(pcl::PointCloud<PointXYZV>::Ptr cloud);

//> Velocity extraction 
Eigen::Vector3f extract_velocity(const pcl::PointCloud<PointXYZV>::Ptr &cloud);

//> Sequential velocity-based filtering for near-origin clusters
/**
 * @brief Filter road points from vehicle cluster using sequential velocity clustering
 * 
 * Algorithm:
 * 1. Point 1 -> subcluster 1
 * 2. For each next point:
 *    - If |velocity - any_velocity_in_subcluster1| <= VELOCITY_EPS -> add to subcluster 1
 *    - Else -> add to subcluster 2
 * 3. Compare mean_z of two subclusters
 * 4. Remove subcluster with lower z (road), keep higher z (vehicle)
 * 
 * @param cluster Input cluster to filter
 * @param filtered_cluster Output vehicle points only
 * @return true if filtering applied, false if cluster is homogeneous
 */
bool filterRoadFromVehicleCluster(const std::vector<Point>& cluster, 
                                   std::vector<Point>& filtered_cluster);

/**
 * @brief Filter road contamination from near-origin clusters only
 * Must be called AFTER mark_slope_clusters() in pipeline
 * 
 * @param segmented_clusters Clusters to process (input/output)
 * @param near_origin_cluster_ids IDs of clusters near vehicle origin
 * @return Number of clusters filtered
 */
void filterNearOriginClusters(std::vector<std::vector<Point>>& segmented_clusters,
                              const std::vector<int>& near_origin_cluster_ids);

//> Visualization - for final objects (with velocity)
void visualizeRadarObjects2D(
    const std::vector<RadarObject>& objects,
    int max_range_y = 260,
    int max_range_x = 50,
    bool show_velocity = true,
    bool show_ids = true,
    bool show_grid = true,
    bool show_info = true,
    const std::string& window_name = "4D Radar Top-Down View"
);

// Visualization - for segmented clusters (without velocity)
void visualizeSegmentedClusters(
    const std::vector<std::vector<Point>>& segmented_clusters,
    int max_range_y = 260,
    int max_range_x = 50,
    bool show_cluster_ids = true,
    bool show_grid = true,
    bool show_info = true,
    const std::string& window_name = "4D Radar Top-Down View"
);

//> UI Elements
void drawMenuPanel(cv::Mat& img, int img_width, int img_height);
void onMouseEnhanced(int event, int x, int y, int flags, void* userdata);
void handleKeyboardInput(int key);

//> Logging functions (declared here, implemented in radar_front.cpp)
void logRadarData(const std::vector<RadarObject>& objects, int64_t timestamp_ns);
void logSegmentedClusters(const std::vector<std::vector<Point>>& segmented_clusters, int64_t timestamp_ns);
std::string getCurrentTimestamp();
void startLogging();
void stopLogging();

#endif //UTILS_H