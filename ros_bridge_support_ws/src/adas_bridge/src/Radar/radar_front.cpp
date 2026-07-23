// ROS2 header
#include <rclcpp/rclcpp.hpp>
// Eigen header
#include <Eigen/Dense>
// PCL header
#include <pcl/common/centroid.h>
#include <random>
// System header
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <limits>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <queue>

// User header
#include "radar_front.hpp"
#include "utilities.h"
#include "dbscan.h"
#include "logger/general_logger.h"
#include "adas_service_if.h"

// Structure to hold data for async processing
struct RadarFrameData {
    std::vector<RadarObject> objects;
    std::vector<std::vector<Point>> segmented_clusters;
    int64_t timestamp_ns;
    int visualize_max_y;
    int visualize_max_x;
    bool should_visualize;
    bool should_log;
};

void packRadarMsg(
    ipc_helper::msg::Radar &radar_msg, 
    const int64_t &num_clouds, 
    const std::vector<RadarObject> &radar_clouds)
{
    radar_msg.num_clouds = num_clouds;
    radar_msg.radar_clouds.resize(num_clouds);
    for (int64_t i = 0; i < num_clouds; ++i) {
        radar_msg.radar_clouds[i].location.x = radar_clouds[i].x;
        radar_msg.radar_clouds[i].location.y = radar_clouds[i].y;
        radar_msg.radar_clouds[i].location.z = radar_clouds[i].z;
        radar_msg.radar_clouds[i].velocity.x = radar_clouds[i].v_x;
        radar_msg.radar_clouds[i].velocity.y = radar_clouds[i].v_y;
        radar_msg.radar_clouds[i].velocity.z = radar_clouds[i].v_z;
        radar_msg.radar_clouds[i].min_point.x = radar_clouds[i].min_point.x;
        radar_msg.radar_clouds[i].min_point.y = radar_clouds[i].min_point.y;
        radar_msg.radar_clouds[i].min_point.z = radar_clouds[i].min_point.z;
        radar_msg.radar_clouds[i].max_point.x = radar_clouds[i].max_point.x;
        radar_msg.radar_clouds[i].max_point.y = radar_clouds[i].max_point.y;
        radar_msg.radar_clouds[i].max_point.z = radar_clouds[i].max_point.z;
        radar_msg.radar_clouds[i].closest_point.x = radar_clouds[i].closest_point.x;
        radar_msg.radar_clouds[i].closest_point.y = radar_clouds[i].closest_point.y;
        radar_msg.radar_clouds[i].closest_point.z = radar_clouds[i].closest_point.z;
    }
}

/**
 * @brief Log final radar objects with per-point velocity
 * Format: x y z closest_x closest_y closest_z point_velocity
 * Total: 7 values per point
 */
void logRadarData(const std::vector<RadarObject>& objects, int64_t timestamp_ns)
{
    std::lock_guard<std::mutex> lock(g_ui_state.log_mutex);
    
    if (!g_ui_state.is_logging || !g_ui_state.log_file.is_open())
        return;
    
    // Write frame header
    g_ui_state.log_file << g_ui_state.frame_counter << "=========================[" 
                        << timestamp_ns << "]\n";
    
    // Write each cluster
    for (size_t m = 0; m < objects.size(); ++m) {
        const auto& obj = objects[m];
        
        // Apply logging filter if enabled - check if ANY point in cluster satisfies filter
        bool should_log_cluster = true;
        if (g_ui_state.use_logging_filter) {
            should_log_cluster = false;
            // Check if at least one point in cluster is within filter range
            for (const auto& pt : obj.point_cloud->points) {
                if (pt.x >= g_ui_state.log_x_min && pt.x <= g_ui_state.log_x_max &&
                    pt.y >= g_ui_state.log_y_min && pt.y <= g_ui_state.log_y_max &&
                    pt.z >= g_ui_state.log_z_min && pt.z <= g_ui_state.log_z_max) {
                    should_log_cluster = true;  // At least one point in range
                    break;
                }
            }
        }
        
        // If cluster should be logged, log ALL points in the cluster
        if (should_log_cluster) {
            g_ui_state.log_file << m << "____________________________\n";
            
            // FORMAT: x y z closest_x closest_y closest_z point_velocity
            for (const auto& pt : obj.point_cloud->points) {
                g_ui_state.log_file << std::fixed << std::setprecision(6)
                                   << pt.x << " " << pt.y << " " << pt.z << " "
                                   << obj.closest_point.x << " " 
                                   << obj.closest_point.y << " " 
                                   << obj.closest_point.z << " "
                                   << pt.velocity << "\n";
            }
        }
    }
    
    g_ui_state.frame_counter++;
}

/**
 * @brief Log segmented clusters with cluster ID and per-point velocity
 * Format: x y z closest_x closest_y closest_z point_velocity cluster_id
 * Total: 8 values per point
 */
void logSegmentedClusters(const std::vector<std::vector<Point>>& segmented_clusters, int64_t timestamp_ns)
{
    std::lock_guard<std::mutex> lock(g_ui_state.log_mutex);
    
    if (!g_ui_state.is_logging || !g_ui_state.log_file.is_open())
        return;
    
    // Write frame header
    g_ui_state.log_file << g_ui_state.frame_counter << "=========================[" 
                        << timestamp_ns << "]\n";
    
    // Write each cluster
    for (size_t i = 0; i < segmented_clusters.size(); ++i) {
        const auto& cluster = segmented_clusters[i];
        
        // Skip empty clusters or slope clusters (ID = -99)
        if (cluster.empty() || cluster[0].clusterID == -99)
            continue;
        
        int cluster_id = cluster[0].clusterID;
        
        // Apply logging filter if enabled - check if ANY point in cluster satisfies filter
        bool should_log_cluster = true;
        if (g_ui_state.use_logging_filter) {
            should_log_cluster = false;
            for (const auto& pt : cluster) {
                if (pt.x >= g_ui_state.log_x_min && pt.x <= g_ui_state.log_x_max &&
                    pt.y >= g_ui_state.log_y_min && pt.y <= g_ui_state.log_y_max &&
                    pt.z >= g_ui_state.log_z_min && pt.z <= g_ui_state.log_z_max) {
                    should_log_cluster = true;
                    break;
                }
            }
        }
        
        // If cluster should be logged, log ALL points in the cluster
        if (should_log_cluster) {
            g_ui_state.log_file << cluster_id << "____________________________\n";
            
            // Calculate closest point for reference
            float min_distance = std::numeric_limits<float>::max();
            Point closest_pt = cluster[0];
            
            for (const auto& pt : cluster) {
                float dist = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
                if (dist < min_distance) {
                    min_distance = dist;
                    closest_pt = pt;
                }
            }
            
            // FORMAT: x y z closest_x closest_y closest_z point_velocity cluster_id
            for (const auto& pt : cluster) {
                g_ui_state.log_file << std::fixed << std::setprecision(6)
                                   << pt.x << " " << pt.y << " " << pt.z << " "
                                   << closest_pt.x << " " 
                                   << closest_pt.y << " " 
                                   << closest_pt.z << " "
                                   << pt.velocity << " "      // Per-point velocity
                                   << cluster_id << "\n";     // Cluster ID at the end
            }
        }
    }
    
    g_ui_state.frame_counter++;
}

std::string getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}

void startLogging()
{
    std::lock_guard<std::mutex> lock(g_ui_state.log_mutex);
    
    if (g_ui_state.is_logging)
        return;
    
    std::string filename = "raw_radar_data_" + getCurrentTimestamp() + ".txt";
    g_ui_state.log_file.open(filename, std::ios::out);
    
    if (g_ui_state.log_file.is_open()) {
        g_ui_state.is_logging = true;
        g_ui_state.frame_counter = 0;
        RCLCPP_INFO(rclcpp::get_logger("radar_logging"), 
                    "Started logging to: %s", filename.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("radar_logging"), 
                     "Failed to create log file: %s", filename.c_str());
    }
}

void stopLogging()
{
    std::lock_guard<std::mutex> lock(g_ui_state.log_mutex);
    
    if (!g_ui_state.is_logging)
        return;
    
    if (g_ui_state.log_file.is_open()) {
        g_ui_state.log_file.close();
        g_ui_state.is_logging = false;
        RCLCPP_INFO(rclcpp::get_logger("radar_logging"), 
                    "Stopped logging. Total frames: %d", g_ui_state.frame_counter);
    }
}

// === Node Class ===
class RadarFrontNode : public rclcpp::Node
{
public:
    RadarFrontNode() : Node("adas_bridge_radar_front_node"), ds(MINIMUM_POINTS, EPSILON)
    {
        this->declare_parameter("vehicle_name", "hero0");
        this->declare_parameter("visualize_radar", false);
        this->declare_parameter("visualize_max_y", 260);
        this->declare_parameter("visualize_max_x", 50);

        // Get parameter value
        std::string vehicle_name = this->get_parameter("vehicle_name").as_string();
        visualize_radar_ = this->get_parameter("visualize_radar").as_bool();
        visualize_max_y_ = this->get_parameter("visualize_max_y").as_int();
        visualize_max_x_ = this->get_parameter("visualize_max_x").as_int();

        // Validate parameters - use defaults if invalid
        if (visualize_max_y_ <= 0 || visualize_max_y_ > 1000) {
            RCLCPP_WARN(this->get_logger(), 
                        "Invalid visualize_max_y=%d, using default=260", visualize_max_y_);
            visualize_max_y_ = 260;
        }

        if (visualize_max_x_ <= 0 || visualize_max_x_ > 500) {
            RCLCPP_WARN(this->get_logger(), 
                        "Invalid visualize_max_x=%d, using default=50", visualize_max_x_);
            visualize_max_x_ = 50;
        }

        if (visualize_radar_) 
        {
            RCLCPP_INFO(this->get_logger(), "Radar visualization ENABLED");
            RCLCPP_INFO(this->get_logger(), "Visualization range: Y=%.dm, X=+/-%.dm", 
                       visualize_max_y_, visualize_max_x_);
        } 
        else 
            RCLCPP_INFO(this->get_logger(), "Radar visualization DISABLED");

        // Initialize UI state
        g_ui_state.min_points_text = std::to_string(MINIMUM_POINTS);
        g_ui_state.epsilon_text = std::to_string(EPSILON);
        g_ui_state.temp_min_points = MINIMUM_POINTS;
        g_ui_state.temp_epsilon = EPSILON;
        
        g_ui_state.cutoff_distance_text = std::to_string(static_cast<int>(CUTOFF_DISTANCE));
        g_ui_state.z_min_text = "0.1";
        g_ui_state.z_max_text = "2.5";
        g_ui_state.temp_cutoff_distance = CUTOFF_DISTANCE;
        g_ui_state.temp_z_min = -0.1f;
        g_ui_state.temp_z_max = 2.5f;
        
        g_ui_state.log_x_min_text = "-1000";
        g_ui_state.log_x_max_text = "1000";
        g_ui_state.log_y_min_text = "-1000";
        g_ui_state.log_y_max_text = "1000";
        g_ui_state.log_z_min_text = "-1000";
        g_ui_state.log_z_max_text = "1000";

        // Initialzie API for ADAS Service Interface
        adas_service_if_ = std::make_unique<AdasServiceIF>(vehicle_name, "");
        adas_service_if_->registerTopic<TopicRadarFrontHandler>(AdasServiceIF::TopicNameT::RADAR);
        adas_publisher_ = this->create_publisher<ipc_helper::msg::Radar>(
            adas_service_if_->getTopicName(AdasServiceIF::TopicNameT::RADAR),
            adas_service_if_->getQoS(AdasServiceIF::TopicNameT::RADAR)
        );

        std::string topic = "/carla/" + vehicle_name + "/radar_front";
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 1, 
            std::bind(&RadarFrontNode::radarFrontCallback, this, std::placeholders::_1)
        );

        // Only start async processing thread if visualization is enabled
        // (Logging menu is only available when visualization window is shown)
        if (visualize_radar_) {
            async_running_ = true;
            async_thread_ = std::thread(&RadarFrontNode::asyncProcessingThread, this);
            RCLCPP_INFO(this->get_logger(), "Started async visualization/logging thread");
        } else {
            async_running_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "Radar front node initialized successfully");
    }

    ~RadarFrontNode()
    {
        // Stop async thread if running
        if (async_running_) {
            async_running_ = false;
            async_cv_.notify_all();
            if (async_thread_.joinable()) {
                async_thread_.join();
                RCLCPP_INFO(this->get_logger(), "Stopped async visualization/logging thread");
            }
        }

        if (g_ui_state.is_logging) {
            stopLogging();
        }
    }

private:
    bool visualize_radar_{false};
    int visualize_max_y_{260};
    int visualize_max_x_{50};
    std::unique_ptr<AdasServiceIF> adas_service_if_;
    // Subscriber
    rclcpp::Publisher<ipc_helper::msg::Radar>::SharedPtr adas_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    // DBSCAN instance
    DBSCAN ds;
    
    // Async processing members
    std::thread async_thread_;
    std::atomic<bool> async_running_;
    std::queue<RadarFrameData> async_queue_;
    std::mutex async_mutex_;
    std::condition_variable async_cv_;
    const size_t MAX_QUEUE_SIZE = 3;  // Drop frames if queue is full
    
    void asyncProcessingThread()
    {
        while (async_running_) {
            std::unique_lock<std::mutex> lock(async_mutex_);
            
            // Wait for data or shutdown signal
            async_cv_.wait(lock, [this] { 
                return !async_queue_.empty() || !async_running_; 
            });
            
            if (!async_running_ && async_queue_.empty()) {
                break;
            }
            
            if (!async_queue_.empty()) {
                // Get frame data
                RadarFrameData frame_data = std::move(async_queue_.front());
                async_queue_.pop();
                lock.unlock();
                
                // Check mode and visualize accordingly
                if (frame_data.should_visualize) {
                    if (g_ui_state.use_segmented_clusters_mode) {
                        // Visualize segmented clusters (no velocity)
                        visualizeSegmentedClusters(
                            frame_data.segmented_clusters,
                            frame_data.visualize_max_y,
                            frame_data.visualize_max_x,
                            true,  // show_cluster_ids
                            true,  // show_grid
                            true,  // show_info
                            "4D Radar Top-Down View"
                        );
                    } else {
                        // Visualize final objects (with velocity)
                        visualizeRadarObjects2D(
                            frame_data.objects,
                            frame_data.visualize_max_y,
                            frame_data.visualize_max_x,
                            true,  // show_velocity
                            true,  // show_ids
                            true,  // show_grid
                            true,  // show_info
                            "4D Radar Top-Down View"
                        );
                    }
                }
                
                // Logging based on mode
                if (frame_data.should_log) {
                    if (g_ui_state.use_segmented_clusters_mode) {
                        logSegmentedClusters(frame_data.segmented_clusters, frame_data.timestamp_ns);
                    } else {
                        logRadarData(frame_data.objects, frame_data.timestamp_ns);
                    }
                }
            }
        }
    }
    
    void radarFrontCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Update DBSCAN parameters if changed
        ds.updateParameters(g_ui_state.temp_min_points, g_ui_state.temp_epsilon);
        
        // Perform DBSCAN clustering with filter parameters
        std::vector<Point> dbscan_points_vector = extract_points_from_cloud(
            msg, 
            g_ui_state.temp_cutoff_distance,
            g_ui_state.temp_z_min,
            g_ui_state.temp_z_max
        );
        ds.run(dbscan_points_vector);
        std::vector<std::vector<Point>> segmented_clusters = segment_clusters_by_id(ds.m_points, ds.detected_clusters);
        std::vector<int> near_origin_cluster_ids = find_near_origin_cluster_ids(segmented_clusters);
        mark_slope_clusters(segmented_clusters, near_origin_cluster_ids);

        // Filter road contamination from near-origin clusters
        filterNearOriginClusters(segmented_clusters, near_origin_cluster_ids);

        // Remove all unused clusters
        std::vector<pcl::PointCloud<PointXYZV>::Ptr> point_cloud_list;
        for (size_t i = 1; i < segmented_clusters.size(); ++i) {
            pcl::PointCloud<PointXYZV>::Ptr cloud(new pcl::PointCloud<PointXYZV>());

            if (segmented_clusters[i].size() == 0 || segmented_clusters[i][0].clusterID == -99)
                continue;

            for (const Point &point : segmented_clusters[i]) {
                PointXYZV pcl_point;
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;
                pcl_point.azimuth = point.azimuth;
                pcl_point.elevate = point.elevate;
                pcl_point.velocity = point.velocity;

                cloud->points.push_back(pcl_point);
            }

            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = true;

            point_cloud_list.push_back(cloud);
        }

        // Process point cloud
        std::vector<RadarObject> objects;
        for(size_t i = 0; i < point_cloud_list.size(); i++)
        {
            pcl::PointCloud<PointXYZV>::Ptr cloud = point_cloud_list[i];
            RadarObject radar_object;
            
            if(cloud->points.size() < static_cast<size_t>(g_ui_state.temp_min_points)) continue;

            radar_object.point_cloud = cloud;

            Eigen::Vector3f V_i = extract_velocity(cloud);
            radar_object.v_x = V_i(1);
            radar_object.v_y = V_i(0);
            radar_object.v_z = V_i(2);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid<PointXYZV>(*cloud, centroid);
            radar_object.x = centroid[0];
            radar_object.y = centroid[1];
            radar_object.z = centroid[2];
            
            // Calculate bounding box
            pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
            feature_extractor.setInputCloud(convertToXYZ(cloud));
            feature_extractor.compute();
            pcl::PointXYZ min_point, max_point;
            feature_extractor.getAABB(min_point, max_point);
            // Fill bounding box to point cloud
            radar_object.min_point.x = min_point.x;
            radar_object.min_point.y = min_point.y;
            radar_object.min_point.z = min_point.z;
            radar_object.max_point.x = max_point.x;
            radar_object.max_point.y = max_point.y;
            radar_object.max_point.z = max_point.z;

            float min_distance = std::numeric_limits<float>::max(); 
            for (size_t j = 0; j < cloud->points.size(); ++j)
            {
                Eigen::Vector3f point_position(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
                float distance = point_position.norm();
                if (distance < min_distance)
                {
                    min_distance = distance;
                    radar_object.closest_point.x = cloud->points[j].x;
                    radar_object.closest_point.y = cloud->points[j].y;
                    radar_object.closest_point.z = cloud->points[j].z;
                }
            }
            objects.push_back(radar_object);
        }

        // Push to async processing thread if visualization is enabled
        // (Logging is only available through the visualization UI)
        if (visualize_radar_) {
            std::lock_guard<std::mutex> lock(async_mutex_);
            
            // Drop frame if queue is full (prevent memory buildup)
            if (async_queue_.size() >= MAX_QUEUE_SIZE) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Async queue full, dropping frame");
            } else {
                RadarFrameData frame_data;
                frame_data.objects = objects;  // Copy data
                frame_data.segmented_clusters = segmented_clusters;
                frame_data.timestamp_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
                frame_data.visualize_max_y = visualize_max_y_;
                frame_data.visualize_max_x = visualize_max_x_;
                frame_data.should_visualize = true;  // Always visualize when this block runs
                frame_data.should_log = g_ui_state.is_logging;
                
                async_queue_.push(std::move(frame_data));
                async_cv_.notify_one();
            }
        }

        // Pack and send message to ADAS (this must be fast, no blocking operations)
        ipc_helper::msg::Radar radar_msg;
        radar_msg.header.stamp = msg->header.stamp;
        packRadarMsg(radar_msg, objects.size(), objects);
        adas_publisher_->publish(radar_msg);

        RCLCPP_DEBUG(this->get_logger(), "num_clouds: %ld", radar_msg.num_clouds);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    std::signal(SIGINT, [](int) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        if (g_ui_state.is_logging) {
            stopLogging();
        }
        rclcpp::shutdown();
    });

    auto radar_node = std::make_shared<RadarFrontNode>();
    rclcpp::spin(radar_node);

    rclcpp::shutdown();
    return 0;
}