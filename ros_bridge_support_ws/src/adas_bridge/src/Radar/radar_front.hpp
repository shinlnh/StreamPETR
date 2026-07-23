#ifndef RADAR_FRONT_H
#define RADAR_FRONT_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <Eigen/Dense>

#include <vector>
#include <cmath>

struct EIGEN_ALIGN16 PointXYZV {
    PCL_ADD_POINT4D;
    float velocity;  // Custom field for velocity
    float azimuth;  // Custom field for velocity
    float elevate;

    // Constructor for easy initialization
    PointXYZV(float x = 0, float y = 0, float z = 0, float v = 0, float a = 0) 
        : x(x), y(y), z(z), velocity(v), azimuth(a), elevate(0) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZV,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, velocity, velocity)
    (float, azimuth, azimuth)
    (float, elevate, elevate)
)

class RadarObject {
public:
    pcl::PointCloud<PointXYZV>::Ptr point_cloud;            // Pointer to a point cloud
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float v_x = 0.0f;
    float v_y = 0.0f;
    float v_z = 0.0f;
    PointXYZV min_point;
    PointXYZV max_point;
    PointXYZV closest_point;

    long long time_stamp = 0;

    RadarObject() : point_cloud(new pcl::PointCloud<PointXYZV>()) {}
};

//> DBSCAN COEFFICIENTS
#define MINIMUM_POINTS 10    // minimum number of points in a cluster
#define EPSILON (0.15)       // distance for clustering, metre^2
#define CUTOFF_DISTANCE 250.0

#endif // RADAR_FRONT_H
