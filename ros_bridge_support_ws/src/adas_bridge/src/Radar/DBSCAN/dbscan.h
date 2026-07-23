#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define DBCAN_SUCCESS 0
#define FAILURE -3

using namespace std;

typedef struct Point_
{
    float x, y, z;      // X, Y, Z position
    float velocity;     // Custom field: Used for tracking vehicles speed.
    int clusterID;      // Cluster ID
    float width;        // Object size
    float azimuth;      // Represents the azimuth angle (in radians)
    float elevate;

    // Constructor.
    Point_(
    float x_val = 0.0f, 
    float y_val = 0.0f, 
    float z_val = 0.0f,
    float velocity_val = 0.0f, 
    int clusterID_val = -1, 
    float width_val = 0.0f, 
    float azimuth_val = 0.0f,
    float elevate_val = 0.0f
    ) : 
    x(x_val), 
    y(y_val), 
    z(z_val),
    velocity(velocity_val), 
    clusterID(clusterID_val), 
    width(width_val), 
    azimuth(azimuth_val),
    elevate(elevate_val) {}
} Point;

class DBSCAN {
public:    
    DBSCAN(unsigned int minPts, float eps)
    {
        m_minPoints = minPts;
        m_epsilon = eps;
    }
    ~DBSCAN(){}

    int run(vector<Point> points);
    vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
    inline double calculateDistance3D(const Point& pointCore, const Point& pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    float getEpsilonSize() {return m_epsilon;}
    
    // Runtime parameter update
    void updateParameters(unsigned int minPts, float eps) {
        m_minPoints = minPts;
        m_epsilon = eps;
    }
    
public:
    int detected_clusters;
    vector<Point> m_points;
    
private:    
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif // DBSCAN_H