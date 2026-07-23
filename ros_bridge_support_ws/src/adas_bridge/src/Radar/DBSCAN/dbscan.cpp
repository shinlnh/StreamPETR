#include <iostream>

#include "dbscan.h"
/**
 * @brief Walk the Point vector, try to expand every point until it's not possible.
 * The expansive happens at the beginning of the vector, as is the cluster ID.
 * 
 * @return int 
 */
int DBSCAN::run(vector<Point> points)
{
    // Setup the pipeline to run.
    m_points = points;
    m_pointSize = points.size();

    int clusterID = 1;
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)    // iter is pointer to a Point object
    {
        if ( iter->clusterID == UNCLASSIFIED )  // If a point is unclassified, try to expand the cluster until not possible.
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )   // Dereference the pointer, pass it in expand function.
            {
                clusterID += 1; // Everytime we fail to expand the cluster around a point (it's noise), we make a new cluster.
            }
        }
    }
    if (clusterID == 1) return 0;
    detected_clusters = clusterID - 1;
    return 0;
}

/**
 * @brief 
 * What is a seed: A group of points before performing calculations.
 * 
 * @param point 
 * @param clusterID 
 * @return int 
 */
int DBSCAN::expandCluster(Point point, int clusterID)
{    
    vector<int> clusterSeeds = calculateCluster(point);

    if ( clusterSeeds.size() < m_minPoints )    // If the cluster list size is smaller than a required size.
    {
        point.clusterID = NOISE;    // Then that point is marked as noise.
        return FAILURE;             // And we don't try to expand the cluster round that point.
    }
    else    // Else we try to expand the cluster.
    {
        int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points.at(*iterSeeds).clusterID = clusterID;  // All adjacent point is assigned the cluster ID (painting the cluster).
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y)
            {
                indexCorePoint = index; // Mark the core.
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);    // The calculation doesn't take into account the core point, so we have to remove that point.

        for( vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )    // Iterate the clusterseed.
        {
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));   // Find another cluster around each point in the clusterseed.

            if ( clusterNeighors.size() >= m_minPoints )    // Only perform calculation if neighbor list is big enough.
            {
                vector<int>::iterator iterNeighors;
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )   // Iterate every neighbors.
                {
                    if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                    {
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED ) // If the neighbor is unclassifed then extend the list
                        {                                                           // Then continue the calculations recursively.            
                            clusterSeeds.push_back(*iterNeighors);  // Add neighbor to list.
                            n = clusterSeeds.size();                // Update list size.
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;   // Paint neighbor to cluster.
                    }
                }
            }
        }

        return DBCAN_SUCCESS;
    }
}

/**
 * @brief Pass in a point, the loop returns the list of indices of every points that satisfies the epsilon distance with that point.
 * 
 * @param point 
 * @return vector<int> 
 */
vector<int> DBSCAN::calculateCluster(Point point)
{
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    for( iter = m_points.begin(); iter != m_points.end(); ++iter)   // Walk the point list again.
    {   
        double distance = calculateDistance3D(point, *iter);
        if ( distance <= m_epsilon* std::log1p(distance / 0.002) ) // Check if between the passed in point and other points <= epsilon
        {
            clusterIndex.push_back(index);  // Saves the index of every point the satisfies the epsilon distance (including the passed in point itself).
        }
        index++;
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(const Point& pointCore, const Point& pointTarget )
{
    return pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2);
}

inline double DBSCAN::calculateDistance3D(const Point& pointCore, const Point& pointTarget )
{
    return std::sqrt(pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2) + pow(pointCore.z - pointTarget.z, 2));
}


