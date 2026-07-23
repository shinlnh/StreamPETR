#include <cmath>
#include <math.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include "intermediate_representations.h"
#include "common.h"
#include "logger/general_logger.h"
#include "planning_queue.h"

// #define LOG_PLANNED_DATA
#define CUT_OFF_ANGLE 4

using Point = std::array<float, 3>;

// Calculate distance between two points
static inline float distance(const Point &a, const Point &b) {
    return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]));
}

// Function to calculate the heading angle from a point to another point
static inline float calculateHeadingAngle(const Point& origin, const Point& point) {
    float dy = point[1] - origin[1];
    float dx = point[0] - origin[0];
    float angle = std::atan2(dy, dx); 
    angle *= 180 / M_PI; 
    if (angle < 0)
        angle += 360;
    return angle; // Angle in degrees
}

static double bernstein_poly(int n, int i, float t) {
    // Calculate binomial coefficient C(n, i)
    double binom = 1.0f;
    for (int j = 0; j < i; ++j) {
        binom *= (n - j) / static_cast<float>(j + 1);
    }

    return binom * std::pow(t, i) * std::pow(1 - t, n - i);
}

std::vector<std::array<float, 3>> PlanningQueue::bezier_curve(const std::vector<std::array<float, 3>>& control_points, int num_points = 130) {
    int n = control_points.size() - 1;
    std::vector<std::array<float, 3>> curve(num_points);

    for (int k = 0; k < num_points; ++k) {
        float t = static_cast<float>(k) / (num_points - 1);
        Point point = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i <= n; ++i) {
            double b = bernstein_poly(n, i, t);
            point[0] += b * control_points[i][0];
            point[1] += b * control_points[i][1];
        }
        curve[k] = {point[0], point[1], 0.0f};
    }

    return curve;
}

// Function to compute a cubic Bezier curve
static std::vector<std::array<float, 3>> cubic_bezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3, int num_points = 100) {
    std::vector<std::array<float, 3>> curve(num_points);
    for (int i = 0; i < num_points; ++i) {
        float t = static_cast<float>(i) / (num_points - 1);
        float u = 1 - t;
        float uu = u * u;
        float uuu = uu * u;
        float tt = t * t;
        float ttt = tt * t;

        curve[i][0] = uuu * p0[0] + 3 * uu * t * p1[0] + 3 * u * tt * p2[0] + ttt * p3[0];
        curve[i][1] = uuu * p0[1] + 3 * uu * t * p1[1] + 3 * u * tt * p2[1] + ttt * p3[1];
        curve[i][2] = 0.0f;
    }
    return curve;
}

PlanningQueue::PlanningQueue(){
    this->planning_results = {};
    this->policy = NEWEST;
    // this->policy = NEWEST;
    this->root_yaw = this->buffer_yaw;
    this->root_location = this->buffer_position;
    // Enable log to file, meaning that the logged values will be persisted to log files.
    // log.enableLogFile();    
}

// Initiate the logger
LogUtility PlanningQueue::log("log/", "planning_queue");

PlanningQueue::~PlanningQueue(){
    this->planning_results.clear();
}

void PlanningQueue::enqueue(std::vector<std::array<float, 3>> new_result){
    this->current_position = this->buffer_position;
    this->car_yaw = this->buffer_yaw;
    
    if (new_result.size() <= 0){
        this->root_position = this->current_position;
        return;
    }

    std::vector<std::array<float, 3>> new_points;
    // Transform the waypoints from our car coordinates to global coordinates 
    // Shifting the car orientation to 0 degrees
    // Computed shifted coordinates
    float cos_yaw = cos(this->car_yaw);
    float sin_yaw = sin(this->car_yaw);
    for (auto &point: new_result){
        float delta_x = point[0] * cos_yaw - point[1] * sin_yaw;
        float delta_y = point[0] * sin_yaw + point[1] * cos_yaw;
        std::array<float, 3> tmp = {this->current_position[0] + delta_x, this->current_position[1] + delta_y, 0.0f};
        new_points.push_back(tmp);
    }

    if (this->planning_results.size() <= 0){
        this->root_yaw = this->car_yaw;
        this->root_position = this->current_position;
    }

    switch (this->policy){
        case NEWEST:
            this->planning_results.clear();
            this->planning_results = new_points;
            this->root_position = this->current_position;
            break;
        
        case MERGING:
            this->merge(new_points);
            break;

        case REPLAN:
            //TODO
            break;
            
        default:
            break;
    }
}

void PlanningQueue::setCarLocation(float x, float y, float z){
    this->buffer_position[0] = x;
    this->buffer_position[1] = y;
    this->buffer_position[2] = z;

    #ifdef LOG_PLANNED_DATA
    log.d(VV"GNSS: ", x, ":", y, ":", z);
    #endif
}

std::vector<std::array<float, 3>> PlanningQueue::getPlanningResults(){
    std::vector<std::array<float, 3>> tmp;
    tmp.reserve(this->planning_results.size() + 1);

    Point cur_point = {0.0f, 0.0f, 0.0f};
    tmp.push_back(cur_point);
    Point last_point = cur_point;

    // Check if planning_results has enough elements
    if (this->planning_results.size() > 1) {
        float cos_yaw = cos(0.0f - car_yaw);
        float sin_yaw = sin(0.0f - car_yaw);
        
        for (auto point: this->planning_results) {
            cur_point[0] = (point[0] - current_position[0])*cos_yaw - (point[1] - current_position[1])*sin_yaw;
            cur_point[1] = (point[0] - current_position[0])*sin_yaw + (point[1] - current_position[1])*cos_yaw;
            cur_point[2] = 90 - calculateHeadingAngle(last_point, cur_point);
            tmp.push_back(cur_point);
            last_point = cur_point;
        }

        // Erase first 2 points in tmp
        tmp.erase(tmp.begin(), tmp.begin() + 2);
    }

    return tmp;
};

void PlanningQueue::setYaw(float yaw){
    this->buffer_yaw = yaw - M_PI / 2;
}

void PlanningQueue::setPolicy(planning_queue_policy policy){
    this->policy = policy;
}

void PlanningQueue::reset(){
    this->planning_results.clear();
    this->root_yaw = this->buffer_yaw;
    this->root_location = this->buffer_position;
}

void PlanningQueue::merge(std::vector<std::array<float, 3>>& new_points){
    int passed_count = 0;
    int num_of_newpoints = 0;

    // Remove passed points first
    float current_distance = distance(this->root_position, this->current_position);
    float est_distance = 0;

    for (size_t i = 0; i < planning_results.size(); ++i){
        est_distance = distance(this->root_position, planning_results[i]);
        if (est_distance <= current_distance)
            passed_count++;
        else
            break;
    }

    // Remove passed points from previous planning results
    if (passed_count){
        if (passed_count <= planning_results.size())
            this->planning_results.erase(planning_results.begin(), planning_results.begin() + passed_count);
        else 
            this->planning_results.clear();
    }
    
    // Add new points
    if (this->planning_results.size() <= 20){
        // The number of points is not enough for a merge,
        // We will take the newest instead
        this->planning_results = new_points;
        this->root_position = this->current_position;
    }
    
    else {
        float heading = 90 - calculateHeadingAngle(this->current_position, this->planning_results[0]);
        if (std::fabs(heading) > CUT_OFF_ANGLE){
            // When jump in this case, our heading angle with the planning is not safe enough for a merge,
            // so maybe the newest maybe better
            // this is just a simple workarround, we may need to improve this policy
            this->planning_results.clear();
            this->planning_results = new_points;
            this->root_position = this->current_position;
            return;
        }
        auto lastPoint = this->planning_results.end() - 1;
        auto prevLastPoint = this->planning_results.end() - 2;
        // We will check the distance first
        // A new point only be marked if its distance with root point
        // is larger than the distance between the root point and the last planning point
        current_distance = distance(*lastPoint, this->root_position);
        float ref_heading = calculateHeadingAngle(*prevLastPoint, *lastPoint);
        bool first_new = true;
        
        for (size_t i = 0; i < new_points.size(); ++i){
            est_distance = distance(new_points[i], this->root_position);
            if (est_distance > current_distance + 0.1){
                // Heading angle = atan2(y2 - y1, x2 - x1)
                if (first_new){
                    heading = calculateHeadingAngle(*lastPoint, new_points[i]);
                    if (std::fabs(ref_heading- heading) > CUT_OFF_ANGLE)
                        continue;
                    first_new = false;
                }
                    
                else{ 
                    heading = calculateHeadingAngle(new_points[i - 1], new_points[i]);
                    if (std::fabs(new_points[i - 1][2] - heading) > CUT_OFF_ANGLE)
                        continue;
                }

                new_points[i][2] = heading;
                this->planning_results.push_back(new_points[i]);
                num_of_newpoints++;
            }
        }
    }
    
    // Update root position
    if (passed_count) 
        this->root_position = this->current_position;

    if (!num_of_newpoints)
        return;

    // Smooth the planning using bezier
    this->planning_results = bezier_curve(this->planning_results, this->planning_results.size());

    #ifdef LOG_PLANNED_DATA
    // Append new points to the log file
    for (size_t i = this->planning_results.size() - num_of_newpoints; i < this->planning_results.size(); ++i)
        log.d(VV"Planned: ", this->planning_results[i][0], ":", this->planning_results[i][1]);
    #endif
}

float PlanningQueue::getYaw(){
    float yaw = (this->car_yaw - this->root_yaw) * 180 / M_PI;
    if (yaw < 0)
        yaw += 360;
    return yaw;
};

Point PlanningQueue::getCarLocation(){
    return this->buffer_position;
}