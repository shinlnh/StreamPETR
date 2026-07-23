#ifndef PLANNING_QUEUE_H
#define PLANNING_QUEUE_H

#ifndef UT_TEST

#else
#include "lib4test.h"
#endif

typedef enum planning_queue_policy_t {
    NEWEST,
    MERGING,
    REPLAN,
} planning_queue_policy;

class PlanningQueue{
private:
    // Root loacation of our vehicle
    std::array<float, 3> root_position;
    float root_yaw = 0;
    std::array<float, 3> root_location;

    // Current loacation of our vehicle from GNSS
    std::array<float, 3> current_position;

    // Current global PlanningResults
    std::vector<std::array<float, 3>> planning_results;

    // Current car yaw
    float car_yaw;

    // Buffer for receving data from sensor callbacks
    std::array<float, 3> buffer_position;
    float buffer_yaw;

    // Queue policy
    planning_queue_policy policy;

    // Merging functions
    std::vector<std::array<float, 3>> bezier_curve(const std::vector<std::array<float, 3>>&, int);
    void merge(std::vector<std::array<float, 3>>&);

    // Logger
    static LogUtility log;

public:
    // Constructor
    PlanningQueue();
    
    // Destructor
    ~PlanningQueue();
    
    // Add a new PlanningResults to the queue
    void enqueue(std::vector<std::array<float, 3>>);

    // Update current vehicle's location
    void setCarLocation(float, float, float);
    void setYaw(float);

    std::vector<std::array<float, 3>> getPlanningResults();

    void setPolicy(planning_queue_policy);

    void reset();

    // Get current vehicle's location
    std::array<float, 3> getCarLocation();
    float getYaw();
};

#endif 