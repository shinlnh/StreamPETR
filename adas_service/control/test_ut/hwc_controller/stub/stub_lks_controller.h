#ifndef STUB_LKS_CONTROLLER_H
#define STUB_LKS_CONTROLLER_H


#include "intermediate_representations.h"
#include <chrono>

// Stub for class LSKController
class LKSController
{
public:
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    float max_tracking_point_heading = 0.0;
    float curvature_max = 0.0;
    float car_steering_angle;
    SteeringParameters wheel_angle;

    LKSController(){}

    ControlResults execute(const PlanningResults& planning_results, const float& my_car_velocity,
                           ControlConfigs configs) 
    {
        ControlResults result;
        result.steering_value = 5;
        return result;
    }

    void setInput(const PlanningResults& planning_results, ControlConfigs configs);

    void resetClock (void) {last_time = std::chrono::high_resolution_clock::now();}

    void setSteeringAngle(float car_steering_angle)
    {
        this->car_steering_angle = car_steering_angle;
        DEBUG("The car_steering_angle  is: %f", this->car_steering_angle);
    }

    void setWheelAngle(SteeringParameters wheel_angle)
    {
        this->wheel_angle = wheel_angle;
    }
 };
#endif