#ifndef _STANLEY_CONTROLLER_H
#define _STANLEY_CONTROLLER_H
#include "intermediate_representations.h"

class StanleyController
{
public:
    StanleyController();
    bool straight_road_param = false;
    float max_tracking_point_heading = 0.0;
    float execute(const PlanningResults& planning_results, const float& my_car_velocity);

    template <typename T>
    int sign(T val) {
        return (T(0) < val) - (val < T(0));
    }
    float findOptimalDirection(float desired_heading_angle);
private:
    float k_error_straight_road = 6.3f;
    float k_soft_straight_road = 65.5f;
    float k_velocity_straight_road = 0.8f;

    float k_error_curve_road = 7.35f;
    float k_soft_curve_road = 58.5f;
    float k_velocity_curve_road = 0.017f;

    // Variable for switching the Stanley parameters
    float max_point_heading = 3.5;
};

#endif // _STANLEY_CONTROLLER_H
