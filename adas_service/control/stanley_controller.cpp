#include "stanley_controller.h"

StanleyController::StanleyController() {}

float StanleyController::execute(const PlanningResults& planning_results, const float& my_car_velocity)
{
    max_tracking_point_heading = 0.0;
    float tracking_point_heading = 0.0;
    float min_cross_track_error = 999.0;
    float tracking_point_x = 0.0;
    float tracking_point_y = 0.0;
    float cross_track_steering = 0.0;

    for (const auto &point : planning_results.points) {
        float cross_track_error = std::sqrt(std::pow(point[0], 2) + std::pow(point[1], 2));
        if (cross_track_error < min_cross_track_error) {
            min_cross_track_error = cross_track_error;
            tracking_point_x = point[0];
            tracking_point_y = point[1];
            tracking_point_heading = point[2];
        }
        if (max_tracking_point_heading < std::fabs(point[2]));
            max_tracking_point_heading = std::fabs(point[2]);
    }
    if (max_tracking_point_heading < max_point_heading) {
        cross_track_steering = sign(tracking_point_x) * std::atan2(k_error_straight_road * min_cross_track_error,
                                k_soft_straight_road + k_velocity_straight_road * (my_car_velocity / 3.6)) * 180 / M_PI;
        straight_road_param = true;
    } else {
        cross_track_steering = sign(tracking_point_x) * std::atan2(k_error_curve_road * min_cross_track_error,
                                k_soft_curve_road + k_velocity_curve_road * (my_car_velocity / 3.6)) * 180 / M_PI;
        straight_road_param = false;
    }
    // if sign(tracking_point_x) > 0, the car is to the left of the path
    // if sign(tracking_point_x) < 0, the car is to the right of the path
    // if sign(tracking_point_x) == 0, the car radials with the reference point of the path through the origin x0y
    float desired_heading_angle = findOptimalDirection(tracking_point_heading + cross_track_steering);
    return desired_heading_angle;
}

float StanleyController::findOptimalDirection(float desired_heading_angle)
{
    if (desired_heading_angle < -180)
        return desired_heading_angle + 360;
    else if (desired_heading_angle > 180)
        return desired_heading_angle - 360;
    else
        return desired_heading_angle;
}
