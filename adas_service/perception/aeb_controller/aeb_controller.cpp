#include "aeb_controller.h"

#include "controller.h"
#include "common.h"

AEBController::AEBController() 
{}

void AEBController::limitRange(float* x, float upper_limit, float lower_limit)
{
    *x = std::min(std::max(*x, lower_limit), upper_limit);
}

float AEBController::timeToCollision(float distance_to_car_ahead, float relative_leading_car_velocity,
                                     float following_car_velocity, float relative_acceleration)
{
    if (relative_acceleration == 0) {
        return distance_to_car_ahead / relative_leading_car_velocity;
    } else {
        float delta = std::pow(relative_leading_car_velocity, 2) + 2 * relative_acceleration * distance_to_car_ahead;
        float time_to_collision = (-relative_leading_car_velocity + std::sqrt(delta)) / relative_acceleration;
        return time_to_collision;
    }
}

float AEBController::timeStop(float distance_to_car_ahead, float relative_leading_car_velocity,
                              float following_car_velocity)
{
    float jerk1;
    float time_stop;
    if (following_car_velocity < 8.5 / 3.6)
        jerk1 = jerk1_extremely_low_velocity;
    else if (following_car_velocity < 15 / 3.6)
        jerk1 = jerk1_very_low_velocity;
    else if (following_car_velocity < 20 / 3.6)
        jerk1 = jerk1_low_velocity;
    else if (following_car_velocity < 50 / 3.6)
        jerk1 = jerk1_mid_low_velocity;
    else if (following_car_velocity < 70 / 3.6)
        jerk1 = jerk1_high_velocity;
    else
        jerk1 = jerk1_very_high_velocity;
    if (following_car_velocity <= -0.5 * jerk1 * jerk1_time * jerk1_time) {
        // In this case, the following vehicle will come to a stop in the first phase of braking (jerk_1).
        // As a result, the time_to_stop will be less than jerk1_time.
        time_stop = std::sqrt(-2 * following_car_velocity / jerk1);
        DEBUG("time_stop jerk 1");
        return time_stop;
    }
    float vel_start_after1 = following_car_velocity + 0.5 * jerk1 * jerk1_time * jerk1_time;
    float dec2_start = jerk1 * jerk1_time;
    float jerk2_time = (acceleration_phase3-(jerk1 * jerk1_time))/jerk2;
    float vel_start_phase2 = -(dec2_start * jerk2_time + 0.5 * jerk2 * jerk2_time * jerk2_time);
    if (vel_start_after1 < vel_start_phase2)
    {
        float delta = std::pow(jerk1 * jerk1_time, 2) - 2 * jerk2 * following_car_velocity;
        time_stop = jerk1_time + (-jerk1 * jerk1_time - std::sqrt(delta)) / jerk2;
        DEBUG("time_stop jerk 2");
        return time_stop;
    }
    else
    {
        float vel_start_phase3 = vel_start_after1 + dec2_start * jerk2_time + 0.5 * jerk2 * jerk2_time * jerk2_time;
        time_stop = jerk1_time + jerk2_time + (-vel_start_phase3 / acceleration_phase3);
        DEBUG("time_stop jerk 3");
        return time_stop;
    }
}

bool AEBController::dangerCheck(float distance_to_car_ahead, float relative_leading_car_velocity,
                                float following_car_velocity, float longitudinal_acceleration, bool is_reverse)
{
    DEBUG("distance_to_car_ahead: %f", distance_to_car_ahead);
    DEBUG("relative_leading_car_velocity: %f", relative_leading_car_velocity * 3.6);
    this->forward_collision_warning = false;
    if (true == is_reverse) {
        return false;
    }

    if (distance_to_car_ahead < minimum_allow_distance) {
        DEBUG("distance_to_car_ahead < minimum_allow_distance");
        DEBUG("distance_to_car_ahead: %f", distance_to_car_ahead);
        return true;
    }

    if (relative_leading_car_velocity >= 0 || following_car_velocity == 0) {
        return false;
    }

    float longitudinal_linear_acceleration = longitudinal_acceleration;
    limitRange(&longitudinal_linear_acceleration, MAX_CAR_ACCELERATION, MAX_CAR_DECELERATION);

    if (std::pow(-relative_leading_car_velocity, 2) + 2 * longitudinal_linear_acceleration * distance_to_car_ahead < 0) {
        return false; // In this condition, the following car will stop before colliding with the leading car
    } else {
        float time_to_collision = timeToCollision(distance_to_car_ahead, -relative_leading_car_velocity,
                                                  following_car_velocity, longitudinal_linear_acceleration);
        float time_stop = timeStop(distance_to_car_ahead, -relative_leading_car_velocity, following_car_velocity);
        DEBUG("time_to_collision: %f", time_to_collision);
        DEBUG("time_stop: %f", time_stop);
        if (time_stop + reaction_time >= time_to_collision)
        {
        // DANGER
            return true;
        }

        else {
            // We need to warn the driver for at least warning_time seconds before AEB activating
            this->forward_collision_warning =
                (time_stop + reaction_time + warning_time >= time_to_collision) ? true : false;
        }
        return false;
    }
}

bool AEBController::warningCheck() { return this->forward_collision_warning; }

ControlResults AEBController::execute()
{
    ControlResults control_results;
    control_results.steering_value = 0.0f;
    control_results.throttle_value = 0.0f;
    control_results.brake_value = 1.0f;
    return control_results;
}
