#include "hwc_controller.h"


HWCController::HWCController(ACCController& acc_controller, LKSController& lks_controller)
: acc_controller(acc_controller), lks_controller(lks_controller)
{   
    max_heading_planning = 60.0;
    heading_exponent = 1.85;
    velocity_exponent = 3.0;
    acceleration_exponent = 2.0;
    total_exponent = 1.0;
    velocity_weight = 0.705;
}


void HWCController::reset()
{
    // Reset ACC controller
    acc_controller.reset();

    // Reset controller 
    Controller::reset();
}


ControlResults HWCController::execute(const PlanningResults& planning_results, const float& my_car_velocity,
                                      ControlConfigs configs)
{
    ControlResults control_results_longitudinal;
    ControlResults control_resultsl_lateral;
    ControlResults control_results;
    control_results.steering_value = 0.0f;
    control_results.throttle_value = 0.0f;
    control_results.brake_value = 0.0f;

    /*-----The program for caculating and returning the steering value------------------*/
    lks_controller.setSteeringAngle(car_steering_angle);
    lks_controller.setWheelAngle(wheel_angle);
    control_resultsl_lateral = lks_controller.execute(planning_results, my_car_velocity, configs);
    control_results.steering_value = control_resultsl_lateral.steering_value; 
    /*----------------------------------------------------------------------------------*/

    /*-------The program for caculating and returning the throttle and brake values-----*/
    float car_acceleration_ = car_acceleration;
    // limitRange (&car_acceleration_, max_car_acceleration, max_car_deceleration);
    if (car_acceleration_ < 0) { car_acceleration_ = 0; }
    float velocity_coefficient = calculateVelCoef(my_car_velocity, lks_controller.max_tracking_point_heading, car_acceleration_);
    acc_controller.setDetectedVehicle(detected_vehicle);
    acc_controller.setVelCarHead(velocity_of_car_ahead);
    acc_controller.setDisCarHead(distance_with_car_ahead);
    acc_controller.setVelCoef(velocity_coefficient);
    control_results_longitudinal = acc_controller.execute(planning_results, my_car_velocity, configs);
    control_results.throttle_value = control_results_longitudinal.throttle_value;
    control_results.brake_value = control_results_longitudinal.brake_value;
    acc_controller.setVelCoef(1.0f);  // TEMPORARY, reset to avoid affect ACC controller

    return control_results;
}


float HWCController::calculateVelCoef(float car_velocity, float max_heading_point, float car_acceleration)
{
    // Calculate first part of coefficient -> affect by heading angle change
    float heading_factor = powf(std::fabs(max_heading_point) / max_heading_planning, heading_exponent);
    float velocity_factor = powf(1 + (car_velocity / MAX_CAR_VELOCITY), velocity_exponent);
    float acceleration_factor = powf(1 + (car_acceleration / MAX_CAR_ACCELERATION), acceleration_exponent);;
    float velocity_coefficient =  1 - heading_factor * velocity_factor * acceleration_factor;
    velocity_coefficient =  powf(velocity_coefficient, total_exponent);
    limitRange(&velocity_coefficient, 1.0, 0.0);

    // Calculate second part of coefficient -> affect by road curvature change
    // -- Calculate possible speed of car when moves to the curvature road
    float g = 9.81;
    float u = 0.5;
    float v_max = 3.6 * sqrt((g * u) / lks_controller.curvature_max);

    // INFO("car_velocity: %f", car_velocity);
    float curvature_coefficient = v_max / car_velocity;
    // -- Calculate Curvature coefficient following the reference car speed
    limitRange(&curvature_coefficient, 1, 0);    

    // Create the linear interpolation formula
    auto lerp = [](float a, float b, float t) {
        return (1.0f - t) * a + t * b;
    };

    // Calculate the combined_coefficient
    float combined_coefficient = lerp(curvature_coefficient, velocity_coefficient, velocity_weight);

    // Smooth the coefficient if speed change at high speed to smooth the dynamic
    if (last_combined_coefficient > 0.85f) 
    { 
        float max_up_rate = 0.02f;
        float delta = combined_coefficient - last_combined_coefficient;
        if (delta > max_up_rate) {
            combined_coefficient = last_combined_coefficient + max_up_rate;
        }
    }

    return combined_coefficient;
}


void HWCController::limitRange(float* x, float upper_limit, float lower_limit)
{
    *x = std::min(std::max(*x, lower_limit), upper_limit);
}