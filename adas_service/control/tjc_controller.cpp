#include "tjc_controller.h"


TJCController::TJCController(TJAController& tja_controller, LKSController& lks_controller)
: tja_controller(tja_controller), lks_controller(lks_controller)
{
    max_heading_planning = 25.0;
    heading_exponent = 1.0;
    velocity_exponent = 1.0;
    acceleration_exponent = 1.0;
    total_exponent = 1.0;   
    velocity_weight = 0.5;
}


void TJCController::reset()
{
    // Reset ACC controller
    tja_controller.reset();

    // Reset controller 
    Controller::reset();
}


ControlResults TJCController::execute(const PlanningResults &planning_results, const float &my_car_velocity, ControlConfigs configs)
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
    if (car_acceleration < 0) { car_acceleration = 0; };
    tja_controller.setDetectedVehicle(detected_vehicle);
    tja_controller.setVelCarHead(velocity_of_car_ahead);
    tja_controller.setDisCarHead(distance_with_car_ahead);
    tja_controller.setVelCoef(1.0);
    control_results_longitudinal = tja_controller.execute(planning_results, my_car_velocity, configs);
    control_results.throttle_value = control_results_longitudinal.throttle_value;
    control_results.brake_value = control_results_longitudinal.brake_value;

    return control_results;
}

void TJCController::limitRange(float* x, float upper_limit, float lower_limit)
{
    *x = std::min(std::max(*x, lower_limit), upper_limit);
}