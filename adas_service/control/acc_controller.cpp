#include "acc_controller.h"

ACCController::ACCController()
{
    // Initialize PD Fuzzy controller parameters for distance keeping
    pd_fuzzy_distance.Ke = 0.0084f;
    pd_fuzzy_distance.Ke_dot = 0.6f;
    pd_fuzzy_distance.Ku = 24.0f;
    pd_fuzzy_distance.uk_1 = 0.0f;
    pd_fuzzy_distance.ek_1 = 0.0f;
    pd_fuzzy_distance.a = 0.5f;
    pd_fuzzy_distance.b = 0.72f;
    pd_fuzzy_distance.c = 0.94f;

    // Initialize PI Fuzzy controller parameters for velocity keeping
    pi_fuzzy_velocity.Ke = 0.004f;
    pi_fuzzy_velocity.Ke_dot = 0.64f;
    pi_fuzzy_velocity.Ku = 0.778f;
    pi_fuzzy_velocity.uk_1 = 0.0f;
    pi_fuzzy_velocity.ek_1 = 0.0f;
    pi_fuzzy_velocity.a = 0.45f;
    pi_fuzzy_velocity.b = 0.70f;
    pi_fuzzy_velocity.c = 0.92;

    velocity_coefficient = 1;

#ifdef TUNE_ACC_SPEED_KEEPING
    acc_speed_logging.enableLogFile();
#endif

#ifdef TUNE_ACC_DISTANCE_KEEPING
    acc_distance_logging.enableLogFile();
#endif
}


void ACCController::reset()
{
    // Reset Fuzzy controller last states
    pd_fuzzy_distance.uk_1 = 0.0f;
    pd_fuzzy_distance.ek_1 = 0.0f;

    pi_fuzzy_velocity.uk_1 = 0.0f;
    pi_fuzzy_velocity.ek_1 = 0.0f;

    velocity_coefficient = 1.0f;

    // Reset controller 
    Controller::reset();
}


ControlResults ACCController::execute(const PlanningResults& planning_results, const float& my_car_velocity,
                                      ControlConfigs configs)
{
    ControlResults control_results;
    float desired_velocity_ACC = 0.0f;
    
    // Get time
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = current_time - last_time;
    sample_time = elapsed.count();
    last_time = current_time;

    // Reset controller if it has been a while since last run
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    static constexpr decltype(elapsed_seconds) reset_threshold = 1;  // second(s)
    if (elapsed_seconds > reset_threshold) {
        reset();
        sample_time = 0.0f;
    }

    bool isVelCase = true;
    // Calculate speed based on distance
    if (detected_vehicle) {
        desired_velocity_ACC = velocity_of_car_ahead + pdFuzzyController(reference_distance, distance_with_car_ahead, sample_time, &pd_fuzzy_distance);
        isVelCase = false;
#ifdef TUNE_ACC_DISTANCE_KEEPING
        acc_distance_logging.i(VV "desired_distance_ACC: ", reference_distance, "; actual_distance_ACC: ", distance_with_car_ahead);
#endif    
    } else {
        desired_velocity_ACC = reference_velocity;

#ifdef TUNE_ACC_SPEED_KEEPING
        acc_speed_logging.i(VV "desired_velocity_ACC: ", desired_velocity_ACC, "; actual_velocity_ACC: ", my_car_velocity);
#endif
    }
    desired_velocity_ACC *= velocity_coefficient;
    
    // Limit speed
    if (!isVelCase)
    {
        desired_velocity_ACC = std::min(desired_velocity_ACC, reference_velocity-2);
    }
    else
    {
        desired_velocity_ACC = std::min(desired_velocity_ACC, reference_velocity);
    }

    // Calculate control based on speed
    tau_value = piFuzzyController(desired_velocity_ACC, my_car_velocity, sample_time, &pi_fuzzy_velocity);
    
    // Control value
    if (tau_value >= 0.0f) {
        control_results.throttle_value = tau_value;
    } else {
        // prevent partially braking when car's velocity approach zero
        if (my_car_velocity <= 0.1f) {
            control_results.brake_value = 1.0f; 
        } else {
            control_results.brake_value = -tau_value; 
        }
    }
    DEBUG("Steering of car: %f", control_results.steering_value);
    DEBUG("Throttle of car: %f", control_results.throttle_value);
    DEBUG("Break of car: %f", control_results.brake_value);
    return control_results;
}
