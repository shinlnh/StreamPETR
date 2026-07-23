#ifndef __HWC_CONTROLLER_H__
#define __HWC_CONTROLLER_H__

#ifndef UT_TEST
#include "controller.h"
#include "acc_controller.h"
#include "lks_controller.h"
#include "logger/general_logger.h"

#else
#include "lib4test.h"
#endif

// #define TUNE_HWC_CONTROLLER
class HWCController: public Controller
{
private:
    ACCController& acc_controller;
    LKSController& lks_controller;

    float max_heading_planning;     //[Deg]
    float velocity_weight;                        
    float heading_exponent;
    float velocity_exponent;
    float acceleration_exponent;
    float total_exponent;

    float last_combined_coefficient;

public:
    HWCController(ACCController& acc_controller, LKSController& lks_controller);
    void reset();
    void setInput(const PlanningResults& planning_results, ControlConfigs configs) override {}
    ControlResults execute(const PlanningResults& planning_results, const float& my_car_velocity,
                           ControlConfigs configs) override; 

    float calculateVelCoef(float car_velocity, float max_heading_point, float car_acceleration);  
    void resetHwcClock(void)
    {
       acc_controller.resetClock();
       lks_controller.resetClock();
    }

    void limitRange(float *x, float upper_limit, float lower_limit);

    void updateParameters(float new_heading_exponent, float new_velocity_exponent, float new_acceleration_exponent, float new_total_exponent, float new_velocity_weight)
    {
#ifdef TUNE_HWC_CONTROLLER    
        heading_exponent = new_heading_exponent;
        velocity_exponent = new_velocity_exponent;
        acceleration_exponent = new_acceleration_exponent;
        total_exponent = new_total_exponent;
        velocity_weight = new_velocity_weight;
        INFO("heading_exponent: %f", heading_exponent);
        INFO("velocity_exponent: %f", velocity_exponent);
        INFO("acceleration_exponent: %f", acceleration_exponent);
        INFO("velocity_weight: %f", velocity_weight);
#endif
    }
};
#endif // __HWC_CONTROLLER_H__
