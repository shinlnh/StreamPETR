#ifndef __TJC_CONTROLLER_H__
#define __TJC_CONTROLLER_H__

#ifndef UT_TEST
#include "controller.h"
#include "tja_controller.h"
#include "lks_controller.h"

#else
#include "lib4test.h"
#endif

// #define TUNE_TJC_CONTROLLER
class TJCController : public Controller
{
private:
    TJAController& tja_controller;
    LKSController& lks_controller;

    float max_heading_planning;                             //[Deg]
    float heading_exponent;
    float velocity_exponent;
    float acceleration_exponent;
    float total_exponent;  
    float velocity_weight;

public:
    TJCController(TJAController& tja_controller, LKSController& lks_controller);
    void reset();
    void setInput(const PlanningResults& planning_results, ControlConfigs configs) override {}
    ControlResults execute(const PlanningResults& planning_results, const float& my_car_velocity,
                           ControlConfigs configs) override; 

    void resetTjcClock(void)
    {
       tja_controller.resetClock();
       lks_controller.resetClock();
    }

    void updateParameters(float new_heading_exponent, float new_velocity_exponent, float new_acceleration_exponent, float new_total_exponent, float new_velocity_weight)
    {
#ifdef TUNE_TJC_CONTROLLER    
        heading_exponent = new_heading_exponent;
        velocity_exponent = new_velocity_exponent;
        acceleration_exponent = new_acceleration_exponent;
        total_exponent = new_total_exponent;
        velocity_weight = new_velocity_weight;
        INFO("velocity_weight: %f", velocity_weight);
        INFO("steering_exponent: %f", heading_exponent);
        INFO("velocity_exponent: %f", velocity_exponent);
        INFO("acceleration_exponent: %f", acceleration_exponent);
        INFO("total_exponent: %f", total_exponent);
#endif
    }

    #ifndef UT_TEST
        void limitRange(float *x, float upper_limit, float lower_limit);
    #else
        virtual void limitRange(float *x, float upper_limit, float lower_limit);
    #endif
};

#endif // __TJC_CONTROLLER_H__