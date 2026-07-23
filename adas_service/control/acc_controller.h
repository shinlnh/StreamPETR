#ifndef __ACC_CONTROLLER_H__
#define __ACC_CONTROLLER_H__

#ifndef UT_TEST
#include "controller.h"
#include "fuzzy_controller.h"

#else
#include "lib4test.h"
#endif

// #define TUNE_ACC_SPEED_KEEPING
// #define TUNE_ACC_DISTANCE_KEEPING
class ACCController: public Controller, public FuzzyController
{
protected:
    FuzzyParameters pd_fuzzy_distance;
    FuzzyParameters pi_fuzzy_velocity;
    float velocity_coefficient;
#ifdef TUNE_ACC_SPEED_KEEPING
    LogUtility acc_speed_logging = LogUtility("log/", "acc_speed_keeping_log", false);       
#endif

#ifdef TUNE_ACC_DISTANCE_KEEPING
    LogUtility acc_distance_logging = LogUtility("log/", "acc_distance_keeping_log", false);
#endif

public:
    ACCController();
    void reset();
    void setInput(const PlanningResults& planning_results, ControlConfigs configs) override {}
    ControlResults execute(const PlanningResults& planning_results, const float& my_car_velocity,
                           ControlConfigs configs) override;   

    void setVelCoef(float velocity_coefficient)
    {
        this->velocity_coefficient = velocity_coefficient;
        DEBUG("The velocity_coefficient is: %f", velocity_coefficient);
    }                                            
    void updateParameters(float new_ke, float new_ke_dot, float new_ku, float new_a, float new_b, float new_c)
    {
#ifdef TUNE_ACC_SPEED_KEEPING
        pi_fuzzy_velocity.Ke = new_ke;
        pi_fuzzy_velocity.Ke_dot = new_ke_dot;
        pi_fuzzy_velocity.Ku = new_ku;
        pi_fuzzy_velocity.a = new_a;
        pi_fuzzy_velocity.b = new_b;
        pi_fuzzy_velocity.c = new_c;
        INFO("Ke velocity: %lf", pi_fuzzy_velocity.Ke);
        INFO("Ke_dot: %lf", pi_fuzzy_velocity.Ke_dot);
        INFO("Ku: %lf", pi_fuzzy_velocity.Ku);
        INFO("a: %lf", pi_fuzzy_velocity.a);
        INFO("b: %lf", pi_fuzzy_velocity.b);
        INFO("c: %lf", pi_fuzzy_velocity.c);
#endif

#ifdef TUNE_ACC_DISTANCE_KEEPING
        pd_fuzzy_distance.Ke = new_ke;
        pd_fuzzy_distance.Ke_dot = new_ke_dot;
        pd_fuzzy_distance.Ku = new_ku;
        pd_fuzzy_distance.a = new_a;
        pd_fuzzy_distance.b = new_b;
        pd_fuzzy_distance.c = new_c;
        INFO("Ke distance : %lf", pd_fuzzy_distance.Ke);
        INFO("Ke_dot: %lf", pd_fuzzy_distance.Ke_dot);
        INFO("Ku: %lf", pd_fuzzy_distance.Ku);
        INFO("a: %lf", pd_fuzzy_distance.a);
        INFO("b: %lf", pd_fuzzy_distance.b);
        INFO("c: %lf", pd_fuzzy_distance.c);
#endif
    }

    /**
     * @brief Initialize the PI velocity controller with the current throttle value.
     * 
     * This prevents the controller from starting at 0 when transitioning from manual
     * to speed-keeping mode, avoiding a momentary speed drop.
     * 
     * @param throttle Current throttle value (0.0 to 1.0)
     */
    void initializeWithThrottle(float throttle)
    {
        // Convert throttle to internal PI controller range
        // uk_1 is stored before Ku multiplication, so we need to divide by Ku
        if (pi_fuzzy_velocity.Ku != 0.0f) {
            pi_fuzzy_velocity.uk_1 = throttle / pi_fuzzy_velocity.Ku;
            // Limit to valid range
            pi_fuzzy_velocity.uk_1 = std::min(std::max(pi_fuzzy_velocity.uk_1, -1.0f), 1.0f);
        }
        // Reset the clock to avoid triggering the reset logic in execute()
        this->resetClock();
        DEBUG("Initialized ACC controller with throttle: %f (uk_1: %f)", throttle, pi_fuzzy_velocity.uk_1);
    }
};
#endif // __ACC_CONTROLLER_H__
