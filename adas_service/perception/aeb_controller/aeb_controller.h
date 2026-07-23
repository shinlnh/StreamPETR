#ifndef __AEB_CONTROLLER_H__
#define __AEB_CONTROLLER_H__

#include "controller.h"
#include "common.h"
#include "logger/general_logger.h"

/*
 * The AEBController class is deployed for implementing the Autonomous Emergency Braking (AEB) module.
 * A detailed document for this module can be found here: https://confluence.banvien.com.vn/display/ESRNDAUT/Autonomous+Emergency+Braking+System%3A+Implementation
 */

class AEBController
{
private:
    float reaction_time = 0.0;                  // [unit] = [s]; reaction time in seconds of the driver can reflect in time before the accident happens
    float warning_time = 3.0;                   // [unit] = [s]; Warning time in seconds for driver before AEB activating
    float minimum_allow_distance = 1.5;         // [unit] = [m]; minimum distance is validated between objects with our car

    // I modeled the relationship between deceleration and time when the brake is applied 
    // You can view the relevant image at the following file path: ./Relationship between deceleration and braking time.PNG
    // The deceleration jerk in the first step may vary depending on the vehicle's velocity
    // To account for this, I defined various "jerk_1" parameters as constants based on the velocity, as shown below

    //                          --------------- IMPORTANT NOTE -------------------
    // In the near future, these parameters could be refined and optimized using an approximate formula or an simple AI model 
    // that dynamically represents the relationship between deceleration and the vehicle's velocity 
    // This improvement would be especially beneficial for AEB activation
    
    float jerk1_extremely_low_velocity = -1.4;    // [unit] = [m/s^3]; Extremely low speed range: 0 km/h to < 8.5 km/h
    float jerk1_very_low_velocity = -5.595;        // [unit] = [m/s^3]; Very low speed range: 8.5 km/h to < 15 km/h
    float jerk1_low_velocity = -12.048;            // [unit] = [m/s^3]; Low speed range: 15 km/h to < 20 km/h
    float jerk1_mid_low_velocity = -19.448;        // [unit] = [m/s^3]; Mid-low speed range: 20 km/h to < 50 km/h

    float jerk1_high_velocity = -22.448;           // [unit] = [m/s^3]; High speed range: 50 km/h to 70 km/h
    float jerk1_very_high_velocity = -26.348;      // [unit] = [m/s^3]; Very high speed range: > 70 km/h

    float jerk1_time = 0.7;                        // [unit] = [s]; Time duration for deceleration jerk at the first step after braking is applied
    float jerk2 = 2.986;                           // [unit] = [m/s^3]; Value of the second jerk parameter

    float acceleration_phase3 = -3;

#ifndef UT_TEST
    void limitRange(float * x, float upper_limit, float lower_limit);
    float timeToCollision(float distance_to_car_ahead, float leading_car_velocity, float following_car_velocity, float relative_acceleration);
    float timeStop(float distance_to_car_ahead, float leading_car_velocity, float following_car_velocity);
#else
    virtual void limitRange(float * x, float upper_limit, float lower_limit);
    virtual float timeToCollision(float distance_to_car_ahead, float leading_car_velocity, float following_car_velocity, float relative_acceleration);
    virtual float timeStop(float distance_to_car_ahead, float leading_car_velocity, float following_car_velocity);
#endif

    bool forward_collision_warning = false;
    
public:
    AEBController();
    bool dangerCheck(float distance_to_car_ahead, float relative_leading_car_velocity, float following_car_velocity,
                     float longitudinal_acceleration, bool is_reverse);
    bool warningCheck();
    ControlResults execute();
    LogUtility log_AEBcontroller_info;
    bool logInfo_flag = false;
};
#endif // __AEB_CONTROLLER_H__
