#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <vector>
#include <array>
#include <math.h>
#include <chrono>
#include "common.h"
#include "intermediate_representations.h"
#include "logger/general_logger.h"

// Define parameters for the car to execute the control module
#define MAX_CAR_VELOCITY    240.0f             // [km/h]
#define MAX_THROTTLE_VALUE  1.0f               // Maximum throttle value (normalized)
#define MAX_CAR_STEERING_ANGLE 37.0f      // [Deg]
#define MAX_CAR_ACCELERATION 10.5f             // [m/s^2]; this parameter defines the car's dynamic model
#define MAX_CAR_DECELERATION -16.5f            // [m/s^2]; this parameter defines the car's dynamic model

/**
 * @brief Interface for the controller classes which inherit this.
 * You can implement your controller all you want, but you must inherit from this class.
 * You can drive your controller all you like, but you must implement your input in setInput(), your output in execute()
 */
class Controller
{
protected:
    //parameters need to be caculated for the controller features
    float tau_value = 0.0;
    float car_acceleration = 0.0;                                               //[m/s^2]
    float car_steering_angle = 0.0;                                             //[deg]
    float reference_distance = 0.0;                                             //[m]
    float reference_velocity = 0.0;                                             //[km/h]
    bool detected_vehicle = false;
    float velocity_of_car_ahead = 0.0;                                          //[km/h]
    float distance_with_car_ahead = 0.0;                                        //[m]
    float epsSteeringAngle = 0.0f;

    SteeringParameters wheel_angle;
    uint64_t lastTimeProcessingMpc;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    float sample_time = 0.0;
public:
    Controller() = default;
    virtual ~Controller() = default;
    virtual void reset()
    {
        car_acceleration = 0.0;
        car_steering_angle = 0.0;
        detected_vehicle = false;
        velocity_of_car_ahead = 0.0;
        distance_with_car_ahead = 0.0;
    }
    virtual void setInput(const PlanningResults &planning_results, ControlConfigs configs) = 0;
    virtual ControlResults execute(const PlanningResults &planning_results, const float &my_car_velocity, ControlConfigs configs) = 0;

    float getThrotlle(void) { return tau_value; }

    float getSteeringAngle(void) { return car_steering_angle; }

    void setSteeringAngle(float car_steering_angle)
    {
        this->car_steering_angle = car_steering_angle;
        DEBUG("The car_steering_angle  is: %f", this->car_steering_angle);
    }
    
    void setEpsSteeringAngle(float epsSteeringAngle)
    {
        this->epsSteeringAngle = epsSteeringAngle;
    }

    float getEpsSteeringAngle(void)
    {
        return this->epsSteeringAngle;
    }
    
    SteeringParameters getWheelAngle(void)
    {
        return this->wheel_angle;
    }

    void setWheelAngle(SteeringParameters wheel_angle)
    {
        this->wheel_angle = wheel_angle;
    }

    void setRefDistance(float reference_distance)
    {
        this->reference_distance = reference_distance;
        DEBUG("The new car's reference distance  is: %f", reference_distance);
    }

    float getRefDist() { return reference_distance; }

    float getRefVel(void) { return reference_velocity; }

    void setRefVel(float reference_velocity)
    {
        this->reference_velocity = reference_velocity;
        DEBUG("The new car's reference velocity is: %f", reference_velocity);
    }

    void setVelCarHead(float velocity_of_car_ahead)
    {
        this->velocity_of_car_ahead = velocity_of_car_ahead;
        DEBUG("The velocity_of_car_ahead is: %f", velocity_of_car_ahead);
    }

    void setDisCarHead(float distance_with_car_ahead)
    {
        this->distance_with_car_ahead = distance_with_car_ahead;
        DEBUG("The distance_with_car_ahead is: %f", distance_with_car_ahead);
    }

    void setDetectedVehicle(bool detected_vehicle)
    {
        this->detected_vehicle = detected_vehicle;
        DEBUG("Detect vehicle ahead");
    }

    void setCarAcceleration(float car_acceleration)
    {
        this->car_acceleration = car_acceleration;
        DEBUG("The car_acceleration is: %f", car_acceleration);
    }

    void resetClock (void) {last_time = std::chrono::high_resolution_clock::now();}
};

#endif // __CONTROLLER_H__
