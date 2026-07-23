#ifndef STUB_TJA_CONTROLLER_H
#define STUB_TJA_CONTROLLER_H

#include "controller.h"
#include <chrono>
#include "intermediate_representations.h"

// Stub for clas TJAController
class TJAController
{  
public:
    float velocity_coefficient;
    bool detected_vehicle = false;
    float velocity_of_car_ahead = 0.0; 
    float distance_with_car_ahead = 0.0;                                        //[m] 
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;

    TJAController()
    {
        velocity_coefficient = 1;    
    }

    void reset()
    { 
        float velocity_coefficient = 1;
        bool detected_vehicle = false;
        float velocity_of_car_ahead = 0.0; 
        float distance_with_car_ahead = 0.0;
    }

    // Stub of function excute
    ControlResults execute(const PlanningResults& planning_results, const float& my_car_velocity,
                           ControlConfigs configs) 
    {
        ControlResults result;
        result.brake_value = 5;
        result.throttle_value = 5;
        return result;
    };

    void setInput(const PlanningResults& planning_results, ControlConfigs configs);

    void setVelCoef(float velocity_coefficient)
    {
        this->velocity_coefficient = velocity_coefficient;
        DEBUG("The velocity_coefficient is: %f", velocity_coefficient);
    }
    
    void setDetectedVehicle(bool detected_vehicle)
    {
        this->detected_vehicle = detected_vehicle;
        DEBUG("Detect vehicle ahead");
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
    
    void resetClock (void) {last_time = std::chrono::high_resolution_clock::now();}    
};

#endif