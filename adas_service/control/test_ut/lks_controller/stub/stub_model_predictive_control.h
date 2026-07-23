#ifndef STUB_MODEL_PREDICTIVE_CONTROL_H
#define STUB_MODEL_PREDICTIVE_CONTROL_H

#include <vector>
#include <array>
#include <math.h>
#include <chrono>
#include "common.h"

#include "intermediate_representations.h"

// Stub for clas ModelPredictiveControl
class ModelPredictiveControl
{  
public:
  std::array<float, 3> car_position;
  Eigen::Matrix<float, Eigen::Dynamic,  Eigen::Dynamic> Q;                    // Q is the output weight matrix, a positive semi-definite weighting matrix
  Eigen::Matrix<float, Eigen::Dynamic,  Eigen::Dynamic> R;   
  
  static constexpr uint8_t predicted_steps = 10;
  static constexpr uint8_t control_steps = 10;

  ModelPredictiveControl()
  {
  	Q.resize(3 * predicted_steps, 3 * predicted_steps); 
  	R.resize(control_steps, control_steps); 
  }
  
  void setCarPosition(const std::array<float, 3>& updated_car_position)
  {
    this->car_position = updated_car_position;
  }
  
  float execute(const PlanningResults& planning_results, const float& my_car_velocity, const float& my_car_acceleration, 
                const float& sample_time_mpc, const float& car_steering_angle)
  {
    return 3.0;
  }
};

#endif