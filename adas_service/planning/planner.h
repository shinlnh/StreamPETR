#ifndef __PLANNER_H__
#define __PLANNER_H__

#include "frenet_environment/frenet_environment.hpp"
#include "frenet_environment/settings.h"
#include "trajectory_generator/trajectory_generator.h"
#include "decision_making/decisionmakingmodule.h"
#include "curve_fitter/curve_fitter.h"
#include "intermediate_representations.h"
#include "aeb_controller/aeb_controller.h"
#include "general_maths.h"
#include "safety_force_field/safety_force_field.h"

class Planner
{
private:
    AEBController* aeb_controller_;
    FrenetEnvironment frenet_environment_;
    TrajectoryGenerator trajectory_generator_;
    DecisionMakingModule decision_maker_;
    SafetyForceField safety_force_field_;

    float car_width;
    bool aeb_debug = false;
    bool is_warning_aeb = false;
    bool is_danger_aeb = false;
    bool is_traffic_jam = false;
    int trafficJamCounter = 0;

    const std::shared_ptr<OutputObject> getClosestObject(PerceptionOutputObject& perception_output);
    bool isObjectInRange(const std::shared_ptr<const OutputObject>& object);
    bool isObjectInLane(const std::shared_ptr<const OutputObject>& object, 
                        const std::vector<float>& left_coeffs_world,
                        const std::vector<float>& right_coeffs_world);
    std::vector<std::vector<float>> retrieveSelectedPath();
    std::vector<FusionObject> convertToFusionObjects(PerceptionOutputObject& perception_output);
    std::vector<std::vector<cv::Point2f>> convertToObjectPointsWorld(PerceptionOutputObject& perception_output);
    void computeCenterLane(const LaneMarking& left, const LaneMarking& right, PlanningResults& centerLine);

public:
    Planner(bool aeb_debug = false);
    PlanningResults execute(PerceptionOutputObject& perception_output,
                            LaneDetection& lane_detection, 
                            TrajectoryGenerationPolicy trajectory_generation_policy, 
                            DecisionMakingPolicy decision_policy);
    bool dangerCheck(PerceptionOutputObject& perception_output);
    const std::shared_ptr<OutputObject> getClosestVehicle(PerceptionOutputObject& perception_output);
    std::pair<std::shared_ptr<OutputObject>, float> getClosestVehicleInLane(PerceptionOutputObject& perception_output,
                                                                            const LaneDetection& lane_detection);
    bool detectTrafficJam(const PerceptionResults &perceptionResult);
    bool isWarningAEB() const {return is_warning_aeb;}
    bool isDangerAEB() const {return is_danger_aeb;}
    bool isAebDebug() const {return aeb_debug;}
    const SafetyForceField& getSFF() const {return safety_force_field_;}
    void resetAEBState();
    bool isTrafficJam() const {return is_traffic_jam;}
    static LogUtility log_planner;
};

#endif // __PLANNER_H__