#ifndef DECISIONMAKINGMODULE_H
#define DECISIONMAKINGMODULE_H

#ifndef UT_TEST

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "frenet_environment/frenet_environment.hpp"

#else
#include "lib4test.h"
#endif

/**
 * The Decision Making Module is used in tandem with the Path Planning module.
 * More specifically it is placed between the candipath path generation step and the visualizion and control step.
 * It takes in a vector of generated candidate paths and then output the chosen path based on the selected policy and cost function.
 * NOTE: This implementation set the chosen path's cost to -1 as the SELECTED_PATH_INDICATOR below.
 * 
 */

#define SELECTED_PATH_INDICATOR -1
typedef enum
{
    LOWEST_COLLISION_SCORE,
    LOWEST_COST,
    ONLY_GO_STRAIGHT
} DecisionMakingPolicy;

typedef enum
{
    GENERAL,
    PERSONAL
} CostFunction;

/**
 * @brief Simple implementation of general cost function here which considers the path length and 
 * curvature. If you want to create your own cost function calculation, create your own namespace 
 * and then add your function type to the enum above.
 * 
 */
namespace general_cost
{
typedef struct
{
    double weight_collision_score;
    double weight_path_length;
    double weight_path_curvature;
} CostWeight;

typedef struct
{
    std::vector<double> collsion_score_vector;
    std::vector<double> path_length_vector;
    std::vector<double> path_curve_vector;
} PathScores;

double getDistance(const CoorPoint2i p1, const CoorPoint2i p2);
double getCurvature(const CoorPoint2i prev,
                    const CoorPoint2i current,
                    const CoorPoint2i next);
std::vector<double> normalizeVector(const std::vector<double> input);
std::vector<double> calculateScore(CandidatePath candidate_path);
}

class DecisionMakingModule
{
public:
    DecisionMakingModule(FrenetEnvironment *frenet_env_ptr);

    void execute(DecisionMakingPolicy decision_policy);
    
private:
    void calculateCost(CostFunction cost_function);

    FrenetEnvironment* frenet_env_ptr_ = nullptr;
    std::vector<double> cost_score_vtr_; // For storing the calculate weight of paths.
    
};

#endif // DECISIONMAKINGMODULE_H
