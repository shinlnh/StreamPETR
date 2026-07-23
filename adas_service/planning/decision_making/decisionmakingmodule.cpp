#include "decisionmakingmodule.h"

#include "common.h"

namespace general_cost {
/**
 * Calculate the distance between 2 points. (Euclidean distance)
 * @param p1 point1
 * @param p2 point2
 * @return The Euclidean distance between 2 points.
 */
double getDistance(const CoorPoint2i p1, const CoorPoint2i p2)
{
    int dx = p2.x - p1.x;
    int dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * Estimate a curvature from 3 points.
 * @param prev Point
 * @param current Point
 * @param next Point
 * @return Absolute value of the estimated curvature between 3 points prev, current, next.
 */
double getCurvature(const CoorPoint2i prev, const CoorPoint2i current, const CoorPoint2i next)
{
    double dx1 = current.x - prev.x;
    double dy1 = current.y - prev.y;
    double dx2 = next.x - current.x;
    double dy2 = next.y - current.y;

    double x_prime = (dx1 + dx2) / 2.0; // Approximate first derivative
    double y_prime = (dy1 + dy2) / 2.0;

    double x_double_prime = dx2 - dx1; // Approximate second derivative
    double y_double_prime = dy2 - dy1;

    double denominator = std::pow(x_prime * x_prime + y_prime * y_prime, 1.5);
    double signed_curvature = (x_prime * y_double_prime - y_prime * x_double_prime) / denominator;
    return std::abs(signed_curvature);
}

/**
 * Normalize every element into the range [0,100].
 * @param input The vector you want to normalize.
 * @return The normalized vector.
 */
std::vector<double> normalizeVector(const std::vector<double> input)
{
    if (input.empty())
    {
        WARN("Empty input vector to be normalize!");
        return input;
    }

    double max_val = *std::max_element(input.begin(), input.end());

    // Calculate the scaling factors, defaults to 1.
    double scale_factor = (max_val > 0) ? 100.0 / max_val : 1.0;

    // Create a new vector to store the normalized values.
    std::vector<double> normalized_values;

    // Normalize the values, range [0, 100].
    for (double val : input)
    {
        double normalized_val = val * scale_factor;
        normalized_values.push_back(normalized_val);
    }

    return normalized_values;

    /*!
     * OLD CODE: The range is calculated from the lowest value, not 0
     * -> Exaggerate the difference, could be modified to use as a second option
     *
    double min_val = *std::min_element(input.begin(), input.end());
    double max_val = *std::max_element(input.begin(), input.end());

    // Calculate the scaling factors
    double range = max_val - min_val;
    double scale_factor = (range > 0) ? 100.0 / range : 1.0;

    // Create a new vector to store the normalized values
    std::vector<double> normalized_values;

    // Normalize the values and scale them to the range [0, 100]
    for (double val : input) {
        double normalized_val = (val - min_val) * scale_factor;
        normalized_values.push_back(normalized_val);
    }
    *
    */
}

/**
 * Takes in a candidate path, calculate the scores based on its collision check score, total length 
 * and total curvature.
 * 
 * @param candidate_path The input candidate path.
 * @return The score vector {collsion score, path_length, path_curvature}.
 */
std::vector<double> calculateScore(CandidatePath candidate_path)
{
    double collision_check = (double)candidate_path.collision_check_score_;
    double total_length = 0;
    double tocal_curvature = 0;

    // Get total length
    std::vector<CoorPoint2i> body_points = *(candidate_path.getBodyPointsAddr());
    body_points.emplace_back(candidate_path.getTail());

    for (size_t i = 1; i < body_points.size(); i++)
    {
        total_length += getDistance(body_points[i - 1], body_points[i]);
    }
    // Get total curvature
    for (size_t i = 1; i < body_points.size() - 1; i++)
    {
        tocal_curvature += getCurvature(body_points[i - 1], body_points[i], body_points[i + 1]);
    }

    std::vector<double> return_vector = {collision_check, total_length, tocal_curvature};
    return return_vector;
}
}

DecisionMakingModule::DecisionMakingModule(FrenetEnvironment *frenet_env_ptr)
{
    if (frenet_env_ptr != nullptr) {
        frenet_env_ptr_ = frenet_env_ptr;
    }
}

/**
 * This function is executed if the policy choice is the LOWEST_COST.
 * Takes in the cost function selected and return the desired output.
 */
void DecisionMakingModule::calculateCost(CostFunction cost_function)
{
    // Simple checks
    if (frenet_env_ptr_->getCandidatePath().size() == 0)
    {
        WARN("Candidate vector not found");
        return;
    }

    switch (cost_function) {
        case GENERAL: {
            using namespace general_cost;

            PathScores path_scores; // Stores the score / cost of every candidate paths.

            std::vector<CandidatePath> candidate_vector = frenet_env_ptr_->getCandidatePath();

            path_scores.collsion_score_vector.reserve(candidate_vector.size());
            path_scores.path_length_vector.reserve(candidate_vector.size());
            path_scores.path_curve_vector.reserve(candidate_vector.size());

            for (size_t i = 0; i < candidate_vector.size(); i++) {
                std::vector<double> single_path_score = calculateScore(candidate_vector[i]);
                path_scores.collsion_score_vector[i] = single_path_score[0];
                path_scores.path_length_vector[i] = single_path_score[1];
                path_scores.path_curve_vector[i] = single_path_score[2];
            }

            path_scores.collsion_score_vector =
                normalizeVector(path_scores.collsion_score_vector);

            path_scores.path_length_vector =
                normalizeVector(path_scores.path_length_vector);

            path_scores.path_curve_vector =
                normalizeVector(path_scores.path_curve_vector);

            size_t cost_vector_size = path_scores.collsion_score_vector.size();

            CostWeight weight_vector = {1.0 / 3, 1.0 / 3, 1.0 / 3};

            for (size_t i = 0; i < cost_vector_size; i++) {
                this->cost_score_vtr_.emplace_back(
                    weight_vector.weight_collision_score *
                        path_scores.collsion_score_vector[i] +
                    weight_vector.weight_path_length *
                        path_scores.path_length_vector[i] +
                    weight_vector.weight_path_curvature *
                        path_scores.path_curve_vector[i]);
            }
            break;
        }
        case PERSONAL: {
            // Implement your cost function calculation here.
            break;
        }
    }
}

/**
 * Central operation function. Perform action based on the policy choice, cost function and the 
 * specified weights. The logic behind is that we set the collision_check_score of the chosen path 
 * to SELECTED_PATH_INDICATOR. The planner will look at the collision_check_score of every paths 
 * and then select the one marked SELECTED_PATH_INDICATOR and convert it into the output.
 * 
 * @param
 * @return
 */
void DecisionMakingModule::execute(DecisionMakingPolicy decision_policy)
{
    switch (decision_policy) {
        case LOWEST_COLLISION_SCORE: {
            // Print the collision scores
            std::vector<CandidatePath> objectVector = frenet_env_ptr_->getCandidatePath();

            // Find the object with the smallest collision score, set score to SELECTED_PATH_INDICATOR
            std::vector<CandidatePath> tempVector = objectVector;
            auto smallestObject = std::min_element(tempVector.begin(),
                                                tempVector.end(),
                                                CandidatePath::compareScore);

            if (smallestObject != tempVector.end()) {
                smallestObject->collision_check_score_ = SELECTED_PATH_INDICATOR;
                DEBUG("A dedicated planned path's been chosen!");
                objectVector = tempVector;
            } else {
                WARN("Vector is empty.");
            }

            break;
        }
        case LOWEST_COST: {
            calculateCost(GENERAL);
            auto smallestObject = std::min_element(cost_score_vtr_.begin(),
                                                cost_score_vtr_.end());

            if (smallestObject != cost_score_vtr_.end()) {
                size_t index = std::distance(cost_score_vtr_.begin(), smallestObject);
                CandidatePath *chosenPath = &(frenet_env_ptr_->getCandidatePath())[index];
                chosenPath->collision_check_score_ = SELECTED_PATH_INDICATOR;
            } else {
                WARN("Vector is empty.");
            }

            std::vector<CandidatePath> objectVector = frenet_env_ptr_->getCandidatePath();
            for (auto elem : objectVector) {
                DEBUG("Element collision check score: %d", elem.collision_check_score_);
            }
            break;
        }
        case ONLY_GO_STRAIGHT: {
            const int straight_path_index = 0;
            std::vector<CandidatePath>& candidate_paths = frenet_env_ptr_->getCandidatePath();
            if (candidate_paths.size() > 0) {
                candidate_paths.at(straight_path_index).collision_check_score_ = SELECTED_PATH_INDICATOR;
            }
            break;
        }
    }
}
