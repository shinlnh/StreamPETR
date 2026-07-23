#ifndef STUB_FRENT_ENVIRONMENT_H
#define STUB_FRENT_ENVIRONMENT_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "stub/stub_frenet_environment_entities.h"

#define LANE_WIDTH 3.0f         /**< @brief The width of each lanes (in meters), tune this to match the real world dimensions for better results. */
#define SCALE 100               /**< @brief Ratio between pixels and meters. Currently 100 pixels correlates to 1 meter. */
#define TOPDOWN_RANGE_Y 40.0f   /**< @brief How far can the top down map see forward (in meters) */
#define DEFAULT_ENVIRONMENT_Y_DIMENSION (TOPDOWN_RANGE_Y * SCALE)   /**< @brief Top down map Y dimension (in pixels) */
#define DEFAULT_MAX_DISTANCE_TO_NEAREST 100 * SCALE

class FrenetEnvironment
{
public:
    //> Functions
    FrenetEnvironment() {
        updateEnvLanes(num_lanes_ + 1);
        updateEnvEgoVehicle(car_x_position_, 10);
    };

    // Getters
    CoorPoint2i getDefaultEnvDimension() const;
    int getPlanningLaneOffset() const { return planning_lane_offset_; }
    const EgoVehicle* getEgoVehiclePtr() {
        return &(ego_vehicle_);
    };
    std::vector<LaneLine> *getLanesStructAddr() {
        return &(lanes_);
    };
    std::vector<CollisionObject> *getSurroundVehicleAddr() {
        return &(surrounding_objects_);
    };
    std::vector<CandidatePath>& getCandidatePath() {
        return candidate_paths_;
    };

    //> Data Members
    /**
     * @brief Frenet environment objects. Units: meters. Theses objects are very compilcated and 
     * these representations are not as clear as possible because of time constraints. A Frenet object
     * is represented by a vector of 5 elements {x, y , s, d, index}
     * x and y are differential values in discrete steps taken on the real world lane curves
     * s and d represents a distance along the path and the displacement to it respectively.
     * index is only used on the right lane to search for the corresponding point on the left lane
     */
    std::vector<std::vector<float>> frenet_left_lane_; 
    std::vector<std::vector<float>> frenet_right_lane_; // 2 curves (left and right) of current lane inside Frenet space
    std::vector<std::vector<std::vector<float>>> frenet_objects_; // Objects in Frenet space
    float frenet_standardized_distance_ = 0.0f; //distance from right line to ref left line

private:
    //> Functions
    // Entities
    void updateEnvLanes(int number_lanes_lines) {
        lanes_.clear();
        lanes_.reserve(number_lanes_lines);
    };
    void updateEnvEgoVehicle(int location, int width) {
        ego_vehicle_.setCarLocation(location, 0);
        ego_vehicle_.setCarWidth(width);
    };
    //> Data Members
    // Environment Attributes
    int image_width_ = 0;
    int image_height_ = 0;
    bool is_image_dimension_setup_ = false;
    int lane_lines_count_ = 4;
    float average_lane_postition_ = 400;
    int default_x_min_ = 0;
    int default_y_min_ = 0;
    int num_lanes_ = 3;
    int top_down_range_x_ = num_lanes_ * LANE_WIDTH;                          /**< @brief Horizontal view size (in meters) */
    int default_x_env_dimension_ = num_lanes_ * LANE_WIDTH * SCALE;           /**< @brief Top down map X dimension (in pixels) */
    int planning_lane_offset_ = ((num_lanes_ - 1) / 2) * LANE_WIDTH * SCALE;  /**< @brief The width of all the lanes to the left of us (in pixels) */
    int car_x_position_ = planning_lane_offset_ + LANE_WIDTH * SCALE / 2;     /**< @brief Our car X location on planning map (in pixels) */
    int default_y_max_ = DEFAULT_ENVIRONMENT_Y_DIMENSION;
    int smallest_distance_inside_lane_ = DEFAULT_MAX_DISTANCE_TO_NEAREST; // set to-be the largest, cuz each update will cause this number to decrease

    // Environment Entities
    EgoVehicle ego_vehicle_;   // The vehicle that is being controlled.
    std::vector<LaneLine> lanes_;  // The lanes.
    std::vector<CollisionObject> surrounding_objects_; // Objects around the driver's car, mostly other cars.
    std::vector<CandidatePath> candidate_paths_;    // Candidate paths.
};

#endif
