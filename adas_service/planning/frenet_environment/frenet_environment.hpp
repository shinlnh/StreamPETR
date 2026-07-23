#ifndef FRENET_ENVIRONMENT_HPP
#define FRENET_ENVIRONMENT_HPP

#ifndef UT_TEST_FRENET
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "frenet_environment/frenet_environment_entities.hpp"
#include "traffic_object.h"

#else
#include "lib4test.h"
#endif
/**
 * General top down environment parameters. You should only tune these parameters. 
 * Everything else is calculated according to them.
 */
#define TOPDOWN_RANGE_Y 40.0f   /**< @brief How far can the top down map see forward (in meters) */
#define LANE_WIDTH 3.0f         /**< @brief The width of each lanes (in meters), tune this to match the real world dimensions for better results. */
#define SCALE 100               /**< @brief Ratio between pixels and meters. Currently 100 pixels correlates to 1 meter. */

/**
 * Values calculated from the general parameters above.
 */
#define DEFAULT_ENVIRONMENT_Y_DIMENSION (TOPDOWN_RANGE_Y * SCALE)   /**< @brief Top down map Y dimension (in pixels) */
#define DEFAULT_MAX_DISTANCE_TO_NEAREST 100 * SCALE                 /**< @brief = 100 m. Arbitrarily large for AEB find min algo. */

#define CAR_Y_POSITION 0                                                /**< @brief Y location is at the bottom of the map. */

struct GeneralEnvironmentUpdate {
    int image_width = 0;
    int image_height = 0;
    bool is_image_dimension_setup = false;
    int lane_lines_count = 4;
    float average_lane_postition = 400;
};

/**
 * @brief The Frenet environment. Handles the conversion between the Cartesian space and the Frenet space.
 * Also has helpers for visualization purposes.
 * 
 */
class FrenetEnvironment
{
public:
    //> Functions
    FrenetEnvironment();

    void updateEnvironment(GeneralEnvironmentUpdate& general_environment_update, std::vector<float> leftCoeffs, 
                           std::vector<float> rightCoeffs, std::vector<std::vector<cv::Point2f>> objectsPoints, 
                           std::vector<FusionObject> objects);

    // Getters
    CoorPoint2i getDefaultEnvDimension() const;
    int getPlanningLaneOffset() const { return planning_lane_offset_; }
    const EgoVehicle* getEgoVehiclePtr();
    std::vector<LaneLine> *getLanesStructAddr();
    std::vector<CollisionObject> *getSurroundVehicleAddr();
    std::vector<CandidatePath>& getCandidatePath();

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
    void addVehicle(CollisionObject _single_vehicle);
#ifndef UT_TEST
    bool addVehicle(int x_location, int y_location, int object_width, int object_height, float x_offset, float y_offset, int id = 0);
#else
    virtual bool addVehicle(int x_location, int y_location, int object_width, int object_height, float x_offset, float y_offset, int id = 0);
#endif
    void resetSurroundingVehicles();
    bool isInsideLaneFrenet(int id, float& left_line_relative);
    void calculateDynamicVariables(int num_lanes);
    void convertEntitiesToFrenetSpace(std::vector<float> left_coeffs_world, 
                                      std::vector<float> right_coeffs_world, 
                                      std::vector<std::vector<cv::Point2f>> objectsPoints);
    std::vector<float> cartesianToFrenet(std::vector<float> point, std::vector<float> leftCoeffs, 
                                         std::vector<std::vector<float>> frenet_left_lane);
    // Entities
    void updateEnvLanes(int number_lanes_lines);
    void updateEnvEgoVehicle(int location, int width);
    void updateEnvObjects(std::vector<FusionObject>& objects);

    // Calculated Frenet Lanes.
    void updateCalculatedFrenetLanes(std::vector<std::vector<float>> map_line, 
                                     std::vector<std::vector<float>> right_line);

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

#endif // FRENET_ENVIRONMENT_HPP
