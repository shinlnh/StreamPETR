#include "frenet_environment.hpp"

#include <iostream>
#include <sstream>
#include <random>
#include <chrono>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include "common.h"
#include "general_maths.h"

#define NUM_SAMPLES 160

FrenetEnvironment::FrenetEnvironment()
{
    updateEnvLanes(num_lanes_ + 1);
    updateEnvEgoVehicle(car_x_position_, 10);
}

void FrenetEnvironment::updateEnvironment(GeneralEnvironmentUpdate& general_environment_update, 
                                          std::vector<float> leftCoeffs, std::vector<float> rightCoeffs, 
                                          std::vector<std::vector<cv::Point2f>> objectsPoints, 
                                          std::vector<FusionObject> objects)
{
    //> Dynamic Environment Update
    image_width_ = general_environment_update.image_width;
    image_height_ = general_environment_update.image_height;
    is_image_dimension_setup_ = general_environment_update.is_image_dimension_setup;
    lane_lines_count_ = general_environment_update.lane_lines_count;
    average_lane_postition_ = general_environment_update.average_lane_postition;
    calculateDynamicVariables(general_environment_update.lane_lines_count);

    updateEnvObjects(objects);

    //> Frenet Environment Entities Update
    convertEntitiesToFrenetSpace(leftCoeffs, rightCoeffs, objectsPoints);
}

CoorPoint2i FrenetEnvironment::getDefaultEnvDimension() const
{
    return CoorPoint2i(default_x_env_dimension_, default_y_max_);
}

std::vector<LaneLine>* FrenetEnvironment::getLanesStructAddr()
{
   return &(lanes_);
}

std::vector<CollisionObject>* FrenetEnvironment::getSurroundVehicleAddr()
{
   return &(surrounding_objects_);
}

const EgoVehicle* FrenetEnvironment::getEgoVehiclePtr()
{
    return &(ego_vehicle_);
}

std::vector<CandidatePath>& FrenetEnvironment::getCandidatePath()
{
    return candidate_paths_;
}

void FrenetEnvironment::updateEnvLanes(int number_lanes_lines)
{
    lanes_.clear();
    lanes_.reserve(number_lanes_lines);

    if(number_lanes_lines <= 1)
        return;

    int distance_between_lane = default_x_env_dimension_ / (number_lanes_lines - 1);

    for (int line = 0; line < number_lanes_lines; line++) {
        int x_coordinate = line * distance_between_lane;
        //start point lane will have y = 0
        CoorPoint2i line_start_point = CoorPoint2i(x_coordinate, default_y_min_);
        CoorPoint2i line_end_point = CoorPoint2i(x_coordinate, default_y_max_);

        LaneLine this_lane_line = LaneLine(line, line_start_point, line_end_point);
        //first line and end line always be LINE TYPE EDGE
        if(line == 0 || line == number_lanes_lines - 1) {
            // All lanes are crossable by default. We will reassign at the end.
            this_lane_line.line_type_ = LANE_LINE_CROSSABLE;
        }
        lanes_.emplace_back(this_lane_line);
    }

    // Assumming the 2 outermost lanes are edge lanes. This should be changed as 
    // you install better lane detection model.
    lanes_[0].line_type_ = LANE_LINE_EDGE;
    lanes_[lanes_.size() - 1].line_type_ = LANE_LINE_EDGE;
}

/**
 * @brief Set the new value of the both left and right line in frenet space.
 * 
 * This is an overload function for quick API to add both left and right line in 1-line of code.
 * 
 * @param map_line: the new value to assign to the reference left line attribute.
 * @param right_line: the new value to assign to the frenet right line, must calc based on the left line.
*/
void FrenetEnvironment::updateCalculatedFrenetLanes(std::vector<std::vector<float>> map_line, 
                                                    std::vector<std::vector<float>> right_line)
{
    if (map_line.size() != 0) {
        frenet_left_lane_ = map_line;
    } else {
        DEBUG("Can not assign an empty frenet line to class attribute");
    }

    if (right_line.size() != 0) {
        frenet_right_lane_ = right_line;
        const int distance_index = 3; // x, y, s, d, i. We're getting d (distance) so the index is 3

        // The assumption here is that the lane width in Frenet space is constant.
        // This isn't true, but since we are mostly working at a specific distance, it's a reasonable assumption.
        // After experimenting, we have decided to take the distance of the first point in the right Frenet line 
        // (highest point on the screen) as the lane width.
        frenet_standardized_distance_ = frenet_right_lane_.begin()->at(distance_index);
    } else {
        DEBUG("Can not assign an empty frenet line class attribute");
    }
}

void FrenetEnvironment::updateEnvEgoVehicle(int location, int width)
{
    ego_vehicle_.setCarLocation(location, 0);
    ego_vehicle_.setCarWidth(width);
}

void FrenetEnvironment::updateEnvObjects(std::vector<FusionObject>& objects)
{
    // Each time caller call this function, it doesn't need 
    // from caller to delete objects anymore
    //reset surround object will soon moved into private section
    
    const int number_of_objects = objects.size();
    resetSurroundingVehicles();

    // Early return conditions.
    if (number_of_objects <= 0) {
        DEBUG("No objects ready to be added");
        return;
    } else if (is_image_dimension_setup_ == false) {
        //No information about the input image frame, cannot determine object dimension relative
        ERROR("Must set-up the input frame dimension for the FrenetEnvironment module first");
        return;
    }

    // All conditions passed
    DEBUG("%d objects to be added", number_of_objects);

    // Add objects to the environment one by one.
    int index_count = 0;
    for (auto &obj : objects) {
        //Extract x location and width(by percent with ref of frame) of the object
        if (image_height_ == 0 || image_width_ == 0) {
            DEBUG("Frame dimensions not set! Please check your input.");
            return;
        }

        double percent_x_left = obj.bbox.x1 / (double)(image_width_);
        double percent_x_right = obj.bbox.x2 / (double)(image_width_);
        double percent_x_middle = (percent_x_left + percent_x_right)/2;
        double percent_object_width = (percent_x_right - percent_x_left);
        if (percent_object_width < 0) 
            DEBUG("Wrong obj width! X left %lf must larger than X right %lf", percent_x_left, percent_x_right);
        
        //Extract object's height (by percent with ref of frame)
        double percent_y_top = obj.bbox.y1 / (double)(image_height_);
        double percent_y_bottom = obj.bbox.y2 /(double)(image_height_);
        double percent_object_height = (percent_y_top + percent_y_bottom)/2;
        if (percent_object_height < 0) 
            DEBUG("Wrong obj height! Y top (%lf) must smaller than Y bottom (%lf)", percent_y_top, percent_y_bottom);

        //Extract y location from bird-eye's view information
        DEBUG("Detected an object near my car:x=%lf, y=%lf", percent_object_width, obj.y_offset);

        // Add object to planing space.
        obj.is_inside_lane = addVehicle(
            car_x_position_ + obj.x_offset / top_down_range_x_ * default_x_env_dimension_, // X location (pixels)
            CAR_Y_POSITION + (obj.y_offset + CAR_LENGTH / 2.0) / TOPDOWN_RANGE_Y * DEFAULT_ENVIRONMENT_Y_DIMENSION, // Y location (pixels)
            CAR_WIDTH / top_down_range_x_ * default_x_env_dimension_, // Width (pixels)
            CAR_LENGTH / TOPDOWN_RANGE_Y * DEFAULT_ENVIRONMENT_Y_DIMENSION, // Length (pixels)
            obj.x_offset,   // X offset (meters)
            obj.y_offset,   // Y offset (meters)
            index_count);   // Object index (for bookkeeping)
        // If an object can only be seen from radar and not the camera, it's probably noises.
        if (obj.from_radar && !obj.from_camera)
            obj.is_inside_lane = false;

        ++index_count;
    }

    DEBUG("Done added %d objects to environment", number_of_objects);
}

void FrenetEnvironment::addVehicle(CollisionObject single_vehicle)
{
    int index = surrounding_objects_.size();
    single_vehicle.index_ = index;
    surrounding_objects_.push_back(single_vehicle);
}

/**
 * @brief Add vehicle to the Frenet space, units in Frenet space are measured in pixels.
 * 
 * @param x_location       X location (in pixels)
 * @param y_location       Y location (in pixels)
 * @param object_width     Width (in pixels)
 * @param object_height    Height (in pixels)
 * @param x_offset          X offset (in meters)
 * @param y_offset          Y offset (in meters)
 * @param the_id 
 */
bool FrenetEnvironment::addVehicle(int x_location, int y_location, int object_width, int object_height, 
                                   float x_offset, float y_offset, int the_id)
{
    CollisionObject new_collision_object = CollisionObject(x_location, y_location, object_width, object_height, 
                                                           x_offset, y_offset);
    new_collision_object.index_ = the_id;

    /**
     * @brief Variable: normalized_object_position
     * 
     * This variable is the normalized distance of the vehicle relative to the left lane.
     * If the vehicle is on top of the left lane edge, it should return 0.
     * If the vehicle is on top of the right lane edge, it should return 1.
     * Every location in between those 2 edges return a number from 0 -> 1.
     * If the vehicle is on the right lane is should return a number > 1, likewise on the left should return < 0.
     */
    float normalized_object_position = 0.5f;

    // Check if the vehicle is inside our current lane. Also return the normalized position in the passed in variable.
    bool is_inside_lane = isInsideLaneFrenet(the_id, normalized_object_position);
    
    // Handle special case where pure radar object gets early return
    if (!is_inside_lane && normalized_object_position == 0.5f)
        normalized_object_position += x_offset / LANE_WIDTH * frenet_standardized_distance_;
        
    // This is supposed to mark the index of the lane the vehicle is in, but currently it doesn't do anything yet.
    if (!is_inside_lane) {
        new_collision_object.inside_lanes_.push_back(2);
    }

    // If the object is inside the current lane, updates the smallest front distance, this information will be used in AEB.
    if (is_inside_lane == true) {
        if (y_location < smallest_distance_inside_lane_)
            smallest_distance_inside_lane_ = y_location;
    }

    // Add the object to the planning space after the calculations.
    addVehicle(new_collision_object);

    return is_inside_lane;
}

void FrenetEnvironment::resetSurroundingVehicles()
{
    surrounding_objects_.clear();
}

/**
 * @brief Checks if an object identified by ID is inside the lane detected in Frenet space.
 * Also returns the relative distance of the object inside of a passed in float.
 * 
 * @param id The identifier of the object.
 * @param left_line_relative Output parameter to store the relative position of the left line (map line).
 * @return True if the point is inside the lane, false otherwise.
 */
bool FrenetEnvironment::isInsideLaneFrenet(int id, float &left_line_relative)
{
#define IS_INSIDE_LANE_THRESHOLD 30 // How much does the vehicle overlap the lane (100 means the vehicle is completely in the lane)
                                    // This threshold is set as low as possible for safety but not so low it wrongly detect vehicles on other lanes.

    const int distance_index = 3;   //* [x, y, s, d, index] so 3 means d which is the distance to the reference line.
                                    //* All units are in meters except for index.
    try
    {
        // Retrieve the object from frenet_objects_ using the provided ID
        const std::vector<std::vector<float>> & object = frenet_objects_.at(id);

        //* Objects are represented as lines / a vector of [x, y, s, d, index] vectors, running from left to right.

        // Check if the retrieved object is empty
        if(object.empty()) {
            DEBUG("Empty object inside lane line check frenet");
            return false;
        }

        // Get the distance values of the leftmost and rightmost points in the object
        const float left_most_point_distance = object.front().at(distance_index);
        const float right_most_point_distance = object.back().at(distance_index);
        
        // The vehicle is determined to be inside of the lane if that vehicle overlaps the lane more than a threshold.
        double overlap = getOverlap(left_most_point_distance, right_most_point_distance, 
                                    0, frenet_standardized_distance_);

        bool is_inside_lane = (overlap / frenet_standardized_distance_) > (IS_INSIDE_LANE_THRESHOLD / 100.0f);

        if ((right_most_point_distance - left_most_point_distance) > frenet_standardized_distance_) {
            DEBUG("Object is bigger than the lane, currently it is ignored, might need to handle it in the future");
        }

        // Calculate the middle point and assign the relative position of the left line
        auto middle_point = (left_most_point_distance + right_most_point_distance) / 2;
        left_line_relative = (middle_point / frenet_standardized_distance_);   //* This is the ratio in the d axis. In %
                                                                                    // This ratio is only in the current lane and has nothing to do with the right lane.

        return is_inside_lane;
    }
    // Handle exceptions related to indexing errors
    catch(const std::out_of_range& e)
    {
        ERROR("Out of range error inside lane line check frenet: %s", e.what());
    }
    // Handle other exceptions
    catch(const std::exception& e)
    {
        ERROR("Error inside lane line check frenet: %s", e.what());
    }
    // Return false in case of exceptions
    return false;
#undef IS_INSIDE_LANE_THRESHOLD
}

/**
 * @brief Update the top down map's dynamic variables that are changed based on the lane counts.
 * 
 * @param lane_lines_count The count of lane lines, not lanes. If we have 3 lanes, we expect 4 lane lines.
 */
void FrenetEnvironment::calculateDynamicVariables(int lane_lines_count)
{
    // The number of lanes equals the lane lines (a.k.a lane edges) - 1
    num_lanes_ = lane_lines_count - 1;

    // Any results with less than 2 lane lines we consider 0 lanes.
    if (lane_lines_count < 2)
        num_lanes_ = 0;

    top_down_range_x_ = num_lanes_ * LANE_WIDTH;                          /**< @brief Horizontal view size (in meters) */
    default_x_env_dimension_ = num_lanes_ * LANE_WIDTH * SCALE;           /**< @brief Top down map X dimension (in pixels) */
    planning_lane_offset_ = ((num_lanes_ - 1) / 2) * LANE_WIDTH * SCALE;  /**< @brief The width of all the lanes to the left of us (in pixels) */

    // Handles a special case where the number of lanes is even. We will need to decide which side the vehicle is on from the perception information.
    if (num_lanes_ % 2 == 0) {
        if (average_lane_postition_ < image_width_ / 2)   // Right side
            planning_lane_offset_ = ((num_lanes_) / 2) * LANE_WIDTH * SCALE;  /**< @brief The width of all the lanes to the left of us (in pixels) */
    }

    // Handles a special case where the number of lanes is zero.
    if (num_lanes_ == 0) {
        top_down_range_x_ = LANE_WIDTH;
        default_x_env_dimension_ = LANE_WIDTH * SCALE;
        planning_lane_offset_ = 0;
    }

    car_x_position_ = planning_lane_offset_ + LANE_WIDTH * SCALE / 2;
    updateEnvLanes(num_lanes_ + 1);
    updateEnvEgoVehicle(car_x_position_, 10);
}

void FrenetEnvironment::convertEntitiesToFrenetSpace(std::vector<float> leftCoeffs, std::vector<float> rightCoeffs, 
                                                     std::vector<std::vector<cv::Point2f>> objectsPoints)
{
    if (leftCoeffs.empty() || rightCoeffs.empty()) {
        DEBUG("No lanes to be converted.");
        return;
    }
    if (leftCoeffs.size() != rightCoeffs.size()) {
        ERROR("Lanes' polynomial are of different degree!");
        return;
    }
    
    //> Chuyển đổi lane về khung Frenet
    /**
     * @brief The Frenet Conversion Process: The Frenet Space is converted from real world space. Units both meters.
     * The left lane is constructed simply by sampling the left real world lane. s is the simply the longtitudinal displacement and d is 0
     * The right lane is constructed by sampling the right real world lane. At each sampled real world point, we search for the corresponding point
     * on the left lane. The corresponding point is simply the closest point on the left lane. s of this sampled point is then assigned the same as
     * the s of the corresponding point on the left lane, and d is simply the distance to this point.
     * Object in Frenet space is constructed exactly the same as the right lane points.
     */

    /**
     * @brief These vectors hold the Frenet objects.  A Frenet object is represented by a vector of 5 elements {x, y , s, d, index}
     * x and y are differential values in discrete steps taken on the real world lane curves
     * s and d represents a distance along the path and the displacement to it respectively.
     * index is only used on the right lane to search for the corresponding point on the left lane
     */
    std::vector<std::vector<float>> temp_frenet_left_lane;
    std::vector<std::vector<float>> temp_frenet_right_lane;
    std::vector<std::vector<std::vector<float>>> temp_frenet_objects;

    // B1: Tạo map cho đường chuẩn (x, y, s, d, map_index)
    // **Only work in 8.0 meters**
    float deltaY = 0.1f;
    for(int i = 0; i < NUM_SAMPLES; i++) {
        float y = i*deltaY;
        float x = leftCoeffs[2]*y*y + leftCoeffs[1]*y + leftCoeffs[0];
        if(i == 0) {
            temp_frenet_left_lane.push_back({x, y, 0, 0, 0});
        }
        else {
            float xDot = 2*leftCoeffs[2]*y + leftCoeffs[1];
            float s = temp_frenet_left_lane[i-1][2] + sqrt(1 + xDot*xDot)*deltaY; // s = prev_s + sqrt(dx^2 + dy^2) <-- dx = xdot*dy;
            temp_frenet_left_lane.push_back({x, y, s, 0, 0});
        }
    }

    // B2: Biến đổi (x, y) lane phải sang (x, y, s, d, map_index)
    for(int i = 0; i < NUM_SAMPLES; i++) {
        float y = i*deltaY;
        float x = rightCoeffs[2]*y*y + rightCoeffs[1]*y + rightCoeffs[0];
        temp_frenet_right_lane.push_back(cartesianToFrenet({x, y}, leftCoeffs, temp_frenet_left_lane));
    }

    // B3: Biến đổi (x, y) detected object sang (x, y, s, d, map_index)
    for(int i = 0; i < objectsPoints.size(); i++) {
        std::vector<std::vector<float>> objectPointsFrenet;
        for(int j = 0; j < objectsPoints[i].size(); j++) {
            objectPointsFrenet.push_back(cartesianToFrenet({objectsPoints[i][j].x, objectsPoints[i][j].y}, leftCoeffs, temp_frenet_left_lane));
        }
        temp_frenet_objects.push_back(objectPointsFrenet);
    }
    updateCalculatedFrenetLanes(temp_frenet_left_lane, temp_frenet_right_lane);
    frenet_objects_ = temp_frenet_objects;
}

/**
 * @brief  Convert Cartesian coordinates to Frenet coordinates. You pass in the a non-standardized Cartesian point here, 
 * that is any point that is not on the left lane. This function assumes that the reference points on the left lane are
 * already created. 
 * 
 * @param point 
 * @return std::vector<float> 
 */
std::vector<float> FrenetEnvironment::cartesianToFrenet(std::vector<float> point, std::vector<float> leftCoeffs, 
                                                        std::vector<std::vector<float>> frenet_left_lane)
{
    // Tìm điểm gần nhất trên đường chuẩn
    float closest_len = 1000000.0f;
    int closest_idx = 0;
    for(int i = 0; i < frenet_left_lane.size(); i++) {
        float x_proj = point[0] - frenet_left_lane[i][0];
        float y_proj = point[1] - frenet_left_lane[i][1];
        float distance_square = x_proj*x_proj + y_proj*y_proj;
        if(distance_square < closest_len) {
            closest_len = distance_square;
            closest_idx = i;
        }
    }

    // Tìm phép chiếu lên vector pháp tuyến
    std::vector<float> norm = {1, -(2*leftCoeffs[2]*frenet_left_lane[closest_idx][1] + leftCoeffs[1])};
    std::vector<float> delta = {point[0] - frenet_left_lane[closest_idx][0], point[1] - frenet_left_lane[closest_idx][1]};
    float proj_norm = (delta[0]*norm[0] + delta[1]*norm[1]) / (norm[0]*norm[0] + norm[1]*norm[1]);
    float proj_x = proj_norm*norm[0];
    float proj_y = proj_norm*norm[1];
    float length = sqrt(proj_x*proj_x + proj_y*proj_y);
    if(delta[0] < 0) {
        length = -length;
    }

    // Kết quả trả về (x, y, s, d, map_index)
    std::vector<float> frenet_point = {point[0], point[1], frenet_left_lane[closest_idx][2], length, static_cast<float>(closest_idx)};
    return frenet_point; 
}

