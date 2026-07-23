#define _USE_MATH_DEFINES
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include "trajectory_generator.h"
#include "common.h"

#define PLANNING_RANGE 10 /**< @brief How far is the path planned ahead (in meters) */

TrajectoryGenerator::TrajectoryGenerator(FrenetEnvironment *frenet_environment_ptr)
{
    if (frenet_environment_ptr != nullptr) {
        frenet_environment_ptr_ = frenet_environment_ptr;
    }
}

void TrajectoryGenerator::execute(TrajectoryGenerationPolicy trajectory_generation_policy)
{
    if (frenet_environment_ptr_ == nullptr) {
        ERROR("No Environment Found. No paths will be generated.");
        return;
    }
    
    switch (trajectory_generation_policy)
    {
    case DEFAULT_GENERATION: {
        generateCandidatePaths();
        break;
    }
    case FIFTH_ORDER: {
        // Implement the fifth order trajectory here.
        break;
    }
    default:
        break;
    }
    
    findCandidatePathsCollisionWithObjects();
    findCandidatePathsCollisionWithLanes();
    // activateDodging();

    //> Assign the generated candidate paths to the frenet environment.
    frenet_environment_ptr_->getCandidatePath() = current_candidate_paths_;
}

/**
 * @brief Main function that generates candidate paths for planning. Generated directly on the 
 * planning view. Generate simple hardcoded paths.
 * 
 */
void TrajectoryGenerator::generateCandidatePaths()
{
    current_candidate_paths_.clear();
    CoorPoint2i ego_vehicle_coordinate = frenet_environment_ptr_->getEgoVehiclePtr()->getCarLocation();

    const int number_of_candidate_path = 11; // Hardcoded and must be an odd number.
    current_candidate_paths_.reserve(number_of_candidate_path);
    int path_index = 0;

    // Add the 1st path / middle path
    CandidatePath current_candidate_path = CandidatePath(path_index++, 
                                                         CoorPoint2i(ego_vehicle_coordinate.x, PLANNING_RANGE * SCALE));
    current_candidate_path.addSingleBodyPoint(ego_vehicle_coordinate);
    current_candidate_paths_.emplace_back(current_candidate_path);

    // Maximum Angle is 65 degree (Hardcoded)
    // Pos = Right
    // Neg = Left

    int number_of_candidate_pairs = (number_of_candidate_path - 1) / 2;
    //> Generate candidate paths in pairs and add them to the candidate path list
    double delta_angle_radian = (35.0 / number_of_candidate_pairs) * M_PI / 180.0;
    for (int pair_index = 0; pair_index < number_of_candidate_pairs; pair_index++) {
        int current_x_pos = ego_vehicle_coordinate.x + tan(delta_angle_radian * (pair_index + 1)) * PLANNING_RANGE * SCALE;
        CoorPoint2i current_tail_pos = CoorPoint2i(current_x_pos, PLANNING_RANGE * SCALE);

        int current_x_neg = ego_vehicle_coordinate.x - tan(delta_angle_radian * (pair_index + 1)) * PLANNING_RANGE * SCALE;
        CoorPoint2i current_tail_neg = CoorPoint2i(current_x_neg, PLANNING_RANGE * SCALE);

        //> Add a positive path / right path to the candidate path list
        // Construct a candidate path using a tail point only.
        current_candidate_path = CandidatePath(path_index++, current_tail_pos);

        // Hardcoded control points
        int delta_x = current_candidate_path.tail_.x - ego_vehicle_coordinate.x;
        int average_x = (current_candidate_path.tail_.x + ego_vehicle_coordinate.x) / 2;
        CoorPoint2i control_point_0 = CoorPoint2i(average_x - (delta_x / 2) * 0.65, PLANNING_RANGE * SCALE * 0.1);
        CoorPoint2i control_point_1 = CoorPoint2i(average_x - (delta_x / 2) * 0.35, PLANNING_RANGE * SCALE * 0.2);
        current_candidate_path.addSingleBodyPoint(ego_vehicle_coordinate);
        current_candidate_path.addSingleBodyPoint(control_point_0);
        current_candidate_path.addSingleBodyPoint(control_point_1);
        current_candidate_paths_.emplace_back(current_candidate_path);

        //> Add a negative path / left path to the candidate path list
        // Construct a candidate path using a tail point only.
        current_candidate_path = CandidatePath(path_index++, current_tail_neg);

        // Hardcoded control points
        delta_x = current_candidate_path.tail_.x - ego_vehicle_coordinate.x;
        average_x = (current_candidate_path.tail_.x + ego_vehicle_coordinate.x) / 2;
        control_point_0 = CoorPoint2i(average_x - (delta_x / 2) * 0.65, PLANNING_RANGE * SCALE * 0.1);
        control_point_1 = CoorPoint2i(average_x - (delta_x / 2) * 0.35, PLANNING_RANGE * SCALE * 0.2);
        current_candidate_path.addSingleBodyPoint(ego_vehicle_coordinate);
        current_candidate_path.addSingleBodyPoint(control_point_0);
        current_candidate_path.addSingleBodyPoint(control_point_1);
        current_candidate_paths_.emplace_back(current_candidate_path);
    }
}

void TrajectoryGenerator::findCandidatePathsCollisionWithObjects()
{
    if (frenet_environment_ptr_->getSurroundVehicleAddr()->size() == 0)
        return;
    
    auto all_collision_object = *(frenet_environment_ptr_->getSurroundVehicleAddr());
    for (auto &cpath : current_candidate_paths_) {
        std::vector<CoorPoint2i> list_collision_with_path;
        for (auto &vehicle : all_collision_object) {
            ObjRect obj_rect = generateFourPointRect(vehicle);
            auto collision_point = checkPathCollisionSpecificCar(cpath, obj_rect);
            if (collision_point.empty()) continue;
            cpath.addCollisionObjectIndex(vehicle.index_);
            // Concat collision point
            list_collision_with_path.insert(list_collision_with_path.end(),
                                            collision_point.begin(),
                                            collision_point.end());
        }

        if (list_collision_with_path.size() > 0) {
            CoorPoint2i nearest_collision = cpath.tail_;
            for (auto point : list_collision_with_path) {
                if (point.y < nearest_collision.y) nearest_collision = point;
            }
            cpath.tail_ = nearest_collision;
        }
    }
}

void TrajectoryGenerator::activateDodging()
{
    // Find all objects that are collided, the output vector is a vector of unique
    // collided objects
    std::vector<int> collidedObjectVector;
    for (size_t i = 0; i < current_candidate_paths_.size(); i++)
    {
        std::vector<int> collisionList = current_candidate_paths_[i].getCollisionObjectList();
        for (size_t j = 0; j < collisionList.size(); j++)
        {
            int newElement = collisionList[j];
            auto it = std::find(collidedObjectVector.begin(), collidedObjectVector.end(), newElement);

            if (it == collidedObjectVector.end())
            {
                collidedObjectVector.push_back(newElement);
            }
            else
            {
                DEBUG("The element ID=%d is already in the vector.", newElement);
            }
        }
    }

    // From the list of collided objects, find a way around

    std::vector<CollisionObject> allCollisionObjectVector = *(frenet_environment_ptr_->getSurroundVehicleAddr());
    for (const auto &obj : allCollisionObjectVector)
    {
        // Get all surrounding cars, check if that car is collided.
        // obj is the handle that we will use to interact with that car
        auto it = std::find(collidedObjectVector.begin(), collidedObjectVector.end(), obj.index_);

        if (it != collidedObjectVector.end())
        {
            // Found a collided car
            // The main logic block, obj.index is the car we are working with at this level

            for (size_t i = 0; i < current_candidate_paths_.size(); i++)
            {

                std::vector<int> collisionList = current_candidate_paths_[i].getCollisionObjectList();

                // Find which candidate path has the collided object
                auto it = std::find(collisionList.begin(), collisionList.end(), obj.index_);

                if (it != collisionList.end())
                {

                    // Insert 2 planning points (the Y coordinates are sorted ascending)
                    auto it = std::lower_bound(current_candidate_paths_[i].getBodyPointsAddr()->begin(),
                                               current_candidate_paths_[i].getBodyPointsAddr()->end(),
                                               CoorPoint2i(obj.coordinate_.x + offset_,
                                                           obj.coordinate_.y - offset_),
                                               CoorPoint2i::compareByY);

                    current_candidate_paths_[i].insertBodyPointAt(it, CoorPoint2i(obj.coordinate_.x + offset_ + obj.object_width_ / 2,
                                                                            obj.coordinate_.y - offset_ - obj.object_height_ / 2));
                    it = std::lower_bound(current_candidate_paths_[i].getBodyPointsAddr()->begin(),
                                          current_candidate_paths_[i].getBodyPointsAddr()->end(),
                                          CoorPoint2i(obj.coordinate_.x + offset_,
                                                      obj.coordinate_.y + offset_),
                                          CoorPoint2i::compareByY);

                    current_candidate_paths_[i].insertBodyPointAt(it, CoorPoint2i(obj.coordinate_.x + offset_ + obj.object_width_ / 2,
                                                                            obj.coordinate_.y + offset_ + obj.object_height_ / 2));

                    // Delete the middle points of the inserting to solve the zig zag pathing
                    // Define the Y range for deletion
                    int minY = obj.coordinate_.y - offset_ - obj.object_width_ / 2;
                    int maxY = obj.coordinate_.y + offset_ + obj.object_height_ / 2;

                    // Iterate through the vector and delete elements with Y in the specified range
                    current_candidate_paths_[i].getBodyPointsAddr()->erase(
                        // consider uncomment
                        std::remove_if(current_candidate_paths_[i].getBodyPointsAddr()->begin(),
                                       current_candidate_paths_[i].getBodyPointsAddr()->end(),
                                       [minY, maxY](const CoorPoint2i &point)
                                       {
                                           return (point.y > minY && point.y < maxY);
                                       }),
                        current_candidate_paths_[i].getBodyPointsAddr()->end());
                }
            }
        }
    }
}

void TrajectoryGenerator::findCandidatePathsCollisionWithLanes()
{
    for (auto &cpath : current_candidate_paths_) {
        CoorPoint2i path_tail = cpath.tail_;
        CoorPoint2i path_start = cpath.getBodyPointsAddr()->at(0);
        auto ptr_for_all_lane = frenet_environment_ptr_->getLanesStructAddr();
        for (unsigned int lane_line = 0; lane_line < ptr_for_all_lane->size(); lane_line++) {
            LaneLine this_lane_line = ptr_for_all_lane->at(lane_line);
            CoorPoint2i end_line_coor = this_lane_line.end_point_;
            if (path_tail.x > path_start.x) {
                // Tail nằm bên phải start --> line > start , line < tail sẽ cắt qua cpath
                if (end_line_coor.x >= path_start.x && end_line_coor.x <= path_tail.x) {
                    cpath.addCollisionLaneLineIndex(this_lane_line.index_, this_lane_line.line_type_);
                }
            }
            else {// nếu không thì tail sẽ nằm bên trái start --> line > tail và line < start thì sẽ cắt qua cpath
                if (end_line_coor.x >= path_tail.x && end_line_coor.x <= path_start.x) {
                    cpath.addCollisionLaneLineIndex(this_lane_line.index_, this_lane_line.line_type_);
                }
            }
        }
    }
}

// Class private function
TrajectoryGenerator::ObjRect TrajectoryGenerator::generateFourPointRect(CollisionObject &obj)
{
    int x_left = obj.coordinate_.x - obj.object_width_ / 2;
    int x_right = obj.coordinate_.x + obj.object_width_ / 2;
    int y_top = obj.coordinate_.y + obj.object_height_ / 2;
    int y_bottom = obj.coordinate_.y - obj.object_height_ / 2;
    
    CoorPoint2i tl{x_left, y_top};
    CoorPoint2i tr{x_right, y_top};
    CoorPoint2i bl{x_left, y_bottom};
    CoorPoint2i br{x_right, y_bottom};

    return ObjRect(tl, tr, bl, br);
}

std::vector<CoorPoint2i> TrajectoryGenerator::checkTwoLineIntersect(CoorPoint2i tail_line1, CoorPoint2i head_line1, CoorPoint2i tail_line2, CoorPoint2i head_line2)
{
    std::vector<CoorPoint2i> result;
    if (doLineSegmentsIntersect(tail_line1, head_line1, tail_line2, head_line2))
    {
        CoorPoint2i intersection_point = findIntersection(tail_line1, head_line1, tail_line2, head_line2);
        result.push_back(intersection_point);
    }
    else
    {
        result.clear();
    }
    return result;
}

std::vector<CoorPoint2i> TrajectoryGenerator::checkPathCollisionSpecificCar(CandidatePath &cpath, ObjRect &specific_car)
{
    CoorPoint2i path_tail = cpath.tail_;
    CoorPoint2i path_nearest = cpath.getBodyPointsAddr()->back();

    // Check bottom side first
    auto bot_side_collision = checkTwoLineIntersect(path_tail, path_nearest, specific_car.bottom_left, specific_car.bottom_right);
    if (bot_side_collision.size() > 0) { 
        return bot_side_collision;
    }

    auto top_side_collision = checkTwoLineIntersect(path_tail, path_nearest, specific_car.top_left, specific_car.top_right);
    if (top_side_collision.size() > 0) { 
        return top_side_collision;
    }   
    
    auto left_side_collision = checkTwoLineIntersect(path_tail, path_nearest, specific_car.bottom_left, specific_car.top_left);
    if (left_side_collision.size() > 0) {
        return left_side_collision;
    }

    auto right_side_collision = checkTwoLineIntersect(path_tail, path_nearest, specific_car.bottom_right, specific_car.top_right);
    if (right_side_collision.size() > 0) {
        return right_side_collision;
    }
    return {}; //Return empty if no collision found
}

// Function to check if three points are collinear
bool TrajectoryGenerator::areCollinear(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i p3)
{
    return ((p2.y - p1.y) * (p3.x - p2.x) == (p3.y - p2.y) * (p2.x - p1.x));
}

// Function to check if two line segments intersect
bool TrajectoryGenerator::doLineSegmentsIntersect(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i q1, CoorPoint2i q2)
{
    // Check if the points of both line segments are collinear
    if (areCollinear(p1, p2, q1) || areCollinear(p1, p2, q2) || areCollinear(q1, q2, p1) || areCollinear(q1, q2, p2))
        return false;

    // Check for orientation of three points (p1, p2, q1)
    bool orient_1 = ((q1.y - p1.y) * (p2.x - p1.x)) > ((p2.y - p1.y) * (q1.x - p1.x));

    // Check for orientation of three points (p1, p2, q2)
    bool orient_2 = ((q2.y - p1.y) * (p2.x - p1.x)) > ((p2.y - p1.y) * (q2.x - p1.x));

    // Check for orientation of three points (q1, q2, p1)
    bool orient_3 = ((p1.y - q1.y) * (q2.x - q1.x)) > ((q2.y - q1.y) * (p1.x - q1.x));

    // Check for orientation of three points (q1, q2, p2)
    bool orient_4 = ((p2.y - q1.y) * (q2.x - q1.x)) > ((q2.y - q1.y) * (p2.x - q1.x));

    return (orient_1 != orient_2) && (orient_3 != orient_4);
}

// Function to find the intersection point of two line segments
CoorPoint2i TrajectoryGenerator::findIntersection(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i q1, CoorPoint2i q2)
{
    double A1 = (double)p2.y - p1.y;
    double B1 = (double)p1.x - p2.x;
    double C1 = A1 * p1.x + B1 * p1.y;

    double A2 = (double)q2.y - q1.y;
    double B2 = (double)q1.x - q2.x;
    double C2 = A2 * q1.x + B2 * q1.y;

    double determinant = A1 * B2 - A2 * B1;

    double intersectX = (C1 * B2 - C2 * B1) / determinant;
    double intersectY = (A1 * C2 - A2 * C1) / determinant;

    return {(int)intersectX, (int)intersectY};
}
