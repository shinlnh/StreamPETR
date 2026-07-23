#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "frenet_environment/frenet_environment_entities.hpp"
#include "common.h"

LaneLine::LaneLine()
{
    index_ = 0;
    start_point_ = CoorPoint2i(0, 0);
    end_point_ = CoorPoint2i(0, 0);
    line_type_ = LANE_LINE_UNSPECIFIED;
}

LaneLine::LaneLine(LaneLineIndex index, CoorPoint2i start_point, CoorPoint2i end_point)
{
    if (index < 0) {
        throw std::invalid_argument("The lane line index " + std::to_string(index) + "is not valid (must equal or greater than 0)");
    }
    else {
        index_ = index;
    }

    // TODO: refine and check assign condition (if need)
    start_point_ = start_point;
    end_point_ = end_point;
    line_type_ = default_lane_line_;
}

CollisionObject::CollisionObject(int x_location, int y_location, int owidth = 5, int oheight = 5,
                                 float x_offset = 0.0f, float y_offset = 0.0f)
{
    // TODO: make default assign for object
    coordinate_.x = x_location;
    coordinate_.y = y_location;
    object_width_ = owidth;
    object_height_ = oheight;
    object_x_offset_ = x_offset;
    object_y_offset_ = y_offset;
}

const CoorPoint2i EgoVehicle::getCarLocation() const
{
    return location_;
}

int EgoVehicle::getCarWidth() const
{
    return width_;
}

void EgoVehicle::setCarLocation(int x, int y)
{
    // TODO: check assign condition if need
    location_.x = x;
    location_.y = y;
}

void EgoVehicle::setCarWidth(int width)
{
    if (width <= 0)
    {
        throw std::invalid_argument("The car width (= " + std::to_string(width) + ") is not valid (must equal or greater than 0)");
        return;
    }
    else
    {
        width_ = width;
    }
}

CoorPoint2i CandidatePath::getTail() const
{
    return tail_;
}

CandidatePath::CandidatePath(int index, CoorPoint2i tail_coor)
{
    index_ = index;
    tail_ = tail_coor;
}

void CandidatePath::addSingleBodyPoint(CoorPoint2i a_point)
{
    body_points_.push_back(a_point);
}

void CandidatePath::insertBodyPointAt(std::vector<CoorPoint2i>::iterator it, CoorPoint2i a_point)
{
    body_points_.insert(it, a_point);
}

bool CandidatePath::compareScore(const CandidatePath &a, const CandidatePath &b)
{
    return a.collision_check_score_ < b.collision_check_score_;
}

void CandidatePath::addCollisionObjectIndex(int index)
{
    collision_object_index_.push_back(index);
    collision_check_score_ = 100;
}

void CandidatePath::addCollisionLaneLineIndex(int line_index, LaneLineType lane_line)
{
    collision_lane_index_.push_back(line_index);
    switch (lane_line)
    {
    case LANE_LINE_CROSSABLE:
        collision_check_score_ = 20;
        return;
    case LANE_LINE_EDGE:
        collision_check_score_ = 100;
        return;
    case LANE_LINE_NON_CROSSABLE:
        collision_check_score_ = 70;
        return;
    case LANE_LINE_UNSPECIFIED:
        collision_check_score_ = 0;
        return;
    }
}

std::vector<CoorPoint2i> *CandidatePath::getBodyPointsAddr()
{
    return &(body_points_);
}

int CandidatePath::getNumberOfCollisionObject()
{
    return collision_object_index_.size();
}

std::vector<int> CandidatePath::getCollisionObjectList()
{
    return collision_object_index_;
}

