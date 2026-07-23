#ifndef FRENET_ENVIRONMENT_ENTITIES_H
#define FRENET_ENVIRONMENT_ENTITIES_H

#include <vector>
#include <string>
#include <iostream>

/**
 * @brief Definition lane line type
 *
 */
typedef int LaneLineIndex;

/**
 * @brief LaneLineType: Support lane line type for grading purpose
 */
typedef enum {
    LANE_LINE_CROSSABLE,
    LANE_LINE_NON_CROSSABLE,
    LANE_LINE_EDGE,
    LANE_LINE_UNSPECIFIED
} LaneLineType;

typedef enum {
    NORMAL,
    DASHED,
    DOTTED
} BreaklineAttributes;

/**
 * @brief Represent a int 2D-point in the path planning enviroment
 */
struct CoorPoint2i
{
public:
    CoorPoint2i() : x(0), y(0) {};
    CoorPoint2i(int x, int y) : x(x), y(y) {};

    friend auto operator<<(std::ostream &os, CoorPoint2i const& point) -> std::ostream& {
        return os << "(" << point.x << ", " << point.y << ")";
    }
    static bool compareByY(const CoorPoint2i &p1, const CoorPoint2i &p2) {
        return (p1.y < p2.y);
    }

    int x;
    int y;
};

// Class define each lane line
class LaneLine
{
private:
    LaneLineType default_lane_line_ = LANE_LINE_CROSSABLE;

public:
    LaneLineIndex index_;
    CoorPoint2i start_point_;
    CoorPoint2i end_point_;
    LaneLineType line_type_;

    LaneLine();
    LaneLine(LaneLineIndex index, CoorPoint2i start_point, CoorPoint2i end_point) {
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
};

class EgoVehicle
{
private:
    CoorPoint2i location_;
    LaneLineIndex left_line_index_;
    LaneLineIndex right_line_index_;

    int width_;

public:
    EgoVehicle() = default;

    const CoorPoint2i getCarLocation() const;
    int getCarWidth() const;
    void setCarLocation(int x, int y) {
        // TODO: check assign condition if need
        location_.x = x;
        location_.y = y;
    };
    void setCarWidth(int width) {
        if (width <= 0) {
            throw std::invalid_argument("The car width (= " + std::to_string(width) + ") is not valid (must equal or greater than 0)");
            return;
        } else {
            width_ = width;
        }
    };
};

// Class define each single object around
class CollisionObject
{
public:
    CoorPoint2i coordinate_;                  // center of the object

    CollisionObject(int x_location, int y_location, int owidth, int oheight, float x_offset, float y_offset) {
        // TODO: make default assign for object
        coordinate_.x = x_location;
        coordinate_.y = y_location;
        object_width_ = owidth;
        object_height_ = oheight;
        object_x_offset_ = x_offset;
        object_y_offset_ = y_offset;
    }

    int index_;
    int object_type_;                         // from object detection label
    float object_x_offset_;                   // from linear estimate - x dimension
    float object_y_offset_;                   // from bird's eye view - y dimension
    int object_width_;                        // for avoid collision checking
    int object_height_;
    std::vector<LaneLineIndex> inside_lanes_;  // which lane is contain the object
};

// Define class for each candidate Path
class CandidatePath
{
private:
    std::vector<CoorPoint2i> body_points_;
    std::vector<int> collision_object_index_;
    std::vector<int> collision_lane_index_;

public:
    CoorPoint2i tail_;

    CandidatePath() = default;
    CandidatePath(int index, CoorPoint2i tail_coor);

    void addSingleBodyPoint(CoorPoint2i a_point);
    void insertBodyPointAt(std::vector<CoorPoint2i>::iterator it, CoorPoint2i a_point);
    void addCollisionObjectIndex(int index);
    void addCollisionLaneLineIndex(int line_index, LaneLineType lane_line);
    std::vector<CoorPoint2i> *getBodyPointsAddr();
    int getNumberOfCollisionObject();
    std::vector<int> getCollisionObjectList();
    CoorPoint2i getTail() const;
    static bool compareScore(const CandidatePath &a, const CandidatePath &b);

    int index_;
    int collision_check_score_ = 0;
};

#endif
