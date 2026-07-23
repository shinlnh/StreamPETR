#ifndef ROAD_ENVIRONMENT_DEFINE_H
#define ROAD_ENVIRONMENT_DEFINE_H

#include <vector>
#include <string>
#include <iostream>

typedef enum {
    LANE_LINE_CROSSABLE,
    LANE_LINE_NON_CROSSABLE,
    LANE_LINE_EDGE,
    LANE_LINE_UNSPECIFIED
} LaneLineType;

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

    std::vector<CoorPoint2i> *getBodyPointsAddr() { return &(body_points_); }
    CoorPoint2i getTail() const { return tail_; };
    static bool compareScore(const CandidatePath &a, const CandidatePath &b) {
        return a.collision_check_score_ < b.collision_check_score_;
    }

    int index_;
    int collision_check_score_ = 0;
};

#endif
