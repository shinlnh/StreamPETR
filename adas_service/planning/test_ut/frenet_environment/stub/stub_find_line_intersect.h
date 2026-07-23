#ifndef STUB_LINE_INTERSECT_H
#define STUB_LINE_INTERSECT_H

#include <vector>
#include <cmath>

namespace line_intersect
{
    std::vector<double> findLineIntersect(double p1x, double p1y, double p2x, double p2y, double q1x, double q1y, double q2x, double q2y);
}

#endif