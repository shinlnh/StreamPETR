#include "stub_find_line_intersect.h"

std::vector<double> line_intersect::findLineIntersect(double p1x, double p1y, double p2x, double p2y, double q1x, double q1y, double q2x, double q2y)
{
    double A1 = (double)p2y - p1y;
    double B1 = (double)p1x - p2x;
    double C1 = A1 * p1x + B1 * p1y;

    double A2 = (double)q2y - q1y;
    double B2 = (double)q1x - q2x;
    double C2 = A2 * q1x + B2 * q1y;

    double determinant = A1 * B2 - A2 * B1;

    double intersectX = (C1 * B2 - C2 * B1) / determinant;
    double intersectY = (A1 * C2 - A2 * C1) / determinant;

    return std::vector<double>{intersectX, intersectY};
}