#include "curve_fitter.h"

#include <iostream>

CurveFitter::CurveFitter()
{
    this->error = 1000;
    this->coarse = 85;
    this->offset = 50;
}

CurveFitter::CurveFitter(int error, int coarse, int offset)
{
    this->error = error;
    this->coarse = coarse;
    this->offset = offset;
}

CurveFitter::~CurveFitter()
{

}

int CurveFitter::getError() const
{
    return offset;
}

void CurveFitter::setError(int newError)
{
    error = newError;
}

void CurveFitter::setCoarse(int newCoarse)
{
    coarse = newCoarse;
}

void CurveFitter::setOffset(int newOffset)
{
    offset = newOffset;
}

int CurveFitter::getCoarse() const
{
    return coarse;
}

int CurveFitter::getOffset() const
{
    return offset;
}

std::vector<Point2> CurveFitter::getPlannedPath(std::vector<Point2> pathVector, int numSamples)
{
    Point2* inputPath = pathVector.data();
    int length = pathVector.size();

    BezierCurve controlPoints =  getControlPoints(length, inputPath, this->error, this->coarse);

    std::vector<Point2> plannedControlPoints; // These are control points,
                                        // not to confuse with path points

    // Append control points to a vector, this will be used to draw later
    for (int i = 0; !(controlPoints[i].x == END_OF_LIST) && !(controlPoints[i].y == END_OF_LIST); i++)
    {
        plannedControlPoints.push_back({controlPoints[i].x, controlPoints[i].y});
    }

    const int totalSamples = numSamples;
    const int n = plannedControlPoints.size() / 4;
    
    const std::vector<unsigned int> sections = bestSplit(totalSamples, n);
    std::vector<Point2> sampledPath;

    for (unsigned int i = 0; i < sections.size(); i++) 
    {
        std::vector<Point2> subControlPoints(plannedControlPoints.begin() + i * 4,
                                             plannedControlPoints.begin() + (i + 1) * 4);
        for (unsigned int j = 0; j < sections[i]; ++j) {
            double t1 = (double)(j) / (double)(sections[i] - 1);
            Point2 point = cubicBezier(subControlPoints, t1);
            sampledPath.push_back(point);
        }
    }
    return sampledPath;
}

Point2 CurveFitter::cubicBezier(const std::vector<Point2>& controlPoints, double t)
{
    double oneMinusT = 1.0 - t;
    double oneMinusTSquared = oneMinusT * oneMinusT;
    double tSquared = t * t;

    double bezierPointX = oneMinusTSquared * oneMinusT * controlPoints[0].x
                          + 3 * oneMinusTSquared * t * controlPoints[1].x
                          + 3 * oneMinusT * tSquared * controlPoints[2].x
                          + tSquared * t * controlPoints[3].x;

    double bezierPointY = oneMinusTSquared * oneMinusT * controlPoints[0].y
                          + 3 * oneMinusTSquared * t * controlPoints[1].y
                          + 3 * oneMinusT * tSquared * controlPoints[2].y
                          + tSquared * t * controlPoints[3].y;

    Point2 bezierPoint = {bezierPointX, bezierPointY};
    return bezierPoint;
}

// Keep samples on each part as even as possible
std::vector<unsigned int> CurveFitter::bestSplit(int number, int numParts) { 
    int quotient = number / numParts;
    int remainder = number % numParts;

    std::vector<unsigned int> parts(numParts, quotient);

    for (int i = 0; i < remainder; ++i) {
        parts[i]++;
    }

    return parts;
}
