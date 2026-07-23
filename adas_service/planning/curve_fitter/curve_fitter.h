#ifndef PLANNER_H
#define PLANNER_H

#ifndef UT_TEST
#include <vector>
#include "FitCurves.h"

#else
#include "lib4test.h"
#endif

class CurveFitter
{
private:
    int error;
    int coarse;
    int offset;
    std::vector<unsigned int> bestSplit(int number, int numParts);

public:
    int getError() const;
    void setError(int newError);
    int getCoarse() const;
    void setCoarse(int newCoarse);
    int getOffset() const;
    void setOffset(int newOffset);
    std::vector<Point2> getPlannedPath(std::vector<Point2> pathVector, int numSamples);
    Point2 cubicBezier(const std::vector<Point2>& controlPoints, double t);
    CurveFitter();
    CurveFitter(int error, int coarse, int offset);
    ~CurveFitter();
};

#endif