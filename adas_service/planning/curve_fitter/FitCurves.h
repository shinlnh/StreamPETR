#ifndef FITCURVES_H
#define FITCURVES_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

typedef struct Point2Struct {
    double x, y;
} Point2;

typedef Point2 *BezierCurve;

BezierCurve getControlPoints(size_t numPoints, Point2* points, double error, int coarse);

#define END_OF_LIST -6666

#ifdef __cplusplus
}
#endif

#endif // FITCURVES_H
