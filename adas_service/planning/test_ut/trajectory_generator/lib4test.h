#ifndef LIB4TEST_H
#define LIB4TEST_H

#include "stub/stub_frenet_environment.h"
#include "stub/stub_frenet_environment_entities.h"

std::vector<double> estimateEquationOfLine(CoorPoint2i _1st_point, CoorPoint2i _2nd_point);
bool doLineSegmentsIntersect(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i q1, CoorPoint2i q2);
CoorPoint2i findIntersection(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i q1, CoorPoint2i q2);
bool areCollinear(CoorPoint2i p1, CoorPoint2i p2, CoorPoint2i p3);

#endif
