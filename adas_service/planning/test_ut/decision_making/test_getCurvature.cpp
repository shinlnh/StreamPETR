#include "fixture.h"
#include "mocking.h"

/**
 * @brief  TEST 1: Test_getCurvature
 * Description: Test for getCurvature() method to verify the curvature calculation between three points.
 * Expected output: The curvature between points (0, 0), (3, 4), and (6, 8) should be 0, indicating a straight line.
 * curvature: 0
*/
TEST(DecisionMakingModuleTest, Test_getCurvature)
{
    using namespace general_cost;
    CoorPoint2i prev(0,0);
    CoorPoint2i current(3,4);
    CoorPoint2i next(6,8);
    double result = getCurvature(prev,current,next);
    EXPECT_EQ(result,0);
}
