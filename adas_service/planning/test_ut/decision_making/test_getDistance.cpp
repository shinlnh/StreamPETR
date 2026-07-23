#include "fixture.h"
#include "mocking.h"

/**
 * @brief  TEST 1: GetDistance_CalculatesCorrectly
 * Description: Test for getDistance() method to verify the distance calculation between two points.
 * Expected output: The distance between (0, 0) and (3, 4) should be 5.0
 * distance: 5.0
*/
TEST(DecisionMakingModuleTest, GetDistance_CalculatesCorrectly)
{
    using namespace general_cost;
    CoorPoint2i p1( 0, 0);
    CoorPoint2i p2( 3, 4);
    double result = getDistance(p1,p2);
    EXPECT_EQ(result,5.0);
}
