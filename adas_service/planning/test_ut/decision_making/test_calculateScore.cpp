#include "fixture.h"
#include "mocking.h"


/**
 * @brief  TEST 1: Test_generalCos_0
 * Description: Verify the behavior of `calculateScore()` when the `CandidatePath` contains no body points.
 * Expected output:
 * - `result.size()` should be 3.
 * - `result[0]` (collision check score) should be 0.
 * - `result[1]` (total distance) should be 0.
 * - `result[2]` (curvature score) should be 0.
 */

TEST(DecisionMakingModuleTest, Test_generalCos_0)
{
    CandidatePath candidate_path;
    using namespace general_cost;
    calculateScore(candidate_path);
}
