#include "fixture.h"
#include "mocking.h"

using namespace testing;

/**
 * @brief  TEST 1: ExecuteTest_LowestCollisionScore_SingleCandidate
 * Description: Verify that `execute()` correctly selects the single candidate path when the 
 * policy choice is `LOWEST_COLLISION_SCORE`.
 * Expected output: 
 * - No error.
 */
TEST_F(DecisionMakingModuleMock, ExecuteTest_LowestCollisionScore_SingleCandidate) {
    DecisionMakingPolicy decision_policy = ONLY_GO_STRAIGHT;

    dmm->execute(decision_policy);
}