#include "fixture.h"
#include "mocking.h"

using namespace testing;

/**
 * @brief  TEST 1: CalculateCostTest_EmptyCandidateVectors
 * Description: Verify that `calculateCost()` handles the case when the candidate vector is empty.
 * Expected output: 
 * - The size of `candidates` should remain 0.
 */
TEST_F(DecisionMakingModuleMock, CalculateCostTest_EmptyCandidateVectors) {
    CostFunction cost_func_choice = GENERAL;
    dmm->calculateCost(cost_func_choice);
}
