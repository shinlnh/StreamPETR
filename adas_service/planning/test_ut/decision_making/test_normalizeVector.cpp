#include "fixture.h"
#include "mocking.h"

/**
 * @brief  TEST 1: TEST_normalizeVector_1
 * Description: Test for normalizeVector() method with an empty input vector.
 * Expected output: The output should also be an empty vector.
 * output: empty vector
*/
TEST(DecisionMakingModuleTest, TEST_normalizeVector_1)
{
    using namespace general_cost;
    std::vector<double> input{};
    std::vector<double> output = normalizeVector(input);
    EXPECT_TRUE(output.empty());
}

/**
 * @brief  TEST 2: TEST_normalizeVector_2
 * Description: Test for normalizeVector() method with a vector of positive values.
 * Expected output: The vector should be normalized, scaling the values to the maximum value in the input (100%).
 * output: {33.33, 66.67, 100.0}
*/
TEST(DecisionMakingModuleTest, TEST_normalizeVector_2)
{
    using namespace general_cost;
    std::vector<double> input= {10, 20, 30};
    std::vector<double> expect= {33.33, 66.67, 100.0};
    std::vector<double> output = normalizeVector(input);
    EXPECT_EQ(expect.size(), output.size());
    for (size_t i = 0; i < expect.size(); i++) {
        EXPECT_NEAR(output[i], expect[i], 0.01); /* allow small precision error due to calculation */
    }
}

/**
 * @brief  TEST 3: TEST_normalizeVector_3
 * Description: Test for normalizeVector() method with a vector containing only zero values.
 * Expected output: The output should remain a vector of zeros.
 * output: {0, 0, 0}
*/
TEST(DecisionMakingModuleTest, TEST_normalizeVector_3)
{
    using namespace general_cost;
    std::vector<double> input= {0, 0, 0};
    std::vector<double> expect= {0, 0, 0};
    std::vector<double> output= normalizeVector(input);
    EXPECT_EQ(output, expect);
}

/**
 * @brief  TEST 4: TEST_normalizeVector_4
 * Description: Test for normalizeVector() method with a vector of negative values.
 * Expected output: The output should remain the same as the input since the values are negative and normalization typically scales the values based on the largest magnitude.
 * output: {-10, -20, -30}
*/
TEST(DecisionMakingModuleTest, TEST_normalizeVector_4)
{
    using namespace general_cost;
    std::vector<double> input= {-10, -20, -30};
    std::vector<double> expect= {-10, -20, -30};
    std::vector<double> output = normalizeVector(input);
    EXPECT_EQ(output, expect);
}
