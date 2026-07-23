#include <gtest/gtest.h>
#include "fixture.h"

/**
 * @brief  TEST 1: FuzzyController()
*/
TEST(FuzzyControllerTest, Contructor_FuzzyController)
{
    FuzzyController fuzzy;
}

/**
 * @brief  TEST 2: fuzzyVariableTriangle()
 * 
 * Description: created default case at line number 80
 * 
 * Expected output: 0.5
*/
TEST_F(FuzzyController_Default_Case, test_01)
{
    expected = 0.5f;

    result = fuzzy.fuzzyVariableTriangle(2.0f, 0.0f, 1.0f, 3.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 3: fuzzyVariableTriangle()
 * 
 * Description: created to solve true case of x > c at line number 78
 * 
 * Expected output: 0.0
*/
TEST_F(FuzzyController_Default_Case, test_02)
{
    expected = 0.0f;

    result = fuzzy.fuzzyVariableTriangle(1.0f, 0.0f, 0.0f, 0.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 4: fuzzyVariableTriangle()
 * 
 * Description: created to solve true case of x < a at line number 78
 * 
 * Expected output: 0.0
*/
TEST_F(FuzzyController_Default_Case, test_03)
{
    expected = 0.0f;

    result = fuzzy.fuzzyVariableTriangle(0.0f, 1.0f, 0.0f, 0.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 5: fuzzyVariableTriangle()
 * 
 * Description: created to solve true case of x < b at line number 79
 * 
 * Expected output: 0.0
*/
TEST_F(FuzzyController_Default_Case, test_04)
{
    expected = 0.0f;

    result = fuzzy.fuzzyVariableTriangle(0.0f, 0.0f, 1.0f, 0.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 6: fuzzyVariableTriangle()
 * 
 * Description: created to solve true case of x == b at line number 79
 * 
 * Expected output: 0.0
*/

TEST_F(FuzzyController_Default_Case, test_05)
{
    expected = 0.0f;

    result = fuzzy.fuzzyVariableTriangle(1.0f, 1.0f, 2.0f, 2.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 7: fuzzyVariableTrapezoidal()
 * 
 * Description: created to default case at line number 88 and return true case (c == d)
 * 
 * Expected output: 1.0
*/
TEST_F(FuzzyController_Default_Case, test_06)
{
    expected = 1.0f;  

    result = fuzzy.fuzzyVariableTrapezoidal(5.0f, 2.0f, 3.0f, 5.0f, 5.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 8: fuzzyVariableTrapezoidal()
 * 
 * Description: created to default case at line number 88 and return false case (c == d)
 * 
 * Expected output: 0.0
*/
TEST_F(FuzzyController_Default_Case, test_07)
{
    expected = 0.0f;  

    result = fuzzy.fuzzyVariableTrapezoidal(5.0f, 2.0f, 3.0f, 4.0f, 5.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 9: fuzzyVariableTrapezoidal()
 * 
 * Description: created to solve true case of x < a at line number 85
 * 
 * Expected output: 0.0
*/
TEST_F(FuzzyController_Default_Case, test_08)
{
    expected = 0.0f;

    result = fuzzy.fuzzyVariableTrapezoidal(1.0f, 2.0f, 3.0f, 4.0f, 5.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 10: fuzzyVariableTrapezoidal()
 * 
 * Description: created to solve true case of x < b | x == b and return true case of (a == b) at line number 86
 * 
 * Expected output: 1.0
*/
TEST_F(FuzzyController_Default_Case, test_09)
{
    expected = 1.0f;

    result = fuzzy.fuzzyVariableTrapezoidal(2.0f, 2.0f, 2.0f, 4.0f, 5.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 11: fuzzyVariableTrapezoidal()
 * 
 * Description: created to solve true case of x < b | x == b at and false case of (a == b) line number 86
 * 
 * Expected output: 0.5
*/
TEST_F(FuzzyController_Default_Case, test_10)
{
    expected = 0.5f;

    result = fuzzy.fuzzyVariableTrapezoidal(6.0f, 2.0f, 10.0f, 4.0f, 6.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 12: fuzzyVariableTrapezoidal()
 * 
 * Description: created to solve true case of x < c | x == c at line number 87
 * 
 * Expected output: 1.0
*/
TEST_F(FuzzyController_Default_Case, test_11)
{
    expected = 1.0f;

    result = fuzzy.fuzzyVariableTrapezoidal(4.5f, 2.0f, 3.0f, 5.0f, 5.0f);
    
    EXPECT_FLOAT_EQ(result, expected);
}

/**
 * @brief  TEST 13: limitRange()
 * 
 * Description: 
 * 
 * Expected output: 5.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_12)
{
    result = 10.0f;
    expected = 5.0f;

    fuzzy.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 14: limitRange()
 * 
 * Description: An initial value below lower limit
 * 
 * Expected output: -5.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_13)
{
    result = -10.0f;
    expected = -5.0f;

    fuzzy.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 15: limitRange()
 * 
 * Description: An initial value within the limits
 * 
 * Expected output: 0.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_14)
{
    result = 0.0f;
    expected = 0.0f;

    fuzzy.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 16: limitRange()
 * 
 * Description: An initial value at the upper limit
 * 
 * Expected output: 5.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_15)
{
    result = 5.0f;
    expected = 5.0f;

    fuzzy.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 17: limitRange()
 * 
 * Description: An initial value at the lower limit
 * 
 * Expected output: -5.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_16)
{
    result = -5.0f;
    expected = -5.0f;

    fuzzy.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 18: limitRange()
 * 
 * Description: An initial value equal to the same upper and lower limit
 * 
 * Expected output: 2.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_17)
{
    result = 3.0f;
    expected = 2.0f;

    fuzzy.limitRange(&result, 2.0f, 2.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 19: limitRange()
 * 
 * Description: An initial value within the symmetric limits
 * 
 * Expected output: 0.0f 
 * 
*/
TEST_F(FuzzyController_Default_Case, test_18)
{
    result = 0.0f;
    expected = 0.0f;

    fuzzy.limitRange(&result, 1.0f, -1.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 20: limitRange()
 * 
 * Description: An initial value of floating point precision
 * 
 * Expected output: 1.0f 
 * 
*/
TEST_F(FuzzyController_Default_Case, test_19)
{
    result = 1.000001f;
    expected = 1.0f;

    fuzzy.limitRange(&result, 1.0f, -1.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 21: limitRange()
 * 
 * Description: An initial value within negative limits
 * 
 * Expected output: -3.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_20)
{
    result = -3.0f;
    expected = -3.0f;

    fuzzy.limitRange(&result, -1.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 22: limitRange()
 * 
 * Description: Test the limitRange function with an upper limit lower than the lower limit, which should result in clamping to the lower limit.
 * 
 * Expected output: -4.0f
 * 
*/
TEST_F(FuzzyController_Default_Case, test_21)
{
    result = -3.0f;
    expected = -4.0f;

    fuzzy.limitRange(&result, -4.0f, -4.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 23: runFuzzy()
 * 
 * Description: 
 * 
 * Expected output: 0.0f
 * 
*/
TEST_F(MockFuzzyController_Default_Case, test_22)
{
    FuzzyController::FuzzyParameters fuzzy_parameters = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    ::testing::InSequence seq;

    EXPECT_CALL(mock, fuzzyVariableTrapezoidal(_, _, _, _, _)).WillOnce(Return(5.0f));

    EXPECT_CALL(mock, fuzzyVariableTriangle(_, _, _, _)).Times(3)
                                                            .WillRepeatedly(Return(5.0f));

    EXPECT_CALL(mock, fuzzyVariableTrapezoidal(_, _, _, _, _)).WillOnce(Return(5.0f));
    EXPECT_CALL(mock, fuzzyVariableTrapezoidal(_, _, _, _, _)).WillOnce(Return(5.0f));
    EXPECT_CALL(mock, fuzzyVariableTriangle(_, _, _, _)).WillOnce(Return(1.0f));
    EXPECT_CALL(mock, fuzzyVariableTrapezoidal(_, _, _, _, _)).WillOnce(Return(5.0f));
    
    result = mock.runFuzzy(0.0f, 0.0f, &fuzzy_parameters);

    EXPECT_EQ(result, expected) << "Expected: " << expected << " Actual: " << result;
}

/**
 * @brief  TEST 24: pdFuzzyController()
 * 
 * Description: created to default case
 * 
 * Expected output: 0.0f
 * 
*/
TEST_F(MockFuzzyController_Custom_Default_Case, test_23)
{
    FuzzyController::FuzzyParameters fuzzy_parameters = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    ::testing::InSequence seq;

    EXPECT_CALL(mock, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });
    EXPECT_CALL(mock, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });
    EXPECT_CALL(mock, runFuzzy(_, _, _)).WillOnce(Return(5.0f));
    EXPECT_CALL(mock, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });

    result = mock.pdFuzzyController(0.0f, 0.0f, 0.0f, &fuzzy_parameters);

    EXPECT_EQ(result, expected) << "Expected: " << expected << " Actual: " << result;
}

/**
 * @brief  TEST 25: piFuzzyController()
 * 
 * Description: 
 * 
 * Expected output: 0.0f
 * 
*/
TEST_F(MockFuzzyController_Custom_Default_Case, test_24)
{
    FuzzyController::FuzzyParameters fuzzy_parameters = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    ::testing::InSequence seq;

    EXPECT_CALL(mock, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });
    EXPECT_CALL(mock, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });
    EXPECT_CALL(mock, runFuzzy(_, _, _)).WillOnce(Return(5.0f));
    EXPECT_CALL(mock, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });

    result = mock.piFuzzyController(0.0f, 0.0f, 0.0f, &fuzzy_parameters);

    EXPECT_EQ(result, expected) << "Expected: " << expected << " Actual: " << result;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}