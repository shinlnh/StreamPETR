#include <gtest/gtest.h>
#include "fixture.h"

/**
 * @brief  TEST 1: ACController()
*/
TEST(AEBController, test_01)
{
    AEBController aeb;
}

/**
 * @brief  TEST 2: limitRange()
 * 
 * Description: 
 * 
 * Expected output: 5.0f
 * 
*/
TEST_F(AebController_Default_Case, test_02)
{
    result = 10.0f;
    expected = 5.0f;

    aeb.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 3: limitRange()
 * 
 * Description: An initial value below lower limit
 * 
 * Expected output: -5.0f
 * 
*/
TEST_F(AebController_Default_Case, test_03)
{
    result = -10.0f;
    expected = -5.0f;

    aeb.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 4: limitRange()
 * 
 * Description: An initial value within the limits
 * 
 * Expected output: 0.0f
 * 
*/
TEST_F(AebController_Default_Case, test_04)
{
    result = 0.0f;
    expected = 0.0f;

    aeb.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 5: limitRange()
 * 
 * Description: An initial value at the upper limit
 * 
 * Expected output: 5.0f
 * 
*/
TEST_F(AebController_Default_Case, test_05)
{
    result = 5.0f;
    expected = 5.0f;

    aeb.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 6: limitRange()
 * 
 * Description: An initial value at the lower limit
 * 
 * Expected output: -5.0f
 * 
*/
TEST_F(AebController_Default_Case, test_06)
{
    result = -5.0f;
    expected = -5.0f;

    aeb.limitRange(&result, 5.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 7: limitRange()
 * 
 * Description: An initial value equal to the same upper and lower limit
 * 
 * Expected output: 2.0f
 * 
*/
TEST_F(AebController_Default_Case, test_07)
{
    result = 3.0f;
    expected = 2.0f;

    aeb.limitRange(&result, 2.0f, 2.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 8: limitRange()
 * 
 * Description: An initial value within the symmetric limits
 * 
 * Expected output: 0.0f 
 * 
*/
TEST_F(AebController_Default_Case, test_08)
{
    result = 0.0f;
    expected = 0.0f;

    aeb.limitRange(&result, 1.0f, -1.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 9: limitRange()
 * 
 * Description: An initial value of floating point precision
 * 
 * Expected output: 1.0f 
 * 
*/
TEST_F(AebController_Default_Case, test_09)
{
    result = 1.000001f;
    expected = 1.0f;

    aeb.limitRange(&result, 1.0f, -1.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 10: limitRange()
 * 
 * Description: An initial value within negative limits
 * 
 * Expected output: -3.0f
 * 
*/
TEST_F(AebController_Default_Case, test_10)
{
    result = -3.0f;
    expected = -3.0f;

    aeb.limitRange(&result, -1.0f, -5.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 11: limitRange()
 * 
 * Description: Test the limitRange function with an upper limit lower than the lower limit, which should result in clamping to the lower limit.
 * 
 * Expected output: -4.0f
 * 
*/
TEST_F(AebController_Default_Case, test_11)
{
    result = -3.0f;
    expected = -4.0f;

    aeb.limitRange(&result, -4.0f, -4.0f);
    
    EXPECT_EQ(result, expected);
}

/**
 * @brief  TEST 12: warningCheck()
 * 
 * Description: Return false value of forward_collision_warning
 * 
 * Expected output: false
 *
*/
TEST_F(AebController_Check_Bool, test_12)
{
    aeb.forward_collision_warning = false;

    result = aeb.warningCheck();
    
    EXPECT_FALSE(result);
}

/**
 * @brief  TEST 13: warningCheck()
 * 
 * Description: Return true value of forward_collision_warning
 * 
 * Expected output: true
 *
*/
TEST_F(AebController_Check_Bool, test_13)
{
    aeb.forward_collision_warning = true;

    result = aeb.warningCheck();
    
    EXPECT_TRUE(result);
}

/**
 * @brief  TEST 14: execute()
 * 
 * Description: created default case
 * 
 * Expected output: ControlResults
 * 
 * steering_value: 0.0f
 * throttle_value: 0.0f
 * brake_value: 1.0f
 */

TEST_F(AebController_Default_Case, test_14)
{
    control_results = aeb.execute();
    
    EXPECT_EQ(control_results.steering_value, 0.0f);
    EXPECT_EQ(control_results.throttle_value, 0.0f);
    EXPECT_EQ(control_results.brake_value, 1.0f);
}

/**
 * @brief  TEST 15: timeToCollision()
 * 
 * Description: created true case of relative acceleration at line 21
 * 
 * Expected output: 0.0f
 *
*/
TEST_F(AebController_timeToCollision_Case, test_15)
{
    distance_to_car_ahead = 0.0f;
    leading_car_velocity = 20.0f;
    following_car_velocity = 30.0f;
    relative_acceleration = 0.0f;

    result = aeb.timeToCollision(distance_to_car_ahead, leading_car_velocity, following_car_velocity, relative_acceleration);
    
    EXPECT_EQ(result, 0.0f);
}

/**
 * @brief  TEST 16: timeToCollision()
 * 
 * Description: created false case of relative acceleration at line 24
 * 
 * Expected output: 1.0f
 *
*/
TEST_F(AebController_timeToCollision_Case, test_16)
{
    distance_to_car_ahead = 1.0f;
    leading_car_velocity = 20.0f;
    following_car_velocity = 30.0f;
    relative_acceleration = 1.0f;

    result = aeb.timeToCollision(distance_to_car_ahead, leading_car_velocity, following_car_velocity, relative_acceleration);
    
    EXPECT_LT(result, 1.0f);
}

/**
 * @brief  TEST 17: timeStop()
 * 
 * Description: created case that:
 * - following_car_velocity less than 4.17 m/s ( 15 / 3.6) at line 37
 * - leading_car_velocity less or equal than - 0.5*jerk1* jerk1_time* at line 47
 * - 
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_17)
{
    leading_car_velocity = 5.0f / 3.6f;     // [m/s]
    following_car_velocity = 10.0f / 3.6f;  // [m/s]
    expected = 0.5f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 18: timeStop()
 * 
 * Description: created case that 
 * - following_car_velocity more than 4.17 m/s ( 15 / 3.6) at line 37
 * - leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * - delta less or equal than 0 at line 54
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_18)
{
    leading_car_velocity = 23.0f / 3.6;             // [m/s]
    following_car_velocity = 10.0f / 3.6f;          // [m/s]
    expected = 2.0f;
    
    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 19: timeStop()
 * 
 * Description: created case that: 
 * - following_car_velocity less than 4.17 m/s ( 15 / 3.6) at line 37
 * - leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * - delta greater than 0 at line 56
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_19)
{
    leading_car_velocity = 15.0f / 3.6;             // [m/s]
    following_car_velocity = 10.0f / 3.6f;          // [m/s]
    expected = 2.0f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

// /*---------------------------------------------------------------------------*/

/**
 * @brief  TEST 20: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity less than 13.89 m/s ( 50 / 3.6) at line 39
 * * leading_car_velocity less or equal than - 0.5*jerk1* jerk1_time* at line 47
 * * 
 * 
 * Expected output: 
 *
 */

TEST_F(AebController_timeStop_Case, test_20)
{
    leading_car_velocity = 5.0f / 3.6f;            // [m/s]
    following_car_velocity = 49.0f / 3.6f;          // [m/s]
    expected = 0.5f;
    
    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 21: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity less than 13.89 m/s ( 50 / 3.6) at line 39
 * * leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * * delta less or equal than 0 at line 54
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_21)
{
    leading_car_velocity = 70.0f / 3.6;    // [m/s]
    following_car_velocity = 45.0f / 3.6f;  // [m/s]
    expected = 3.5f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 22: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity less than 13.89 m/s ( 50 / 3.6) at line 39
 * * leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * * delta greater or equal than 0 at line 54
 * 
 * Expected output:
 *
 */
TEST_F(AebController_timeStop_Case, test_22)
{
    leading_car_velocity = 55.0f / 3.6;           // [m/s]
    following_car_velocity = 49.0f / 3.6;  // [m/s]
    expected = 3.0f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

// /*---------------------------------------------------------------------------*/

/**
 * @brief  TEST 23: timeStop()
 * 
 * Description: created case that 
 * - following_car_velocity less than 20.83 m/s ( 75 / 3.6) at line 41
 * - leading_car_velocity less or equal than - 0.5*jerk1* jerk1_time* at line 47
 * - 
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_23)
{
    following_car_velocity = 74.0f / 3.6f;  // [m/s]
    leading_car_velocity = 25.0f / 3.6f;    // [m/s]
    expected = 0.5f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 24: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity less than 20.83 m/s ( 75 / 3.6) at line 41
 * * leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * * delta less or equal than 0 at line 54
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_24)
{
    following_car_velocity = 70.0f / 3.6f;  // [m/s]
    leading_car_velocity = 120.0f / 3.6;    // [m/s]
    expected = 4.5f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 25: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity less than 20.83 m/s ( 75 / 3.6) at line 41
 * * leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * * delta greater than 0 at line 56
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_25)
{
    following_car_velocity = 70.0f / 3.6f;  // [m/s]
    leading_car_velocity = 90.0f / 3.6;      // [m/s]
    expected = 2.5f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

// // /*---------------------------------------------------------------------------*/

/**
 * @brief  TEST 26: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity more than 20.83 m/s ( > 75 / 3.6) at line 43
 * * leading_car_velocity less or equal than - 0.5*jerk1* jerk1_time* at line 47
 * * 
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_26)
{
    following_car_velocity = 90.0f / 3.6f;  // [m/s]
    leading_car_velocity = 10.0f / 3.6f;    // [m/s]
    expected = 0.5f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 27: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity more than 20.83 m/s ( > 75 / 3.6) at line 43
 * * leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * * delta less or equal than 0 at line 54
 * 
 * Expected output: 
 *
 */
TEST_F(AebController_timeStop_Case, test_27)
{
    following_car_velocity = 90.0f / 3.6f;  // [m/s]
    leading_car_velocity = 160.0f / 3.6;   // [m/s]
    expected = 3.0f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 28: timeStop()
 * 
 * Description: created case that 
 * * following_car_velocity more than 20.83 m/s ( > 75 / 3.6) at line 43
 * * leading_car_velocity greater or equal than - 0.5*jerk1* jerk1_time* at line 50
 * * delta greater than 0 at line 56
 * 
 * Expected output: 
 *
 */

TEST_F(AebController_timeStop_Case, test_28)
{
    following_car_velocity = 90.0f / 3.6;   // [m/s]
    leading_car_velocity = 120.0f;            // [m/s]
    expected = 7.0f;

    result = aeb.timeStop(0, leading_car_velocity, following_car_velocity);
}

/**
 * @brief  TEST 29: dangerCheck()
 * 
 * Description: created true case at line 72
 * 
 * Expected output: true
 *
 */

TEST_F(AebController_dangerCheck_Case, test_29)
{
    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_TRUE(result);
}

/**
 * @brief  TEST 30: dangerCheck()
 * 
 * Description: created true case that
 * - leading_car_velocity greater or equal than following_car_velocity at line 74
 * 
 * Expected output: false
 *
 */
TEST_F(AebController_dangerCheck_Case, test_30)
{
    distance_to_car_ahead = 4.0f;           // [m]
    leading_car_velocity = 70.0f / 3.6f;    // [m/s]
    following_car_velocity = 60.0f / 3.6f;  // [m/s]

    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_FALSE(result);
}

/**
 * @brief  TEST 31: dangerCheck()
 * 
 * Description: created true case that
 * - following_car_velocity equal to 0 at line 74
 * 
 * Expected output: false
 *
 */
TEST_F(AebController_dangerCheck_Case, test_31)
{
    distance_to_car_ahead = 4.0f;           // [m]
    leading_car_velocity = -70.0f / 3.6f;   // [m/s]
    following_car_velocity = 0.0f;          // [m/s]

    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_FALSE(result);
}

/**
 * @brief  TEST 32: dangerCheck()
 * 
 * Description: created true case that
 * - is_reverse equal to true at line 74
 * 
 * Expected output: false
 *
 */
TEST_F(AebController_dangerCheck_Case, test_32)
{
    distance_to_car_ahead = 4.0f;           // [m]
    leading_car_velocity = 50.0f / 3.6f;    // [m/s]
    following_car_velocity = 60.0f / 3.6f;  // [m/s]
    is_reverse = true;

    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_FALSE(result);
}

/**
 * @brief  TEST 33: dangerCheck()
 * 
 * Description: created true case at line 81
 * 
 * 
 * Expected output: false
 *
 */

TEST_F(AebController_dangerCheck_Case, test_33)
{
    distance_to_car_ahead = 4.0f;           // [m]
    leading_car_velocity = -20.0f / 3.6f;    // [m/s]
    following_car_velocity = 60.0f / 3.6f;  // [m/s]

    EXPECT_CALL(mock_aeb, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = -5.0f; });

    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_FALSE(result);
}

/**
 * @brief  TEST 34: dangerCheck()
 * 
 * Description: created false case at line 85
 * 
 * 
 * Expected output: false
 *
 */
TEST_F(AebController_dangerCheck_Case, test_34)
{
    distance_to_car_ahead = 4.0f;           // [m]
    leading_car_velocity = -30.0f / 3.6f;    // [m/s]
    following_car_velocity = 60.0f / 3.6f;  // [m/s]

    EXPECT_CALL(mock_aeb, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = 5.0f; });
    EXPECT_CALL(mock_aeb, timeToCollision(_, _, _, _)).WillOnce(Return(5.0f));
    EXPECT_CALL(mock_aeb, timeStop(_, _, _)).WillOnce(Return(1.0f));

    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_FALSE(result);
}

/**
 * @brief  TEST 35: dangerCheck()
 * 
 * Description: created true case at line 91
 * 
 * 
 * Expected output: true
 *
 */
TEST_F(AebController_dangerCheck_Case, test_35)
{
    distance_to_car_ahead = 4.0f;           // [m]
    leading_car_velocity = -50.0f / 3.6f;    // [m/s]
    following_car_velocity = 60.0f / 3.6f;  // [m/s]

    EXPECT_CALL(mock_aeb, limitRange(_, _, _)).WillOnce([&](float *x, float, float) { *x = -1.0f; });
    EXPECT_CALL(mock_aeb, timeToCollision(_, _, _, _)).WillOnce(Return(1.0f));
    EXPECT_CALL(mock_aeb, timeStop(_, _, _)).WillOnce(Return(5.0f));

    result = mock_aeb.dangerCheck(distance_to_car_ahead, leading_car_velocity, following_car_velocity, longitudinal_acceleration, is_reverse);
    EXPECT_TRUE(result);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}