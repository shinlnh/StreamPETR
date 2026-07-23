#include "statevehicle.h"
#include <gtest/gtest.h>
#include <math.h>

/**
 * @brief TEST 1: StateVehicle()
 * 
 */
TEST(test_statevehicle, StateVehicle_Contructor)
{
    StateVehicle statevehicle();
}

/**
 * @brief TEST 2: ~StateVehicle()
 * 
 */
TEST(test_statevehicle, StateVehicle_Destructor)
{
    StateVehicle* statevehicle = new StateVehicle();
    
    delete statevehicle;
}

/**
 * @brief TEST 3: GetState()
 * 
 *  Expected output: 0.0, 0.0, 0.0, 0.0
 */
TEST(test_statevehicle, GetState)
{
    StateVehicle statevehicle(0.0f, 0.0f, 0.0f, 0.0f);
    
    EXPECT_FLOAT_EQ(statevehicle.GetState(0), 0.0f);
    EXPECT_FLOAT_EQ(statevehicle.GetState(1), 0.0f);
    EXPECT_FLOAT_EQ(statevehicle.GetState(2), 0.0f);
    EXPECT_FLOAT_EQ(statevehicle.GetState(3), 0.0f);
}

/**
 * @brief TEST 4: SetState()
 * 
 *  Expected output: 1.0, 2.0, 3.0, 4.0
 */
TEST(test_statevehicle, SetState)
{
    StateVehicle statevehicle(0.0f, 0.0f, 0.0f, 0.0f);

    statevehicle.SetState(0, 1.0f);
    statevehicle.SetState(1, 2.0f);
    statevehicle.SetState(2, 3.0f);
    statevehicle.SetState(3, 4.0f);
    statevehicle.SetState(4, 5.0f);
    
    float expected_ouput[5] = {1.0f, 2.0f, 3.0f, 4.0f, nanf("")};

    EXPECT_FLOAT_EQ(statevehicle.GetState(0), expected_ouput[0]);
    EXPECT_FLOAT_EQ(statevehicle.GetState(1), expected_ouput[1]);
    EXPECT_FLOAT_EQ(statevehicle.GetState(2), expected_ouput[2]);
    EXPECT_FLOAT_EQ(statevehicle.GetState(3), expected_ouput[3]);
    ASSERT_TRUE(std::isnan(statevehicle.GetState(4)));
}

/**
 * @brief TEST 5: GetCarLength()
 * 
 *  Expected output: 0.29
 */
TEST(test_statevehicle, GetCarLength)
{
    StateVehicle statevehicle(0.0f, 0.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(statevehicle.GetCarLength(), 0.29f);
}

/**
 * @brief TEST 6: SaturationAngle()
 * 
 *  Expected output: 1.0, -1.0
 */
TEST(test_statevehicle, SaturationAngle)
{
    StateVehicle statevehicle;

    float angle = 0.0f;

    statevehicle.SaturationAngle(angle, 1.0f, 5.0f);
    EXPECT_FLOAT_EQ(angle, 1.0f);

    statevehicle.SaturationAngle(angle, 0.0f, -1.0f);
    EXPECT_FLOAT_EQ(angle, -1.0f);

}

/**
 * @brief TEST 7: SaturationAngle()
 * 
 *  Expected output: 3.5, -3.5
 */
TEST(test_statevehicle, NormalizeAngle)
{
    StateVehicle statevehicle;

    float pi = M_PI;
    float angle = 3.5f;

    statevehicle.NormalizeAngle(angle);
    
    angle = -3.5f;
    statevehicle.NormalizeAngle(angle);
}

/**
 * @brief TEST 8: UpdateState()
 * 
 */
TEST(test_statevehicle, UpdateState)
{
    StateVehicle statevehicle(0.0f, 0.0f, 0.0f, 1.0f);;

    statevehicle.UpdateState(0.0f, 0.0f);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}