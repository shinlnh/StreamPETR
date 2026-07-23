#include <gtest/gtest.h>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief TEST 1: resetSurrondVehicle()
 * 
 *  Expected output: FrenetEnvironment::around_object
 * 
 *  surrounding_objects_.size() = 0
 *  
 */
TEST(test_FrenetEnvironment, resetSurrondVehicle_001)
{
    FrenetEnvironment frenet_environment;

    CollisionObject obj1(
        150,    // x_location
        200,    // y_location
        40,     // width (4 meters)
        60,     // height (6 meters)
        0.5f,   // x_offset
        1.2f    // y_offset
    );

    CollisionObject obj2(
        300,    // x_location
        180,    // y_location
        80,     // width (8 meters)
        120,    // height (12 meters)
        -0.3f,  // x_offset
        2.1f    // y_offset
    );

    CollisionObject obj3(
        220,    // x_location
        250,    // y_location
        20,     // width (2 meters)
        20,     // height (2 meters)
        0.1f,   // x_offset
        0.5f    // y_offset
    );
    /* Set Surround Objects*/
    frenet_environment.addVehicle(obj1);
    frenet_environment.addVehicle(obj2);
    frenet_environment.addVehicle(obj3);

    /* Set Surround Vehicles*/
    frenet_environment.resetSurroundingVehicles();
    EXPECT_EQ(frenet_environment.surrounding_objects_.size(), 0);
}




