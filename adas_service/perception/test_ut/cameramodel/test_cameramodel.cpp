#include <gtest/gtest.h>
#include <fstream>

#define private public
#define protected public
#include "cameramodel.h"
#undef private
#undef protected

/**
 * @brief  TEST 1: Default Constructor
*/
TEST(CameraModel, test_01)
{
    CameraModel model("test_cameramodel.cpp", "undistort_calib.txt", "camera_config.txt");
}

/**
 * @brief  TEST 2: Getter Method
*/
TEST(CameraModel, test_02)
{
    CameraModel model("test_cameramodel.cpp", "undistort_calib.txt", "camera_config.txt");
    EXPECT_EQ(model.car_width, model.getCarWidth());
}

/**
 * @brief  TEST 3: Getter Method
*/
// TEST(CameraModel, test_03)
// {
//     CameraModel model("test_cameramodel.cpp", "undistort_calib.txt", "camera_config.txt");
//     EXPECT_EQ(&model.birdview_model, model.getBirdViewModel());
// }

/**
 * @brief TEST 4: Update parameters
 */
TEST(CameraModel, test_04) 
{
    CameraModel model("test_cameramodel.cpp", "undistort_calib.txt", "camera_config.txt");
    model.calibTopdownView();
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}