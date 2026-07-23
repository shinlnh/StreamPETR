#include <gtest/gtest.h>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

/**
 * @brief  TEST 1: TJAController()
*/
TEST(TJAControllerTestt, test_01)
{
    TJAController tja;
}

/**
 * @brief  TEST 2: setInput()
 * 
 * Description: 
 * 
 * Expected output:
 * 
*/
TEST_F(TJAController_Default_Case, test_02)
{
    tja.setInput(planning_results, configs);
}

// /**
//  * @brief  TEST 3: execute()
//  * 
//  * Description: created default case that
//  * - Xe minh chạy với tốc độ 11km/h và xe phía trước chạy với tộc độ 10km/h
//  * - Xe phía trước cách xe mình 7m và khoảng cách giảm dần cho đến khi bằng với khoảng cách xác lập
//  * - Xác lập khoảng cách giữ ở GUI là 4m
//  * 
//  * Purpose:
//  * - Giá trị Tau >=0 để xe tiến về phía trước và giữ khoảng cách.
//  * 
//  * Expected output: ControlResults
//  * 
// */
// TEST_F(TJAController_Default_Case, test_03)
// {
//     tja.detected_vehicle = true;
//     actual_distance = 7.0f;       // Distance
//     actual_velocity = 11.0f;
//     reference_distance = 4.0f;      // From GUI
//     velocity_of_car_ahead = 10.0f;

//     tja.setDisCarHead(actual_distance);
//     tja.setVelCarHead(velocity_of_car_ahead);
//     tja.setRefDistance(reference_distance);
//     tja.resetClock();
    
//     for (double i = 0; i < episode; i+=0.001)
//     {
//         actual_velocity = actual_velocity - i;
//         result = tja.execute(planning_results, actual_velocity, configs);
        
//         if (actual_velocity <= reference_distance)
//             break;
//         EXPECT_GT(result.throttle_value, 0.0f);
//     }

//     EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
//     EXPECT_FLOAT_EQ(result.brake_value, 0.0f);
// }

// /**
//  * @brief  TEST 4: execute()
//  * 
//  * Description: created default case that
//  * - Xe minh chạy với tốc độ 11km/h và xe phía trước chạy với tộc độ 10km/h
//  * - Xe phía trước cách xe mình 7m và khoảng cách giảm dần cho đến khi bằng với khoảng cách xác lập
//  * - Xác lập khoảng cách giữ ở GUI là 4m
//  * 
//  * Purpose:
//  * - Giá trị Tau >=0 để xe tiến về phải trước và giữ khoảng cách.
//  * 
//  * Expected output: ControlResults
//  * 
// */
// TEST_F(TJAController_Default_Case, test_04)
// {
//     tja.detected_vehicle = true;
//     reference_distance = 4.0f;      // From GUI
//     actual_velocity = 11.0f;
//     actual_distance = 10.0f;
//     velocity_of_car_ahead = 10.0f;

//     tja.setDisCarHead(actual_distance);
//     tja.setVelCarHead(velocity_of_car_ahead);
//     tja.setRefDistance(reference_distance);
//     tja.resetClock();
    
//     result = tja.execute(planning_results, actual_velocity, configs);
    
//     EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
//     EXPECT_FLOAT_EQ(result.brake_value, 0.0f);
//     EXPECT_FLOAT_EQ(result.throttle_value, 1.0f);
// }

// /**
//  * @brief  TEST 5: execute()
//  * 
//  * Description: created default case that
//  * - Xe minh chạy với tốc độ 30km/h và xe phía trước chạy với tộc độ 10km/h
//  * - Xe phía trước cách xe mình 9m và khoảng cách giảm dần cho đến khi bằng với khoảng cách xác lập
//  * - Xác lập khoảng cách giữ ở GUI là 8m
//  * 
//  * Purpose:
//  * - Giá trị Tau <=0 để xe giảm tốc nếu tốc độ hiện tại quá lớn so với tốc độ xe phía trước và khoảng cách giữa giá trị xác lập khoảng cách và hiện tại.
//  * 
//  * Expected output: ControlResults
//  * 
// */
// TEST_F(TJAController_Default_Case, test_05)
// {
//     tja.detected_vehicle = true;
//     reference_distance = 8.0f;          // From GUI

//     actual_distance = 9.0f;           // Distance Vehicle
//     actual_velocity = 30.0f;            // Actual Velocity Vehicle
//     velocity_of_car_ahead = 10.0f;      // Velocity Car Ahead

//     tja.setDisCarHead(actual_distance);
//     tja.setVelCarHead(velocity_of_car_ahead);
//     tja.setRefDistance(reference_distance);
//     tja.resetClock();

//     // Vehicle before
//     // Tau <= 0
//     result = tja.execute(planning_results, actual_velocity, configs);
    
//     EXPECT_GT(result.brake_value, 0.0f);
//     EXPECT_EQ(result.steering_value, 0.0f);
//     EXPECT_EQ(result.throttle_value, 0.0f);
// }

// /**
//  * @brief  TEST 6: execute()
//  * 
//  * Description: created default case that
//  * - Xe chạy với tốc độ 10km/h và thiết lập vận tốc xác lập 35km/h
//  * - 
//  * 
//  * Purpose:
//  * - Giá trị Tau >=0 để xe tăng tốc vì tốc độ của xe hiện tại < tốc độ vận tốc xác lập.
//  * 
//  * Expected output: ControlResults
//  * 
// */
// TEST_F(TJAController_Default_Case, test_06)
// {
//     tja.detected_vehicle = false;
//     actual_velocity = 10.0f;            // Actual Speed
//     reference_velocity = 35.0f;         // From GUI

//     tja.setRefVel(reference_velocity);
//     tja.resetClock();

//     // No vehicle before
//     // Tau >= 0
//     result = tja.execute(planning_results, actual_velocity, configs);
    
//     EXPECT_FLOAT_EQ(result.steering_value, 0.0f);
//     EXPECT_FLOAT_EQ(result.brake_value, 0.0f);
//     EXPECT_FLOAT_EQ(result.throttle_value, 1.0f);
// }

// /**
//  * @brief  TEST 7: execute()
//  * 
//  * Description: created default case that
//  * - Xe chạy với tốc độ 40km/h và thiết lập vận tốc tham chiếu 35km/h
//  * - Không có xe phía trước
//  * 
//  * Purpose:
//  * - Giá trị Giảm dần về 0 để giảm vận tốc của xe hiện tại về gần với vận tốc tham chiếu 35km/h
//  * 
//  * Expected output: ControlResults
//  * 
// */
// TEST_F(TJAController_Default_Case, test_07)
// {
//     tja.detected_vehicle = false;
//     actual_velocity = 10.0f;          // Speed
//     reference_velocity = 35.0f;         // From GUI

//     tja.setRefVel(reference_velocity);
//     tja.resetClock();
    
//     // No vehicle before
//     // Tau >= 0
//     result = tja.execute(planning_results, actual_velocity, configs);

//     // Decrease Tau to 0
//     actual_velocity = 40.0f; 
//     for (double i = 0; i < episode; i+=0.05)
//     {
//         result = tja.execute(planning_results, actual_velocity, configs);
        
//         if (result.throttle_value <= 0.0f || result.brake_value > 0.0f)
//             break;
//         EXPECT_GT(result.throttle_value, 0.0f);
//         EXPECT_LT(result.throttle_value, 1.0f);
//     }
// }

// /**
//  * @brief  TEST 8: execute()
//  * 
//  * Description: created default case that
//  * - Ban đầu xe minh chạy với tốc độ 30km/h và không có xe phía trước
//  * - Xác lập khoảng cách giữ ở GUI là 8m
//  * - Sau đó thì phát hiện xe phía trước chạy với tốc độ 10km/h và cách xe 10m
//  * - Lúc này vận tốc hiện tại của xe là 20km/h và giữ khoảng cách với xe phía trước
//  * - 
//  * 
//  * Purpose: 
//  * - Giá trị Tau giảm dần sau khi xe phía trước xuất hiện
//  * 
//  * Expected output: ControlResults
//  * 
// */
// TEST_F(TJAController_Default_Case, test_08)
// {
//     tja.detected_vehicle = false;
//     reference_distance = 8.0f;          // From GUI
//     reference_velocity = 35.0f;         // From GUI
//     velocity_of_car_ahead = 10.0f;
//     actual_velocity = 30.0f;            // Speed

   
//     tja.setVelCarHead(velocity_of_car_ahead);
//     tja.setRefDistance(reference_distance);
//     tja.setRefVel(reference_velocity);
//     tja.resetClock();
    
//     // No vehicle before
//     // Tau >= 0
//     result = tja.execute(planning_results, actual_velocity, configs);
//     EXPECT_EQ(result.throttle_value, 1.0f);

//     // Occurs vehicle before
//     tja.detected_vehicle = true;
//     tja.setDisCarHead(10.0f);  // Distance
//     result = tja.execute(planning_results, actual_velocity, configs);
//     EXPECT_LT(result.throttle_value, 1.0f);

//     // Decrease tau to keep distance and speed
//     result = tja.execute(planning_results, actual_velocity, configs);
//     EXPECT_LT(result.throttle_value, 1.0f);
    
//     // Decrease tau to keep distance and speed
//     tja.setDisCarHead(8.0f);   // Distance
//     result = tja.execute(planning_results, actual_velocity, configs);
//     EXPECT_LT(result.throttle_value, 1.0f);

// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}