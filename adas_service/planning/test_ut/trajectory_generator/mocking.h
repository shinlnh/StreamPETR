#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "trajectory_generator.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

class MockTrajectory_generator : public TrajectoryGenerator
{
    public:
        // MOCK_METHOD(std::vector<CoorPoint2i>, checkPathCollisionSpecificCar, ( (CandidatePath) &cpath , (CarRectFrom4CoorPoint2i) &specific_car ), (override));
};