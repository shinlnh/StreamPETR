#include <gmock/gmock.h>
#include "gtest/gtest.h"
 
#define private public
#define protected public
#include "frenet_environment.hpp"
// #include "visualization_helper.hpp"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

class MockFrenetEnvironment : public FrenetEnvironment {
public:
    MOCK_METHOD(bool, addVehicle, (int, int, int, int, float, float, int), (override));
};