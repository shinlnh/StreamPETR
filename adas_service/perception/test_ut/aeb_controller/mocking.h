#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "aeb_controller.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

class MockAEBController: public AEBController
{
public:
    MOCK_METHOD(void, limitRange, (float*, float, float), (override));
    MOCK_METHOD(float, timeToCollision, (float, float, float, float), (override));
    MOCK_METHOD(float, timeStop, (float, float, float), (override));
};