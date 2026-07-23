#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "tjc_controller.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

// Mock class
class MockTJCController : public TJCController {
    public:
        MockTJCController(TJAController &tja_controller, LKSController &lks_controller) : TJCController(tja_controller, lks_controller){}
        MOCK_METHOD(void, limitRange, (float *, float, float), (override));
};