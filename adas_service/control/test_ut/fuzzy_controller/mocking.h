#include <gmock/gmock.h>
#include <gtest/gtest.h>

#define private public
#define protected public
#include "fuzzy_controller.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

/** Mocking */
class MockFuzzyController : public FuzzyController {
public:
    MOCK_METHOD(float, fuzzyVariableTrapezoidal, (float, float, float, float, float),(override));
    MOCK_METHOD(float, fuzzyVariableTriangle, (float, float, float, float),(override));
    MOCK_METHOD(void, limitRange, (float*, float, float),(override));
};

class MockFuzzyController_Custom : public FuzzyController {
public:
    MOCK_METHOD(float, runFuzzy, (float, float, FuzzyController::FuzzyParameters *), (override));
    MOCK_METHOD(void, limitRange, (float*, float, float),(override));
};