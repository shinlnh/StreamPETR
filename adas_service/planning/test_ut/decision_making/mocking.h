#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "../../adas_service/planning/decision_making/decisionmakingmodule.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

