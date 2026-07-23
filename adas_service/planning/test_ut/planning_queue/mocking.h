#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "planning_queue.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

