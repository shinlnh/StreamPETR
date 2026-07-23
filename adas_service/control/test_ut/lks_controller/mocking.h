#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "lks_controller.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;
