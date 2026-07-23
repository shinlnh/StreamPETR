#include <gmock/gmock.h>
#include "gtest/gtest.h"

#define private public
#define protected public
#include "linear_kalman_filter.hpp"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

