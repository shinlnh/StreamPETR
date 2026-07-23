#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream> 
#include <vector>

#define private public
#define protected public
#include "ReceiverGTCP.hpp"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;

