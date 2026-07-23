#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>

#define private public
#define protected public
#include "RtspServer.hpp"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;
