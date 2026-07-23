#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mocking.h>
#include <gst/app/gstappsink.h>

using ::testing::_;
using ::testing::Return;


class ReceiverUdpTest : public ::testing::Test {
protected:

    void SetUp() override {
        gst_init(nullptr, nullptr);
    }

    void TearDown() override {

    }
};
