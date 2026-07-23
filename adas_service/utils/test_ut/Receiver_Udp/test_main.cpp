#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"
#include "mocking.h"
#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>


using ::testing::_;
using ::testing::Return;

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

