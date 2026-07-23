#include <gtest/gtest.h>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}