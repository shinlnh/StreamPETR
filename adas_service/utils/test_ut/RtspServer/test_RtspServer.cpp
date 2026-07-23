#include "fixture.h"

using ::testing::_;
using ::testing::Return;
/*
* @brief TEST 1: Constructor_Destructor()
* Description: 
* Input: 
* Expected output: run successfully
*
 */
TEST_F(RtspServer_Test,Constructor_1)
{
    EXPECT_NO_THROW
    ({
        RtspServer* module = new RtspServer();
        delete module;
    });
}
