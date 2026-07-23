#include <fixture.h>
#include <iostream>

using ::testing::_;
using ::testing::Return;


/*
/**
 * @brief  TEST 1: setRunning() getRunning()
 * 
 * Description:
 * Input: set value by using setRunning -> get value by using getRunning
 * Expected output: value = 1
*/
TEST_F(RtspServer_Test,setRunning_1)
{
    RtspServer* server= new RtspServer;
    server->setRunning(1);
    bool result = server->getRunning();
    delete server;
    EXPECT_EQ(result,1);
}

/*
/**
 * @brief  TEST 1: setHasFrames() getHasFrames()
 * 
 * Description:
 * Input: set value by using setHasFrames -> get value by using getHasFrames
 * Expected output: value = 1
*/
TEST_F(RtspServer_Test,setHasFrames_1)
{
    RtspServer* server= new RtspServer;
    server->setHasFrames(1);
    bool result = server->getHasFrames();
    delete server;
    EXPECT_EQ(result,1);
}
