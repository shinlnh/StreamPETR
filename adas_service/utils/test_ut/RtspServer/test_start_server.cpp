#include "fixture.h"

using ::testing::_;
using ::testing::Return;
/*
* @brief TEST 1: start_server() and stop_server() funtion
* Description: 
* Input: 
* Expected output: run successfully
*
 */
static gboolean stop_server_timeout(gpointer user_data)
{
    auto server = static_cast<RtspServer*>(user_data);
    server-> stop_server();
    return G_SOURCE_REMOVE;
}
TEST_F(RtspServer_Test,start_server_1)
{
    RtspServer server;

    g_timeout_add(100, stop_server_timeout,&server);
    server.start_server();
}
