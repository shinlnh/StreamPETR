#ifndef RECEIVERUDP_HPP_
#define RECEIVERUDP_HPP_

#include <limits.h> // for PATH_MAX
#include <unistd.h>
#include <fstream>
#include "common.h"

#ifndef UT_TEST
#include "Receiver.hpp"

#else 
#include "lib4test.h" 
#endif

class ReceiverUdp : public Receiver
{
public:
    long long timeStamp;
    GstElement *depayloader;
    GstPad *src_pad;
    ReceiverUdp(unsigned int width, unsigned int height);
    ~ReceiverUdp();
    int getPort();
    cv::Mat getNextFrame() override;

};

inline int ReceiverUdp::getPort()
{
    INFO("Confirm unique port: %d", 3445);
    return 3445;
}
#endif // RECEIVERUDP_HPP_
