#ifndef RECEIVERGTCP_HPP_
#define RECEIVERGTCP_HPP_

#ifndef UT_TEST
#include "Receiver.hpp"

#else
#include "lib4test.h"
#endif

class ReceiverGTCP : public Receiver
{
public:
    ReceiverGTCP(unsigned int width, unsigned int height);
    ~ReceiverGTCP();
    cv::Mat getNextFrame() override;
};

#endif // RECEIVERGTCP_HPP_
