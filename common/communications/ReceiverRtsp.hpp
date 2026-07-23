#ifndef RECEIVERRTSP_HPP_
#define RECEIVERRTSP_HPP_

#include "Receiver.hpp"

class ReceiverRtsp : public Receiver
{
private:
    uint64_t timestamp;  // unit: ns
    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);

public:
    bool allow_grabbing = false;
    ReceiverRtsp(unsigned int width, unsigned int height, std::string server_ip);
    ~ReceiverRtsp();
    cv::Mat getNextFrame() override;
    uint64_t getTimestamp() { return this->timestamp; };
};

#endif // RECEIVERRTSP_HPP_