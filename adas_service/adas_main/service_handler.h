#ifndef __SERVICE_HANDLER_H__
#define __SERVICE_HANDLER_H__

#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include "adas_main.h"
#include "common.h"

class ServiceHandler
{
private:
    // Fields
    std::string user_input;
    AdasMain *adas_if;
    bool handler_alive_status = false;
    boost::thread* input_thread_handler;
    boost::mutex status_mutex;

    // Functions
    void inputThread();
    void printHelpMessage();
    void processInput(std::string user_input);
public:
    ServiceHandler(AdasMain* adas_if);
    void setHandlerAliveStatus(bool status);
    bool getHandlerAliveStatus();
    ~ServiceHandler();
};

#endif // __SERVICE_HANDLER_H__