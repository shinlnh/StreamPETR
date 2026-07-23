#include "service_handler.h"

ServiceHandler::ServiceHandler(AdasMain* adas_if)
{
    this->adas_if = adas_if;
    setHandlerAliveStatus(true);
    input_thread_handler = new boost::thread(&ServiceHandler::inputThread, this);
    input_thread_handler->detach();
}

ServiceHandler::~ServiceHandler()
{
    delete input_thread_handler;
}

void ServiceHandler::inputThread()
{
    INFO("You are in service mode, type help for more information.");

    while (1) 
    {
        std::getline(std::cin, user_input);
        processInput(user_input);

        if (!getHandlerAliveStatus()) 
        {
            break;
        }
    }
    
    INFO("Execution done.");
}

void ServiceHandler::processInput(std::string user_input)
{
    DEBUG("Service Handler Started.");
    adas_if->setValue(STATUS_BTN_OBJ_ENABLE, true); // This is required to not crash the program.
    if ("exit" == user_input) 
    {
        setHandlerAliveStatus(false);
    } 
    else if ("start" == user_input)
    {
        adas_if->TriggerSignal(true);
    }
    else if ("stop" == user_input)
    {
        adas_if->TriggerSignal(false);
    }
    else if ("help" == user_input)
    {
        printHelpMessage();
    }
    else
    {
        //INFO("Illegal Input, type 'help' for more instructions");
    }
    
}

void ServiceHandler::printHelpMessage()
{
    INFO(   "============= Instructions =============\n"
            " Usage: [command]\n"
            " Options:\n"
            " help      : Display this help message\n"
            " start     : Start the service handler\n"
            " stop      : Stop the service handler\n"
            " exit      : Exit the service handler\n"
            " grab      : Save the current image to disk"
            "========================================\n");
}

void ServiceHandler::setHandlerAliveStatus(bool status) 
{
    boost::mutex::scoped_lock lock(status_mutex);
    handler_alive_status = status;
}

bool ServiceHandler::getHandlerAliveStatus() 
{
    boost::mutex::scoped_lock lock(status_mutex);
    return handler_alive_status;
}


