#ifndef CANMANAGER_H
#define CANMANAGER_H

#include "common.h"

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

/*define buffer TX*/
#define MAX_BUFFER_TX 100
#define MAX_BUFFER_RX 100

/*Class define ADAS Can service*/

class CanManager
{
    using CanFrameCallback = std::function<void(const can_frame&)>;
    
private:
    static std::mutex constructor_mutex;
    static std::unordered_set<std::string> registeredInterfaces;  // Static list of registered interfaces

     /*can device*/
    const char *can_interface;
    int can_socket_tx;
    int can_socket_rx;

    /*thread id*/
    std::vector<std::pair<std::vector<uint32_t>, CanFrameCallback>> callbacks_;
    
    
    /*mutex for can_buffer_tx resource*/
    std::mutex callback_register_mutex;

    /*buffer for tx can*/
    std::mutex write_queue_mutex;
    std::mutex read_queue_mutex;
    std::condition_variable cond_can_write;
    std::condition_variable cond_can_read;
    std::queue<can_frame> can_write_queue;
    std::queue<can_frame> can_read_queue;

    /*sysfs can function*/
    bv_err_return_t canSocketOpen();
    bv_err_return_t canSocketClose();

    /*Threading*/
    std::atomic<bool> allow_run_threads;
    std::thread can_write_thread;
    std::thread can_read_thread;
    std::thread can_process_msg_thread;
    void canWriteThread();
    void canReadThread();
    void canProcessMsgThread();

    /*start thread and cancel thread*/
    bv_err_return_t startThreads();
    bv_err_return_t stopThreads();
    bv_err_return_t canManagerInit();
    bv_err_return_t canManagerDeinit();

    void debugCanFrame(const can_frame& frame);

public:
    ~CanManager();
    CanManager(const char *device);
    // Function to check if the interface is already registered
    static bool isInterfaceRegistered(const std::string& device);
    // Function to register an interface
    static void registerInterface(const std::string& device);

    /*Can service public api*/
    void registerCallback(const std::vector<uint32_t>& canIDs, CanFrameCallback callback);
    bv_err_return_t sendCanFrame(can_frame can_msg_frame);
};

#endif