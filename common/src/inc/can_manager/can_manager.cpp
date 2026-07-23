#include "can_manager.h"
#include <cstring>
#include <sstream>
#include <algorithm>
#include <fcntl.h>

/*Init static attributes*/
std::unordered_set<std::string> CanManager::registeredInterfaces;
std::mutex CanManager::constructor_mutex;

/**
 * @brief  Constructor. Does not allow creating 2 CanManagers with the same interface. Will throw if you try.
 * @param  device
 * @retval none
 */
CanManager::CanManager(const char *device)
: can_interface(device)
{
    std::lock_guard<std::mutex> lock(constructor_mutex);
    // Check if the interface is already registered
    if (isInterfaceRegistered(device)) {
        ERROR("Error: Interface %s is already registered!", device);
        // Handle the error, e.g., throw an exception or exit
        throw std::runtime_error("Interface already registered, you cannot register an interface twice!");
    }

    // Register the new interface
    registerInterface(device);
    INFO("CanManager for %s socket created", device);
    canManagerInit();
}

// Static function to check if a CAN interface is registered
bool CanManager::isInterfaceRegistered(const std::string& device) {
    return registeredInterfaces.find(device) != registeredInterfaces.end();
}

// Static function to register a CAN interface
void CanManager::registerInterface(const std::string& device) {
    registeredInterfaces.insert(device);
}

/**
 * @brief  Destructor. Close the sockets and stop the threads when object is destroyed.
 * @retval none
 */
CanManager::~CanManager()
{
    INFO("Exit %s service", "CAN");
    canManagerDeinit();
}

/**
 * @brief  Open a CAN socket from the passed in interface name.
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t CanManager::canSocketOpen()
{
    // Create TX socket
    can_socket_tx = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_tx < 0) {
        ERROR("TX socket creation error: %s", strerror(errno));
        return BV_RETURN_ERROR;
    }

    // Create RX socket
    can_socket_rx = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_rx < 0) {
        ERROR("RX socket creation error: %s", strerror(errno));
        close(can_socket_tx);
        return BV_RETURN_ERROR;
    }

    // Set receive buffer size to minimum
    // int rcvbuf_size = 0;
    // if (setsockopt(can_socket_rx, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
    //     ERROR("setsockopt SO_RCVBUF error: %s", strerror(errno));
    //     close(can_socket_tx);
    //     close(can_socket_rx);
    //     return BV_RETURN_ERROR;
    // }

    // // Set minimum bytes to process (one CAN frame)
    // int lowat = 16;
    // if (setsockopt(can_socket_rx, SOL_SOCKET, SO_RCVLOWAT, &lowat, sizeof(lowat)) < 0) {
    //     ERROR("setsockopt SO_RCVLOWAT error: %s", strerror(errno));
    //     close(can_socket_tx);
    //     close(can_socket_rx);
    //     return BV_RETURN_ERROR;
    // }

    // // Set socket to non-blocking mode
    // int flags = fcntl(can_socket_rx, F_GETFL, 0);
    // if (flags == -1) {
    //     ERROR("fcntl F_GETFL error: %s", strerror(errno));
    //     close(can_socket_tx);
    //     close(can_socket_rx);
    //     return BV_RETURN_ERROR;
    // }
    // if (fcntl(can_socket_rx, F_SETFL, flags | O_NONBLOCK) == -1) {
    //     ERROR("fcntl F_SETFL error: %s", strerror(errno));
    //     close(can_socket_tx);
    //     close(can_socket_rx);
    //     return BV_RETURN_ERROR;
    // }

    // // Set receive timeout (100ms)
    // struct timeval timeout = {0, 100000};
    // if (setsockopt(can_socket_rx, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    //     ERROR("setsockopt SO_RCVTIMEO error: %s", strerror(errno));
    //     close(can_socket_tx);
    //     close(can_socket_rx);
    //     return BV_RETURN_ERROR;
    // }

    // // Optionally verify the buffer size
    // int actual_rcvbuf_size;
    // socklen_t size_len = sizeof(actual_rcvbuf_size);
    // if (getsockopt(can_socket_rx, SOL_SOCKET, SO_RCVBUF, &actual_rcvbuf_size, &size_len) < 0) {
    //     ERROR("getsockopt SO_RCVBUF error: %s", strerror(errno));
    // } else {
    //     printf("Actual receive buffer size: %d\n", actual_rcvbuf_size);
    // }

    struct ifreq ifr;
    struct sockaddr_can addr;

    // Get interface index (only need to do this once)
    strcpy(ifr.ifr_name, this->can_interface);
    if (ioctl(can_socket_tx, SIOCGIFINDEX, &ifr) == -1) {
        ERROR("ioctl error: %s", strerror(errno));
        close(can_socket_tx);
        close(can_socket_rx);
        return BV_RETURN_ERROR;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind both sockets
    if (bind(can_socket_tx, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ERROR("TX socket bind error: %s", strerror(errno));
        close(can_socket_tx);
        close(can_socket_rx);
        return BV_RETURN_ERROR;
    }

    if (bind(can_socket_rx, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ERROR("RX socket bind error: %s", strerror(errno));
        close(can_socket_tx);
        close(can_socket_rx);
        return BV_RETURN_ERROR;
    }

    return BV_RETURN_OK;
}

/**
 * @brief  Close CAN socket
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t CanManager::canSocketClose()
{
    if (can_socket_tx >= 0) {
        close(can_socket_tx);
        can_socket_tx = -1;
    }

    if (can_socket_rx >= 0) {
        close(can_socket_rx);
        can_socket_rx = -1;
    }

    return BV_RETURN_OK;
}

/**
 * @brief  Can write thread. We use separate sockets for read and write to avoid race condition.
 * @param  arg argument
 * @retval void
 */
void CanManager::canWriteThread()
{
    INFO("start %s tx service thread success. ", "CAN");
    while (allow_run_threads)
    {
        /*check the buffer*/
        std::unique_lock<std::mutex> tx_thread_lock(write_queue_mutex);
        cond_can_write.wait(tx_thread_lock, [this] { 
            return !can_write_queue.empty() || !allow_run_threads; 
        });

        while (!can_write_queue.empty()) {
            can_frame frame = can_write_queue.front();  // Get the front frame
            can_write_queue.pop();  // Remove it from the queue

            debugCanFrame(frame);

            // Send the can_frame to the CAN socket
            ssize_t bytes_sent = send(can_socket_tx, &frame, sizeof(frame), 0);
            if (bytes_sent == -1) {
                ERROR("Error sending CAN frame: %s", strerror(errno));
            }
        }
    }
    INFO("Stop %s tx service success. ", "CAN");
}

/**
 * @brief  Can read thread. 
*          We use separate sockets for read and write to avoid race condition. 
*          Only push CAN message to queue and handle buffer overflow
 * @param  arg argument
 * @retval void
 */
void CanManager::canReadThread()
{

    INFO("start %s rx service thread success. ", "CAN");
    fd_set read_fds;
    timeval timeout;

    while (allow_run_threads)
    {
        /*check the buffer*/
        if (can_read_queue.size() < MAX_BUFFER_RX)
        {
            //> This code block prevents the read loop to spin infinitely when there is nothing to read
            // Functions to add the current socket to a monitoring set.
            FD_ZERO(&read_fds);
            FD_SET(can_socket_rx, &read_fds);

            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            // This function blocks until there is new data at the socket, there is a timeout which is set above.
            int select_result = select(can_socket_rx + 1, &read_fds, nullptr, nullptr, &timeout);

            // Loop again, check 'allow_run_threads' if there is an error or timeout.
            if (select_result == -1) {
                ERROR("Error in select: %s", strerror(errno));
                continue;
            } else if (select_result == 0) {
                // Timeout occurred, check allow_run_threads and continue
                continue;
            }
            //< End block.

            can_frame frame;
            ssize_t bytes_read = read(can_socket_rx, &frame, sizeof(struct can_frame));

            if (bytes_read >= 0)
            {
                {
                    std::lock_guard<std::mutex> queue_lock(read_queue_mutex);
                    can_read_queue.push(frame);
                }
                // Notify the processing thread that the queue is not empty.
                cond_can_read.notify_one();
            }
            else
            {
                ERROR("Failed to read from %s", "CAN");
            }
        }
    }
    INFO("Stop %s rx service success. ", "CAN");
}

/**
 * @brief CAN processing message thread.
 *        Process all meassage in message queue
 */
void CanManager::canProcessMsgThread()
{
    INFO("Start %s processing thread success.", "CAN");

    while (allow_run_threads)
    {
        std::unique_lock<std::mutex> queue_lock(read_queue_mutex);

        // Wait until the queue is not empty or `allow_run_threads` is false.
        cond_can_read.wait(queue_lock, [this] {
            return !can_read_queue.empty() || !allow_run_threads;
        });

        // Check again to ensure allow_run_threads was not turned off.
        if (!allow_run_threads && can_read_queue.empty())
        {
            break;
        }

        // Process all messages in the queue.
        while (!can_read_queue.empty())
        {
            can_frame frame = can_read_queue.front();
            can_read_queue.pop();

            queue_lock.unlock(); // Unlock during processing to allow other threads to push new messages.
            for (const auto& callbackPair : callbacks_) {
                const auto& canIDs = callbackPair.first;
                const auto& callback = callbackPair.second;

                // Check if the CanID is in the list of cared about CanIDs
                if (std::find(canIDs.begin(), canIDs.end(), frame.can_id) != canIDs.end()) {
                    callback(frame);  // Call the callback if the CanID matches
                }
            }
            queue_lock.lock(); // Re-lock to access the queue safely.
        }
    }

    INFO("Stop %s processing thread success.", "CAN");
}


/**
 * @brief  Start the read and write threads
 * @retval void
 */
bv_err_return_t CanManager::startThreads()
{
    bv_err_return_t return_code = BV_RETURN_OK;

    try {
        /*thread TX*/
        allow_run_threads = true;
        can_write_thread = std::thread(&CanManager::canWriteThread, this);

        /*thead RX*/
        can_read_thread = std::thread(&CanManager::canReadThread, this);

        /*thread process message queue*/
        can_process_msg_thread = std::thread(&CanManager::canProcessMsgThread, this);

    } catch (const std::system_error& e) {
        ERROR("Failed to create thread: %s", e.what());
        allow_run_threads = false;
        return_code = BV_RETURN_ERROR;
        if (this->stopThreads() == BV_RETURN_ERROR)
        {
            ERROR("stopThreads %s", "failed");
        }
    }

    return return_code;
}

/**
 * @brief  Stop the read and write threads
 * @retval void
 */
bv_err_return_t CanManager::stopThreads()
{
    bv_err_return_t return_code = BV_RETURN_OK;
    allow_run_threads = false;
    cond_can_write.notify_one();
    cond_can_read.notify_one();
    
    if (can_write_thread.joinable()) {
        can_write_thread.join();
    }
    
    if (can_read_thread.joinable()) {
        can_read_thread.join();
    }

    if (can_process_msg_thread.joinable()) {
        can_process_msg_thread.join();
    }
    return BV_RETURN_OK;
}

/**
 * @brief  Initialize the CanManager object. Open the CAN socket and start the threads
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t CanManager::canManagerInit()
{
    bv_err_return_t return_code = BV_RETURN_OK;

    /*open can socket*/
    if (this->canSocketOpen() == BV_RETURN_ERROR)
    {
        ERROR("canSocketOpen %s", "failed");
        return_code = BV_RETURN_ERROR;
    }

    /*starting threads*/
    if (this->startThreads() == BV_RETURN_ERROR)
    {
        ERROR("startThreads %s", "failed");
        return_code = BV_RETURN_ERROR;
    }

    /*check return code*/
    if (return_code == BV_RETURN_ERROR)
    {
        ERROR("init %s", "failed");
    }
    else
    {
        INFO("init %s", "success");
    }

    return return_code;
}

/**
 * @brief  Deinitialize the CanManager object. Close the CAN socket and stop the threads
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t CanManager::canManagerDeinit()
{
    bv_err_return_t return_code = BV_RETURN_OK;

    /*close canx*/
    this->canSocketClose();

    /*stropping threads*/
    if (this->stopThreads() == BV_RETURN_ERROR)
    {
        ERROR("stopThreads %s", "failed");
        return_code = BV_RETURN_ERROR;
    }

    /*check return*/
    if (return_code == BV_RETURN_ERROR)
    {
        ERROR("deinit %s", "failed");
    }
    else
    {
        INFO("deinit %s", "success");
    }
    return return_code;
}

/**
 * @brief  Register a callback function for a specific list of CAN IDs
 * 
 * @param canIDs What IDs of CAN message do you care about (a.k.a the CAN ID that will trigger the callback you registered)
 * @param callback The callback that will be called. Signature must contain a can_frame parameter
 */
void CanManager::registerCallback(const std::vector<uint32_t>& canIDs, CanFrameCallback callback) {
    std::lock_guard<std::mutex> lock(callback_register_mutex);
    callbacks_.emplace_back(canIDs, std::move(callback));
}

/**
 * @brief Simple CAN send API. Just pack your data in the can_frame and send it here.
 * 
 * @param can_frame_msg Your data
 * @return bv_err_return_t 
 */
bv_err_return_t CanManager::sendCanFrame(can_frame can_frame_msg)
{
    std::unique_lock<std::mutex> tx_thread_lock(write_queue_mutex);
    if (can_write_queue.size() < MAX_BUFFER_TX)
        can_write_queue.push(can_frame_msg);
    
    cond_can_write.notify_one();
    return BV_RETURN_OK;
}

/**
 * @brief Simple debug function that prints out the CAN frame's data
 * 
 * @param frame 
 */
void CanManager::debugCanFrame(const can_frame& frame) {
     // Create a string stream to build the output string
    std::ostringstream oss;

    // Add the CAN ID in hexadecimal
    oss << "[Sent Frame] " << "ID: " << std::hex << frame.can_id << ", Data: ";

    // Add each byte in the data array in hexadecimal
    for (int i = 0; i < frame.can_dlc; ++i) {
        oss << std::hex << static_cast<int>(frame.data[i]) << " ";
    }

    oss << "\n";
    // Convert the string stream to a standard string
    std::string output = oss.str();

    // Print the entire output in a single printf statement
    DEBUG("%s\n", output.c_str());
}