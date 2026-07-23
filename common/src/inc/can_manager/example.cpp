#include "can_manager.h"
#include <atomic>
#include <csignal>

int selfpipe[2]{};

std::atomic_bool exit_requested;

void hanlde_signal(int signum)
{
    if (SIGTERM == signum) {
        exit_requested = true;
        if (-1 == write(selfpipe[1], "\0", 1)) {
            INFO("Error pipe");
        }
    }
}

void help_funct()
{
    printf("\ncan test\n");
    printf("            --start\n");
    printf("                    domain_id true_value\n");
    printf("            --help\n");
    printf("example: ./CanManager_main --start 42 5\n");
}

void printFrame(const can_frame& frame) 
{
    std::cout << "Trao cho anh" << std::endl;
    std::cout << "ID: " << std::hex << frame.can_id 
              << " DLC: " << std::dec << (int)frame.can_dlc 
              << " Data: ";
    
    for (int i = 0; i < frame.can_dlc; i++) {
        std::cout << std::hex << (int)frame.data[i] << " ";
    }
    std::cout << std::endl;
}

class DummyClass {
private:
    CanManager canManager; // Member variable for CanManager
    int field;

public:
    DummyClass(const char* device) : canManager(device) {
        // Register a callback for CAN frames with ID 123
        canManager.registerCallback({0x500, 0x124}, [this](const can_frame& frame) {
            handleCanFrame(frame);
        });
    }

    // Handle received CAN frame
    void handleCanFrame(const can_frame& frame) {
        // Assuming the data is an array of bytes and the int field is at the beginning
        std::cout << "Callback in DummyClass" << std::endl;
        this->field = frame.data[0];  // Modify the int field using data[0]
        std::cout << "Received CAN frame with ID: " << frame.can_id << std::endl;
        std::cout << "Modified int field value: " << field << std::endl;
    }
};

int main(int argc, char **argv)
{

    if (-1 == pipe(selfpipe)) {
        INFO("Error pipe");
    }

    string cmd(argv[1]);
    if (cmd == "--help")
    {
        help_funct();
        goto exit;
    }
    else if (cmd == "--start")
    {
        string device(argv[2]);
        // Example of using CanManager as a stand alone class.

        // CanManager *can;
        // CanManager can("CanManager", device.c_str());
        // CanManager can("vcan0");
        // can.registerCallback({0x100, 0x200}, printFrame);
        // can.registerCallback({0x400, 0x300}, [](const can_frame& frame) {
        //     std::cout << "Received ID: 0x" << std::hex << frame.can_id 
        //             << " DLC: " << std::dec << (int)frame.can_dlc 
        //             << " Data: ";
        //     for (int i = 0; i < frame.can_dlc; i++) {
        //         std::cout << std::hex << (int)frame.data[i] << " ";
        //     }
        //     std::cout << std::endl;
        // });

        // Example of using CanManager in another class. You can register callbacks using the dummy class's member function.
        DummyClass dummy("vcan0");
        printf("\nStart can service \n");

        signal(SIGTERM, hanlde_signal);

        while (!exit_requested) {
            int buffer;
            // Blocking read on selfpipe_
            ssize_t bytesRead(read(selfpipe[0], &buffer, sizeof(int)));
            if (bytesRead > 0) {
                // selfpipe_ is only used to signal an exit request.
                INFO("Exit can service");
                exit_requested = true;
            } else {
                INFO("Error pipe");
            }
        }

        signal(SIGTERM, SIG_DFL);

        close(selfpipe[0]);
        close(selfpipe[1]);
    
    }
    else
    {
        printf("\nArgument incorrect\n");
    }

exit:

    return 0;
}