#include "oled_service.h"
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
    INFO("\noled_service_main\n");
    INFO("            --start\n");
    INFO("                    domain_id path_to_device\n");
    INFO("            --help\n");
    INFO("example: ./oled_service_main --start 42 /dev/fb2\n");
}

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
        string server_ip(argv[2]);
        string device(argv[3]);

        oled_service *oled;
        oled = oled_service::get_instance("oled_service", server_ip.c_str(), device.c_str());
        oled->oled_service_init();
        INFO("Start oled service type e to exit");

        signal(SIGTERM, hanlde_signal);
        
        while (!exit_requested) {
            int buffer;
            // Blocking read on selfpipe_
            ssize_t bytesRead(read(selfpipe[0], &buffer, sizeof(int)));
            if (bytesRead > 0) {
                // selfpipe_ is only used to signal an exit request.
                INFO("Exit oled service");
                oled->oled_service_deinit();
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
