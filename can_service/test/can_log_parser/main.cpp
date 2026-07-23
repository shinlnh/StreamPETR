#include <atomic>
#include <csignal>
#include <fstream>
#include <sstream>

#include "can_manager.h"
#include "tesla.h"

int selfpipe[2]{};

std::atomic_bool exit_requested;

#define FREQUENCY 20

void hanlde_signal(int signum)
{
    if (SIGTERM == signum) {
        exit_requested = true;
        if (-1 == write(selfpipe[1], "\0", 1)) {
            ERROR("Error pipe");
        }
    }
}

void help_funct()
{
    printf("\ncan_service_main\n");
    printf("            --start\n");
    printf("                    domain_id true_value\n");
    printf("            --help\n");
    printf("example: ./can_service_main --start 42 5\n");
}

can_frame parse_candump_line(const std::string& line) {
    can_frame frame = {0};
    
    // Skip if empty line
    if (line.empty()) {
        return frame;
    }

    // Parse ID
    size_t id_start = line.find("0x");
    size_t id_end = line.find(">");
    if (id_start == std::string::npos || id_end == std::string::npos) {
        return frame;
    }
    
    std::string id_str = line.substr(id_start + 2, id_end - id_start - 2);
    frame.can_id = std::stoul(id_str, nullptr, 16);

    // Parse DLC
    size_t dlc_start = line.find("[");
    size_t dlc_end = line.find("]");
    if (dlc_start == std::string::npos || dlc_end == std::string::npos) {
        return frame;
    }
    
    std::string dlc_str = line.substr(dlc_start + 1, dlc_end - dlc_start - 1);
    frame.can_dlc = std::stoul(dlc_str);

    // Parse data bytes
    size_t data_start = line.find("]") + 2;  // Skip "] "
    std::string data_str = line.substr(data_start);
    std::stringstream ss(data_str);
    std::string byte;
    int i = 0;
    
    while (ss >> byte && i < 8) {
        frame.data[i++] = std::stoul(byte, nullptr, 16);
    }

    return frame;
}

void parse_candump_file(const std::string& filename) {
    CanManager can("can0");
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        can_frame frame = parse_candump_line(line);
        can.sendCanFrame(frame);
        usleep(1000000 / FREQUENCY);
    }
}

int main(int argc, char **argv)
{

    if (-1 == pipe(selfpipe)) {
        ERROR("Error pipe");
    }

    string cmd(argv[1]);
    char cmd_can_dump[200] = "candump";
    if (cmd == "--help")
    {
        help_funct();
        goto exit;
    }
    else if (cmd == "--start")
    {
        string domain_id(argv[2]);
        string device(argv[3]);

        printf("\nStart can service \n");
        signal(SIGTERM, hanlde_signal);

        parse_candump_file("../can_log.txt");
    }
    else
    {
        printf("\nArgument incorrect\n");
    }

exit:

    return 0;
}
