#include "can_service.h"
#include "version.h"
#include <atomic>
#include <csignal>

int selfpipe[2]{};

std::atomic_bool exit_requested;

void handle_signal(int signum)
{
    if (signum == SIGTERM || signum == SIGINT) {
        exit_requested = true;
        ssize_t res = write(selfpipe[1], "\0", 1);
        (void)res; // suppress unused variable warning
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

int main(int argc, char **argv)
{
    // Display version information
    printf("==============================================\n");
    printf("BV CAN Service %s\n", GIT_VERSION);
    printf("Commit: %s\n", GIT_COMMIT_HASH);
    printf("Branch: %s\n", GIT_BRANCH);
    printf("Build: %s %s\n", BUILD_DATE, BUILD_TIME);
    printf("==============================================\n");

    rclcpp::init(argc, argv);
    if (-1 == pipe(selfpipe)) {
        ERROR("Error pipe");
    }

    std::string cmd(argv[1]);
    char cmd_can_dump[200] = "candump";
    if (cmd == "--help")
    {
        help_funct();
        rclcpp::shutdown();
        return 0;
    }
    else if (cmd == "--start")
    {
        std::string domain_id(argv[2]);
        std::string device(argv[3]);

        can_service can("can_service", stoi(domain_id), device.c_str());
        printf("\nStart can service \n");

        signal(SIGTERM, handle_signal);
        signal(SIGINT, handle_signal);

        while (!exit_requested) {
            char buffer;
            ssize_t bytesRead = read(selfpipe[0], &buffer, 1);
            if (bytesRead > 0) {
                INFO("Exit can service");
                exit_requested = true;
            }
        }

        close(selfpipe[0]);
        close(selfpipe[1]);

        rclcpp::shutdown();
        return 0;
    }
    else
    {
        printf("\nArgument incorrect\n");
    }

    rclcpp::shutdown();
    return 0;
}
