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

/** always send
 * throttle: 50%
 * brake: 50%
 * steer: 0 deg
 * state: IDLE -> LKS -> IDLE -> ACC -> ILDE -> HWC
 * State period: 100 frame
 * Freq: 20 Hz 
 * */ 
void scenario1() 
{
    CanManager can("can0");
    // throttle, brake
    can_frame control_msg;
    {    
        control_msg.can_id = TESLA_DI_DRIVER_SYSTEM_STATUS_FRAME_ID;
        control_msg.can_dlc = TESLA_DI_DRIVER_SYSTEM_STATUS_LENGTH;
        tesla_di_driver_system_status_t temp;
        temp.di_accel_pedal_pos = tesla_di_driver_system_status_di_accel_pedal_pos_encode(50.0);
        temp.di_brake_pedal_state = tesla_di_driver_system_status_di_brake_pedal_state_encode(50.0);
        temp.di_gear = tesla_di_driver_system_status_di_gear_encode(4);
        tesla_di_driver_system_status_pack(control_msg.data, &temp, control_msg.can_dlc);
    }    

    // steer
    can_frame steer_msg;
    {
        steer_msg.can_id = TESLA_STEERING_ANGLE_FRAME_ID;
        steer_msg.can_dlc = TESLA_STEERING_ANGLE_LENGTH;
        tesla_steering_angle_t temp;
        temp.steering_angle = tesla_steering_angle_steering_angle_encode(0);
        tesla_steering_angle_pack(steer_msg.data, &temp, steer_msg.can_dlc);
    }    

    // state
    can_frame state_msg;
    {
        state_msg.can_id = TESLA_DAS_STATUS_FRAME_ID;
        state_msg.can_dlc = TESLA_DAS_STATUS_LENGTH;
        tesla_das_status_t temp;
        temp.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_IDLE_CHOICE;
        tesla_das_status_pack(state_msg.data, &temp, state_msg.can_dlc);
    }    
    

    const int num_state = 6, num_counter = 100;
    int count_state = num_state, counter = num_counter;
    for (; count_state > 0; --count_state) {
        for (; counter > 0; --counter) {
            can.sendCanFrame(control_msg);
            can.sendCanFrame(steer_msg);
            can.sendCanFrame(state_msg);
            usleep(1000000 / FREQUENCY);
        }
        counter = num_counter;
        if (count_state % 2 == 0) {
            switch (count_state)
            {
            case 6:
                {
                    tesla_das_status_t temp;
                    temp.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_LKA_ACTIVE_CHOICE;
                    tesla_das_status_pack(state_msg.data, &temp, state_msg.can_dlc);
                }
                break;
            
            case 4:
                {
                    tesla_das_status_t temp;
                    temp.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_ACC_ACTIVE_CHOICE;
                    tesla_das_status_pack(state_msg.data, &temp, state_msg.can_dlc);
                }
                break;
            
            case 2:
                {
                    tesla_das_status_t temp;
                    temp.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_HWC_ACTIVE_CHOICE;
                    tesla_das_status_pack(state_msg.data, &temp, state_msg.can_dlc);
                }
                break;
            
            default:
                break;
            }
        } else {
            tesla_das_status_t temp;
            temp.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_IDLE_CHOICE;
            tesla_das_status_pack(state_msg.data, &temp, state_msg.can_dlc);
        }
    }
}

int main(int argc, char **argv)
{

    if (-1 == pipe(selfpipe)) {
        ERROR("Error pipe");
    }

    string cmd(argv[1]);

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
        printf("\nStart Scenario 1 \n");
        signal(SIGTERM, hanlde_signal);

        scenario1();
    }
    else
    {
        printf("\nArgument incorrect\n");
    }

exit:

    return 0;
}
