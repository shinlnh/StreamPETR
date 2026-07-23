#include <iostream>
#include <string>
#include "can_service.h" 

using namespace std;
int main(int argc, char **argv)
{
    printf("\nStart can client: %s\n", "");
    printf("\nDomain ID: %s", "");
    uint32_t domain_id;
    cin >> domain_id;
    can_service_if *client = new can_service_if("can_client", domain_id, false);

    while(1)
    {
        int request; 
        std::string str;
        uint32_t input;
        uint8_t steer_request;
        uint8_t lka_state;
        uint8_t gas_pedal;
        uint8_t gas_released;
        uint8_t drive_engaged;
        uint8_t gear;
        uint8_t brake_pedal;
        uint8_t brake_amount;
        printf("\nType 1 to send steering lka control message\n");
        printf("\nType 2 to send gas pendal control message \n");
        printf("\nType 3 to send gear packet control message \n");
        printf("\nType 4 to send brake control message\n");
        
        printf("\nType 0 to exit \n");
        cout << "\n\nPlease type your request: ";
        cin >> request;
        switch (request)
        {
            case 1: 
            {
                printf("Enter LKA_STATE: ");
                scanf("%hhu", &lka_state);
                printf("Enter STEER_REQUEST: ");
                scanf("%hhu", &steer_request);

                client->set_steering_lka(lka_state, steer_request);
                break;
            }
            case 2:
            {
                printf("Enter GAS_RELEASED: ");
                scanf("%hhu", &gas_released);
                printf("Enter GAS_PEDAL: ");
                scanf("%hhu", &gas_pedal);
                client->set_gas_pendal(gas_released, gas_pedal);
                break;
            }
            case 3:
            {
                printf("Enter DRIVE_ENGAGED: ");
                scanf("%hhu", &drive_engaged);
                printf("Enter GEAR: ");
                scanf("%hhu", &gear);
                getline(std::cin, str);
                client->set_gear_packet(drive_engaged, gear);
                break;
            }

            case 4:
            {
                printf("Enter BRAKE_AMOUNT: ");
                scanf("%hhu", &brake_amount);
                printf("Enter BRAKE_PEDAL: ");
                scanf("%hhu", &brake_pedal);
                client->set_brake(brake_amount, brake_pedal);;
                break;
            }
            default:
            {
                break;
            }
        }
        if (request == 0)
        {
            goto exit;
        }
    }
    exit: 
    delete client;
    return 0;
}
