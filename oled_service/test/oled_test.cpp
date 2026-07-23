#include <iostream>
#include <string>
#include "oled_service.h" 

using namespace std;
int main(int argc, char **argv)
{
    std::string id;
    printf("\nStart oled client: %s\n", "");
    printf("\nDomain ID: %s", "");
    getline(std::cin, id);
    oled_service_if *client = new oled_service_if("oled_client", id.c_str());

    while(1)
    {
        int request; 
        std::string str;
        uint32_t input;
        printf("\nType 1 to send battery data\n");
        printf("\nType 2 to send wifi client data \n");
        printf("\nType 3 to send warning message \n");
        printf("\nType 4 to on/off Access point \n");
        printf("\nType 5 to on/off OLED \n");
        printf("\nType 6 to send battery data 100 times \n");
        printf("\nType 7 to send 100 warning message \n");
        printf("\nType 8 to send service status message \n");
        printf("\nType 9 to send ip address message \n");
        printf("\nType 0 to exit \n");
        cout << "\n\nPlease type your request: ";
        cin >> request;
        switch (request)
        {
            case 1: 
            {
                cout << "Please type the battery percent: ";
                cin >> input;
                client->set_battery_percent(input);
                break;
            }
            case 2:
            {
                cout << "Please type number of client: ";
                cin >> input;
                client->set_number_wifi_client(input);
                break;
            }
            case 3:
            {
                cin.ignore();
                cout << "Please type error message: ";
                getline(std::cin, str);
                client->set_text_message(str.c_str(), OLED_WARNING_MSG);
                break;
            }

            case 4:
            {
                cout << "Switch access point on off: \n";
                cout << "Input value:\n";
                cout << "1: On. \n";
                cout << "0: Off. \n";
                cin >> input;
                client->on_off_access_point(input);
                break;
            }
            case 5:
            {
                cout << "Switch sceeen on off: \n";
                cout << "Input value:\n";
                cout << "1: On. \n";
                cout << "0: Off. \n";
                cin >> input;
                client->on_off_screen(input);
                break;
            }
            case 6:
            {
                for(int i = 0; i <= 100; i++)
                {
                    client->set_battery_percent(i);
                    usleep(100000);
                }
                break;
            }
            case 7:
            {
                for(int i = 0; i < 100; i++)
                {
                    client->set_text_message(std::to_string(i).c_str(), OLED_WARNING_MSG);
                    usleep(5000000);
                }
                break;
            }
            case 8:
            {
                cin.ignore();
                cout << "Please type service status message: ";
                getline(std::cin, str);
                client->set_text_message(str.c_str(), OLED_SERVICE_STATUS);
                break;
            }
            case 9:
            {
                cin.ignore();
                cout << "Please type ip address message: ";
                getline(std::cin, str);
                client->set_text_message(str.c_str(), OLED_IP_ADDRESS);
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