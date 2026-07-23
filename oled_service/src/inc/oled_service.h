#ifndef OLED_SERVICE_H
#define OLED_SERVICE_H
#include "common.h"
#include <vector>
#include <mutex>
#include "oled_service_if.h"
#include "ssd1307.h"
#include "BV_logo_bitmap.h"
#include <map>

/* Includes board and MCU related header files. */
extern "C"
{
    #include <pthread.h>
    #include <time.h>
    #include <zlib.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <stdint.h>
    #include <stdio.h>
    #include <string.h>
}
/*define delay time to sent one packet*/
#define DELAY_TO_SENT 100000 // 100ms

#define DELAY_WARNING_MESSAGE 5

/*define buffer TX*/
#define MAX_BUFFER_TX 100

#define BACKGROUND_COLOR    BLACK
#define OBJECT_COLOR        WHITE

/*Class define ADAS Can service*/
class oled_service : public oled_service_if, public ssd1307
{
private:
    static oled_service *mInstancePtr;
    static mutex mLocker;

    /*thread id*/
    pthread_t oled_thread_render;
    pthread_t service_thread_render;
    bool oled_thread_render_status;

    /*mutex for can_buffer_tx resource*/
    pthread_mutex_t tx_mutex = PTHREAD_MUTEX_INITIALIZER;   
    /*condition variable*/
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;      

    /*buffer for tx oled*/
    std::vector<struct oled_frame> oled_frame_buffer;

    /*buffer for services*/
    std::map<std::string, std::string> services;

    /*oled page*/
    uint8_t home_page[SSD1307_BUFFER_SIZE];
    uint8_t warning_page[SSD1307_BUFFER_SIZE];
    uint8_t off_page[SSD1307_BUFFER_SIZE];

    /*OLED service thread*/
    static void *oled_render_thread(void *arg);

    /*service status render thread*/
    static void *service_render_thread(void *arg);

    /*private constructer*/
    oled_service(const char *client_id, const char *server_address, const char *device);

    /*OLED service private*/
    bv_err_return_t oled_on_off(uint32_t data);
    bv_err_return_t oled_num_client(uint32_t data);
    bv_err_return_t oled_access_point(uint32_t data);
    bv_err_return_t oled_battery_percent(uint32_t data);
    bv_err_return_t oled_warning_msg(unsigned char* data);
    bv_err_return_t oled_service_status_msg(unsigned char* data);
    bv_err_return_t oled_ip_address_msg(unsigned char* data);
    
    /*Start thread and cancel thread*/
    bv_err_return_t threads_start();
    bv_err_return_t threads_stop();

    /*Service message current row*/
    uint8_t service_msg_row;

public:
    ~oled_service();

    static oled_service *get_instance(const char *client_id, const char *server_address, const char *device);

    /*Can service public api*/
    bv_err_return_t oled_service_init();
    bv_err_return_t oled_service_deinit();
    static int msg_arrived(void *context, const char *topic_name, int topic_len, OledServiceTopic::Oled *message);
    static void connection_lost(void *context, char *cause);
};

#endif
