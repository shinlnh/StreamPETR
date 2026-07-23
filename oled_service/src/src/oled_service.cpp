#include "oled_service.h"

/*define oled_service instance*/
oled_service *oled_service::mInstancePtr = nullptr;
mutex oled_service::mLocker;

/**********private function**********/

/**
 * @brief  Constructer oled service and return singalton object
 * @param  domain_server register domain name of oled service on dbus
 * @retval oled_service
 */
oled_service *oled_service::get_instance(const char *client_id, const char *server_address, const char *device)
{
    mLocker.lock();
    if (nullptr == mInstancePtr) {
        mInstancePtr = new oled_service(client_id, server_address, device);
    }
    mLocker.unlock();
    return mInstancePtr;
}

/**
 * @brief  constructor function
 * @param  fbfd name of OLED interface (dev/fbfd)
 * @param  domain_server domain name in dbus
 * @retval none
 */
oled_service::oled_service(const char *client_id, const char *server_address, const char *device) : oled_service_if(client_id, server_address, true), ssd1307(device)
{
    this->oled_frame_buffer.clear();
    this->oled_thread_render_status = false;
    
    for (int i=0; i<SSD1307_BUFFER_SIZE; i++) {
        this->home_page[i] = bitmap_logo_BanVien[i];
        this->warning_page[i] = 0;
        this->off_page[i] = 0;
    }

    this->service_msg_row = 1;

    INFO("Initial %s service", "OLED");
}
/**
 * @brief  Destructer function
 * @retval none
 */
oled_service::~oled_service()
{
    INFO("Exit %s service", "OLED");
    /*cancel thread*/
    if (this->oled_thread_render_status == true) {
        this->threads_stop();
    }
    /*clear data*/
    this->oled_frame_buffer.clear();
}

/*Private API*/

/**
 * @brief  starting the threads
 * @retval void
 */
bv_err_return_t oled_service::threads_start()
{
    bv_err_return_t return_code = BV_RETURN_OK;
    int log_error;

    /*thread TX*/
    log_error = pthread_create(&this->oled_thread_render, NULL, &oled_service::oled_render_thread, this);
    log_error = pthread_create(&this->service_thread_render, NULL, &oled_service::service_render_thread, this);
    if (log_error != 0) {
        ERROR("pthread_create: %d ", log_error);
        return_code = BV_RETURN_ERROR;
    }
    else {
        this->oled_thread_render_status = true;
    }

    /*check return code*/
    if (return_code == BV_RETURN_ERROR) {
        ERROR("threads_start %s", "failed");
        /*stop threads*/
        if (this->threads_stop() == BV_RETURN_ERROR) {
            ERROR("threads_stop %s", "failed");
        }
    }
    else {
        INFO("threads_start %s", "success");
    }
    return return_code;
}

/**
 * @brief  stopping the threads
 * @retval void
 */
bv_err_return_t oled_service::threads_stop()
{
    bv_err_return_t return_code = BV_RETURN_OK;
    int log_error;
    if (this->oled_thread_render_status == true) {
        log_error = pthread_cancel(this->oled_thread_render);
        if (log_error != 0) {
            ERROR("pthread_cancel: %d", log_error);
            return_code = BV_RETURN_ERROR;
        }
        else {
            INFO("pthread_cancel %s", "success");
            this->oled_thread_render_status = false;
        }
    }

    /*check return code*/
    if (return_code == BV_RETURN_ERROR) {
        ERROR("threads_stop %s", "failed");
    }
    else {
        INFO("threads_stop %s", "success");
    }
    return return_code;
}


/**
 * @brief  initialize oled service. Starting dbus connect and oled thread.
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_service_init()
{
    bv_err_return_t return_code = BV_RETURN_OK;

    /*starting threads*/
    if (this->threads_start() == BV_RETURN_ERROR) {
        ERROR("threads_start %s", "failed");
        return_code = BV_RETURN_ERROR;
    }

    ssd1307_fill(BLACK);

    this->buffer = this->home_page;

    //First, dispay logo in 5 seconds
    ssd1307_refresh();
    sleep(5);

    //Clear logo
    for (int i=0; i<SSD1307_BUFFER_SIZE; i++) {
        this->home_page[i] = 0;
    }
    ssd1307_refresh();

    //Display another information
    oled_battery_percent(100);
    oled_access_point(1);
    oled_num_client(0);
    oled_ip_address_msg((unsigned char*)("192.168.201.6"));

    ssd1307_refresh();

    /*check return code*/
    if (return_code == BV_RETURN_ERROR) {
        ERROR("oled_service_init %s", "failed");
    }
    else {
        INFO("oled_service_init %s", "success");
    }

    return return_code;
}

/**
 * @brief  delete oled service thread and disconnect dbus, oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_service_deinit()
{
    bv_err_return_t return_code = BV_RETURN_OK;

    /*stropping threads*/
    if (this->threads_stop() == BV_RETURN_ERROR) {
        ERROR("threads_stop %s", "failed");
        return_code = BV_RETURN_ERROR;
    }

    /*check return*/
    if (return_code == BV_RETURN_ERROR) {
        ERROR("oled_service_deinit %s", "failed");
    }
    else {
        INFO("oled_service_deinit %s", "success");
    }
    return return_code;
}

/**
 * @brief  on/off oled home page
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_on_off(uint32_t data)
{
    if (data == 0) {
        this->buffer = this->off_page;
        this->ssd1307_refresh();
    } else {
        this->buffer = this->home_page;
        this->ssd1307_refresh();
    }    
    return BV_RETURN_OK;
}

/**
 * @brief  set battery percent on oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_battery_percent(uint32_t data)
{
    ssd1307_fill_rectangle(110,0,122,6,BACKGROUND_COLOR);
    ssd1307_draw_rectangle(110,0,122,6,OBJECT_COLOR);
    ssd1307_draw_line(123, 1, 123, 5, OBJECT_COLOR);
    ssd1307_draw_line(124, 1, 124, 5, OBJECT_COLOR);

    string empty = "    ";
    ssd1307_write_string((unsigned char*) empty.c_str(), 10, 0, OBJECT_COLOR);
    int index = floor(data/9.3)+1;
    ssd1307_write_string((unsigned char*) to_string(data).append("%").c_str(), 10, 0, OBJECT_COLOR);
    for (int i=0; i<data; i++) {
        ssd1307_draw_line(floor(i/8.3)+110, 1, floor(i/8.3)+110, 5, OBJECT_COLOR);
    }

    ssd1307_refresh();

    return BV_RETURN_OK;
}

/**
 * @brief  set number of client on oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_num_client(uint32_t data)
{
    string empty = "   ";
    ssd1307_write_string((unsigned char*) empty.c_str(), 1, 0, OBJECT_COLOR);
    ssd1307_write_string((unsigned char*) to_string(data).c_str(), 1, 0, OBJECT_COLOR);

    oled_access_point(1);

    ssd1307_refresh();

    return BV_RETURN_OK;
}

/**
 * @brief  on/off access point on oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_access_point(uint32_t data)
{
    if (data == 1)  {
        ssd1307_draw_arc(0, 7, 7, 90, 180, OBJECT_COLOR);
        ssd1307_draw_arc(0, 7, 5, 90, 180, OBJECT_COLOR);
        ssd1307_draw_arc(0, 7, 3, 90, 180, OBJECT_COLOR);
        ssd1307_draw_arc(0, 7, 1, 90, 180, OBJECT_COLOR);
    } else {
        ssd1307_draw_arc(0, 7, 7, 90, 180, BACKGROUND_COLOR);
        ssd1307_draw_arc(0, 7, 5, 90, 180, BACKGROUND_COLOR);
        ssd1307_draw_arc(0, 7, 3, 90, 180, BACKGROUND_COLOR);
        ssd1307_draw_arc(0, 7, 1, 90, 180, BACKGROUND_COLOR);
        string empty = "   ";
        ssd1307_write_string((unsigned char*) empty.c_str(), 9, 0, OBJECT_COLOR);
    }

    ssd1307_refresh();

    return BV_RETURN_OK;
}

/**
 * @brief  show warning message on oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_warning_msg(unsigned char* data)
{
    ssd1307_write_string(data, 0, 0, OBJECT_COLOR);
    ssd1307_refresh();
    return BV_RETURN_OK;
}

/**
 * @brief  show service status message on oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_service_status_msg(unsigned char* data)
{
    //Clear current row
    string empty = " ";
    for (int i = 0; i < 16; ++i)
        ssd1307_write_string((unsigned char*) empty.c_str(), i, this->service_msg_row, OBJECT_COLOR);

    ssd1307_write_string(data, 0, this->service_msg_row, OBJECT_COLOR);
    ssd1307_refresh();

    //Move to next row
    this->service_msg_row++;
    if (this->service_msg_row > 2) this->service_msg_row = 1;
    
    return BV_RETURN_OK;
}

/**
 * @brief  show ip address message on oled
 * @retval bv_err_return_t check and log error
 */
bv_err_return_t oled_service::oled_ip_address_msg(unsigned char* data)
{
    //Clear current row
    string empty = " ";
    for (int i = 0; i < 16; ++i)
        ssd1307_write_string((unsigned char*) empty.c_str(), i, 3, OBJECT_COLOR);

    ssd1307_write_string(data, 0, 3, OBJECT_COLOR);
    ssd1307_refresh();
    return BV_RETURN_OK;
}

/**
 * @brief  This thread use for write oled frame into oled device
 * @param  arg argument
 * @retval void
 */
void *oled_service::oled_render_thread(void *arg)
{
    oled_service *_this = static_cast<oled_service *>(arg);
    INFO("start %s tx service thread success. ", "OLED");
    std::string input;
    size_t delimiterPos;
    while (1)
    {
        /*check the buffer*/
        pthread_mutex_lock(&_this->tx_mutex);
        oled_frame oled;

        if (!_this->oled_frame_buffer.empty()) {
            oled = _this->oled_frame_buffer[_this->oled_frame_buffer.size() - 1];
            _this->oled_frame_buffer.pop_back();
            switch (oled.type)
            {
            case on_off_oled:
                _this->buffer = _this->home_page;
                _this->oled_on_off(oled.data);
                break;
            case bat_percent:
                _this->buffer = _this->home_page;
                _this->oled_battery_percent(oled.data);
                break;
            case num_client:
                _this->buffer = _this->home_page;
                _this->oled_num_client(oled.data);
                break;
            case on_off_ap:
                _this->buffer = _this->home_page;
                _this->oled_access_point(oled.data);
                break;
            case war_message:
                _this->buffer = _this->warning_page;
                _this->ssd1307_fill(BACKGROUND_COLOR);
                _this->oled_warning_msg(oled.warn_msg);
                sleep(DELAY_WARNING_MESSAGE);
                _this->ssd1307_fill(BACKGROUND_COLOR);
                _this->buffer = _this->home_page;
                _this->ssd1307_refresh();
                delete(oled.warn_msg);
                break;
            case service_status:
                // Convert to std::string
                _this->buffer = _this->home_page;
                input = (reinterpret_cast<const char*>(oled.service_status_msg));

                // Find the position of the delimiter
                delimiterPos = input.find(":");

                if (delimiterPos != std::string::npos) {
                    // Extract the name and value substrings
                    std::string name = input.substr(0, delimiterPos);
                    std::string value = input.substr(delimiterPos + 1);
                    //Store to buffer
                    _this->services[name] = value;
                }

                break;
            case ip_address:
                _this->buffer = _this->home_page;
                _this->oled_ip_address_msg(oled.ip_address_msg);
                break;
            default:
                break;
            }

            INFO("TX buffer: %ld", _this->oled_frame_buffer.size());
        }
        else {
            pthread_cond_wait(&_this->cond, &_this->tx_mutex);
        }
 
        pthread_mutex_unlock(&_this->tx_mutex);

        /*delay for write next packet*/
        usleep(DELAY_TO_SENT);
    }

    INFO("Stop %s tx service success. ", "OLED");
    pthread_exit(NULL);
}

/**
 * @brief  This thread use for write service status into oled device
 * @param  arg argument
 * @retval void
 */
void *oled_service::service_render_thread(void *arg)
{
    oled_service *_this = static_cast<oled_service *>(arg);
    INFO("start %s service render thread success. ", "OLED");
    // Using an iterator
    std::map<std::string, std::string>::iterator it;
    it = _this->services.begin();
    int i = 0;
    while (1)
    {
        /*check the buffer*/
        if (it == _this->services.end())
            it = _this->services.begin();
        
        if (_this->services.size()){
            std::string str = it->first + ":" + it->second;
            // Convert std::string to unsigned char*
            unsigned char* ucharPtr = reinterpret_cast<unsigned char*>(const_cast<char*>(str.data()));
            cout << ucharPtr;
            _this->oled_service_status_msg(ucharPtr);
        }
        /*delay for write next packet*/
        ++it;
        i = ++i % 2;
        if (!i) sleep(4);
    }

    INFO("Stop %s render service success. ", "SERVICE RENDER");
    pthread_exit(NULL);
}

/**
 * @brief  Callback function when received message from client
 * @param  context oled service object
 * @param  topic_name topic name
 * @param  topic_len data length
 * @param  message message received
 * @retval void
 */
int oled_service::msg_arrived(void *context, const char *topic_name, int topic_len, OledServiceTopic::Oled *message)
{
    oled_service *_this = static_cast<oled_service *>(context);
    
    if(_this) {
        struct oled_frame oled_frame_data;
        pthread_mutex_lock(&_this->tx_mutex);
        if (!strcmp(topic_name, OLED_ON_OFF)) {
            oled_frame_data.type = on_off_oled;
            oled_frame_data.data = static_cast<OledServiceTopic::Status>(message->status_data).display_status;
        } else
        if (!strcmp(topic_name, OLED_BATTERY_PERCENT)) {
            oled_frame_data.type = bat_percent;
            oled_frame_data.data = static_cast<OledServiceTopic::Status>(message->status_data).battery_percent;
        } else 
        if (!strcmp(topic_name, OLED_NUM_CLIENT)) {
            oled_frame_data.type = num_client;
            oled_frame_data.data = static_cast<OledServiceTopic::Status>(message->status_data).num_client;
        } else
        if (!strcmp(topic_name, OLED_ACCESS_POINT)) {
            oled_frame_data.type = on_off_ap;
            oled_frame_data.data = static_cast<OledServiceTopic::Status>(message->status_data).access_point;
        } else
        if (!strcmp(topic_name, OLED_WARNING_MSG)) {
            oled_frame_data.warn_msg = new unsigned char[topic_len+1];
            oled_frame_data.type = war_message;
            for (int i=0; i<topic_len; i++) {
                oled_frame_data.warn_msg[i] = (char) (static_cast<OledServiceTopic::Text>(message->text_data).text_msg[i]);
            }
            oled_frame_data.warn_msg[topic_len] = '\0';
        } else
        if (!strcmp(topic_name, OLED_SERVICE_STATUS)) {
            oled_frame_data.service_status_msg = new unsigned char[topic_len+1];
            oled_frame_data.type = service_status;
            for (int i=0; i<topic_len; i++) {
                oled_frame_data.service_status_msg[i] = (char) (static_cast<OledServiceTopic::Text>(message->text_data).text_msg[i]);
            }
            oled_frame_data.service_status_msg[topic_len] = '\0';
        } else
        if (!strcmp(topic_name, OLED_IP_ADDRESS)) {
            oled_frame_data.ip_address_msg = new unsigned char[topic_len+1];
            oled_frame_data.type = ip_address;
            for (int i=0; i<topic_len; i++) {
                oled_frame_data.ip_address_msg[i] = (char) (static_cast<OledServiceTopic::Text>(message->text_data).text_msg[i]);
            }
            oled_frame_data.ip_address_msg[topic_len] = '\0';
        }        
        else  {
            ERROR("Topic name %s is not used", topic_name);
            oled_frame_data.type = 0xFF;
        }
        _this->oled_frame_buffer.emplace(_this->oled_frame_buffer.begin(), oled_frame_data);
        INFO("push oled_frame data with type = %d into fifo ", _this->oled_frame_buffer[0].type);
    }
    else {
        ERROR("%s", "Can not found oled service \n");
    }

    pthread_cond_signal(&_this->cond);
    pthread_mutex_unlock(&_this->tx_mutex);

    return 1;
}

void oled_service::connection_lost(void *context, char *cause)
{
    ERROR("\nConnection lost with cause %s\n", cause);
}