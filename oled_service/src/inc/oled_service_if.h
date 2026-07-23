#ifndef OLED_SERVICE_IF_H
#define OLED_SERVICE_IF_H
/* Includes OpenDDS libary*/
#include <ace/Log_Msg.h>
#include <ace/Global_Macros.h>
#include <ace/OS_NS_stdlib.h>

#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DdsDcpsPublicationC.h>
#include <dds/DdsDcpsSubscriptionC.h>

#include <dds/DCPS/LocalObject.h>
#include <dds/DCPS/Definitions.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/WaitSet.h>

#include <dds/DCPS/StaticIncludes.h>
#ifdef ACE_AS_STATIC_LIBS
#  include <dds/DCPS/RTPS/RtpsDiscovery.h>
#  include <dds/DCPS/transport/rtps_udp/RtpsUdp.h>
#endif

/* Includes board and MCU related header files. */
#include "common.h"
#include <vector>
#include "OledServiceTopicTypeSupportImpl.h"
#include <mutex>
extern "C"
{
    #include <pthread.h>
    #include <time.h>
    #include <string.h>
    #include <stdio.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <errno.h>
    #include <ctype.h>
}

struct oled_frame
{
    uint32_t type;
    unsigned char* warn_msg;
    uint32_t data;
    unsigned char* service_status_msg;
    unsigned char* ip_address_msg; 
};

/*define data type of oled frame*/
enum type_data 
{
    bat_percent = 1, 
    num_client = 2, 
    war_message = 3, 
    on_off_ap = 4, 
    on_off_oled = 5,
    service_status = 6,
    ip_address = 7
};

/*define on/off state for screen and */
enum on_off_state {on = 1, off = 0};

// topic name
#define OLED_BATTERY_PERCENT "oled_battery_percent"
#define OLED_NUM_CLIENT      "oled_num_client"
#define OLED_ON_OFF          "oled_on_off"
#define OLED_WARNING_MSG     "oled_warning_msg"
#define OLED_ACCESS_POINT    "oled_access_point"
#define DEFAULT_DOMAIN_ID    "42"
#define OLED_SERVICE_STATUS  "oled_service_status"
#define OLED_IP_ADDRESS      "oled_ip_address"

class oled_service_if
{
    private:
        const char* domain_id;
        int argc;
        std::vector<char*> argv;
        DDS::DomainParticipantFactory_var dpf;
        DDS::DomainParticipant_var participant;
        DDS::WaitSet_var ws;
        DDS::Topic_var topic_status;
        DDS::Topic_var topic_text;
        OledServiceTopic::StatusTypeSupport_var ts_status;
        OledServiceTopic::TextTypeSupport_var ts_text;
        DDS::Duration_t dds_timeout;
        CORBA::String_var type_name_status;
        CORBA::String_var type_name_text;

        DDS::Publisher_var publisher;
        DDS::DataWriter_var writer_status;
        DDS::DataWriter_var writer_text;
        DDS::DataWriterQos writer_qos;
        OledServiceTopic::StatusDataWriter_var message_writer_status;
        OledServiceTopic::TextDataWriter_var message_writer_text;

        DDS::Subscriber_var subscriber;
        DDS::DataReaderListener_var listener;
        DDS::DataReader_var reader_status;
        DDS::DataReader_var reader_text;
        DDS::DataReaderQos reader_qos;
        OledServiceTopic::StatusDataReader_var message_reader_status;
        OledServiceTopic::TextDataReader_var message_reader_text;

        OledServiceTopic::Status message_status;
        OledServiceTopic::Text message_text;

        class DataReaderListenerImpl: public virtual OpenDDS::DCPS::LocalObject<DDS::DataReaderListener> {
            public:
                void (*connection_lost)(void*, char*);
                void (*delivery_complete_handler)(void*, int);
                int (*message_arrived)(void*, const char*, int, OledServiceTopic::Oled*);
                oled_service_if* helper;
                virtual void on_requested_deadline_missed(
                    DDS::DataReader_ptr reader,
                    const DDS::RequestedDeadlineMissedStatus& status);

                virtual void on_requested_incompatible_qos(
                    DDS::DataReader_ptr reader,
                    const DDS::RequestedIncompatibleQosStatus& status);

                virtual void on_sample_rejected(
                    DDS::DataReader_ptr reader,
                    const DDS::SampleRejectedStatus& status);

                virtual void on_liveliness_changed(
                    DDS::DataReader_ptr reader,
                    const DDS::LivelinessChangedStatus& status);

                virtual void on_data_available(
                    DDS::DataReader_ptr reader);

                virtual void on_subscription_matched(
                    DDS::DataReader_ptr reader,
                    const DDS::SubscriptionMatchedStatus& status);

                virtual void on_sample_lost(
                    DDS::DataReader_ptr reader,
                    const DDS::SampleLostStatus& status);
        };

        DataReaderListenerImpl* listener_servant;

        void register_oled_type();
        void create_oled_sub();
        void create_oled_pub();
        bv_err_return_t wait_for_sub(DDS::DataWriter_var writer);

    public:
        oled_service_if(const char *client_id = "oled_service", const char *domain_id = DEFAULT_DOMAIN_ID, bool sub_topic = false);
        ~oled_service_if();
        /*write data to oled*/
        bv_err_return_t write_oled_dbus_data(oled_frame *oled_dbus);

        /*user API*/
        inline const char *get_client_id(){return this->domain_id;};
        bv_err_return_t set_battery_percent(uint32_t percent, uint32_t qos = 2);
        bv_err_return_t on_off_access_point(uint32_t status, uint32_t qos = 2);
        bv_err_return_t set_number_wifi_client(uint32_t number_client, uint32_t qos = 2);
        bv_err_return_t set_text_message(const char *mes, const char *type, uint32_t qos = 2);
        bv_err_return_t on_off_screen(uint32_t status, uint32_t qos = 2);
};

#endif
