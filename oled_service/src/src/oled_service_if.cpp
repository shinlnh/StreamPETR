#include "oled_service_if.h"
#include "oled_service.h"

/**
 * @brief  constructer of oled_service_dbus
 * @param  domain the domain name of client process. Note that each client process has unique bus name
 * @retval void
 */
oled_service_if::oled_service_if(const char *client_id, const char *domain_id, bool sub_topic)
{
    INFO("constructer %s", "oled_service_if");
    this->domain_id = domain_id;
    
    // Initialize DomainParticipantFactory
    this->argc = 2;
    std::vector<std::string> arguments = {"-DCPSConfigFile", "/lib/banvien/rtps.ini"};
    for (const auto &arg : arguments) {
        argv.push_back((char *)arg.data());
    }
    argv.push_back(nullptr);

    this->dpf = TheParticipantFactoryWithArgs(this->argc, (ACE_TCHAR **)argv.data());

    // Create DomainParticipant
    this->participant =
        this->dpf->create_participant(stoll(domain_id),
                                      PARTICIPANT_QOS_DEFAULT,
                                      0,
                                      OpenDDS::DCPS::DEFAULT_STATUS_MASK);
    if (!this->participant)
    {
        ERROR("create participant \n %s", "failed");
    }

    register_oled_type();

    if (sub_topic)
    {
        create_oled_sub();
    }
    else
    {
        create_oled_pub();
        wait_for_sub(this->writer_status);
        wait_for_sub(this->writer_text);
    }
}

/**
 * @brief  register for oled type
 * @retval void
 */
void oled_service_if::register_oled_type() {
    // Register TypeSupport (OledServiceTopic::Status)
    this->ts_status = new OledServiceTopic::StatusTypeSupportImpl;
    if (this->ts_status->register_type(participant, "") != DDS::RETCODE_OK)
    {
        ERROR("register type for status \n %s", "failed");
    }
    this->type_name_status = ts_status->get_type_name();
    this->topic_status =
        participant->create_topic("OledServiceTopicStatus",
                                  this->type_name_status,
                                  TOPIC_QOS_DEFAULT,
                                  0,
                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->topic_status)
    {
        ERROR("create topic oled status \n %s", "failed");
    }

    // Register TypeSupport (OledServiceTopic::Text)
    this->ts_text = new OledServiceTopic::TextTypeSupportImpl;
    if (this->ts_text->register_type(participant, "") != DDS::RETCODE_OK)
    {
        ERROR("register type for text \n %s", "failed");
    }
    this->type_name_text = ts_text->get_type_name();
    this->topic_text =
        participant->create_topic("OledServiceTopicText",
                                  this->type_name_text,
                                  TOPIC_QOS_DEFAULT,
                                  0,
                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->topic_text)
    {
        ERROR("create topic text status \n %s", "failed");
    }
}

/**
 * @brief  create subscriber with datareader
 * @retval void
 */
void oled_service_if::create_oled_sub()
{
    this->listener = (new oled_service_if::DataReaderListenerImpl);
    this->listener_servant = dynamic_cast<oled_service_if::DataReaderListenerImpl *>(listener.in());
    this->listener_servant->helper = this;
    this->listener_servant->message_arrived = oled_service::msg_arrived;
    this->listener_servant->connection_lost = oled_service::connection_lost;

    INFO("Subscribe to topic OledServiceTopic %s ", "");

    // Create Subscriber
    this->subscriber =
        this->participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                             0,
                                             OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->subscriber)
    {
        ERROR("create subscriber \n %s", "failed");
    }

    // Create DataReader for Status Topic
    this->subscriber->get_default_datareader_qos(this->reader_qos);
    this->reader_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;

    this->reader_status =
        this->subscriber->create_datareader(this->topic_status,
                                            this->reader_qos,
                                            this->listener.in(),
                                            OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->reader_status)
    {
        ERROR("create status datareader \n %s", "failed");
    }

    this->message_reader_status =
        OledServiceTopic::StatusDataReader::_narrow(this->reader_status);

    if (!this->message_reader_status)
    {
        ERROR("create narrow status reader \n %s", "failed");
    }

    // Create DataReader for Text Topic
    this->subscriber->get_default_datareader_qos(this->reader_qos);
    this->reader_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;

    this->reader_text =
        this->subscriber->create_datareader(this->topic_text,
                                            this->reader_qos,
                                            this->listener.in(),
                                            OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->reader_text)
    {
        ERROR("create text datareader \n %s", "failed");
    }

    this->message_reader_text =
        OledServiceTopic::TextDataReader::_narrow(this->reader_text);

    if (!this->message_reader_text)
    {
        ERROR("create narrow text datareader \n %s", "failed");
    }
}

/**
 * @brief  create publisher with datawriter
 * @retval void
 */
void oled_service_if::create_oled_pub()
{
    // Create Publisher
    this->publisher =
        this->participant->create_publisher(PUBLISHER_QOS_DEFAULT,
                                            0,
                                            OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->publisher)
    {
        ERROR("create publisher \n %s", "failed");
    }

    // Create DataWriter for Status topic
    this->writer_status =
        this->publisher->create_datawriter(this->topic_status,
                                           DATAWRITER_QOS_DEFAULT,
                                           0,
                                           OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->writer_status)
    {
        ERROR("create status datawriter \n %s", "failed");
    }

    this->message_writer_status =
        OledServiceTopic::StatusDataWriter::_narrow(this->writer_status);

    if (!this->message_writer_status)
    {
        ERROR("create narrow status writer \n %s", "failed");
    }

    // Create DataWriter for Text topic
    this->writer_text =
        this->publisher->create_datawriter(this->topic_text,
                                           DATAWRITER_QOS_DEFAULT,
                                           0,
                                           OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (!this->writer_text)
    {
        ERROR("create data text writer \n %s", "failed");
    }

    this->message_writer_text =
        OledServiceTopic::TextDataWriter::_narrow(this->writer_text);

    if (!this->message_writer_text)
    {
        ERROR("create narrow text writer \n %s", "failed");
    }
}

/**
 * @brief  destructer of oled_service_dbus
 * @retval void
 */
oled_service_if::~oled_service_if()
{
    INFO("destructer %s", "oled_service_if");
    this->participant->delete_contained_entities();
    this->dpf->delete_participant(this->participant);
    TheServiceParticipant->shutdown();
}

/*......................................USER FUNCTION......................................*/

/**
 * @brief  the function used to send battery percent to oled
 * @param  percent the battery percent
 * @retval bv_err_return_t
 */
bv_err_return_t oled_service_if::set_battery_percent(uint32_t percent, uint32_t qos)
{
    bv_err_return_t return_code = BV_RETURN_OK;

    // Write data
    this->message_status.subject_id = 5;
    this->message_status.battery_percent = percent;
    this->message_status.type = OLED_BATTERY_PERCENT;
    this->message_status.payload_len = 4;

    if (this->message_writer_status->write(this->message_status, DDS::HANDLE_NIL) != DDS::RETCODE_OK)
    {
        return_code = BV_RETURN_ERROR;
    }

    if (BV_RETURN_ERROR == return_code)
    {
        ERROR("control AP message sending %s", "failed");
    }

    return return_code;
}

/**
 * @brief  the function used to turn on/off access point (display/undisplay access point icon)
 * @param  status the status of access point (on or off)
 * @retval bv_err_return_t
 */
bv_err_return_t oled_service_if::on_off_access_point(uint32_t status, uint32_t qos)
{
    bv_err_return_t return_code = BV_RETURN_OK;

    // Write data
    this->message_status.subject_id = 5;
    this->message_status.access_point = status;
    this->message_status.type = OLED_ACCESS_POINT;
    this->message_status.payload_len = 1;

    if (this->message_writer_status->write(this->message_status, DDS::HANDLE_NIL) != DDS::RETCODE_OK)
    {
        return_code = BV_RETURN_ERROR;
    }

    if (return_code == BV_RETURN_ERROR)
    {
        ERROR("control AP message sending %s", "failed");
    }
    return return_code;
}

/**
 * @brief  the function used to send number of wifi client to oled
 * @param  number_client the number of client
 * @retval bv_err_return_t
 */
bv_err_return_t oled_service_if::set_number_wifi_client(uint32_t number_client, uint32_t qos)
{
    bv_err_return_t return_code = BV_RETURN_OK;

    // Write data
    this->message_status.subject_id = 5;
    this->message_status.num_client = number_client;
    this->message_status.type = OLED_NUM_CLIENT;
    this->message_status.payload_len = 4;

    if (this->message_writer_status->write(this->message_status, DDS::HANDLE_NIL) != DDS::RETCODE_OK)
    {
        return_code = BV_RETURN_ERROR;
    }

    if (return_code == BV_RETURN_ERROR)
    {
        ERROR("number wifi client sending %s", "failed");
    }
    return return_code;
}

/**
 * @brief  the function used to send text message to oled
 * @param  mes the message want to display
 * @param  type type of message we want to send (warning message, service status,...)
 * @retval bv_err_return_t
 */
bv_err_return_t oled_service_if::set_text_message(const char *mes, const char *type, uint32_t qos)
{
    INFO("Text message sent %s", mes);
    bv_err_return_t return_code = BV_RETURN_OK;

    // Write data
    this->message_text.subject_id = 5;
    this->message_text.text_msg = CORBA::string_dup(mes);
    this->message_text.payload_len = strlen(mes);
    this->message_text.type = CORBA::string_dup(type);

    if (this->message_writer_text->write(this->message_text, DDS::HANDLE_NIL) != DDS::RETCODE_OK)
    {
        return_code = BV_RETURN_ERROR;
    }

    if (return_code == BV_RETURN_ERROR)
    {
        ERROR("Text message sending %s", "failed");
    }

    return return_code;
}

/**
 * @brief  the function used to turn on/off screen
 * @param  status the status of screen (on or off)
 * @retval bv_err_return_t
 */
bv_err_return_t oled_service_if::on_off_screen(uint32_t status, uint32_t qos)
{
    char data = status;
    bv_err_return_t return_code = BV_RETURN_OK;
    INFO("Publish message with topic OledServiceTopic::Status %s ", "");

    // Write data
    this->message_status.subject_id = 5;
    this->message_status.display_status = (int)data;
    this->message_status.type = OLED_ON_OFF;
    this->message_status.payload_len = 1;

    if (this->message_writer_status->write(this->message_status, DDS::HANDLE_NIL) != DDS::RETCODE_OK)
    {
        return_code = BV_RETURN_ERROR;
    }

    if (return_code == BV_RETURN_ERROR)
    {
        ERROR("control AP message sending %s", "failed");
    }

    return return_code;
}

/**
 * @brief  the function used to wait for subscriber available
 * @param  writer datawriter
 * @retval bv_err_return_t
 */
bv_err_return_t oled_service_if::wait_for_sub(DDS::DataWriter_var writer)
{
    bv_err_return_t return_code = BV_RETURN_OK;
    // Block until Subscriber is available
    DDS::StatusCondition_var condition = writer->get_statuscondition();
    condition->set_enabled_statuses(DDS::PUBLICATION_MATCHED_STATUS);

    DDS::WaitSet_var ws = new DDS::WaitSet;
    ws->attach_condition(condition);

    while (true)
    {
        DDS::PublicationMatchedStatus matches;
        if (writer->get_publication_matched_status(matches) != DDS::RETCODE_OK)
        {
            ERROR("get_publication_matched_status %s", "failed");
            return_code = BV_RETURN_ERROR;
            break;
        }

        if (matches.current_count >= 1)
        {
            break;
        }

        DDS::ConditionSeq conditions;
        DDS::Duration_t timeout = {60, 0};
        if (ws->wait(conditions, timeout) != DDS::RETCODE_OK)
        {
            ERROR("wait %s", "failed");
            return_code = BV_RETURN_ERROR;
            break;
        }
    }

    ws->detach_condition(condition);

    return return_code;
}

/*......................................DATA READER LISTENER IMPLEMENTATION......................................*/

void oled_service_if::DataReaderListenerImpl::on_requested_deadline_missed(DDS::DataReader_ptr, const DDS::RequestedDeadlineMissedStatus &)
{
    INFO("On requested deadline missed %s ", "");
    oled_service_if::DataReaderListenerImpl::connection_lost(helper, NULL);
}

void oled_service_if::DataReaderListenerImpl::on_requested_incompatible_qos(DDS::DataReader_ptr, const DDS::RequestedIncompatibleQosStatus &)
{
    INFO("On requested incompatible qos %s ", "");
}

void oled_service_if::DataReaderListenerImpl::on_sample_rejected(DDS::DataReader_ptr, const DDS::SampleRejectedStatus &)
{
    INFO("On sample rejected %s ", "");
}

void oled_service_if::DataReaderListenerImpl::on_liveliness_changed(DDS::DataReader_ptr, const DDS::LivelinessChangedStatus &)
{
    INFO("On liveliness changed %s ", "");
}

void oled_service_if::DataReaderListenerImpl::on_data_available(DDS::DataReader_ptr reader)
{
    OledServiceTopic::StatusDataReader_var reader_status =
        OledServiceTopic::StatusDataReader::_narrow(reader);

    OledServiceTopic::TextDataReader_var reader_text =
        OledServiceTopic::TextDataReader::_narrow(reader);

    if (reader_status)
    {
        OledServiceTopic::Status message_status;
        DDS::SampleInfo info_status;
        const DDS::ReturnCode_t error_status = reader_status->take_next_sample(message_status, info_status);

        if (error_status == DDS::RETCODE_OK)
        {
            if (info_status.valid_data)
            {
                INFO("Received status message");
                OledServiceTopic::Oled topic;
                topic.status_data = message_status;
                oled_service_if::DataReaderListenerImpl::message_arrived(helper, message_status.type._retn(), message_status.payload_len, &topic);
            }
        }
    }
    else if (reader_text)
    {
        OledServiceTopic::Text message_text;
        DDS::SampleInfo info_text;
        const DDS::ReturnCode_t error_text = reader_text->take_next_sample(message_text, info_text);

        if (error_text == DDS::RETCODE_OK)
        {
            if (info_text.valid_data)
            {
                INFO("Received text message");
                OledServiceTopic::Oled topic;
                topic.text_data = message_text;
                oled_service_if::DataReaderListenerImpl::message_arrived(helper, message_text.type, message_text.payload_len, &topic);
            }
        }
    }
    else
    {
        ERROR("wait %s", "failed");
    }
}

void oled_service_if::DataReaderListenerImpl::on_subscription_matched(DDS::DataReader_ptr, const DDS::SubscriptionMatchedStatus &)
{
    INFO("On subscription matched %s ", "");
}

void oled_service_if::DataReaderListenerImpl::on_sample_lost(DDS::DataReader_ptr, const DDS::SampleLostStatus &)
{
    INFO("On sample lost %s ", "");
}