#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include "can_service_if.h"
#include "can_manager.h"
#include "tesla.h"
#include <message_filters/simple_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#define MAX_STEERING_WHEEL 37.0f  // unit: deg


/*Class define ADAS Can service*/
class can_service
{
private:
    CanServiceIF can_if;
    CanManager can_manager;
    std::queue<can_frame> msg_queue;

    // Threading
    std::atomic<bool> allow_run_threads;
    std::mutex mux_mutex;
    std::thread mux_thread;
    std::condition_variable cond_mux_thread;
    void canMUXThread();

    // Subscriber pass through class for use with message filter
    template<typename MsgT>
    class SubPassThrough : public message_filters::SimpleFilter<MsgT> {
    public:
        void operator()(const std::shared_ptr<const MsgT>& msg) {
            this->signalMessage(msg);
        }
    };

    SubPassThrough<ipc_helper::msg::ImuParameters> IMU_PassThr;
    SubPassThrough<ipc_helper::msg::Odometry> ODO_PassThr;

    using ImU_ODO_SyncPolicy = 
        message_filters::sync_policies::ExactTime<ipc_helper::msg::ImuParameters,
                                                  ipc_helper::msg::Odometry>;
    message_filters::Synchronizer<ImU_ODO_SyncPolicy> IMU_ODO_Sync;

public:
    can_service(const char *client_id, uint32_t domain_id, const char *device);
    ~can_service();
    void frameQueuePush(const can_frame& frame);
    void gatewayMode();

    void cbCarStatus(const ipc_helper::msg::CarStatus::ConstSharedPtr msg);
    void cbIMU_ODO(const ipc_helper::msg::ImuParameters::ConstSharedPtr IMU_msg,
                   const ipc_helper::msg::Odometry::ConstSharedPtr ODO_msg);
};


#endif
