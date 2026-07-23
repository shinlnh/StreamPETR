#include "can_service.h"
#include <unistd.h>


static inline constexpr GEAR convertCanAdasGear(int canGear)
{
    switch (canGear) {
        case TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_P_CHOICE:
            return GEAR::P;
        case TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_R_CHOICE:
            return GEAR::R;
        case TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_N_CHOICE:
            return GEAR::N;
        case TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_D_CHOICE:
            return GEAR::D;
        default:
            return GEAR::COUNT; // Return COUNT to indicate invalid gear
    }
}


/**
 * @brief  constructor function
 * @param  client_id
 * @param  server_address
 * @retval none
 */
can_service::can_service(const char *client_id, uint32_t domain_id, const char *device) 
: can_if("hero0")
, can_manager(device)
, IMU_PassThr()
, ODO_PassThr()
, IMU_ODO_Sync(ImU_ODO_SyncPolicy(10), IMU_PassThr, ODO_PassThr)
{
    INFO("Initial %s service", "CAN");

    // Register RoS callbacks
    can_if.getTopicCarStatusHandler()->registerCallback([this](auto msg){ this->cbCarStatus(msg); });
    can_if.getTopicIMUHandler()->registerCallback([this](auto msg){ this->IMU_PassThr(msg); });
    can_if.getTopicOdometerHandler()->registerCallback([this](auto msg){ this->ODO_PassThr(msg); });

    // Synchronizer callbacks
    IMU_ODO_Sync.registerCallback(&can_service::cbIMU_ODO, this);

    // Register CAN messages
    can_manager.registerCallback({TESLA_EPS_DRIVER_CONTROL_FRAME_ID}, [this](const can_frame& frame) {
        frameQueuePush(frame);
    });

    // Start CAN MUX thread
    allow_run_threads = true;
    mux_thread = std::thread(&can_service::canMUXThread, this);
}


/**
 * @brief  Destructer function
 * @retval none
 */
can_service::~can_service()
{
    INFO("Exit %s service", "CAN");
    /*cancel thread*/
    allow_run_threads = false;
    cond_mux_thread.notify_one();
    if (mux_thread.joinable()) {
        mux_thread.join();
    }
}


void can_service::frameQueuePush(const can_frame& frame)
{
    {
        std::lock_guard<std::mutex> mux_lock(mux_mutex);
        msg_queue.push(frame);
    }
    // Notify the processing thread that the queue is not empty.
    cond_mux_thread.notify_one();
}


void can_service::canMUXThread()
{
    std::unique_lock<std::mutex> mux_lock(mux_mutex, std::defer_lock);
    while (1)
    {
        mux_lock.lock();
        // Wait until the queue is not empty or `allow_run_threads` is false.
        cond_mux_thread.wait(mux_lock, [this] {
            return !msg_queue.empty() || !allow_run_threads;
        });

        // If allow_run_threads is false, close the thread.
        if (!allow_run_threads)
        {
            break;
        }

        // Retrieve the CAN frame from queue
        can_frame frame = msg_queue.front();
        msg_queue.pop();
        mux_lock.unlock(); // Unlock queue mutex to allow other threads to push new messages.
        
        // Handle the CAN message
        switch (frame.can_id)
        {
            case TESLA_EPS_DRIVER_CONTROL_FRAME_ID:
            {
                // Unpack
                tesla_eps_driver_control_t driver_system_status;
                tesla_eps_driver_control_unpack(&driver_system_status, frame.data, frame.can_dlc);

                // Get value
                float throttle = tesla_eps_driver_control_eps_accel_pedal_pos_decode(driver_system_status.eps_accel_pedal_pos);
                float brake    = tesla_eps_driver_control_eps_brake_pedal_pos_decode(driver_system_status.eps_brake_pedal_pos);
                float steer    = tesla_eps_driver_control_eps_steering_angle_decode(driver_system_status.eps_steering_angle);
                int gear       = tesla_eps_driver_control_eps_gear_lever_pos_decode(driver_system_status.eps_gear_lever_pos);

                // Gear Signal Processing: Debounce + Validation + Hold Last State
                
                // 1. Debouncing: Requires 'GEAR_STABLE_THRESHOLD' consecutive frames
                static int current_output_gear = -1; // Initial state (-1 = Unknown)
                static int stable_counter = 0;
                static int last_raw_can_gear = -1;
                const int GEAR_STABLE_THRESHOLD = 3; 

                if (gear == last_raw_can_gear) {
                    if (stable_counter < GEAR_STABLE_THRESHOLD) {
                        stable_counter++;
                    }
                } else {
                    stable_counter = 0; // Signal changed, reset counter
                    last_raw_can_gear = gear;
                }

                // 2. Validation & Update: Only update if stable AND value is strictly valid
                // Ignore 0 (Invalid), transitional states, or undefined values.
                // This implements "Hold Last State" behavior.
                if (stable_counter >= GEAR_STABLE_THRESHOLD) {
                    bool isValidGear = (gear == TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_P_CHOICE ||
                                        gear == TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_R_CHOICE ||
                                        gear == TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_N_CHOICE ||
                                        gear == TESLA_DI_DRIVER_SYSTEM_STATUS_DI_GEAR_DI_GEAR_D_CHOICE);
                    
                    if (isValidGear) {
                        current_output_gear = gear;
                    } else {
                        // Log warning for invalid stable gear (optional, to debug hardware issues)
                        if (gear != 0) { // 0 is common invalid, maybe skip log to avoid spam
                             // DEBUG("Ignored invalid stable gear: %d", gear);
                        }
                    }
                }

                // 3. Output Generation
                // If current_output_gear is still -1 (Startup, no valid gear yet), 
                // convertCanAdasGear(-1) will return GEAR::COUNT (Invalid), result in -1 sent.
                
                ipc_helper::msg::Control control_msg;
                control_msg.throttle = throttle / 100;
                control_msg.brake    = brake / 100;
                control_msg.steer    = steer / MAX_STEERING_WHEEL;
                
                GEAR adasGear = convertCanAdasGear(current_output_gear);
                if (adasGear == GEAR::COUNT) {
                    // Gear hasn't been established yet (Startup phase or constant invalid signals).
                    // Sending -1 would cause ROS Bridge to trigger Reverse (Unsafe).
                    // Safer to NOT send any control command until we have a confirmed Gear state.
                    WARN("Gear state unknown (Input: %d). Skipping control publish.", current_output_gear);
                } else {
                    control_msg.gear = static_cast<int>(adasGear);
                    
                    // Send Carla control message
                    can_if.publishControls(control_msg);
                    DEBUG("throttle: %f", throttle);
                    DEBUG("brake: %f", brake);
                    DEBUG("steer: %f",steer);
                    DEBUG("Send message success");
                }
                break;
            }
            default:
                break;
        }
    }
    INFO("Exit canMUXThread");
}


void can_service::cbCarStatus(const ipc_helper::msg::CarStatus::ConstSharedPtr msg)
{
    // Retrieve data from ROS message
    DEBUG("Steering: %f", msg->steering);
    float wheel_angle = msg->steering / 180 * M_PI;
    int gear = msg->gear + 1;

    // create CAN frame
    can_frame frame;
    frame.can_id = TESLA_SS_CHASSIS_STATUS_FRAME_ID;
    frame.can_dlc = TESLA_SS_CHASSIS_STATUS_LENGTH;

    // encode data
    tesla_ss_chassis_status_t wheel_angle_msg;
    wheel_angle_msg.ss_wheel_angles = tesla_ss_chassis_status_ss_wheel_angles_encode(wheel_angle);
    wheel_angle_msg.ss_gear_lever = tesla_ss_chassis_status_ss_gear_lever_encode(gear);
    tesla_ss_chassis_status_pack(frame.data, &wheel_angle_msg, frame.can_dlc);

    // send CAN frame
    can_manager.sendCanFrame(frame);
}


void can_service::cbIMU_ODO(const ipc_helper::msg::ImuParameters::ConstSharedPtr IMU_msg,
                            const ipc_helper::msg::Odometry::ConstSharedPtr ODO_msg)
{
    // Retrieve data from ROS message
    DEBUG("IMU Acc: %f", IMU_msg->imu_linear_acceleration.x);
    DEBUG("Linear Vel X: %f", ODO_msg->linear_velocity.x);
    DEBUG("Linear Vel Y: %f", ODO_msg->linear_velocity.y);
    
    float lateral_accel = IMU_msg->imu_linear_acceleration.y;
    float longitudinal_accel = IMU_msg->imu_linear_acceleration.x;
    float longitudinal_velocity = ODO_msg->linear_velocity.y;
    float lateral_velocity = ODO_msg->linear_velocity.x;
    float yaw_rate = ODO_msg->angular_velocity.z;
    
    // create CAN frame
    can_frame frame;
    frame.can_id = TESLA_SS_STATE_MOTION_FRAME_ID;
    frame.can_dlc = TESLA_SS_STATE_MOTION_LENGTH;

    // encode data
    tesla_ss_state_motion_t state_msg;
    state_msg.ss_lateral_accel = tesla_ss_state_motion_ss_lateral_accel_encode(lateral_accel);
    state_msg.ss_longitudinal_accel = tesla_ss_state_motion_ss_longitudinal_accel_encode(longitudinal_accel);
    state_msg.ss_longitudinal_velocity = tesla_ss_state_motion_ss_longitudinal_velocity_encode(longitudinal_velocity);
    state_msg.ss_lateral_velocity = tesla_ss_state_motion_ss_lateral_velocity_encode(lateral_velocity);
    state_msg.ss_yaw_rate = tesla_ss_state_motion_ss_yaw_rate_encode(yaw_rate);
    tesla_ss_state_motion_pack(frame.data, &state_msg, frame.can_dlc);

    // send CAN frame
    can_manager.sendCanFrame(frame);
}