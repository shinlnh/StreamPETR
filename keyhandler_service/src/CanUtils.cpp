#include "CanUtils.h"

#define EPS_MANUAL_CONTROL_FRAME_ID (0x119)

can_frame packDrivingMode(int mode, int aeb) {
    // create CAN frame
    can_frame frame;
    frame.can_id = TESLA_DAS_STATUS_FRAME_ID;
    frame.can_dlc = TESLA_DAS_STATUS_LENGTH;
    struct tesla_das_status_t das_status;
    switch (mode) {
        case IDLE_STATE_ID:
            das_status.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_IDLE_CHOICE;
            break;
        case ACC_STATE_ID:
            das_status.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_ACC_ACTIVE_CHOICE;
            break;
        case LKS_STATE_ID:
            das_status.das_autopilot_state = TESLA_DAS_STATUS_DAS_AUTOPILOT_STATE_LKA_ACTIVE_CHOICE;
            break;
        default:
            das_status.das_autopilot_state = mode;
            break;
    }

    das_status.das_aeb_state = aeb;
    tesla_das_status_pack(frame.data, &das_status, frame.can_dlc);
    
    // send msg
    return frame;
}

can_frame packThrottleBrakeGear(double throttle, double brake, uint8_t gear) {
    // create CAN frame
    can_frame frame;
    frame.can_id = EPS_MANUAL_CONTROL_FRAME_ID;
    frame.can_dlc = TESLA_DI_DRIVER_SYSTEM_STATUS_LENGTH;

    // create msg
    tesla_di_driver_system_status_t msg;
    msg.di_accel_pedal_pos = tesla_di_driver_system_status_di_accel_pedal_pos_encode(throttle * 100);
    msg.di_gear = gear;
    msg.di_brake_pedal_state = tesla_di_driver_system_status_di_brake_pedal_state_encode(brake * 100);
    tesla_di_driver_system_status_pack(frame.data, &msg, frame.can_dlc);
    
    return frame;
}

can_frame packSteer(double steer) {
    // create CAN frame
    can_frame frame;
    frame.can_id = TESLA_STEERING_ANGLE_FRAME_ID;
    frame.can_dlc = TESLA_STEERING_ANGLE_LENGTH;

    // create msg
    tesla_steering_angle_t msg;
    msg.steering_angle = tesla_steering_angle_steering_angle_encode(steer * 450.5325);
    tesla_steering_angle_pack(frame.data, &msg, frame.can_dlc);
    
    // send msg
    return frame;
}

can_frame packAccFollowingDistance(double value) {
    // create CAN frame
    can_frame frame;
    frame.can_id = TESLA_UI_DRIVER_ASSIST_CONTROL_FRAME_ID;
    frame.can_dlc = TESLA_UI_DRIVER_ASSIST_CONTROL_LENGTH;

    // create msg
    tesla_ui_driver_assist_control_t msg;
    msg.ui_acc_follow_distance_setting = tesla_ui_driver_assist_control_ui_acc_follow_distance_setting_encode(value);
    tesla_ui_driver_assist_control_pack(frame.data, &msg, frame.can_dlc);
    
    // send msg
    return frame;
}

can_frame packAccSpeedLimit(double value) {
    // create CAN frame
    can_frame frame;
    frame.can_id = TESLA_DAS_STATUS2_FRAME_ID;
    frame.can_dlc = TESLA_DAS_STATUS2_LENGTH;

    // create msg
    tesla_das_status2_t msg;
    msg.das_acc_speed_limit = tesla_das_status2_das_acc_speed_limit_encode(value);
    tesla_das_status2_pack(frame.data, &msg, frame.can_dlc);
    
    // send msg
    return frame;
}