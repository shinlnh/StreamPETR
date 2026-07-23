#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include "tesla.h"

#define IDLE_STATE_ID   1
#define BRAKE_STATE_ID  2
#define LKS_STATE_ID    3
#define TJA_STATE_ID    4
#define ACC_STATE_ID    5
#define HWC_STATE_ID    6
#define TJC_STATE_ID    7

can_frame packDrivingMode(int mode, int aeb = TESLA_DAS_STATUS_DAS_AEB_STATE_AEB_DISABLE_CHOICE);
can_frame packThrottleBrakeGear(double throttle, double brake, uint8_t gear = 0);
can_frame packSteer(double steer);
can_frame packAccFollowingDistance(double value);
can_frame packAccSpeedLimit(double value);