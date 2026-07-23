#pragma once

#include <cstdint>

typedef struct {
    float brake;
    int32_t gear;
    bool hand_brake;
    bool manual_gear_shift;
    bool reverse;
    float steer;
    float throttle;
} control_msg;

