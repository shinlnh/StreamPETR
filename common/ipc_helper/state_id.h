#ifndef __STATE_ID__
#define __STATE_ID__

enum PolicyStateID : int {
    IDLE_STATE_ID = 1,
    BRAKE_STATE_ID = 2,
    LKS_STATE_ID = 3,
    TJA_STATE_ID = 4,
    ACC_STATE_ID = 5,
    HWC_STATE_ID = 6,
    TJC_STATE_ID = 7
};

#endif // __STATE_ID__