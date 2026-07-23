#include "key_handler.h"

#define ZERO_THRESH 0.01f

static constexpr inline bool isZero(float x) { return std::abs(x) <= ZERO_THRESH; }
static constexpr inline bool notZero(float x) { return std::abs(x) > ZERO_THRESH; }

KeyPressHandler::KeyPressHandler(QObject *parent, MainViewController *controller, AdasMainVisualize *adas_main_visualize_ptr_instace) 
    : QObject(parent), adas_ptr(controller), adas_main_visualize_ptr(adas_main_visualize_ptr_instace)
{
    timer.setInterval(100); // adjust the interval as needed
    connect(&timer, &QTimer::timeout, this, [this]() {
        this->adjustDirection();
        this->vehicleControlUnit(); 
    });
    timer.start();
}

bool KeyPressHandler::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        int key = keyEvent->key();

        // Check if the key is one of W, A, S, D, Q, G, Space keys
        // Automatically data to GUI
        if (key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_W 
            || key == Qt::Key_1 || key == Qt::Key_2 || key == Qt::Key_3 || key == Qt::Key_4)
        {
            if (!pressedKeys.contains(key)) {
                pressedKeys.insert(key);
                emit keysChanged(pressedKeys);
            }
        }
    } else if (event->type() == QEvent::KeyRelease && static_cast<QKeyEvent *>(event)->isAutoRepeat() == false) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        int key = keyEvent->key();

        // Check if the key is one of ASDW keys
        if (key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_W) {
            pressedKeys.remove(key);
            emit keysChanged(pressedKeys);
        }
    }
    return false;
}

/**
 * @brief Handle the key pressed from keyboard when running in DDS mode
 * @details Implements VCU logic with gear shift interlock (2 km/h threshold)
 */
void KeyPressHandler::adjustDirection()
{
    int mode_id = adas_main_visualize_ptr->getStateID();
    float car_speed = adas_main_visualize_ptr->getSpeed();
    if (pressedKeys.size() == 0) {
        if (!reset) {
            return;
        }
        // Reset keyhandler values
        m_thottleSlider = 0.0f;
        m_brakeSlider = 0.0f;
        m_steerSlider = 0.0f;

        // Automatically data to GUI
        adas_ptr->setSteerSlider(m_steerSlider);
        adas_ptr->setThottleSlider(m_thottleSlider);
        adas_ptr->setBrakeSlider(m_brakeSlider);

        // Reset cached control message
        if (mode_id != -1) {
            if (mode_id != LKS_STATE_ID && mode_id != HWC_STATE_ID && mode_id != TJC_STATE_ID) {
                adas_main_visualize_ptr->doSteer(m_steerSlider);
            }
            if (mode_id != ACC_STATE_ID && mode_id != TJA_STATE_ID &&
                mode_id != HWC_STATE_ID && mode_id != TJC_STATE_ID &&
                mode_id != BRAKE_STATE_ID)
            {
                adas_main_visualize_ptr->doThrottle(m_thottleSlider);
                adas_main_visualize_ptr->doBrake(m_brakeSlider);
            }
        }
        reset = false;
        return;
    }
    reset = true;

    if (pressedKeys.contains(Qt::Key_W))
        m_thottleSlider = std::min(m_thottleSlider + 0.1f, 1.0f);
    else
        m_thottleSlider = 0.0f;

    if (pressedKeys.contains(Qt::Key_S))
        m_brakeSlider = std::min(m_brakeSlider + 0.2f, 1.0f);
    else
        m_brakeSlider = 0.0f;

    static constexpr float steer_increment = 0.03f;
    if (pressedKeys.contains(Qt::Key_A)) {
        if (m_steerSlider > 0.0f)
            m_steerSlider = 0.0f;
        else
            m_steerSlider -= steer_increment;
    } else if (pressedKeys.contains(Qt::Key_D)) {
        if (m_steerSlider < 0.0f)
            m_steerSlider = 0.0f;
        else
            m_steerSlider += steer_increment;
    } else
        m_steerSlider = 0.0f;
    m_steerSlider = clamp(m_steerSlider, -1.0f, 1.0f);

    // printDirectionState();
    
    // ======================================================================
    // VCU Logic: Park Gear Safety - Cut throttle and brake control
    // When vehicle is in Park, no throttle/brake control should be sent
    // This matches EPS VCU behavior (similar to lines 445-450 in EPS VCU)
    // ======================================================================
    GEAR current_gear = adas_main_visualize_ptr->getGear();
    
    if (mode_id != -1) {
        if (mode_id != LKS_STATE_ID && mode_id != HWC_STATE_ID && mode_id != TJC_STATE_ID) {
            adas_main_visualize_ptr->doSteer(m_steerSlider);
            // Automatically data to GUI
            adas_ptr->setSteerSlider(m_steerSlider);
        }
        if (mode_id != ACC_STATE_ID && mode_id != TJA_STATE_ID &&
            mode_id != HWC_STATE_ID && mode_id != TJC_STATE_ID &&
            mode_id != BRAKE_STATE_ID)
        {
            // VCU Safety: Cut throttle and brake in Park gear
            if (current_gear == GEAR::P) {
                // Park gear: No throttle or brake control
                adas_main_visualize_ptr->doThrottle(0.0f);
                adas_main_visualize_ptr->doBrake(0.0f);
            }
            else {
                // Other gears: Normal control
                adas_main_visualize_ptr->doThrottle(m_thottleSlider);
                adas_main_visualize_ptr->doBrake(m_brakeSlider);
            }
            // Automatically data to GUI
            adas_ptr->setThottleSlider(m_thottleSlider);
            adas_ptr->setBrakeSlider(m_brakeSlider);
        }
    }
    else {
        // Automatically data to GUI
        adas_ptr->setThottleSlider(m_thottleSlider);
        adas_ptr->setSteerSlider(m_steerSlider);
        adas_ptr->setBrakeSlider(m_brakeSlider);
    }


    // ======================================================================
    // VCU Logic: Gear Shift Interlock (similar to EPS VCU)
    // Safety threshold: 2 km/h (matching EPS implementation)
    // ======================================================================
    static constexpr float GEAR_SHIFT_SPEED_THRESHOLD = 2.0f; // km/h
    // current_gear already declared above
    float abs_speed = std::abs(car_speed);
    bool is_moving_forward = (car_speed > GEAR_SHIFT_SPEED_THRESHOLD);
    bool is_moving_reverse = (car_speed < -GEAR_SHIFT_SPEED_THRESHOLD);

    
    if (pressedKeys.contains(Qt::Key_1)) {
        // Park gear request - only if vehicle is nearly stopped
        if (abs_speed <= GEAR_SHIFT_SPEED_THRESHOLD) {
            adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::P));
        }
        // Reject if moving too fast (safety interlock)
        pressedKeys.remove(Qt::Key_1);
    }

    if (pressedKeys.contains(Qt::Key_2)) {
        // Reverse gear request
        if (current_gear == GEAR::D) {
            // D -> R: Only if nearly stopped
            if (abs_speed <= GEAR_SHIFT_SPEED_THRESHOLD) {
                adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::R));
            }
        }
        else if (current_gear == GEAR::N) {
            // N -> R: Block if moving forward
            if (!is_moving_forward) {
                adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::R));
            }
        }
        else if (current_gear == GEAR::P) {
            // P -> R: Always allowed if nearly stopped
            if (abs_speed <= GEAR_SHIFT_SPEED_THRESHOLD) {
                adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::R));
            }
        }
        // R -> R: Do nothing (already in R)
        
        pressedKeys.remove(Qt::Key_2);
    }

    if (pressedKeys.contains(Qt::Key_3)) {
        // Neutral - always allowed (emergency escape gear)
        adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::N));
        pressedKeys.remove(Qt::Key_3);
    }
    
    if (pressedKeys.contains(Qt::Key_4)) {
        // Drive gear request
        if (current_gear == GEAR::R) {
            // R -> D: Only if nearly stopped
            if (abs_speed <= GEAR_SHIFT_SPEED_THRESHOLD) {
                adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::D));
            }
        }
        else if (current_gear == GEAR::N) {
            // N -> D: Block if moving reverse
            if (!is_moving_reverse) {
                adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::D));
            }
        }
        else if (current_gear == GEAR::P) {
            // P -> D: Always allowed if nearly stopped
            if (abs_speed <= GEAR_SHIFT_SPEED_THRESHOLD) {
                adas_main_visualize_ptr->doGear(static_cast<int>(GEAR::D));
            }
        }
        // D -> D: Do nothing (already in D)
        
        pressedKeys.remove(Qt::Key_4);
    }
}

void KeyPressHandler::printDirectionState()
{
    INFO("Throttle: %f; Steer: %f; Brake: %f ", m_thottleSlider, m_steerSlider, m_brakeSlider);
}

/**
 * @brief VCU Logic: AEB takeover and ADAS cancellation (similar to EPS VCU)
 * @details Implements automotive-grade logic:
 *          - AEB Takeover: Gear P/R or throttle release-press cycle
 *          - ADAS Cancellation: Gear change from D, brake pedal, throttle override
 *          
 *          Note: This function does NOT implement 2-second hold logic
 *          because ADAS Service already handles it (policy_state_machine.cpp).
 *          Client only detects takeover conditions and sends request.
 */
void KeyPressHandler::vehicleControlUnit()
{
    // ======================================================================
    // Static variables for state machine tracking
    // ======================================================================
    static bool prev_brake_state = false;
    static bool prev_adas_active = false;
    static bool throttle_released_after_aeb = false;
    static GEAR prev_gear = GEAR::P;
    
    // ======================================================================
    // Retrieve current vehicle/ADAS status
    // ======================================================================
    int mode_id = adas_main_visualize_ptr->getStateID();
    GEAR current_gear = adas_main_visualize_ptr->getGear();
    
    bool brake_state_active = (mode_id == BRAKE_STATE_ID);
    bool adas_active = (mode_id == ACC_STATE_ID || mode_id == TJA_STATE_ID ||
                        mode_id == HWC_STATE_ID || mode_id == TJC_STATE_ID ||
                        mode_id == LKS_STATE_ID);
    
    // ======================================================================
    // 1. AEB (Emergency Brake) Takeover Detection
    // ======================================================================
    // Edge detection: AEB just became active (0→1 transition)
    if (brake_state_active && !prev_brake_state)
    {
        // AEB just activated - reset throttle release flag
        // Driver must release throttle from this point forward
        throttle_released_after_aeb = false;
    }
    
    bool should_cancel_aeb = false;
    
    if (brake_state_active)
    {
        // AEB Takeover Condition 1: Gear shift to Park or Reverse
        if (current_gear == GEAR::P || current_gear == GEAR::R)
        {
            should_cancel_aeb = true;
        }
        
        // AEB Takeover Condition 2: Release throttle, then press again
        // State machine: Wait for release → Then detect press
        // Thresholds match EPS VCU scaled to 0-1 range
        static constexpr float THROTTLE_RELEASE_THRESHOLD = 0.06f;  // 6% (similar to 15/250 in EPS)
        static constexpr float THROTTLE_PRESS_THRESHOLD = 0.12f;    // 12% (similar to 30/250 in EPS)
        
        // State 1: Wait for throttle release
        if (!throttle_released_after_aeb && m_thottleSlider < THROTTLE_RELEASE_THRESHOLD)
        {
            // Driver released throttle to near-zero
            throttle_released_after_aeb = true;
        }
        
        // State 2: After release, detect throttle press
        if (throttle_released_after_aeb && m_thottleSlider > THROTTLE_PRESS_THRESHOLD)
        {
            // Driver released throttle AND is now pressing → Override AEB
            should_cancel_aeb = true;
        }
        
        // Send AEB takeover signal to ADAS service if conditions met
        if (should_cancel_aeb)
        {
            adas_main_visualize_ptr->sendAEBTakeover();
            // Note: Service will check 2-second hold requirement before actually releasing
        }
    }
    else
    {
        // AEB not active - reset state machine
        throttle_released_after_aeb = false;
    }
    
    // ======================================================================
    // 2. ADAS C ancellation (Control Takeover)
    // ======================================================================
    bool should_cancel_adas = false;
    
    if (adas_active)
    {
        // Condition 1: Gear shift FROM Drive to any other gear
        if (prev_gear == GEAR::D && current_gear != GEAR::D)
        {
            should_cancel_adas = true;
        }
        
        // Condition 2: Brake pedal pressed above threshold
        static constexpr float BRAKE_PRESSED_THRESHOLD = 0.1f; // 10%
        if (m_brakeSlider > BRAKE_PRESSED_THRESHOLD)
        {
            should_cancel_adas = true;
        }
        
        // Condition 3: Throttle override (for longitudinal control modes)
        bool adas_longitudinal = (mode_id == ACC_STATE_ID || mode_id == TJA_STATE_ID ||
                                  mode_id == HWC_STATE_ID || mode_id == TJC_STATE_ID);
        if (adas_longitudinal)
        {
            // Detect significant throttle input as override
            static constexpr float THROTTLE_OVERRIDE_THRESHOLD = 0.3f; // 30%
            if (m_thottleSlider > THROTTLE_OVERRIDE_THRESHOLD)
            {
                should_cancel_adas = true;
            }
        }
        
        // Note: Steering override (torque-based) is detected by ADAS service
        // Client doesn't need to handle steering takeover detection
        
        if (should_cancel_adas)
        {
            // Cancel ADAS by disabling control features
            adas_main_visualize_ptr->setValue(STATUS_BTN_LANE_KEEPING_SYSTEM, false);
            adas_main_visualize_ptr->setValue(STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, false);
        }
    }
    
    // ======================================================================
    // Update previous state for next cycle edge detection
    // ======================================================================
    prev_brake_state = brake_state_active;
    prev_adas_active = adas_active;
    prev_gear = current_gear;
}