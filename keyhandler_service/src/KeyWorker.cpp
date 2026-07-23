#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "common.h"
#include "CanUtils.h"
#include "tesla.h"
#include "KeyWorker.h"

extern volatile float m_thottleSlider;
extern volatile float m_brakeSlider;
extern volatile float m_steerSlider;

KeyWorker::KeyWorker(int code, CanManager *canManager) : keyCode(code), canManager(canManager) {
    keyName = getKeyName(keyCode);
    processThread = std::thread(&KeyWorker::process, this);
    repeatThread = std::thread(&KeyWorker::autoRepeatLoop, this);
}

KeyWorker::~KeyWorker() {
    running = false;
    autoRepeating = false;

    // Push dummy event to wake up thread if waiting
    eventQueue.push(KeyEvent{keyCode, 0, {}});
    if (processThread.joinable())
        processThread.join();

    repeatCv.notify_all();
    if (repeatThread.joinable())
        repeatThread.join();
}

void KeyWorker::pushEvent(const KeyEvent& ev) {
    eventQueue.push(ev);
}

void KeyWorker::process() {
    while (running) {
        KeyEvent ev = eventQueue.wait_and_pop();

        if (!running) break;

        if (ev.value == 1) { // Pressed
            pressed = true;
            lastPressTime = ev.timestamp;
            DEBUG("%s pressed", keyName.c_str());
            if (pressed && !autoRepeating) {
                startAutoRepeat();
            }
        }
        else if (ev.value == 2) { // Held
            if (pressed && !autoRepeating) {
                DEBUG("%s held", keyName.c_str());
                startAutoRepeat();
            }
        }
        else if (ev.value == 0) { // Released
            if (pressed) {
                pressed = false;
                stopAutoRepeat();
                long durationMs = (ev.timestamp.tv_sec - lastPressTime.tv_sec) * 1000 +
                                  (ev.timestamp.tv_usec - lastPressTime.tv_usec) / 1000;
                DEBUG("%s released after %ld", keyName.c_str(), durationMs);
            }
        }
        adjustDirection();
    }
}

void KeyWorker::autoRepeatLoop() {
    while (running) {
        std::unique_lock<std::mutex> lock(repeatMutex);
        repeatCv.wait(lock, [this] {
            return !running || autoRepeating;
        });

        // Exit condition
        if (!running) break;

        lock.unlock();

        while (autoRepeating && pressed && running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (autoRepeating && pressed && running) {
                DEBUG("%s auto-repeat increment", keyName.c_str());
                // You can also push an event or increment a counter here
                adjustDirection();
            }
        }
    }
}

void KeyWorker::startAutoRepeat() {
    autoRepeating = true;
    repeatCv.notify_one(); // Wake up the repeat thread
}

void KeyWorker::stopAutoRepeat() {
    autoRepeating = false;
    // The repeat thread will exit its inner loop naturally
}

void KeyWorker::adjustDirection() {
    float steer_increment = 1.5e-4f * 200;

    switch (keyCode) {
        case KEY_W:
            if (pressed)
                m_thottleSlider = std::min(m_thottleSlider + 0.1f, 1.00f);
            else
                m_thottleSlider = 0.0f;
            break;
        
        case KEY_S:
            canManager->sendCanFrame(packDrivingMode(IDLE_STATE_ID));
            if (pressed)
                m_brakeSlider = std::min(m_brakeSlider + 0.2f, 1.0f);
            else
                m_brakeSlider = 0.0f;
            break;
        
        case KEY_A:
            if (pressed) {
                if (m_steerSlider > 0)
                    m_steerSlider = 0;
                else
                    m_steerSlider -= steer_increment;
            }
            else
                m_steerSlider = 0;
            m_steerSlider = static_cast<volatile float>(std::min(1.0f, std::max(-1.0f, static_cast<float>(m_steerSlider))));
            break;
        
        case KEY_D:
            if (pressed) {
                if (m_steerSlider < 0)
                    m_steerSlider = 0;
                else
                    m_steerSlider += steer_increment;
            }
            else
                m_steerSlider = 0;
            m_steerSlider = static_cast<volatile float>(std::min(1.0f, std::max(-1.0f, static_cast<float>(m_steerSlider))));
            break;

        case KEY_Q:
            if (!pressed) {
                // PRND values are 1, 2, 3, and 4, so we use 5 to indicate that this is a request to the Gateway (GW).
                // Then, GW will receive the status from ADAS and update the gear status on the cluster accordingly.
                canManager->sendCanFrame(packThrottleBrakeGear(m_thottleSlider, m_brakeSlider, 5));
            }
            break;
        
        case KEY_0:
            if (!pressed) {
                canManager->sendCanFrame(packDrivingMode(IDLE_STATE_ID));
            }
            break;

        case KEY_1:
            if (!pressed) {
                canManager->sendCanFrame(packDrivingMode(LKS_STATE_ID));
            }
            break;
        
        case KEY_2:
            if (!pressed) {
                canManager->sendCanFrame(packDrivingMode(ACC_STATE_ID));
            }
            break;

        case KEY_6:
            if (!pressed) {
                // The value 10 here is just a work around to make the Gateway know that this is a request for AEB only
                canManager->sendCanFrame(packDrivingMode(10, TESLA_DAS_STATUS_DAS_AEB_STATE_AEB_ENABLE_CHOICE));
            }
            break;

        case KEY_UP:
            if (pressed) {
                canManager->sendCanFrame(packAccFollowingDistance(1));
            }
            break;

        case KEY_DOWN:
            if (pressed) {
                canManager->sendCanFrame(packAccFollowingDistance(-1));
            }
            break;

        case KEY_LEFT:
            if (pressed) {
                canManager->sendCanFrame(packAccSpeedLimit(-1));
            }
            break;

        case KEY_RIGHT:
            if (pressed) {
                canManager->sendCanFrame(packAccSpeedLimit(1));
            }
            break;

        case KEY_ENTER:
            if (!pressed) {
                // Workaround: press this to notify the Gateway (GW) that we want to enable capture for the ADAS service.
                canManager->sendCanFrame(packThrottleBrakeGear(m_thottleSlider, m_brakeSlider, 6));
            }
            break;

        default:
            break;
    }
}