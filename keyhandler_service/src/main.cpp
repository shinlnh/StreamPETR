#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <unordered_map>
#include <memory>
#include <linux/input.h>
#include <termios.h>

#include "CanUtils.h"
#include "can_manager.h"
#include "tesla.h"
#include "InputDeviceUtils.h"
#include "KeyWorker.h"

volatile float m_thottleSlider = 0.0;
volatile float m_brakeSlider = 0.0;
volatile float m_steerSlider = 0.0;
volatile bool m_handbrakeMode = false;
volatile bool m_manualGearMode = false;
volatile bool m_reverseMode = false;
volatile int m_autoDrivingMode = 0;

volatile static bool running = true;

constexpr int RETRY_INTERVAL_MS = 1000;
constexpr int MAX_RETRIES = 600; // ~10 minute

struct termios orig_termios;

void disableTerminalEcho() {
    if (!isatty(STDIN_FILENO)) return; // Only if running in a terminal

    struct termios new_termios;
    tcgetattr(STDIN_FILENO, &orig_termios); // get current terminal attributes
    new_termios = orig_termios;

    new_termios.c_lflag &= ~(ECHO | ICANON); // Disable echo and canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

void restoreTerminalEcho() {
    if (!isatty(STDIN_FILENO)) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

static void sendControlMessage(CanManager *canManager) {
    while (running && (canManager != nullptr)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // send msg
        canManager->sendCanFrame(packThrottleBrakeGear(m_thottleSlider, m_brakeSlider));
        canManager->sendCanFrame(packSteer(m_steerSlider));
    }
}

int main() {
    atexit(restoreTerminalEcho);
    disableTerminalEcho(); // Disable echo at the start
    auto devices = listInputDevices();
    int fd = -1;
    CanManager canManager("can0");

    INFO("BanVien Keyhandler Service Start");

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        auto devices = listInputDevices();

        for (const auto& dev : devices) {
            int tmp_fd = open(dev.c_str(), O_RDONLY);
            if (tmp_fd < 0) {
                INFO("Failed to open %s", dev.c_str());
                continue;
            }

            if (isKeyboardDevice(tmp_fd)) {
                fd = tmp_fd;
                INFO("Using keyboard device: %s", dev.c_str());
                break;
            }

            close(tmp_fd);
        }

        if (fd >= 0) break;

        INFO("No keyboard found, retrying in %dms (attempt %d/%d)...",
              RETRY_INTERVAL_MS, attempt + 1, MAX_RETRIES);
        std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_INTERVAL_MS));
    }

    if (fd < 0) {
        INFO("No input device found after retries.");
        return 1;
    }

    std::thread sendControlMessageThread = std::thread(sendControlMessage, &canManager);

    std::unordered_map<int, std::unique_ptr<KeyWorker>> workers;
    std::unordered_map<int, int> lastKeyValue;

    struct input_event ev;
    while (read(fd, &ev, sizeof(ev)) == sizeof(ev)) {
        if (workers.size() > 256) {
            INFO("Too many key codes. Stopping to prevent overflow");
            break;
        }
        if (ev.type == EV_KEY && (ev.value == 0 || ev.value == 1 || ev.value == 2)) {
            if (!isKeyMapped(ev.code)) {
                // Skip unknown keys
                continue;
            }
            // Skip repeated same-value events for the same key
            if (lastKeyValue[ev.code] == ev.value) {
                continue;
            }
            lastKeyValue[ev.code] = ev.value;
            if (workers.find(ev.code) == workers.end()) {
                workers[ev.code] = std::make_unique<KeyWorker>(ev.code, &canManager);
            }
            KeyEvent kev{ev.code, ev.value, ev.time};
            workers[ev.code]->pushEvent(kev);
        }
    }

    close(fd);

    running = false;
    if (sendControlMessageThread.joinable())
        sendControlMessageThread.join();
    
    restoreTerminalEcho(); // Restore terminal settings before exiting
    return 0;
}
