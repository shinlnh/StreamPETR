#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include "can_manager.h"
#include "InputDeviceUtils.h"
#include "ThreadSafeQueue.h"

class KeyWorker {
    int keyCode;
    std::string keyName;
    ThreadSafeQueue<KeyEvent> eventQueue;
    std::thread processThread;
    std::atomic<bool> running{true};
    
    std::thread repeatThread;
    std::atomic<bool> autoRepeating{false};
    std::condition_variable repeatCv;
    std::mutex repeatMutex;

    timeval lastPressTime{};
    std::atomic<bool> pressed{false};

    void process();
    void autoRepeatLoop();
    void startAutoRepeat();
    void stopAutoRepeat();
    void adjustDirection();
    CanManager *canManager;

public:
    explicit KeyWorker(int code, CanManager *canManager);
    ~KeyWorker();

    void pushEvent(const KeyEvent& ev);
};
