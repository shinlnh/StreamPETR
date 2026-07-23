// InputDeviceUtils.h
#pragma once

#include <vector>
#include <string>
#include <linux/input.h>

std::vector<std::string> listInputDevices();
bool isKeyboardDevice(int fd);

struct KeyEvent {
    int code;
    int value;
    timeval timestamp;
};

std::string getKeyName(int code);
bool isKeyMapped(int code);