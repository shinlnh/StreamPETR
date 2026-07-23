#include "InputDeviceUtils.h"
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unordered_map>
#include <iostream>

static std::unordered_map<int, std::string> keyMap = {
    {KEY_0, "KEY_0"},
    {KEY_1, "KEY_1"},
    {KEY_2, "KEY_2"},
    {KEY_6, "KEY_6"},
    {KEY_W, "KEY_W"},
    {KEY_A, "KEY_A"},
    {KEY_S, "KEY_S"},
    {KEY_D, "KEY_D"},
    {KEY_Q, "KEY_Q"},
    {KEY_UP, "KEY_UP"},
    {KEY_LEFT, "KEY_LEFT"},
    {KEY_RIGHT, "KEY_RIGHT"},
    {KEY_DOWN, "KEY_DOWN"},
    {KEY_ENTER, "KEY_ENTER"},
    // Add more keys as needed
};

std::string getKeyName(int code) {
    auto it = keyMap.find(code);
    return (it != keyMap.end()) ? it->second : "UNKNOWN";
}

bool isKeyboardDevice(int fd) {
    unsigned long evbit = 0;
    if (ioctl(fd, EVIOCGBIT(0, sizeof(evbit)), &evbit) < 0) return false;
    if (!(evbit & (1 << EV_KEY))) return false;

    unsigned char key_bits[KEY_MAX / 8 + 1] = {0};
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0) return false;

    auto testKey = [&](int key) {
        return key_bits[key / 8] & (1 << (key % 8));
    };

    return testKey(KEY_A) && testKey(KEY_Z) && testKey(KEY_ENTER);
}

std::vector<std::string> listInputDevices() {
    std::vector<std::string> devices;
    DIR* dir = opendir("/dev/input");
    if (!dir) return devices;

    struct dirent* ent;
    while ((ent = readdir(dir))) {
        if (strncmp(ent->d_name, "event", 5) == 0) {
            devices.push_back(std::string("/dev/input/") + ent->d_name);
        }
    }
    closedir(dir);
    return devices;
}

bool isKeyMapped(int code) {
    return keyMap.find(code) != keyMap.end();
}
