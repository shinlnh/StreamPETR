#include <iostream>
#include "general_logger.h"
#include <algorithm>
#include <vector>
#include <thread>
#include <mutex>

void tc1() {
    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
    std::time_t now = std::chrono::high_resolution_clock::to_time_t(tp);
    std::tm *local_time = localtime(&now);
    std::string to_str = std::ctime(&now);
    std::cout << to_str << std::endl;
    std::cout << "year: " << local_time->tm_year + 1900 << std::endl
              << "month: " << local_time->tm_mon + 1 << std::endl
              << "day: " << local_time->tm_mday << std::endl
              << "hour: " << local_time->tm_hour << std::endl
              << "minute: " << local_time->tm_min << std::endl
              << "second: " << local_time->tm_sec << std::endl;
}

void tc2() {
    Stopwatch watch;
    std::cout << "current time: " << watch.getTime() << std::endl;
}

// simple log file
void tc3() {
    LogUtility log("log/", "_test_module.log");
    log.enableLogFile();
    log.e(VV"hello in error", "---1---\t\t", log.timestamp());
    log.w(VV"hello in warning", "---2---\t", log.timestamp());
    log.i(VV"hello in info", "---3---\t\t", log.timestamp());
    log.d(VV"hello in debug", "---4---\t", log.timestamp());
}

std::string print_vector(std::vector<int> &list) {
    std::ostringstream oss;
    for (int item : list) {
        oss << item << ", ";
    }
    return oss.str();
}

void tc4() {
    const int MAX = 100;
    srand(0);
    std::vector<int> list;
    for (int i = 0; i < MAX; ++i) {
        list.push_back(rand() % (MAX * 10));
    }

    LogUtility log;
    log.enableLogFile();
    log.reset();
    log.i(VV"time: ", log.timestamp(), "\tlist: ", print_vector(list));
    int len = list.size();
    for (; len > 0; --len) {
        for (int i = 0; i < len - 1; ++i) {
            if (list[i] > list[i + 1]) {
                std::swap(list[i], list[i + 1]);
            }
        }
        log.i(VV"time: ", log.timestamp(), "\tlist: ", print_vector(list));
    }

}

void tc5() {

}

// test timestamp return double
void tc6() {
    LogUtility log;
    for (int i = 0; i < 100; ++i);
    uint64_t temp = log.timestamp(MICRO, false, true);
    double duration = *(double *)(&temp);
    std::cout << "duration: " << duration << " microsecond\n";
}

// log into 1 file
void tc7() {
    // initial logger 1
    LogUtility log1;
    log1.enableLogFile();

    // log data
    log1.e(VV"hello in error", "---1---\t\t", log1.timestamp());
    log1.w(VV"hello in warning", "---2---\t", log1.timestamp());
    log1.i(VV"hello in info", "---3---\t\t", log1.timestamp());
    log1.d(VV"hello in debug", "---4---\t", log1.timestamp());

    // initial logger 2
    LogUtility log2;
    log2.enableLogFile();

    // log data
    log2.e(VV"hello in error", "---5---\t\t", log2.timestamp());
    log2.w(VV"hello in warning", "---6---\t", log2.timestamp());
    log2.i(VV"hello in info", "---7---\t\t", log2.timestamp());
    log2.d(VV"hello in debug", "---8---\t", log2.timestamp());
}

// log file csv
void tc8() {
    LogUtility log;
    srand(0);
    for (int row = 0; row < 5; ++row) {
        log.logCSV(rand() % 20, rand() % 20, rand() % 20, rand() % 20, rand() % 20);
    }
}

#define MAX_CHAR 1e7

void writeFile(const char *filename, int id, std::mutex *lock) {
    std::lock_guard<std::mutex> lock_file(*lock);
    std::ofstream file(filename, std::ios::app);
    for (int i = 0; i < MAX_CHAR; ++i)
        file << id;
    file.close();
}

void tc9() {
    const char *filename = "log/temp.log";
    std::mutex lock;
    std::thread thread1(&writeFile, filename, 1, &lock);
    std::thread thread2(&writeFile, filename, 2, &lock);

    thread1.join();
    thread2.join();
}

void callback(LogUtility *log, int ID) {
    (*log).i(VV "callback function in thread  ", ID, " is called after ", (*log).t(MILLI), " ms");
    (*log).s();
}
void threadFunc(int ID) {
    LogUtility log("log/", "thread", false);
    log.enableLogFile();
    for (int i = 0; i < 5; ++i) {
        callback(&log, ID);
        usleep(ID * 10000);
    }
}
void tc10() {
    // test call logger in thread    
    std::thread t1(threadFunc, 1);
    std::thread t2(threadFunc, 2);

    t1.join();
    t2.join();
}

class A {
private:
    LogUtility log;

public:
    A() : log("log/", "A", false) {
        log.enableLogFile();
    }
    void callback() {
        log.i(VV "callback function is called after ", log.t(MILLI), " ms");
        log.s();
    }
    void test() {
        for (int i = 0; i < 10; ++i) {
            callback();
            usleep(10000);
        }
    }
};

void tc11() {
    A obj;
    obj.test();
}


int main() {
    // tc1();
    // tc4();
    // tc3();
    // tc6();
    // tc7();
    // tc8();
    // tc9();
    tc10();

    // Stopwatch watch;
    // for (int i = 0; i < 100; ++i) {
    //     std::cout << watch.getTime(true) << std::endl;
    //     sleep(1);
    // }

    // for (int i = 0; i < 10; ++i) {
    //     tc3();
    // }
    return 0;
}