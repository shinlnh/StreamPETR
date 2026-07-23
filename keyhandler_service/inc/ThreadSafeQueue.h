#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T>
class ThreadSafeQueue {
    std::queue<T> q;
    std::mutex mtx;
    std::condition_variable cv;
public:
    void push(const T& item) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            q.push(item);
        }
        cv.notify_one();
    }

    T wait_and_pop() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !q.empty(); });
        T val = q.front();
        q.pop();
        return val;
    }
};
