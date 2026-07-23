#ifndef __DATA_SHARING_QUEUE_H__
#define __DATA_SHARING_QUEUE_H__

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <semaphore.h>
#include <string>
#include <queue>
#include <unordered_map>
#include "common.h"

template <typename T>
class DataSharingQueue {
private:
    size_t expectedSensors_;
    boost::chrono::nanoseconds timeout_;
    boost::mutex mtx_;
    boost::condition_variable cv_;
    sem_t semaphore_;

    bool block_if_empty_;

    std::unordered_map<int, bool> hasData_;
    std::unordered_map<int, std::shared_ptr<T>> pendingData_;

    // Each consumer is identified by an integer.
    static int consumers;
    std::unordered_map<int, std::queue<std::shared_ptr<T>>> subscribers_;
    unsigned int max_size_subscriber_;

    bool combineData() {
        if (pendingData_.empty()) return false;
        // Distribute each sensor's data to all registered consumers.
        for (auto& entry : pendingData_) {
            for (auto& sub : subscribers_) {
                // sub is a pair (consumerId, queue), so push to the queue:
                sub.second.push(entry.second);
                if (sub.second.size() > max_size_subscriber_) {
                    sub.second.pop();
                }
            }
        }
        reset();
        return true;
    }

    void reset() {
        for(auto& entry : hasData_) {
            entry.second = false;
        }
        pendingData_.clear();
    }

public:
    /* Attribute */
    std::string type = "";

    /* Method */
    DataSharingQueue(size_t expectedSensors = 2,
                          boost::chrono::milliseconds timeout = boost::chrono::milliseconds(1),
                          unsigned int max_size_subscriber = 100)
        : expectedSensors_(expectedSensors), timeout_(timeout), max_size_subscriber_(max_size_subscriber)
    {
        // Allow up to expectedSensors threads to push concurrently.
        sem_init(&semaphore_, 0, expectedSensors_); 

        // Preallocate the hasData_ map with sensor IDs 0 .. expectedSensors_-1.
        hasData_.reserve(expectedSensors_);
        for (size_t i = 0; i < expectedSensors_; ++i) {
            hasData_[i] = false;
        }

        // Initiale with non blocking mode
        block_if_empty_ = false;
    }

    ~DataSharingQueue() {
        sem_destroy(&semaphore_);
    }

    // Create a new consumer (subscriber) and return its consumer id.
    int createConsumer() {
        boost::unique_lock<boost::mutex> lock(mtx_);
        int consumerId = ++DataSharingQueue::consumers;
        subscribers_[consumerId] = std::queue<std::shared_ptr<T>>();
        return consumerId;
    }

    // Return the total number of consumers.
    int getConsumer() {
        return subscribers_.size();
    }

    // Push sensor data into the synchronizer.
    // Blocks (via a semaphore) if too many sensors are pushing concurrently.
    void pushData(int sensorId, std::shared_ptr<T> data) {
        sem_wait(&semaphore_); 
        boost::unique_lock<boost::mutex> lock(mtx_);

        if (DataSharingQueue::consumers <= 0) {
            DEBUG("Have no consumer in Data Sharing Queue");
        }

        if (hasData_[sensorId] == true) {
            DEBUG("Warning: Data for sensorId %d is already coming.", sensorId);
        }

        pendingData_[sensorId] = data;
        hasData_[sensorId] = true;
        // Wait for either all expected sensor data to arrive or until timeout.
        cv_.wait_for(lock, timeout_, [&] { return pendingData_.size() >= expectedSensors_; });
        bool processed = combineData();
        sem_post(&semaphore_); // Release the semaphore.
        if (processed)
            cv_.notify_all(); // Notify all waiting consumers.
    }

    // Retrieve the next combined set of data for the given consumer.
    // This function will block until there is data available in the consumer's queue.
    std::queue<std::shared_ptr<T>> getNext(int consumerId, bool block = true, bool drop = true) {
        boost::unique_lock<boost::mutex> lock(mtx_);
        if(block) cv_.wait(lock, [&]() { return !subscribers_[consumerId].empty(); });
        std::queue<std::shared_ptr<T>> return_queue = subscribers_[consumerId];
        while (drop && !subscribers_[consumerId].empty()) {
            subscribers_[consumerId].pop();
        }
        return return_queue;
    }

    /**
     * @brief Unregister customer
     * @param customer_id
     * @return status
     */
    bool unregisterCustomer(int customer_id) {
        boost::lock_guard<boost::mutex> lock(mtx_);
        return subscribers_.erase(customer_id);
    }

    void setBlockIfEmpty(const bool &enable) {
        boost::unique_lock<boost::mutex> lock(mtx_);
        block_if_empty_ = enable;
    }
    
    /**
     * @brief Get number of element contained in queue of customer
     */
    int sizeOf(int customer_id) {
        boost::lock_guard<boost::mutex> lock(mtx_);
        return subscribers_[customer_id].size();
    }
};

// Initialize the static consumer counter.
template <typename T>
int DataSharingQueue<T>::consumers = 0;

#endif