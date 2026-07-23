#include <gtest/gtest.h>
#include "mocking.h"
#include <thread>

class PlanningQueueTest : public PlanningQueue 
{
    public:
        void change (std::vector<std::array<float, 3>> &change_value)
        {   
            std::this_thread::sleep_for(std::chrono::microseconds(2));  // Sleep 1 milliseconds 
            change_value.resize(1, std::array<float, 3>{1.0f, 2.0f, 3.0f}); // Change size of planning_results
        }
};



