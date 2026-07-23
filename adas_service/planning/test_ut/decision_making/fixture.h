#include <gtest/gtest.h>
#include <memory>
#include "mocking.h"
#include <stdexcept>

class DecisionMakingModuleMock : public ::testing::Test {
protected:
    DecisionMakingModule* dmm ;
    FrenetEnvironment dummy_data;

    DecisionMakingModuleMock()
    {
        dmm = new DecisionMakingModule(&dummy_data);
    }
    ~DecisionMakingModuleMock()
    {
        delete dmm;
    }

    void setupTestData(std::vector<CandidatePath>& data) {
    }
};

