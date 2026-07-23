#ifndef FIXTURE_H
#define FIXTURE_H
#include <gtest/gtest.h>
#include "mocking.h"
#include <cmath>
#include "common.h"
#include <array>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <math.h>
#include "types.h"

class LKF_TEST : public ::testing::Test 
{
protected:

    void SetUp() override {}
    void TearDown() override {}
};

#endif
