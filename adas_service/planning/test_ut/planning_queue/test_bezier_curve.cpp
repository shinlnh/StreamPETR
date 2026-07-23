#include <gtest/gtest.h>
#include <iostream>
#include "fixture.h"

using ::testing::_;
using ::testing::Return;

// Definition function ernstein_poly
static double bernstein_poly(int n, int i, float t) {
    // Calculate binomial coefficient C(n, i)
    double binom = 1.0f;
    for (int j = 0; j < i; ++j) {
        binom *= (n - j) / static_cast<float>(j + 1);
    }

    return binom * std::pow(t, i) * std::pow(1 - t, n - i);
}

/**
 * @brief  TEST 1: bezier_curve(const PlanningResults& control_points, int num_points = 130)
 * Input: PlanningResults control_points, int num_points = 130
 * Expected output: curve
*/
TEST(PlanningQueue, bezier_curve_test)
{
    // Define inputs for the constructor of function PlanningQueue
    PlanningQueue planning_queue;

    // Define inputs for the test
    int num_points = 130;
    std::vector<std::array<float, 3>> control_points(num_points);
    
    // Define expect resut for the test
    std::vector<std::array<float, 3>> expect_curve(num_points);
    std::vector<std::array<float, 3>> curve(num_points);
    int n = control_points.size() - 1;

    for (int k = 0; k < num_points; ++k) {
        float t = static_cast<float>(k) / (num_points - 1);
        Point point = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i <= n; ++i) {
            double b = bernstein_poly(n, i, t);
            point[0] += b * control_points[i][0];
            point[1] += b * control_points[i][1];
        }
        expect_curve[k] = {point[0], point[1], 0.0f};
    }

    // Call the method under test
    curve = planning_queue.bezier_curve(control_points, num_points);
    
    // Compare expect result with actual resutl
    EXPECT_EQ(curve, expect_curve);
}
