#include <gmock/gmock.h>
#include "gtest/gtest.h"
#include <Eigen/Dense>

#define private public
#define protected public
// Function
#include "model_predictive_control.h"
#undef private
#undef protected

using ::testing::_;
using ::testing::Return;


class MockModelPredictiveControl : public ModelPredictiveControl 
{
public:
    static constexpr uint8_t predicted_steps = 37;
    static constexpr uint8_t control_steps = 20;

    MOCK_METHOD(bool, checkForConstraintViolations, 
            ((const Eigen::Matrix<float, control_steps, 1>& )matrix_to_be_checked,
            (const Eigen::Matrix<float, 4 * control_steps, control_steps>&) constraint_matrix,
            (const Eigen::Matrix<float, 4 * control_steps, 1>&) gamma_matrix),
            (override));

    // Uncomment and modify as needed for other methods
    // MOCK_METHOD(void, limitRange, (float*, float, float), (override));
};

