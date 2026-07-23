#ifndef __LKS_CONTROLLER_H__
#define __LKS_CONTROLLER_H__

#ifndef UT_TEST
#include "controller.h"
#include "model_predictive_control.h"
// #define TUNE_MPC
#else 
#include "lib4test.h" 
#endif

#define CONVERT_SIMULATION_TIME 1e9
#define CENTER_STEERING_CONTROL_DEADBAND (0.25*M_PI/180.0f)
#define CURVE_DELTA_STEERING_CONTROL_DEADBAND (0.17*M_PI/180.0f)
#define CENTER_STEERING_CONTROL_THRESHOLD (0.25*M_PI/180.0f)
#define CURVE_STEERING_CONTROL_THRESHOLD (0.7*M_PI/180.0f)


class LKSController: public Controller, public ModelPredictiveControl
{
protected:
    ControlResults controlResPrev;
public:
    LKSController();
    void setInput(const PlanningResults& planning_results, ControlConfigs configs) override {}
    ControlResults execute(const PlanningResults& planning_results, const float& my_car_velocity,
                           ControlConfigs configs) override;
    float time_since_last_mpc;
    void updateParameters(float Q_11, float Q_22, float Q_33, float r_value)
    {
#ifdef TUNE_MPC
        Eigen::Matrix3f Q_minor;
        Q_minor << Q_11, 0, 0,
                    0, Q_22, 0,
                    0, 0, Q_33;  

        for (int i = 0; i < predicted_steps; i++)
        {
            Q.block(i * 3, i * 3 ,3, 3) = Q_minor;
        }

        for (int i = 0; i < control_steps; i++)
        {
            R.coeffRef(i, i) = r_value;
        }
        INFO("Q_11: %lf; Q_22: %lf; Q_33: %lf; r_value: %lf", Q_11, Q_22, Q_33, r_value);
#endif
    }

    /**
     * @brief Initialize the LKS controller with the current steering value.
     * 
     * This prevents the controller from starting at 0 when transitioning from manual
     * to lane-keeping mode, avoiding a momentary steering jerk.
     * 
     * @param steering Current steering value (-1.0 to 1.0)
     */
    void initializeWithSteering(float steering)
    {
        // Initialize the previous control result with current steering
        controlResPrev.steering_value = steering;
        // Reset the time tracker to avoid triggering reset logic in execute()
        this->time_since_last_mpc = 0.0f;
        this->lastTimeProcessingMpc = 0;
        DEBUG("Initialized LKS controller with steering: %f", steering);
    }

private:
    void restrictSignalControl(const float& curWheelAngle, float& curSignalCtrl); // input: radian
};

#endif // __LKS_CONTROLLER_H__