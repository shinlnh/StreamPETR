#include "lks_controller.h"

LKSController::LKSController()
{
	Q.setIdentity();
	R.setIdentity();
    Eigen::Matrix3f Q_minor;
    #ifdef ENABLE_CAN
        Q_minor << 0.23, 0, 0,
                0, 3.62, 0, 
                0, 0, 50; 
        for (int i = 0; i < predicted_steps; i++) {
            Q.block(i * 3, i * 3 ,3, 3) = Q_minor;
        }

        for (int i = 0; i < control_steps; i++) {
            R.coeffRef(i, i) = 199;
        }
    #else
        Q_minor << 0.23, 0, 0,
               0, 3.62, 0,
               0, 0, 12.5;  
        for (int i = 0; i < predicted_steps; i++) {
            Q.block(i * 3, i * 3 ,3, 3) = Q_minor;
        }

        for (int i = 0; i < control_steps; i++) {
            R.coeffRef(i, i) = 50;
        }
    #endif
    controlResPrev.init();
    this->lastTimeProcessingMpc = 0;
    this->time_since_last_mpc = 0;
}

void LKSController::restrictSignalControl(const float& curWheelAngle, float& curSignalCtrl)
{
    if (curWheelAngle <= CENTER_STEERING_CONTROL_THRESHOLD && curWheelAngle >= -CENTER_STEERING_CONTROL_THRESHOLD)
    {
        // when in straight path -> using absolute signal control
        if (curSignalCtrl >= -CENTER_STEERING_CONTROL_DEADBAND && curSignalCtrl <= CENTER_STEERING_CONTROL_DEADBAND)
        {
            curSignalCtrl = 0.0f;
        }
    }
    else if (curWheelAngle <= -CURVE_STEERING_CONTROL_THRESHOLD || curWheelAngle >= CURVE_STEERING_CONTROL_THRESHOLD)
    {
        float deltaU = curSignalCtrl - curWheelAngle; 
        if (deltaU >= - CURVE_DELTA_STEERING_CONTROL_DEADBAND && deltaU <= CURVE_DELTA_STEERING_CONTROL_DEADBAND)
        {
            curSignalCtrl = curWheelAngle;
        }
    }
}

ControlResults LKSController::execute(const PlanningResults& planning_results, 
                                      const float& my_car_velocity, ControlConfigs configs)
{
    // ControlResults control_results;
    if (this->lastTimeProcessingMpc == 0)
    {
        this->lastTimeProcessingMpc = this->wheel_angle.time_stamp; // first time
        // return to avoid accumulate watch dog mpc
        return this->controlResPrev;
    }
    sample_time = (this->wheel_angle.time_stamp - this->lastTimeProcessingMpc)/((float)CONVERT_SIMULATION_TIME);
    this->lastTimeProcessingMpc = this->wheel_angle.time_stamp;
    this->time_since_last_mpc += sample_time;

    // Reset controller if it has been a while since last run
    static constexpr float reset_threshold = 1;  // second(s)
    if (this->time_since_last_mpc > reset_threshold) {
        reset();
        this->time_since_last_mpc = 0.0f;
        return this->controlResPrev;
    }
    sample_time = 0.05;
    if (sample_time >= 0.05)
    {
        //                                  ----------IMPORTANT NOTE----------
        // In the near future, once the localization module becomes available, log the car position and desired path here for evaluation
        ModelPredictiveControl::setCarPosition({0.0, -1.5, 0.0});
        float feedbackSignalControl = 0;
        #ifdef ENABLE_CAN
            feedbackSignalControl = this->getEpsSteeringAngle()*M_PI/180.0f;
        #else 
            feedbackSignalControl = this->getWheelAngle().steering*M_PI/180.0f; 
        #endif
        float desired_steering_angle = ModelPredictiveControl::execute(planning_results, my_car_velocity / 3.6, car_acceleration,
                                                                    sample_time, feedbackSignalControl);
        // Convert rad of desired_steering_angle to steeing control signal
        desired_steering_angle = desired_steering_angle / ((MAX_CAR_STEERING_ANGLE / 180.0f) * M_PI);
        this->controlResPrev.steering_value = desired_steering_angle;

        this->time_since_last_mpc = 0;
    }

    return this->controlResPrev;
}