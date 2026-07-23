#ifndef _MODEL_PREDICTIVE_CONTROL_H_
#define _MODEL_PREDICTIVE_CONTROL_H_
#include "controller.h"
#include "common.h"

#define OFFSET_PLANNING_RESULT  -3.895
class ModelPredictiveControl
{
public:
    ModelPredictiveControl();
    float execute(const PlanningResults& planning_results, const float& my_car_velocity, const float& my_car_acceleration,
                  const float& sample_time_mpc, const float& car_steering_angle);
    static constexpr uint8_t predicted_steps = 37; // 55
    static constexpr uint8_t control_steps = 20; // 37
    
    float max_tracking_point_heading = 0.0;
    float curvature_max = 0.0;
    float max_steering_rate = 30*M_PI/180; // rad/s
    void setCarPosition(const std::array<float, 3>& updated_car_position)
    {
        this->car_position = updated_car_position;
    }

    Eigen::Matrix<float, Eigen::Dynamic,  Eigen::Dynamic> Q;                    // Q is the output weight matrix, a positive semi-definite weighting matrix
    Eigen::Matrix<float, Eigen::Dynamic,  Eigen::Dynamic> R;                    // R is the control weight matrix
public:
    struct PathProfileYaw 
    {
        std::vector<cv::Point2f> path_v;   // (x,y)
        std::vector<float> path_length_v;            // arc-length cum
        std::vector<float> yaw_path_v;          // yaw per node
        float totalPathLength = 0.0f;
    };

    struct TrackingResults
    {
        std::vector<std::array<float, 3>> points;
    };

private:
    int hildreth_max_iteration = 2000;                                                 // Maximum number iterations of Hildreth algorithm
    double epsilon = 1e-5;                                                            // This is the convergent acceptance value of  Hildreth algorithm.
    float max_steering_angle = MAX_CAR_STEERING_ANGLE / 180.0f * M_PI;                             // Car's maximum steering angle [unit = radian]
    float min_steering_angle = - MAX_CAR_STEERING_ANGLE / 180.0f * M_PI;                           // Car's minimum steering angle [unit = radian]
    float steeringThreshold = 70*M_PI/180.0f;
    std::array<float, 3> car_position;                    

    Eigen::Matrix<float, 3 + 1, 3 + 1> Ae;                                      // Ae is a enhanced state matrix
    Eigen::Matrix<float, 3 + 1, 1> Be;                                          // Be is a enhanced control matrix
    Eigen::Matrix<float, 3, 3 + 1> Ce;                                          // Ce is a enhanced output matrix
    Eigen::Matrix<float, 3 + 1, 1> Xe;                                          // State values of enhanced model

    void enhancedLinearKinematicModel(const float& steering_angle, const float& yaw_angle,
                                      const float& my_car_velocity, const float& sample_time_mpc);

    void predictedTrajectoryMatrix(const float& x_coordinate, const float& y_coordinate,
                                   const float& yaw_angle, const float& steering_angle,
                                   const float& my_car_velocity, const float& my_car_acceleration, 
                                   const float& sample_time_mpc, 
                                   const PlanningResults& planning_results, 
                                   Eigen::Matrix<float, 3 * predicted_steps, 1>& Rs);

    uint8_t findMinVectorLength(const PlanningResults& planning_results,
                                const std::array<float, 3>& trajectory_vector, 
                                const uint8_t& index_begin);
                                   
    void createQuadraticProgrammingAlgorithmMatrices(Eigen::Matrix<float, 3 * predicted_steps, 4>& F,
                                                     Eigen::Matrix<float, 3 * predicted_steps, control_steps>& G);

    void createMatricesToCheckConstraints(Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix, 
                                          Eigen::Matrix<float, 4 * control_steps, control_steps>& M,
                                          const float& steering_angle, const float& sample_time_mpc);
    float computeKappaMax(const PlanningResults& planning_results);
    #ifndef UT_TEST
        bool checkForConstraintViolations(const Eigen::Matrix<float, control_steps, 1>& matrix_to_be_checked,
                                      const Eigen::Matrix<float, 4 * control_steps, control_steps>& constraint_matrix,
                                      const Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix);
    #else
        virtual bool checkForConstraintViolations(const Eigen::Matrix<float, control_steps, 1>& matrix_to_be_checked,
                                      const Eigen::Matrix<float, 4 * control_steps, control_steps>& constraint_matrix,
                                      const Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix);
    #endif

    Eigen::Matrix<float, control_steps, 1> hildrethAlgorithm(const Eigen::Matrix<float, control_steps, control_steps>& H,
                            const Eigen::Matrix<float, control_steps, 1>& f_matrix,
                            const Eigen::Matrix<float, 4 * control_steps, control_steps>& M,
                            const Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix,
                            const Eigen::Matrix<float, control_steps, 1>& delta_control_signal);
    PathProfileYaw buildProfileFromPlanningResult(const PlanningResults& xyyaw_Path, int offsetPoint);
    TrackingResults buildTrackingTrajectoryFromProfile(const PathProfileYaw& profile,
                                                    float s0,        // arc-length (ego projection)
                                                    float v,         // current velocity
                                                    float a,         // acceleration
                                                    float deltaT,    // MPC dt
                                                    int   N);
    PathProfileYaw profileTracking;
    TrackingResults trackingTraj;
};

#endif // _MODEL_PREDICTIVE_CONTROL_H_