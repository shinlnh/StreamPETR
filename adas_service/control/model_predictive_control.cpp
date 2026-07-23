#include "model_predictive_control.h"

#include "frenet_environment/settings.h"
#define LANE_DETECTION_RANGE 40 // in meters

ModelPredictiveControl::ModelPredictiveControl()
{
	Q.resize(3 * predicted_steps, 3 * predicted_steps); 
	R.resize(control_steps, control_steps); 
}

float ModelPredictiveControl::execute(const PlanningResults& planning_results, const float& my_car_velocity, const float& my_car_acceleration,
                                      const float& sample_time_mpc, const float& car_steering_angle)
{   
    for (const auto &point: planning_results.points) {
        if (max_tracking_point_heading < point[2]);
            max_tracking_point_heading = point[2];
    }

    // curvature_max = computeKappaMax(planning_results);

    Eigen::Matrix<float, 3 * predicted_steps, 1> Rs;                // Rs is matrix of diffential instantaneous trajectory points that the MPC needs to follow
    Eigen::Matrix<float, 3 * predicted_steps, 3 + 1> F; 
    Eigen::Matrix<float, 3 * predicted_steps, control_steps> G; 
	G.setZero();

    enhancedLinearKinematicModel(car_steering_angle, car_position[2], my_car_velocity, sample_time_mpc);
    predictedTrajectoryMatrix(car_position[0], car_position[1], car_position[2], car_steering_angle,
                              my_car_velocity, my_car_acceleration, sample_time_mpc, planning_results, Rs);
    createQuadraticProgrammingAlgorithmMatrices(F, G);

    Eigen::MatrixXf H;
    Eigen::MatrixXf f_matrix;
    H = G.transpose() * Q * G + R;
    f_matrix = G.transpose() * Q * (F * Xe - Rs);

    // Initial control signal (without constraints)
    Eigen::MatrixXf delta_control_signal = -1 * (H.inverse() * f_matrix);

    // CHECK IF VIOLATE CONSTRAINTS OR NOT
    Eigen::Matrix<float, 4 * control_steps, control_steps> M;            // M represents the constraint matrix
    Eigen::Matrix<float, 4 * control_steps, 1> gamma_matrix;
    M.setZero();

    createMatricesToCheckConstraints(gamma_matrix, M, car_steering_angle, sample_time_mpc);
    bool check_constraint = checkForConstraintViolations(delta_control_signal, M, gamma_matrix);
    float mpc_steering_angle = 0.0;

    if(false == check_constraint) {
        mpc_steering_angle = delta_control_signal(0,0);
    } else {
        DEBUG("Constraint violation");
        delta_control_signal =  hildrethAlgorithm(H, f_matrix, M, gamma_matrix, delta_control_signal);
        mpc_steering_angle = delta_control_signal(0,0);
    }
    delta_control_signal.setZero();

    return mpc_steering_angle + car_steering_angle;
}

void ModelPredictiveControl::enhancedLinearKinematicModel(const float& steering_angle, const float& yaw_angle,
                                                          const float& my_car_velocity, const float& sample_time_mpc)  
{
    Ae << 1,   0,   my_car_velocity * std::cos(yaw_angle) * sample_time_mpc,      0,
          0,   1, - my_car_velocity * std::sin(yaw_angle) * sample_time_mpc,      0,
          0,   0,   1,   my_car_velocity / (CAR_WHEEL_BASE * std::cos(steering_angle) * std::cos(steering_angle)) * sample_time_mpc,
          0,   0,   0,   1;

    Be << 0,
          0,
          my_car_velocity / (CAR_WHEEL_BASE * std::cos(steering_angle) * std::cos(steering_angle)) * sample_time_mpc,
          1;

    Ce << 1,   0,   0,   0,
          0,   1,   0,   0,
          0,   0,   1,   0;

    Xe << 0, 
          0,
          0,
          steering_angle;
}

void ModelPredictiveControl::predictedTrajectoryMatrix(const float& x_coordinate, const float& y_coordinate,
                                                       const float& yaw_angle, const float& steering_angle,
                                                       const float& my_car_velocity, const float& my_car_acceleration, 
                                                       const float& sample_time_mpc,
                                                       const PlanningResults& planning_results, 
                                                       Eigen::Matrix<float, 3 * predicted_steps, 1>& Rs)
{
    std::array<float, 3> predicted_trajectory[predicted_steps + 1];
    predicted_trajectory[0] = {x_coordinate, y_coordinate, yaw_angle};

    for (int i = 1; i < predicted_steps + 1; i++) {
        predicted_trajectory[i][0] = predicted_trajectory[i-1][0] + my_car_velocity * std::sin(predicted_trajectory[i-1][2]) * sample_time_mpc;
        predicted_trajectory[i][1] = predicted_trajectory[i-1][1] + my_car_velocity * std::cos(predicted_trajectory[i-1][2]) * sample_time_mpc;
        predicted_trajectory[i][2] = predicted_trajectory[0][2] + i * (my_car_velocity / CAR_WHEEL_BASE) * std::tan(steering_angle) * sample_time_mpc;
        if (predicted_trajectory[i][2] >= steeringThreshold)
        {
            predicted_trajectory[i][2] = steeringThreshold;
        }
        else if (predicted_trajectory[i][2] <= -steeringThreshold)
        {
            predicted_trajectory[i][2] = -steeringThreshold;
        }
    }

    profileTracking = buildProfileFromPlanningResult(planning_results, 0);
    trackingTraj = buildTrackingTrajectoryFromProfile(profileTracking, 0.0f, my_car_velocity, 0, 0.05, predicted_steps); // y_coordinate

    uint8_t init_index = 0;
    for (int i = 1; i < predicted_steps + 1; i++) {
        if (predicted_trajectory[i][1] >= LANE_DETECTION_RANGE) {
            Rs.coeffRef(3 * (i - 1)) = 0;
            Rs.coeffRef(3 * (i - 1) + 1) = 0;
            Rs.coeffRef(3 * (i - 1) + 2) = 0;
        } 
        else 
        {
            Rs.coeffRef(3 * (i - 1)) = trackingTraj.points[i-1][0] - predicted_trajectory[i][0];
            Rs.coeffRef(3 * (i - 1) + 1) = trackingTraj.points[i-1][1] - predicted_trajectory[i][1];
            Rs.coeffRef(3 * (i - 1) + 2) = trackingTraj.points[i-1][2] - predicted_trajectory[i][2];
        }
    }
}

uint8_t ModelPredictiveControl::findMinVectorLength(const PlanningResults& planning_results, 
                                                    const std::array<float, 3>& trajectory_vector,
                                                    const uint8_t& index_begin)
{
    float min_value = 999;
    uint8_t index = 0;
    for (int i = index_begin; i < planning_results.points.size(); i++) {     
		float vector_length = std::sqrt(std::pow(planning_results.points[i][0] - trajectory_vector[0], 2) 
									  + std::pow(planning_results.points[i][1] - trajectory_vector[1], 2));
		if (min_value > vector_length) {
            min_value = vector_length;
            index = i;
		}
    }

    return index;
}

void ModelPredictiveControl::createQuadraticProgrammingAlgorithmMatrices(Eigen::Matrix<float, 3 * predicted_steps, 4>& F,
                                                                          Eigen::Matrix<float, 3 * predicted_steps, control_steps>& G)
{
    Eigen::MatrixXf Ae_power_F = Eigen::MatrixXf::Identity(Ae.rows(), Ae.cols()); 
    Eigen::MatrixXf Ae_power_G = Eigen::MatrixXf::Identity(Ae.rows(), Ae.cols()); 

	for (int i = 0; i < predicted_steps; i++) {
		Ae_power_F = Ae_power_F * Ae; 
		F.block(3 * i, 0, 3, 4) = Ce * Ae_power_F;
		for (int j = 0; j < std::min(i + 1, static_cast<int>(control_steps)); j++) {
			for (int k = 0; k < i - j; k++) {
				Ae_power_G = Ae_power_G * Ae; 										// Compute Ae_power_G^(i-j)
			}
			G.block(3 * i, j, 3, 1) = Ce * Ae_power_G * Be;
			Ae_power_G = Eigen::MatrixXf::Identity(Ae.rows(), Ae.cols()); 			// Reset Ae_power_G to I
		}
    }
}

void ModelPredictiveControl::createMatricesToCheckConstraints(Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix, 
                                                              Eigen::Matrix<float, 4 * control_steps, control_steps>& M,
                                                              const float& steering_angle, const float& sample_time_mpc)
{
    for(int i = 0; i < control_steps; i++) 
    {
        /* ========= Steering angle constraints ========= */
        gamma_matrix.coeffRef(i,0) = max_steering_angle - steering_angle;
        gamma_matrix.coeffRef(i + control_steps,0) = - min_steering_angle + steering_angle;
        for(int j = 0; j <= i; j++) {
            M.coeffRef(i,j) = 1;
            M.coeffRef(i + control_steps, j) = -1;
        }
        /* ========= Steering rate constraints ========= */

        // Δφ_i <= max_delta_phi
        gamma_matrix.coeffRef(i + 2 * control_steps, 0) =
            max_steering_rate * sample_time_mpc;
        M.coeffRef(i + 2 * control_steps, i) = 1.0f;

        // -Δφ_i <= max_delta_phi
        gamma_matrix.coeffRef(i + 3 * control_steps, 0) =
            max_steering_rate * sample_time_mpc;
        M.coeffRef(i + 3 * control_steps, i) = -1.0f;
    }

    float scale = 10.0f;
    for (int i = 0; i < control_steps; i++)
    {
        M.row(i + 2 * control_steps) *= scale;
        M.row(i + 3 * control_steps) *= scale;

        gamma_matrix.coeffRef(i + 2 * control_steps, 0) *= scale;
        gamma_matrix.coeffRef(i + 3 * control_steps, 0) *= scale;
    }
}

bool ModelPredictiveControl::checkForConstraintViolations(const Eigen::Matrix<float, control_steps, 1>& matrix_to_be_checked,
                                                          const Eigen::Matrix<float, 4 * control_steps, control_steps>& constraint_matrix,
                                                          const Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix) 
{
    int violation_count  = 0;
    Eigen::MatrixXf check_violation = constraint_matrix * matrix_to_be_checked - gamma_matrix;                  // Size of check_violation is [2*control_steps, 1] 

    for(int i = 0; i < constraint_matrix.rows();i++) {
        if(check_violation.coeffRef(i) > 0) 
        {
            violation_count ++;
        }
    }

    return (0 < violation_count);
}


Eigen::Matrix<float, ModelPredictiveControl::control_steps, 1> ModelPredictiveControl::hildrethAlgorithm(const Eigen::Matrix<float, control_steps, control_steps>& H,
                            const Eigen::Matrix<float, control_steps, 1>& f_matrix,
                            const Eigen::Matrix<float, 4 * control_steps, control_steps>& M,
                            const Eigen::Matrix<float, 4 * control_steps, 1>& gamma_matrix,
                            const Eigen::Matrix<float, control_steps, 1>& delta_control_signal)
{
    constexpr float P_DIAG_EPS = 1e-8f;

    Eigen::MatrixXf H_inv = H.inverse();

    // P = M H⁻¹ Mᵀ
    Eigen::MatrixXf P = M * H_inv * M.transpose();    // (4N × 4N)

    // d = γ + M H⁻¹ f
    Eigen::MatrixXf d_matrix = gamma_matrix + M * H_inv * f_matrix;
    Eigen::MatrixXf lambda = Eigen::MatrixXf::Zero(gamma_matrix.rows(), 1);
    Eigen::MatrixXf lambda_prev = Eigen::MatrixXf::Zero(gamma_matrix.rows(), 1);


    double diff_norm = std::numeric_limits<double>::max();
    int iter = 0;

    for (; iter < hildreth_max_iteration && diff_norm > epsilon; ++iter)
    {
        for (int i = 0; i < d_matrix.rows(); ++i)
        {
            // Skip ill-conditioned constraint
            float P_ii = P(i, i);
            if (std::fabs(P_ii) < P_DIAG_EPS) {
                lambda(i, 0) = 0.0f;
                continue;
            }
            float w = (P.row(i) * lambda)(0, 0) - P_ii * lambda(i, 0) + d_matrix(i, 0);
            float new_lambda = -w / P_ii;
            lambda(i, 0) = std::max(0.0f, new_lambda);
        }
        // Convergence check
        Eigen::MatrixXf diff = lambda - lambda_prev;
        diff_norm = (diff.transpose() * diff)(0, 0);

        lambda_prev = lambda;
    }

    if (iter >= hildreth_max_iteration) {
        DEBUG("Hildreth: reached max iteration without full convergence");

    }

    Eigen::MatrixXf delta_u_opt = delta_control_signal - H_inv * M.transpose() * lambda;
    return delta_u_opt;
}

ModelPredictiveControl::PathProfileYaw ModelPredictiveControl::buildProfileFromPlanningResult(const PlanningResults& xyyaw_Path, int offsetPoint)
{
    PathProfileYaw profile;
    const int N = (int)xyyaw_Path.points.size();
    if (N == 0 || offsetPoint >= N)
    {
        return profile;
    }

    const int newN = N - offsetPoint;

    profile.path_v.reserve(newN);
    profile.yaw_path_v.reserve(newN);
    profile.path_length_v.assign(newN, 0.f);

    // first point
    profile.path_v.emplace_back(
        xyyaw_Path.points[offsetPoint][0],
        xyyaw_Path.points[offsetPoint][1]);

    // IMPORTANT: keep yaw as-is (planner frame)
    profile.yaw_path_v.push_back(
        xyyaw_Path.points[offsetPoint][2]*M_PI/180.0f);

    profile.path_length_v[0] = 0.f;

    for (int i = offsetPoint+1; i < N; ++i)
    {
        const float x0 = xyyaw_Path.points[i-1][0];
        const float y0 = xyyaw_Path.points[i-1][1];
        const float x1 = xyyaw_Path.points[i][0];
        const float y1 = xyyaw_Path.points[i][1];

        profile.path_v.emplace_back(x1, y1);

        float ds = std::hypot(x1 - x0, y1 - y0);
        profile.path_length_v[i-offsetPoint] = profile.path_length_v[i-offsetPoint-1] + ds;

        // KEEP yaw, no wrapping
        profile.yaw_path_v.push_back(xyyaw_Path.points[i][2]*M_PI/180.0f);
    }

    profile.totalPathLength = profile.path_length_v.back();
    return profile;
}


ModelPredictiveControl::TrackingResults ModelPredictiveControl::buildTrackingTrajectoryFromProfile(
    const ModelPredictiveControl::PathProfileYaw& profile,
    float s0,        // arc-length (ego projection)
    float v,         // current velocity
    float a,         // acceleration
    float deltaT,    // MPC dt
    int   N)         // number of points
{
    ModelPredictiveControl::TrackingResults traj;
    traj.points.reserve(N);

    const auto& P = profile.path_v;
    const auto& S = profile.path_length_v;
    const auto& Y = profile.yaw_path_v;
    const int M = (int)P.size();

    if (M == 0)
        return traj;

    for (int k = 0; k < N; ++k)
    {
        float t = k * deltaT;

        // arc-length evolution
        float s = s0 + v * t + 0.5f * a * t * t;

        float x, y, yaw;

        if (s <= profile.totalPathLength)
        {
            float s_clamp = std::max(0.f, s);

            int i = int(std::upper_bound(
                        S.begin(), S.end(), s_clamp)
                        - S.begin()) - 1;

            if (i < 0) i = 0;
            if (i >= M-1) i = M-2;

            float s0_i = S[i];
            float s1_i = S[i+1];
            float alpha = (s1_i > s0_i)
                            ? (s_clamp - s0_i) / (s1_i - s0_i)
                            : 0.f;

            auto lerp = [](float A, float B, float a)
            {
                return A + a * (B - A);
            };

            x = lerp(P[i].x, P[i+1].x, alpha);
            y = lerp(P[i].y, P[i+1].y, alpha);

            // IMPORTANT: yaw linear interpolate, NO normalize
            yaw = lerp(Y[i], Y[i+1], alpha);
        }
        else
        {
            // extrapolate straight from last point
            float extra_s = s - profile.totalPathLength;
            const cv::Point2f& lastP = P.back();
            float yaw_last = Y.back();

            x = lastP.x + extra_s * std::sin(yaw_last);
            y = lastP.y + extra_s * std::cos(yaw_last);
            yaw = yaw_last;
        }

        traj.points.push_back({x, y, yaw});
    }

    return traj;
}

float ModelPredictiveControl::computeKappaMax(const PlanningResults& planning_results) {
    auto &points = planning_results.points;
    if (points.size() < 3) return 0.0f;

    std::vector<float> kappas;
    kappas.reserve(points.size());

    // Step 1: Calculate kappa for each reference point
    for (size_t i = 1; i + 1 < points.size(); ++i) {
        float x1 = points[i - 1][0];
        float y1 = points[i - 1][1];
        float x2 = points[i][0];
        float y2 = points[i][1];
        float x3 = points[i + 1][0];
        float y3 = points[i + 1][1];

        float dx1 = x2 - x1;
        float dy1 = y2 - y1;
        float dx2 = x3 - x2;
        float dy2 = y3 - y2;

        float dx = (dx1 + dx2) * 0.5f;
        float dy = (dy1 + dy2) * 0.5f;
        float ddx = dx2 - dx1;
        float ddy = dy2 - dy1;
        // Calculate numerator and denominator part
        float num = std::fabs(dx * ddy - dy * ddx);
        float den = std::pow(dx * dx + dy * dy, 1.5f);

        if (den > 1e-6f) {
            kappas.push_back(num / den);
        }
    }

    if (kappas.empty()) return 0.0f;

    // Step 2: moving average
    std::vector<float> smoothed(kappas.size());
    int window = 5; // The sliding moving average
    int half = window / 2;

    for (size_t i = 0; i < kappas.size(); ++i) {
        float sum = 0.0f;
        int count = 0;
        for (int j = -half; j <= half; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < (int)kappas.size()) {
                sum += kappas[idx];
                count++;
            }
        }
        if (count != 0){ smoothed[i] = sum / count; }
    }

    // Step 3: Take percentile 95 to avoid the extreme noise value
    std::sort(smoothed.begin(), smoothed.end());
    size_t idx = static_cast<size_t>(0.95f * (smoothed.size() - 1));

    return smoothed[idx];
}

