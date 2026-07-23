#include "safety_force_field.h"
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <limits>

LogUtility SafetyForceField::log("log/", "SFF");

SafetyForceField::SafetyForceField() : sffSet_state_() , sffOverlap()
{
    // sffObjSet_.emplace_back(sffSet_state_);
    this->in_braking             = false;
    this->is_reset_steer_filter_ = true;
    this->decisionChecking       = SafetyForceField::SFFDecision::SAFE;
    // log.enableLogFile();
}

float SafetyForceField::normalizeAngleToPi(float inputValue) 
{
    while (inputValue >  M_PI) 
    {
        inputValue -= 2.f * M_PI;
    }
    while (inputValue < -M_PI) 
    {
        inputValue += 2.f * M_PI;
    }
    return SafetyForceField::roundNum(inputValue);
}

float SafetyForceField::convertYaw(float yaw_carla) 
{
    float yaw_new = 90.0f - yaw_carla;
    if (yaw_new < 0) 
    {
        yaw_new += 360.0f;
    }
    return yaw_new;
}

std::string SafetyForceField::getDateTimeString() 
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_time);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    return oss.str();
}


// get width of object
float SafetyForceField::getWidthCar(SafetyForceField::ObjectType object_type)
{
    if (object_type == SafetyForceField::ObjectType::EGO)
    {
        return VEHICLE_WIDTH_TOTAL;
    }
    if (object_type == SafetyForceField::ObjectType::OBJECT)
    {
        return VEHICLE_WIDTH_TOTAL - OFFSET_WIDTH_CAR_OBJECT;
    }
    if (object_type == SafetyForceField::ObjectType::UNDEFINED_OBJECT)
    {
        return UNDEFINED_WIDTH;
    }
    return 0.0f;
}

float SafetyForceField::dot2(const cv::Point2f& pointA, const cv::Point2f& pointB) 
{
    return pointA.x*pointB.x + pointA.y*pointB.y;
}
float SafetyForceField::length(const cv::Point2f& pointA) 
{
    return std::sqrt(pointA.x*pointA.x + pointA.y*pointA.y);
}
cv::Point2f SafetyForceField::axisLong(float yaw) 
{            
    return { std::cos(yaw), std::sin(yaw) };
}
cv::Point2f SafetyForceField::axisLat(float yaw) 
{           
    return { -std::sin(yaw), std::cos(yaw) };
}

float SafetyForceField::projExtentOBBOnAxis(const SafetyForceField::OrientedBoxShape& box,
                                const cv::Point2f& axis_unit)
{
    cv::Point2f ex = SafetyForceField::axisLong(box.yaw);
    cv::Point2f ey = SafetyForceField::axisLat(box.yaw);
    float r = std::fabs(SafetyForceField::dot2(ex, axis_unit)) * box.half_l
            + std::fabs(SafetyForceField::dot2(ey, axis_unit)) * box.half_w;
    return r;
}


bool SafetyForceField::overlapOBBOBB(const OrientedBoxShape& A, const OrientedBoxShape& B)
{
    const float EPS = 1e-6f;

    cv::Point2f u0 = SafetyForceField::axisLong(A.yaw);
    cv::Point2f u1 = SafetyForceField::axisLat(A.yaw);
    cv::Point2f v0 = SafetyForceField::axisLong(B.yaw);
    cv::Point2f v1 = SafetyForceField::axisLat(B.yaw);

    float R00 = SafetyForceField::dot2(u0,v0);
    float R01 = SafetyForceField::dot2(u0,v1);
    float R10 = SafetyForceField::dot2(u1,v0);
    float R11 = SafetyForceField::dot2(u1,v1);

    float AbsR00 = std::fabs(R00) + EPS;
    float AbsR01 = std::fabs(R01) + EPS;
    float AbsR10 = std::fabs(R10) + EPS;
    float AbsR11 = std::fabs(R11) + EPS;

    cv::Point2f t = B.center - A.center;
    float t0 = SafetyForceField::dot2(t, u0);
    float t1 = SafetyForceField::dot2(t, u1);

    float a0 = A.half_l;
    float a1 = A.half_w;
    float b0 = B.half_l;
    float b1 = B.half_w;

    if (std::fabs(t0) > a0 + b0*AbsR00 + b1*AbsR01) 
    {
        return false;
    }
    if (std::fabs(t1) > a1 + b0*AbsR10 + b1*AbsR11) 
    {
        return false;
    }

    float tB0 = std::fabs(t0*R00 + t1*R10);
    float tB1 = std::fabs(t0*R01 + t1*R11);

    if (tB0 > b0 + a0*AbsR00 + a1*AbsR10) 
    {
        return false;
    }
    if (tB1 > b1 + a0*AbsR01 + a1*AbsR11) 
    {
        return false;
    }

    return true;
}

cv::RotatedRect SafetyForceField::toRotRect(const OrientedBoxShape& obb) 
{
    float angle_deg;
    if (obb.yaw >= 0) 
    { 
        // 0 -> +pi maps to 360 -> 180
        angle_deg = 360.0 - (obb.yaw / M_PI) * 180.0;
    } else 
    {
        // 0 -> -pi maps to 0 -> 180
        angle_deg = (std::abs(obb.yaw) / M_PI) * 180.0;
    }
    if (angle_deg >= 360.0) 
    {
        angle_deg -= 360.0;
    }
    if (angle_deg < 0.0) 
    {
        angle_deg += 360.0;
    }

    return cv::RotatedRect(
        obb.center,
        cv::Size2f(2.0f * obb.half_l, 2.0f  * obb.half_w), // width, height
        SafetyForceField::roundNum(angle_deg)
    );
}

bool SafetyForceField::intersectOBB_OBB(const OrientedBoxShape& boxA, const OrientedBoxShape& boxB) 
{
    std::vector<cv::Point2f> interPts;
    int itype = cv::rotatedRectangleIntersection(SafetyForceField::toRotRect(boxA), SafetyForceField::toRotRect(boxB), interPts);
    return (itype == cv::INTERSECT_PARTIAL || itype == cv::INTERSECT_FULL);
}

bool SafetyForceField::intersectCircle_Circle(const CircleShape& circleA, const CircleShape& circleB) 
{
    cv::Point2f distance = circleA.center - circleB.center;
    float radius_to_radius = (circleA.radius + circleB.radius);
    return (distance.x * distance.x + distance.y * distance.y) <= (radius_to_radius * radius_to_radius);
}

bool SafetyForceField::intersectCircle_OBB(const CircleShape& circleA, const OrientedBoxShape& boxB) 
{
    const float cosY = std::cos(boxB.yaw);
    const float sinY = std::sin(boxB.yaw);
    cv::Point2f rel = circleA.center - boxB.center;

    float xL =  cosY * rel.x + sinY * rel.y;
    float yL = -sinY * rel.x + cosY * rel.y;

    float half_x = boxB.half_l; 
    float half_y = boxB.half_w; 

    float dx = std::max(std::abs(xL) - half_x, 0.0f);
    float dy = std::max(std::abs(yL) - half_y, 0.0f);
    return (dx*dx + dy*dy) <= (circleA.radius * circleA.radius);
}

bool SafetyForceField::statesIntersect(const ClaimState& ego, const ClaimState& obj) 
{
    if (ego.shape_type == SafetyForceField::ShapeType::OBB && obj.shape_type == SafetyForceField::ShapeType::OBB) 
    {
        return SafetyForceField::overlapOBBOBB(ego.OBB, obj.OBB);
    }
    // if (ego.shape_type == SafetyForceField::ShapeType::CIRCLE && obj.shape_type == SafetyForceField::ShapeType::CIRCLE) 
    // {
    //     return SafetyForceField::intersectCircle_Circle(ego.circle, obj.circle);
    // }
    // if (ego.shape_type == SafetyForceField::ShapeType::CIRCLE && obj.shape_type == SafetyForceField::ShapeType::OBB) 
    // {
    //     return SafetyForceField::intersectCircle_OBB(ego.circle, obj.OBB);
    // }
    if (ego.shape_type == SafetyForceField::ShapeType::OBB && obj.shape_type == SafetyForceField::ShapeType::CIRCLE) 
    {
        return SafetyForceField::intersectCircle_OBB(obj.circle, ego.OBB);
    }
    return false;
}

float SafetyForceField::SteerFilter::update(float delta_cmd, float v, float dt) 
{
    const float eps_error_cmd = SafetyForceField::roundNum(0.8f * M_PI / 180.0f);
    const float eps_error_rate  = SafetyForceField::roundNum(2.0f * M_PI / 180.0f);
    // Change rate limit when change direction or release turn
    // float rate_     = rate_lim;
    float cmd_error   = delta_cmd - delta_prev; // rad
    return cmd_error;

    //-- NOTES: In the initial phase of developing SFF, the EPS is not integrated into System and the Wheel Angle node is not implemented,
    //-- therefore the steeringFilter is essential to slow down the rate of change of steering and filter it to enhance the SFF precision
    //-- Now, after 2 of them are implemented, the filter should be ignore when using EPS to Control vehicle
    //-- If you use the Keyboard to control, you can uncomment the below code to have a comfortable feeling
    //-- -> https://jira.banvien.com.vn/browse/BDCI-1173

    // if(std::abs(cmd_error) < eps_error_cmd) 
    // {
    //     cmd_error = 0.0f;
    // }
    // float cmd_rate = std::abs(cmd_error) / std::max(dt, 1e-6f); // rad/s
    // if(std::abs(cmd_rate) < eps_error_rate)
    // {
    //     cmd_rate = 0.0f;
    // }
    // // coefficient
    // float base        = rate_lim;
    // float kp          = 0.03f;
    // float return_gain = 0.6f;
    // float switch_gain = 1.4f;
    // float min_rate    = SafetyForceField::roundNum(10 * M_PI / 180.0f);
    // float max_rate    = SafetyForceField::roundNum(180 * M_PI / 180.0f);  

    // // adaptive increment (delta change -> smooth) (use all case: same, opposite direction and no turn)
    // float incr = kp * cmd_rate;

    // // boost when release to center
    // if (std::fabs(delta_cmd) < 1e-2f) 
    // {
    //     delta_prev = 0.0f;
    //     return 0.0f; 
    //     // incr += return_gain * std::min(cmd_rate, min_rate); // cap cmd_rate used here
    // }

    // // boost when switching side
    // if ((delta_cmd > 0 && delta_prev < 0) || (delta_cmd < 0 && delta_prev > 0)) 
    // {
    //     base *= switch_gain; // multiply base only for this update
    // }

    // // compute final rate and clamp
    // float rate_ = clamp(base + incr, min_rate, max_rate);


    // // Rate limit 
    // float max_step = rate_ * dt;
    // float step = clamp(delta_cmd - delta_prev, -max_step, +max_step);
    // float delta_rl = delta_prev + step;

    // // Low-pass
    // float tau = clamp(tau0 + tau_k * v, 0.10f, 0.60f);
    // float alpha = dt / (tau + dt);
    // float delta_lp = delta_prev + alpha * (delta_rl - delta_prev);

    // const float delta_max = SafetyForceField::roundNum(37.f * M_PI / 180.f);  // rad
    // delta_lp = clamp(delta_lp, -delta_max, +delta_max);

    // delta_prev = delta_lp;
    // return delta_lp;
}

void SafetyForceField::SteerFilter::reset()
{
    *this = SteerFilter();
}

float SafetyForceField::curvatureFromSteering(float delta_rad, float L, bool steerLeftPositive) 
{
    if (std::fabs(delta_rad) < 1e-4f) 
    {
        return 0.0f;
    }
    const float t = std::tan(delta_rad);
    if (std::fabs(t) < 1e-6f)
    {
        return 0.0f;
    } 
    const float R_rear = L / t;

    const float R_cg = std::sqrt(R_rear*R_rear + (L*0.5f)*(L*0.5f));
    float kappa_mag = 1.f / R_cg; 
    float sgn;
    if (delta_rad > 0.0f)
    {
        sgn = 1.0f;
    }
    else
    {
        sgn = -1.0f;
    }
    return steerLeftPositive ? (sgn * kappa_mag) : (-sgn * kappa_mag);
}

float SafetyForceField::roundNum(const float num, int precision)
{
    float factor = std::pow(10.0f, precision);
    return std::round(num * factor) / factor;
}
cv::Point2f SafetyForceField::roundPoint2f(const cv::Point2f& point_, int precision)
{
    float factor = std::pow(10.0f, precision);
    return cv::Point2f(
        std::round(point_.x * factor) / factor,
        std::round(point_.y * factor) / factor
    );
}

bool SafetyForceField::convertClaimsetStop(const ClaimSet ego_claimset, ClaimSet& output_claimset)
{
    // function just use for update drawing ego nearing stop state-> Not use for calculating
    if(ego_claimset.claim_state_vector.size() == 1)
    {
        ClaimState cs_;
        output_claimset = ego_claimset;
        output_claimset.claim_state_vector.clear();
        output_claimset.claim_state_vector.reserve(2);
        float braking_distance = ego_claimset.claim_state_vector[0].OBB.half_l - VEHICLE_LENGTH_TOTAL/2.0f;
        // Update for the first point
        cs_ = ego_claimset.claim_state_vector[0];
        cs_.OBB.half_l = VEHICLE_LENGTH_TOTAL/2.0f;
        cs_.path_length = VEHICLE_LENGTH_TOTAL/2.0f;
        output_claimset.claim_state_vector.emplace_back(cs_);
        output_claimset.claim_state_vector.emplace_back(cs_);

        // Update for the second point
        if (output_claimset.claim_state_vector.size() > 1)
        {
            output_claimset.claim_state_vector[1].pos.y        = VEHICLE_LENGTH_TOTAL/2.0f + braking_distance/2.0f;
            output_claimset.claim_state_vector[1].OBB.center.y = VEHICLE_LENGTH_TOTAL/2.0f + braking_distance/2.0f;
            output_claimset.claim_state_vector[1].OBB.half_l   = braking_distance/2.0f;
            output_claimset.claim_state_vector[1].path_length  = braking_distance/2.0f;
            output_claimset.claim_state_vector[1].t            = 0.01; // => determine this point in reaction zone
        }
        output_claimset.time_danger = 0.0f;
        output_claimset.time_reaction = 0.01;

        return true;
    }
    else
    {
        return false;
    }
    
}
// Helper function SolveCubic is used for solving root of 3-order function ax^3 + bx^2 + cx + d = 0
std::vector<float> solveCubic(float &a, float &b, float &c, float &d) 
{
    std::vector<float> roots;

    // If a = 0 -> Solve 2-order equation
    if (fabs(a) < 1e-12) {
        if (fabs(b) < 1e-12) {
            if (fabs(c) < 1e-12) return roots; // no solution
            roots.push_back(-d / c);
            return roots;
        }
        float delta = c * c - 4 * b * d;
        if (delta < 0) return roots;
        roots.push_back((-c + sqrt(delta)) / (2 * b));
        roots.push_back((-c - sqrt(delta)) / (2 * b));
        return roots;
    }

    // Eliminate 2-order: x^3 + px + q = 0
    float A = b / a;
    float B = c / a;
    float C = d / a;

    float p = B - A * A / 3.0;
    float q = (2.0 * A * A * A) / 27.0 - (A * B) / 3.0 + C;

    float discriminant = (q * q) / 4.0 + (p * p * p) / 27.0;

    if (discriminant > 1e-12) {
        // One root
        float sqrtD = sqrt(discriminant);
        float u = cbrt(-q / 2.0 + sqrtD);
        float v = cbrt(-q / 2.0 - sqrtD);
        float x1 = u + v - A / 3.0;
        roots.push_back(x1);
    } 
    else if (fabs(discriminant) <= 1e-12) {
        // Two same roots
        float u = cbrt(-q / 2.0);
        float x1 = 2 * u - A / 3.0;
        float x2 = -u - A / 3.0;
        roots.push_back(x1);
        roots.push_back(x2);
    } 
    else {
        // Three different roots
        float phi = acos(-q / (2.0 * sqrt(-(p * p * p) / 27.0)));
        float r = 2 * sqrt(-p / 3.0);
        float x1 = r * cos(phi / 3.0) - A / 3.0;
        float x2 = r * cos((phi + 2 * M_PI) / 3.0) - A / 3.0;
        float x3 = r * cos((phi + 4 * M_PI) / 3.0) - A / 3.0;
        roots.push_back(x1);
        roots.push_back(x2);
        roots.push_back(x3);
    }

    return roots;
}



float SafetyForceField::predictEgoDecelerationLowVel(const float input_vel, const float input_acc, const float time, float& total_time, bool& is_phase_12)
{
    static float latency = 0.0f;
    #ifdef ENABLE_CAN
    latency = 0.15;
    #endif
    // use when input_vel < 40 km/h
    float time_phase_0_positive = 0.0f;
    float time_phase_0_negative = 0.0f;
    float initial_acc_phase_1 = 0.0f;
    float vel_phase_1_only = 12;
    float time_phase_1 = this->a_param_Time_jerk_phase_1_low*input_vel*input_vel + this->b_param_Time_jerk_phase_1_low*input_vel + this->c_param_Time_jerk_phase_1_low;
    float time_phase_2 = this->a_param_Time_jerk_phase_2_low*input_vel*input_vel + this->b_param_Time_jerk_phase_2_low*input_vel + this->c_param_Time_jerk_phase_2_low;
    total_time = time_phase_1 + time_phase_2 + latency;
    // log.i("input_vel:", input_vel, "; time_phase_1:", time_phase_1, " ; time_phase_2:", time_phase_2, " ; total_time:", total_time);   

    if (input_acc >= 0.5)
    {
        float time_estimate_nonAcc = this->predictEgoMaximumDeceleration(input_vel);
        float time_estimate_Acc = 0;
        if (input_acc >= 6)
        {
            time_estimate_Acc = this->a68_param_A_timeBraking_low*input_vel*input_vel + this->a68_param_B_timeBraking_low*input_vel + this->a68_param_C_timeBraking_low;
            float offset = 0.04; // low 25
            if (input_vel > 30 && input_vel < 34)
                offset = 0.022;
            if (input_vel >= 34)
                offset = -0.3;
            // if (input_vel >= 20 && input_vel <= 30)
            //     offset = 0.0;
            // float offset = 0.0475; //high
            // float offset = 0.09; //low
            // float offset = 0.2; // high 31 -> 
            // if (input_vel > 30)
            //     offset = -0.02;
            time_estimate_Acc += offset;
        }
        else if (input_acc >= 4 && input_acc < 6)
        {
            time_estimate_Acc = this->a46_param_A_timeBraking_low*input_vel*input_vel + this->a46_param_B_timeBraking_low*input_vel + this->a46_param_C_timeBraking_low;
            // Offset to eliminate error following velocity
            // float offset = 0.035;
            float offset = 0.037; //20 -> 30
            time_estimate_Acc += offset;
        }
        else if (input_acc >= 2 && input_acc < 4)
        {
            time_estimate_Acc = this->a24_param_A_timeBraking_low*input_vel*input_vel + this->a24_param_B_timeBraking_low*input_vel + this->a24_param_C_timeBraking_low;
            float offset = 0.01;
            time_estimate_Acc += offset;
        }
        else if (input_acc > 0 && input_acc < 2)
        {
            time_estimate_Acc = this->a02_param_A_timeBraking_low*input_vel*input_vel + this->a02_param_B_timeBraking_low*input_vel + this->a02_param_C_timeBraking_low;
            float offset = 0.01;
            time_estimate_Acc += offset;
        }
        // Postive initial acceleration
        time_phase_0_positive = time_estimate_Acc - time_estimate_nonAcc;
        initial_acc_phase_1 = this->a_param_D_jerk_phase_1_low*input_vel + this->b_param_D_jerk_phase_1_low;
        
        // Update total_time
        total_time += time_phase_0_positive;
        // Estimate parameters of Phase 0
        this->a_param_acc_phase0 = (initial_acc_phase_1 - input_acc) / (time_phase_0_positive*time_phase_0_positive);
    }
    else if (input_acc <= -0.5)
    {   
        // Get exact timestamp of Input acceleration which is in Phase 1.1
        float a_coef_jerk = this->a_param_A_jerk_phase_1_low*input_vel*input_vel*input_vel + this->b_param_A_jerk_phase_1_low*input_vel*input_vel +
                        this->c_param_A_jerk_phase_1_low*input_vel + this->d_param_A_jerk_phase_1_low;

        float b_coef_jerk = this->a_param_B_jerk_phase_1_low*input_vel*input_vel*input_vel + this->b_param_B_jerk_phase_1_low*input_vel*input_vel +
                         this->c_param_B_jerk_phase_1_low*input_vel + this->d_param_B_jerk_phase_1_low;

        float c_coef_jerk = this->a_param_C_jerk_phase_1_low*input_vel + this->b_param_C_jerk_phase_1_low;
        float d_coef_jerk = this->a_param_D_jerk_phase_1_low*input_vel + this->b_param_D_jerk_phase_1_low - input_acc;
        auto roots = solveCubic(a_coef_jerk, b_coef_jerk, c_coef_jerk, d_coef_jerk);
        for (float root : roots)
        {
            if (root > 0 && root <= time_phase_1)
                time_phase_0_negative = std::max(double(0), (root - 0.05));
        }
        // time_phase_0_negative -= 0.02;
        total_time -= time_phase_0_negative;
    }

    // log.i("input_vel:", input_vel, "input_acc:", input_acc, "; time_phase_0_negative:", time_phase_0_negative, "; time_phase_0_positive:", time_phase_0_positive, "; a_param_acc_phase0:", this->a_param_acc_phase0);   

    float a_coef_jerk;
    float b_coef_jerk;
    float c_coef_jerk;
    float d_coef_jerk;
    float deceleration = -35.0f;
    
    if (time <= latency)
        return input_acc;

    if (time_phase_0_positive != 0 && (time - latency) <= time_phase_0_positive)
    {
        float t = time - latency;
        deceleration = this->a_param_acc_phase0*t*t + input_acc;   
        return deceleration;
    }

    else if ((time + time_phase_0_negative - time_phase_0_positive - latency) <= time_phase_1 || input_vel < vel_phase_1_only)
    {
        float t = time + time_phase_0_negative - time_phase_0_positive - latency;
        a_coef_jerk = this->a_param_A_jerk_phase_1_low*input_vel*input_vel*input_vel + this->b_param_A_jerk_phase_1_low*input_vel*input_vel +
                        this->c_param_A_jerk_phase_1_low*input_vel + this->d_param_A_jerk_phase_1_low;

        b_coef_jerk = this->a_param_B_jerk_phase_1_low*input_vel*input_vel*input_vel + this->b_param_B_jerk_phase_1_low*input_vel*input_vel +
                         this->c_param_B_jerk_phase_1_low*input_vel + this->d_param_B_jerk_phase_1_low;

        c_coef_jerk = this->a_param_C_jerk_phase_1_low*input_vel + this->b_param_C_jerk_phase_1_low;
        d_coef_jerk = this->a_param_D_jerk_phase_1_low*input_vel + this->b_param_D_jerk_phase_1_low;
        // log.i("Phase 1 -- a_coef_jerk:", a_coef_jerk, " ; b_coef_jerk:", b_coef_jerk, " ; c_coef_jerk:", c_coef_jerk, " ; d_coef_jerk:", d_coef_jerk);   
        deceleration = a_coef_jerk*t*t*t + b_coef_jerk*t*t +c_coef_jerk*t + d_coef_jerk;
        deceleration = SafetyForceField::roundNum(deceleration, 5);
        // log.i("Phase 1 -- deceleration:", deceleration);   
        if (deceleration > 0)
        {
            deceleration = 0.0f;
        }
        if (time + time_phase_0_negative - time_phase_0_positive +0.01 > time_phase_1)
        {
            if (!is_phase_12)
            {
                is_phase_12 = true;
            }
        }
        
        return deceleration;
    }
    else if ((time + time_phase_0_negative - time_phase_0_positive - latency) <= time_phase_1 + time_phase_2)
    {
        float t = time + time_phase_0_negative - time_phase_0_positive - latency;
        a_coef_jerk = this->a_param_A_jerk_phase_2_low*input_vel*input_vel*input_vel + this->b_param_A_jerk_phase_2_low*input_vel*input_vel +
                        this->c_param_A_jerk_phase_2_low*input_vel + this->d_param_A_jerk_phase_2_low;

        b_coef_jerk = this->a_param_B_jerk_phase_2_low*input_vel*input_vel*input_vel + this->b_param_B_jerk_phase_2_low*input_vel*input_vel +
                         this->c_param_B_jerk_phase_2_low*input_vel + this->d_param_B_jerk_phase_2_low;

        c_coef_jerk = this->a_param_C_jerk_phase_2_low*input_vel*input_vel*input_vel + this->b_param_C_jerk_phase_2_low*input_vel*input_vel +
                         this->c_param_C_jerk_phase_2_low*input_vel + this->d_param_C_jerk_phase_2_low;

        d_coef_jerk = this->a_param_D_jerk_phase_2_low*input_vel*input_vel*input_vel + this->b_param_D_jerk_phase_2_low*input_vel*input_vel +
                         this->c_param_D_jerk_phase_2_low*input_vel + this->d_param_D_jerk_phase_2_low;
        // log.i("Phase 2 -- a_coef_jerk:", a_coef_jerk, " ; b_coef_jerk:", b_coef_jerk, " ; c_coef_jerk:", c_coef_jerk, " ; d_coef_jerk:", d_coef_jerk);   
        deceleration = a_coef_jerk*t*t*t + b_coef_jerk*t*t + c_coef_jerk*t + d_coef_jerk;
        // log.i("Phase 2 -- deceleration:", deceleration);   
        deceleration = SafetyForceField::roundNum(deceleration, 5);
        // log.i("Phase 2 -- round deceleration:", deceleration);   
        return deceleration;
    }
    return deceleration;
}

float SafetyForceField::predictEgoDecelerationHighVel(const float input_vel /*km/h*/, const float input_acc, const float time, float& total_time, bool& is_phase_12)
{
    static float latency = 0.0f;
    #ifdef ENABLE_CAN
    latency = 0.15;
    #endif
    // use when input_vel > 40 km/h
    float time_phase_0_positive = 0.0f;
    float initial_acc_phase_11 = 0.0f;
    float time_phase_0_negative = 0.0f;
    float time_phase_11 = this->a_param_Time_jerk_phase_11*input_vel*input_vel + this->b_param_Time_jerk_phase_11*input_vel + this->c_param_Time_jerk_phase_11;
    float time_phase_12 = this->a_param_Time_jerk_phase_12*input_vel*input_vel + this->b_param_Time_jerk_phase_12*input_vel + this->c_param_Time_jerk_phase_12;
    float time_phase_13 = this->a_param_Time_jerk_phase_13*input_vel*input_vel + this->b_param_Time_jerk_phase_13*input_vel + this->c_param_Time_jerk_phase_13;
    float time_phase_2 = this->a_param_Time_jerk_phase_2*input_vel*input_vel + this->b_param_Time_jerk_phase_2*input_vel + this->c_param_Time_jerk_phase_2;
    total_time = time_phase_11 + time_phase_12 + time_phase_13 + time_phase_2 + latency;
    // log.i("input_vel:", input_vel, "; time_phase_11:", time_phase_11, " ; time_phase_12:", time_phase_12, " ; time_phase_13:", time_phase_13, " ; time_phase_2:", time_phase_2);   

    if (input_acc >= 1)
    {
        float time_estimate_nonAcc = this->predictEgoMaximumDeceleration(input_vel);
        float time_estimate_Acc = 0;
        if (input_acc >= 6)
        {
            time_estimate_Acc = this->a68_param_A_timeBraking_high*input_vel*input_vel + this->a68_param_B_timeBraking_high*input_vel + this->a68_param_C_timeBraking_high;
            float offset = 0.12;
            if (input_vel > 40 && input_vel < 55)
                offset += 0.032;
            if (input_vel > 55 && input_vel < 70)
                offset += 0.065;
            if (input_vel > 70 && input_vel < 85)
                offset += 0.05;
            if (input_vel > 85 && input_vel < 100)
                offset = 0.05;
            time_estimate_Acc += offset;
        }
        else if (input_acc >= 4 && input_acc < 6)
        {
            time_estimate_Acc = this->a46_param_A_timeBraking_high*input_vel*input_vel + this->a46_param_B_timeBraking_high*input_vel + this->a46_param_C_timeBraking_high;
            // Offset to eliminate error following vselocity
            float offset = 0.03;
            time_estimate_Acc += offset;
        }
        else if (input_acc >= 2 && input_acc < 4)
        {
            time_estimate_Acc = this->a24_param_A_timeBraking_high*input_vel*input_vel + this->a24_param_B_timeBraking_high*input_vel + this->a24_param_C_timeBraking_high;
            float offset = 0.03;
            if (input_vel > 70 && input_vel < 95)
                offset = -0.025;
            time_estimate_Acc += offset;
        }
        else if (input_acc > 0 && input_acc < 2)
        {
            time_estimate_Acc = this->a02_param_A_timeBraking_high*input_vel*input_vel + this->a02_param_B_timeBraking_high*input_vel + this->a02_param_C_timeBraking_high;
            float offset = 0.02;
            if (input_vel > 70 && input_vel < 95)
                offset = 0.035;
            time_estimate_Acc += offset;
        }
        // if (time_estimate_Acc > time_estimate_nonAcc)
        // {
            // Postive initial acceleration
            time_phase_0_positive = time_estimate_Acc - time_estimate_nonAcc;
            initial_acc_phase_11 = input_vel*a_param_C_jerk_phase_11 + b_param_C_jerk_phase_11;
            // Update total_time
            total_time += time_phase_0_positive;
            // Estimate parameters of Phase 0
            this->a_param_acc_phase0 = (initial_acc_phase_11 - input_acc) / (time_phase_0_positive*time_phase_0_positive);
    }
    else if (input_acc <= -0.5)
    {    
        // Negative initial acceleration
        // This value is used for shifting the timestamp to corresponding moment of kinematic model
        // time_phase_0_negative = time_estimate_Acc - time_estimate_nonAcc;
        // total_time += time_phase_0_negative;

        // Get exact timestamp of Input acceleration which is in Phase 1.1
        float a_coef_jerk = this->a_param_A_jerk_phase_11*input_vel + this->b_param_A_jerk_phase_11;
        float b_coef_jerk = this->a_param_B_jerk_phase_11*input_vel + this->b_param_B_jerk_phase_11;
        float c_coef_jerk = this->a_param_C_jerk_phase_11*input_vel + this->b_param_C_jerk_phase_11;
        float delta = b_coef_jerk*b_coef_jerk - 4*a_coef_jerk*(c_coef_jerk - input_acc);
        float root_1, root_2;
        root_1 = (-b_coef_jerk + std::sqrt(delta)) / (2*a_coef_jerk);
        root_2 = (-b_coef_jerk - std::sqrt(delta)) / (2*a_coef_jerk);
        if (root_1 < time_phase_11)
            time_phase_0_negative = root_1;
        else if (root_2 < time_phase_11)
            time_phase_0_negative = root_2;
        else
            time_phase_0_negative = 0;
        // log.i("root_1:", root_1, "; root_2:", root_2);   
        time_phase_0_negative -= 0.02;
        total_time -= time_phase_0_negative;
    }

    float a_coef_jerk;
    float b_coef_jerk;
    float c_coef_jerk;
    float deceleration = -35.0f;

    if (time <= latency)
        return input_acc;

    if (time_phase_0_positive != 0 && (time - latency) <= time_phase_0_positive)
    {
        float t = time - latency;
        deceleration = this->a_param_acc_phase0*t*t + input_acc;
        // log.i("time:", time, "; acc_phase_0:", deceleration);   
        return deceleration;
    }

    else if ((time + time_phase_0_negative - time_phase_0_positive - latency) <= time_phase_11)
    // if (time <= time_phase_11)
    {
        float t = time + time_phase_0_negative - time_phase_0_positive - latency;
        a_coef_jerk = this->a_param_A_jerk_phase_11*input_vel + this->b_param_A_jerk_phase_11;
        b_coef_jerk = this->a_param_B_jerk_phase_11*input_vel + this->b_param_B_jerk_phase_11;
        c_coef_jerk = this->a_param_C_jerk_phase_11*input_vel + this->b_param_C_jerk_phase_11;
        deceleration = a_coef_jerk*t*t + b_coef_jerk*t+c_coef_jerk;
        deceleration = SafetyForceField::roundNum(deceleration, 5);
        // log.i("Phase 11 -- a_coef_jerk:", a_coef_jerk, " ; b_coef_jerk:", b_coef_jerk, " ; c_coef_jerk:", c_coef_jerk, " ; deceleration:", deceleration);   
        return deceleration;
    }
    else if ((time + time_phase_0_negative - time_phase_0_positive - latency) <= time_phase_11+time_phase_12)
    {
        float t = time + time_phase_0_negative - time_phase_0_positive - latency;
        a_coef_jerk = this->a_param_A_jerk_phase_12*input_vel + this->b_param_A_jerk_phase_12;
        b_coef_jerk = this->a_param_B_jerk_phase_12*input_vel + this->b_param_B_jerk_phase_12;
        deceleration = a_coef_jerk*t + b_coef_jerk;
        deceleration = SafetyForceField::roundNum(deceleration, 5);
        // log.i("Phase 12 -- a_coef_jerk:", a_coef_jerk, " ; b_coef_jerk:", b_coef_jerk, " ; deceleration:", deceleration);   

        if (t + 0.01 > time_phase_11+time_phase_12)
        {
            if (!is_phase_12)
            {
                is_phase_12 = true;
            }
        }
        return deceleration;
    }
    else if ((time + time_phase_0_negative - time_phase_0_positive - latency) <= time_phase_11 + time_phase_12 + time_phase_13)
    {
        float t = time + time_phase_0_negative - time_phase_0_positive - latency;
        a_coef_jerk = this->a_param_A_jerk_phase_13*input_vel*input_vel*input_vel + this->b_param_A_jerk_phase_13*input_vel*input_vel +
                        this->c_param_A_jerk_phase_13*input_vel + this->d_param_A_jerk_phase_13;
        b_coef_jerk = this->a_param_B_jerk_phase_13*input_vel*input_vel*input_vel + this->b_param_B_jerk_phase_13*input_vel*input_vel +
                         this->c_param_B_jerk_phase_13*input_vel + this->d_param_B_jerk_phase_13;
        deceleration = a_coef_jerk*t + b_coef_jerk;
        deceleration = SafetyForceField::roundNum(deceleration, 5);            
        // log.i("Phase 13 -- a_coef_jerk:", a_coef_jerk, " ; b_coef_jerk:", b_coef_jerk, " ; deceleration:", deceleration);   
        return deceleration;
    }
    else if ((time + time_phase_0_negative - time_phase_0_positive) <= time_phase_11 + time_phase_12 + time_phase_13 + time_phase_2)
    {
        float t = time + time_phase_0_negative - time_phase_0_positive;
        a_coef_jerk = this->a_param_A_jerk_phase_2*input_vel*input_vel + this->b_param_A_jerk_phase_2*input_vel +
                        this->c_param_A_jerk_phase_2;
        b_coef_jerk = this->a_param_B_jerk_phase_2*input_vel*input_vel+ this->b_param_B_jerk_phase_2*input_vel +
                         this->c_param_B_jerk_phase_2;
        
        deceleration = a_coef_jerk*t + b_coef_jerk; 
        deceleration = SafetyForceField::roundNum(deceleration, 5);
        // log.i("Phase 2 -- a_coef_jerk:", a_coef_jerk, " ; b_coef_jerk:", b_coef_jerk, " ; deceleration:", deceleration);   
        return deceleration;
    }
    return deceleration;
}


float SafetyForceField::predictEgoMaximumDeceleration(const float input_velocity)
{
    // if (input_velocity <= 10)
    // {
    //     return max_dec_const_low;
    // } 
    // else if (input_velocity > 10 && input_velocity < 40)
    // {
    //     return (maxL_dec_a_coeff * input_velocity + maxL_dec_b_coeff);
    // }
    // else if (input_velocity >= 40 && input_velocity <= 120)
    // {
    //     return (maxH_dec_a_coeff * input_velocity + maxH_dec_b_coeff);
    //     // return (maxH_dec_a_coeff * input_velocity * input_velocity + maxH_dec_b_coeff * input_velocity + maxH_dec_c_coeff);
    // }
    // else
    // {
    //     return max_dec_const_high;
    // }
    float braking_time = 0.0202 * input_velocity - 0.0429;
    if (braking_time < 0)
        braking_time = 0;
    return braking_time;
}


bool  SafetyForceField::createOffsetZone(float cur_vel, float time_danger, ClaimState& claimset_input)
{
    float yaw = claimset_input.yaw;
    if (this->vel_lowest_range <= cur_vel && cur_vel <= this->reaction_vel_lowest_range)
    {
        claimset_input.path_length +=  this->reaction_distance_lowest_range; 
        claimset_input.OBB.half_l  = this->reaction_distance_lowest_range/2.0f;
        claimset_input.pos.x       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_lowest_range/2.0f) * std::cos(yaw));
        claimset_input.pos.y       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_lowest_range/2.0f) * std::sin(yaw)); 
        claimset_input.t           = time_danger + 0.01;
        claimset_input.v           = 0;
    }
    else if ( this->reaction_vel_lowest_range < cur_vel && cur_vel <= this->reaction_vel_low_range)
    {
        claimset_input.path_length +=  this->reaction_distance_low_range;
        claimset_input.OBB.half_l  = this->reaction_distance_low_range/2.0f;
        claimset_input.pos.x       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_low_range/2.0f) * std::cos(yaw));
        claimset_input.pos.y       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_low_range/2.0f) * std::sin(yaw)); 
        claimset_input.t           = time_danger + 0.01;
        claimset_input.v           = 0;
    }
    else if ( this->reaction_vel_low_range < cur_vel && cur_vel <= this->reaction_vel_middle_range)
    {
        claimset_input.path_length +=  this->reaction_distance_middle_range; 
        claimset_input.OBB.half_l  = this->reaction_distance_middle_range/2.0f;
        claimset_input.pos.x       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_middle_range/2.0f) * std::cos(yaw));
        claimset_input.pos.y       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_middle_range/2.0f) * std::sin(yaw)); 
        claimset_input.t           = time_danger + 0.01;
        claimset_input.v           = 0;
    }
    else if ( this->reaction_vel_middle_range < cur_vel && cur_vel <= this->reaction_vel_high_range)
    {
        claimset_input.path_length +=  this->reaction_distance_high_range; 
        claimset_input.OBB.half_l  = this->reaction_distance_high_range/2.0f;
        claimset_input.pos.x       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_high_range/2.0f) * std::cos(yaw));
        claimset_input.pos.y       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_high_range/2.0f) * std::sin(yaw)); 
        claimset_input.t           = time_danger + 0.01;
        claimset_input.v           = 0;
    }
    else if ( this->reaction_vel_high_range < cur_vel)
    {
        claimset_input.path_length +=  this->reaction_distance_highest_range; 
        claimset_input.OBB.half_l  = this->reaction_distance_highest_range/2.0f;
        claimset_input.pos.x       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_highest_range/2.0f) * std::cos(yaw));
        claimset_input.pos.y       += ((VEHICLE_LENGTH_TOTAL/2.0f + this->reaction_distance_highest_range/2.0f) * std::sin(yaw)); 
        claimset_input.t           = time_danger + 0.01;
        claimset_input.v           = 0;
    }
    else 
    {
        return false;
    }

    claimset_input.OBB.center = claimset_input.pos;
    return true;
    
    
}
bool  SafetyForceField::getTimeReactionForOffset(float cur_vel,float danger_distance, float check_distance, float cur_time,float& time_reaction)
{
    if (this->vel_lowest_range <= cur_vel && cur_vel <= this->reaction_vel_lowest_range && time_reaction == 0)
    {
        if (check_distance >= danger_distance + this->reaction_distance_lowest_range)
        {
            time_reaction = cur_time;
            return true;
            // log.i(VV"Path_reaction_last: ", check_distance);
        }
    }
    else if ( this->reaction_vel_lowest_range < cur_vel && cur_vel <= this->reaction_vel_low_range  && time_reaction == 0)
    {
        if (check_distance >= danger_distance + this->reaction_distance_low_range)
        {
            time_reaction = cur_time;
            return true;
            // log.i(VV"Path_reaction_last: ", check_distance);
        }
    }
    else if ( this->reaction_vel_low_range < cur_vel && cur_vel <= this->reaction_vel_middle_range  && time_reaction == 0)
    {
        if (check_distance >= danger_distance + this->reaction_distance_middle_range)
        {
            time_reaction = cur_time;
            return true;
            // log.i(VV"Path_reaction_last: ", check_distance);
        }
    }
    else if ( this->reaction_vel_middle_range < cur_vel && cur_vel <= this->reaction_vel_high_range  && time_reaction == 0)
    {
        if (check_distance >= danger_distance + this->reaction_distance_high_range)
        {
            time_reaction = cur_time;
            return true;
            // log.i(VV"Path_reaction_last: ", check_distance);
        }
    }
    else if ( this->reaction_vel_high_range < cur_vel && time_reaction == 0)
    {
        if (check_distance >= danger_distance + this->reaction_distance_highest_range)
        {
            time_reaction = cur_time;
            return true;
            // log.i(VV"Path_reaction_last: ", cs.path_length);
        }
    }
    return false;
}

SafetyForceField::ClaimSet SafetyForceField::buildClaimestFromVehicleState(float v0, float a0, float steering_angle, 
                                            float T_danger, float T_reaction, float T_warning, float dt, bool isDanger, float a_min)
{
    const float EPS = 1e-6f;
    const float eps_vel = 1/3.6f;
    const float eps_acc = 5e-1f;
    ClaimSet claimset;
    claimset.claim_state_vector.clear();
    claimset.state_type = SafetyForceField::ObjectStateType::DYNAMIC;
    float a_react       = std::max(a0, 0.f);
    float v_react       = std::max(0.0f, v0 + a_react * T_reaction);
    float v0_danger     = v_react;
    a_min               = this->predictEgoMaximumDeceleration(v0_danger * 3.6);
    // T_danger            = v0_danger/std::fabs(a_min);
    T_danger            = 0.0f;

    if (std::isinf(T_danger) || std::isnan(T_danger))
    {
        T_danger = 2.0f;
    }

    if (std::abs(v0) <= eps_vel && a0 <= eps_acc)
    {
        claimset.distance_brake_threshold = this->distance_low_threshold_state_stop;
        claimset.in_stop_state    = true;
        ClaimState claim_state_;
        claim_state_.v            = v0;
        claim_state_.a            = a0;
        claim_state_.t            = 0.0f;
        claim_state_.pos          = cv::Point2f(0.0f, 0.0f); //  VEHICLE_LENGTH_TOTAL/2.0f
        claim_state_.yaw          = SafetyForceField::roundNum(M_PI/2.0f);
        claim_state_.b            = 0.0f;
        claim_state_.path_length  = VEHICLE_LENGTH_TOTAL/2.0f + claimset.distance_brake_threshold;
        claim_state_.shape_type   = SafetyForceField::ShapeType::OBB;
        claim_state_.OBB.half_l   = VEHICLE_LENGTH_TOTAL/2.0f + claimset.distance_brake_threshold;
        claim_state_.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
        claim_state_.OBB.center   = claim_state_.pos;
        claim_state_.OBB.yaw      = claim_state_.yaw;
        claimset.time_danger      = 2.0f;
        claimset.time_reaction    = 0.0f;
        claimset.time_warning     = 0.0f;
        claimset.claim_state_vector.push_back(claim_state_);
        return claimset;
    }

    // --- compute curvature from steering ---
    float steering_rad_cmd = SafetyForceField::roundNum(steering_angle * M_PI / 180.0f);
    
    float dt_wall          = 0.02f; // fallback 50Hz 
    auto now_tp            = std::chrono::steady_clock::now();
    if (steer_last_tp_valid_) 
    {
        dt_wall = std::chrono::duration<float>(now_tp - steer_last_tp_).count();
        dt_wall = clamp(dt_wall, 1e-3f, 0.2f);
    }
    steer_last_tp_         = now_tp;
    steer_last_tp_valid_   = true;

    float delta_filtered   = this->steer_filter_.update(steering_rad_cmd, v0, dt_wall);
    float curvature_fixed  = SafetyForceField::curvatureFromSteering(
                                delta_filtered, VEHICLE_LENGTH_BASE, steer_left_positive_);

    // init pose
    float x                = 0.0f; 
    float y                = 0.0f;
    float yaw              = SafetyForceField::roundNum(M_PI / 2.0f);
    float v                = v0;
    float s_prev           = 0.0f;
    float path_length_     = 0.0f;

    if (!isDanger) 
    {
        // ====== CASE 1: ISDANGER = FALSE ======
        float T_h          = T_danger + T_warning;
        int stepSize       = static_cast<int>(std::floor(T_h / dt)) + 1;
        claimset.claim_state_vector.reserve(stepSize);

        for (int k = 0; k < stepSize; ++k) 
        {
            float t         = k * dt;
            float curvature = curvature_fixed;

            // record state
            ClaimState cs;
            cs.t            = t;
            cs.pos          = cv::Point2f(x, y);
            cs.yaw          = yaw;
            cs.v            = v;
            cs.a            = a0;
            cs.b            = curvature;
            cs.path_length  = path_length_;
            cs.shape_type   = SafetyForceField::ShapeType::OBB;
            cs.OBB.half_l   = VEHICLE_LENGTH_TOTAL / 2.0f;
            cs.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
            cs.OBB.center   = cs.pos;
            cs.OBB.yaw      = yaw;

            if (k == 0)
            {
                claimset.claim_state_vector.push_back(cs);
            }
            else
            {
                claimset.claim_state_vector.emplace_back(cs);
            }
                
            // integrate
            float yaw_rate = v * curvature;
            float dx       = v * std::cos(yaw) * dt;
            float dy       = v * std::sin(yaw) * dt;
            x              += dx;
            y              += dy;
            yaw            += yaw_rate * dt;
            v = std::max(0.f, v + a0 * dt);
            path_length_   += std::hypot(dx, dy);
        }

        claimset.time_danger = T_danger;
        claimset.time_warning = T_warning;
    }
    else 
    {
        // ====== CASE 2: ISDANGER = TRUE ======
        float time = 0;
        bool  is_phase_12_end = false;
        // Case: In front of wall stop and can move forward after breaking althoung in danger zone
        

        // if ((v0 <= this->vel_in_low_threshold_state_1 && a0 <= this->accel_in_low_threshold_state_1) ||
        //     v0 <= this->vel_in_low_threshold_state_2 && a0 <= this->accel_in_low_threshold_state_2) // v0_danger
        
        if (v0 <= this->vel_lowest_range)
        {
            // log.i(VV"In Low");
            float s_end_predict;
            if (std::abs(a0) <= this->accel_in_low_threshold_state_noAcc)
            {
                s_end_predict = this->distance_low_threshold_state_noAcc;
            }
            else if (std::abs(a0) <= this->accel_in_low_threshold_state_1)
            {
                s_end_predict = this->distance_low_threshold_state_1;
            }
            else if (std::abs(a0) <= this->accel_in_low_threshold_state_2)
            {
                s_end_predict = this->distance_low_threshold_state_2; //claimSet_.distance_brake_threshold;// 1.0f;
            }
            else
            {
                s_end_predict = this->distance_low_threshold_state_2; 
            }
            // float s_end_predict  = distance_low_threshold_state_1;//claimset.distance_brake_threshold;
            float A_coeff        = 0.5f * a0;
            float B_coeff        = v0;
            float C_coeff        = -s_end_predict;
            float delta          = B_coeff * B_coeff - 4 * A_coeff * C_coeff;
            if (delta < 0) 
            {
                delta = 0;
            }
            float sqrt_delta     = std::sqrt(delta);

            float t1             = (-B_coeff + sqrt_delta) / (2 * A_coeff);
            float t2             = (-B_coeff - sqrt_delta) / (2 * A_coeff);

            time                 = -1.0f;
            if (t1 >= 0 && t2 >= 0) 
            {
                time = std::min(t1, t2);
            }
            else if (t1 >= 0) 
            {
                time = t1;
            }
            else if (t2 >= 0) 
            {
                time = t2;
            }

            if (time < dt) 
            {
                time = 0.0f;        
            }    
            if (std::isnan(time) || std::isinf(time) || time == 0.0f)
            {
                claimset.distance_brake_threshold = this->distance_low_threshold_state_stop;
                claimset.in_stop_state    = true;
                ClaimState claim_state_;
                claim_state_.v            = v0;
                claim_state_.a            = a0;
                claim_state_.t            = 0.0f;
                claim_state_.pos          = cv::Point2f(0.0f, 0.0f);
                claim_state_.yaw          = SafetyForceField::roundNum(M_PI / 2.0f);
                claim_state_.b            = 0.0f;
                claim_state_.path_length  = VEHICLE_LENGTH_TOTAL/2.0f + claimset.distance_brake_threshold;
                claim_state_.shape_type   = SafetyForceField::ShapeType::OBB;
                claim_state_.OBB.half_l   = VEHICLE_LENGTH_TOTAL/2.0f + claimset.distance_brake_threshold;
                claim_state_.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
                claim_state_.OBB.center   = claim_state_.pos;
                claim_state_.OBB.yaw      = claim_state_.yaw;
                claimset.time_danger      = 2.0f;
                claimset.time_reaction    = 0.0f;
                claimset.claim_state_vector.push_back(claim_state_);
                return claimset;
            }
            if (time > 0)
            {
                v            = v0;
                int stepSize = static_cast<int>(std::floor(time / dt)) + 1;
                claimset.claim_state_vector.reserve(stepSize);

                for (int k = 0; k < stepSize; ++k) 
                {
                    float t         = k * dt;
                    float curvature = curvature_fixed;

                    ClaimState cs;
                    cs.t           = t;
                    cs.pos         = cv::Point2f(x, y);
                    cs.yaw         = yaw;
                    cs.v           = v;
                    cs.path_length = path_length_;
                    cs.a           = a0;
                    cs.b           = curvature;
                    cs.shape_type  = SafetyForceField::ShapeType::OBB;
                    cs.OBB.half_l  = VEHICLE_LENGTH_TOTAL / 2.0f;
                    cs.OBB.half_w  = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
                    cs.OBB.center  = cs.pos;
                    cs.OBB.yaw     = yaw;
                    claimset.claim_state_vector.push_back(cs);
                    
                    // integrate
                    float yaw_rate = v * curvature;
                    float dx       = v * std::cos(yaw) * dt;
                    float dy       = v * std::sin(yaw) * dt;
                    x              += dx; 
                    y              += dy;
                    yaw            += yaw_rate * dt;
                    v              = std::max(0.f, v + a0 * dt);
                    path_length_   += std::hypot(dx, dy);
                }

                T_danger               = time;
                T_warning              = 1.0f;
                // Phase 2: Warning
                v = v0;
                int stepSizeWarning = static_cast<int>(std::floor(T_warning / dt)) + 1;
                for (int k = 1; k < stepSizeWarning; ++k) 
                {
                    // float t = T_danger + k * dt;
                    float t         = time + k * dt;
                    float curvature = curvature_fixed;

                    ClaimState cs;
                    cs.t            = t;
                    cs.pos          = cv::Point2f(x, y);
                    cs.yaw          = yaw;
                    cs.v            = v;
                    cs.a            = a0;
                    cs.b            = curvature;
                    cs.shape_type   = SafetyForceField::ShapeType::OBB;
                    cs.OBB.half_l   = VEHICLE_LENGTH_TOTAL / 2.0f;
                    cs.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
                    cs.OBB.center   = cs.pos;
                    cs.OBB.yaw      = yaw;
                    
                    // integrate
                    float yaw_rate  = v * curvature;
                    float dx        = v * std::cos(yaw) * dt;
                    float dy        = v * std::sin(yaw) * dt;
                    x               += dx; 
                    y               += dy;
                    yaw             += yaw_rate * dt;
                    v               = std::max(0.f, v + a0 * dt);
                    path_length_    += std::hypot(dx, dy);
                    cs.path_length  = path_length_;
                    claimset.claim_state_vector.push_back(cs);
                }

                if (v0 <= this->vel_lowest_range)
                {
                    claimset.time_reaction = 0.0f;
                }

                claimset.time_danger  = time;
                claimset.time_warning = T_warning;
            }
        }
        
        else
        {
            const float v0_kmh = v0 * 3.6f; // predict needs km/h
            
            float t          = 0.0f;
            float v_prev     = v0;
            float a_prev     = 0.0f;
            if (this->vel_lowest_range <= v0 && v0 <= this->vel_low_range)
            {
                a_prev = this->predictEgoDecelerationLowVel(v0_kmh, a0, t, T_danger, is_phase_12_end);
            }
            else
            {
                a_prev = this->predictEgoDecelerationHighVel(v0_kmh, a0, t, T_danger, is_phase_12_end); 
            }
            float v_cur;
            float a_cur      = 0.0f;
            float ds_cur     = 0;
            float s_accum    = 0.0f; 
            int k = 0.f;             
            // const float kappa = curvature_fixed;  

            int stepSizeDanger = std::min(static_cast<int>(std::floor(T_danger / dt)) + 1, 200);
            claimset.claim_state_vector.reserve(stepSizeDanger);
            T_reaction         = 0.0f;
            // for (int k = 0; k < stepSizeDanger; ++k)
            while (true)
            {
                if (k == 0)
                {
                    a_cur = a_prev;
                    v_cur = v_prev;
                    ds_cur = 0.0f;
                }
                else
                {
                    if (this->vel_lowest_range <= v0 && v0 <= this->vel_low_range)
                    {
                        a_cur = this->predictEgoDecelerationLowVel(v0_kmh, a0, t, T_danger, is_phase_12_end);
                    }
                    else
                    {
                        a_cur = this->predictEgoDecelerationHighVel(v0_kmh, a0, t, T_danger, is_phase_12_end);
                    }
                    v_cur = std::max(0.0f, v_prev + a_cur * dt);
                    if (v_cur <= 1e-4f) 
                    {
                        break;
                    } 
                    ds_cur =  0.5f * (v_cur + v_prev) * dt;
                }
                s_accum        += ds_cur;
                
                float yaw_rate = v_cur * curvature_fixed;
                float dx = ds_cur * std::cos(yaw); 
                float dy = ds_cur * std::sin(yaw);
                x  += dx;
                y  += dy;
                yaw += yaw_rate * dt;

                ClaimState cs;
                cs.t            = t;
                cs.pos          = cv::Point2f(x, y);
                cs.yaw          = yaw;
                cs.v            = v_cur;
                cs.a            = a_cur;
                cs.b            = curvature_fixed;
                cs.path_length  = s_accum;
                cs.shape_type   = SafetyForceField::ShapeType::OBB;
                cs.OBB.half_l   = VEHICLE_LENGTH_TOTAL / 2.0f;
                cs.OBB.half_w   = VEHICLE_WIDTH_TOTAL / 2.0f;
                cs.OBB.center   = cs.pos;
                cs.OBB.yaw      = yaw;
                claimset.claim_state_vector.emplace_back(cs);


                // ----------------------------------------

                if (is_phase_12_end && T_reaction == 0)
                {
                    if (!claimset.claim_state_vector.empty() && v0 > this->vel_lowest_range)
                    {
                        
                        ClaimState reaction_cs = claimset.claim_state_vector.back();
                        float last_time = reaction_cs.t;
                        // log.i(VV"T_danger: ", reaction_cs.t);
                        bool reaction_create = this->createOffsetZone(v0, last_time, reaction_cs);
                        if (reaction_create)
                        {
                            claimset.claim_state_vector.emplace_back(reaction_cs);
                            T_reaction = reaction_cs.t;
                        }
                        t += 0.01; // hard code for reaction time
                        // log.i(VV"T_reaction: ", T_reaction);
                        float braking_distance_temp = reaction_cs.OBB.half_l*2.0f;
                        float yaw = reaction_cs.yaw;
                        s_accum += braking_distance_temp;
                        x += braking_distance_temp * std::cos(yaw);
                        y += braking_distance_temp * std::sin(yaw);
                    }
                }
                //------------------------------------------
                // Update variable
                v_prev = v_cur;
                t     += dt;

                k += 1;
                if (k > 3e2) 
                {
                    // log.i(VV"v0: ", v0*3.6f);
                    // log.i(VV"infinite loop");
                    break;
                }
            }

            if (T_reaction == 0)
            {
                if (!claimset.claim_state_vector.empty() && v0 > this->vel_lowest_range)
                {
                    
                    ClaimState reaction_cs = claimset.claim_state_vector.back();
                    float last_time = reaction_cs.t;
                    // log.i(VV"T_danger: ", reaction_cs.t);
                    bool reaction_create = this->createOffsetZone(v0, last_time, reaction_cs);
                    if (reaction_create)
                    {
                        claimset.claim_state_vector.emplace_back(reaction_cs);
                        // T_reaction = reaction_cs.t;
                    }
                    // t += 0.01; // hard code for reaction time
                    // log.i(VV"T_reaction: ", T_reaction);
                    // s_accum += reaction_cs.OBB.half_l*2.0f;
                    float braking_distance_temp = reaction_cs.OBB.half_l*2.0f;
                    float yaw = reaction_cs.yaw;
                    s_accum += braking_distance_temp;
                    x += braking_distance_temp * std::cos(yaw);
                    y += braking_distance_temp * std::sin(yaw);
                }
            }
            // Phase 2: Warning
            v = v0;
            if (!claimset.claim_state_vector.empty())
            {
                T_danger = claimset.claim_state_vector.back().t;
                path_length_ = claimset.claim_state_vector.back().path_length;
            }
            else
            {
                T_danger = 0.0f;
                path_length_ = 0.f;
            }
            float s_danger = path_length_;

            int stepSizeWarning = static_cast<int>(std::floor(T_warning / dt)) + 1;
            v_prev = v;
            a_cur  = a0;
            for (int k = 1; k < stepSizeWarning; ++k) 
            {
                v = std::max(0.0f, v_prev + a_cur*dt);
                ds_cur =  0.5f * (v + v_prev) * dt;
                float t         = T_danger + k * dt;
                float curvature = curvature_fixed;

                // integrate
                float yaw_rate  = v * curvature;
                float dx        = ds_cur*std::cos(yaw);
                float dy        = ds_cur * std::sin(yaw);
                x               += dx; 
                y               += dy;
                yaw             += yaw_rate * dt;
                path_length_    += ds_cur;

                ClaimState cs;
                cs.t            = t;
                cs.pos          = cv::Point2f(x, y);
                cs.yaw          = yaw;
                cs.v            = v;
                cs.a            = a_cur;
                cs.b            = curvature;
                cs.shape_type   = SafetyForceField::ShapeType::OBB;
                cs.OBB.half_l   = VEHICLE_LENGTH_TOTAL / 2.0f;
                cs.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
                cs.OBB.center   = cs.pos;
                cs.OBB.yaw      = yaw;
                cs.path_length  = path_length_;
                
                v_prev  = v;
                
                claimset.claim_state_vector.push_back(cs);
                
            }

            claimset.time_danger  = T_danger;
            claimset.time_reaction = 0.0f;
            claimset.time_warning = T_warning;
            // T_reaction = 0.0f;
            // int stepSizeWarning = static_cast<int>(std::floor(T_warning / dt)) + 1;
            // v_prev = v;
            // a_cur  = a0;
            // for (int k = 1; k < stepSizeWarning; ++k) 
            // {
            //     v = std::max(0.0f, v_prev + a_cur*dt);
            //     ds_cur =  0.5f * (v_cur + v_prev) * dt;
            //     float t         = T_danger + k * dt;
            //     float curvature = curvature_fixed;

            //     // integrate
            //     float yaw_rate  = v * curvature;
            //     float dx        = ds_cur*std::cos(yaw);
            //     float dy        = ds_cur * std::sin(yaw);
            //     x               += dx; 
            //     y               += dy;
            //     yaw             += yaw_rate * dt;
            //     path_length_    += ds_cur;

            //     ClaimState cs;
            //     cs.t            = t;
            //     cs.pos          = cv::Point2f(x, y);
            //     cs.yaw          = yaw;
            //     cs.v            = v;
            //     cs.a            = a_cur;
            //     cs.b            = curvature;
            //     cs.shape_type   = SafetyForceField::ShapeType::OBB;
            //     cs.OBB.half_l   = VEHICLE_LENGTH_TOTAL / 2.0f;
            //     cs.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
            //     cs.OBB.center   = cs.pos;
            //     cs.OBB.yaw      = yaw;
            //     cs.path_length  = path_length_;
                
            //     v_prev  = v;

            //     bool get_reaction_time_result = this->getTimeReactionForOffset(v0, s_danger, cs.path_length, t, T_reaction);
            //     claimset.claim_state_vector.push_back(cs);
            //     if (k == stepSizeWarning-1 && v0 > this->vel_lowest_range && T_reaction == 0)
            //     {
            //         // Case a < 0 -> s_Warning < s_reaction
            //         int count = 0;
            //         while (v > 2e-1)
            //         {
            //             v = std::max(0.0f, v_prev + a_cur*dt);
            //             ds_cur =  0.5f * (v_cur + v_prev) * dt;
            //             float t         = T_danger + k * dt;
            //             float curvature = curvature_fixed;

            //             // integrate
            //             float yaw_rate  = v * curvature;
            //             float dx        = ds_cur*std::cos(yaw);
            //             float dy        = ds_cur * std::sin(yaw);
            //             x               += dx; 
            //             y               += dy;
            //             yaw             += yaw_rate * dt;
            //             path_length_    += ds_cur;

            //             ClaimState cs;
            //             cs.t            = t;
            //             cs.pos          = cv::Point2f(x, y);
            //             cs.yaw          = yaw;
            //             cs.v            = v;
            //             cs.a            = a_cur;
            //             cs.b            = curvature;
            //             cs.shape_type   = SafetyForceField::ShapeType::OBB;
            //             cs.OBB.half_l   = VEHICLE_LENGTH_TOTAL / 2.0f;
            //             cs.OBB.half_w   = this->getWidthCar(SafetyForceField::ObjectType::EGO) / 2.0f;
            //             cs.OBB.center   = cs.pos;
            //             cs.OBB.yaw      = yaw;
            //             cs.path_length  = path_length_;
                        
            //             v_prev  = v;
            //             get_reaction_time_result = this->getTimeReactionForOffset(v0, s_danger, cs.path_length, t, T_reaction);
            //             claimset.claim_state_vector.push_back(cs);
            //             if (get_reaction_time_result)
            //             {
            //                 T_warning = 0.0f;
            //                 break;
            //             }
            //             count++;
            //             if(count > 2e2)
            //             {
            //                 log.i(VV"Infinite loop");
            //                 break;
            //             }
            //         } 
            //         // Check get_reaction_time_result true or not
            //         // false -> v -> 0 before get t_reaction
            //         if (!get_reaction_time_result) 
            //         {
            //             get_reaction_time_result = true;
            //             T_warning = 0.0f;
            //             T_reaction = claimset.claim_state_vector.back().t;
            //         }
            //     }
            
            // }
            // // claimset.time_reaction = T_reaction; //T_reaction;
            // claimset.time_danger  = T_danger;
            // if (T_reaction > T_danger)
            // {
            //     claimset.time_reaction = T_reaction - T_danger;
            // }
            // else
            // {
            //     claimset.time_reaction = T_reaction;
            // }
            // claimset.time_warning = T_warning;

        }
    }

    return claimset;
}

void SafetyForceField::predictEgoClaimSet(float v0, float a0, const PerceptionOutputObject& perception_output,
                                        float dt, std::vector<ClaimSet>& ego_claimset, bool is_braking)
{
    const float eps = 1e-3f;
    this->sffSet_DangerCheck_.clear();

    float time_danger = this->predictEgoMaximumDeceleration(v0 * 3.6);
    float max_decel = -time_danger * v0;
    ClaimSet sffSet_state_;
    ClaimSet sffset_danger_;

    // If is braking => build claim set from safety distance instead of vel and accel
    if ((v0 < eps && a0 < eps) || is_braking)
    {
        v0 = 0.0f;
        a0 = 0.0f;
        time_danger = 2.0f; // get default time danger
        max_decel   = 0.0f;
    }

    float steering_angle            = perception_output.steering_angle.steering;

    // Generate Claimset for Danger stop from current vehicle state
    sffset_danger_                  = this->buildClaimestFromVehicleState(v0, a0, steering_angle,
                                        time_danger, sffset_danger_.time_reaction,
                                        sffset_danger_.time_warning, dt, true, max_decel);

    sffSet_state_.state_type        = SafetyForceField::ObjectStateType::DYNAMIC;
    sffset_danger_.state_type       = SafetyForceField::ObjectStateType::DYNAMIC;

    this->sffSet_state_             = sffSet_state_;
    this->sffSet_danger_            = sffset_danger_;

    this->sffSet_DangerCheck_.push_back(sffset_danger_);

    ego_claimset.push_back(sffSet_state_);
    ego_claimset.push_back(sffset_danger_);
}

SafetyForceField::BoxEstimate SafetyForceField::estimateBoxCar(const cv::Point2f& closest, const cv::Point2f& position, const cv::Point2f& obj_velocity, 
                                                                float half_width, float half_length, const cv::Point2f& ego_pos, Eigen::Vector4f bounding_box)
{
    float vel_x = obj_velocity.x;
    float vel_y = obj_velocity.y;

    // Filter for noise velocity
    if (std::fabs(vel_x) < 1.0f)
        vel_x = 0.0f;
    if (std::fabs(vel_y) < 0.8f)
        vel_y = 0.0f;
    // Calculate Orientation of Opposite car
    float speed = std::sqrt(vel_x * vel_x + vel_y * vel_y);
    float object_angle;
    if (speed < 0.3f) {
        // Remain PI/2 if car stays unchanebly
        object_angle = static_cast<float>(M_PI_2);
    } else {
        object_angle = std::atan2(vel_y, vel_x);
    }

    // Vector from Opposite car to ego car
    cv::Point2f v_closest_to_ego = ego_pos - closest;
    float vector_angle = std::atan2(v_closest_to_ego.y, v_closest_to_ego.x);

    float angle_diff = vector_angle - object_angle;
    while (angle_diff > M_PI)  angle_diff -= 2.0f * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;

    float angle_deg = angle_diff * 180.0f / M_PI;
    float abs_ang = std::fabs(angle_deg);

    // Threshold to determine the closest point correspond to car
    const float REAR_MIN = 70.0f;
    const float REAR_MAX = 110.0f;
    const float CORNER_MIN = 40.0f;
    const float CORNER_MAX = 140.0f;
    const float REAR_C1 = 0.0f;
    const float REAR_C2 = 180.0f;
    const float TOL_REAR = 10.0f;
    // Classify surface
    enum SurfaceType { REAR, SIDE_LEFT, SIDE_RIGHT, REAR_LEFT, REAR_RIGHT };
    SurfaceType surf = REAR;
    cv::Point2f offset_local;
    auto near = [](float a, float b, float tol) -> bool { return std::fabs(a - b) <= tol; };
    if (near(abs_ang, REAR_C1, TOL_REAR) || near(abs_ang, REAR_C2, TOL_REAR)) 
    {
        // Rear
        surf = REAR;
        offset_local = cv::Point2f(-half_length, 0.0f);
    }
    else if ((abs_ang > CORNER_MIN && abs_ang < REAR_MIN) ||
            (abs_ang > REAR_MAX && abs_ang < CORNER_MAX)) 
    {
        // Rear left/right corner
        if (angle_diff > 0) 
        {
            surf = REAR_LEFT;
            offset_local = cv::Point2f(-half_length, +half_width);
        } 
        else 
        {
            surf = REAR_RIGHT;
            offset_local = cv::Point2f(-half_length, -half_width);
        }
    }
    else {
        // side
        if (angle_diff > 0) 
        {
            surf = SIDE_LEFT;
            offset_local = cv::Point2f(0.0f, +half_width);
        } 
        else 
        {
            surf = SIDE_RIGHT;
            offset_local = cv::Point2f(0.0f, -half_width);
        }
    }

    // Calculate Offset
    float ca = std::cos(object_angle);
    float sa = std::sin(object_angle);
    cv::Point2f offset_world(ca * offset_local.x - sa * offset_local.y,
                            sa * offset_local.x + ca * offset_local.y);

    // Calculate Center 
    cv::Point2f center = closest - offset_world;

    BoxEstimate result;
    result.center = center;
    result.width  = 2 * half_width;
    result.length = 2 * half_length;
    result.yaw    = object_angle;
    return result;
}

SafetyForceField::BoxEstimate SafetyForceField::estimateBox(
                const cv::Point2f& closest, const cv::Point2f& position, float half_width,
                float half_length, const cv::Point2f& ego_pos, const OutputObject& object)
{
    const float eps = 1e-6f;
    const float eps_x = VEHICLE_WIDTH_TOTAL/2.0f; // 1.5f;//;
    const float eps_dx = 1.2f;
    const float x_diff_car = 0.3f;
    BoxEstimate result;
    cv::Point2f center_box;
    cv::Point2f mid_edge_pos;

    bool in_front_of_static_obj = false;

    cv::Point2f edge_vector = position - closest;
    float dx                = edge_vector.x;
    float dy                = edge_vector.y;
    float edge_length       = cv::norm(edge_vector);

    if (std::abs(edge_length) < eps)
    {
        // position point and closet point at the same point
        cv::Point2f ego_to_closest_vector = position - ego_pos;
        float norm_dir                    = cv::norm(ego_to_closest_vector);

        edge_length = half_length;

        if (norm_dir > eps)
        {
            cv::Point2f ego_dir_unit      = ego_to_closest_vector / norm_dir;
            // unit direction vector from edge of undefined object
            edge_vector                   = cv::Point2f(ego_dir_unit.y, -ego_dir_unit.x);
        }
        else
        {
            edge_vector = cv::Point2f(1.0f, 0.f); // unit vector
        }
        if (std::find(mindfull_object_IDs.begin(), mindfull_object_IDs.end(), object.classID) != mindfull_object_IDs.end())
        {
            mid_edge_pos = (position + closest) / 2;
            half_width   = std::abs(position.y - closest.y) / 2;
        }
        else
        {
            // get the mid point of closet
            mid_edge_pos = position;
            in_front_of_static_obj = true;
            half_width   = std::abs(position.y - closest.y);
        }
    }
    else if (std::abs(dx) < eps_dx && std::abs(position.x) < eps_x && std::find(mindfull_object_IDs.begin(), mindfull_object_IDs.end(), object.classID)
            != mindfull_object_IDs.end()) // 
    {   
        // object in front of car and in width range of car
        // redefine unit direction edge vector
        cv::Point2f ego_to_closest_vector = position - ego_pos;
        float norm_dir                    = cv::norm(ego_to_closest_vector);
        if (norm_dir > eps)
        {
            cv::Point2f ego_dir_unit      = ego_to_closest_vector / norm_dir;
            // unit direction vector from edge of undefined object
            edge_vector                   = cv::Point2f(ego_dir_unit.y, -ego_dir_unit.x);
        }
        else
        {
            edge_vector = cv::Point2f(1.0f, 0.f); // unit vector
        }
        edge_length = std::round((object.bbox(2) - object.bbox(0)))/100/2.0f; // convert to m with 2 precision
        // half_width  = std::abs(dy);
        // get the mid point of closet
        mid_edge_pos = (position + closest) / 2;
        in_front_of_static_obj = true;
        // edge_length = VEHICLE_WIDTH_TOTAL/2.0f;
        half_width   = std::abs(position.y - closest.y) / 2;
    }
    else
    {
        // get the mid point of closet and location point
        mid_edge_pos = position;
    }

    float yaw = std::atan2(edge_vector.y, edge_vector.x);
    // Determine unit direction vector and normal vector 
    cv::Point2f direciton_unit_vector(std::cos(yaw), std::sin(yaw));
    cv::Point2f right_normal_ego_unit_vector(std::sin(yaw), -std::cos(yaw));

    // get the vector from ego position to mid point position
    cv::Point2f mid_ego_vector = mid_edge_pos - ego_pos;
    /*
    Determine direction between (mid_edge_pos - ego) vector and right vector
        If dot > 0 => two vector at the same direction
        if dot < 0 => opposite
    */ 
    float direction_2_dot = mid_ego_vector.x*right_normal_ego_unit_vector.x + mid_ego_vector.y*right_normal_ego_unit_vector.y;

    if (!in_front_of_static_obj)
    {
        if (direction_2_dot >= 0)
        {
            // at the same direction with right normal vector => point to the right => center at the right of mid_edge_pos
            center_box = (mid_edge_pos + half_width*right_normal_ego_unit_vector);
        }
        else
        {
            center_box = (mid_edge_pos - half_width*right_normal_ego_unit_vector);
        }
    }
    else
    {
        center_box = mid_edge_pos;
    }
    if (!isfinite(edge_length))
        edge_length = 0;
    result.center = center_box;
    result.width  = 2*half_width;
    result.length = 2*edge_length;
    result.yaw    = yaw;
    return result;
}

void SafetyForceField::predictObjectClaimSet(const OutputObject& object, 
                                            float total_time, float dt,
                                            ClaimSet& claimSet_, const cv::Point2f& ego_pos)
{
    const float eps=1e-6f;
    // Initial state
    cv::Point2f closest_obj_point(object.closest_point.x(), object.closest_point.y());
    cv::Point2f location_obj_point(object.location.x(), object.location.y());
    // log.i("location: (", object.location.x(), ";", object.location.y(), ") ; ", 
    //         "closest: (", object.closest_point.x(), ";", object.closest_point.y(), ")");
    float yaw0          = SafetyForceField::normalizeAngleToPi(SafetyForceField::roundNum(90*M_PI/180));
    float input_vel_y   = object.velocity.y(); 
    float input_vel_x   = object.velocity.x();
    float input_accel_y = object.acceleration.y(); 
    float input_accel_x = object.acceleration.x();

    // Boundary for noise of velocity and acceleration
    float bound_vel_y   = 1.5;  // m/s 
    float bound_accel_y = 1;    // m/s^2
    float bound_vel_x   = 1.0;  // m/s 
    float bound_accel_x = 1;    // m/s^2

    float bound_vel_x_car = 1.0;
    float bound_vel_y_car = 1.0;

    // Filter to eliminate noise velocity and acceleration of static object car
    if (input_vel_x < bound_vel_x_car && input_vel_y < bound_vel_y_car && object.classID == 2)
    {
        input_vel_y = 0;
        input_vel_x = 0;
        input_accel_x = 0;
        input_accel_y = 0;
    }

    // Filter for low range velocity and static objects
    if (input_vel_y < 6 && object.classID == -1)
    {
        if (input_vel_y<bound_vel_y)
        {
            input_vel_y = 0;
        }
        if (input_vel_y <= 0 ||  input_accel_y < bound_accel_y)
        {
            input_accel_y = 0;
        }
        if (input_vel_x>-bound_vel_x && input_vel_x<bound_vel_x)
        {
            input_vel_x = 0;
        }
        if (input_accel_x>-bound_accel_x && input_accel_x < bound_accel_x)
        {
            input_accel_x = 0;
        }
    }

    int stepSize = (int)std::floor(total_time / dt) + 1;
    claimSet_.claim_state_vector.reserve(std::max(1, stepSize));
    cv::Point2f car_center(0,0);
    if (std::find(mindfull_object_IDs.begin(), mindfull_object_IDs.end(), object.classID) != mindfull_object_IDs.end()) 
    {
        // condition for detecting undefined car: (sff_vel_x > 1.5 && sff_vel_y > 1.5)
        Eigen::Vector4f bounding_box = object.bbox;
        cv::Point2f obj_velocity(object.velocity.x(), object.velocity.y());
        SafetyForceField::BoxEstimate box = estimateBoxCar(closest_obj_point, location_obj_point, obj_velocity, VEHICLE_WIDTH_TOTAL/2,
                                            VEHICLE_LENGTH_TOTAL/2, ego_pos, bounding_box);
        car_center = box.center;
    }
    float sff_vel_y_prev = input_vel_y;
    float sff_vel_x_prev = input_vel_x;
    float ds_cur_y = 0.0f;
    float ds_cur_x = 0.0f;
    float s_cur_y  = 0.0f;
    float s_cur_x  = 0.0f;
    float s_prev   = 0.0f;

    for (int k = 0; k < stepSize; ++k) 
    {
        float time = k * dt;
        float sff_vel_y;
        float sff_vel_x;
        if (input_vel_y <= 0)
        { 
            sff_vel_y = 0;
            input_vel_y = 0;
            // is Static only if sff_vel_y == 0 at the first time
            if (k == 0)
            {
                claimSet_.state_type = SafetyForceField::ObjectStateType::STATIC;
            }
            else
            {
                claimSet_.state_type = SafetyForceField::ObjectStateType::DYNAMIC;
            }
        }
        else
        {
            sff_vel_y = sff_vel_y_prev + input_accel_y*dt;
            claimSet_.state_type = SafetyForceField::ObjectStateType::DYNAMIC;
        }
        sff_vel_x = sff_vel_x_prev + input_accel_x*dt;

        // Restrict bound of velocity
        if ( sff_vel_y < 0 ) 
        {
            sff_vel_y = 0;
        }
        
        ds_cur_y = 0.5f*(sff_vel_y_prev + sff_vel_y)*dt;
        ds_cur_x = 0.5f*(sff_vel_x_prev + sff_vel_x)*dt;

        s_cur_y  += ds_cur_y;
        s_cur_x  += ds_cur_x;
        float sff_distance = std::sqrt(s_cur_y * s_cur_y +
                                s_cur_x * s_cur_x);
        
        if (sff_distance < s_prev)
        {
            sff_distance = s_prev;
        }

        s_prev = sff_distance;
        sff_vel_y_prev = sff_vel_y;
        sff_vel_x_prev = sff_vel_x;

        if (sff_vel_y != 0 || sff_vel_x != 0)
        {
            if (sff_vel_x == 0) 
            {
                sff_vel_x = eps;
                if (sff_vel_y > 0)
                {
                    yaw0 = SafetyForceField::roundNum(M_PI/2.0f);
                }
                else
                {
                    yaw0 = -SafetyForceField::roundNum(M_PI/2.0f);
                }
            }
            else
            {
                yaw0 = std::atan2(sff_vel_y,sff_vel_x);
            }
            
        }
        yaw0 = SafetyForceField::normalizeAngleToPi(yaw0);

        cv::Point2f pos(closest_obj_point.x + s_cur_x, closest_obj_point.y + s_cur_y);
        if (std::find(mindfull_object_IDs.begin(), mindfull_object_IDs.end(), object.classID) 
            != mindfull_object_IDs.end() && claimSet_.state_type == SafetyForceField::ObjectStateType::DYNAMIC && (sff_vel_x > 1.5 || sff_vel_y > 1.5)) 
        {
            pos = cv::Point2f(car_center.x + s_cur_x, car_center.y + s_cur_y);       
        }

        ClaimState claimState_(
            pos, yaw0, time, sff_distance, sff_vel_y, input_accel_y, 0.0f,
            SafetyForceField::ShapeType::OBB,
            4.0f, UNDEFINED_LENGTH / 2, UNDEFINED_WIDTH / 2
        );

        if ((std::find(mindfull_object_IDs.begin(), mindfull_object_IDs.end(), object.classID) 
            != mindfull_object_IDs.end() || claimSet_.state_type == SafetyForceField::ObjectStateType::DYNAMIC) && (sff_vel_x > 1.5 || sff_vel_y > 1.5))  //  && (sff_vel_x > 1.5 || sff_vel_y > 1.5)
        {
            // condition for detecting undefined car: (sff_vel_x > 1.5 && sff_vel_y > 1.5)
            claimState_.shape_type = SafetyForceField::ShapeType::OBB;
            cv::Point2f dir_long(std::cos(yaw0), std::sin(yaw0));

            claimState_.pos        = pos;
            claimState_.yaw        = yaw0;
            claimState_.OBB.center = pos;
            claimState_.OBB.half_l = VEHICLE_LENGTH_TOTAL * 0.5f;
            claimState_.OBB.half_w = this->getWidthCar(SafetyForceField::ObjectType::OBJECT) * 0.5f;
            claimState_.OBB.yaw    = yaw0;
        }

        // Dertermine for undefined and static object
        if (input_vel_x <= 1.5 && input_vel_y <= 1.5 && claimSet_.state_type == SafetyForceField::ObjectStateType::STATIC)
        {            
            // Eigen::Vector4f bounding_box = object.bbox;
            claimSet_.state_type = SafetyForceField::ObjectStateType::STATIC;
            SafetyForceField::BoxEstimate box = estimateBox(
                closest_obj_point, location_obj_point, UNDEFINED_WIDTH/2,
                UNDEFINED_LENGTH/2, ego_pos, object);
            
            claimState_.shape_type  = SafetyForceField::ShapeType::OBB;
            claimState_.OBB.center  = box.center;
            claimState_.OBB.half_l  = box.length / 2;
            claimState_.OBB.half_w  = box.width / 2;
            claimState_.yaw         = box.yaw;
            claimState_.OBB.yaw     = box.yaw;
            claimState_.pos         = box.center;
            claimState_.v           = 0.0f;
            claimState_.a           = 0.0f;
            claimState_.path_length = 1.0f;
            claimState_.t           = 0.0f;
            
        }
        
        if (k != 0)
        {
            claimState_.circle.radius = 3.0f;
            claimSet_.claim_state_vector.emplace_back(claimState_);
        }
        else
        {
            // override the first default value
            claimState_.circle.radius = 3.0f;
            claimSet_.claim_state_vector[0] = claimState_;
        }
        
        if (k==0 && claimSet_.state_type == SafetyForceField::ObjectStateType::STATIC)
        {
            break;
        }
    }
}

void SafetyForceField::predictObjectSetClaimSet(const PerceptionOutputObject& perception_output, 
                                                float total_time, float dt, std::vector<ClaimSet>& objects_claimset,
                                                const cv::Point2f& ego_pos)
{
    objects_claimset.reserve(perception_output.objects.size());

    for (const std::shared_ptr<OutputObject>& object : perception_output.objects)
    {
        ClaimSet claimSet_;
        this->predictObjectClaimSet(*object, total_time, dt, claimSet_, ego_pos);
        objects_claimset.push_back(claimSet_);
    }
}

SafetyForceField::SFFDecision SafetyForceField::decideLevelFromTime(float t_hit_ego, float time_danger_reaction, float time_warning) 
{
    if (t_hit_ego <= time_danger_reaction) 
    {
        return SafetyForceField::SFFDecision::DANGER;
    }
    if (t_hit_ego <= time_warning) 
    {
        return SafetyForceField::SFFDecision::WARNING;
    }
    return SafetyForceField::SFFDecision::SAFE;
}

SafetyForceField::OverlapResult SafetyForceField::firstOverlapWithObject(
    const ClaimSet& ego, const ClaimSet& obj)
{
    OverlapResult out;
    const size_t ego_size    = ego.claim_state_vector.size();
    const size_t object_size = obj.claim_state_vector.size();
    if (ego_size == 0 || object_size == 0) 
    {
        return out;
    }
    // check overlap between ego (In low velocity range and dynamic object)
    if (ego.in_low_vel_range)
    {
        float total_time_ego = ego.claim_state_vector.back().t;
        ClaimState ego_state = ego.claim_state_vector[1]; // Have three point for danger sff => get middle point in (0, 1, 2)
        ego_state.OBB.half_l = ego.distance_brake_threshold/2;
        if (obj.state_type == SafetyForceField::ObjectStateType::DYNAMIC)
        {  
            for(size_t obj_index = 0; obj_index < object_size; ++obj_index)
            {   
                ClaimState object_state = obj.claim_state_vector[obj_index];
                if (object_state.t < total_time_ego)
                {
                    if (SafetyForceField::statesIntersect(ego_state, object_state))
                    {
                        out.hit              = true;
                        out.ego_sample_index = 1;
                        out.obj_sample_index = obj_index;
                        out.t_hit_ego            = ego.claim_state_vector[1].t;
                        out.level            = SafetyForceField::SFFDecision::DANGER;
                        return out;
                    }
                }
            }
        }
        else
        {   
            for (size_t ego_index = 0; ego_index < ego_size; ++ego_index)
            {
                ego_state            = ego.claim_state_vector[ego_index];
                ego_state.OBB.half_l = ego.distance_brake_threshold/2;
                if (SafetyForceField::statesIntersect(ego_state, obj.claim_state_vector.back()))
                {
                    out.hit              = true;
                    out.ego_sample_index = 1;
                    out.obj_sample_index = 0;
                    out.t_hit_ego            = ego.claim_state_vector[ego_index].t;
                    out.level            = SafetyForceField::SFFDecision::DANGER;
                    return out;
                }
            }
            
        }
        return out;
    }
    for (size_t ego_index = 0; ego_index < ego_size; ++ego_index) 
    {
        float offset_from_sff_bound = 3.0f; //m 
        size_t object_index;
        ClaimState ego_state = ego.claim_state_vector[ego_index];
        ClaimState object_state;
        if (obj.state_type == SafetyForceField::ObjectStateType::STATIC)
        {
            object_index = 0;
            object_state = obj.claim_state_vector.back();
            ClaimState last_ego_state = ego.claim_state_vector.back();
            // check static object in range of sff of ego
            float ego_obj_dx = object_state.OBB.center.x - last_ego_state.OBB.center.x;
            float ego_obj_dy = object_state.OBB.center.y - last_ego_state.OBB.center.y;
            float dis_obj_ego = std::hypot(ego_obj_dx, ego_obj_dy);
            
            if (dis_obj_ego - object_state.OBB.half_w > last_ego_state.path_length + VEHICLE_LENGTH_TOTAL/2.0f + offset_from_sff_bound)
            {
                return out;
            }
        }
        else
        {
            if (ego_index >= object_size)
            {
                // out of object range
                return out;
            }
            object_state = obj.claim_state_vector[ego_index]; 
            object_index = ego_index;
        }

        if (SafetyForceField::statesIntersect(ego_state, object_state)) 
        {
            out.hit              = true;
            out.ego_sample_index = ego_index;
            out.obj_sample_index = object_index;
            out.ego_in_stop_state= ego.in_stop_state;
            out.t_hit_ego        = ego_state.t;
            out.t_hit_obj        = object_state.t;
            out.level            = SafetyForceField::decideLevelFromTime(out.t_hit_ego, 
                                            ego.time_danger + ego.time_reaction,
                                            ego.time_danger + ego.time_warning);
            return out; 
        }
    }
    return out;
}

SafetyForceField::SFFDecision SafetyForceField::checkOverlapsAndDecide()
{
    OverlapResult firstly_overlap;
    SFFDecision overall = SFFDecision::SAFE;
    int size_obj        = this->sffObjSet_.size();
    int size_ego        = this->sffSet_DangerCheck_.size();
    if (this->sffSet_DangerCheck_.empty() || this->sffSet_DangerCheck_[0].claim_state_vector.empty() || size_obj == 0) 
    {    
        this->sffOverlap = firstly_overlap;
        return overall;
    }
    try
    {
        for (int j = 0; j < size_ego; j++)
        {
            for (int i = 0; i < size_obj; ++i) // this->sffObjSet_.size() 
            {
                const ClaimSet& obj      = this->sffObjSet_[i];
                OverlapResult is_overlap = SafetyForceField::firstOverlapWithObject(this->sffSet_DangerCheck_[j], obj);
                if (!is_overlap.hit) 
                {
                    continue;
                }

                is_overlap.ego_overlap = this->sffSet_DangerCheck_[j].claim_state_vector[is_overlap.ego_sample_index];
                is_overlap.obj_overlap = obj.claim_state_vector[is_overlap.obj_sample_index];

                if (!firstly_overlap.hit || is_overlap.t_hit_ego < firstly_overlap.t_hit_ego) 
                {
                    firstly_overlap = is_overlap;
                }

                if (is_overlap.level == SafetyForceField::SFFDecision::DANGER) 
                {
                    overall = SafetyForceField::SFFDecision::DANGER;
                    break;
                } 
                else if (is_overlap.level == SafetyForceField::SFFDecision::WARNING) 
                {
                    if (overall != SafetyForceField::SFFDecision::DANGER) 
                    {
                        overall = SafetyForceField::SFFDecision::WARNING;
                    }
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error: Overlap" << e.what() << '\n';
    }

    this->sffOverlap = firstly_overlap;
    
    return overall;
}
