#ifndef SAFETY_FORCE_FIELD_H
#define SAFETY_FORCE_FIELD_H

#include <array>
#include <vector>
#include "intermediate_representations.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>    
#include <cmath>
#include <algorithm>
#include <chrono> 
#include "logger/general_logger.h"
#include "common.h"

// Ego Car Dimension Define
#define VEHICLE_LENGTH_BASE                 3.0
#define VEHICLE_WIDTH_BASE                  1.67
#define VEHICLE_LENGTH_TOTAL                4.79
#define VEHICLE_WIDTH_TOTAL                 2.16
#define UNDEFINED_LENGTH                    0.5
#define UNDEFINED_WIDTH                     0.5
#define OFFSET_WIDTH_CAR_OBJECT             0.2
#define NUMBER_POINT_DANGER_LOW_VEL_RANGE   3

class SafetyForceField 
{
    private:
        const float vel_in_low_threshold_state_1      = 35 / 3.6; // m/s
        const float vel_in_low_threshold_state_2      = 10 / 3.6; // m/s
        const float accel_in_low_threshold_state_noAcc=0.3; // m/s^2
        const float accel_in_low_threshold_state_1    = 4; // m/s^2
        const float accel_in_low_threshold_state_2    = 7; // m/s^2
        const float distance_low_threshold_state_stop = 1; // m
        const float distance_low_threshold_state_noAcc= 2; // m
        const float distance_low_threshold_state_1    = 2; // m
        const float distance_low_threshold_state_2    = 2; // m

        // Constant Interpolation function get the maximum deceleration in velocity range 0-30 [km/h]
        float max_dec_const_low = -1.5f;
        float maxL_dec_a_coeff = -0.236f;
        float maxL_dec_b_coeff = -1.499;

        const float a0_param_A_timeBraking = -0.00002f;
        const float a0_param_B_timeBraking = 0.0218f;
        const float a0_param_C_timeBraking = -0.2126f;
        float a_param_acc_phase0 = 0.0f;
        
        // Phase 0 - Low Velocity
        // Low acceleration
        const float a02_param_A_timeBraking_low = 0.0f;
        const float a02_param_B_timeBraking_low = 0.0224f;
        const float a02_param_C_timeBraking_low = 0.0f;
        // -- Parameter Offset to complement the error of model
        const float a02_param_error_offset_low = 0.1f;
        // Medium acceleration
        const float a24_param_A_timeBraking_low = 0.00003f;
        const float a24_param_B_timeBraking_low = 0.0219f;
        const float a24_param_C_timeBraking_low = 0.0f;
        // -- Parameter Offset to complement the error of model
        const float a24_param_error_offset_low = 0.1f;
        // High acceleration
        const float a46_param_A_timeBraking_low = 0.00005f;
        const float a46_param_B_timeBraking_low = 0.0224f;
        const float a46_param_C_timeBraking_low = 0.0f;
        // -- Parameter Offset to complement the error of model
        const float a46_param_error_offset_low = 0.1f;

        const float a68_param_A_timeBraking_low = 0.0002f;
        const float a68_param_B_timeBraking_low = 0.0199f;
        const float a68_param_C_timeBraking_low = 0.0411f;
        // -- Parameter Offset to complement the error of model
        const float a68_param_error_offset_low = 0.2f;

        // Phase 0 - High Velocity
        // Low acceleration
        const float a02_param_A_timeBraking_high = 0.0f;
        const float a02_param_B_timeBraking_high = 0.0205;
        const float a02_param_C_timeBraking_high = 0.0f;
        // -- Parameter Offset to complement the error of model
        const float a02_param_error_offset_high = 0.1f;
        // Medium acceleration
        const float a24_param_A_timeBraking_high = 0.000022f;
        const float a24_param_B_timeBraking_high = 0.0195f;
        const float a24_param_C_timeBraking_high = 0.0f;
        // -- Parameter Offset to complement the error of model
        const float a24_param_error_offset_high = 0.1f;
        // Medium acceleration
        const float a46_param_A_timeBraking_high = 0.000045f;
        const float a46_param_B_timeBraking_high = 0.01775f;
        const float a46_param_C_timeBraking_high = 0.0f;
        // -- Parameter Offset to complement the error of model
        const float a46_param_error_offset_high = 0.1f;
        // High acceleration
        // const float a68_param_A_timeBraking_high = 0.000049f;
        // const float a68_param_B_timeBraking_high = 0.0195f;
        // const float a68_param_C_timeBraking_high = 0.0f;
        const float a68_param_A_timeBraking_high = 0.000005f;
        const float a68_param_B_timeBraking_high = 0.0169f;
        const float a68_param_C_timeBraking_high = 0.1838f;
        // -- Parameter Offset to complement the error of model
        const float a68_param_error_offset_high = 0.2f;
        
        // Low velocity range
        // Phase 1
        const float a_param_A_jerk_phase_1_low = 1.232f;
        const float b_param_A_jerk_phase_1_low = -87.802f;
        const float c_param_A_jerk_phase_1_low = 1771.6f;
        const float d_param_A_jerk_phase_1_low = -10168.0f;

        const float a_param_B_jerk_phase_1_low = -0.8192f;
        const float b_param_B_jerk_phase_1_low = 63.122f;
        const float c_param_B_jerk_phase_1_low = -1380.3f;
        const float d_param_B_jerk_phase_1_low = 7190.3f;

        const float a_param_C_jerk_phase_1_low = -4.9189;
        const float b_param_C_jerk_phase_1_low = 103.61f;

        const float a_param_D_jerk_phase_1_low = 0.0723f;
        const float b_param_D_jerk_phase_1_low = -2.6664f;

        const float a_param_Time_jerk_phase_1_low = 0.0005f;
        const float b_param_Time_jerk_phase_1_low = -0.015f;
        const float c_param_Time_jerk_phase_1_low = 0.25f;
        
        // Phase 2
        const float a_param_A_jerk_phase_2_low = -3.4797f;
        const float b_param_A_jerk_phase_2_low = 280.42f;
        const float c_param_A_jerk_phase_2_low = -6498.6f;
        const float d_param_A_jerk_phase_2_low = 40425.0f;

        const float a_param_B_jerk_phase_2_low = 2.1805f;
        const float b_param_B_jerk_phase_2_low = -186.8f;
        const float c_param_B_jerk_phase_2_low = 4389.1f;
        const float d_param_B_jerk_phase_2_low = -27029.0f;

        const float a_param_C_jerk_phase_2_low = -0.2221f;
        const float b_param_C_jerk_phase_2_low = 28.654f;
        const float c_param_C_jerk_phase_2_low = -764.24f;
        const float d_param_C_jerk_phase_2_low = 4918.0f;

        const float a_param_D_jerk_phase_2_low = -0.0556f;
        const float b_param_D_jerk_phase_2_low = 1.9469f;
        const float c_param_D_jerk_phase_2_low = -17.997f;
        const float d_param_D_jerk_phase_2_low = 40.852f;

        const float a_param_Time_jerk_phase_2_low = -0.0003;
        const float b_param_Time_jerk_phase_2_low = 0.0195f;
        const float c_param_Time_jerk_phase_2_low = -0.125f;

        // High Velocity Range
        // Phase 1
        // Phase 1.1
        const float a_param_A_jerk_phase_11 = -1.861f;
        const float b_param_A_jerk_phase_11 = 257.9f;
        const float a_param_B_jerk_phase_11 = 0.5144f;
        const float b_param_B_jerk_phase_11 = -112.64f;
        const float a_param_C_jerk_phase_11 = 0.0077f;
        const float b_param_C_jerk_phase_11 = -0.7367f;
        
        const float a_param_Time_jerk_phase_11 = 0.0000000000000000009f;
        const float b_param_Time_jerk_phase_11 = 0.0067f;
        const float c_param_Time_jerk_phase_11 = -0.0333f;
        // Phase 1.2
        const float a_param_A_jerk_phase_12 = -0.0106f;
        const float b_param_A_jerk_phase_12 = 1.7977f;
        const float a_param_B_jerk_phase_12 = -0.0401f;
        const float b_param_B_jerk_phase_12 = -9.8237f;

        const float a_param_Time_jerk_phase_12 = -0.00005f;
        const float b_param_Time_jerk_phase_12 = 0.021;
        const float c_param_Time_jerk_phase_12 = -0.5952f;

        // Phase 1.3
        const float a_param_A_jerk_phase_13 = 0.0005f;
        const float b_param_A_jerk_phase_13 = -0.0929f;
        const float c_param_A_jerk_phase_13 = 5.0169f;
        const float d_param_A_jerk_phase_13 = -301.43f;
        const float a_param_B_jerk_phase_13 = -0.0033f;
        const float b_param_B_jerk_phase_13 = 0.6851f;
        const float c_param_B_jerk_phase_13 = -40.455f;
        const float d_param_B_jerk_phase_13 = 829.71f;

        const float a_param_Time_jerk_phase_13 = 0.00001f;
        const float b_param_Time_jerk_phase_13 = -0.0021f;
        const float c_param_Time_jerk_phase_13 = 0.1451f;
        // Phase 21

        const float a_param_A_jerk_phase_2 = -0.0261f;
        const float b_param_A_jerk_phase_2 = 4.3698f;
        const float c_param_A_jerk_phase_2 = -132.01f;

        const float a_param_B_jerk_phase_2 = 0.0326f;
        const float b_param_B_jerk_phase_2 = -6.1314f;
        const float c_param_B_jerk_phase_2 = 186.35f;


        const float a_param_Time_jerk_phase_2 = -0.000009f;
        const float b_param_Time_jerk_phase_2 = 0.0016f;
        const float c_param_Time_jerk_phase_2 = 0.1961f;

        const float vel_lowest_range = 10.f/3.6f;
        const float vel_low_range = 37.5f/3.6f;

        const float reaction_vel_lowest_range = 22.0f/3.6f;
        const float reaction_vel_low_range    = 37.0f/3.6f;
        const float reaction_vel_middle_range = 73.0f/3.6f;
        const float reaction_vel_high_range   = 92.0f/3.6f;
        
        const float reaction_distance_lowest_range  = 1.15f;//.175f;//2.395f; //m 0 -> vel_lowest
        const float reaction_distance_low_range     = 1.555f;//2.395f; //m vel_lowest -> vel_low
        const float reaction_distance_middle_range  = 2.255f; //m vel_low -> vel_middle
        const float reaction_distance_high_range    = 3.05f; // vel_middle -> vel_high
        const float reaction_distance_highest_range = 3.855f; // vel_high -> 120 km/h

        // Liear Interpolation function to get the maximum deceleration in velocity range 40-60 [km/h]
        // deceleration = a_coeff * v + b_coef
        // v [km/h] ; a [m/s^2]
        float maxH_dec_a_coeff = -0.000009f;
        float maxH_dec_b_coeff = 0.0025f;
        float maxH_dec_c_coeff = -0.2689f;
        float maxH_dec_d_coeff = -2.8689f;
        
        // Constant Interpolation function get the maximum deceleration when velocity >100 [km/h]
        float max_dec_const_high = -20.0f;

    public:
        SafetyForceField();
    
    public:
        enum class ShapeType 
        {
            CIRCLE = 0,
            OBB = 1,
        };

        enum class ObjectStateType
        {
            STATIC = 0,
            DYNAMIC = 1,
        };

        enum class SFFDecision 
        { 
            SAFE = 0,
            WARNING = 1,
            DANGER = 2,
        };

        struct CircleShape
        {
            cv::Point2f center;
            float radius;

            CircleShape() 
            : center(cv::Point2f(0.0f, 0.0f)), radius(1.0f) {}

            CircleShape(const cv::Point2f& center_, float radius_)
            :center(center_), radius(radius_) {}
        };

        struct OrientedBoxShape
        {
            cv::Point2f center;     // center
            float half_l;      // half length
            float half_w;      // half width  
            float yaw;         // rad

            OrientedBoxShape()
            : center(cv::Point2f(0.0f, 0.0f)), half_l(1.0f), half_w(1.0f), yaw(90.0f*M_PI/180.0f) {}

            OrientedBoxShape(const cv::Point2f& center_, float half_l_, float half_w_, float yaw_) 
            :center(center_), half_l(half_l_), half_w(half_w_), yaw(yaw_) {}
        };

        struct BoxEstimate {
            cv::Point2f center;
            float length; 
            float width;  
            float yaw;    
        };

        struct ClaimState 
        {
            cv::Point2f pos;   // x,y in world
            float       yaw;   // rad (world)
            float       t;     // s
            float       path_length;     // arc-length along path [m]
            float       v;     // m/s
            float       a;     // m/s^2
            float       b;     // curvature
            
            ShapeType   shape_type;
            CircleShape circle;
            OrientedBoxShape OBB;

            ClaimState()
            : pos(cv::Point2f(0.0f, 0.0f)), yaw(0.0f), t(0.0f), path_length(0.0f), v(0.0f), a(0.0f), b(0.0f),
            shape_type(ShapeType::CIRCLE),
            circle(pos, 1.0f), OBB(pos, 1.0f, 1.0f, 90.0f*M_PI/180.0f) {}

            ClaimState(
                const cv::Point2f& pos_, float yaw_, float t_, float path_length_,
                float v_, float a_, float b_, ShapeType type_, float radius_ = 4.0f, 
                float half_l_ = 1.0f, float half_w_ = 1.0f)
            : pos(pos_), yaw(yaw_), t(t_), path_length(path_length_),
              v(v_), a(a_), b(b_),
              shape_type(type_),
              circle(pos_, radius_), 
              OBB(pos_, half_l_, half_w_, yaw_) {}
        };

        struct ClaimSet
        {
            std::vector<ClaimState> claim_state_vector;
            ObjectStateType state_type;
            float time_danger;
            float time_reaction;
            float time_warning;
            float time_hit;
            float distance_brake_threshold;
            float in_low_vel_range;
            float in_stop_state;
            ClaimSet()
            : claim_state_vector({ClaimState()}), time_danger(0.0f), time_reaction(0.5f), in_low_vel_range(false),
                in_stop_state(false), distance_brake_threshold(1.5f), time_warning(1.5f), time_hit(-1.0f),
                state_type(SafetyForceField::ObjectStateType::STATIC) {}
        };
        
        struct OverlapResult 
        {
            bool hit;                 
            int object_index;          
            int ego_sample_index;     // index of state in claimset of ego
            int obj_sample_index;
            bool ego_in_stop_state;  
            float t_hit_ego;        
            float t_hit_obj;
            ClaimState ego_overlap; 
            ClaimState obj_overlap;      
            SFFDecision level; 
            OverlapResult() 
            : hit(false), object_index(0), ego_sample_index(0), obj_sample_index(0), 
            ego_in_stop_state(false), t_hit_ego(-1.0f), t_hit_obj(-1.0f),
            level(SFFDecision::SAFE) {}
        };

        struct SteerFilter 
        {
            float delta_prev;   // rad
            float rate_lim;     // rad/s  (limit rate of stearing)
            float tau0;         // s      ()
            float tau_k;        // s/(m/s) 
            SteerFilter()
            : delta_prev(0.f), rate_lim(45.f * M_PI/180.f),
                tau0(0.25f), tau_k(0.01f) {}
            void reset();
            float update(float delta_cmd, float v, float dt);
        };

        enum class ObjectType
        {
            EGO = 0,
            OBJECT = 1,
            UNDEFINED_OBJECT = 2,
        };

        // --------------- Helpers ---------------
    private:
        static bool convertClaimsetStop(const ClaimSet ego_claimset, ClaimSet& output_claimset);
        static float convertYaw(float yaw_carla);
        static float normalizeAngleToPi(float inputValue);
        
        static float roundNum(const float num, int precision = 2);
        static cv::Point2f roundPoint2f(const cv::Point2f& point_, int precision = 2);
        static std::string getDateTimeString();

        static cv::RotatedRect toRotRect(const OrientedBoxShape& obb);
        static bool intersectOBB_OBB(const OrientedBoxShape& boxA, const OrientedBoxShape& boxB);
        static bool intersectCircle_Circle(const CircleShape& circleA, const CircleShape& circleB);
        static bool intersectCircle_OBB(const CircleShape& circleA, const OrientedBoxShape& boxB);
        static bool overlapOBBOBB(const OrientedBoxShape& boxA, const OrientedBoxShape& boxB);
        
        static bool statesIntersect(const ClaimState& ego, const ClaimState& obj);

        
        static float dot2(const cv::Point2f& pointA, const cv::Point2f& pointB);
        static float length(const cv::Point2f& pointA);
        static cv::Point2f axisLong(float yaw);
        static cv::Point2f axisLat(float yaw);
        static float projExtentOBBOnAxis(const SafetyForceField::OrientedBoxShape& box,
                                    const cv::Point2f& axis_unit);
        float getWidthCar(ObjectType object_type);
        bool  createOffsetZone(float cur_vel, float time_danger, ClaimState& claimset_input);
        bool  getTimeReactionForOffset(float cur_vel, float danger_distance, float check_distance, float cur_time, float& time_reaction);
        float predictEgoDecelerationLowVel(const float input_vel, const float input_acc, const float time, float& total_time, bool& is_phase_12);
        float predictEgoDecelerationHighVel(const float input_vel, const float input_acc, const float time, float& total_time, bool& is_phase_12);
        BoxEstimate estimateBox(const cv::Point2f& closest, const cv::Point2f& position,
                                float half_width, float half_length,
                                const cv::Point2f& ego_pos, const OutputObject& object);
        BoxEstimate estimateBoxCar(const cv::Point2f& closest, const cv::Point2f& position,
                                const cv::Point2f& obj_velocity, float half_width,
                                float half_length, const cv::Point2f& ego_pos, Eigen::Vector4f bounding_box);
        float predictEgoMaximumDeceleration(const float input_velocity);
        // ---- Build Ego claimsets ----
        // δ>0 (left) => κ>0 if steerLeftPositive=true
        static float curvatureFromSteering(float delta_rad, float L, bool steerLeftPositive=true);
        ClaimSet buildClaimestFromVehicleState(float v0, float a0, float steering_angle,
                                                float T_danger, float T_reaction, float T_warning,
                                                float dt, bool isDanger = false, float a_min = 0);        
        
        
        // ---- Predict Objects claimsets ----
        void predictObjectClaimSet(const OutputObject& object, float total_time,
                                    float dt, ClaimSet& claimSet_, const cv::Point2f& ego_pos);
        
        SFFDecision decideLevelFromTime(float t_hit_ego, float time_danger_reaction, float time_warning);
        OverlapResult firstOverlapWithObject(const ClaimSet& ego, const ClaimSet& obj);
        
    public:
        // ---- Predict Ego claimsets ----
        void predictEgoClaimSet(float v0, float a0, const PerceptionOutputObject& perception_output,
                                float dt, std::vector<ClaimSet>& ego_claimset, bool is_braking);
        void predictObjectSetClaimSet(const PerceptionOutputObject& perception_output, float total_time,
                                        float dt, std::vector<ClaimSet>& objects_claimset, const cv::Point2f& ego_pos);
        SFFDecision checkOverlapsAndDecide();

        // Valid class ID
        std::array<int, 6> mindfull_object_IDs = {0, 1, 2, 3, 5, 7};
        
        SteerFilter steer_filter_;
        bool is_reset_steer_filter_;
        bool steer_left_positive_ = false;  //  old rule: delta>0 => κ<0
        
        std::chrono::steady_clock::time_point steer_last_tp_;
        bool steer_last_tp_valid_ = false;

        ClaimSet sffSet_state_;           // SFF EgoCar calculated by Current State
        ClaimSet sffSet_danger_;    // SFF EgoCar calculated by Emergency Danger
        std::vector<SafetyForceField::ClaimSet> sffSet_DangerCheck_;
        std::vector<SafetyForceField::ClaimSet> sffEgoSet_;
        std::vector<SafetyForceField::ClaimSet> sffObjSet_;           

        OverlapResult sffOverlap;
        SFFDecision decisionChecking;
        bool in_braking;
        static LogUtility log;
};

#endif  // SAFETY_FORCE_FIELD_H