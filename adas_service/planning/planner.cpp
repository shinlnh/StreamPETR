#include "planner.h"
LogUtility Planner::log_planner("log/", "SFF_in_Planner");
#define REFERENCE_POINTS_DENSITY 1

Planner::Planner(bool aeb_debug)
: trajectory_generator_(&frenet_environment_)
, decision_maker_(&frenet_environment_)
, aeb_debug(aeb_debug)
{
    this->aeb_controller_ = new AEBController();
    // log_planner.enableLogFile();
}

PlanningResults Planner::execute(PerceptionOutputObject& perception_output, 
                                 LaneDetection &lane_detection,
                                 TrajectoryGenerationPolicy trajectory_generation_policy, 
                                 DecisionMakingPolicy decision_policy)
{
    //-----** IMPORTANT NOTE **-----//
    // -- To handle the defect https://jira.banvien.com.vn/browse/BEP-1830 and enhance the HWC performance based on the enhanced Perception module
    // -- The Planning module is handled simply in this phase, therefore temporarily do not use the Frenet and PathGenegration, but in the future,
    // -- this flow will be restruct!

    // //> Update the environment everytime new perception results come in.
    // GeneralEnvironmentUpdate general_env_update;
    // general_env_update.image_width = perception_output.src_image_width;
    // general_env_update.image_height = perception_output.src_image_height;
    // general_env_update.lane_lines_count = lane_detection.laneLinesCount;
    // general_env_update.average_lane_postition = lane_detection.averageLanePosition;
    // general_env_update.is_image_dimension_setup = true;


    // // Need to fix here: 
    // std::vector<FusionObject> fusion_objects = convertToFusionObjects(perception_output);
    // std::vector<std::vector<cv::Point2f>> object_points_world = convertToObjectPointsWorld(perception_output);

    // frenet_environment_.updateEnvironment(general_env_update, lane_detection.left_coeffs_world, 
    //                                       lane_detection.right_coeffs_world, 
    //                                       object_points_world, 
    //                                       fusion_objects);

    // //> Generate candidate paths
    // trajectory_generator_.execute(trajectory_generation_policy);
    
	// //> Choose the best path
    // decision_maker_.execute(decision_policy);

    // //> Visualize, doesn't affect logic, you can comment this.
    // // visualizer_.execute();

    // return to_arrayof3_vector(retrieveSelectedPath());   
    
    //-----** IMPORTANT NOTE **-----//

    std::vector<FusionObject> fusion_objects = convertToFusionObjects(perception_output);
    std::vector<std::vector<cv::Point2f>> object_points_world = convertToObjectPointsWorld(perception_output);
   
    // Get the 2 lane information
    LaneMarking leftLane = lane_detection.laneMarkingsWorld[LaneMarkingID::LEFT_MID];
    LaneMarking rightLane = lane_detection.laneMarkingsWorld[LaneMarkingID::RIGHT_MID];

    // Extract center lanes
    static PlanningResults centerLine;
    static constexpr float turnThreshold = 0.002f;
    if (!((leftLane.type == LaneMarkingType::NOT_EXIST && (leftLane.xCoeffs[2] > -turnThreshold && leftLane.xCoeffs[2] < turnThreshold)) ||
        (rightLane.type == LaneMarkingType::NOT_EXIST && (rightLane.xCoeffs[2] > -turnThreshold && rightLane.xCoeffs[2] < turnThreshold))))
    {  
        computeCenterLane(leftLane, rightLane, centerLine);
    }

    centerLine.is_danger_aeb = is_danger_aeb;
    centerLine.is_warning_aeb = is_warning_aeb;
    return centerLine;   
}

void Planner::computeCenterLane(const LaneMarking& left, const LaneMarking& right, 
                                PlanningResults& centerLine)
{
    // if (left.type == LaneMarkingType::NOT_EXIST || right.type == LaneMarkingType::NOT_EXIST) {
    //     return;
    // }
    auto &centerPoints = centerLine.points;
    float y_min = std::max(left.startY, right.startY);
    float y_max = std::min(left.endY, right.endY);
    int sample_points = int((y_max - y_min) * REFERENCE_POINTS_DENSITY);
    sample_points = std::max(0, sample_points);

    centerPoints.clear();
    centerPoints.reserve(sample_points);

    float step = (y_max - y_min) / (sample_points - 1);

    for (int i = 0; i < sample_points; i++) {
        float y = y_min + step * i;
        float x_left = left.x(y);
        float x_right = right.x(y);
        float x_center = (x_left + x_right) / 2.0f;

        centerPoints.push_back({x_center, y, 0.0f});
    }
}

std::vector<std::vector<float>> Planner::retrieveSelectedPath()
{
    std::vector<std::vector<float>> cartesian_chosen_path;

    //> Get all the candidate paths
    std::vector<CandidatePath> candidate_paths = frenet_environment_.getCandidatePath();
    int number_of_paths = candidate_paths.size();
    
    // Early Return
    if(number_of_paths <= 0) 
        return {};

    //Init points's vector
    std::vector<Point2> points_data = {}; //Normal points
    std::vector<Point2> curve_points_data = {}; // Fitted curve points

    //> Loop through candidate path to find the CHOSEN PATH, then fit a curve through it.
    for (int path_index = 0; path_index < number_of_paths ; path_index++ )
    {
        CandidatePath now_cpath = candidate_paths.at(path_index);
        if (now_cpath.collision_check_score_ != SELECTED_PATH_INDICATOR) continue;
        
        // If found the selected path 
        // Retrieve all body points of the path
        if(now_cpath.getBodyPointsAddr()->size() <= 0) break;
        for (const auto& _point : *(now_cpath.getBodyPointsAddr()))
        {
            Point2 tmp_point = {static_cast<double>(_point.x),
                                static_cast<double>(_point.y)};
            points_data.emplace_back(tmp_point);
        }

        //At the tail point( at end of the path)
        CoorPoint2i tail_point = now_cpath.tail_;
        Point2 tmp_tail_point = {static_cast<double>(tail_point.x),
                                static_cast<double>(tail_point.y)};
        points_data.emplace_back(tmp_tail_point);
        
        //now construct the curve path
        const int number_of_curve_points = 130;
        curve_points_data.resize(number_of_curve_points);
        //error = 0 , coarse = 300 , offset = 100
        CurveFitter curve_planner(0, 300, 100);
        curve_points_data = curve_planner.getPlannedPath(points_data, number_of_curve_points);
    }

	//> Convert the selected path to the Cartesian space.
    
    /*!  This function check conditions for a valid conversion*/ 
    /*!  - 1st condition: has a selected path in the \p selected_curve_path member variable*/
    if (curve_points_data.empty()) {
        cartesian_chosen_path = {};
        DEBUG("No path to be converted");
    } else if (frenet_environment_.frenet_left_lane_.empty() || frenet_environment_.frenet_right_lane_.empty()) {
    /*! - 2nd condition: has both left and right line in frenet space data*/
        cartesian_chosen_path = {};
        DEBUG("No lane detection in frenet available");
    } else {
        std::vector<std::vector<float>> mapPointsFrenet = frenet_environment_.frenet_left_lane_;
        std::vector<std::vector<float>> rightPointsFrenet = frenet_environment_.frenet_right_lane_;
        /**
         * If both conditions is qualified, start convert path from \p xy to frenet for each point, 
         * using the right-line lane points as reference.
         */
        for (size_t i = 0; i < frenet_environment_.frenet_right_lane_.size(); i++) {
            if (i >= curve_points_data.size()) {
                break;
            }

            float s = rightPointsFrenet[i][2];
            /**
             * As the right line is taken as a reference, the amount of differential between
             * the planned path and the left-line zero based is calculated as
             * \f[ \text{ratio} = \frac{x_{path}}{max_{\text{right line}}} = \frac{x_{path}}{500} \f]
             */
            float ratio = (curve_points_data[i].x - frenet_environment_.getPlanningLaneOffset())/(LANE_WIDTH * SCALE); //choose the amount from the 1st lane line fixed
            float d = rightPointsFrenet[i][3]*ratio;
            float map_index = rightPointsFrenet[i][4];

            /**
             * After getting the ratio, compute backward the (x,y) coordinate related to Frenet Space for the planned path
             * \f[ x_{frenet} = x_{\text{left line}}*(1-\text{ratio}) + x_{\text{right line}}*{\text{ratio}} \f]
             * \f[ y_{frenet} = y_{\text{left line}}*(1-\text{ratio}) + y_{\text{right line}}*{\text{ratio}} \f]
             */
            float x = mapPointsFrenet[map_index][0]*(1-ratio) + rightPointsFrenet[i][0]*ratio;
            float y = mapPointsFrenet[map_index][1]*(1-ratio) + rightPointsFrenet[i][1]*ratio;

            /**
             * A single point will have 5 data fields inside: \p x, \p y represent the relative distance in Euler space (normal 2D space) 
             * of the planned path to the mapPointsFrenet. \p s, \p d represent the same information as \p x, \p y but in Frenet space.
             * \p id parameter for identify between point (for furture usecase). 
             */
            cartesian_chosen_path.push_back({x, y, s, d, map_index});
        }
    }
    return cartesian_chosen_path;
}

void Planner::resetAEBState()
{
    this->is_danger_aeb = false;
    this->is_warning_aeb = false;
}

bool Planner::dangerCheck(PerceptionOutputObject& perception_output)
{
    const float eps = 1e-5f;
    float following_car_velocity = perception_output.my_car_speed.linear_velocity.y;                   //[Unit] =[m/s]

    // Skip AEB detection if vehicle is moving backward (negative velocity)
    // This handles N gear rolling backward scenario without complex gear logic
    if (following_car_velocity < 0.0f) {
        this->safety_force_field_.decisionChecking = SafetyForceField::SFFDecision::SAFE;
        this->is_danger_aeb = false;
        this->is_warning_aeb = false;
        return false;
    }

    if (!perception_output.is_reverse)
    {
        this->safety_force_field_.sffObjSet_.clear();
        this->safety_force_field_.sffObjSet_.reserve(perception_output.objects.size());

        // Claim set for Ego Car
        this->safety_force_field_.sffEgoSet_.clear();
        this->safety_force_field_.sffEgoSet_.reserve(2);
        this->safety_force_field_.predictEgoClaimSet(following_car_velocity, perception_output.my_car_state.Imu_linear_acceleration.x,
                                                        perception_output, 0.01f,
                                                        this->safety_force_field_.sffEgoSet_, this->is_danger_aeb);
        float time_field_obj = 0;
        // Claim set for Objects
        if (this->safety_force_field_.sffSet_danger_.in_low_vel_range) // danger-warning sff of ego in low range
        {
            time_field_obj = this->safety_force_field_.sffSet_state_.time_danger + this->safety_force_field_.sffSet_state_.time_warning;
        }
        else
        {
            if (!this->safety_force_field_.sffSet_danger_.claim_state_vector.empty())
                time_field_obj = this->safety_force_field_.sffSet_danger_.claim_state_vector.back().t;
            else
                time_field_obj = 0;
        }
        this->safety_force_field_.predictObjectSetClaimSet(perception_output, time_field_obj, 0.01f,
                                                            this->safety_force_field_.sffObjSet_,
                                                            this->safety_force_field_.sffSet_danger_.claim_state_vector[0].pos);
    }
    else
    {
        this->safety_force_field_.sffObjSet_.clear();
        this->safety_force_field_.sffObjSet_.reserve(perception_output.objects.size());


        // Claim set for Ego Car
        this->safety_force_field_.sffEgoSet_.clear();
        this->safety_force_field_.sffEgoSet_.reserve(2);
    }
    if (!perception_output.is_reverse ) // && ((std::abs(following_car_velocity) > 1e-3f) || this->is_danger_aeb)
    {
        if (this->safety_force_field_.is_reset_steer_filter_)
        {
            this->safety_force_field_.is_reset_steer_filter_ = false;
        }
        this->safety_force_field_.decisionChecking = this->safety_force_field_.checkOverlapsAndDecide();
    }
    else
    {
        if (!this->safety_force_field_.is_reset_steer_filter_)
        {
            this->safety_force_field_.steer_filter_.reset();
            this->safety_force_field_.is_reset_steer_filter_ = true;
        }  
        this->safety_force_field_.decisionChecking = SafetyForceField::SFFDecision::SAFE;
    }
    

    // AEB debug visualization moved to adas_visualize_service via /sff_debug DDS topic
        
    if (this->safety_force_field_.in_braking)
    {
        if(following_car_velocity <= eps)
        {
           this->safety_force_field_.in_braking = false;
        }
    }

    if (this->safety_force_field_.decisionChecking == SafetyForceField::SFFDecision::DANGER)
    {
        this->is_danger_aeb                  = true;
        this->is_warning_aeb                 = false;
        this->safety_force_field_.in_braking = true;
    }
    else if (this->safety_force_field_.decisionChecking == SafetyForceField::SFFDecision::WARNING)
    {
        if(!this->safety_force_field_.in_braking)
        {
            this->is_danger_aeb = false;
            this->is_warning_aeb = true;
        }   

    }
    else if (this->safety_force_field_.decisionChecking == SafetyForceField::SFFDecision::SAFE)
    {
        if (!this->safety_force_field_.in_braking)
        {
            this->is_danger_aeb = false;
            this->is_warning_aeb = false;
        }
    }

    return this->is_danger_aeb;
}

bool Planner::isObjectInRange(const std::shared_ptr<const OutputObject>& object)
{
    float horizontal_bound = (this->car_width + 0.1)/2.0f;
    return (object->closest_point.x() > -horizontal_bound && object->closest_point.x() < horizontal_bound);
}

bool Planner::isObjectInLane(const std::shared_ptr<const OutputObject>& object,
                             const std::vector<float>& left_coeffs_world,
                             const std::vector<float>& right_coeffs_world)
{
    // calculat f(x), input is coeffs of function and value of x
    auto f_x = [](const std::vector<float>& coeffs, float y) -> float {
        if (coeffs.empty()) return 0.0f;  // Tránh lỗi nếu vector rỗng
    
        float result = coeffs.back();  // Hệ số cao nhất
        for (int idx = coeffs.size() - 2; idx >= 0; --idx) {
            result = result * y + coeffs[idx];
        }
        return result;
    };
    
    // Check whether the object is inside the lane boundaries.
    // For an object at point P(x, y), compute the lane boundary x-coordinates at its y position:
    //    leftBoundary = f_x(left_coeffs_world, y)
    //    rightBoundary = f_x(right_coeffs_world, y)
    // The object is considered within the lane if its x-coordinate lies between these boundaries,
    // which is true when (x - leftBoundary) and (x - rightBoundary)
    // have opposite signs, their product is negative.
    float fx_lane_left = f_x(left_coeffs_world, object->location.y());
    float fx_lane_right = f_x(right_coeffs_world, object->location.y());
    return (object->location.x() - fx_lane_left) * (object->location.x() - fx_lane_right)  < 0;
}

const std::shared_ptr<OutputObject> Planner::getClosestObject(PerceptionOutputObject& perception_output)
{
    std::shared_ptr<OutputObject> object_ptr = nullptr;
    float danger_distance = std::numeric_limits<float>::max();
    for (const std::shared_ptr<OutputObject>& object : perception_output.objects) 
    {
        if (isObjectInRange(object) && object->closest_point.y() < danger_distance) {
            object_ptr = object;
            danger_distance = object->closest_point.y();
        }
    }
    return object_ptr;
}

const std::shared_ptr<OutputObject> Planner::getClosestVehicle(PerceptionOutputObject& perception_output)
{
    // Find the closest vehicle (classId == 2) to us.
    std::shared_ptr<OutputObject> object_ptr = nullptr;
    float minDistance = std::numeric_limits<float>::max();
    for (const std::shared_ptr<OutputObject>& object : perception_output.objects) {
        if (isObjectInRange(object) && object->classID == 2 && object->location.y() < minDistance) {
            object_ptr = object;
            minDistance = object->location.y();
        }
    }

    // If we didn't find any vehicle, return nullptr.
    return object_ptr;
}

float computeLaneArcLength(const std::vector<float>& coeffs, float y_start, float y_end, int numSamples = 1000) {
    // Compute derivative coefficients for f'(y).
    // For f(y) = a0 + a1*y + a2*y^2 + ... + an*y^n, the derivative is
    // f'(y) = a1 + 2*a2*y + 3*a3*y^2 + ... + n*an*y^(n-1).
    std::vector<float> derivCoeffs;
    derivCoeffs.reserve(coeffs.size() - 1);
    for (size_t i = 1; i < coeffs.size(); ++i) {
        derivCoeffs.push_back(i * coeffs[i]);
    }
    
    // Step size in y.
    float dy = (y_end - y_start) / numSamples;
    float arcLength = 0.0f;
    
    // Trapezoidal rule: Sum the integrand values.
    for (int i = 0; i <= numSamples; ++i) {
        float y = y_start + i * dy;
        // Evaluate the derivative f'(y) at y.
        float dfdY = 0.0f;
        float power = 1.0f;
        for (float dcoef : derivCoeffs) {
            dfdY += dcoef * power;
            power *= y;
        }
        // The integrand is sqrt(1 + (f'(y))^2)
        float integrand = std::sqrt(1.0f + dfdY * dfdY);
        
        // Apply half weight for endpoints.
        if (i == 0 || i == numSamples) {
            arcLength += 0.5f * integrand;
        } else {
            arcLength += integrand;
        }
    }
    
    // Multiply by step size to complete the integration.
    arcLength *= dy;
    return arcLength;
}

std::pair<std::shared_ptr<OutputObject>, float> Planner::getClosestVehicleInLane(
    PerceptionOutputObject& perception_output,
    const LaneDetection& lane_detection)
{
    std::shared_ptr<OutputObject> closest_object = nullptr;
    float min_distance_in_lane = std::numeric_limits<float>::max();

    // Compute centerline coefficients by averaging left and right lane coefficients.
    LaneMarking const &leftLane = lane_detection.laneMarkingsWorld[LaneMarkingID::LEFT_MID],
                      &rightLane = lane_detection.laneMarkingsWorld[LaneMarkingID::RIGHT_MID];
    int n = 0;
    if (leftLane.xCoeffs.size() == rightLane.xCoeffs.size()) {
        n = leftLane.xCoeffs.size();
    }
    std::vector<float> centerCoeffs(n);
    for (int i = 0; i < n; ++i) {
        centerCoeffs[i] = (leftLane.xCoeffs[i] + rightLane.xCoeffs[i]) / 2.0f;
    }

    // Define the starting y-position for your vehicle.
    float ego_y = 0.0f;  // Adjust this value based on your coordinate system.

    // Iterate through each detected object.
    for (const auto& object : perception_output.objects) {
        // Check if the object is in the lane and is of the desired type (classID == 2, e.g., a vehicle).
        if (isObjectInLane(object, leftLane.xCoeffs, rightLane.xCoeffs) 
            && object->classID == 2) 
        {
            // Compute the arc length along the centerline from ego_y to the object's y-coordinate.
            float laneArcDistance = computeLaneArcLength(centerCoeffs, ego_y, std::max(0.0f, object->location.y()));
            
            // Update the closest object if this one is nearer along the lane.
            if (laneArcDistance < min_distance_in_lane) {
                min_distance_in_lane = laneArcDistance;
                closest_object = object;
            }
        }
    }

    // Return the closest object and the corresponding lane distance.
    return std::make_pair(closest_object, min_distance_in_lane);
}

/**
 * @brief Based on perception result, check if there is traffic jam.
 * Apply a Low pass filter to help stablize output.
 * 
 * @return True if there is traffic jam.
 */
bool Planner::detectTrafficJam(const PerceptionResults &perceptionResult)
{
    // Static setting
    static constexpr float trafficJamDistThreshold = 25.0f;     // meters
    static constexpr float trafficJamSpeedThreshold = 40.0f;    // km/h
    static constexpr int maxTrafficJamCount = 50;

    // Check if found any vehicle in our lane, if not then not traffic jam
    if (perceptionResult.closest_car_distance < 0.0f) {
        if (--trafficJamCounter <= 0) {
            is_traffic_jam = false;
            trafficJamCounter = 0;
        }
        return is_traffic_jam;
    }

    // Information from perception
    const auto& objects = perceptionResult.adv_objs;
    const auto& srcs = perceptionResult.src_objs;
    const auto& lanes = perceptionResult.laneResult;

    // Check number of lane and vehicle that are close and move slowly
    int numLane = std::max(lanes.laneLinesCount - 1, 0);
    int numVehAhead = 0;
    for (size_t i = 0; i < objects.size(); i++) {
        const auto& object = objects[i];
        const auto& src = srcs[i];
        // Check class and sensor source
        if (object.classId != 2 || !src.at(SensorType::FUSION))
            continue;

        // Check velocity
        float objectVelocity = sqrtf((object.x_velocity * object.x_velocity) + (object.y_velocity * object.y_velocity));
        if (objectVelocity < trafficJamSpeedThreshold) {
            numVehAhead ++;
            continue;
        }

        // Check dist
        float objectDist = sqrt((object.x_offset * object.x_offset) + (object.y_offset * object.y_offset));
        if (objectDist < trafficJamDistThreshold)
            numVehAhead ++;
    }

    // Increase counter if many vehicles are close, moving slowly and fill the lanes
    if (numVehAhead >= numLane) {
        trafficJamCounter++;
    }
    else {
        trafficJamCounter--;
    }

    // Change state if counter reach min or max
    if (trafficJamCounter >= maxTrafficJamCount) {
        is_traffic_jam = true;
        trafficJamCounter = maxTrafficJamCount;
    }
    else if (trafficJamCounter <= 0) {
        is_traffic_jam = false;
        trafficJamCounter = 0;
    }

    return is_traffic_jam;
}

/**
 * @brief Function to convert to simple objects to put into Frenet space. This function does the 
 * required encapsulation job so that FrenetEnvironment doesn't need to know of complex objects
 * passed between Perception and Planning modules. If more speed is necessary, consider removing 
 * this function and directly couple the percetion results to Frenet Environment.
 * 
 * @param advanced_objects 
 * @return std::vector<FusionObject> 
 */
std::vector<FusionObject> Planner::convertToFusionObjects(PerceptionOutputObject& perception_output) 
{
    std::vector<FusionObject> fusion_objects;
    fusion_objects.reserve(perception_output.objects.size()); // Pre-allocate memory for better performance
    
    // Convert each AdvanceFusionObject to FusionObject through object slicing
    for (std::shared_ptr<OutputObject> object : perception_output.objects) {
        //==================================================================
        //convert from outputObject to fusionobject to keep the integration of code, required fix here: 
        FusionObject temp = FusionObject();
        temp.x_offset = object->location.x();
        temp.y_offset = object->location.y();
        temp.z_offset = object->location.z();
        temp.velocity = object->velocity.y();

        temp.center_point.x = object->location.x();
        temp.center_point.y = object->location.y();
        temp.mask           = object->segment_mask;
        temp.classId = object->classID;
        temp.bbox = {object->bbox[0], object->bbox[1], object->bbox[2], object->bbox[3]};
        fusion_objects.push_back(temp);
        //==================================================================
    }
    
    return fusion_objects;
}

/**
 * @brief Converts detected objects' bounding boxes into real-world points.
 * 
 * This function encapsulates the conversion of complex object detection results
 * into a simpler representation (real-world points along the bottom edge of each object's
 * bounding box). This simplifies further processing by downstream modules.
 * An extra set of points representing the car's width is appended.
 * 
 * @param objects Detected objects with bounding boxes.
 * @return std::vector<std::vector<cv::Point2f>> A vector of vectors, each containing real-world
 * coordinates for an object's bottom edge (plus the car points as the last element).
 */
std::vector<std::vector<cv::Point2f>> Planner::convertToObjectPointsWorld(
    PerceptionOutputObject& perception_output)
{
    std::vector<std::vector<cv::Point2f>> objectsPointsWorld;
    
    // For each detected object, simply extract its pre-computed location.
    for (const auto &object : perception_output.objects)
    {
        std::vector<cv::Point2f> objectPoints;
        objectPoints.push_back(cv::Point2f(object->location.x(), object->location.y()));
        objectsPointsWorld.push_back(objectPoints);
    }
    
    // Append car points based on the car's width.
    float carWidth = perception_output.my_car_width;
    std::vector<cv::Point2f> carPoints;
    for (float x = -carWidth * 0.5f; x <= carWidth * 0.5f; x += 0.05f)
    {
        carPoints.push_back(cv::Point2f(x, 0));
    }
    objectsPointsWorld.push_back(carPoints);
    
    return objectsPointsWorld;
}