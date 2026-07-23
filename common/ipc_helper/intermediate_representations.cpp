#include "intermediate_representations.h"

#include <random>

void InputObject::update(std::shared_ptr<InputObject> obj, const Eigen::Matrix<float, 4, 1>& ego_states) 
{
    if(this->sensor_source[SensorType::CAMERA])
    {
        // [1] Update position tracker
        Eigen::Matrix<float,LinearKalmanFilterCam::MEASUREMENT_VECTOR_SIZE_3D_CAM, 1> measurement_vector_3D;
        measurement_vector_3D <<obj->location[0], obj->location[1];  
        this->kalmanFilter[0]->update(measurement_vector_3D);

        // [2] Update bounding box tracker
        float center_x = (obj->bbox[0] + obj->bbox[2]) / 2.0;
        float center_y = (obj->bbox[1] + obj->bbox[3]) / 2.0;
        float width = (obj->bbox[2] - obj->bbox[0]);
        float height = (obj->bbox[3] - obj->bbox[1]);
        Eigen::Matrix<float, BboxLinearKalmanFilter::MEASUREMENT_VECTOR_SIZE_2D_CAM, 1> measurement_vector_2D;
        measurement_vector_2D << center_x, center_y, width, height;
        this->kalmanFilter[1]->update(measurement_vector_2D);
        
        // [3] Update class ID
        // Increase/decrease count
        #define maxClassCount 20
        if (this->classID == obj->classID)
            this->classCount++;
        else 
            this->classCount--;

        // Limit count in range (0, maxClassCount], if too low then change class
        if (this->classCount <= 0) {
            this->classID = obj->classID;
            this->classCount = 1;
        }
        else if (this->classCount > maxClassCount)
            this->classCount = maxClassCount;
    }
    else if(this->sensor_source[SensorType::RADAR])
    {
        Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1> measurement_vector;
        measurement_vector << obj->location[0], obj->velocity[0] + ego_states.coeffRef(0), obj->location[1], obj->velocity[1] + ego_states.coeffRef(2);
        this->kalmanFilter[0]->update(measurement_vector);
        this->classID       = obj->classID;
    }

    this->kalmanFilter[0]->refreshObjectData(this->location, this->velocity, this->acceleration);
    this->bbox          = obj->bbox;
    this->segment_mask  = obj->segment_mask;
    this->closest_point       = obj->closest_point;
    this->death_hit_streak_ = 0;
    ++(this->life_hit_streak_);
}

void InputObject::predict(const float &sample_time, const Eigen::Matrix<float, 4, 1> &ego_states)
{
    // Call all related Kalman Filter predict function
    if(this->sensor_source[SensorType::CAMERA])
    {
        this->kalmanFilter[0]->recalcKalmanFilterMatrices(sample_time);
        this->kalmanFilter[0]->predict(ego_states);
        this->kalmanFilter[0]->assignPredictedStatsToEstimatedStates();

        this->kalmanFilter[1]->recalcKalmanFilterMatrices(sample_time);
        this->kalmanFilter[1]->predict();
        this->kalmanFilter[1]->assignPredictedStatsToEstimatedStates();
    }
    else if(this->sensor_source[SensorType::RADAR])
    {
        this->kalmanFilter[0]->recalcKalmanFilterMatrices(sample_time);
        this->kalmanFilter[0]->predict(ego_states);
        this->kalmanFilter[0]->assignPredictedStatsToEstimatedStates();
    }
    // Reset life streak if the object is lost in *some* lastest frame
    if (this->death_hit_streak_ > 0)
        this->life_hit_streak_ = 0;

    ++(this->death_hit_streak_);
}

void InputObject::init(std::shared_ptr<InputObject> obj, const Eigen::Matrix<float, 4, 1> &ego_states)
{
    if(this->sensor_source[SensorType::RADAR])
    {   
        this->kalmanFilter.resize(1);
        this->kalmanFilter[0] = std::make_shared<LinearKalmanFilterRadar>();

        Eigen::Matrix<float, LinearKalmanFilterRadar::MEASUREMENT_VECTOR_SIZE_RADAR, 1> measurement_vector;
        measurement_vector << obj->location.x(), obj->velocity.x() + ego_states.coeffRef(0), obj->location.y(), obj->velocity.y() + ego_states.coeffRef(2);
        this->kalmanFilter[0]->update(measurement_vector);
    }
    else if(this->sensor_source[SensorType::CAMERA])
    {   
        this->kalmanFilter.resize(2);
        // The camera tracker includes two linear Kalman filters.
        // kalmanFilter[0] is used to smooth parameters in 3D space calculated by the bird's-eye view.
        // kalmanFilter[1] is used to track operations that use the camera bbox as an element of the cost function.

        this->kalmanFilter[0] = std::make_shared<LinearKalmanFilterCam>();
        this->kalmanFilter[1] = std::make_shared<BboxLinearKalmanFilter>();
        
        Eigen::Matrix<float, LinearKalmanFilterCam::MEASUREMENT_VECTOR_SIZE_3D_CAM, 1> measurement_vector_3D;
        measurement_vector_3D <<obj->location[0], obj->location[1]; 
        this->kalmanFilter[0]->update(measurement_vector_3D);
        
        float center_x = (obj->bbox[0] + obj->bbox[2]) / 2.0;
        float center_y = (obj->bbox[1] + obj->bbox[3]) / 2.0;
        float width = (obj->bbox[2] - obj->bbox[0]);
        float height = (obj->bbox[3] - obj->bbox[1]);
        Eigen::Matrix<float, BboxLinearKalmanFilter::MEASUREMENT_VECTOR_SIZE_2D_CAM, 1> measurement_vector_2D;
        measurement_vector_2D << center_x, center_y, width, height;
        this->kalmanFilter[1]->update(measurement_vector_2D);
    }
    
    this->kalmanFilter[0]->refreshObjectData(this->location, this->velocity, this->acceleration);
    if(this->sensor_source[SensorType::RADAR]) {
        this->location      = obj->location;
        this->velocity.x()  = obj->velocity.x() + ego_states.coeffRef(0);
        this->velocity.y()  = obj->velocity.y() + ego_states.coeffRef(2);
    }
    this->bbox          = obj->bbox;
    this->segment_mask  = obj->segment_mask;
    this->sensor_source = obj->sensor_source;
    this->classID       = obj->classID;
    this->closest_point = obj->closest_point;
}


/**
 * @brief
 * Decode lane marking type to check if this has doulbe line,
 * and if its lines are broken or solid.
 */
int LaneMarkingType::decode(int type, bool &double_line, bool line_types[2])
{
    switch (type) {  // Decide lines type, double or single
        case LaneMarkingType::NOT_EXIST:
            return -1;  // Error: lane does not exist
        case LaneMarkingType::OTHERS:
            double_line = false;
            line_types[0] = 1;
            line_types[1] = 0;
            break;
        case LaneMarkingType::BROKEN:
            double_line = false;
            line_types[0] = 0;
            line_types[1] = 0;
            break;
        case LaneMarkingType::SOLID:
            double_line = false;
            line_types[0] = 1;
            line_types[1] = 0;
            break;
        case LaneMarkingType::BROKEN_BROKEN:
            double_line = true;
            line_types[0] = 0;
            line_types[1] = 0;
            break;
        case LaneMarkingType::BROKEN_SOLID:
            double_line = true;
            line_types[0] = 0;
            line_types[1] = 1;
            break;
        case LaneMarkingType::SOLID_BROKEN:
            double_line = true;
            line_types[0] = 1;
            line_types[1] = 0;
            break;
        case LaneMarkingType::SOLID_SOLID:
            double_line = true;
            line_types[0] = 1;
            line_types[1] = 1;
            break;
        case LaneMarkingType::CURB:
            double_line = false;
            line_types[0] = 0;
            line_types[1] = 1;
            break;
        default:
            return 1;  // Warn: Unknown type
    }
    return 0;  // Decoded successfully
}


/**
 * @brief Calculate x at y using the coefficients of x's polynomial function.
 * @param y(float) The value to calculate at
 * @return (float) The value of x at y. Return 0.0 if lane marking is invalid.
 */
float LaneMarking::x(float y) const
{
    float x = 0.0f, y_pow = 1.0f;
    for (int i = 0; i < xCoeffs.size(); i++) {
        x += (xCoeffs[i] * y_pow);
        y_pow *= y;
    }
    return x;
}


/**
 * @brief Calculate z at y using the coefficients of z's polynomial function.
 * @param y(float) The value to calculate at
 * @return (float) The value of z at y. Return 0.0 if lane marking is invalid.
 */
// float LaneMarking::z(float y) const
// {
//     float z = 0.0f, y_pow = 1.0f;
//     if (valid()) {
//         for (int i = 0; i < zCoeffs.size(); i++) {
//             z += (zCoeffs[i] * y_pow);
//             y_pow *= y;
//         }
//     }
//     return z;
// }


/**
 * @brief Calculate both x and z at y using their corresponding polynomial function.
 * @param y(float) The value to calculate at
 * @param x(float) The value of x at y. Return 0.0 if lane marking is invalid.
 * @param z(float) The value of z at y. Return 0.0 if lane marking is invalid.
 */
// void LaneMarking::xz(float y, float &x, float &z) const
// {
//     float y_pow = 1.0f;
//     x = 0.0f;
//     z = 0.0f;
//     if (valid()) {
//         for (int i = 0; i <= degree; i++) {
//             if (i < xCoeffs.size()) x += (xCoeffs[i] * y_pow);
//             if (i < zCoeffs.size()) z += (zCoeffs[i] * y_pow);
//             y_pow *= y;
//         }
//     }
// }


/**
 * @brief Transform the current objects list's location and velocity
 * from local coordinates to global coordinates, and then to the target sensor's
 * coordinate system if a target sensor configuration is specified.
 *
 * This function performs a multi-stage transformation:
 * 1. Local -> Global
 * 2. Global -> Target Sensor (if specified)
 *
 * @param target_sensor: std::shared_ptr<SensorConfig> The target sensor configuration for final transformation.
 */
void SensorDataType::converTo(std::shared_ptr<SensorConfig> target_sensor) 
{
    Eigen::Matrix3f rotation_self;
    rotation_self =     Eigen::AngleAxisf(config->orientation[0], Eigen::Vector3f::UnitX()) *
                        Eigen::AngleAxisf(config->orientation[1], Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxisf(config->orientation[2], Eigen::Vector3f::UnitZ());
    
    for (auto& obj : this->objects) {
        obj->location = rotation_self * obj->location + config->position;
        obj->closest_point.y() = obj->closest_point.y() + 2.175f;//rotation_self * obj->closest_point + config->position;
        obj->velocity = rotation_self * obj->velocity;
        if(this->getSensorType() == SensorType::RADAR)
        {
            Eigen::Vector3f radar_coords_min(obj->bbox[0], obj->location.y(), obj->bbox[1]);
            Eigen::Vector3f radar_coords_max(obj->bbox[2], obj->location.y(), obj->bbox[3]);
            Eigen::Vector3f global_coords_min = rotation_self * radar_coords_min + config->position;
            Eigen::Vector3f global_coords_max = rotation_self * radar_coords_max + config->position;
            obj->bbox = Eigen::Vector4f(global_coords_min[0],global_coords_min[2],global_coords_max[0],global_coords_max[2]);
        }
    }


    if(target_sensor == nullptr) return;

    Eigen::Matrix3f rotation_target;
    rotation_target =   Eigen::AngleAxisf(target_sensor->orientation[0], Eigen::Vector3f::UnitX()) *
                        Eigen::AngleAxisf(target_sensor->orientation[1], Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxisf(target_sensor->orientation[2], Eigen::Vector3f::UnitZ());

    for (auto& obj : this->objects) {
        obj->location = rotation_target.transpose() * (obj->location - target_sensor->position);
        // obj->closest_point = rotation_target.transpose() * (obj->closest_point - target_sensor->position);
        obj->velocity = rotation_target.transpose() * obj->velocity;
    }  
}

/**
 * @brief Transform the current objects list's location and velocity
 * from the target sensor's coordinate system (if specified) to global coordinates,
 * and then to local coordinates.
 *
 * This function performs a multi-stage transformation:
 * 1. Target Sensor -> Global (if specified)
 * 2. Global -> Local
 *
 * @param source_sensor: std::shared_ptr<SensorConfig> The source sensor configuration for initial transformation.
 */
void SensorDataType::convertFrom(std::shared_ptr<SensorConfig> source_sensor)
{
    if (!source_sensor) {
        for (auto& obj : this->objects) {
            obj->location -= config->position;
        }
    }
    else {
        // Transform the object from the source sensor's local frame to the car's coordinate frame
        Eigen::Matrix3f rotation_source;
        rotation_source =   Eigen::AngleAxisf(source_sensor->orientation[0], Eigen::Vector3f::UnitX()) *
                            Eigen::AngleAxisf(source_sensor->orientation[1], Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(source_sensor->orientation[2], Eigen::Vector3f::UnitZ());
        for (auto& obj : this->objects) {
            obj->location = rotation_source * obj->location + source_sensor->position;
            obj->velocity = rotation_source * obj->velocity;
        }
    }

    // Transform the object from the car's coordinate frame to this sensor's local frame
    Eigen::Matrix3f rotation_self;
    rotation_self = Eigen::AngleAxisf(config->orientation[0], Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxisf(config->orientation[1], Eigen::Vector3f::UnitY()) *
                    Eigen::AngleAxisf(config->orientation[2], Eigen::Vector3f::UnitZ());
    for (auto& obj : this->objects) {
        obj->location = rotation_self.transpose() * obj->location;
        obj->velocity = rotation_self.transpose() * obj->velocity;
    }
}