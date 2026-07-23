#ifndef TRAFFIC_OBJECT_H
#define TRAFFIC_OBJECT_H

#include <string>
#include <array>
#include <vector>
#include "types.h"
#include "linear_kalman_filter.hpp"
#include "linear_kalman_filter_for_cam.hpp"
#include <opencv2/opencv.hpp>

#define UNKNOWN_VELOCITY -666.0f
//Car dimensions. (in meters). Model after the Toyota Corolla
#define CAR_LENGTH                  4.46
#define CAR_WIDTH                   1.82

class TrafficObject {
public:        
    Box bbox;                          // Bounding box coordinates
    int classId = -1;                 // ID number of class
    float prob = 0.0f;                 // Probability of class of object
    float distance_to_my_car = -1;     // Distance from object to car
    cv::Mat mask;                      // Pixel mask of object segmentation

    TrafficObject(Box bbox, 
                  int classId = -1, 
                  float prob = 0.0f, 
                  float distance_to_my_car = -1, 
                  cv::Mat mask = cv::Mat{}) 
        : bbox(bbox), 
          classId(classId), 
          prob(prob), 
          distance_to_my_car(distance_to_my_car), 
          mask(mask) {}

    TrafficObject() = default;
    // Inline area calculation
    inline float area() const {
        return (bbox.x2 - bbox.x1) * (bbox.y2 - bbox.y1);
    }
};

/**
 * @brief This is the representation of a 'final' object after the perception pipeline, resulted after all
 * the matching procedures is done. This object contains the most probable location and velocity of the percepted
 * object the combined information of all the sensors.
 * 
 * Contains all the information from object detections. Have new field to hold additionals information.
 * 
 * x_offset         : Offset in the X direction (perpendicular to your vision, in meters)
 * y_offset         : Offset in the Y direction (in meters)
 * velocity         : In km/h
 * is_inside_lane   : A flag denoted if an object is in the same lane as our car. This is set in the FrenetEnvironment
 */
class FusionObject : public TrafficObject
{
public:
    float x_offset;
    float y_offset;
    float z_offset;
    float velocity = UNKNOWN_VELOCITY;
    float x_velocity = UNKNOWN_VELOCITY;
    float y_velocity = UNKNOWN_VELOCITY;
    bool is_inside_lane = false;
    bool from_camera = false;
    bool from_radar = false;
    float visual_x;
    float visual_y;
    landmarks center_point;
    float width_obj = CAR_WIDTH;
    float length_obj = CAR_LENGTH;

    // Inherit the base class's constructor.
    using TrafficObject::TrafficObject;

    FusionObject() 
    {
        x_offset = 0.0f;
        y_offset = 0.0f;
        z_offset = 0.0f;
        x_velocity = UNKNOWN_VELOCITY;
        y_velocity = UNKNOWN_VELOCITY;
        is_inside_lane = false;
        from_camera = false;
        from_radar = false;
        visual_x = -1.0f;
        visual_y = -1.0f;
        width_obj = CAR_WIDTH;
        length_obj = CAR_LENGTH;
    }

    FusionObject(float x_offset, 
                 float y_offset, 
                 float z_offset,
                 float velocity,
                 bool is_inside_lane, 
                 bool from_camera, 
                 bool from_radar,
                 float visual_x = -1.0f,
                 float visual_y = -1.0f
                 ) 
    : 
    x_offset(x_offset),
    y_offset(y_offset),
    z_offset(z_offset),
    velocity(velocity),
    is_inside_lane(is_inside_lane), 
    from_camera(from_camera),
    from_radar(from_radar),
    visual_x(visual_x),
    visual_y(visual_y)
    {
    }

    FusionObject(float x_offset, 
                 float y_offset, 
                 float z_offset,
                 float x_velocity,
                 float y_velocity,
                 bool is_inside_lane, 
                 bool from_camera, 
                 bool from_radar,
                 float visual_x = -1.0f,
                 float visual_y = -1.0f
                 ) 
    : 
    x_offset(x_offset),
    y_offset(y_offset),
    z_offset(z_offset),
    x_velocity(x_velocity),
    y_velocity(y_velocity),
    is_inside_lane(is_inside_lane), 
    from_camera(from_camera),
    from_radar(from_radar),
    visual_x(visual_x),
    visual_y(visual_y)
    {
    }

    //*Construct a FusionObject from traffict object
    FusionObject(const TrafficObject &detection,
                 float x_offset, 
                 float y_offset, 
                 float z_offset,
                 float x_velocity, 
                 float y_velocity, 
                 bool is_inside_lane,
                 bool from_camera, 
                 bool from_radar)
    :
    TrafficObject(detection.bbox,
                  detection.classId,
                  detection.prob,
                  detection.distance_to_my_car,
                  detection.mask),
    x_offset(x_offset),
    y_offset(y_offset),
    z_offset(z_offset),
    x_velocity(x_velocity),
    y_velocity(y_velocity),
    is_inside_lane(is_inside_lane), 
    from_camera(from_camera),
    from_radar(from_radar)
    {}
};

class AdvanceFusionObject : public FusionObject
{
public:
    uint32_t id_ = 0; // Object with id = 0 is considered error, in runtime there're shouldn't any id_=0 object available
    float velocity_absoulute = UNKNOWN_VELOCITY;
    AdvanceFusionObject(const FusionObject &obj) : FusionObject(obj)
    {
    }

    AdvanceFusionObject()
    :
    FusionObject(0.0f,
                 0.0f,
                 0.0f,
                 UNKNOWN_VELOCITY,
                 UNKNOWN_VELOCITY,
                 false,
                 false,
                 false,
                 -1.0f,
                 -1.0f)
    {
    }
};

#endif // TRAFFIC_OBJECT_H
