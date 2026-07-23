#ifndef STUB_TRAFFIC_OBJECT_H
#define STUB_TRAFFIC_OBJECT_H

#include <string>
#include "types.h"
#include "vector"
#include "array"
#include <opencv2/opencv.hpp>

#define UNKNOWN_VELOCITY -666.0f

class TrafficObject {
public:
   Box bbox;
   int classId = -1;
   float prob = 0.0f;
   float distance_to_my_car = -1;
   cv::Mat mask;

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

   inline float area() const {
       return (bbox.x2 - bbox.x1) * (bbox.y2 - bbox.y1);
   }
};

class FusionObject: public TrafficObject {
public:
   float x_offset;
   float y_offset; 
   float z_offset;
   bool from_radar;
   bool from_camera;
   bool is_inside_lane = false;
   std::vector<std::array<float,5>> frenet_points;
   float visual_x;
   float visual_y;
   float velocity = UNKNOWN_VELOCITY;

   using TrafficObject::TrafficObject;

   FusionObject() {
       x_offset = 0.0f;
       y_offset = 0.0f;
       z_offset = 0.0f;
       velocity = UNKNOWN_VELOCITY;
       is_inside_lane = false;
       from_camera = false;
       from_radar = false;
       visual_x = -1.0f;
       visual_y = -1.0f;
   }

   FusionObject(Box bbox, int classId, float x, float y, cv::Mat mask) 
       : TrafficObject(bbox, classId, 0.0f, -1, mask) {
       x_offset = x;
       y_offset = y;
   }

   FusionObject(float x_offset,
                float y_offset,
                float z_offset, 
                float velocity,
                bool is_inside_lane,
                bool from_camera,
                bool from_radar,
                float visual_x = -1.0f,
                float visual_y = -1.0f)
       : x_offset(x_offset),
         y_offset(y_offset),
         z_offset(z_offset),
         velocity(velocity), 
         is_inside_lane(is_inside_lane),
         from_camera(from_camera),
         from_radar(from_radar),
         visual_x(visual_x),
         visual_y(visual_y) {}
};

class AdvanceFusionObject : public FusionObject {
public:
   AdvanceFusionObject() 
       : FusionObject(0.0f, 0.0f, 0.0f, UNKNOWN_VELOCITY, false, false, false, -1.0f, -1.0f) {}
   AdvanceFusionObject(const FusionObject &obj) : FusionObject(obj) {}
};

#endif