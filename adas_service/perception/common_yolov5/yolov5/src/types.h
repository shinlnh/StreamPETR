#pragma once

#include "config.h"

struct YoloKernel {
  int width;
  int height;
  float anchors[OD_NUM_ANCHOR * 2];
};

struct Box{
    float x1;
    float y1;
    float x2;
    float y2;
};

struct landmarks{
    float x;
    float y;
};

struct DetectionBase {
    //x1 y1 x2 y2
    Box bbox;
    int classId;
    float prob;
    landmarks marks[5];
};

struct alignas(float) Detection {
  float bbox[4];  // center_x center_y w h
  float conf;     // bbox_conf * cls_conf
  float class_id;
  float mask[32];
};

