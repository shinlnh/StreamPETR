#ifndef BIRDVIEWMODEL_TEST_H
#define BIRDVIEWMODEL_TEST_H

#include <opencv2/opencv.hpp>

class BirdViewModel
{
public:
    void calibrate(float carpet_width = 0,
                   float car_to_carpet_distance = 0,
                   float carpet_length = 0,
                   std::vector<cv::Point2f> four_image_points = std::vector<cv::Point2f>()) { /* do nothing */ }
};

#endif