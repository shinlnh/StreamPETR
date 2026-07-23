#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#include "common.h"
#ifndef UT_TEST
#include "engine.h"
#include "laneatt_utils.h"
#include "ctpl_stl.h"
#include "intermediate_representations.h"
#else
#include "lib4test.h"
#endif


class LaneDetector
{
private:
    TRTEngine trt_engine;
    int lane_lines_count = 4;
    int count_change_number = 0;
    float average_lane_postition = 400;
    
    // Thread Pooling
    ctpl::thread_pool detect_thread_pool;

    // ---- Lanes properties ----
    // Lane line types
    std::vector<LanePointsRaw> lanePoints;

    std::mutex laneDetectorMutex;

private:
    void updateLaneLineCount(int new_lane_lines_count);
    
public:
    LaneDetector();
    ~LaneDetector();
    void reset(void);
     
    void detect(const cv::Mat &srcImage);

    std::vector<LanePointsRaw> getLanes() const {return lanePoints;}

    int getLaneLinesCount() const {return lane_lines_count;}
    float getAverageLanePosition() const {return average_lane_postition;}
};

#endif // LANE_TRACKING_V2_H
