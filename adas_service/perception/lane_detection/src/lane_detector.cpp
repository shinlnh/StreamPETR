#include "lane_detector.h"

// #define DEBUG_LANE_COUNT

LaneDetector::LaneDetector()
: detect_thread_pool(2)
, lanePoints(LaneMarkingID::NUM_LANE_MARKING)
{}

LaneDetector::~LaneDetector()
{}

void LaneDetector::reset(void) 
{
    std::lock_guard<std::mutex> guard(laneDetectorMutex);

    // Clear points
    for (int i = 0; i < LaneMarkingID::NUM_LANE_MARKING; i++) {
        lanePoints[i].clear();
    }
}

void LaneDetector::detect(const cv::Mat &srcImage)
{
    std::lock_guard<std::mutex> guard(laneDetectorMutex);
    if(srcImage.channels() != INPUT_C) {
        ERROR("LANE - Received image with %d channels, expected %d!",
              srcImage.channels(), INPUT_C);
        return;
    }

    // ---- [1] Inference lane using model ----
    cv::Mat network_output; 
    auto engine_network_future = detect_thread_pool.push([&](int id) {
        trt_engine.runInferenceWithThread(srcImage, network_output);
    });
    engine_network_future.get();

    if(network_output.empty()) {
        ERROR("LANE - Engine's output is empty!");
        return;
    }

    // ---- [2] Post processing: Filter lane by class, threshold, NMS ----
    cv::Mat lanes = LaneATTPostProcess(network_output, CONF_THRESH, IOU_THRESH, TOPK);

    // ---- [3] Extract lane points from lane results ----
    for (int i = 0; i < lanes.rows; i++) {  // TEMPORARY REQUIRE LANE TO OUTPUT POINTS ON 35TH ANCHOR
        float *lane = lanes.ptr<float>(i);
        int y0 = (int) (lane[2] * N_POINT_PER_LINE + 0.5);
        float &length = lane[4];
        length = 35 + 1 - y0;
    }
    std::vector<std::vector<cv::Point2f>> lanesAsPoints = LaneATTLanesToPoints(lanes);

    // ---- [4] Update module's attributes ----
    /**
     * THIS IS TEMPORARY.
     * This part guess detected lanes' relative position to the car.
     * This function will be handled by localization module in the future.
     */
    std::vector<cv::Vec4f> laneLineSlopes(lanes.rows);
    for (int row_id = 0; row_id < lanes.rows; row_id++) {
        std::vector<cv::Point2f> &curLanePoints = lanesAsPoints[row_id];
        if (curLanePoints.empty()) {
            continue;
        }
        cv::fitLine(curLanePoints, laneLineSlopes[row_id], cv::DIST_L2, 0, 0.01, 0.01);
        
        laneLineSlopes[row_id][1] = -1 * laneLineSlopes[row_id][1];  // Image y axis is reversed
        laneLineSlopes[row_id][2] = (curLanePoints.back().x - CROP_X) / CROP_W;
        laneLineSlopes[row_id][3] = (curLanePoints.back().y - CROP_Y) / CROP_H;
    }

    std::vector<int> laneIds(LaneMarkingID::NUM_LANE_MARKING, -1);
    for (size_t i = 0; i < laneLineSlopes.size(); i++) {
        if (lanesAsPoints[i].empty()) {
            continue;
        }
        cv::Vec4f &curLane = laneLineSlopes[i];
        float dx = curLane[0];
        float dy = curLane[1];
        float x = curLane[2];
        float y = curLane[3];

        float slope = 1e-9;
        if (dx != 0) {
            slope = dy / dx;
        }

        if (y < 0.75) {
            if (0.0f <= x && x <= 0.1f && slope > 0.0f) {
                if (laneIds[LaneMarkingID::LEFT_SIDE] == -1) {
                    laneIds[LaneMarkingID::LEFT_SIDE] = i;
                }
                else if (y < laneLineSlopes[laneIds[LaneMarkingID::LEFT_SIDE]][3] && laneIds[LaneMarkingID::LEFT_MID] == -1) {
                    laneIds[LaneMarkingID::LEFT_MID] = laneIds[LaneMarkingID::LEFT_SIDE];
                    laneIds[LaneMarkingID::LEFT_SIDE] = i;
                }
            }
            else if (0.6f <= x && x < 1.0f && slope < 0.0f) {
                if (laneIds[LaneMarkingID::RIGHT_SIDE] == -1) {
                    laneIds[LaneMarkingID::RIGHT_SIDE] = i;
                }
                else if (y < laneLineSlopes[laneIds[LaneMarkingID::RIGHT_SIDE]][3] && laneIds[LaneMarkingID::RIGHT_MID] == -1) {
                    laneIds[LaneMarkingID::RIGHT_MID] = laneIds[LaneMarkingID::RIGHT_SIDE];
                    laneIds[LaneMarkingID::RIGHT_SIDE] = i;
                }
            }
        }
        else if ((0.0f <= x && x < 0.5f) && (0.0f < slope || slope < -1.0f)) {
            if (laneIds[LaneMarkingID::LEFT_MID] == -1) {
                laneIds[LaneMarkingID::LEFT_MID] = i;
            }
            else if (laneLineSlopes[laneIds[LaneMarkingID::LEFT_MID]][2] < x && laneIds[LaneMarkingID::LEFT_SIDE] == -1) {
                laneIds[LaneMarkingID::LEFT_SIDE] = laneIds[LaneMarkingID::LEFT_MID];
                laneIds[LaneMarkingID::LEFT_MID] = i;
            }
        }
        else if ((0.5f <= x && x < 1.0f) && (slope < 0.0f || slope > 1.0f)) {
            if (laneIds[LaneMarkingID::RIGHT_MID] == -1) {
                laneIds[LaneMarkingID::RIGHT_MID] = i;
            }
            else if (x < laneLineSlopes[laneIds[LaneMarkingID::RIGHT_MID]][2] && laneIds[LaneMarkingID::RIGHT_SIDE] == -1) {
                laneIds[LaneMarkingID::RIGHT_SIDE] = laneIds[LaneMarkingID::RIGHT_MID];
                laneIds[LaneMarkingID::RIGHT_MID] = i;
            }
        }
    }

    for (int i = 0; i < LaneMarkingID::NUM_LANE_MARKING; i++) {
        lanePoints[i].clear();
        if (laneIds[i] != -1) {
            lanePoints[i].points = lanesAsPoints[laneIds[i]];
            lanePoints[i].type = static_cast<short>(lanes.row(laneIds[i]).at<float>(0, 0));
        }
    }
    /*************************************************************************/

    updateLaneLineCount(lanes.rows); // Update number of lane

    return;
}


/**
 * @brief Update the new lane count. Integrated a stablizer as well, the new lane count number is only updated
 * if there are 3 consecutive updates with the same value. This is to prevent noises.
 * 
 * @param new_lane_lines_count New lane count calculated from perception
 */
void LaneDetector::updateLaneLineCount(int new_lane_lines_count)
{
    if (new_lane_lines_count != this->lane_lines_count) {
        count_change_number++;
        if (count_change_number == 3) {
            this->lane_lines_count = new_lane_lines_count;
            count_change_number = 0;
        }
    } else {
        this->count_change_number = 0;
    }
}
