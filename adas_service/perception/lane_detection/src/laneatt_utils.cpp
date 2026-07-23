#include "laneatt_utils.h"

cudaStream_t NMS_Stream;

/** @brief
 * Filter lane proposals by its class.
 * Behavior: Based on class (first col), remove proposals have class in filter.
 * Output matrix will be continuous. Return empty header if none qualify.
 * 
 * @param proposals Reference to the lane proposals
 * @param filter The vector of class to filter. Default is {0}, empty vector {}
 * means no filter.
 */
cv::Mat classFilter(cv::Mat &proposals, std::vector<float> filter)
{
    if(proposals.empty() || filter.empty()) {  // Return all if no filter
        return proposals;
    }
    cv::Mat output;
    cv::Mat mask;
    cv::Mat lanes_type = proposals.col(0);
    // Get mask for each class in filter
    for(int cls : filter) {
        if(mask.empty()){
            mask = lanes_type != cls;
        }
        else{
            mask = mask & (lanes_type != cls);
        }
    }
    
    
    // Move qualifiable proposals into a vector
    std::vector<cv::Mat> filtered_proposals;
    for(int i = 0; i < mask.rows; i++) {
        if(mask.at<uchar>(i, 0)) {
            filtered_proposals.push_back(proposals.row(i));
        }
    }

    // Put qualifiable proposals back in
    if(!filtered_proposals.empty()) {
        cv::vconcat(filtered_proposals, output);
        // Guard: make continuous and clone
        output = output.clone();
    }

    return output;
}


/** @brief
 * Apply thresholding on the lane proposals.
 * Behavior: Based on confidence, remove proposals bellow threshold,
 * Output matrix will be continuous. Return empty header if none qualify.
 */
cv::Mat confThresholding(cv::Mat &proposals, float confThresh)
{
    if(proposals.empty()){
        return proposals;
    }
    cv::Mat output;
    // Get confidence score & Apply threshold to get a mask
    cv::Mat mask = proposals.col(1) >= confThresh;
    
    // Move qualifiable proposals into a vector
    std::vector<cv::Mat> filtered_proposals;
    for(int i = 0; i < mask.rows; i++) {
        if(mask.at<uchar>(i, 0)) {
            filtered_proposals.push_back(proposals.row(i));
        }
    }

    // Put qualifiable proposals back in
    if(!filtered_proposals.empty()) {
        cv::vconcat(filtered_proposals, output);
        // Guard: make continuous and clone
        output = output.clone();
    }

    return output;
}


/**
 * @brief 
 * Post-processing of LaneATT model.
 * This apply confidence thresholding, class filter and Non Max Supression.
 * @param proposals(cv::Mat) The raw output of LaneATT model
 * @return cv::Mat which holds the filtered lanes, each rows is one lane
 */
cv::Mat LaneATTPostProcess(cv::Mat &proposals, float confThresh, float IoUThresh, int topK)
{
    // Conf Thresholding
    cv::Mat filtered = confThresholding(proposals, confThresh);

    // Class filter
    filtered = classFilter(filtered);

    // Sort
    if(filtered.empty()) {
        return cv::Mat();  // Return empty
    }
    cv::Mat sorted_idx;
    cv::sortIdx(filtered.col(1), sorted_idx, cv::SORT_EVERY_COLUMN + cv::SORT_DESCENDING);

    // NMS
    cv::Mat lanes = nms_lane_cuda(filtered, sorted_idx, IoUThresh, topK);

    return lanes;
}


/** @brief
 * Convert detected lanes from LaneATT model into vectors of points.
 * Will return empty vector if no lane or point is available.
 */
std::vector<std::vector<cv::Point2f>> LaneATTLanesToPoints(cv::Mat const &lanes)
{
    std::vector<std::vector<cv::Point2f>> lanesAsPoints;
    for(int i = 0; i < lanes.rows; i++) {
        std::vector<cv::Point2f> lane_points;
        // ---- Retrieve lane's properties ----
        static float y_step = 1.0 / N_STRIP; // Pre-calculate
        float const *lane = lanes.ptr<float>(i);

        int length = (int) (lane[4] + 0.5);
        // int x0 = lane[3]; // Doesn't need x0. Why is it in there? Dunno~ LaneATT repo is lame
        int y0 = (int) (lane[2] * N_POINT_PER_LINE + 0.5);
        
        // ---- Get points ----
        // Points based on detection
        for(int j = y0 + length - 1; y0 <= j && (0 <= j && j < N_POINT_PER_LINE); j--) {
            float x = lane[j + 5] / INPUT_W * CROP_W + CROP_X;
            if (x < CROP_X || x >= (CROP_W + CROP_X)) continue; // Skip if point is out of image
            float y = (1.0 - (y_step * j)) * CROP_H + CROP_Y;
            lane_points.push_back(cv::Point2f(x, y));
        }
        // Points extend below detection
        for(int j = y0 - 1; j >= 0; j--) {
            float x = lane[j + 5] / INPUT_W * CROP_W + CROP_X;
            if (x < CROP_X || x >= (CROP_W + CROP_X)) break; // Stop if point is out of image
            float y = (1. - (y_step * j)) * CROP_H + CROP_Y;
            lane_points.push_back(cv::Point2f(x, y));
        }

        lanesAsPoints.push_back(lane_points);
    }

    return lanesAsPoints;
}