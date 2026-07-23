#include "MainViewController.h"

#include <QDebug>
#include <QDir>
#include <QTimer>
#include <QPixmap>
#include <QMetaObject>
#include <QMessageBox>
#include <QApplication>

#include <fstream>

#include <cmath>

#define RENDER_MAIN_SCREEN(img) \
    QMetaObject::invokeMethod(mainDisplayWindowObject, "updateImage", Qt::DirectConnection, Q_ARG(QImage, img));
#define RENDER_CALIBRATION_SCREEN(img) \
    QMetaObject::invokeMethod(calibrationDisplayWindowObject, "updateImage", Qt::DirectConnection, Q_ARG(QImage, img))

// CROP_W and CROP_H is referred from adas_service/perception/lane_detection/inc/laneatt_utils.h
#define CROP_W 800
#define CROP_H 450

static QString getCurrentConfigPath()
{
    return QDir::currentPath() + "/common/config/sensor/";
}

MainViewController::MainViewController(AdasMainVisualize *adas_main_visualize, QQmlApplicationEngine* engine, QObject *parent)
    : QObject{parent}
    , adas_visualize_if(adas_main_visualize)
    , engine_if(engine)
{
    // Initialize Property for QML
    topDownViewController = new TopDownViewController(this);
    m_radarViewConfig = new RadarViewConfig(this);
    m_cameraViewConfig = new CameraViewConfig(this);

    if (!m_radarViewConfig->loadFromFile(getCurrentConfigPath() + "radar_config.txt")) {
        qWarning() << "Failed to load RADAR configuration";
    }
    if (!m_cameraViewConfig->loadFromFile(getCurrentConfigPath() + "camera_config.txt")) {
        qWarning() << "Failed to load CAMERA configuration";
    }

    if (engine_if) {
        engine_if->rootContext()->setContextProperty("radarViewConfig", m_radarViewConfig);
        engine_if->rootContext()->setContextProperty("cameraViewConfig", m_cameraViewConfig);
        engine_if->rootContext()->setContextProperty("topDownViewController", topDownViewController);
    }

    perception_visualize_thread_handler = new boost::thread(&MainViewController::perceptionVisualizationThread, this);
    perception_visualize_thread_handler->detach();
}

MainViewController::~MainViewController() {
    perception_visualize_thread_handler->join();
    delete perception_visualize_thread_handler;

    delete topDownViewController;
    delete m_radarViewConfig;
    delete m_cameraViewConfig;
}

void MainViewController::perceptionVisualizationThread() {
    PerceptionResults perception_results;
    PlanningResults planning_results;
    cv::Mat newFrame;
    if (consumer_perception_results_id_ < 0) {
        consumer_perception_results_id_ = adas_visualize_if->perceptionResultsQueue.createConsumer();
    }
    if (consumer_planning_results_id_ < 0) {
        consumer_planning_results_id_ = adas_visualize_if->PlanningResultsQueue.createConsumer();
    }
    if (consumer_sff_debug_id_ < 0) {
        consumer_sff_debug_id_ = adas_visualize_if->sffDebugQueue.createConsumer();
    }

    while (true) {
        // Get all perception and planning results from the queue
        // "/nfs/share/..adas_sdk/adas_service/adas_main/adas_main.h please change the back to front, I set that to consume the latest"
        std::queue<std::shared_ptr<PerceptionResults>> all_perception_results = 
            adas_visualize_if->perceptionResultsQueue.getNext(consumer_perception_results_id_);
        std::queue<std::shared_ptr<PlanningResults>> all_planning_results = 
            adas_visualize_if->PlanningResultsQueue.getNext(consumer_planning_results_id_);
        
        // Retrieve perception, planning results and latest cam image
        perception_results = *(all_perception_results.back());
        planning_results = adas_visualize_if->getPlanningResults();
        newFrame = adas_visualize_if->getGrapCam(CAM);

        // If frame is empty then skip
        if (newFrame.empty()) {
            continue;
        }
        setMainviewMat(newFrame);

        // Visualize Top down view
        this->topDownViewController->updateObjects({perception_results.adv_objs, perception_results.src_objs});
        this->topDownViewController->updateLanes(perception_results);
        this->topDownViewController->updatePlanningPath(planning_results);

        // Visualize dash cam view
        {
            // Lock and access the frame buffer
            std::lock_guard<std::mutex> lock(frameBuffer.getMutex());
            auto& backBuffer = frameBuffer.getBackBuffer();

            // Clone the original frame for visualization
            backBuffer.visualizedFrame = backBuffer.originalFrame.clone();

            // Draw on the frame
            this->visualizeCamWindow(backBuffer.visualizedFrame, perception_results, planning_results);
            backBuffer.isVisualized = true;

            // Update on GUI
            frameBuffer.swapBuffers();
        }

        // Consume SFF debug data if available (non-blocking)
        auto sff_queue = adas_visualize_if->sffDebugQueue.getNext(consumer_sff_debug_id_, false);
        if (!sff_queue.empty()) {
            auto sff_data = sff_queue.back();
            if (sff_data) {
                // Create debug window on first data arrival (must be on GUI thread)
                if (!aebDebugWindow_) {
                    QMetaObject::invokeMethod(this, [this]() {
                        aebDebugWindow_ = new AEBDebugWindow();
                        aebDebugWindow_->show();
                    }, Qt::BlockingQueuedConnection);
                }
                if (aebDebugWindow_) {
                    aebDebugWindow_->updateData(*sff_data);
                }
            }
        }
    }
}

void MainViewController::updateGUI()
{    
    if (m_capturing) {
        // Render all windows
        RENDER_MAIN_SCREEN(cvtMatToQImage(this->getMainviewMat()));

        // Gauges.
        setFpsInfo(static_cast<int>(adas_visualize_if->getFps()));
        setKmhInfo(std::abs(adas_visualize_if->getSpeed())); // Always show positive speed
        setBatInfo(81);


        // Update reference values
        setSpeedCaptured(std::round(adas_visualize_if->getRefVel()));
        setDistanceCaptured(std::round(adas_visualize_if->getRefDist()));

        // Set distance info.
        setDistanceInput(QString::number(adas_visualize_if->getDistanceCaptured()));

        // Update gear
        setGear(static_cast<int>(adas_visualize_if->getGear()));

        // Update AEB button
        setAebButton(adas_visualize_if->getAEBState());

        // Update Mode buttons based on current state ID
        setAutoDrivingButton(adas_visualize_if->getStateID());
    }
}


void MainViewController::visualizeCamWindow(cv::Mat& cam_frame,
                                            const PerceptionResults &perception_result,
                                            const PlanningResults &planning_result)
{
    int imgWidth = cam_frame.cols - 1;  // -1 because we count from 0
    int imgHeight = cam_frame.cols - 1;

    // ---- OBJECT DETECTION VISUALIZATION ----
    // Get all detected objects
    const auto& objects = perception_result.adv_objs;
    const auto& src = perception_result.src_objs;
    size_t numObjects = std::min(objects.size(), src.size());
    for (size_t i = 0; i < numObjects; i++) {
        cv::Rect r;
        r.x = objects[i].bbox.x1;
        r.y = objects[i].bbox.y1;
        r.width = std::abs(objects[i].bbox.x2 - objects[i].bbox.x1);
        r.height = std::abs(objects[i].bbox.y2 - objects[i].bbox.y1);

        // Clamp
        r.x = clamp(r.x, 0, imgWidth);
        r.y = clamp(r.y, 0, imgHeight);
        r.width = clamp(r.width, 0, imgWidth);
        r.height = clamp(r.height, 0, imgHeight);
        
        // Set bbox and text color based on classId
        cv::Scalar bbox_color;
        cv::Scalar text_color;
        
        if (src[i].at(SensorType::FUSION)) {
            // Blue color
            bbox_color =  cv::Scalar(255, 139, 0);         // BLUE  (BGR)
            text_color =  cv::Scalar(255, 139, 0);         // BLUE  (BGR)
        }
        else if (src[i].at(SensorType::RADAR)) {
            // Red color
            bbox_color = cv::Scalar(13, 0, 255);         // RED (BGR)
            text_color = cv::Scalar(13, 0, 255);         // RED (BGR)
        }
        else if (src[i].at(SensorType::CAMERA)) {
            // Yellow color
            bbox_color = cv::Scalar(51, 153, 255);       // Orange (BGR)
            text_color = cv::Scalar(51, 153, 255);       // Orange (BGR)
        }        
        else {
            // Default color for others
            bbox_color = cv::Scalar(255, 255, 255);       // White (BGR)
            text_color = cv::Scalar(255, 255, 255);       // White (BGR)
        }

        // Draw bounding box for each object into the output image
        cv::rectangle(cam_frame, r, bbox_color, 2);

        // Add ID for each object
        std::string label = std::to_string(objects[i].id_);
        cv::putText(cam_frame, label, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, text_color, 1.5);
    }

    // ---- LANE DETECTION VISUALIZATION ----
    // Constants
    static const cv::Point baseOffset(0.01 * CROP_W, 0);
    static const float alpha = 0.5f;
    static const int lineWidth = std::max(3.0f, 0.005f * CROP_W);
    static const cv::Vec4b lineColor(0, 0, 255, alpha * 255);
    
    // Get lane result
    LaneDetection laneResult = perception_result.laneResult;
    auto &laneMarkingsPoint = laneResult.laneMarkingsPoint;

    size_t numLaneMarking = laneMarkingsPoint.size();
    if (numLaneMarking != LaneMarkingID::NUM_LANE_MARKING) {
        // If got wrong number of lane properties, log error
        ERROR("Lane visualization: Expected %d lane markings but got %lu!",
             LaneMarkingID::NUM_LANE_MARKING, numLaneMarking);
    }

    // Limit lane points if intersect
    static std::vector<size_t> numPointErase(numLaneMarking, 0);
    for (size_t i = 0; i < numLaneMarking; i++) {
        auto &lane0_points = laneMarkingsPoint[i].points;

        // Skip non exist lanes
        if (laneMarkingsPoint[i].type == LaneMarkingType::NOT_EXIST || lane0_points.empty()) {
            continue;
        }

        for (size_t j = i + 1; j < numLaneMarking; j++) {
            auto &lane1_points = laneMarkingsPoint[j].points;
            // Skip non exist lanes
            if (laneMarkingsPoint[j].type == LaneMarkingType::NOT_EXIST || lane1_points.empty()) {
                continue;
            }

            // Check which lane is on the left (compare x value at first available y)
            auto lane0_end_it = lane0_points.end() - 1;
            auto lane1_end_it = lane1_points.end() - 1;
            float startY = std::min(lane0_end_it->y, lane1_end_it->y);
            while (lane0_end_it != lane0_points.begin() && lane0_end_it->y > startY) {
                lane0_end_it--;
            }
            while (lane1_end_it != lane1_points.begin() && lane1_end_it->y > startY) {
                lane1_end_it--;
            }
            if (lane0_end_it == lane0_points.begin() || lane1_end_it == lane1_points.begin()) {
                continue;
            }
            bool leftLane_isLane0 = (lane0_end_it->x <= lane1_end_it->x);

            // Iterator from front (for checking intersect)
            auto lane0_front_it = lane0_points.begin();
            auto lane1_front_it = lane1_points.begin();

            // Find the last Y that both lane is available
            float endY = std::max(lane0_front_it->y, lane1_front_it->y);
            while (lane0_front_it != lane0_points.end() && lane0_front_it->y < endY) {
                lane0_front_it++;
            }
            while (lane1_front_it != lane1_points.end() && lane1_front_it->y < endY) {
                lane1_front_it++;
            }

            // Check if has intersection
            if ((lane0_front_it == lane0_points.end() || lane1_front_it == lane1_points.end())
                || (leftLane_isLane0 && (lane0_front_it->x <= lane1_front_it->x))
                || (!leftLane_isLane0 && (lane1_front_it->x <= lane0_front_it->x)))
            {
                continue;
            }

            // Find point before the intersect
            while ((lane0_front_it != lane0_points.end() && lane1_front_it != lane1_points.end())
                   && ((leftLane_isLane0 && (lane1_front_it->x < lane0_front_it->x))
                       || (!leftLane_isLane0 && (lane0_front_it->x < lane1_front_it->x))))
            {
                lane0_front_it++;
                lane1_front_it++;
            }
            
            // Store point index to remove later
            size_t numPointEraseLane0 = std::distance(lane0_points.begin(), lane0_front_it);
            size_t numPointEraseLane1 = std::distance(lane1_points.begin(), lane1_front_it);
            if (numPointEraseLane0 > numPointErase[i]) {
                numPointErase[i] = numPointEraseLane0;
            }
            if (numPointEraseLane1 > numPointErase[j]) {
                numPointErase[j] = numPointEraseLane1;
            }
        }
    }

    // Remove intersect part at the end
    for (size_t i = 0; i < numLaneMarking; i++) {
        if (numPointErase[i] == 0) {
            continue;
        }
        auto &lanePoints = laneMarkingsPoint[i].points;
        lanePoints.erase(lanePoints.begin(), lanePoints.begin() + numPointErase[i]);
        numPointErase[i] = 0;
    }

    // Draw lane markings onto overlay
    cv::Mat overlay_mask(cam_frame.size(), CV_8UC4, cv::Vec4b(0, 0, 0, 0));
    for (auto const &laneMarking : laneMarkingsPoint) {
        bool double_line = false;
        bool line_types[2] = {0, 0};  // 0: broken  -  1: solid
        if (LaneMarkingType::decode(laneMarking.type, double_line, line_types) < 0)
            continue;  // Continue if decode has error

        for (int i = 1; i < laneMarking.points.size(); i++) { // Draw lane marking by points
            cv::Point p1(laneMarking.points[i-1].x, laneMarking.points[i-1].y);
            cv::Point p2(laneMarking.points[i].x, laneMarking.points[i].y);

            // Draw points
            // cv::circle(this->threshImage, p1, 10, cv::Scalar(0, 0, 255), cv::FILLED);

            if (double_line) {
                // Calculate offset based on y value
                cv::Point p1Offset = baseOffset * ((p1.y - 0.4*CROP_H) / 0.6*CROP_H);
                cv::Point p2Offset = baseOffset * ((p2.y - 0.4*CROP_H) / 0.6*CROP_H);
                
                // Draw right side of line
                if(line_types[1] || i % 2) {
                    cv::line(overlay_mask, p1 + p1Offset, p2 + p2Offset, lineColor, lineWidth);
                }

                // Update points coord for left side of line
                p1 = p1 - p1Offset;
                p2 = p2 - p2Offset;
            }

            // Draw single line or left side of line
            if (line_types[0] || i % 2) {
                cv::line(overlay_mask, p1, p2, lineColor, lineWidth);
            }
        }
    }

    // Blend image and overlay
    cv::parallel_for_(cv::Range(0, cam_frame.rows), [&](const cv::Range& range) {
        for (size_t i = range.start; i < range.end; i++) {
            cv::Vec3b* bgPtr = cam_frame.ptr<cv::Vec3b>(i);
            const cv::Vec4b* fgPtr = overlay_mask.ptr<cv::Vec4b>(i);

            for (int j = 0; j < cam_frame.cols; ++j) {
                uchar alpha_value = fgPtr[j][3];
                if (alpha_value == 0) {
                    // Fully transparent, skip
                }
                else if (alpha_value == 255) {
                    // Fully opaque, copy the value
                    bgPtr[j] = cv::Vec3b(fgPtr[j][0], fgPtr[j][1], fgPtr[j][2]);
                }
                else {
                    // Blend based on alpha value
                    bgPtr[j][0] = (fgPtr[j][0] * alpha_value + bgPtr[j][0] * (255 - alpha_value)) >> 8;
                    bgPtr[j][1] = (fgPtr[j][1] * alpha_value + bgPtr[j][1] * (255 - alpha_value)) >> 8;
                    bgPtr[j][2] = (fgPtr[j][2] * alpha_value + bgPtr[j][2] * (255 - alpha_value)) >> 8;
                }
            }
        }
    });

    // ---- CAR STATUS VISUALIZATION ----
    // Keep red danger indicator ON during entire brake state to prevent flickering
    // Static flag tracks if we're in brake state
    static bool in_brake_state = false;
    static bool danger_indicator_locked = false;
    
    // Get current state from adas_visualize_if
    int current_state_id = adas_visualize_if->getStateID();
    
    // Detect brake state transition
    if (current_state_id == BRAKE_STATE_ID) {
        in_brake_state = true;
        danger_indicator_locked = true;  // Lock red indicator ON
    }
    else if (in_brake_state) {
        // Just exited brake state
        in_brake_state = false;
        danger_indicator_locked = false;  // Allow indicator to update
    }
    
    // Display logic with brake state override
    if (danger_indicator_locked || planning_result.is_danger_aeb) {
        // Show red indicator if: locked in brake state OR planning detects danger
        cv::circle(cam_frame, cv::Point(50, 30), 20, cv::Scalar(0, 0, 255), cv::FILLED);
    }
    else if (planning_result.is_warning_aeb) {
        // Show yellow warning (only if not in brake state)
        cv::circle(cam_frame, cv::Point(50, 30), 20, cv::Scalar(0, 200, 200), cv::FILLED);
    }
    switch (perception_result.lane_status) {
        case LaneStatus::MISSING_LANE:
            cv::circle(cam_frame, cv::Point(100, 30), 20, cv::Scalar(150, 150, 150), cv::FILLED);
            break;
        case LaneStatus::NOT_IN_LANE:
            cv::circle(cam_frame, cv::Point(100, 30), 20, cv::Scalar(20, 150, 20), cv::FILLED);
            break;
        case LaneStatus::IN_LANE:
            cv::circle(cam_frame, cv::Point(100, 30), 20, cv::Scalar(0, 255, 0), cv::FILLED);
            break;
    }
}

// // ------------------------------------------- VISUALIZATION ADAS -------------------------------------------

QImage MainViewController::cvtMatToQImage(cv::Mat input_image)
{
    if (input_image.empty()) {
        // Return an empty QImage if the input image is empty
        return QImage();
    }

    // Convert CV_BGR to QImage::Format_BGR888
    cv::Mat rgbImage;
    cv::cvtColor(input_image, rgbImage, cv::COLOR_BGR2RGB);

    // Create a QImage
    QImage return_image(rgbImage.data,
                        rgbImage.cols,
                        rgbImage.rows,
                        rgbImage.step,
                        QImage::Format_RGB888);

    return return_image.copy();
}

bool MainViewController::aebButton() const
{
    return m_aebButton;
}

void MainViewController::setAebButton(bool newAebButton)    
{
    if (m_aebButton == newAebButton)
        return;
    m_aebButton = newAebButton;
    emit aebButtonChanged();
}

void MainViewController::aebButtonClicked()
{
    adas_visualize_if->setValue(STATUS_BTN_AUTO_EMERGENCY_BRAKING, !m_aebButton);
}

bool MainViewController::capturing() const
{
    return m_capturing;
}

void MainViewController::setCapturing(bool newCapturing)
{
    if (m_capturing == newCapturing)
        return;
    m_capturing = newCapturing;
    emit capturingChanged();
}

void MainViewController::startStopCapture()
{
    m_capturing = !m_capturing;

    emit capturingChanged();

    if (m_capturing) {
        // Capturing
        DEBUG("Capture Button clicked %s", "state capture");
        adas_visualize_if->setValue(STATUS_BTN_CAPTURE, true);
        adas_visualize_if->TriggerSignal(true);
        INFO("IP Value: %s", ipAddress().toStdString().c_str());
        adas_visualize_if->handleConnection(ipAddress().toStdString());
    } else {
        // Stop
        DEBUG("Capture Button clicked %s", "state stop");
        adas_visualize_if->setValue(STATUS_BTN_CAPTURE, false);
        adas_visualize_if->TriggerSignal(false);
    }
}

void MainViewController::undistortClicked()
{
    qDebug() << "Undistort button pressed!";
}

void MainViewController::confirmSpeedInput(QString newValue)
{
    bool check;
    m_floatSpeedInput = round(newValue.toFloat(&check));

    if (!check) {
        ERROR("Can not convert string to float speed");
        return;
    }

    DEBUG("Convert speed ok, value: %0.1f", m_floatSpeedInput);
    if (m_floatSpeedInput < 0.0f) {
        WARN("Invalid input, set to minimum speed of 0 km/h");
        m_floatSpeedInput = 0.0f;
    }
    if (m_floatSpeedInput > 120.0f) {
        WARN("Invalid input, set to maximum speed of 120 km/h");
        m_floatSpeedInput = 120.0f;
    }
    adas_visualize_if->setRefDistance(m_distanceCaptured);  /** @todo This is a trick to avoid sending outdated value, remove ASAP */
    adas_visualize_if->setRefVel(m_floatSpeedInput);
}

void MainViewController::confirmDistanceInput(QString newValue)
{
    bool check;
    m_floatDistanceInput = round(newValue.toFloat(&check));

    if (!check) {
        ERROR("Can not convert string to float distance");
        return;
    }

    DEBUG("Convert distance ok, value: %0.1f", m_floatDistanceInput);
    if (m_floatDistanceInput >= 0.0f && m_floatDistanceInput <= 60.0f) {
        if (m_floatDistanceInput < 5.0f) {
            WARN("Invalid input, set to minimum distance of 5m");
            m_floatDistanceInput = 5.0f;
        }
    }
    else {
        WARN("Invalid input, set to maximum distance of 60m");
        m_floatDistanceInput = 60.0f;
    }
    adas_visualize_if->setRefVel(m_speedCaptured);  /** @todo This is a trick to avoid sending outdated value, remove ASAP */
    adas_visualize_if->setRefDistance(m_floatDistanceInput);
}

void MainViewController::confirmCalibInput(const QString &w1, const QString &l1, const QString &w2, const QString &l2)
{
    bool check_w1, check_l1, check_w2, check_l2;
    m_floatcarWidth1Input = w1.toFloat(&check_w1);
    m_floatdistanceToCarpetInput = l1.toFloat(&check_l1);
    m_floatcarWidth2Input = w2.toFloat(&check_w2);
    m_floatcarpetLengthInput = l2.toFloat(&check_l2);

    if (!check_w1) {
        ERROR("Can not convert w1 string to float");
        return;
    }

    if (!check_w2) {
        ERROR("Can not convert w2 string to float");
        return;
    }

    if (!check_l1) {
        ERROR("Can not convert l1 string to float");
        return;
    }

    if (!check_l2) {
        ERROR("Can not convert l2 string to float");
        return;
    }
    // DEBUG("Convert ok, value w1, l1, w2, l2: %0.2f", m_floatcarWidth1Input, m_floatdistanceToCarpetInput, m_floatcarWidth2Input, m_floatcarpetLengthInput);
}

void MainViewController::enableObjButtonClicked()
{
    m_enableObjButton = !m_enableObjButton;

    qDebug() << (m_enableObjButton ? "Enable object detection" : "Disable object detection");
}

void MainViewController::classIDButtonClicked()
{
    m_classIDButton = !m_classIDButton;

    qDebug() << (m_classIDButton ? "Class ID enable" : "Class ID disable");
}

void MainViewController::distanceButtonClicked()
{
    m_distanceButton = !m_distanceButton;

    qDebug() << (m_distanceButton ? "Distance enable" : "Distance disable");
}

void MainViewController::dangerButtonClicked()
{
    m_dangerButton = !m_dangerButton;

    qDebug() << (m_dangerButton ? "Danger zone enable" : "Danger zone disable");
}

void MainViewController::enableLTButtonClicked()
{
    m_enableLTButton = !m_enableLTButton;

    qDebug() << (m_enableLTButton ? "Enable lane tracking" : "Disable lane tracking");
}

void MainViewController::featureButtonClicked()
{
    m_featureButton = !m_featureButton;

    qDebug() << (m_featureButton ? "Feature Extraction enable" : "Feature Extraction disable");
}

void MainViewController::slidingButtonClicked()
{
    m_slidingButton = !m_slidingButton;

    qDebug() << (m_slidingButton ? "Sliding window enable" : "Sliding window disable");
}

void MainViewController::localButtonClicked()
{
    m_localButton = !m_localButton;

    qDebug() << (m_localButton ? "Localization enable" : "Localization disable");
}


void MainViewController::lksButtonClicked()
{
    DEBUG("Path tracking controller click%s", "");

    // Check if can toggle
    if (adas_visualize_if->getDisableDriveAssist()) {
        INFO("Driving assist features are disabled!");
        return;
    }

    adas_visualize_if->setValue(STATUS_BTN_LANE_KEEPING_SYSTEM, !m_lksButton);     // Update state for ADAS
}

void MainViewController::accButtonClicked()
{
    DEBUG("Adaptive Cruise Control click%s", "");

    // Check if can toggle
    if (adas_visualize_if->getDisableDriveAssist()) {
        INFO("Driving assist features are disabled!");
        return;
    }

    adas_visualize_if->setValue(STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, !m_accButton); // Update state for ADAS
}

void MainViewController::calib_tlButtonClicked()
{
    DEBUG("Enable Top Left Button Mode%s", "");

    setCalib_tlButton(!m_calib_tlButton);
    setCalib_trButton(false);
    setCalib_blButton(false);
    setCalib_brButton(false);
}

void MainViewController::calib_trButtonClicked()
{
    DEBUG("Enable Top Right Button Mode%s", "");

    setCalib_trButton(!m_calib_trButton);
    setCalib_tlButton(false);
    setCalib_blButton(false);
    setCalib_brButton(false);
}

void MainViewController::calib_blButtonClicked()
{
    DEBUG("Enable Bottom Left Button Mode%s", "");

    setCalib_blButton(!m_calib_blButton);
    setCalib_brButton(false);
    setCalib_tlButton(false);
    setCalib_trButton(false);
}

void MainViewController::calib_brButtonClicked()
{
    DEBUG("Enable Bottom Right Button Mode%s", "");

    setCalib_brButton(!m_calib_brButton);
    setCalib_blButton(false);
    setCalib_tlButton(false);
    setCalib_trButton(false);
}

void MainViewController::resetAutoDrivingClicked()
{
    DEBUG("Reset Auto Driving");
    
    // Reset UI
    resetAutoDrivingButton();
    
    // Send reset to ADAS service
    adas_visualize_if->setValue(STATUS_BTN_LANE_KEEPING_SYSTEM, false);
    adas_visualize_if->setValue(STATUS_BTN_ADAPTIVE_CRUISE_CONTROL, false);
}

void MainViewController::steerSliderChangeValue(float newValue)
{
    if (qFuzzyCompare(m_steerSlider, newValue))
        return;
    m_steerSlider = newValue;
    qDebug() << "Steer value: " << m_steerSlider;
    emit steerSliderChanged();
    adas_visualize_if->setValue(VALUE_SLIDER_STEER, m_steerSlider);
}

void MainViewController::thottleSliderChangeValue(float newValue)
{
    if (qFuzzyCompare(m_thottleSlider, newValue))
        return;
    m_thottleSlider = newValue;
    qDebug() << "Thottle value: " << m_thottleSlider;
    emit thottleSliderChanged();
    adas_visualize_if->setValue(VALUE_SLIDER_THROTTLE, m_thottleSlider);
}

void MainViewController::brakeSliderChangeValue(float newValue)
{
    if (qFuzzyCompare(m_brakeSlider, newValue))
        return;
    m_brakeSlider = newValue;
    qDebug() << "Brake value: " << m_brakeSlider;
    emit brakeSliderChanged();
    adas_visualize_if->setValue(VALUE_SLIDER_BRAKE, m_thottleSlider);
}

void MainViewController::initMainViewController()
{

    // Attempt to find the PaintItem with id "inputView"
    QObject* rootItem = engine_if->rootObjects().first();
    if (rootItem) {
        QVariant mainDisplayWindowVar = rootItem->property("mainDisplayWindow");
        if (mainDisplayWindowVar.isValid()) {
            mainDisplayWindowObject = qvariant_cast<QObject*>(mainDisplayWindowVar);
            if (mainDisplayWindowObject) {
                DEBUG("Main Display Window found");
            }
        }

        QVariant calibrationDisplayWindowVar = rootItem->property("calibrationDisplayWindow");
        if (calibrationDisplayWindowVar.isValid()) {
            calibrationDisplayWindowObject = qvariant_cast<QObject*>(calibrationDisplayWindowVar);
            if (calibrationDisplayWindowObject) {
                DEBUG("Top Down Display Window found");
            }
        }
    }

    // Need to delete this
    QTimer* timer = new QTimer(this);

    // Connect the timeout signal to a function that prints the message
    connect(timer, &QTimer::timeout, this, &MainViewController::updateGUI);

    // Set the interval to 1000 milliseconds (1 second)
    timer->start(30);
}

void MainViewController::setAutoDrivingButton(int stateID)
{
    bool lksEnabled = false;
    bool accEnabled = false;
    switch (stateID) {
        case LKS_STATE_ID:
            lksEnabled = true;
            break;
        case TJA_STATE_ID:
        case ACC_STATE_ID:
            accEnabled = true;
            break;
        case HWC_STATE_ID:
        case TJC_STATE_ID:
            lksEnabled = true;
            accEnabled = true;
            break;
    }
    setLksButton(lksEnabled);
    setAccButton(accEnabled);
}

void MainViewController::resetAutoDrivingButton()
{
    setLksButton(false);
    setAccButton(false);
    
    resetRefValue();
}

QString MainViewController::ipAddress() const
{
    return m_ipAddress;
}

void MainViewController::setIpAddress(const QString &newIpAddress)
{
    if (m_ipAddress == newIpAddress)
        return;
    m_ipAddress = newIpAddress;
    emit ipAddressChanged();
}

int MainViewController::fpsInfo() const
{
    return m_fpsInfo;
}

void MainViewController::setFpsInfo(int newFpsInfo)
{
    m_fpsInfo = newFpsInfo;
    emit fpsInfoChanged();
}

bool MainViewController::undistort() const
{
    return m_undistort;
}

void MainViewController::setUndistort(bool newUndistort)
{
    if (m_undistort == newUndistort)
        return;
    m_undistort = newUndistort;
    emit undistortChanged();
}

bool MainViewController::objButton() const
{
    return m_objButton;
}

void MainViewController::setObjButton(bool newObjButton)
{
    if (m_objButton == newObjButton)
        return;
    m_objButton = newObjButton;
    emit objButtonChanged();
}

bool MainViewController::laneButton() const
{
    return m_laneButton;
}

void MainViewController::setLaneButton(bool newLaneButton)
{
    if (m_laneButton == newLaneButton)
        return;
    m_laneButton = newLaneButton;
    emit laneButtonChanged();
}

bool MainViewController::ctrlButton() const
{
    return m_ctrlButton;
}

void MainViewController::setCtrlButton(bool newCtrlButton)
{
    if (m_ctrlButton == newCtrlButton)
        return;
    m_ctrlButton = newCtrlButton;
    emit ctrlButtonChanged();
}

bool MainViewController::enableObjButton() const
{
    return m_enableObjButton;
}

void MainViewController::setEnableObjButton(bool newEnableObjButton)
{
    if (m_enableObjButton == newEnableObjButton)
        return;
    m_enableObjButton = newEnableObjButton;
    emit enableObjButtonChanged();
}

bool MainViewController::classIDButton() const
{
    return m_classIDButton;
}

void MainViewController::setClassIDButton(bool newClassIDButton)
{
    if (m_classIDButton == newClassIDButton)
        return;
    m_classIDButton = newClassIDButton;
    emit classIDButtonChanged();
}

bool MainViewController::distanceButton() const
{
    return m_distanceButton;
}

void MainViewController::setDistanceButton(bool newDistanceButton)
{
    if (m_distanceButton == newDistanceButton)
        return;
    m_distanceButton = newDistanceButton;
    emit distanceButtonChanged();
}

bool MainViewController::dangerButton() const
{
    return m_dangerButton;
}

void MainViewController::setDangerButton(bool newDangerButton)
{
    if (m_dangerButton == newDangerButton)
        return;
    m_dangerButton = newDangerButton;
    emit dangerButtonChanged();
}

bool MainViewController::enableLTButton() const
{
    return m_enableLTButton;
}

void MainViewController::setEnableLTButton(bool newEnableLTButton)
{
    if (m_enableLTButton == newEnableLTButton)
        return;
    m_enableLTButton = newEnableLTButton;
    emit enableLTButtonChanged();
}

bool MainViewController::featureButton() const
{
    return m_featureButton;
}

void MainViewController::setFeatureButton(bool newFeatureButton)
{
    if (m_featureButton == newFeatureButton)
        return;
    m_featureButton = newFeatureButton;
    emit featureButtonChanged();
}

bool MainViewController::slidingButton() const
{
    return m_slidingButton;
}

void MainViewController::setSlidingButton(bool newSlidingButton)
{
    if (m_slidingButton == newSlidingButton)
        return;
    m_slidingButton = newSlidingButton;
    emit slidingButtonChanged();
}

bool MainViewController::localButton() const
{
    return m_localButton;
}

void MainViewController::setLocalButton(bool newLocalButton)
{
    if (m_localButton == newLocalButton)
        return;
    m_localButton = newLocalButton;
    emit localButtonChanged();
}

bool MainViewController::lksButton() const
{
    return m_lksButton;
}

void MainViewController::setLksButton(bool newLksButton)
{
    if (m_lksButton == newLksButton) 
        return;

    m_lksButton = newLksButton;
    emit lksButtonChanged();
}

bool MainViewController::accButton() const
{
    return m_accButton;
}

void MainViewController::setAccButton(bool newAccButton)
{
    if (m_accButton == newAccButton)
        return;

    m_accButton = newAccButton;
    emit accButtonChanged();
}

bool MainViewController::calib_tlButton() const
{
    return m_calib_tlButton;
}

void MainViewController::setCalib_tlButton(bool newCalib_tlButton)
{
    m_calib_tlButton = newCalib_tlButton;
    emit calib_tlButtonChanged();
}

bool MainViewController::calib_trButton() const
{
    return m_calib_trButton;
}

void MainViewController::setCalib_trButton(bool newCalib_trButton)
{
    m_calib_trButton = newCalib_trButton;
    emit calib_trButtonChanged();
}

bool MainViewController::calib_blButton() const
{
    return m_calib_blButton;
}

void MainViewController::setCalib_blButton(bool newCalib_blButton)
{
    m_calib_blButton = newCalib_blButton;
    emit calib_blButtonChanged();
}

bool MainViewController::calib_brButton() const
{
    return m_calib_brButton;
}

void MainViewController::setCalib_brButton(bool newCalib_brButton)
{
    m_calib_brButton = newCalib_brButton;
    emit calib_brButtonChanged();
}

int MainViewController::batInfo() const
{
    return m_batInfo;
}

void MainViewController::setBatInfo(int newBatInfo)
{
    if (m_batInfo == newBatInfo)
        return;
    m_batInfo = newBatInfo;
    emit batInfoChanged();
}

int MainViewController::kmhInfo() const
{
    return m_kmhInfo;
}

void MainViewController::setKmhInfo(int newKmhInfo)
{
    if (m_kmhInfo == newKmhInfo)
        return;
    m_kmhInfo = newKmhInfo;
    emit kmhInfoChanged();
}

int MainViewController::gear() const
{
    return m_gear;
}

void MainViewController::setGear(int newGear)
{
    m_gear = newGear;
    emit gearChanged();
}

float MainViewController::steerSlider() const
{
    return m_steerSlider;
}

void MainViewController::setSteerSlider(float newSteerSlider)
{
    if (qFuzzyCompare(m_steerSlider, newSteerSlider))
        return;
    m_steerSlider = newSteerSlider;
    emit steerSliderChanged();
}

float MainViewController::thottleSlider() const
{
    return m_thottleSlider;
}

void MainViewController::setThottleSlider(float newThottleSlider)
{
    if (qFuzzyCompare(m_thottleSlider, newThottleSlider))
        return;
    m_thottleSlider = newThottleSlider;
    emit thottleSliderChanged();
}

float MainViewController::brakeSlider() const
{
    return m_brakeSlider;
}

void MainViewController::setBrakeSlider(float newBrakeSlider)
{
    if (qFuzzyCompare(m_brakeSlider, newBrakeSlider))
        return;
    m_brakeSlider = newBrakeSlider;
    emit brakeSliderChanged();
}

QString MainViewController::speedInput() const
{
    return m_speedInput;
}

void MainViewController::setSpeedInput(QString newSpeedInput)
{
    m_speedInput = newSpeedInput;
    emit speedInputChanged();
}

QString MainViewController::distanceInput() const
{
    return m_distanceInput;
}

void MainViewController::setDistanceInput(QString newDistanceInput)
{
    bool check = false;
    float new_value_input = round(newDistanceInput.toFloat(&check));
    if (!check)
    {
        ERROR("Can not convert string to float distance");
        return;
    }
    if (new_value_input != 0)
        m_distanceInput = newDistanceInput;
    else 
        m_distanceInput = "60";
    emit distanceInputChanged();
}

float MainViewController::distanceCaptured() const
{
    return m_distanceCaptured;
}

void MainViewController::setDistanceCaptured(float newdistanceCaptured)
{
    if (m_distanceCaptured == newdistanceCaptured)
        return;

    m_distanceCaptured = newdistanceCaptured;
    emit distanceCapturedChanged();
}

float MainViewController::speedCaptured() const
{
    return m_speedCaptured;
}

void MainViewController::setSpeedCaptured(float newspeedCaptured)
{
    if (m_speedCaptured == newspeedCaptured)
        return;

    m_speedCaptured = newspeedCaptured;
    emit speedCapturedChanged();
}

QString MainViewController::carWidth1Input() const
{
    return m_carWidth1Input;
}

void MainViewController::setcarWidth1Input(QString newcarWidth1Input)
{
    if (m_carWidth1Input == newcarWidth1Input)
        return;
    m_carWidth1Input = newcarWidth1Input;
    emit carWidth1InputChanged();
}

QString MainViewController::distanceToCarpetInput() const
{
    return m_distanceToCarpetInput;
}

void MainViewController::setdistanceToCarpetInput(QString newdistanceToCarpetInput)
{
    if (m_distanceToCarpetInput == newdistanceToCarpetInput)
        return;
    m_distanceToCarpetInput = newdistanceToCarpetInput;
    emit distanceToCarpetInputChanged();
}

QString MainViewController::carWidth2Input() const
{
    return m_carWidth2Input;
}

void MainViewController::setcarWidth2Input(QString newcarWidth2Input)
{
    if (m_carWidth2Input == newcarWidth2Input)
        return;
    m_carWidth2Input = newcarWidth2Input;
    emit carWidth2InputChanged();
}

QString MainViewController::carpetLengthInput() const
{
    return m_carpetLengthInput;
}

void MainViewController::setcarpetLengthInput(QString newcarpetLengthInput)
{
    if (m_carpetLengthInput == newcarpetLengthInput)
        return;
    m_carpetLengthInput = newcarpetLengthInput;
    emit carpetLengthInputChanged();
}

void MainViewController::calibrationCapture()
{
    // TODO
    // cv::Mat grapCamFrame = adas_if->getSrcImage();
    // this->image = grapCamFrame;
    // RENDER_CALIBRATION_SCREEN(cvtMatToQImage(grapCamFrame));
}

void MainViewController::updateX(int value, int max)
{
    float fval = static_cast<float>(value) / max;
    points[currentPointId].setX(fval);
    updateVisualization();
}

void MainViewController::updateY(int value, int max)
{
    float fval = static_cast<float>(value) / max;
    points[currentPointId].setY(fval);
    updateVisualization();
}

void MainViewController::selectPoint(int pointIdx)
{
    currentPointId = pointIdx;
    updateVisualization();
}

void MainViewController::on_btnDone_clicked()
{
    // std::remove(CAMERA_CALIB_PARAM_DIR.toStdString().c_str());
    float carWidth = m_floatcarWidth1Input;
    float carpetWidth = m_floatdistanceToCarpetInput;
    float distanceCarToCarpet = m_floatcarWidth2Input;
    float carpetLength = m_floatcarpetLengthInput;

    std::ofstream calibFile;
    calibFile.open("camera_calib.txt");
    calibFile << "car_width " << carWidth << std::endl;
    calibFile << "carpet_width " << carpetWidth << std::endl;
    calibFile << "car_to_carpet_distance " << distanceCarToCarpet << std::endl;
    calibFile << "carpet_length " << carpetLength << std::endl;

    calibFile << "tl_x " << points[0].x() << std::endl;
    calibFile << "tl_y " << points[0].y() << std::endl;
    calibFile << "tr_x " << points[1].x() << std::endl;
    calibFile << "tr_y " << points[1].y() << std::endl;
    calibFile << "br_x " << points[2].x() << std::endl;
    calibFile << "br_y " << points[2].y() << std::endl;
    calibFile << "bl_x " << points[3].x() << std::endl;
    calibFile << "bl_y " << points[3].y() << std::endl;

    calibFile.close();
}

void MainViewController::updateVisualization()
{
    cv::Mat vizImage = image.clone();
    if (vizImage.empty()) return;

    // Get x,y
    int x, y;
    x = points[currentPointId].x() * vizImage.cols;
    y = points[currentPointId].y() * vizImage.rows;

    cv::line(vizImage, cv::Point(x, 0), cv::Point(x, vizImage.size().height), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(vizImage, cv::Point(0, y), cv::Point(vizImage.size().width, y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    // Draw 4 points
    for (int i = 0; i < points.size(); ++i) {
        x = points[i].x() * vizImage.cols;
        y = points[i].y() * vizImage.rows;
            if (i != currentPointId) {
            cv::circle(vizImage, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
            } else {
            cv::circle(vizImage, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
        }
    }

    // Fill 4 points
    cv::line(vizImage,
            cv::Point(points[0].x()*vizImage.cols, points[0].y()*vizImage.rows),
            cv::Point(points[1].x()*vizImage.cols, points[1].y()*vizImage.rows),
             cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::line(vizImage,
            cv::Point(points[1].x()*vizImage.cols, points[1].y()*vizImage.rows),
            cv::Point(points[2].x()*vizImage.cols, points[2].y()*vizImage.rows),
             cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::line(vizImage,
            cv::Point(points[2].x()*vizImage.cols, points[2].y()*vizImage.rows),
            cv::Point(points[3].x()*vizImage.cols, points[3].y()*vizImage.rows),
             cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::line(vizImage,
            cv::Point(points[3].x()*vizImage.cols, points[3].y()*vizImage.rows),
            cv::Point(points[0].x()*vizImage.cols, points[0].y()*vizImage.rows),
            cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    RENDER_CALIBRATION_SCREEN(cvtMatToQImage(vizImage));
}

void MainViewController::updateRefValue()
{
    /* Set speed reference to captured speed */ 
    confirmSpeedInput(QString::number(m_kmhInfo));
    /* Set distance reference to captured distance */
    confirmDistanceInput(m_distanceInput);
}

void MainViewController::resetRefValue()
{
    /* Reset speed reference */
    adas_visualize_if->setRefVel(0.0f);

    /* Reset distance reference*/
    adas_visualize_if->setRefDistance(0.0f);
}

void MainViewController::onKeysChanged(const QSet<int> &keys) {
    // Implement your logic to handle key combinations for controlling the car
}
