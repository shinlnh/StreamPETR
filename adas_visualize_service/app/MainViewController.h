#ifndef MAINVIEWCONTROLLER_H
#define MAINVIEWCONTROLLER_H

// TODO
#include "TopDownViewController.h"
#include "AEBDebugWindow.h"
#include "RadarViewConfig.h"
#include "CameraViewConfig.h"

#include "common.h"
#include "adas_main_visualize.h"

#include <QObject>
#include <QQuickItem>
#include <QtQml/QQmlContext>
#include <QQmlApplicationEngine>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/thread.hpp>

using namespace cv;

class FrameBuffer {
private:
    struct Frame {
        cv::Mat originalFrame;
        cv::Mat visualizedFrame;
        bool isVisualized = false;
    };

    Frame buffers[2]; 
    std::atomic<int> frontBuffer{0};
    std::atomic<int> backBuffer{1};
    std::mutex bufferMutex;

public:
    void swapBuffers() {
        int temp = frontBuffer.load();
        frontBuffer.store(backBuffer.load());
        backBuffer.store(temp);
    }

    // For Visualization Thread
    Frame& getBackBuffer() {
        return buffers[backBuffer];
    }

    // For Render/UpdateGUI Thread
    Frame& getFrontBuffer() {
        return buffers[frontBuffer];
    }

    std::mutex& getMutex() { return bufferMutex; }
};

class MainViewController : public QObject
{
    Q_OBJECT

    /**
     * IP address property
    */
    Q_PROPERTY(QString ipAddress READ ipAddress WRITE setIpAddress NOTIFY ipAddressChanged FINAL)

    /**
     * Show status properties
    */
    Q_PROPERTY(int fpsInfo READ fpsInfo WRITE setFpsInfo NOTIFY fpsInfoChanged FINAL)
    Q_PROPERTY(int batInfo READ batInfo WRITE setBatInfo NOTIFY batInfoChanged FINAL)
    Q_PROPERTY(int kmhInfo READ kmhInfo WRITE setKmhInfo NOTIFY kmhInfoChanged FINAL)

    /**
     * Button capturing and undistort
    */
    Q_PROPERTY(bool capturing READ capturing WRITE setCapturing NOTIFY capturingChanged FINAL)
    Q_PROPERTY(bool undistort READ undistort WRITE setUndistort NOTIFY undistortChanged FINAL)

    /**
     * Button AEB
    */
    Q_PROPERTY(bool aebButton READ aebButton WRITE setAebButton NOTIFY aebButtonChanged FINAL)

    /**
     * Button system control
    */
    Q_PROPERTY(bool objButton READ objButton WRITE setObjButton NOTIFY objButtonChanged FINAL)
    Q_PROPERTY(bool laneButton READ laneButton WRITE setLaneButton NOTIFY laneButtonChanged FINAL)
    Q_PROPERTY(bool ctrlButton READ ctrlButton WRITE setCtrlButton NOTIFY ctrlButtonChanged FINAL)

    /**
     * Group button for object detection
    */
    Q_PROPERTY(bool enableObjButton READ enableObjButton WRITE setEnableObjButton NOTIFY enableObjButtonChanged FINAL)
    Q_PROPERTY(bool classIDButton READ classIDButton WRITE setClassIDButton NOTIFY classIDButtonChanged FINAL)
    Q_PROPERTY(bool distanceButton READ distanceButton WRITE setDistanceButton NOTIFY distanceButtonChanged FINAL)
    Q_PROPERTY(bool dangerButton READ dangerButton WRITE setDangerButton NOTIFY dangerButtonChanged FINAL)

    /**
     * Group button for lane tracking
    */
    Q_PROPERTY(bool enableLTButton READ enableLTButton WRITE setEnableLTButton NOTIFY enableLTButtonChanged FINAL)
    Q_PROPERTY(bool featureButton READ featureButton WRITE setFeatureButton NOTIFY featureButtonChanged FINAL)
    Q_PROPERTY(bool slidingButton READ slidingButton WRITE setSlidingButton NOTIFY slidingButtonChanged FINAL)
    Q_PROPERTY(bool localButton READ localButton WRITE setLocalButton NOTIFY localButtonChanged FINAL)

    /**
     * Group button for path tracking
    */
    Q_PROPERTY(bool lksButton READ lksButton WRITE setLksButton NOTIFY lksButtonChanged FINAL)
    Q_PROPERTY(bool accButton READ accButton WRITE setAccButton NOTIFY accButtonChanged FINAL)

    /**
     * Group button for calibration
    */
    Q_PROPERTY(bool calib_tlButton READ calib_tlButton WRITE setCalib_tlButton NOTIFY calib_tlButtonChanged FINAL)
    Q_PROPERTY(bool calib_trButton READ calib_trButton WRITE setCalib_trButton NOTIFY calib_trButtonChanged FINAL)
    Q_PROPERTY(bool calib_blButton READ calib_blButton WRITE setCalib_blButton NOTIFY calib_blButtonChanged FINAL)
    Q_PROPERTY(bool calib_brButton READ calib_brButton WRITE setCalib_brButton NOTIFY calib_brButtonChanged FINAL)

    /**
     * Current Speed and Distance
    */
    Q_PROPERTY(QString speedInput READ speedInput WRITE setSpeedInput NOTIFY speedInputChanged FINAL)       // the same with kmhInfo
    Q_PROPERTY(QString distanceInput READ distanceInput WRITE setDistanceInput NOTIFY distanceInputChanged FINAL)
    /**
     * Captured Speed and Distance
    */
    Q_PROPERTY(float speedCaptured READ speedCaptured WRITE setSpeedCaptured NOTIFY speedCapturedChanged FINAL)
    Q_PROPERTY(float distanceCaptured READ distanceCaptured WRITE setDistanceCaptured NOTIFY distanceCapturedChanged FINAL)

    /**
     * Group mode checkbox
    */
    Q_PROPERTY(int gear READ gear WRITE setGear NOTIFY gearChanged FINAL)


    /**
     * Group slider
    */
    Q_PROPERTY(float steerSlider READ steerSlider WRITE setSteerSlider NOTIFY steerSliderChanged FINAL)
    Q_PROPERTY(float thottleSlider READ thottleSlider WRITE setThottleSlider NOTIFY thottleSliderChanged FINAL)
    Q_PROPERTY(float brakeSlider READ brakeSlider WRITE setBrakeSlider NOTIFY brakeSliderChanged FINAL)

    /**
     * Group Calibration
    */
    Q_PROPERTY(QString carWidth1Input READ carWidth1Input WRITE setcarWidth1Input NOTIFY carWidth1InputChanged FINAL)
    Q_PROPERTY(QString distanceToCarpetInput READ distanceToCarpetInput WRITE setdistanceToCarpetInput NOTIFY distanceToCarpetInputChanged FINAL)
    Q_PROPERTY(QString carWidth2Input READ carWidth2Input WRITE setcarWidth2Input NOTIFY carWidth2InputChanged FINAL)
    Q_PROPERTY(QString carpetLengthInput READ carpetLengthInput WRITE setcarpetLengthInput NOTIFY carpetLengthInputChanged FINAL)

public:
    explicit MainViewController(AdasMainVisualize *adas_main_visualize, QQmlApplicationEngine* engine, QObject *parent = nullptr);
    ~MainViewController() override;
    
    bool capturing() const;
    void setCapturing(bool newCapturing);

    bool aebButton() const;
    void setAebButton(bool newAebButton);

    QString ipAddress() const;
    void setIpAddress(const QString &newIpAddress);

    int fpsInfo() const;
    void setFpsInfo(int newFpsInfo);

    int batInfo() const;
    void setBatInfo(int newBatInfo);
    
    int kmhInfo() const;
    void setKmhInfo(int newKmhInfo);

    bool undistort() const;
    void setUndistort(bool newUndistort);

    bool objButton() const;
    void setObjButton(bool newObjButton);

    bool laneButton() const;
    void setLaneButton(bool newLaneButton);

    bool ctrlButton() const;
    void setCtrlButton(bool newCtrlButton);

    bool enableObjButton() const;
    void setEnableObjButton(bool newEnableObjButton);

    bool classIDButton() const;
    void setClassIDButton(bool newClassIDButton);

    bool distanceButton() const;
    void setDistanceButton(bool newDistanceButton);

    bool dangerButton() const;
    void setDangerButton(bool newDangerButton);

    bool enableLTButton() const;
    void setEnableLTButton(bool newEnableLTButton);

    bool featureButton() const;
    void setFeatureButton(bool newFeatureButton);

    bool slidingButton() const;
    void setSlidingButton(bool newSlidingButton);

    bool localButton() const;
    void setLocalButton(bool newLocalButton);

    bool lksButton() const;
    void setLksButton(bool newLksButton);

    bool accButton() const;
    void setAccButton(bool newAccButton);

    bool calib_tlButton() const;
    void setCalib_tlButton(bool newCalib_tlButton);

    bool calib_trButton() const;
    void setCalib_trButton(bool newCalib_trButton);

    bool calib_blButton() const;
    void setCalib_blButton(bool newCalib_blButton);

    bool calib_brButton() const;
    void setCalib_brButton(bool newCalib_brButton);

    float speedCaptured() const;
    void setSpeedCaptured(float newspeedCaptured);

    float distanceCaptured() const;
    void setDistanceCaptured(float newdistanceCaptured);

    int gear() const;
    void setGear(int newGear);

    float steerSlider() const;
    void setSteerSlider(float newSteerSlider);

    float thottleSlider() const;
    void setThottleSlider(float newThottleSlider);

    float brakeSlider() const;
    void setBrakeSlider(float newBrakeSlider);
    
    QString speedInput() const;
    void setSpeedInput(QString newSpeedInput);

    QString distanceInput() const;
    void setDistanceInput(QString newDistanceInput);

    QString carWidth1Input() const;
    void setcarWidth1Input(QString newcarWidth1Input);

    QString distanceToCarpetInput() const;
    void setdistanceToCarpetInput(QString newdistanceToCarpetInput);

    QString carWidth2Input() const;
    void setcarWidth2Input(QString newcarWidth2Input);

    QString carpetLengthInput() const;
    void setcarpetLengthInput(QString newcarpetLengthInput);

    void updateVisualization();
    void updateRefValue();
    void resetRefValue();
    // Q_SLOT void keyboardButtonHandle(const QString& input);

    Q_SLOT void on_btnDone_clicked();

    Q_SLOT void startStopCapture();
    Q_SLOT void undistortClicked();
    Q_SLOT void aebButtonClicked();

    Q_SLOT void enableObjButtonClicked();
    Q_SLOT void classIDButtonClicked();
    Q_SLOT void distanceButtonClicked();
    Q_SLOT void dangerButtonClicked();

    Q_SLOT void enableLTButtonClicked();
    Q_SLOT void featureButtonClicked();
    Q_SLOT void slidingButtonClicked();
    Q_SLOT void localButtonClicked();

    Q_SLOT void lksButtonClicked();
    Q_SLOT void accButtonClicked();
    Q_SLOT void resetAutoDrivingClicked();

    Q_SLOT void calib_tlButtonClicked();
    Q_SLOT void calib_trButtonClicked();
    Q_SLOT void calib_blButtonClicked();
    Q_SLOT void calib_brButtonClicked();

    Q_SLOT void confirmSpeedInput(QString newValue);
    Q_SLOT void confirmDistanceInput(QString newValue);
    Q_SLOT void confirmCalibInput(const QString &w1, const QString &l1, const QString &w2, const QString &l2);
    Q_SLOT void calibrationCapture();
    Q_SLOT void selectPoint(int pointIdx);
    Q_SLOT void updateX(int value, int max);
    Q_SLOT void updateY(int value, int max);

    Q_SLOT void steerSliderChangeValue(float newValue);
    Q_SLOT void thottleSliderChangeValue(float newValue);
    Q_SLOT void brakeSliderChangeValue(float newValue);

    Q_SLOT void initMainViewController();    
    Q_SLOT void resetAutoDrivingButton(); 
    void setAutoDrivingButton(int stateID);
    
    void setMainviewMat(cv::Mat new_image) {
        std::lock_guard<std::mutex> lock(frameBuffer.getMutex());
        auto& backBuffer = frameBuffer.getBackBuffer();
        backBuffer.originalFrame = new_image.clone();
        backBuffer.isVisualized = false;
    }

    cv::Mat getMainviewMat() {
        std::lock_guard<std::mutex> lock(frameBuffer.getMutex());
        auto& frontBuffer = frameBuffer.getFrontBuffer();
        if (frontBuffer.isVisualized) {
            return frontBuffer.visualizedFrame.clone();
        }
        return frontBuffer.originalFrame.clone();
    }

signals:
    void capturingChanged();
    void undistortChanged();
    void ipAddressChanged();
    void aebButtonChanged();

    void fpsInfoChanged();
    void batInfoChanged();
    void kmhInfoChanged();

    void objButtonChanged();
    void laneButtonChanged();
    void ctrlButtonChanged();

    void enableObjButtonChanged();
    void classIDButtonChanged();
    void distanceButtonChanged();
    void dangerButtonChanged();

    void enableLTButtonChanged();
    void featureButtonChanged();
    void slidingButtonChanged();
    void localButtonChanged();

    void lksButtonChanged();
    void accButtonChanged();

    void calib_tlButtonChanged();
    void calib_trButtonChanged();
    void calib_blButtonChanged();
    void calib_brButtonChanged();

    void speedInputChanged();
    void distanceInputChanged();
    void carWidth1InputChanged();
    void distanceToCarpetInputChanged();
    void carWidth2InputChanged();
    void carpetLengthInputChanged();

    void gearChanged();

    void steerSliderChanged();
    void thottleSliderChanged();
    void brakeSliderChanged();

    void speedCapturedChanged();
    void distanceCapturedChanged();

private:
    // GUI states.
    bool m_capturing = false;
    bool m_aebButton = true;
    QString m_ipAddress = "192.168.240.110";
    bool m_undistort = false;   

    // Status informations
    int m_fpsInfo = 0;
    int m_batInfo = 0;
    int m_kmhInfo = 0;

    bool m_objButton = true;
    bool m_laneButton = false;
    bool m_ctrlButton = false;

    // Object detection group button
    bool m_enableObjButton = false;
    bool m_classIDButton = false;
    bool m_distanceButton = false;
    bool m_dangerButton = false;

    // Lane tracking group button
    bool m_enableLTButton = false;
    bool m_featureButton = false;
    bool m_slidingButton = false;
    bool m_localButton = false;

    // Control policy group button
    bool m_lksButton = false;
    bool m_accButton = false;

    bool m_calib_tlButton = false;
    bool m_calib_trButton = false;
    bool m_calib_blButton = false;
    bool m_calib_brButton = false;

    QString m_speedInput;
    QString m_distanceInput;
    
    // Calibration group button
    QString m_carWidth1Input;
    QString m_distanceToCarpetInput;
    QString m_carWidth2Input;
    QString m_carpetLengthInput;
    int currentPointId = 0;
    QVector<QPointF> points = QVector<QPointF>(4);
    cv::Mat image;

    float m_floatSpeedInput = 0.0f;
    float m_floatDistanceInput = 0.0f;
    float m_floatcarWidth1Input = 0.0f;
    float m_floatdistanceToCarpetInput = 0.0f;
    float m_floatcarWidth2Input = 0.0f;
    float m_floatcarpetLengthInput = 0.0f;

    int m_gear = static_cast<int>(GEAR::N);

    // Group slider
    float m_steerSlider = 0.0f;
    float m_thottleSlider = 0.0f;
    float m_brakeSlider = 0.0f;

    // Distance and speed Captured
    float m_distanceCaptured = 0.0f;
    float m_speedCaptured = 0.0f;

    // Access to the GUI's element.
    QObject* mainDisplayWindowObject;
    QObject* calibrationDisplayWindowObject;

    // Interfaces.
    AdasMainVisualize *adas_visualize_if;
    QQmlApplicationEngine* engine_if;

    // Functions.
    QImage cvtMatToQImage(cv::Mat input_image);

    // Threads.
    boost::thread *perception_visualize_thread_handler;
    std::mutex mainview_mutex;

    // void setMainviewMat(cv::Mat new_image) {
    //     std::lock_guard<std::mutex> lock(mainview_mutex);
    //     this->grapCamFrame = new_image.clone();
    // }

    // cv::Mat getMainviewMat() {
    //     std::lock_guard<std::mutex> lock(mainview_mutex);
    //     return this->grapCamFrame.clone();
    // }

    void perceptionVisualizationThread();

    int consumer_perception_results_id_ = -1;   /**< Consumer ID of Perception Results Queue */
    int consumer_planning_results_id_ = -1;   /**< Consumer ID of Planning Results Queue */
    int consumer_sff_debug_id_ = -1;           /**< Consumer ID of SFF Debug Queue */

    AEBDebugWindow* aebDebugWindow_ = nullptr;

    // 
    cv::Mat grapCamFrame = cv::Mat(cv::Size(800, 600), CV_8UC3, cv::Scalar(0, 0, 0));

    //
    void visualizeCamWindow(cv::Mat& cam_frame,
                            const PerceptionResults &perception_result,
                            const PlanningResults &planning_result);
    
    // Double Buffering
    FrameBuffer frameBuffer;

    // 
    TopDownViewController* topDownViewController;
    RadarViewConfig* m_radarViewConfig;
    CameraViewConfig* m_cameraViewConfig;
    
public slots:
    void updateGUI();

// slots:
    void onKeysChanged(const QSet<int> &keys);
};

#endif // MAINVIEWCONTROLLER_H
