#include <QApplication>
#include <QQmlApplicationEngine>
#include <QIcon>

#include <QObject>
#include <QKeyEvent>
#include <QSet>
#include <QTimer>
#include <iostream>

#include "MainViewController.h"
#include "adas_main_visualize.h"
#include "key_handler.h"
#include "PaintItem.h"
#include "version.h"

int main(int argc, char *argv[])
{
    // Display version information
    printf("==============================================\n");
    printf("BV ADAS Visualize Service %s\n", GIT_VERSION);
    printf("Commit: %s\n", GIT_COMMIT_HASH);
    printf("Branch: %s\n", GIT_BRANCH);
    printf("Build: %s %s\n", BUILD_DATE, BUILD_TIME);
    printf("==============================================\n");

    // ==== [0] INPUT PARSE ==================================================
    const std::string keys =
        "{help h usage ? |                 | print this message   }"
        "{service_ip     | 192.168.240.110 | Adas Service's IP Address}"
        ;

    CommandLineParser parser(argc, argv, keys);
    parser.about("BV ADAS APP v1.0.0");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    std::string service_ip = parser.get<std::string>("service_ip");

    // ==== [1] REGISTER SIGNALS =============================================
    std::signal(SIGINT, [](int sig_num) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SIGINT caught, shutting down...");
        rclcpp::shutdown();
        exit(sig_num);
    });

    // ==== [2] BACKEND SETUP ================================================
    // Start RoS2
    rclcpp::init(argc, argv);

    // Create Visualization node
    AdasMainVisualize master;

    // ==== [3] FRONTEND SETUP ===============================================
    // Create App and Engine
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // App setup
    app.setWindowIcon(QIcon(":/images/logoBV.png"));

    // Engine setup
    engine.addImportPath("qrc:/resources/qml");
    engine.addImportPath("qrc:/resources/qml/components");

    // Register types, definitions, constants for QML
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qmlRegisterType<PaintItem>("Painter", 1, 0, "PaintItem");
#ifdef ENABLE_CAN
    engine.rootContext()->setContextProperty("ENABLE_CAN", true);
#else
    engine.rootContext()->setContextProperty("ENABLE_CAN", false);
#endif

    // Register MainViewController Object
    MainViewController *mainViewController = new MainViewController(&master, &engine);
    mainViewController->setIpAddress(service_ip.c_str());
    qmlRegisterSingletonInstance("com.banvien.MainViewController",
                                1,
                                0,
                                "MainViewController",
                                mainViewController);

    // AutoDrivingStateHandle autostate_reset_handler(mode);
    // QObject::connect(
    //     &autostate_reset_handler, &AutoDrivingStateHandle::resetAutoDrivingButton, //signal
    //     mainViewController, &MainViewController::resetAutoDrivingButton //slot
    // );
    // //pass middle-man to master
    // master.setAutoDriveRestarter(&autostate_reset_handler);

    // // Register CalibController Object
    // CalibController *calibController = new CalibController();
    // qmlRegisterSingletonInstance("com.banvien.CalibController",
    //                             1,
    //                             0,
    //                             "CalibController",
    //                             calibController);

    // // Register ParamController Object
    // ParamController *paramController = new ParamController();
    // qmlRegisterSingletonInstance("com.banvien.ParamController",
    //                             1,
    //                             0,
    //                             "ParamController",
    //                             paramController);

#ifndef ENABLE_CAN
    // Create and register Keyhandler
    KeyPressHandler keyPressHandler(nullptr, mainViewController, &master);
    QObject::connect(&keyPressHandler, &KeyPressHandler::keysChanged,
                     mainViewController, &MainViewController::onKeysChanged);
    app.installEventFilter(&keyPressHandler);
#endif

    // Register Engine callbacks
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
                         if (!obj && url == objUrl)
                             QCoreApplication::exit(-1);
                     },
                     Qt::QueuedConnection);

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     mainViewController, &MainViewController::initMainViewController,
                     Qt::QueuedConnection);

    QObject::connect(&engine, &QQmlApplicationEngine::destroyed,
                     [mainViewController]() { delete mainViewController; });

    // Register App callbacks
    QObject::connect(&app, &QGuiApplication::aboutToQuit,
                     []() { rclcpp::shutdown(); });

    // Start Engine
    engine.load(url);

    // Start App
    return app.exec();
}