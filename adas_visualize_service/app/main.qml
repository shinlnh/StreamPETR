import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.14
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4

import com.banvien.MainViewController 1.0
import Painter 1.0

import style 1.0

import "./resources/qml/components/common"
import "./resources/qml/components/header"
import "./resources/qml/components/control"
import "./resources/qml/components/camera"
import "./resources/qml/components/settings"
import "./resources/qml/components/dialogs"

ApplicationWindow {
    id: root
    title: "ADAS Service"
    visible: true
    width: 1920
    height: 1063
    color: Theme.backgroundColor

    property PaintItem mainDisplayWindow: cameraView.mainCameraDisplay
    property PaintItem calibrationDisplayWindow: calibrationWindow

    property var gWarningDialog: warningDialog

    // Header
    Header {
        id: header
        z: 1
    }

    // Main window
    Item {
        id: controlWindow
        anchors.fill: parent

        // Main layout container
        RowLayout {
            anchors {
                fill: parent
                margins: 16
            }
            spacing: 8

            // Left Panel - Visualization & Controls
            ControlPanel {
                Layout.preferredWidth: 774
                Layout.fillHeight: true
            }

            // Center Panel - Camera View
            CameraPanel {
                id: cameraView
                Layout.preferredWidth: 800
                Layout.fillHeight: true
            }

            // Right Panel - Settings
            SettingsPanel {
                Layout.preferredWidth: 240
                Layout.fillHeight: true
            }
        }
    }

    // Dialogs
    WarningDialog {
        id: warningDialog
    }
}