import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

import style 1.0
import Painter 1.0

Item {
    id: cameraPanel

    // Alias
    property alias mainCameraDisplay: mainViewCamera

    ColumnLayout {
        anchors.fill: parent
        // anchors.margins: 8

        // Main camera view
        PaintItem {
            id: mainViewCamera
            Layout.fillWidth: true
            Layout.preferredHeight: 800
        }

        // Empty space
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "transparent"
        }
    }
}