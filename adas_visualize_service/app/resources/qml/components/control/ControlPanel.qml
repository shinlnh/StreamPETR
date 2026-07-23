import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

import style 1.0

Rectangle {
    id: controlPanel
    width: 778
    height: 1031
    radius: 36
    color: Theme.backgroundColorControlHome

    // Status Info (top section)
    StatusInfo {
        id: statusInfo
        width: parent.width
        height: 186
        anchors {
            top: parent.top
            left: parent.left
        }
    }

    // Main TopDown View
    TopDownView {
        id: topDownView
        width: parent.width
        anchors {
            top: statusInfo.bottom
            left: parent.left
            bottom: parent.bottom
        }
    }

    // Mini TopDown View (bottom-right corner)
    MiniTopDownView {
        id: miniTopDownView
        width: 250
        height: 250  // Taller than container
        objects: topDownView.objects
        
        clipLeft: 40
        clipRight: 40
        clipTop: 0
        clipBottom: 50

        anchors {
            right: parent.right
            bottom: parent.bottom
            rightMargin: 12 - clipRight
            bottomMargin: 12 - clipBottom
        }
    }

    // TopDown View Legend (bottom-left corner)
    TopDownViewLegend {
        id: legend
        
        anchors {
            left: parent.left
            bottom: parent.bottom
            leftMargin: 16
            bottomMargin: 16
        }
    }
}