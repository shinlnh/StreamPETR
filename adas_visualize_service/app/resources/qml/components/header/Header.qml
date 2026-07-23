import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle {
    id: root
    height: parent.height
    radius: 0
    
    property int iconLeftMargin: 26
    property string activeButton: "control"

    ColumnLayout {
        anchors {
            left: parent.left
            leftMargin: iconLeftMargin
            verticalCenter: parent.verticalCenter
        }
        spacing: 62
        
        HeaderIcon {
            id: controlIcon
            Layout.fillHeight: false
            iconSourceOn: "qrc:/assets/icons/control_on.png"
            iconSourceOff: "qrc:/assets/icons/control_off.png"
            isActivated: root.activeButton == "control"
            onClicked: {
                root.activeButton = "control"
                console.log("Control clicked")
            }
        }
        
        HeaderIcon {
            id: calibrationIcon
            Layout.fillHeight: false
            iconSourceOn: "qrc:/assets/icons/calib_on.png"
            iconSourceOff: "qrc:/assets/icons/calib_off.png"
            isActivated: root.activeButton == "calibration"
            onClicked: {
                root.activeButton = "calibration"
                console.log("Calibration clicked")
            }
        }
        
        HeaderIcon {
            id: parameterIcon
            Layout.fillHeight: false
            iconSourceOn: "qrc:/assets/icons/param_on.png"
            iconSourceOff: "qrc:/assets/icons/param_off.png"
            isActivated: root.activeButton == "parameter"
            onClicked: {
                root.activeButton = "parameter"
                console.log("Parameter clicked")
            }
        }
    }
}