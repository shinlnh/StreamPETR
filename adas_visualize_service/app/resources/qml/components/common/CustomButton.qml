import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle {
    id: customButton
    
    property string buttonText: "Button"
    property color textColor: "#FFFFFF"
    property int fontSize: 10
    property bool isBold: false
    property string fontFamily: "Liberation Mono"
    property bool isActive: false
    
    signal clicked()

    radius: 5
    Layout.preferredHeight: 32
    Layout.fillWidth: true
    
    Gradient {
        id: gradientActive
        orientation: Gradient.Horizontal
        GradientStop { position: 0.25; color: Qt.rgba(69/255, 111/255, 255/255, 1.0) }  // #456FFF
        GradientStop { position: 1.0; color: Qt.rgba(0/255, 83/255, 153/255, 1.0) }     // #005399
    }

    Gradient {
        id: gradientDefault
        orientation: Gradient.Horizontal
        GradientStop { position: 0.0; color: "#2B3136" }                               // #2B3136
    }
    
    gradient: isActive ? gradientActive : gradientDefault
    opacity: customButtonArea.containsMouse ? 0.75 : 1
    
    Text {
        id: buttonText
        anchors.centerIn: parent
        text: customButton.buttonText
        color: customButton.textColor
        font {
            pixelSize: customButton.fontSize
            bold: customButton.isBold
            family: customButton.fontFamily
        }
    }
    
    MouseArea {
        id: customButtonArea
        anchors.fill: parent
        hoverEnabled: true
        onClicked: customButton.clicked()
    }
}