import QtQuick 2.14
import QtQuick.Controls 2.14

Item {
    id: root
    
    property bool externalState: false
    
    implicitWidth: toggleSwitch.width
    implicitHeight: toggleSwitch.height

    signal toggleRequested()
    
    Rectangle {
        id: toggleSwitch
        width: 54
        height: 26
        radius: 15
        color: externalState ? "#456FFF" : "#3A5266"
        border.width: 1
        border.color: Qt.darker(color, 1.1)
        
        // The sliding circle
        Rectangle {
            id: toggleButton
            width: parent.height - 8
            height: width
            radius: width / 2
            color: "#FFFFFF"
            x: externalState ? parent.width - width - 4 : 4
            y: 4
            
            // Shadow effect
            Rectangle {
                anchors.fill: parent
                radius: parent.radius
                color: "transparent"
                border.width: 1
                border.color: "#FFFFFF"
            }
            
            Behavior on x {
                NumberAnimation { 
                    duration: 200
                    easing.type: Easing.InOutQuad
                }
            }
        }
        
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: externalState ? "ON" : "OFF"
            color: "#FFFFFF"
            font.pixelSize: 12
            font.bold: true
            font.family: "Liberation Mono"

            x: externalState ? 10 : toggleSwitch.width - width - 10
            
            Behavior on x {
                NumberAnimation { 
                    duration: 200
                    easing.type: Easing.InOutQuad
                }
            }
        }
        
        // Handle click events (Add timer to prevent double click)
        MouseArea {
            id: mouseArea
            anchors.fill: parent
            
            property bool canClick: true
            
            onClicked: {
                if (canClick) {
                    canClick = false
                    toggleRequested()
                    clickTimer.restart()
                }
            }
            
            Timer {
                id: clickTimer
                interval: 300
                onTriggered: mouseArea.canClick = true
            }
            
            hoverEnabled: true
            cursorShape: canClick ? Qt.PointingHandCursor : Qt.ArrowCursor
        }
    }
}