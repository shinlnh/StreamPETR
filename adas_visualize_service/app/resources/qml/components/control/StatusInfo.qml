import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

import settings 1.0
import com.banvien.MainViewController 1.0

Rectangle {
    id: root
    width: parent.width
    height: 186
    color: "transparent"
    radius: 36

    // TopBar
    Item {
        id: topBar
        width: parent.width
        height: 105
        anchors.top: parent.top

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 16
            anchors.rightMargin: 16

            // Left section - LKS and AEB mode
            Row {
                Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
                spacing: 5

                Item {
                    width: 70
                    height: 42

                    Text {
                        text: {
                            if (SettingsState.lksEnabled && SettingsState.accEnabled) {
                                return "HWC"
                            }
                            if (SettingsState.lksEnabled) {
                                return "LKS"
                            }
                            if (SettingsState.accEnabled) {
                                return "ACC"
                            }
                            return ""
                        }
                        color: "#0099ff"
                        font.pixelSize: 36
                        font.family: "Liberation Mono"
                        font.bold: true
                    }
                }

                Item
                {
                    width: 35
                    height: 35

                    Image {
                        source: SettingsState.aebEnabled ? "qrc:/assets/icons/aeb_icon.png" : ""
                        scale: parent.width / sourceSize.height
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }
            
            // Center section
            Item {
                Layout.fillWidth: true
                Layout.leftMargin: -70
                height: parent.height

                Row {
                    anchors.centerIn: parent
                    spacing: 20

                    // Speed KM/H
                    Rectangle {
                        width: 100
                        height: parent.height
                        
                        color: "transparent" // Turn off to debug space 

                        Column {
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.right: parent.right
                            spacing: 0

                            Text {
                                text: SettingsStatusInfo.currentSpeed
                                color: "white"
                                font.pixelSize: 24
                                font.family: "Liberation Mono"
                                anchors.right: parent.right
                            }
                            Text {
                                text: "KMH"
                                color: "white"
                                font.pixelSize: 16
                                font.family: "Liberation Mono"
                                anchors.right: parent.right
                            }
                        }
                    }

                    // Gear (Logo)
                    Row {
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 5
                        Image {
                            source: "qrc:/assets/icons/gauge_left.png"
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Image {
                            source: "qrc:/assets/logo/LogoBV.png"
                            width: 120
                            height: sourceSize.height * (width / sourceSize.width)
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Image {
                            source: "qrc:/assets/icons/gauge_right.png"
                            anchors.verticalCenter: parent.verticalCenter
                        }
                    }

                    // Battery
                    Rectangle {
                        width: 100
                        height: parent.height

                        color: "transparent" // Turn off to debug space 
                        
                        Column {
                            anchors.verticalCenter: parent.verticalCenter
                            anchors.left: parent.left
                            spacing: 0

                            Text {
                                text: `${MainViewController.batInfo}%`
                                color: "white"
                                font.pixelSize: 24
                                font.family: "Liberation Mono"
                            }
                            Text {
                                text: "Battery"
                                color: "white"
                                font.pixelSize: 16
                                font.family: "Liberation Mono"
                            }
                        }
                    }
                }
            }

            // Right section - Settings
            Item {
                id: settingsContainer
                Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                Layout.preferredWidth: 24
                Layout.preferredHeight: 24

                Image {
                    id: settingsIcon
                    anchors.fill: parent
                    source: "qrc:/assets/icons/setting_icon.png"
                    fillMode: Image.PreserveAspectFit

                    opacity: settingsMouseArea.containsMouse ? 0.75 : 1
                    
                    // Add a mouse area to detect clicks
                    MouseArea {
                        id: settingsMouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: settingsPopup.open()
                    }
                }

                Menu {
                    id: settingsPopup
                    y: settingsContainer.height
                    // x: -width + settingsContainer.width  // Move to the right
                    x: settingsContainer.width              // Move to the left
                    closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent

                    background: Rectangle {
                        implicitWidth: 120
                        color: "#1C202C"
                        radius: 11
                    }
                    
                    MenuItem {
                        text: "UNDISTORT"

                        contentItem: Text {
                            text: parent.text
                            color: "#FFFFFF"
                            font.pixelSize: 12
                            font.family: "Liberation Mono"
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 12
                            topPadding: 8
                            bottomPadding: 8
                        }

                        background: Rectangle {
                            color: parent.highlighted ? "#FFFFFF" : "transparent"
                            opacity: 0.2
                            radius: 11
                        }

                        onTriggered: {
                            MainViewController.undistortClicked()
                        }
                    }

                    // Add more MenuItem as needed
                    // ...
                }
            }
        }
    }

    // Divider
    Rectangle {
        id: divider
        width: parent.width - 32
        height: 1
        color: "white"
        anchors.top: topBar.bottom
        anchors.horizontalCenter: parent.horizontalCenter
    }

    // BottomBar
    Rectangle {
        id: bottomBar
        width: parent.width
        height: 186 - 105 - 2  // Total height - TopBar height - Divider height
        anchors.top: divider.bottom
        color: "transparent"

        RowLayout {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 99

            // Speed control
            Item {
                Layout.preferredWidth: parent.width / 3
                Layout.fillHeight: true

                Column {
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: parent.left
                    spacing: 10

                    Text {
                        text: "Speed"
                        color: "white"
                        font.pixelSize: 24
                        font.family: "Liberation Mono"
                        font.bold: true
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    Row {
                        spacing: 60
                        
                        AdjustButton {
                            buttonType: "minus"
                            anchors.verticalCenter: parent.verticalCenter
                            onClicked: {
                                // Handle
                                if (ENABLE_CAN) {
                                    return
                                }
                                if (SettingsStatusInfo.capturedSpeed != 0) {
                                    MainViewController.confirmSpeedInput(SettingsStatusInfo.capturedSpeed - 1)
                                }
                            }
                        }
                        
                        Rectangle {
                            id: speedContainer
                            width: 40
                            height: parent.height
                            color: "transparent"
                            
                            Text {
                                id: speedText
                                text: SettingsStatusInfo.capturedSpeed
                                color: "white"
                                font.pixelSize: 16
                                font.family: "Liberation Mono"
                                anchors.centerIn: parent
                            }
                        }
                        
                        AdjustButton {
                            buttonType: "plus"
                            anchors.verticalCenter: parent.verticalCenter
                            onClicked: {
                                if (ENABLE_CAN) {
                                    return
                                }
                                // Handle
                                MainViewController.confirmSpeedInput(SettingsStatusInfo.capturedSpeed + 1)
                            }
                        }
                    }
                }
            }

            // Gear status
            Item {
                id: gearStatus
                Layout.preferredWidth: parent.width / 3
                Layout.fillHeight: true
                Layout.fillWidth: true

                Row {
                    anchors.centerIn: parent
                    spacing: 15

                    Repeater {
                        model: ["P", "R", "N", "D"]
                        Text {
                            text: modelData
                            color: (index === MainViewController.gear) ? 
                                    "#008BFF" : "#808080"
                            scale: (index === MainViewController.gear) ? 1.2 : 1.0
                            font.pixelSize: 48
                            font.family: "Liberation Mono"
                            font.bold: true

                            // Switch effects
                            Behavior on color {
                                ColorAnimation {
                                    duration: 250      // milliseconds
                                    easing.type: Easing.InOutQuad
                                }
                            }

                            Behavior on scale {
                                NumberAnimation {
                                    duration: 250;
                                    easing.type: Easing.OutBack
                                }
                            }
                        }
                    }
                }
            }

            // Distance control
            Item {
                Layout.preferredWidth: parent.width / 3
                Layout.fillHeight: true

                Column {
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.right: parent.right
                    spacing: 10

                    Text {
                        text: "Distance"
                        color: "white"
                        font.pixelSize: 24
                        font.family: "Liberation Mono"
                        font.bold: true
                        anchors.horizontalCenter: parent.horizontalCenter
                    }

                    Row {
                        spacing: 60
                        
                        AdjustButton {
                            buttonType: "minus"
                            anchors.verticalCenter: parent.verticalCenter
                            onClicked: {
                                if (ENABLE_CAN) {
                                    return
                                }
                                // Handle
                                if (SettingsStatusInfo.capturedDistance != 0) {
                                    MainViewController.confirmDistanceInput(SettingsStatusInfo.capturedDistance - 1)
                                }
                            }
                        }
                        
                        Rectangle {
                            id: distanceContainer
                            width: 40
                            height: parent.height
                            color: "transparent"
                            
                            Text {
                                id: distanceText
                                text: SettingsStatusInfo.capturedDistance
                                color: "white"
                                font.pixelSize: 16
                                font.family: "Liberation Mono"
                                anchors.centerIn: parent
                            }
                        }
                        
                        AdjustButton {
                            buttonType: "plus"
                            anchors.verticalCenter: parent.verticalCenter
                            onClicked: {
                                if (ENABLE_CAN) {
                                    return
                                }
                                // Handle
                                MainViewController.confirmDistanceInput(SettingsStatusInfo.capturedDistance + 1)
                            }
                        }
                    }
                }
            }
        }
    }
    Connections {
        target: SettingsStatusInfo
        function onCapturedDistanceChanged() {
            if (SettingsStatusInfo.capturedDistance >= 999) {
                MainViewController.confirmDistanceInput(0)
            }
        }
    }
}