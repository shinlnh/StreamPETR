import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

import style 1.0
import "../common"
import "../../utils/Dialogs.js" as Dialogs

import com.banvien.MainViewController 1.0

Item {
    id: settingsPanel

    property bool isCapturing: MainViewController.capturing

    ColumnLayout {
        anchors.fill: parent
        Layout.alignment: Qt.AlignTop
        spacing: 16

        // Connection Card
        CardContainer {
            Layout.fillWidth: true
            Layout.preferredHeight: 88
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16

                Text {
                    text: "Connection"
                    color: "#FFFFFF"
                    
                    font {
                        pixelSize: 16
                        bold: true
                        family: "Liberation Mono"
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: "transparent"
                    radius: 7

                    Rectangle {
                        width: 22
                        height: 32
                        anchors.left: parent.left
                        radius: 7
                        gradient: Gradient {
                            GradientStop { position: 0.0; color: Qt.rgba(69/255, 111/255, 255/255, 1.0) }   // #456FFF
                            GradientStop { position: 0.5; color: Qt.rgba(0/255, 83/255, 153/255, 1.0) }     // #005399 
                            orientation: Gradient.Horizontal
                        }

                        // Mask Right
                        Rectangle {
                            id: maskRight
                            width: parent.width / 2
                            height: parent.height
                            anchors.right: parent.right
                            color: "#2B3136"
                        }
                    }
                    
                    CustomTextInput {
                        id: ipInput
                        anchors.fill: parent
                        anchors.leftMargin: maskRight.width + 1
                        textFieldHeight: 32
                        textInputSize: 12
                        enabled: false
                        text: MainViewController.ipAddress
                    }
                }
            }
        }

        // Start/Stop Capture Card
        CardContainer {
            Layout.fillWidth: true
            Layout.preferredHeight: 62

            RowLayout {
                anchors.fill: parent
                anchors {
                    leftMargin: 16
                    rightMargin: 16
                    topMargin: 11
                    bottomMargin: 11
                }

                Image {
                    id: captureImage
                    Layout.preferredHeight: 40
                    Layout.preferredWidth: 40

                    source: isCapturing ? "qrc:/assets/icons/capture_on.png" : "qrc:/assets/icons/capture_off.png"

                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            MainViewController.startStopCapture()
                        }
                    }
                }

                Text {
                    text: isCapturing ? "Stop Capture" : "Start Capture"
                    color: "#FFFFFF"

                    font {
                        pixelSize: 16
                        bold: false
                        family: "Liberation Mono"
                    }
                }
            }
        }

        // Mode Card
        CardContainer {
            id: modeCard
            Layout.fillWidth: true
            Layout.preferredHeight: 193

            ColumnLayout {
                id: modeLevelLayout
                anchors.fill: parent
                anchors.margins: 16
                
                // Mode card label
                Rectangle {
                    id: modeCardLabel
                    Layout.preferredHeight: 32
                    Layout.fillWidth: true
                    radius: 5

                    Text {
                        anchors.centerIn: parent
                        text: "ADAS Features"
                        color: "#FFFFFF"
                        font {
                            pixelSize: 16
                            bold: true
                            family: "Liberation Mono"
                        }
                    }

                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.25; color: Qt.rgba(69/255, 111/255, 255/255, 1.0) }  // #456FFF
                        GradientStop { position: 1.0; color: Qt.rgba(0/255, 83/255, 153/255, 1.0) }     // #005399
                    }
                }

                // Divider
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 1
                    color: "white"
                }

                // Mode buttons
                ColumnLayout {
                    id: modeButtonLayout
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 8

                    // Lane Keep System setting - LKS
                    Rectangle {
                        width: 208
                        height: 32
                        color: "#2B3136"
                        radius: 5

                        RowLayout {
                            anchors.fill: parent
                            Layout.fillWidth: true
                            Layout.fillHeight: true

                            Text {
                                text: "Lane Keep System"
                                Layout.fillWidth: true
                                Layout.alignment: Qt.AlignVCenter
                                Layout.leftMargin: 8
                                color: "#FFFFFF"
                                font {
                                    pixelSize: 10
                                    family: "Liberation Mono"
                                }
                            }

                            ToggleSwitch {
                                Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                Layout.rightMargin: 3
                                implicitWidth: 54
                                implicitHeight: 26
                                externalState: MainViewController.lksButton
                                onToggleRequested: {
                                    if (ENABLE_CAN) {
                                        return
                                    }
                                    if (SettingsState.gear !== 3) {
                                        // Doesn't allow if gear isn't Drive (3)
                                        Dialogs.showWarningDialog(
                                            ApplicationWindow.window.gWarningDialog,
                                            "Shift to Drive first!"
                                        )
                                        return
                                    }
                                    
                                    MainViewController.lksButtonClicked()
                                }
                            }
                        }
                    }

                    // Adaptive Cruise Control setting - ACC
                    Rectangle {
                        width: 208
                        height: 32
                        color: "#2B3136"
                        radius: 5

                        RowLayout {
                            anchors.fill: parent
                            Layout.fillWidth: true
                            Layout.fillHeight: true

                            Text {
                                text: "Adaptive Cruise Control"
                                Layout.fillWidth: true
                                Layout.alignment: Qt.AlignVCenter
                                Layout.leftMargin: 8
                                color: "#FFFFFF"
                                font {
                                    pixelSize: 10
                                    family: "Liberation Mono"
                                }
                            }

                            ToggleSwitch {
                                Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                Layout.rightMargin: 3
                                implicitWidth: 54
                                implicitHeight: 26
                                externalState: MainViewController.accButton
                                onToggleRequested: {
                                    if (SettingsState.gear !== 3) {
                                        if (ENABLE_CAN) {
                                            return
                                        }
                                        // Doesn't allow if gear isn't Drive (3)
                                        Dialogs.showWarningDialog(
                                            ApplicationWindow.window.gWarningDialog, 
                                            "Shift to Drive first!"
                                        )
                                        return
                                    }
                                    
                                    MainViewController.accButtonClicked()
                                }
                            }
                        }
                    }

                    // Auto Emergency Braking setting
                    Rectangle {
                        width: 208
                        height: 32
                        color: "#2B3136"
                        radius: 5

                        RowLayout {
                            anchors.fill: parent
                            Layout.fillWidth: true
                            Layout.fillHeight: true

                            Text {
                                text: "Auto Emergency Braking"
                                Layout.fillWidth: true
                                Layout.alignment: Qt.AlignVCenter
                                Layout.leftMargin: 8
                                color: "#FFFFFF"
                                font {
                                    pixelSize: 10
                                    family: "Liberation Mono"
                                }
                            }

                            ToggleSwitch {
                                Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
                                Layout.rightMargin: 3
                                implicitWidth: 54
                                implicitHeight: 26
                                externalState: MainViewController.aebButton
                                onToggleRequested: {
                                    if (ENABLE_CAN) {
                                        return
                                    }
                                    MainViewController.aebButtonClicked()
                                }
                            }
                        }
                    }
                }
            }
        }

        // Display Window Card by Sliding (Not draggable)
        CardContainer {
            Layout.fillWidth: true
            Layout.preferredHeight: 96

            ColumnLayout {
                anchors.fill: parent
                Layout.fillWidth: true
                anchors.margins: 16
                spacing: 8
                
                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 16

                    Text {
                        text: "Steer"
                        color: "#FFFFFF"
                        font {
                            pixelSize: 16
                            bold: true
                            family: "Liberation Mono"
                        }
                    }

                    // Empty Space
                    Item {
                        Layout.fillWidth: true
                    }

                    Item {
                        Layout.preferredWidth: 120
                        Layout.preferredHeight: 5
                        
                        // Background
                        Rectangle {
                            anchors.fill: parent
                            radius: height / 2
                            color: "#32A6F9"
                            opacity: 0.2
                        }
                        
                        // Progress Bar for Steer
                        Rectangle {
                            id: steerProgressBar
                            height: parent.height
                            radius: height / 2
                            color: "#32A6F9"

                            property real steerValue: MainViewController.steerSlider
                            property real maxSteerValue: 1.0

                            width: Math.min(Math.abs(steerValue) / maxSteerValue, 1.0) * (parent.width / 2)

                            // Anchor to the center of the parent(Progress Bar)
                            anchors.left: steerValue >= 0 ? parent.horizontalCenter : undefined
                            anchors.right: steerValue < 0 ? parent.horizontalCenter : undefined
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 16

                    Text {
                        text: "Throttle"
                        color: "#FFFFFF"
                        font {
                            pixelSize: 16
                            bold: true
                            family: "Liberation Mono"
                        }
                    }

                    // Empty Space
                    Item {
                        Layout.fillWidth: true
                    }

                    Item {
                        Layout.preferredWidth: 120
                        Layout.preferredHeight: 5

                        Rectangle {
                            anchors.fill: parent
                            radius: height / 2
                            color: "#32A6F9"
                            opacity: 0.2
                        }

                        Rectangle {
                            anchors.left: parent.left
                            height: parent.height
                            radius: height / 2
                            color: "#32A6F9"
                            opacity: 1.0

                            property real throttleValue: MainViewController.thottleSlider

                            width: Math.abs(throttleValue) * parent.width
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 16

                    Text {
                        text: "Brake"
                        color: "#FFFFFF"
                        font {
                            pixelSize: 16
                            bold: true
                            family: "Liberation Mono"
                        }
                    }

                    // Empty Space
                    Item {
                        Layout.fillWidth: true
                    }

                    Item {
                        Layout.preferredWidth: 120
                        Layout.preferredHeight: 5

                        Rectangle {
                            anchors.fill: parent
                            radius: height / 2
                            color: "#32A6F9"
                            opacity: 0.2
                        }

                        Rectangle {
                            anchors.left: parent.left
                            height: parent.height
                            radius: height / 2
                            color: "#32A6F9"
                            opacity: 1.0

                            property real brakeValue: MainViewController.brakeSlider

                            width: Math.abs(brakeValue) * parent.width
                        }
                    }
                }
            }
        }
        
        FpsWindow {
            Layout.fillWidth: true
            Layout.preferredHeight: 166
        }

        // Empty Space
        Item {
            Layout.fillHeight: true
        }
    }

    Connections {
        target: MainViewController
        
        function onLksButtonChanged() {
            SettingsState.lksEnabled = MainViewController.lksButton
        }
        
        function onAccButtonChanged() {
            SettingsState.accEnabled = MainViewController.accButton
        }

        function onAebButtonChanged() {
            SettingsState.aebEnabled = MainViewController.aebButton
        }

        function onGearChanged() {
            SettingsState.gear = MainViewController.gear
        }
    }
}