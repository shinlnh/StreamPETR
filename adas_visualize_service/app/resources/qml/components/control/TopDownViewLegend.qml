import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

import settings 1.0
import style 1.0
import com.banvien.MainViewController 1.0

import "../../utils/ObjectDrawing.js" as ObjectDrawing

Item {
    id: root
    width: 32
    height: 32
    Layout.alignment: Qt.AlignRight | Qt.AlignVCenter

    Image {
        id: iconImage
        width: parent.width
        height: parent.height
        source: "qrc:/assets/icons/info.png"
        fillMode: Image.PreserveAspectFit
        mipmap: true

        opacity: iconMouseArea.containsMouse ? 1 : 0.5
        
        // Add a mouse area to detect clicks
        MouseArea {
            id: iconMouseArea
            anchors.fill: parent
            hoverEnabled: true
            onEntered: {
                closeTimer.stop()
                legendItems.open()
            }
            onExited: closeTimer.start()
        }
    }

    Menu {
        id: legendItems
        y: - legendItems.contentHeight
        x: iconImage.width              // Move to the left
        closePolicy: Popup.CloseOnPressOutside | Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent

        Timer {
            id: closeTimer
            interval: 200
            onTriggered: {
                if (!iconMouseArea.containsMouse) {
                    legendItems.close()
                }
            }
        }

        background: Rectangle {
            implicitWidth: 184
            color: "#1C202C"
            radius: 16

            MouseArea {
                anchors.fill: parent
                hoverEnabled: true
                onEntered: closeTimer.stop()
                onExited: closeTimer.start()
            }
        }
        
        // Radar
        MenuItem {
            text: "Radar"
            topPadding: 12

            contentItem: RowLayout {
                anchors.fill: parent 
                anchors.left: parent.left
                spacing: 12

                Rectangle {
                    width: 20
                    height: 20
                    color: "transparent"
                    border.color: ObjectStyles.sensorsType["RADAR"].color
                    border.width: 0.7
                    radius: 1

                    Layout.alignment: Qt.AlignVCenter
                    Layout.leftMargin: 12
                    
                    Text {
                        anchors.centerIn: parent
                        text: "ID"
                        color: ObjectStyles.sensorsType["RADAR"].color
                        font.pixelSize: 12
                        font.family: "Liberation Mono"
                    }
                }

                Text {
                    text: parent.parent.text
                    color: "#FFFFFF"
                    font.pixelSize: 14
                    font.family: "Liberation Mono"
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }
            }

            background: Rectangle {
                color: "transparent"
            }
        }

        // Camera
        MenuItem {
            text: "Camera"
            topPadding: 4

            contentItem: RowLayout {
                anchors.fill: parent 
                anchors.left: parent.left
                spacing: 12

                Rectangle {
                    width: 20
                    height: 20
                    color: "transparent"
                    border.color: ObjectStyles.sensorsType["CAMERA"].color
                    border.width: 0.7
                    radius: 1

                    Layout.alignment: Qt.AlignVCenter
                    Layout.leftMargin: 12
                    
                    Text {
                        anchors.centerIn: parent
                        text: "ID"
                        color: ObjectStyles.sensorsType["CAMERA"].color
                        font.pixelSize: 12
                        font.family: "Liberation Mono"
                    }
                }

                Text {
                    text: parent.parent.text
                    color: "#FFFFFF"
                    font.pixelSize: 14
                    font.family: "Liberation Mono"
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }
            }

            background: Rectangle {
                color: "transparent"
            }
        }

        // Fusion
        MenuItem {
            text: "Fusion"
            topPadding: 4

            contentItem: RowLayout {
                anchors.fill: parent 
                anchors.left: parent.left
                spacing: 12  

                Rectangle {
                    width: 20
                    height: 20
                    color: "transparent"
                    border.color: ObjectStyles.sensorsType["FUSION"].color
                    border.width: 0.7
                    radius: 1

                    Layout.alignment: Qt.AlignVCenter
                    Layout.leftMargin: 12
                    
                    Text {
                        anchors.centerIn: parent
                        text: "ID"
                        color: ObjectStyles.sensorsType["FUSION"].color
                        font.pixelSize: 12
                        font.family: "Liberation Mono"
                    }
                }

                Text {
                    text: parent.parent.text
                    color: "#FFFFFF"
                    font.pixelSize: 14
                    font.family: "Liberation Mono"
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }
            }

            background: Rectangle {
                color: "transparent"
            }
        }

        // Car
        MenuItem {
            text: "Car"
            topPadding: 4

            contentItem: RowLayout {
                anchors.fill: parent
                anchors.left: parent.left
                spacing: 12

                Item {
                    width: 20
                    height: 20

                    Layout.alignment: Qt.AlignVCenter
                    Layout.leftMargin: 12
                    Canvas {
                        width: parent.width
                        height: parent.height
                        onPaint: {
                            // Common attribute
                            var classId = "2"
                            var size = width

                            // Get attribute to config canvas
                            var ctx = getContext("2d")
                            var shape = ObjectStyles.getShapeConfig(classId)
                            var drawFunc = ObjectDrawing.getShapeDrawFunction(shape.type)
                            var color = ObjectStyles.getColor(classId);

                            // Config the canvas
                            ctx.translate(width / 2, height / 2)
                            ctx.fillStyle = color;

                            // Draw symbol
                            drawFunc(ctx, shape.xOffset, shape.yOffset, size, shape.points)
                            
                            // Fill the symbol
                            ctx.fill();
                        }

                        Component.onCompleted: requestPaint()
                    }
                }

                Text {
                    text: parent.parent.text
                    color: "#FFFFFF"
                    font.pixelSize: 14
                    font.family: "Liberation Mono"
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }
            }

            background: Rectangle {
                color: "transparent"
            }
        }

        // Obstacle
        MenuItem {
            text: "Obstacle"
            topPadding: 4

            contentItem: RowLayout {
                anchors.fill: parent 
                anchors.left: parent.left
                spacing: 12 

                Item {
                    width: 20
                    height: 20

                    Layout.alignment: Qt.AlignVCenter
                    Layout.leftMargin: 12
                    Canvas {
                        width: parent.width
                        height: parent.height
                        onPaint: {
                            // Common attribute
                            var classId = "-1"
                            var size = width

                            // Get attribute to config canvas
                            var ctx = getContext("2d")
                            var shape = ObjectStyles.getShapeConfig(classId)
                            var drawFunc = ObjectDrawing.getShapeDrawFunction(shape.type)
                            var color = ObjectStyles.getColor(classId);

                            // Config the canvas
                            ctx.translate(width / 2, height / 2)
                            ctx.fillStyle = color;

                            // Draw symbol
                            drawFunc(ctx, shape.xOffset, shape.yOffset, size, shape.points)
                            
                            // Fill the symbol
                            ctx.fill();
                        }

                        Component.onCompleted: requestPaint()
                    }
                }

                Text {
                    text: parent.parent.text
                    color: "#FFFFFF"
                    font.pixelSize: 14
                    font.family: "Liberation Mono"
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                }
            }

            background: Rectangle {
                color: "transparent"
            }
        }

        // Velocity Vector
        MenuItem {
            text: "Velocity Vector"
            topPadding: 4
            bottomPadding: 12

            contentItem: ColumnLayout {
                anchors.fill: parent 
                anchors.left: parent.left 

                RowLayout {
                    Layout.leftMargin: 12

                    Item {
                        // ScaleVector refer from Objects.qml
                        // Change the ratio of vector velocity from m/s to km/h 
                        width: 120 * (2.5 / 3.6) + 25 // KMH * (ScaleVector / 3.6) + Padding
                        height: 20

                        Canvas {
                            anchors.fill: parent
                            onPaint: {
                                var ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                
                                // Draw velocity scale bar
                                var barWidth = 120 * (2.5 / 3.6)
                                var barHeight = 2
                                var startX = 10
                                var startY = 0
                                
                                // Draw the bar
                                ctx.fillStyle = "#FFFF00"
                                ctx.fillRect(startX, startY, barWidth, barHeight)
                                
                                // Set style for border of ticks
                                ctx.strokeStyle = "#FFFF00"
                                ctx.lineWidth = 1
                                
                                // Draw scale labels and ticks
                                ctx.fillStyle = "#FFFF00"
                                ctx.font = "14px 'Liberation Mono'"
                                ctx.textAlign = "center"
                                
                                // Label: 0 Kmh
                                ctx.fillText("0", startX, startY + barHeight + 16)
                                ctx.beginPath()
                                ctx.moveTo(startX, startY + barHeight)
                                ctx.lineTo(startX, startY + barHeight + 4)
                                ctx.stroke()
                                
                                // Label: 60 Kmh
                                var midX = startX + barWidth / 2
                                ctx.fillText("60", midX, startY + barHeight + 16)
                                ctx.beginPath()
                                ctx.moveTo(midX, startY + barHeight)
                                ctx.lineTo(midX, startY + barHeight + 4)
                                ctx.stroke()
                                
                                // Label: 120 Kmh
                                ctx.fillText("120", startX + barWidth - 10, startY + barHeight + 16)
                                ctx.beginPath()
                                ctx.moveTo(startX + barWidth, startY + barHeight)
                                ctx.lineTo(startX + barWidth, startY + barHeight + 4)
                                ctx.stroke()
                            }

                            Component.onCompleted: requestPaint()
                        }
                    }

                    Text {
                        text: "Kmh"
                        color: "#FFFF00"
                        font.pixelSize: 14
                        font.family: "Liberation Mono"
                        Layout.alignment: Qt.AlignBottom
                    }
                }

                Text {
                    text: parent.parent.text
                    color: "#FFFFFF"
                    font.pixelSize: 14
                    font.family: "Liberation Mono"
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    Layout.topMargin: -8
                }
            }

            background: Rectangle {
                color: "transparent"
            }
        }

        // // Add more MenuItem as needed
        // // ...
    }
}