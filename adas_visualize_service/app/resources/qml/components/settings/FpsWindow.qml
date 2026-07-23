import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

import com.banvien.MainViewController 1.0

Rectangle {
    id: fpsWindow

    width: 240
    height: 166
    gradient: Gradient {
        GradientStop { position: 0.0; color: Qt.rgba(13/255, 17/255, 26/255, 1.0) }  // #0D111A
        GradientStop { position: 1.0; color: Qt.rgba(21/255, 25/255, 33/255, 0.55) }   // #151921 
        orientation: Gradient.Vertical
    }
    radius: 15

    // Add a property for Layout
    property alias layoutWidth: fpsWindow.width
    property alias layoutHeight: fpsWindow.height

    ColumnLayout {
        anchors.centerIn: parent
        spacing: 10

        Rectangle {
            id: blockShowFPS
            property double avgFps: 0
            width: 208
            height: 134
            radius: 7
            color: "#505050"

            Canvas {
                id: fpsCanvas
                anchors.fill: parent
                anchors.margins: 4
                property var fpsHistory: []
                property int maxPoints: 20

                onPaint: {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)

                    const margin = {
                        left: 20,
                        right: 5,
                        top: 5,
                        bottom: 10
                    }
                    const graphWidth = width - margin.left - margin.right
                    const graphHeight = height - margin.bottom - margin.top

                    // Calculate average FPS
                    const avgFPS = fpsHistory.length > 0 ? fpsHistory.reduce((a, b) => a + b) / fpsHistory.length : 0
                    blockShowFPS.avgFps = avgFPS;

                    // Grid lines
                    ctx.strokeStyle = "#D8D7D7"
                    ctx.lineWidth = 0.5
                    
                    // Calculate max FPS for dynamic scaling
                    const maxFPS = Math.max(...fpsHistory, 50)      // Minimum 50 FPS scale
                    const yAxisMax = Math.ceil(maxFPS / 50) * 50    // Round up to nearest 50

                    // Create dynamic intervals for Y-axis
                    const stepSize = yAxisMax / 2
                    const fpsIntervals = Array.from({length: 3}, (_, i) => i * stepSize)
                    
                    // Vertical grid lines (0-30 samples)
                    const timeIntervals = [0, 10, 20, 30]
                    timeIntervals.forEach(time => {
                        const x = margin.left + (time / 30) * graphWidth
                        ctx.beginPath()
                        ctx.moveTo(x, margin.top)
                        ctx.lineTo(x, height - margin.bottom)
                        ctx.stroke()
                        
                        // X labels
                        ctx.fillStyle = "#FFFFFF"
                        ctx.font = "8px 'Liberation Mono'"
                        ctx.textAlign = "center"
                        ctx.fillText(time, x, height - 2)
                    })

                    // Horizontal grid lines
                    fpsIntervals.forEach(fps => {
                        const y = (height - margin.bottom) - (fps / yAxisMax) * graphHeight
                        ctx.beginPath()
                        ctx.moveTo(margin.left, y)
                        ctx.lineTo(width - margin.right, y)
                        ctx.stroke()
                        
                        // Y labels
                        ctx.fillStyle = "#FFFFFF"
                        ctx.textAlign = "right"
                        ctx.textBaseline = "middle"
                        ctx.fillText(Math.round(fps).toString(), margin.left - 5, y)
                    })

                    // Draw average line
                    if (fpsHistory.length > 0) {
                        const avgY = margin.top + (1 - avgFPS / yAxisMax) * graphHeight
                        ctx.beginPath()
                        ctx.strokeStyle = "#33FF99"
                        ctx.moveTo(margin.left, avgY)
                        ctx.lineTo(width - margin.right, avgY)
                        ctx.stroke()
                    }

                    // Draw FPS line
                    if (fpsHistory.length > 0) {
                        ctx.beginPath()
                        ctx.strokeStyle = "#F79520"
                        ctx.lineWidth = 1.5

                        const step = graphWidth / (maxPoints - 1)
                        for (let i = 0; i < fpsHistory.length; i++) {
                            const x = margin.left + i * step
                            const y = margin.top + (1 - fpsHistory[i] / yAxisMax) * graphHeight
                            if (i === 0) ctx.moveTo(x, y)
                            else ctx.lineTo(x, y)
                        }
                        ctx.stroke()
                    }
                }

                Connections {
                    target: MainViewController
                    function onFpsInfoChanged() {
                        var fps = MainViewController.fpsInfo
                        fpsCanvas.fpsHistory.push(fps)
                        if (fpsCanvas.fpsHistory.length > fpsCanvas.maxPoints) {
                            fpsCanvas.fpsHistory.shift()
                        }
                        fpsCanvas.requestPaint()
                    }
                }
            }
        }

        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            Layout.leftMargin: 15
            spacing: 15

            // FPS
            RowLayout {
                spacing: 5
                Rectangle {
                    width: 12
                    height: 2
                    color: "#F79520"
                }
                Text {
                    text: "FPS: " + MainViewController.fpsInfo
                    color: "white"
                    font.pixelSize: 10
                }
            }

            // Average FPS
            RowLayout {
                spacing: 5
                Rectangle {
                    width: 12
                    height: 2
                    color: "#33FF99"
                }
                Text {
                    text: "Avg FPS: " + Math.round(Math.round(blockShowFPS.avgFps))
                    color: "white"
                    font.pixelSize: 10
                }
            }
        }
    }
}