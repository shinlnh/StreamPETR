import QtQuick 2.14
import style 1.0
import "../../utils/VectorDrawing.js" as VectorDrawing
import "../../utils/ObjectDrawing.js" as ObjectDrawing

Item {
    id: miniTopDownView
    
    // Regular properties
    property var objects: []
    property real maxDistance: 250
    property real minDistance: 0
    
    // Add properties for cropping
    property real clipLeft: 0
    property real clipRight: 0
    property real clipTop: 0
    property real clipBottom: 0
    property real cornerRadius: 0                   // Not using in this program
    property color backgroundColor: "#232A35"
    
    // Axis properties
    property bool showAxes: true
    property color axisColor: Qt.rgba(207/255, 204/255, 204/255, 0.4)
    property real axisLineWidth: 1.0
    property int axisLabelInterval: 25 // Distance between axis labels in meters
    property bool showAxisLabels: true
    property color axisLabelColor: Qt.rgba(207/255, 204/255, 204/255, 0.4)
    property int axisLabelFontSize: 8
    
    Rectangle {
        id: outerContainer
        width: parent.width - clipLeft - clipRight
        height: parent.height - clipTop - clipBottom
        // Position it correctly within parent bounds
        x: clipLeft
        y: clipTop
        color: "#FFFFFF"
        radius: cornerRadius
        
        clip: true
        
        // Inner container to position the content
        Item {
            id: contentContainer
            // Full size content
            width: miniTopDownView.width
            height: miniTopDownView.height
            // Position it so the right portion shows after clipping
            // If using left so it adjust x: clipRight, y: clipTop
            x: -clipLeft
            y: -clipTop
            
            // Background of content
            Rectangle {
                id: background
                anchors.fill: parent
                color: backgroundColor
            }
            
            // Coordinate Axes Canvas - behind FOV
            Canvas {
                id: axesCanvas
                anchors.fill: parent
                visible: showAxes
                z: 0
                
                // Compute the adjusted centerY based on clipping
                property real centerX: width * 0.5
                property real centerY: height
                property real adjustedCenterY: height - clipBottom
                property real meterToPixelRatio: height / (maxDistance - minDistance)
                
                onPaint: {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    
                    if (!showAxes) return;
                    
                    // Draw axes
                    drawAxes(ctx)
                }
                
                function drawAxes(ctx) {
                    ctx.lineWidth = axisLineWidth
                    ctx.strokeStyle = axisColor
                    ctx.fillStyle = axisLabelColor
                    ctx.font = axisLabelFontSize + "px 'Liberation Mono'"
                    ctx.textAlign = "center"
                    
                    // Draw X-axis (horizontal)
                    ctx.beginPath()
                    ctx.moveTo(0, adjustedCenterY)
                    ctx.lineTo(width, adjustedCenterY)
                    ctx.stroke()

                    // Draw X-axis labels and tick marks
                    if (showAxisLabels) {
                        // Negative X values (left)
                        for (var x = -axisLabelInterval; x >= -maxDistance; x -= axisLabelInterval) {
                            var pixelX = centerX + (x * meterToPixelRatio)
                            
                            // Draw tick mark
                            ctx.beginPath()
                            ctx.moveTo(pixelX, adjustedCenterY - 5)
                            ctx.lineTo(pixelX, adjustedCenterY + 5)
                            ctx.stroke()
                            
                            // // Draw label
                            // ctx.fillText(-x + "m", pixelX, adjustedCenterY - 10)
                        }
                        
                        // Positive X values (right)
                        for (var x = axisLabelInterval; x <= maxDistance; x += axisLabelInterval) {
                            var pixelX = centerX + (x * meterToPixelRatio)
                            
                            // Draw tick mark
                            ctx.beginPath()
                            ctx.moveTo(pixelX, adjustedCenterY - 5)
                            ctx.lineTo(pixelX, adjustedCenterY + 5)
                            ctx.stroke()
                            
                            // Draw label
                            ctx.fillText(x + "m", pixelX, adjustedCenterY - 10)
                        }
                        
                        // Draw origin tick marks
                        ctx.beginPath()
                        ctx.moveTo(centerX, adjustedCenterY - 5)
                        ctx.lineTo(centerX, adjustedCenterY + 5)
                        ctx.stroke()

                        // Draw origin (0m)
                        ctx.fillText("0m", centerX, adjustedCenterY - 10)
                    }
                }
            }
            
            // FOV visualization
            MiniFOV {
                id: miniFOV
                anchors.fill: parent
                radarFOVAngle: 100
                minDistance: 50
                maxDistance: 250
                z: 1
            }
            
            // Objects canvas
            Canvas {
                id: miniObjectsCanvas
                anchors.fill: parent
                z: 2
                
                property real centerX: width * 0.5
                property real centerY: height
                property real meterToPixelRatio: height / (maxDistance - minDistance)
                property real scaleVector: 2.5
                
                onPaint: {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    
                    // Draw reference circles
                    ctx.lineWidth = 1
                    ctx.setLineDash([2, 2])
                    drawReferenceCircles(ctx)
                    
                    // Draw objects
                    ctx.lineWidth = 1
                    ctx.setLineDash([99, 99])
                    
                    // Filter and draw only objects beyond minDistance
                    objects.forEach(function(obj) {
                        var distance = Math.sqrt(obj.x * obj.x + obj.y * obj.y)
                        if (distance > minDistance && distance <= maxDistance) {
                            var pixelX = centerX + (obj.x * meterToPixelRatio)
                            var pixelY = centerY - ((obj.y - minDistance) * meterToPixelRatio)
                            
                            VectorDrawing.drawVelocityVector(ctx, obj, pixelX, pixelY, scaleVector);
                            ObjectDrawing.drawObject(ctx, obj, pixelX, pixelY, ObjectStyles)
                        }
                    })
                }
                
                function drawReferenceCircles(ctx) {
                    ctx.strokeStyle = Qt.rgba(207/255, 204/255, 204/255, 0.4)
                    ctx.setLineDash([2, 2])
                    
                    var distances = [50, 150, 250]
                    distances.forEach(function(radius) {
                        var pixelRadius = (radius - minDistance) * meterToPixelRatio
                        ctx.beginPath()
                        ctx.arc(centerX, centerY, pixelRadius, 0, 2 * Math.PI)
                        ctx.stroke()
                        
                        ctx.fillStyle = Qt.rgba(207/255, 204/255, 204/255, 0.4)
                        ctx.font = "10px 'Liberation Mono'"
                        ctx.fillText(radius + "m", centerX - 12, centerY - pixelRadius + 15)
                    })
                }
            }
        }
    }
    
    // Debug: Visualize the actual component bounds
    Rectangle {
        anchors.fill: parent
        color: "transparent"
        border.width: 1
        border.color: "red"
        visible: false          // Set to true for debugging
    }
    
    onObjectsChanged: miniObjectsCanvas.requestPaint()
    
    // Make sure axes are repainted when related properties change
    onShowAxesChanged: axesCanvas.requestPaint()
    onAxisLabelIntervalChanged: axesCanvas.requestPaint()
    onMinDistanceChanged: {
        axesCanvas.requestPaint()
        miniObjectsCanvas.requestPaint()
    }
    onMaxDistanceChanged: {
        axesCanvas.requestPaint()
        miniObjectsCanvas.requestPaint()
    }
    
    // Make sure axes are repainted when crop values change
    onClipBottomChanged: axesCanvas.requestPaint()
    onClipLeftChanged: axesCanvas.requestPaint()
    onClipRightChanged: axesCanvas.requestPaint()
    onClipTopChanged: axesCanvas.requestPaint()
}